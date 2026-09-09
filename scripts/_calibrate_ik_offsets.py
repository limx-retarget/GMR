"""Measure and calibrate the rotation offsets of an IK config.

An IK config's ``rot_offset`` maps a source skeleton's bone frame onto a robot link frame, and the
convention differs per capture format, so a config cannot be copied across formats unchanged.
Rather than guessing, this scores a config by how well the retargeted robot reproduces the human
*limb directions*, which is independent of any frame convention, and can search the axis-aligned
offsets for the value that scores best.

.. code-block:: bash

    # score an existing config
    python scripts/_calibrate_ik_offsets.py score --robot unitree_g1 --src bvh_lafan1 \
        --motion_file test_samples/lafan/walk.bvh

    # calibrate a config in place, one body at a time
    python scripts/_calibrate_ik_offsets.py calibrate --robot limx_luna --src bvh_nokov \
        --motion_file test_samples/nokov/walk.bvh --config general_motion_retargeting/ik_configs/bvh_nokov_to_luna.json
"""

import argparse
import itertools
import json
import shutil

import numpy as np
import mujoco as mj
from scipy.spatial.transform import Rotation as R

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting.motion_retarget import human_height_scale_ratio
from general_motion_retargeting.params import IK_CONFIG_DICT, resolve_robot_xml

FIXED_ORIENTATION_SOURCES = {"smplx"}

# Limb segments per source skeleton, as (proximal body, distal body). Only the human body names
# differ between formats; the robot links come from the config being scored.
SEGMENTS = {
    "smplx": [
        ("pelvis", "spine3"),
        ("left_hip", "left_knee"), ("left_knee", "left_foot"),
        ("right_hip", "right_knee"), ("right_knee", "right_foot"),
        ("left_shoulder", "left_elbow"), ("left_elbow", "left_wrist"),
        ("right_shoulder", "right_elbow"), ("right_elbow", "right_wrist"),
    ],
    "bvh_lafan1": [
        ("Hips", "Spine2"),
        ("LeftUpLeg", "LeftLeg"), ("LeftLeg", "LeftFootMod"),
        ("RightUpLeg", "RightLeg"), ("RightLeg", "RightFootMod"),
        ("LeftArm", "LeftForeArm"), ("LeftForeArm", "LeftHand"),
        ("RightArm", "RightForeArm"), ("RightForeArm", "RightHand"),
    ],
    "bvh_nokov": [
        ("Hips", "Spine2"),
        ("LeftUpLeg", "LeftLeg"), ("LeftLeg", "LeftFootMod"),
        ("RightUpLeg", "RightLeg"), ("RightLeg", "RightFootMod"),
        ("LeftArm", "LeftForeArm"), ("LeftForeArm", "LeftHand"),
        ("RightArm", "RightForeArm"), ("RightForeArm", "RightHand"),
    ],
    "bvh_xsens": [
        ("Hips", "Chest4"),
        ("LeftHip", "LeftKnee"), ("LeftKnee", "LeftFootMod"),
        ("RightHip", "RightKnee"), ("RightKnee", "RightFootMod"),
        ("LeftShoulder", "LeftElbow"), ("LeftElbow", "LeftWrist"),
        ("RightShoulder", "RightElbow"), ("RightElbow", "RightWrist"),
    ],
    "bvh_fzmotion": [
        ("Hips", "Chest"),
        ("LeftUpLeg", "LeftLeg"), ("LeftLeg", "LeftFootMod"),
        ("RightUpLeg", "RightLeg"), ("RightLeg", "RightFootMod"),
        ("LeftArm", "LeftForeArm"), ("LeftForeArm", "LeftHand"),
        ("RightArm", "RightForeArm"), ("RightForeArm", "RightHand"),
    ],
    "bvh_noitom": [
        ("Hips", "Spine2"),
        ("LeftUpLeg", "LeftLeg"), ("LeftLeg", "LeftFootMod"),
        ("RightUpLeg", "RightLeg"), ("RightLeg", "RightFootMod"),
        ("LeftArm", "LeftForeArm"), ("LeftForeArm", "LeftHand"),
        ("RightArm", "RightForeArm"), ("RightForeArm", "RightHand"),
    ],
}


def axis_aligned_quaternions() -> list[np.ndarray]:
    """The 24 rotations that map coordinate axes onto coordinate axes, as wxyz quaternions."""
    quats = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product([1, -1], repeat=3):
            matrix = np.zeros((3, 3))
            for row, (col, sign) in enumerate(zip(perm, signs)):
                matrix[row, col] = sign
            if abs(np.linalg.det(matrix) - 1.0) > 1e-9:
                continue
            quats.append(R.from_matrix(matrix).as_quat(scalar_first=True))
    return quats


def load_frames(src_human: str, motion_file: str):
    if src_human == "smplx":
        import pathlib

        from general_motion_retargeting.utils.smpl import get_smplx_data_offline_fast, load_smplx_file

        here = pathlib.Path(__file__).parent
        data, model, output, height = load_smplx_file(motion_file, here / ".." / "assets" / "body_models")
        frames, _ = get_smplx_data_offline_fast(data, model, output, tgt_fps=30)
        return frames, height
    if src_human in ("bvh_lafan1", "bvh_nokov", "bvh_fzmotion", "bvh_noitom"):
        from general_motion_retargeting.utils.lafan1 import load_bvh_file

        frames, height = load_bvh_file(motion_file, format=src_human.replace("bvh_", ""))
        return frames, height
    if src_human == "bvh_xsens":
        from general_motion_retargeting.utils.xsens import load_xsens_file

        class Args:
            bvh_file = motion_file
            scale = 0.01
            start = 0
            end = None
            reset_to_zero = True
            bvh_format = "3DSM"

        frames, height, _ = load_xsens_file(Args())
        return frames, height
    raise ValueError(f"Unsupported source: {src_human}")


def segment_errors(
    robot: str, src_human: str, frames, height: float, num_frames: int, retarget: GMR = None
) -> dict:
    """Angle between each robot limb segment and the human segment it should follow, in degrees."""
    if retarget is None:
        retarget = GMR(actual_human_height=height, src_human=src_human, tgt_robot=robot, verbose=False)
    else:
        # The solver warm-starts from the previous frame, so a reused instance has to be reset or
        # the score depends on whatever was solved before it.
        retarget.configuration.update(retarget.model.qpos0.copy())
    config = json.load(open(IK_CONFIG_DICT[src_human][robot]))
    link_of_human = {entry[0]: link for link, entry in config["ik_match_table1"].items()}

    data = mj.MjData(retarget.model)
    step = max(1, len(frames) // num_frames)
    errors = {segment: [] for segment in SEGMENTS[src_human]}
    orientation_residual = []

    for i in range(0, len(frames), step):
        qpos = retarget.retarget(frames[i])
        data.qpos[:] = qpos
        mj.mj_forward(retarget.model, data)

        # How far the robot ends up from the orientation it was asked for. A rot_offset with the
        # wrong twist commands an orientation the joints cannot reach, so this residual is what
        # exposes it - the limb directions below cannot see rotation about the limb's own axis.
        for human_body, link in link_of_human.items():
            if human_body not in retarget.scaled_human_data:
                continue
            target = R.from_quat(np.asarray(retarget.scaled_human_data[human_body][1]), scalar_first=True)
            achieved = R.from_quat(
                data.xquat[mj.mj_name2id(retarget.model, mj.mjtObj.mjOBJ_BODY, link)].copy(),
                scalar_first=True,
            )
            orientation_residual.append(np.degrees((target.inv() * achieved).magnitude()))

        for proximal, distal in SEGMENTS[src_human]:
            if proximal not in link_of_human or distal not in link_of_human:
                continue
            human_vec = (
                np.asarray(retarget.scaled_human_data[distal][0])
                - np.asarray(retarget.scaled_human_data[proximal][0])
            )
            robot_vec = (
                data.xpos[mj.mj_name2id(retarget.model, mj.mjtObj.mjOBJ_BODY, link_of_human[distal])]
                - data.xpos[mj.mj_name2id(retarget.model, mj.mjtObj.mjOBJ_BODY, link_of_human[proximal])]
            )
            norms = np.linalg.norm(human_vec) * np.linalg.norm(robot_vec)
            if norms < 1e-9:
                continue
            cos = np.clip(float(human_vec @ robot_vec) / norms, -1.0, 1.0)
            errors[(proximal, distal)].append(np.degrees(np.arccos(cos)))

    result = {segment: float(np.mean(values)) for segment, values in errors.items() if values}
    if orientation_residual:
        result[("orientation", "residual")] = float(np.mean(orientation_residual))
    return result


# Kinematic chains out of the root, as (semantic slot) lists. human_scale_table scales a body's
# position relative to the root, so the scale that keeps a chain reachable is the ratio of the
# robot's cumulative bone length to the human's - both pose independent, unlike a raw
# root-to-body distance measured in two different rest poses.
CHAINS = {
    "smplx": [["pelvis", "spine3"], ["left_hip", "left_knee", "left_foot"], ["right_hip", "right_knee", "right_foot"],
              ["left_shoulder", "left_elbow", "left_wrist"], ["right_shoulder", "right_elbow", "right_wrist"]],
    "bvh_lafan1": [["Hips", "Spine2"], ["LeftUpLeg", "LeftLeg", "LeftFootMod"], ["RightUpLeg", "RightLeg", "RightFootMod"],
                   ["LeftArm", "LeftForeArm", "LeftHand"], ["RightArm", "RightForeArm", "RightHand"]],
    "bvh_nokov": [["Hips", "Spine2"], ["LeftUpLeg", "LeftLeg", "LeftFootMod"], ["RightUpLeg", "RightLeg", "RightFootMod"],
                  ["LeftArm", "LeftForeArm", "LeftHand"], ["RightArm", "RightForeArm", "RightHand"]],
    "bvh_xsens": [["Hips", "Chest4"], ["LeftHip", "LeftKnee", "LeftFootMod"], ["RightHip", "RightKnee", "RightFootMod"],
                  ["LeftShoulder", "LeftElbow", "LeftWrist"], ["RightShoulder", "RightElbow", "RightWrist"]],
    "bvh_fzmotion": [["Hips", "Chest"], ["LeftUpLeg", "LeftLeg", "LeftFootMod"], ["RightUpLeg", "RightLeg", "RightFootMod"],
                     ["LeftArm", "LeftForeArm", "LeftHand"], ["RightArm", "RightForeArm", "RightHand"]],
    "bvh_noitom": [["Hips", "Spine2"], ["LeftUpLeg", "LeftLeg", "LeftFootMod"], ["RightUpLeg", "RightLeg", "RightFootMod"],
                   ["LeftArm", "LeftForeArm", "LeftHand"], ["RightArm", "RightForeArm", "RightHand"]],
}


def derive_chain_scales(
    robot: str, src_human: str, motion_file: str, config_path: str, num_frames: int, margin: float = 1.0
) -> None:
    """Rewrite human_scale_table so each chain's reach matches the robot's own bone lengths.

    ``margin`` shrinks the result, which keeps targets inside the limb's reach; measured to make
    little difference on the sources here, so it defaults to off.
    """
    frames, height = load_frames(src_human, motion_file)
    config = json.load(open(config_path))
    link_of_human = {entry[0]: link for link, entry in config["ik_match_table1"].items()}
    root_body = config["human_root_name"]
    root_link = config["robot_root_name"]

    model = mj.MjModel.from_xml_path(resolve_robot_xml(robot))
    data = mj.MjData(model)
    data.qpos[:] = 0
    if model.nq >= 7:
        data.qpos[3] = 1.0
    mj.mj_forward(model, data)

    def robot_pos(link):
        return data.xpos[mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, link)]

    step = max(1, len(frames) // num_frames)
    sample = frames[::step]

    # The solver multiplies every entry by human_height_assumption / actual_height when it loads
    # the config, so the file has to hold the value divided by that ratio.
    ratio = human_height_scale_ratio(config, height)

    scales = {}
    for chain in CHAINS[src_human]:
        # The root heads the spine chain so its offset can be solved, but it is the reference the
        # scales are measured from, not something to scale.
        chain = [b for b in chain if b in link_of_human and b in sample[0] and b != root_body]
        human_total, robot_total = 0.0, 0.0
        previous_human, previous_link = root_body, root_link
        for body in chain:
            # Bone lengths are constant; the median guards against a noisy frame.
            human_total += float(np.median([
                np.linalg.norm(np.asarray(f[body][0]) - np.asarray(f[previous_human][0])) for f in sample
            ]))
            robot_total += float(np.linalg.norm(robot_pos(link_of_human[body]) - robot_pos(previous_link)))
            scales[body] = round(margin * robot_total / human_total / ratio, 3)
            previous_human, previous_link = body, link_of_human[body]

    for body, value in scales.items():
        print(f"  {body:14s} {config['human_scale_table'].get(body, float('nan')):5.3f} -> {value}")
    config["human_scale_table"].update(scales)
    with open(config_path, "w") as f:
        json.dump(config, f, indent=4)
        f.write("\n")


# A chain's last body has no next bone to align to, so it borrows one: the source skeleton's end
# site or next joint, paired with the robot link hanging off that joint.
TERMINAL_BONES = {
    # SMPL-X's "foot" is already the toe joint and has no child, so only the wrists get a bone.
    "smplx": {
        "left_wrist": ("left_middle1", "left_hand_manip"),
        "right_wrist": ("right_middle1", "right_hand_manip"),
    },
    # LAFAN1 and Nokov end at the hand joint, so only the feet have a bone left to align to.
    "bvh_lafan1": {
        "LeftFootMod": ("LeftToe", "contact_foot_tip_L"), "RightFootMod": ("RightToe", "contact_foot_tip_R"),
    },
    "bvh_nokov": {
        "LeftFootMod": ("LeftToeBase", "contact_foot_tip_L"), "RightFootMod": ("RightToeBase", "contact_foot_tip_R"),
    },
    "bvh_xsens": {
        "LeftFootMod": ("LeftToe_end_site", "contact_foot_tip_L"),
        "RightFootMod": ("RightToe_end_site", "contact_foot_tip_R"),
        "LeftWrist": ("LeftWrist_end_site", "left_hand_manip"),
        "RightWrist": ("RightWrist_end_site", "right_hand_manip"),
    },
    "bvh_fzmotion": {
        "LeftFootMod": ("LeftToe", "contact_foot_tip_L"), "RightFootMod": ("RightToe", "contact_foot_tip_R"),
        "LeftHand": ("LeftHandPalm", "left_hand_manip"), "RightHand": ("RightHandPalm", "right_hand_manip"),
    },
    # Noitom's inertial skeleton has no toe joint, so the ankle keeps its own frame.
    "bvh_noitom": {
        "LeftHand": ("LeftHandMiddle1", "left_hand_manip"), "RightHand": ("RightHandMiddle1", "right_hand_manip"),
    },
}


def mirror_quaternion(quat: np.ndarray) -> np.ndarray:
    """Reflect a rotation across the sagittal plane: (w, x, y, z) -> (w, -x, y, -z)."""
    w, x, y, z = quat
    return np.array([w, -x, y, -z])


def mirror_name(body: str) -> str | None:
    if body.startswith("Left"):
        return "Right" + body[4:]
    if body.startswith("left_"):
        return "right_" + body[5:]
    return None


def derive_aligned_offsets(robot: str, src_human: str, motion_file: str, config_path: str, num_frames: int) -> None:
    """Solve each body's rot_offset from the bone it drives instead of searching for it.

    ``rot_offset`` maps the source skeleton's bone frame onto the robot link frame, and the tables
    weight shoulder and hip orientation at 100, so this offset - not the position targets - is what
    decides where a limb points. Writing the bone direction in each frame makes it solvable: with
    ``v_h`` the bone direction in the human parent's frame and ``v_r`` the same bone in the robot
    link's frame, the offset is the rotation carrying ``v_r`` onto ``v_h``. Both are constant, so
    one frame is enough; the median over several only guards against a bad frame.
    """
    if src_human in FIXED_ORIENTATION_SOURCES:
        raise ValueError(f"{src_human} uses fixed frame offsets; --tune align is not supported")
    frames, height = load_frames(src_human, motion_file)
    config = json.load(open(config_path))
    link_of_human = {entry[0]: link for link, entry in config["ik_match_table1"].items()}

    model = mj.MjModel.from_xml_path(resolve_robot_xml(robot))
    data = mj.MjData(model)
    data.qpos[:] = 0
    if model.nq >= 7:
        data.qpos[3] = 1.0
    mj.mj_forward(model, data)

    def robot_frame(link):
        body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, link)
        return data.xpos[body_id], R.from_quat(data.xquat[body_id], scalar_first=True)

    step = max(1, len(frames) // num_frames)
    sample = frames[::step]

    offsets = {}
    for chain in CHAINS[src_human]:
        chain = [body for body in chain if body in link_of_human and body in sample[0]]
        pairs = list(zip(chain, chain[1:]))
        if chain:
            terminal = TERMINAL_BONES.get(src_human, {}).get(chain[-1])
            if terminal and terminal[0] in sample[0]:
                pairs.append((chain[-1], terminal))

        for parent, child in pairs:
            child_human, child_link = child if isinstance(child, tuple) else (child, link_of_human[child])
            if mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, child_link) < 0:
                continue
            child = child_human
            human_dirs = []
            for frame in sample:
                bone = np.asarray(frame[child][0]) - np.asarray(frame[parent][0])
                rotation = R.from_quat(np.asarray(frame[parent][1]), scalar_first=True)
                human_dirs.append(rotation.inv().apply(bone) / max(np.linalg.norm(bone), 1e-9))
            v_h = np.median(np.stack(human_dirs), axis=0)
            v_h /= max(np.linalg.norm(v_h), 1e-9)

            parent_pos, parent_rot = robot_frame(link_of_human[parent])
            child_pos, _ = robot_frame(child_link)
            v_r = parent_rot.inv().apply(child_pos - parent_pos)
            if np.linalg.norm(v_r) < 1e-6 or np.linalg.norm(v_h) < 1e-6:
                # Coincident frames carry no direction to align.
                continue
            v_r /= np.linalg.norm(v_r)

            offsets[parent] = R.align_vectors(v_h[None, :], v_r[None, :])[0]

    # A joint that ends its chain has no outgoing bone, but it still has the incoming one: align
    # the parent-to-body direction as written in the body's own frame. Inheriting the parent's
    # offset instead would assume the two frames coincide, which they do not on these skeletons.
    from_incoming = []
    for chain in CHAINS[src_human]:
        chain = [body for body in chain if body in link_of_human and body in sample[0]]
        for parent, body in zip(chain, chain[1:]):
            if body in offsets:
                continue
            human_dirs = []
            for frame in sample:
                bone = np.asarray(frame[body][0]) - np.asarray(frame[parent][0])
                rotation = R.from_quat(np.asarray(frame[body][1]), scalar_first=True)
                human_dirs.append(rotation.inv().apply(bone) / max(np.linalg.norm(bone), 1e-9))
            v_h = np.median(np.stack(human_dirs), axis=0)

            body_pos, body_rot = robot_frame(link_of_human[body])
            parent_pos, _ = robot_frame(link_of_human[parent])
            v_r = body_rot.inv().apply(body_pos - parent_pos)
            if np.linalg.norm(v_r) < 1e-6 or np.linalg.norm(v_h) < 1e-6:
                continue
            offsets[body] = R.align_vectors(
                (v_h / np.linalg.norm(v_h))[None, :], (v_r / np.linalg.norm(v_r))[None, :]
            )[0]
            from_incoming.append(body)

    # Mirroring the left solution onto the right is tempting but wrong here: the sagittal plane is
    # not the same coordinate plane in every skeleton's bone frames, and align_vectors already
    # gives each side its own correct direction. Only the twist has to be kept symmetric, which
    # calibrate_roll does by rolling a pair by opposite angles.
    mirrored = []

    for table in ("ik_match_table1", "ik_match_table2"):
        for entry in config[table].values():
            if entry[0] in offsets:
                quat = offsets[entry[0]].as_quat(scalar_first=True)
                entry[4] = [round(float(v), 8) for v in quat]
    with open(config_path, "w") as f:
        json.dump(config, f, indent=4)
        f.write("\n")
    for body, rotation in offsets.items():
        note = " (from incoming bone)" if body in from_incoming else ""
        note += " (mirrored)" if body in mirrored else ""
        print(f"  {body:14s} -> {np.round(rotation.as_quat(scalar_first=True), 4)}{note}")
    print(
        f"  ({len(offsets)} bodies solved, {len(from_incoming)} from the incoming bone, "
        f"{len(mirrored)} mirrored onto the right side)"
    )


def calibrate_roll(robot: str, src_human: str, motion_file: str, config_path: str, num_frames: int) -> None:
    """Search the one degree of freedom bone alignment leaves free: roll about the bone axis.

    ``derive_aligned_offsets`` pins each limb's direction but not its twist, and the orientation
    tasks are weighted high enough that a bad twist drives joints into their limits. Rolling about
    the bone keeps the direction and is a single scalar per body, so a coarse sweep suffices.
    """
    if src_human in FIXED_ORIENTATION_SOURCES:
        raise ValueError(f"{src_human} uses fixed frame offsets; --tune roll is not supported")
    frames, height = load_frames(src_human, motion_file)
    config = json.load(open(config_path))
    link_of_human = {entry[0]: link for link, entry in config["ik_match_table1"].items()}

    model = mj.MjModel.from_xml_path(resolve_robot_xml(robot))
    data = mj.MjData(model)
    data.qpos[:] = 0
    if model.nq >= 7:
        data.qpos[3] = 1.0
    mj.mj_forward(model, data)

    retarget = GMR(actual_human_height=height, src_human=src_human, tgt_robot=robot, verbose=False)

    def current_score() -> float:
        errors = segment_errors(robot, src_human, frames, height, num_frames, retarget=retarget)
        return combined_score(errors)

    bone_axis = {}
    for chain in CHAINS[src_human]:
        chain = [b for b in chain if b in link_of_human and b in frames[0]]
        for parent, child in zip(chain, chain[1:]):
            parent_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, link_of_human[parent])
            child_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, link_of_human[child])
            axis = R.from_quat(data.xquat[parent_id], scalar_first=True).inv().apply(
                data.xpos[child_id] - data.xpos[parent_id]
            )
            if np.linalg.norm(axis) > 1e-6:
                bone_axis[parent] = axis / np.linalg.norm(axis)

    def set_offset(body, rotation):
        for offsets in (retarget.rot_offsets1, retarget.rot_offsets2):
            if body in offsets:
                offsets[body] = rotation

    def has_offset(body):
        return body in retarget.rot_offsets1 or body in retarget.rot_offsets2

    def get_offset(body):
        return retarget.rot_offsets1.get(body, retarget.rot_offsets2.get(body))

    best = current_score()
    print(f"starting score {best:.2f}deg")
    chosen = {}
    # Roll a left body and its mirror together, by opposite angles. Searching them separately is
    # what previously left one arm rolled and the other not, breaking the robot's symmetry.
    handled = set()
    for body, axis in bone_axis.items():
        if body in handled or not has_offset(body):
            continue
        other = mirror_name(body)
        pair = [(body, axis, 1.0)]
        if other and other in bone_axis and has_offset(other):
            pair.append((other, bone_axis[other], -1.0))
            handled.add(other)
        handled.add(body)

        bases = {name: get_offset(name) for name, _, _ in pair}
        baseline, best_angle = best, None
        for angle in range(15, 360, 15):
            for name, bone, sign in pair:
                set_offset(name, bases[name] * R.from_rotvec(np.radians(sign * angle) * bone))
            trial = current_score()
            if trial < best - 1.0:
                best, best_angle = trial, angle
        for name, bone, sign in pair:
            final = bases[name] if best_angle is None else \
                bases[name] * R.from_rotvec(np.radians(sign * best_angle) * bone)
            set_offset(name, final)
            if best_angle is not None:
                chosen[name] = [round(float(v), 8) for v in final.as_quat(scalar_first=True)]
        names = " + ".join(name for name, _, _ in pair)
        print(f"  {names:28s} {baseline:6.2f} -> {best:6.2f}deg  roll {best_angle if best_angle is not None else 0}deg")

    for table in ("ik_match_table1", "ik_match_table2"):
        for entry in config[table].values():
            if entry[0] in chosen:
                entry[4] = chosen[entry[0]]
    with open(config_path, "w") as f:
        json.dump(config, f, indent=4)
        f.write("\n")
    print(f"final score {best:.2f}deg")


def lowest_body_height(retarget: GMR, frames, num_frames: int) -> float:
    """Lowest point the robot reaches over the clip. Should be ~0: below is sunken, above floats."""
    retarget.configuration.update(retarget.model.qpos0.copy())
    data = mj.MjData(retarget.model)
    step = max(1, len(frames) // num_frames)
    lowest = np.inf
    for i in range(0, len(frames), step):
        data.qpos[:] = retarget.retarget(frames[i])
        mj.mj_forward(retarget.model, data)
        lowest = min(lowest, float(data.xpos[1:, 2].min()))
    return lowest


def calibrate_ground(
    robot: str, src_human: str, motion_file: str, config_path: str, num_frames: int
) -> None:
    """Set the root body's scale so the robot's lowest point sits on the ground.

    The root scale multiplies the human root's absolute position, so it sets how high the robot's
    pelvis is carried; too large and the legs cannot reach the floor.
    """
    frames, height = load_frames(src_human, motion_file)
    retarget = GMR(actual_human_height=height, src_human=src_human, tgt_robot=robot, verbose=False)
    config = json.load(open(config_path))
    root = config["human_root_name"]
    ratio = human_height_scale_ratio(config, height)

    best_scale, best_gap = None, np.inf
    for scale in np.round(np.arange(0.60, 1.35, 0.025), 4):
        retarget.human_scale_table[root] = float(scale) * ratio
        gap = abs(lowest_body_height(retarget, frames, num_frames))
        if gap < best_gap:
            best_scale, best_gap = float(scale), gap

    config["human_scale_table"][root] = best_scale
    with open(config_path, "w") as f:
        json.dump(config, f, indent=4)
        f.write("\n")
    print(f"  root scale {root} -> {best_scale} (lowest body point {best_gap:.3f}m off the ground)")


ORIENTATION_KEY = ("orientation", "residual")


def combined_score(errors: dict) -> float:
    """Half limb direction, half orientation residual.

    Averaging every entry equally would bury the orientation residual under nine limb segments,
    and it is the term that catches a rot_offset twisted about its own bone.
    """
    directions = [v for k, v in errors.items() if k != ORIENTATION_KEY]
    direction = float(np.mean(directions)) if directions else 0.0
    if ORIENTATION_KEY not in errors:
        return direction
    return 0.5 * direction + 0.5 * errors[ORIENTATION_KEY]


def score(robot: str, src_human: str, motion_file: str, num_frames: int) -> float:
    frames, height = load_frames(src_human, motion_file)
    errors = segment_errors(robot, src_human, frames, height, num_frames)
    for segment, value in errors.items():
        print(f"  {segment[0]:>12s} -> {segment[1]:<14s} {value:6.1f}deg")
    total = combined_score(errors)
    print(f"  {'combined':>12s} {'':<18s} {total:6.1f}deg")
    return total


def calibrate(
    robot: str,
    src_human: str,
    motion_file: str,
    config_path: str,
    num_frames: int,
    only_bodies: list[str] = None,
    min_gain: float = 2.0,
    tune: str = "offsets",
) -> None:
    """Greedily replace each body's rot_offset with the axis-aligned value that scores best.

    The search mutates the live solver's parsed offsets rather than rewriting and reloading the
    config, so the robot model is built once instead of once per candidate.
    """
    frames, height = load_frames(src_human, motion_file)
    candidates = axis_aligned_quaternions()
    shutil.copy(config_path, config_path + ".bak")

    retarget = GMR(actual_human_height=height, src_human=src_human, tgt_robot=robot, verbose=False)

    def current_score() -> float:
        errors = segment_errors(robot, src_human, frames, height, num_frames, retarget=retarget)
        return combined_score(errors)

    def set_offset(body: str, rotation: R) -> None:
        for offsets in (retarget.rot_offsets1, retarget.rot_offsets2):
            if body in offsets:
                offsets[body] = rotation

    best = current_score()
    print(f"starting score {best:.2f}deg")

    config = json.load(open(config_path))
    bodies = [entry[0] for entry in config["ik_match_table1"].values()]
    if only_bodies:
        bodies = [b for b in bodies if any(key.lower() in b.lower() for key in only_bodies)]
    chosen_offsets = {}

    for body in bodies:
        baseline = best
        original = retarget.rot_offsets1.get(body) or retarget.rot_offsets2.get(body)
        best_quat = None
        for quat in candidates:
            set_offset(body, R.from_quat(quat, scalar_first=True))
            trial = current_score()
            # Require a real gain: the metric cannot see rotation about a limb's own axis, so a
            # marginal improvement is as likely to be noise as a better convention.
            if trial < best - min_gain:
                best, best_quat = trial, quat
        set_offset(body, original if best_quat is None else R.from_quat(best_quat, scalar_first=True))
        if best_quat is not None:
            chosen_offsets[body] = [round(float(v), 8) for v in best_quat]
        print(f"  {body:14s} {baseline:6.2f} -> {best:6.2f}deg  {'kept' if best_quat is None else np.round(best_quat, 3)}")

    chosen_scales = {}
    if tune in ("scales", "both"):
        # human_scale_table entries are multiplied by human_height_assumption / actual_height at
        # load time, so the live values have to be divided by that ratio to go back into the file.
        ratio = human_height_scale_ratio(config, height)
        candidates_scale = np.round(np.arange(0.20, 1.65, 0.05), 3)
        for body in bodies:
            if body not in retarget.human_scale_table:
                continue
            baseline, original = best, retarget.human_scale_table[body]
            best_scale = None
            for scale in candidates_scale:
                retarget.human_scale_table[body] = float(scale) * ratio
                trial = current_score()
                if trial < best - min_gain:
                    best, best_scale = trial, float(scale)
            retarget.human_scale_table[body] = original if best_scale is None else best_scale * ratio
            if best_scale is not None:
                chosen_scales[body] = best_scale
            print(f"  scale {body:12s} {baseline:6.2f} -> {best:6.2f}deg  {'kept' if best_scale is None else best_scale}")

    for table in ("ik_match_table1", "ik_match_table2"):
        for entry in config[table].values():
            if entry[0] in chosen_offsets:
                entry[4] = chosen_offsets[entry[0]]
    config["human_scale_table"].update(chosen_scales)
    with open(config_path, "w") as f:
        json.dump(config, f, indent=4)
        f.write("\n")

    print(f"final score {best:.2f}deg (backup at {config_path}.bak)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Score or calibrate IK rotation offsets.")
    parser.add_argument("mode", choices=["score", "calibrate"])
    parser.add_argument("--robot", required=True)
    parser.add_argument("--src", required=True, help="src_human key, e.g. bvh_nokov")
    parser.add_argument("--motion_file", required=True)
    parser.add_argument("--config", help="config to calibrate in place (calibrate mode)")
    parser.add_argument("--num_frames", type=int, default=20)
    parser.add_argument(
        "--bodies",
        nargs="*",
        help="only calibrate bodies whose name contains one of these, e.g. Arm Hand Wrist Elbow",
    )
    parser.add_argument("--min_gain", type=float, default=2.0, help="degrees of gain required to accept")
    parser.add_argument(
        "--chain_margin", type=float, default=1.0,
        help="shrink factor on the chain scales, so targets stay inside the limb's reach",
    )
    parser.add_argument(
        "--tune",
        choices=["offsets", "scales", "both", "ground", "chain", "align", "roll"],
        default="offsets",
    )
    args = parser.parse_args()

    if args.mode == "score":
        score(args.robot, args.src, args.motion_file, args.num_frames)
    else:
        if not args.config:
            parser.error("--config is required in calibrate mode")
        if args.tune == "roll":
            calibrate_roll(args.robot, args.src, args.motion_file, args.config, args.num_frames)
            raise SystemExit(0)
        if args.tune == "align":
            derive_aligned_offsets(args.robot, args.src, args.motion_file, args.config, args.num_frames)
            raise SystemExit(0)
        if args.tune == "chain":
            derive_chain_scales(
                args.robot, args.src, args.motion_file, args.config, args.num_frames, args.chain_margin
            )
            raise SystemExit(0)
        if args.tune == "ground":
            calibrate_ground(args.robot, args.src, args.motion_file, args.config, args.num_frames)
            raise SystemExit(0)
        calibrate(
            args.robot, args.src, args.motion_file, args.config, args.num_frames,
            only_bodies=args.bodies, min_gain=args.min_gain, tune=args.tune,
        )
