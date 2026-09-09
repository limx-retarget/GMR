"""Generate the LimX IK configs for a capture format from the Unitree G1 config of that format.

The G1 configs carry the per-format source-skeleton conventions and task weights; what changes for
another robot is the link names, the body proportions, and the rotation offsets. Link frames of
both LimX robots are identity at rest, so a G1 offset transfers as ``offset * R_g1_link(rest)^-1``.
That is only a starting point for the arms - run ``_calibrate_ik_offsets.py calibrate`` afterwards.

.. code-block:: bash

    python scripts/_make_limx_configs.py --format bvh_nokov
"""

import argparse
import collections
import json

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation as R

from general_motion_retargeting.params import IK_CONFIG_ROOT, resolve_robot_xml

# G1 link -> LimX link. Both LimX robots share these names except for the wrist.
LINK_MAP = {
    "pelvis": "base_link",
    "torso_link": "waist_pitch_link",
    "left_hip_yaw_link": "left_hip_pitch_link",
    "right_hip_yaw_link": "right_hip_pitch_link",
    "left_hip_roll_link": "left_hip_pitch_link",
    "right_hip_roll_link": "right_hip_pitch_link",
    "left_knee_link": "left_knee_link",
    "right_knee_link": "right_knee_link",
    "left_ankle_roll_link": "left_ankle_roll_link",
    "right_ankle_roll_link": "right_ankle_roll_link",
    "left_shoulder_yaw_link": "left_shoulder_pitch_link",
    "right_shoulder_yaw_link": "right_shoulder_pitch_link",
    "left_elbow_link": "left_elbow_link",
    "right_elbow_link": "right_elbow_link",
}
WRIST_LINK = {"limx_oli_edu": "wrist_roll_link", "limx_luna": "wrist_yaw_link"}

# Source skeleton body names per semantic slot, used to carry the calibrated scale table over.
BODY_NAMES = {
    "bvh_lafan1": {"root": "Hips", "spine": "Spine2", "hip": "{S}UpLeg", "knee": "{S}Leg",
                   "foot": "{S}FootMod", "shoulder": "{S}Arm", "elbow": "{S}ForeArm", "wrist": "{S}Hand"},
    "bvh_nokov": {"root": "Hips", "spine": "Spine2", "hip": "{S}UpLeg", "knee": "{S}Leg",
                  "foot": "{S}FootMod", "shoulder": "{S}Arm", "elbow": "{S}ForeArm", "wrist": "{S}Hand"},
    "bvh_xsens": {"root": "Hips", "spine": "Chest4", "hip": "{S}Hip", "knee": "{S}Knee",
                  "foot": "{S}FootMod", "shoulder": "{S}Shoulder", "elbow": "{S}Elbow", "wrist": "{S}Wrist"},
    "bvh_fzmotion": {"root": "Hips", "spine": "Chest", "hip": "{S}UpLeg", "knee": "{S}Leg",
                     "foot": "{S}FootMod", "shoulder": "{S}Arm", "elbow": "{S}ForeArm", "wrist": "{S}Hand"},
    "bvh_noitom": {"root": "Hips", "spine": "Spine2", "hip": "{S}UpLeg", "knee": "{S}Leg",
                   "foot": "{S}FootMod", "shoulder": "{S}Arm", "elbow": "{S}ForeArm", "wrist": "{S}Hand"},
}
# Where a format's task weights and link choices come from. Formats Unitree ships a config for
# start from that; the rest start from the robot's own LAFAN1 config, since only the source body
# names differ and _calibrate_ik_offsets.py re-derives the scales and offsets from data anyway.
G1_CONFIG = {
    "bvh_nokov": "bvh_nokov_to_g1.json",
    "bvh_xsens": "bvh_xsens_to_g1.json",
}
LIMX_TEMPLATE = {"limx_oli_edu": "bvh_lafan1_to_oli_edu.json", "limx_luna": "bvh_lafan1_to_luna.json"}

OUTPUT = {
    "bvh_nokov": {"limx_oli_edu": "bvh_nokov_to_oli_edu.json", "limx_luna": "bvh_nokov_to_luna.json"},
    "bvh_xsens": {"limx_oli_edu": "bvh_xsens_to_oli_edu.json", "limx_luna": "bvh_xsens_to_luna.json"},
    "bvh_fzmotion": {"limx_oli_edu": "bvh_fzmotion_to_oli_edu.json", "limx_luna": "bvh_fzmotion_to_luna.json"},
    "bvh_noitom": {"limx_oli_edu": "bvh_noitom_to_oli_edu.json", "limx_luna": "bvh_noitom_to_luna.json"},
}
# The already calibrated LAFAN1 configs are where the LimX body proportions come from.
REFERENCE = {"limx_oli_edu": "bvh_lafan1_to_oli_edu.json", "limx_luna": "bvh_lafan1_to_luna.json"}


def semantic_slot(body_name: str, source: str) -> str | None:
    for slot, pattern in BODY_NAMES[source].items():
        for side in ("", "Left", "Right"):
            if pattern.format(S=side) == body_name:
                return slot
    return None


def rest_orientations(robot: str) -> dict:
    model = mj.MjModel.from_xml_path(resolve_robot_xml(robot))
    data = mj.MjData(model)
    data.qpos[:] = 0
    if model.nq >= 7:
        data.qpos[3] = 1.0
    mj.mj_forward(model, data)
    return {
        mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, i): R.from_quat(data.xquat[i], scalar_first=True)
        for i in range(1, model.nbody)
    }


def build(source: str, robot: str) -> str:
    from_g1 = source in G1_CONFIG
    template = G1_CONFIG[source] if from_g1 else LIMX_TEMPLATE[robot]
    template_source = "bvh_lafan1" if not from_g1 else source
    config = json.load(open(IK_CONFIG_ROOT / template), object_pairs_hook=collections.OrderedDict)

    reference_scale = json.load(open(IK_CONFIG_ROOT / REFERENCE[robot]))["human_scale_table"]
    scale_by_slot = {semantic_slot(name, "bvh_lafan1"): value for name, value in reference_scale.items()}

    def rename(body: str) -> str:
        slot = semantic_slot(body, template_source)
        if slot is None:
            return body
        side = "Left" if body.startswith("Left") else ("Right" if body.startswith("Right") else "")
        return BODY_NAMES[source][slot].format(S=side)

    config["robot_root_name"] = "base_link"
    config["human_root_name"] = rename(config["human_root_name"])
    config["human_scale_table"] = {
        rename(name): scale_by_slot.get(semantic_slot(name, template_source), value)
        for name, value in config["human_scale_table"].items()
    }

    g1_rest = rest_orientations("unitree_g1") if from_g1 else {}
    for table in ("ik_match_table1", "ik_match_table2"):
        remapped = collections.OrderedDict()
        for link, entry in config[table].items():
            entry = list(entry)
            entry[0] = rename(entry[0])
            if from_g1:
                # LimX link frames are identity at rest, so a G1 offset transfers by undoing the
                # G1 link's own rest orientation. Only a starting point for the arms - the
                # calibration re-derives them from bone directions.
                offset = R.from_quat(entry[4], scalar_first=True) * g1_rest[link].inv()
                entry[4] = [round(float(v), 8) for v in offset.as_quat(scalar_first=True)]
                if "wrist_yaw_link" in link:
                    link = f"{link.split('_')[0]}_{WRIST_LINK[robot]}"
                else:
                    link = LINK_MAP.get(link, link)
            remapped[link] = entry
        config[table] = remapped

    path = IK_CONFIG_ROOT / OUTPUT[source][robot]
    with open(path, "w") as f:
        json.dump(config, f, indent=4)
        f.write("\n")
    return str(path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate LimX IK configs from the G1 config.")
    parser.add_argument("--format", required=True, choices=sorted(OUTPUT), help="source format")
    parser.add_argument("--robot", choices=["limx_oli_edu", "limx_luna"], help="default: both")
    args = parser.parse_args()

    robots = [args.robot] if args.robot else ["limx_oli_edu", "limx_luna"]
    for robot in robots:
        print("wrote", build(args.format, robot))
