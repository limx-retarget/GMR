"""Check a Luna .npy motion against what luna-beyondmimic will accept.

The downstream gate is ``scripts/npy_to_npz.py`` in luna-beyondmimic, which needs Isaac Lab to be
installed. This reproduces its schema checks so a retarget can be validated locally.

.. code-block:: bash

    python scripts/verify_luna_npy.py --motion_file output/luna_motion.npy
"""

import argparse

import numpy as np

# HU_L04_PARALLEL_JOINT_ORDER of luna-beyondmimic minus the joints the retargeter does not
# produce: the 6 achilles motors and the 18 DoF of the 6 spherical rod joints.
LUNA_JOINT_NAMES = (
    "left_hip_pitch_joint",
    "right_hip_pitch_joint",
    "waist_yaw_joint",
    "left_hip_roll_joint",
    "right_hip_roll_joint",
    "waist_roll_joint",
    "left_hip_yaw_joint",
    "right_hip_yaw_joint",
    "waist_pitch_joint",
    "left_knee_joint",
    "right_knee_joint",
    "head_yaw_joint",
    "left_shoulder_pitch_joint",
    "right_shoulder_pitch_joint",
    "left_ankle_pitch_joint",
    "right_ankle_pitch_joint",
    "head_pitch_joint",
    "left_shoulder_roll_joint",
    "right_shoulder_roll_joint",
    "left_ankle_roll_joint",
    "right_ankle_roll_joint",
    "left_shoulder_yaw_joint",
    "right_shoulder_yaw_joint",
    "left_elbow_joint",
    "right_elbow_joint",
    "left_wrist_yaw_joint",
    "right_wrist_yaw_joint",
)

# HU_L04_PARALLEL_BODY_ORDER of luna-beyondmimic. Bodies it carries but the motion does not are
# zero-filled there; bodies the motion carries but it does not are an error.
LUNA_BODY_NAMES = (
    "base_link",
    "base_imu",
    "left_hip_pitch_link",
    "right_hip_pitch_link",
    "waist_yaw_link",
    "left_hip_roll_link",
    "right_hip_roll_link",
    "waist_roll_link",
    "left_hip_yaw_link",
    "right_hip_yaw_link",
    "waist_pitch_link",
    "left_knee_link",
    "right_knee_link",
    "head_yaw_link",
    "left_shoulder_pitch_link",
    "right_shoulder_pitch_link",
    "waist_A_link",
    "waist_B_link",
    "left_A_achilles_link",
    "left_B_achilles_link",
    "left_ankle_pitch_link",
    "right_A_achilles_link",
    "right_B_achilles_link",
    "right_ankle_pitch_link",
    "head_pitch_link",
    "left_shoulder_roll_link",
    "right_shoulder_roll_link",
    "waist_A_rod_link",
    "waist_B_rod_link",
    "left_A_achilles_rod_link",
    "left_B_achilles_rod_link",
    "left_ankle_roll_link",
    "right_A_achilles_rod_link",
    "right_B_achilles_rod_link",
    "right_ankle_roll_link",
    "left_shoulder_yaw_link",
    "right_shoulder_yaw_link",
    "contact_foot_center_L",
    "contact_foot_heel_L",
    "contact_foot_tip_L",
    "contact_foot_center_R",
    "contact_foot_heel_R",
    "contact_foot_tip_R",
    "left_elbow_link",
    "right_elbow_link",
    "left_wrist_yaw_link",
    "right_wrist_yaw_link",
    "left_hand_contact",
    "left_hand_manip",
    "right_hand_contact",
    "right_hand_manip",
)

REQUIRED_KEYS = ("fps", "dof_names", "body_names", "dof_pos_vel", "body_states")


def body_z_axis_alignment(quat: np.ndarray, order: str) -> float:
    """Mean cosine between the body's own z axis and world +z, under the given component order."""
    x_index, y_index = (0, 1) if order == "xyzw" else (1, 2)
    x, y = quat[:, x_index], quat[:, y_index]
    return float((1.0 - 2.0 * (x * x + y * y)).mean())


def verify(motion_file: str) -> list[str]:
    """Return the list of failures; an empty list means the motion is accepted downstream."""
    failures = []
    motion = np.load(motion_file, allow_pickle=True).item()

    missing_keys = [k for k in REQUIRED_KEYS if k not in motion]
    if missing_keys:
        return [f"Missing keys: {missing_keys}"]

    dof_names = list(motion["dof_names"])
    body_names = list(motion["body_names"])
    dof_pos_vel = np.asarray(motion["dof_pos_vel"])
    body_states = np.asarray(motion["body_states"])
    frames = dof_pos_vel.shape[0]

    print(f"frames={frames}  joints={len(dof_names)}  bodies={len(body_names)}  fps={motion['fps']}")

    if dof_pos_vel.shape != (frames, len(dof_names), 2):
        failures.append(f"dof_pos_vel has shape {dof_pos_vel.shape}, expected {(frames, len(dof_names), 2)}.")
    if body_states.shape != (frames, len(body_names), 13):
        failures.append(f"body_states has shape {body_states.shape}, expected {(frames, len(body_names), 13)}.")

    unexpected_joints = sorted(set(dof_names) - set(LUNA_JOINT_NAMES))
    missing_joints = sorted(set(LUNA_JOINT_NAMES) - set(dof_names))
    if unexpected_joints or missing_joints:
        failures.append(
            "Joint name mismatch.\n"
            f"  missing from motion: {missing_joints}\n"
            f"  unexpected in motion: {unexpected_joints}"
        )

    unexpected_bodies = sorted(set(body_names) - set(LUNA_BODY_NAMES))
    if unexpected_bodies:
        failures.append(f"Bodies absent from the articulation: {unexpected_bodies}")
    zero_filled = [n for n in LUNA_BODY_NAMES if n not in body_names]
    if zero_filled:
        print(f"[INFO] {len(zero_filled)} bodies will be zero-filled downstream: {zero_filled}")

    if body_states.shape[-1] == 13:
        quat = body_states[:, :, 3:7]
        quat_error = float(np.abs(np.linalg.norm(quat, axis=-1) - 1.0).max())
        if quat_error > 1e-3:
            failures.append(f"Quaternions are not unit length (max deviation {quat_error:.4g}).")

        if "base_link" in body_names:
            root_quat = quat[:, body_names.index("base_link")]
            as_xyzw = body_z_axis_alignment(root_quat, "xyzw")
            as_wxyz = body_z_axis_alignment(root_quat, "wxyz")
            if as_wxyz > as_xyzw:
                failures.append(
                    "The root orientation looks more upright read as wxyz "
                    f"(wxyz {as_wxyz:.4f} vs xyzw {as_xyzw:.4f}); body_states must store xyzw."
                )
            else:
                print(f"[INFO] Quaternion convention check passed (xyzw {as_xyzw:.4f} vs wxyz {as_wxyz:.4f}).")

    if not isinstance(motion["fps"], (int, np.integer)):
        failures.append(f"fps is {type(motion['fps']).__name__}, expected int.")

    # append_motion_tail refuses to freeze a final frame that is still moving.
    if frames > 1:
        final_speed = float(np.abs(dof_pos_vel[-1, :, 1]).max())
        if final_speed > 0.1:
            print(
                f"[WARN] The final frame still moves ({final_speed:.3f} rad/s). append_motion_tail "
                "will refuse to freeze it; trim the clip boundary with --frame_range."
            )

    return failures


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Verify a Luna .npy motion against luna-beyondmimic.")
    parser.add_argument("--motion_file", type=str, required=True, help="Path to the .npy motion.")
    args = parser.parse_args()

    problems = verify(args.motion_file)
    if problems:
        print("\n[FAIL]")
        for problem in problems:
            print(f"  - {problem}")
        raise SystemExit(1)
    print("\n[PASS] The motion matches the luna-beyondmimic schema.")
