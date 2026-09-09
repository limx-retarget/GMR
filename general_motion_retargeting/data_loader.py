
import pickle

import numpy as np

from .motion_export import (
    beyondmimic_to_legacy_view,
    is_beyondmimic_motion,
    is_luna_npy_motion,
    luna_npy_to_legacy_view,
)


def load_robot_motion(motion_file):
    """
    Load robot motion data from a pickle, npz or npy file.

    Supports:
    - luna-beyondmimic format: body_states, dof_pos_vel, dof_names, body_names, fps
    - BeyondMimic format: joint_pos, body_pos_w, body_quat_w (wxyz), ...
    - Legacy GMR format: root_pos, root_rot (xyzw), dof_pos
    """
    if str(motion_file).endswith(".npy"):
        motion_data = np.load(motion_file, allow_pickle=True).item()
    elif str(motion_file).endswith(".npz"):
        motion_data = dict(np.load(motion_file, allow_pickle=True))
    else:
        with open(motion_file, "rb") as f:
            motion_data = pickle.load(f)

    if is_luna_npy_motion(motion_data):
        motion_fps = float(motion_data["fps"])
        motion_dof_pos = np.asarray(motion_data["dof_pos_vel"])[:, :, 0]
        motion_local_body_pos = None
        motion_link_body_list = list(motion_data["body_names"])
        # root fields filled by luna_npy_to_legacy_view when robot_type is known
        motion_root_pos = None
        motion_root_rot = None
    elif is_beyondmimic_motion(motion_data):
        motion_fps = float(np.asarray(motion_data["fps"]).reshape(-1)[0])
        motion_dof_pos = np.asarray(motion_data["joint_pos"])
        motion_local_body_pos = None
        motion_link_body_list = list(motion_data.get("body_names", []))
        # root fields filled by beyondmimic_to_legacy_view when robot_type is known
        motion_root_pos = None
        motion_root_rot = None
    else:
        motion_fps = motion_data["fps"]
        motion_root_pos = motion_data["root_pos"]
        motion_root_rot = motion_data["root_rot"][:, [3, 0, 1, 2]]  # from xyzw to wxyz
        motion_dof_pos = motion_data["dof_pos"]
        motion_local_body_pos = motion_data.get("local_body_pos")
        motion_link_body_list = motion_data.get("link_body_list")

    return (
        motion_data,
        motion_fps,
        motion_root_pos,
        motion_root_rot,
        motion_dof_pos,
        motion_local_body_pos,
        motion_link_body_list,
    )


def load_robot_motion_for_viewer(motion_file, robot_type):
    """Load motion and return viewer-ready root pose + dof (wxyz root rotation)."""
    (
        motion_data,
        motion_fps,
        motion_root_pos,
        motion_root_rot,
        motion_dof_pos,
        motion_local_body_pos,
        motion_link_body_list,
    ) = load_robot_motion(motion_file)

    if is_luna_npy_motion(motion_data):
        motion_fps, motion_root_pos, motion_root_rot, motion_dof_pos = luna_npy_to_legacy_view(
            motion_data, robot_type
        )
    elif is_beyondmimic_motion(motion_data):
        motion_fps, motion_root_pos, motion_root_rot, motion_dof_pos = beyondmimic_to_legacy_view(
            motion_data, robot_type
        )

    return (
        motion_data,
        motion_fps,
        motion_root_pos,
        motion_root_rot,
        motion_dof_pos,
        motion_local_body_pos,
        motion_link_body_list,
    )
