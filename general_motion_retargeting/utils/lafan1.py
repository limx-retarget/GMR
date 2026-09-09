import numpy as np
from scipy.spatial.transform import Rotation as R

import general_motion_retargeting.utils.lafan_vendor.utils as utils
from general_motion_retargeting.utils.lafan_vendor.extract import read_bvh

# Which joint lends the ankle its orientation. GMR's IK configs target a "FootMod" body that sits
# at the ankle but is oriented like the foot's forward direction, which is what the robot's
# ankle_roll link matches; skeletons that carry no toe joint have to reuse the ankle's own frame.
_FOOT_ORIENTATION_SOURCE = {
    "lafan1": "{side}Toe",
    "nokov": "{side}ToeBase",
    "fzmotion": "{side}Toe",
    "noitom": "{side}Foot",
}

# Y-up to Z-up, the matrix agmr uses. An alternative that also raises Y to Z but differs by 90
# degrees of yaw would leave the subject's left-right axis on the robot's X; this one puts it on Y,
# where the robot's is, so the targets and the robot agree without the root absorbing a quarter turn.
_Y_UP_TO_Z_UP = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])

_AXIS_TRANSFORM = {source: _Y_UP_TO_Z_UP for source in _FOOT_ORIENTATION_SOURCE}

# Every clip is shifted so its lowest point rests on z=0: these skeletons do not agree on where
# their floor is, and a raised floor pushes the leg targets past the robot's leg length.
_GROUND_NORMALISED_FORMATS = set(_FOOT_ORIENTATION_SOURCE)

# Every source changes basis the way agmr does it, R' = T R T^-1.
_SIMILARITY_TRANSFORM_FORMATS = set(_FOOT_ORIENTATION_SOURCE)

# Height is a skeleton property, not the vertical extent of an arbitrary motion frame. These
# chains omit the lateral hip offset and the mostly-horizontal foot segment, then combine the
# pelvis-to-head and hip-to-ankle lengths. Noitom has no head end site, so its Head joint uses the
# same 87%-of-stature convention as the previous fallback.
_HEIGHT_CHAINS = {
    "fzmotion": {
        "torso": ("Hips", "Spine1", "Spine2", "Chest", "Neck", "Head", "HeadEnd"),
        "left_leg": ("LeftUpLeg", "LeftLeg", "LeftFoot"),
        "right_leg": ("RightUpLeg", "RightLeg", "RightFoot"),
        "head_fraction": 1.0,
    },
    "noitom": {
        "torso": ("Hips", "Spine", "Spine1", "Spine2", "Neck", "Neck1", "Head"),
        "left_leg": ("LeftUpLeg", "LeftLeg", "LeftFoot"),
        "right_leg": ("RightUpLeg", "RightLeg", "RightFoot"),
        "head_fraction": 0.87,
    },
}


def _estimate_human_height(frames, format):
    chains = _HEIGHT_CHAINS[format]
    sample_indices = np.linspace(0, len(frames) - 1, min(5, len(frames)), dtype=int)

    def chain_length(frame, names):
        return sum(
            np.linalg.norm(np.asarray(frame[child][0]) - np.asarray(frame[parent][0]))
            for parent, child in zip(names, names[1:])
        )

    torso = np.median([chain_length(frames[i], chains["torso"]) for i in sample_indices])
    left_leg = np.median([chain_length(frames[i], chains["left_leg"]) for i in sample_indices])
    right_leg = np.median([chain_length(frames[i], chains["right_leg"]) for i in sample_indices])
    return round(float((torso + (left_leg + right_leg) / 2) / chains["head_fraction"]), 3)


def load_bvh_file(bvh_file, format="lafan1"):
    """
    Must return a dictionary with the following structure:
    {
        "Hips": (position, orientation),
        "Spine": (position, orientation),
        ...
    }
    """
    if format not in _FOOT_ORIENTATION_SOURCE:
        raise ValueError(
            f"Invalid format: {format}. Supported: {sorted(_FOOT_ORIENTATION_SOURCE)}"
        )

    data = read_bvh(bvh_file)
    global_data = utils.quat_fk(data.quats, data.pos, data.parents)

    rotation_matrix = _AXIS_TRANSFORM[format]
    rotation_quat = R.from_matrix(rotation_matrix).as_quat(scalar_first=True)
    # Changing basis is a similarity transform, R' = T R T^-1: left-multiplying alone moves the
    # positions into the robot's basis while leaving every body's own axes in the source basis, so
    # each body frame ends up twisted against the robot's convention.
    conjugate = format in _SIMILARITY_TRANSFORM_FORMATS
    inverse_quat = R.from_matrix(rotation_matrix.T).as_quat(scalar_first=True)

    frames = []
    for frame in range(data.pos.shape[0]):
        result = {}
        for i, bone in enumerate(data.bones):
            orientation = utils.quat_mul(rotation_quat, global_data[0][frame, i])
            if conjugate:
                orientation = utils.quat_mul(orientation, inverse_quat)
            position = global_data[1][frame, i] @ rotation_matrix.T / 100  # cm to m
            result[bone] = [position, orientation]
            
        for side in ("Left", "Right"):
            orientation_source = _FOOT_ORIENTATION_SOURCE[format].format(side=side)
            result[f"{side}FootMod"] = [result[f"{side}Foot"][0], result[orientation_source][1]]

        frames.append(result)

    if format in _GROUND_NORMALISED_FORMATS:
        # Shift each clip onto the floor so root and foot targets use the same height origin.
        lowest = np.array([min(body[0][2] for body in frame.values()) for frame in frames])
        floor = float(np.percentile(lowest, 2.0))
        for frame in frames:
            for body in frame:
                frame[body][0] = frame[body][0] - np.array([0.0, 0.0, floor])

    if format in ("lafan1", "nokov"):
        human_height = 1.75
    else:
        human_height = _estimate_human_height(frames, format)

    return frames, human_height


