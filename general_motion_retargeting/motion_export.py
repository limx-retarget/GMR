"""Export GMR qpos trajectories in BeyondMimic-compatible motion format."""

from __future__ import annotations

import pickle
from pathlib import Path

import mujoco as mj
import numpy as np

from .params import ROBOT_BASE_DICT, ROBOT_XML_DICT

BEYONDMIMIC_KEYS = (
    "fps",
    "joint_pos",
    "joint_vel",
    "body_pos_w",
    "body_quat_w",
    "body_lin_vel_w",
    "body_ang_vel_w",
    "joint_names",
    "body_names",
)


def _collect_joint_metadata(model: mj.MjModel) -> tuple[list[str], list[int], list[int]]:
    """Return (joint_names, qpos_indices, qvel_indices) for actuated (non-free) joints."""
    entries: list[tuple[int, str, int, int]] = []
    for j in range(model.njnt):
        jtype = int(model.jnt_type[j])
        if jtype == mj.mjtJoint.mjJNT_FREE:
            continue
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, j)
        qadr = int(model.jnt_qposadr[j])
        vadr = int(model.jnt_dofadr[j])
        if jtype in (mj.mjtJoint.mjJNT_HINGE, mj.mjtJoint.mjJNT_SLIDE):
            qdim, vdim = 1, 1
        elif jtype == mj.mjtJoint.mjJNT_BALL:
            qdim, vdim = 4, 3
        else:
            continue
        entries.append((qadr, name, qdim, vadr))
    entries.sort(key=lambda x: x[0])
    joint_names = [e[1] for e in entries]
    qpos_indices: list[int] = []
    qvel_indices: list[int] = []
    for qadr, _name, qdim, vadr in entries:
        qpos_indices.extend(range(qadr, qadr + qdim))
        vdim = 3 if qdim == 4 else 1
        qvel_indices.extend(range(vadr, vadr + vdim))
    return joint_names, qpos_indices, qvel_indices


def get_body_names(model: mj.MjModel, skip_world: bool = True) -> list[str]:
    names = []
    start = 1 if skip_world else 0
    for i in range(start, model.nbody):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, i)
        names.append(name if name is not None else f"body_{i}")
    return names


def get_joint_names(model: mj.MjModel) -> list[str]:
    names, _, _ = _collect_joint_metadata(model)
    return names


def _extract_joint_pos(qpos: np.ndarray, qpos_indices: list[int]) -> np.ndarray:
    return np.asarray(qpos, dtype=np.float64)[qpos_indices]


def _compute_qvel_sequence(model: mj.MjModel, qpos_seq: np.ndarray, fps: float) -> np.ndarray:
    """Per-frame generalized velocities via MuJoCo differentiatePos."""
    T = qpos_seq.shape[0]
    qvel = np.zeros((T, model.nv), dtype=np.float64)
    dt = 1.0 / fps
    scratch = np.zeros(model.nv, dtype=np.float64)
    for i in range(T):
        if T == 1:
            qvel[i] = 0.0
            continue
        if i == 0:
            mj.mj_differentiatePos(model, scratch, dt, qpos_seq[i], qpos_seq[i + 1])
        elif i == T - 1:
            mj.mj_differentiatePos(model, scratch, dt, qpos_seq[i - 1], qpos_seq[i])
        else:
            fwd = np.zeros(model.nv, dtype=np.float64)
            bwd = np.zeros(model.nv, dtype=np.float64)
            mj.mj_differentiatePos(model, fwd, dt, qpos_seq[i], qpos_seq[i + 1])
            mj.mj_differentiatePos(model, bwd, dt, qpos_seq[i - 1], qpos_seq[i])
            scratch = 0.5 * (fwd + bwd)
        qvel[i] = scratch
    return qvel


def qpos_list_to_beyondmimic(
    qpos_list: list[np.ndarray] | np.ndarray,
    fps: float,
    xml_path: str | Path,
) -> dict:
    """Convert a qpos trajectory to BeyondMimic-style motion dict."""
    model = mj.MjModel.from_xml_path(str(xml_path))
    data = mj.MjData(model)

    qpos_seq = np.asarray(qpos_list, dtype=np.float64)
    if qpos_seq.ndim == 1:
        qpos_seq = qpos_seq[None, :]
    T = qpos_seq.shape[0]

    joint_names, qpos_indices, qvel_indices = _collect_joint_metadata(model)
    body_names = get_body_names(model)
    nbody = len(body_names)

    joint_pos = np.stack([_extract_joint_pos(q, qpos_indices) for q in qpos_seq], axis=0)
    qvel_seq = _compute_qvel_sequence(model, qpos_seq, fps)
    joint_vel = qvel_seq[:, qvel_indices]

    body_pos_w = np.zeros((T, nbody, 3), dtype=np.float64)
    body_quat_w = np.zeros((T, nbody, 4), dtype=np.float64)
    body_lin_vel_w = np.zeros((T, nbody, 3), dtype=np.float64)
    body_ang_vel_w = np.zeros((T, nbody, 3), dtype=np.float64)

    body_ids = list(range(1, model.nbody)) if model.nbody > 1 else list(range(model.nbody))

    for t in range(T):
        data.qpos[:] = qpos_seq[t]
        data.qvel[:] = qvel_seq[t]
        mj.mj_forward(model, data)
        for bi, bid in enumerate(body_ids):
            body_pos_w[t, bi] = data.xpos[bid]
            body_quat_w[t, bi] = data.xquat[bid]
            body_ang_vel_w[t, bi] = data.cvel[bid, :3]
            body_lin_vel_w[t, bi] = data.cvel[bid, 3:6]

    return {
        "fps": np.array([float(fps)], dtype=np.float64),
        "joint_pos": joint_pos.astype(np.float32),
        "joint_vel": joint_vel.astype(np.float32),
        "body_pos_w": body_pos_w.astype(np.float32),
        "body_quat_w": body_quat_w.astype(np.float32),
        "body_lin_vel_w": body_lin_vel_w.astype(np.float32),
        "body_ang_vel_w": body_ang_vel_w.astype(np.float32),
        "joint_names": np.array(joint_names, dtype=object),
        "body_names": np.array(body_names, dtype=object),
    }


def save_robot_motion(
    save_path: str | Path,
    qpos_list: list[np.ndarray] | np.ndarray,
    fps: float,
    robot_type: str | None = None,
    xml_path: str | Path | None = None,
) -> dict:
    """Save motion in BeyondMimic format (.npz or .pkl by extension)."""
    if xml_path is None:
        if robot_type is None:
            raise ValueError("Either robot_type or xml_path must be provided")
        xml_path = ROBOT_XML_DICT[robot_type]
    motion = qpos_list_to_beyondmimic(qpos_list, fps, xml_path)

    save_path = Path(save_path)
    save_path.parent.mkdir(parents=True, exist_ok=True)
    if save_path.suffix == ".npz":
        np.savez(save_path, **motion)
    else:
        with open(save_path, "wb") as f:
            pickle.dump(motion, f)
    return motion


def is_beyondmimic_motion(motion_data: dict) -> bool:
    return "joint_pos" in motion_data and "body_pos_w" in motion_data


def beyondmimic_to_legacy_view(motion_data: dict, robot_type: str) -> tuple:
    """Convert BeyondMimic dict to (fps, root_pos, root_rot_wxyz, dof_pos) for the viewer."""
    fps = float(np.asarray(motion_data["fps"]).reshape(-1)[0])
    joint_pos = np.asarray(motion_data["joint_pos"])
    body_names = list(motion_data["body_names"])
    base_name = ROBOT_BASE_DICT[robot_type]
    if base_name not in body_names:
        raise KeyError(f"Base body '{base_name}' not in motion body_names")
    base_idx = body_names.index(base_name)
    root_pos = np.asarray(motion_data["body_pos_w"])[:, base_idx, :]
    root_rot = np.asarray(motion_data["body_quat_w"])[:, base_idx, :]
    return fps, root_pos, root_rot, joint_pos
