"""Export GMR qpos trajectories in BeyondMimic-compatible motion format."""

from __future__ import annotations

import pickle
from pathlib import Path

import mujoco as mj
import numpy as np

from .params import ROBOT_BASE_DICT, resolve_robot_xml

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


def _forward_kinematics_sequence(qpos_seq: np.ndarray, fps: float, xml_path: str | Path) -> dict:
    """Run FK over a qpos trajectory and return per-frame joint and body states.

    Body velocities come from ``mj_objectVelocity`` on ``mjOBJ_XBODY``, which is the body frame
    ``xpos``/``xquat`` are taken from. ``data.cvel`` (com-based, about the root subtree centre of
    mass) and ``mjOBJ_BODY`` (the inertial frame) both report a different reference point, so
    neither is the derivative of the positions exported alongside.
    """
    model = mj.MjModel.from_xml_path(str(xml_path))
    data = mj.MjData(model)

    T = qpos_seq.shape[0]
    joint_names, qpos_indices, qvel_indices = _collect_joint_metadata(model)
    body_names = get_body_names(model)
    nbody = len(body_names)

    joint_pos = np.stack([_extract_joint_pos(q, qpos_indices) for q in qpos_seq], axis=0)
    qvel_seq = _compute_qvel_sequence(model, qpos_seq, fps)
    joint_vel = qvel_seq[:, qvel_indices]

    body_pos = np.zeros((T, nbody, 3), dtype=np.float64)
    body_quat_wxyz = np.zeros((T, nbody, 4), dtype=np.float64)
    body_lin_vel = np.zeros((T, nbody, 3), dtype=np.float64)
    body_ang_vel = np.zeros((T, nbody, 3), dtype=np.float64)

    body_ids = list(range(1, model.nbody)) if model.nbody > 1 else list(range(model.nbody))
    velocity = np.zeros(6, dtype=np.float64)

    for t in range(T):
        data.qpos[:] = qpos_seq[t]
        data.qvel[:] = qvel_seq[t]
        mj.mj_forward(model, data)
        for bi, bid in enumerate(body_ids):
            body_pos[t, bi] = data.xpos[bid]
            body_quat_wxyz[t, bi] = data.xquat[bid]
            mj.mj_objectVelocity(model, data, mj.mjtObj.mjOBJ_XBODY, bid, velocity, 0)
            body_ang_vel[t, bi] = velocity[:3]
            body_lin_vel[t, bi] = velocity[3:]

    return {
        "joint_names": joint_names,
        "body_names": body_names,
        "joint_pos": joint_pos,
        "joint_vel": joint_vel,
        "body_pos": body_pos,
        "body_quat_wxyz": body_quat_wxyz,
        "body_lin_vel": body_lin_vel,
        "body_ang_vel": body_ang_vel,
    }


def _as_qpos_sequence(qpos_list: list[np.ndarray] | np.ndarray) -> np.ndarray:
    qpos_seq = np.asarray(qpos_list, dtype=np.float64)
    if qpos_seq.ndim == 1:
        qpos_seq = qpos_seq[None, :]
    return qpos_seq


def qpos_list_to_beyondmimic(
    qpos_list: list[np.ndarray] | np.ndarray,
    fps: float,
    xml_path: str | Path,
) -> dict:
    """Convert a qpos trajectory to BeyondMimic-style motion dict."""
    fk = _forward_kinematics_sequence(_as_qpos_sequence(qpos_list), fps, xml_path)

    return {
        "fps": np.array([float(fps)], dtype=np.float64),
        "joint_pos": fk["joint_pos"].astype(np.float32),
        "joint_vel": fk["joint_vel"].astype(np.float32),
        "body_pos_w": fk["body_pos"].astype(np.float32),
        "body_quat_w": fk["body_quat_wxyz"].astype(np.float32),
        "body_lin_vel_w": fk["body_lin_vel"].astype(np.float32),
        "body_ang_vel_w": fk["body_ang_vel"].astype(np.float32),
        "joint_names": np.array(fk["joint_names"], dtype=object),
        "body_names": np.array(fk["body_names"], dtype=object),
    }


def qpos_list_to_luna_npy(
    qpos_list: list[np.ndarray] | np.ndarray,
    fps: float,
    xml_path: str | Path,
) -> dict:
    """Convert a qpos trajectory to the ``.npy`` dict luna-beyondmimic consumes.

    Its ``scripts/npy_to_npz.py`` expects ``body_states`` as ``pos(3) + quat_xyzw(4) +
    lin_vel(3) + ang_vel(3)`` and ``dof_pos_vel`` as ``(T, J, 2)``, and reorders both axes by
    name, so only the name lists have to agree - joint names exactly, body names as a subset.
    """
    fk = _forward_kinematics_sequence(_as_qpos_sequence(qpos_list), fps, xml_path)

    quat_xyzw = fk["body_quat_wxyz"][..., [1, 2, 3, 0]]
    body_states = np.concatenate(
        [fk["body_pos"], quat_xyzw, fk["body_lin_vel"], fk["body_ang_vel"]], axis=-1
    )
    dof_pos_vel = np.stack([fk["joint_pos"], fk["joint_vel"]], axis=-1)

    return {
        "fps": int(round(float(fps))),
        "dof_names": list(fk["joint_names"]),
        "body_names": list(fk["body_names"]),
        "dof_pos_vel": dof_pos_vel.astype(np.float32),
        "body_states": body_states.astype(np.float32),
    }


def save_robot_motion(
    save_path: str | Path,
    qpos_list: list[np.ndarray] | np.ndarray,
    fps: float,
    robot_type: str | None = None,
    xml_path: str | Path | None = None,
) -> dict:
    """Save a retargeted motion, choosing the format by file extension.

    ``.npy`` writes the luna-beyondmimic schema; ``.npz`` and ``.pkl`` write the BeyondMimic
    key layout used by the rest of this repository.
    """
    if xml_path is None:
        if robot_type is None:
            raise ValueError("Either robot_type or xml_path must be provided")
        xml_path = resolve_robot_xml(robot_type)

    save_path = Path(save_path)
    save_path.parent.mkdir(parents=True, exist_ok=True)

    if save_path.suffix == ".npy":
        motion = qpos_list_to_luna_npy(qpos_list, fps, xml_path)
        np.save(save_path, motion, allow_pickle=True)
        return motion

    motion = qpos_list_to_beyondmimic(qpos_list, fps, xml_path)
    if save_path.suffix == ".npz":
        np.savez(save_path, **motion)
    else:
        with open(save_path, "wb") as f:
            pickle.dump(motion, f)
    return motion


def is_beyondmimic_motion(motion_data: dict) -> bool:
    return "joint_pos" in motion_data and "body_pos_w" in motion_data


def is_luna_npy_motion(motion_data: dict) -> bool:
    return "dof_pos_vel" in motion_data and "body_states" in motion_data


def describe_motion(motion_data: dict) -> str:
    """One-line summary of a saved motion, for either output schema."""
    if is_luna_npy_motion(motion_data):
        frames, joints, _ = np.asarray(motion_data["dof_pos_vel"]).shape
        bodies = np.asarray(motion_data["body_states"]).shape[1]
        fps = motion_data["fps"]
    else:
        frames, joints = np.asarray(motion_data["joint_pos"]).shape
        bodies = np.asarray(motion_data["body_pos_w"]).shape[1]
        fps = float(np.asarray(motion_data["fps"]).reshape(-1)[0])
    return f"T={frames}, joints={joints}, bodies={bodies}, fps={fps}"


def luna_npy_to_legacy_view(motion_data: dict, robot_type: str) -> tuple:
    """Convert a luna ``.npy`` dict to (fps, root_pos, root_rot_wxyz, dof_pos) for the viewer."""
    body_names = list(motion_data["body_names"])
    base_name = ROBOT_BASE_DICT[robot_type]
    if base_name not in body_names:
        raise KeyError(f"Base body '{base_name}' not in motion body_names")
    base_idx = body_names.index(base_name)

    body_states = np.asarray(motion_data["body_states"])
    root_pos = body_states[:, base_idx, 0:3]
    root_rot = body_states[:, base_idx, 3:7][:, [3, 0, 1, 2]]  # xyzw -> wxyz
    joint_pos = np.asarray(motion_data["dof_pos_vel"])[:, :, 0]
    return float(motion_data["fps"]), root_pos, root_rot, joint_pos


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
