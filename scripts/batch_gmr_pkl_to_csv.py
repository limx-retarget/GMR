import argparse
import os
import pickle

import numpy as np

from general_motion_retargeting.motion_export import is_beyondmimic_motion


def motion_to_csv_arrays(motion_data):
    """Build Unitree/BeyondMimic CSV rows: root_pos, root_quat_xyzw, joint_pos."""
    if is_beyondmimic_motion(motion_data):
        body_names = list(motion_data["body_names"])
        base_idx = body_names.index("pelvis") if "pelvis" in body_names else body_names.index("base_link")
        root_pos = np.asarray(motion_data["body_pos_w"])[:, base_idx, :]
        root_quat_wxyz = np.asarray(motion_data["body_quat_w"])[:, base_idx, :]
        root_rot = root_quat_wxyz[:, [1, 2, 3, 0]]
        dof_pos = np.asarray(motion_data["joint_pos"])
        frame_rate = float(np.asarray(motion_data["fps"]).reshape(-1)[0])
    else:
        dof_pos = motion_data["dof_pos"]
        root_pos = motion_data["root_pos"]
        root_rot = motion_data["root_rot"]
        frame_rate = motion_data["fps"]

    motion = np.zeros((dof_pos.shape[0], dof_pos.shape[1] + 7), dtype=np.float32)
    motion[:, :3] = root_pos
    motion[:, 3:7] = root_rot
    motion[:, 7:] = dof_pos
    return motion, frame_rate


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert GMR motion files to CSV (for BeyondMimic)")
    parser.add_argument(
        "--folder", type=str, help="Path to the folder containing motion files from GMR",
    )
    args = parser.parse_args()

    out_folder = os.path.join(args.folder, "csv")
    os.makedirs(out_folder, exist_ok=True)

    for i, file in enumerate(os.listdir(args.folder)):
        path = os.path.join(args.folder, file)
        if file.endswith(".pkl"):
            with open(path, "rb") as f:
                motion_data = pickle.load(f)
        elif file.endswith(".npz"):
            motion_data = dict(np.load(path, allow_pickle=True))
        else:
            continue

        motion, frame_rate = motion_to_csv_arrays(motion_data)

        if frame_rate > 30:
            downsample_factor = frame_rate / 30.0
            indices = np.arange(0, motion.shape[0], downsample_factor).astype(int)
            old_length = motion.shape[0]
            motion = motion[indices]
            print(f"Downsampled from {old_length} to {motion.shape[0]} frames")

        out_name = file.replace(".pkl", ".csv").replace(".npz", ".csv")
        np.savetxt(os.path.join(out_folder, out_name), motion, delimiter=",")
        print(f"({i}/{len(os.listdir(args.folder))}) Saved to {os.path.join(out_folder, out_name)}")
