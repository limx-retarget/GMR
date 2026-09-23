# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Installation and Setup

This is a Python package for motion retargeting to humanoid robots. Install in development mode:

```bash
conda create -n gmr python=3.10 -y
conda activate gmr
pip install -e .
conda install -c conda-forge libstdcxx-ng -y
```

## Code Architecture

### Core Components

- **`GeneralMotionRetargeting`** (`general_motion_retargeting/motion_retarget.py`): Main class for motion retargeting using inverse kinematics (IK) solver built on mink/mujoco
- **`KinematicsModel`** (`general_motion_retargeting/kinematics_model.py`): Handles robot kinematics calculations
- **`RobotMotionViewer`** (`general_motion_retargeting/robot_motion_viewer.py`): MuJoCo-based visualization for robot motions
- **Configuration System** (`general_motion_retargeting/params.py`): Simplified robot definitions and IK config mappings - cleaned to focus on core supported robots

### Data Flow

1. **Human Motion Input**: SMPL-X (AMASS/OMOMO) or BVH (LAFAN1) format
2. **Motion Format**: Each frame = dict of (human_body_name, 3D translation + rotation)
3. **Robot Output**: Tuple of (base_translation, base_rotation, joint_positions)
4. **IK Configs**: JSON files in `general_motion_retargeting/ik_configs/` define human-to-robot body mappings

### Supported Robots

Core robot models in `assets/` directory:
- Unitree G1 (`unitree_g1`) - 29 DOF humanoid
- Booster T1 (`booster_t1`) - Full-body humanoid 
- Booster K1 (`booster_k1`) - 22 DOF humanoid
- Stanford ToddlerBot (`stanford_toddy`) - Research humanoid
- Fourier N1 (`fourier_n1`) - Commercial humanoid
- ENGINEAI PM01 (`engineai_pm01`) - Industrial humanoid
- Kuavo S45 (`kuavo_s45`) - 28 DOF humanoid
- HighTorque Hi (`hightorque_hi`) - 25 DOF humanoid
- Galaxea R1 Pro (`galaxea_r1pro`) - 24 DOF wheeled humanoid
- LimX OLI EDU (`limx_oli_edu`) - 31 series DOF (12 leg + 3 waist + 2 head + 14 arm). The model comes from the public `humanoid-description` submodule at `assets/humanoid-description/`, or a checkout pointed to by `HUMANOID_DESCRIPTION_DIR`. Its `HU_D04_01_vis.xml` is pure-serial, so no `frozen_joints` are needed.
- LimX Luna / HU_L04 (`limx_luna`) - 27 series DOF (12 leg + 3 waist + 2 head + 10 arm; each arm ends at `wrist_yaw`). The description is **not vendored**: it comes from the private `luna-description` submodule at `assets/luna-description/`, or a checkout pointed to by `LUNA_DESCRIPTION_DIR`. Retargeted motions feed [luna-beyondmimic](https://github.com/limx-luna/luna-beyondmimic) via the `.npy` export.

Additional models retained in ROBOT_BASE_DICT for compatibility:
- `unitree_g1_with_hands` (43 DOF with dexterous hands)
- `dex31_left_hand`, `dex31_right_hand` (hand components)

## Common Commands

### Single Motion Retargeting
```bash
# SMPL-X to robot
python scripts/smplx_to_robot.py --smplx_file <path> --robot <robot_name> --save_path <output.pkl>

# BVH to robot  
python scripts/bvh_to_robot.py --bvh_file <path> --robot <robot_name> --save_path <output.pkl>
```

`--save_path` selects the output schema by extension: `.npy` writes the luna-beyondmimic dict
(`body_states`, `dof_pos_vel`, `dof_names`, `body_names`, `fps`; quaternions xyzw), while `.npz`
and `.pkl` write the BeyondMimic key layout (`joint_pos`, `body_pos_w`, `body_quat_w` wxyz, ...).

### Batch Processing
```bash
# Process datasets
python scripts/smplx_to_robot_dataset.py
python scripts/bvh_to_robot_dataset.py
```

### Visualization
```bash
# Visualize saved robot motion
python scripts/vis_robot_motion.py --robot <robot_name> --robot_motion_path <path.pkl>
```

Add `--record_video --video_path <output.mp4>` to any visualization command to record video.

## Key Technical Details

- **IK Solver**: Uses mink library with configurable solver (default: "daqp") and damping (default: 5e-1)
- **Human Height Scaling**: Automatic scaling based on `actual_human_height` parameter vs config assumptions
- **Real-time Performance**: Optimized for 60-70 FPS on high-end CPUs for teleoperation use cases
- **Body Model Dependencies**: Requires SMPL-X body models in `assets/body_models/smplx/`

## File Organization

- `scripts/`: Entry point scripts for different retargeting workflows
- `general_motion_retargeting/`: Core library code
- `assets/`: Robot models (MuJoCo XML) and body models (SMPL-X)
- `general_motion_retargeting/ik_configs/`: JSON configuration files for human-to-robot body mappings:
  - SMPL-X configs: `smplx_to_{g1,t1,k1,toddy,n1,pm01,kuavo,hi,r1pro,oli_edu,luna}.json`
  - BVH configs: `bvh_{lafan1,nokov,xsens,fzmotion,noitom}_to_{...}.json`
  - Real-time OptiTrack uses `fbx_to_g1.json`. Offline FBX/PoseLib support was removed because
    the vendored NVIDIA ASE PoseLib is restricted to non-commercial use.
  - Both LimX robots support SMPL-X and BVH (LAFAN1 / Nokov / Xsens / FZMotion / Noitom).
  - **FZMotion / Noitom follow agmr's loading conventions**, which differ from LAFAN1/Nokov:
    the Y-up-to-Z-up matrix is `[[0,0,1],[1,0,0],[0,1,0]]` (90 degrees of yaw away from the legacy
    one, and the only one that puts the subject's lateral axis on the robot's Y), and each clip is
    shifted so its lowest point rests on z=0. Both are in `utils/lafan1.py`.
  - SMPL-X uses fixed frame offsets inherited from `6d74f61`; do not run `--tune align` or
    `--tune roll` on SMPL-X because independent body alignment makes the IK rotate the free base.
    For BVH, the root body must be part of a calibration chain, otherwise its `rot_offset` keeps
    the template's value and the whole robot ends up yawed against its targets.
    `scripts/_make_limx_configs.py` derives a new LimX config from the G1 config of that format,
    and `scripts/_calibrate_ik_offsets.py` scores and tunes it against real capture data
    (`--tune chain` limb scales, `--tune align` rot_offsets from bone directions, `--tune roll`
    the twist alignment leaves free, `--tune ground` the root scale). `scripts/_recalibrate_limx.sh`
    runs the whole sequence for every LimX config.
  - **Xsens bone conventions vary between capture sessions** (the upper arm sits on a different
    joint axis in the two files in this repo, 90 degrees apart, while the legs agree), so an Xsens
    config calibrated on one session can put the hands far from their targets on another. Re-run
    `--tune align` plus `--tune roll` on the new file when that happens.

## Project Status & Features

**Current State**: Production-ready motion retargeting system with extensive robot support

**Key Capabilities**:
- **Multi-format Input**: SMPL-X (AMASS/OMOMO), BVH variants, and real-time OptiTrack streaming
- **Real-time Performance**: 60-70 FPS on high-end hardware for teleoperation
- **9 Robot Models**: From research platforms to commercial humanoids
- **Robust IK**: Mink-based solver with automatic human height scaling
- **Visualization**: MuJoCo-based viewer with video recording capabilities
- **Batch Processing**: Dataset-level retargeting workflows

**Use Cases**:
- Real-time whole-body teleoperation (see [TWIST](https://github.com/YanjieZe/TWIST))
- RL policy training data generation
- Motion capture to robot deployment
- Cross-platform humanoid motion transfer

**Recent Additions** (2025):
- Booster K1 support (9th robot)
- Dexterous hand integration (G1 + Dex31)
- Wheeled humanoid support (Galaxea R1 Pro)
- Enhanced OptiTrack real-time streaming
- LimX OLI EDU support, plus a generic `frozen_joints` hook in `motion_retarget.py` for robots that need parallel-mechanism filtering without an XML rewrite (unused by registered robots)
- LimX Luna / HU_L04 support from the private `luna-description` submodule, with a `.npy` exporter matching the luna-beyondmimic motion schema