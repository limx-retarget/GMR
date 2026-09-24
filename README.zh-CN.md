<div align="right">

**语言**：[English](README.md) | [中文](README.zh-CN.md)

</div>

## 快速上手（LimX 机器人 & BeyondMimic 动作导出）

本节记录了本 LimX 版 [General Motion Retargeting (GMR)](https://github.com/YanjieZe/GMR) fork 中的**最新安装、使用与输出格式**：两款 LimX 机器人（`limx_oli_edu`、`limx_luna`）以及对齐 [BeyondMimic](https://github.com/HybridRobotics/whole_body_tracking) / Isaac Lab 参考动作约定的动作导出。

### 安装

已在 **Ubuntu 22.04 / 20.04** 上测试通过。

```bash
conda create -n gmr python=3.10 -y
conda activate gmr
pip install -e .
conda install -c conda-forge libstdcxx-ng -y
```

**SMPL-X 人体模型** —— **不随仓库分发**。由于这些模型受非商用的 [SMPL-X license](https://smpl-x.is.tue.mpg.de/) 约束，请自行从 [SMPL-X 官网](https://smpl-x.is.tue.mpg.de/) 下载，并将 `SMPLX_NEUTRAL.pkl`、`SMPLX_MALE.pkl`、`SMPLX_FEMALE.pkl` 放入 `assets/body_models/smplx/`。详见 `assets/body_models/smplx/README.md`。

加载是通过 `general_motion_retargeting/utils/smpl.py` 中的 `ext="pkl"` 完成的（无需改动 `site_packages`）。

**机器人资产** —— 两款 LimX 的 description 均未随仓库内置。OLI EDU 来自公开的 [`humanoid-description`](https://github.com/limxdynamics/humanoid-description) 子模块；Luna 来自公开的 [`luna-description`](https://github.com/limxdynamics/luna-description) 子模块。克隆后请拉取两个子模块：

```bash
git submodule update --init --recursive
```

### LimX OLI EDU（`limx_oli_edu`）

| 项目 | 值 |
| --- | --- |
| 机器人 key | `limx_oli_edu` |
| DoF | 31 个串联关节（12 腿 + 3 腰 + 2 头 + 14 臂） |
| Base body | `base_link` |
| MuJoCo XML | `assets/humanoid-description/HU_D04_description/xml/HU_D04_01_vis.xml`（公开子模块；纯串联） |
| IK 配置 | `{smplx,bvh_lafan1,bvh_nokov,bvh_xsens,bvh_fzmotion,bvh_noitom}_to_oli_edu.json` |

### LimX Luna / HU_L04（`limx_luna`）

| 项目 | 值 |
| --- | --- |
| 机器人 key | `limx_luna` |
| DoF | 27 个串联关节（12 腿 + 3 腰 + 2 头 + 10 臂） |
| Base body | `base_link` |
| MuJoCo XML | `assets/luna-description/HU_L04_description/xml/HU_L04_01_vis.xml`（子模块） |
| IK 配置 | `{smplx,bvh_lafan1,bvh_nokov,bvh_xsens,bvh_fzmotion,bvh_noitom}_to_luna.json` |
| 下游 | [luna-beyondmimic](https://github.com/limx-luna/luna-beyondmimic) 动作跟踪 |

每条手臂末端为 `wrist_yaw`（每条手臂 5 DoF，无 wrist pitch/roll），因此 IK 目标指向 `*_wrist_yaw_link`。

Luna 目标已归一化到配置的 1.8 m 参考骨架，且首帧的根节点 XY 位置被移到原点。根节点 Z 及后续 XY 轨迹保持不变。

**仅拉取 Luna description**（公开仓库）：

```bash
git submodule update --init assets/luna-description
```

已在别处有检出？直接指向它而无需添加子模块：

```bash
export LUNA_DESCRIPTION_DIR=/path/to/luna-description   # 包含 HU_L04_description/ 的目录
```

### LimX 机器人的输入格式

两款机器人都支持以下采集格式。每一行对应 `--robot limx_oli_edu` / `--robot limx_luna` 的该格式入口。

| 格式 | 入口 | 备注 |
| --- | --- | --- |
| SMPL-X（AMASS / OMOMO） | `scripts/smplx_to_robot.py` | GVHMR 视频输出的处理路径也是它 |
| BVH（LAFAN1） | `scripts/bvh_to_robot.py --format lafan1` | |
| BVH（Nokov） | `scripts/bvh_to_robot.py --format nokov` | |
| BVH（FZMotion） | `scripts/bvh_to_robot.py --format fzmotion` | 胸部节点是 `Chest`，不是 `Spine2` |
| BVH（Noitom） | `scripts/bvh_to_robot.py --format noitom` | 无脚趾关节，脚踝保持自身朝向 |

以下两种来源的加载方式与 agmr 一致，与 LAFAN1 和 Nokov 有两处差异，决定了 IK 目标落点：

- **向上轴转换的偏航角。** 两种约定都把 Y 抬升为 Z，但 LAFAN1/Nokov 使用 `[[1,0,0],[0,0,-1],[0,1,0]]`，而这两种使用 `[[0,0,1],[1,0,0],[0,1,0]]`，相差 90 度。只有后者把人的左右轴放在机器人的 Y 轴上；用前者时目标会侧放，机器人双臂前伸。
- **地面归一化。** 每段 clip 在加载时下移到地面，使根节点与脚使用同一高度原点。
| BVH（Xsens，3ds Max 导出） | `scripts/xsens_bvh_to_robot.py --bvh_format 3DSM` | `offsets.json` 可选；缺省时所有通道 offset 为零 |

OptiTrack FBX 仅支持 Unitree G1。

IK 配置使用 `scripts/_calibrate_ik_offsets.py` 标定，它通过重定向后的机器人复现人体肢体*方向*的接近程度来给配置打分（越小越好；上游 Unitree G1 配置在其自身测试数据上得分为 10–14 度）。

> [!IMPORTANT]
> **Xsens 骨骼约定随采集会话而异。** 两份 Xsens 导出可能把同一根骨骼放在其关节坐标系的不同轴上——这里的两个文件中上臂在一个里是 `-Z`、在另一个里是 `+Y`，相差 90 度，而腿部一致。基于其中一个标定的配置会让双手完全对不上另一个目标。若新采集的臂部看起来不对，请根据该文件重新推导 offset，约需一分钟：
>
> ```bash
> python scripts/_calibrate_ik_offsets.py calibrate --robot limx_luna --src bvh_xsens \
>   --motion_file <your.bvh> --config general_motion_retargeting/ik_configs/bvh_xsens_to_luna.json \
>   --tune align          # 由每根骨骼方向求解 rot_offset
> python scripts/_calibrate_ik_offsets.py calibrate --robot limx_luna --src bvh_xsens \
>   --motion_file <your.bvh> --config general_motion_retargeting/ik_configs/bvh_xsens_to_luna.json \
>   --tune roll           # 再求解每根骨骼的扭转，align 未覆盖的自由度
> ```
>
> 仓库内的 `bvh_xsens_to_{oli_edu,luna}.json` 基于 `data/Xsens/猫步-002.bvh` 标定。
> 另两个步骤是 `--tune chain`（由骨骼长度求肢体缩放）和 `--tune ground`（由脚接触求根缩放）。`scripts/_recalibrate_limx.sh` 保留固定的 SMPL-X 帧 offset，仅对 BVH 来源执行 `align` / `roll`。

### 使用

**SMPL-X → 机器人**（默认输出帧率 30）：

```bash
python scripts/smplx_to_robot.py \
  --smplx_file <path_to_smplx.npz_or.pkl> \
  --robot limx_oli_edu \
  --save_path output/lx_motion.npz
```

**BVH（LAFAN1 / Nokov / FZMotion / Noitom）→ 机器人**：

```bash
python scripts/bvh_to_robot.py \
  --bvh_file <path_to.bvh> \
  --robot limx_oli_edu \
  --format lafan1 \
  --save_path output/lx_motion.npz
```

**BVH（Xsens）→ 机器人** —— 需要 `--scale` 把文件单位换算成米：

```bash
python scripts/xsens_bvh_to_robot.py \
  --bvh_file assets/xsens_bvh_test/251021_04_boxing_120Hz_cm_3DsMax.bvh \
  --robot limx_luna --bvh_format 3DSM --scale 0.01 --reset_to_zero \
  --save_path output/luna_boxing.npy
```


**Luna → luna-beyondmimic `.npy`**（schema 见下；`--save_path` 的扩展名决定格式）：

```bash
python scripts/smplx_to_robot.py \
  --smplx_file <path_to_smplx.npz_or.pkl> \
  --robot limx_luna \
  --save_path output/luna_motion.npy
```

直接喂给训练器，它会运行自己的准备流水线：

```bash
# 在 luna-beyondmimic 检出目录中
python scripts/rsl_rl/train.py \
  --task=Tracking-Flat-HU-L04-Parallel-Deploy-Gravity-v0 \
  --motion_file /path/to/luna_motion.npy \
  --num_envs 4096 --headless
```

**可视化已保存的动作**（`.npy`、`.npz` 与旧版 `.pkl` 均可加载）：

```bash
python scripts/vis_robot_motion.py \
  --robot limx_luna \
  --robot_motion_path output/luna_motion.npy
```

加 `--record_video --video_path videos/demo.mp4` 可录制视频。想达到最大速度，可去掉重定向脚本上的 `--rate_limit`。

**导出 BeyondMimic `csv_to_npz` 流水线所需的 CSV**（根 xyz + 四元数 xyzw + 关节）：

```bash
python scripts/batch_gmr_pkl_to_csv.py --folder output/
# 输出到 output/csv/*.csv
```

### 机器人动作数据格式

`--save_path` 按文件扩展名选择 schema，两者都由 `general_motion_retargeting/motion_export.py` 写入：

| 扩展名 | Schema | 消费方 |
| --- | --- | --- |
| `.npy` | luna-beyondmimic | [luna-beyondmimic](https://github.com/limx-luna/luna-beyondmimic) 中的 `prepare_motion.py` / `train.py` / `play.py` |
| `.npz`, `.pkl` | BeyondMimic key 布局 | `scripts/vis_robot_motion.py`、`scripts/batch_gmr_pkl_to_csv.py` |

身体位置与朝向为 MuJoCo body 坐标系（`xpos` / `xquat`），身体速度取自同一坐标系原点（对 `mjOBJ_XBODY` 的 `mj_objectVelocity`），因此速度是导出位置的导数。

#### luna-beyondmimic `.npy`

一个 pickled dict，用 `np.load(path, allow_pickle=True).item()` 加载：

| Key | 形状 | 描述 |
| --- | --- | --- |
| `fps` | `int` | 帧率 |
| `dof_names` | 长度为 `N` 的 list | 关节名 |
| `body_names` | 长度为 `B` 的 list | 刚体名（不含 world body） |
| `dof_pos_vel` | `(T, N, 2)` | 关节位置 [rad] 与速度 [rad/s] |
| `body_states` | `(T, B, 13)` | `pos(3)` + `quat_xyzw(4)` + `lin_vel(3)` + `ang_vel(3)`，世界系 |

这里的四元数为 **xyzw** 顺序，与 `npy_to_npz.py` 的期望一致。对 `limx_luna`：`N = 27`、`B = 37`。消费方按名称重排两个轴，因此只需名称一致——关节名需完全一致，body 名是其关节体的子集（它追加的 14 个连杆/辅助 body 会被零填充）。

#### BeyondMimic `.npz` / `.pkl`

| Key | 形状 | 描述 |
| --- | --- | --- |
| `fps` | `(1,)` | 帧率，如 `[30.]` |
| `joint_pos` | `(T, N)` | 驱动关节角度 [rad]（**不含**浮动基座） |
| `joint_vel` | `(T, N)` | 关节速度 [rad/s] |
| `body_pos_w` | `(T, B, 3)` | 世界系身体位置 [m] |
| `body_quat_w` | `(T, B, 4)` | 世界系身体朝向，**wxyz** |
| `body_lin_vel_w` | `(T, B, 3)` | 身体线速度 [m/s] |
| `body_ang_vel_w` | `(T, B, 3)` | 身体角速度 [rad/s] |
| `joint_names` | `(N,)` | 关节名，与 `joint_pos` 列同序 |
| `body_names` | `(B,)` | 身体名（不含 world body），与身体张量同序 |

对 `limx_oli_edu`：`N = 31`、`B = 42`（重定向后的示例）。

**在 Python 中加载：**

```python
import numpy as np
motion = dict(np.load("output/lx_motion.npz", allow_pickle=True))
print(motion["joint_pos"].shape, motion["fps"], list(motion["joint_names"][:3]))
```

**关节列顺序** 遵循 MuJoCo 模型，即左腿 → 右腿 → 腰 → 头 → 左臂 → 右臂：`limx_oli_edu` 为 6/6/3/2/7/7（手臂止于 `wrist_roll`），`limx_luna` 为 6/6/3/2/5/5（手臂止于 `wrist_yaw`）。

**旧格式**（更早的运行）：`root_pos`、`root_rot`（xyzw）、`dof_pos` —— 仍可被 `load_robot_motion()` / 可视化辅助函数读取。

---

# GMR: General Motion Retargeting（通用运动重定向）

  <a href="https://arxiv.org/abs/2505.02833">
    <img src="https://img.shields.io/badge/paper-arXiv%3A2505.02833-b31b1b.svg" alt="arXiv Paper"/>
  </a> <a href="https://arxiv.org/abs/2510.02252">
    <img src="https://img.shields.io/badge/paper-arXiv%3A2510.02252-b31b1b.svg" alt="arXiv Paper"/>
  </a> <a href="https://opensource.org/licenses/MIT">
    <img src="https://img.shields.io/badge/License-MIT-yellow.svg" alt="License: MIT"/>
  </a> <a href="https://github.com/YanjieZe/GMR/releases">
    <img src="https://img.shields.io/badge/version-0.2.0-blue.svg" alt="Version"/>
  </a> <a href="https://x.com/ZeYanjie/status/1952446745696469334">
    <img src="https://img.shields.io/badge/twitter-ZeYanjie-blue.svg" alt="Twitter"/>
  </a> <a href="https://yanjieze.github.io/humanoid-foundation/#GMR">
    <img src="https://img.shields.io/badge/blog-GMR-blue.svg" alt="Blog"/>
  </a> <a href="https://www.bilibili.com/video/BV1p1nazeEzC/?share_source=copy_web&vd_source=c76e3ab14ac3f7219a9006b96b4b0f76">
    <img src="https://img.shields.io/badge/tutorial-BILIBILI-blue.svg" alt="Blog"/>
  </a>

![Banner for GMR](./assets/GMR.png)

![GMR](./assets/GMR_pipeline.png)

#### GMR 的核心特性：
- 实时高质量动作重定向，充分释放实时全身遥操作（即 [TWIST](https://github.com/YanjieZe/TWIST)）的潜力。
- 针对 RL 跟踪策略表现精心调优。
- 支持多款人形机器人与多种人体动作数据格式（见下表）。

> [!NOTE]
> 若希望本仓库支持新机器人或新的人体动作数据格式，请将机器人文件（`.xml`、`.urdf` 与网格）／人体动作数据发送给 <a href="mailto:lastyanjieze@gmail.com">Yanjie Ze</a> 或创建 issue，我们会尽快支持。并请确保你发送的机器人文件可以在本仓库中开源。

本仓库采用 [MIT License](LICENSE) 授权。

# 新闻与更新
- **2026-01-21：** GMR 现已支持 [Xsens](https://www.xsens.com/) BVH 离线数据。
- **2026-01-12：** GMR 现已支持 [Fourier GR3](https://www.fftai.com/)，这是仓库中第 17 款人形机器人。
- **2025-12-02：** GMR 现已支持 [TWIST2](https://yanjieze.com/TWIST2)，其使用 [XRoboToolkit SDK](https://github.com/XR-Robotics/XRoboToolkit-PC-Service)。
- **2025-11-17：** 想加入社区讨论，可添加我的微信 [二维码](https://yanjieze.com/TWIST2/images/my_wechat.jpg)，备注如 "[GMR] [你的名字] [你的单位]"。
- **2025-11-08：** Jason Peng 的 [MimicKit] 现已支持 GMR 格式。见 [这里](https://github.com/xbpeng/MimicKit/tree/main/tools/gmr_to_mimickit)。
- **2025-10-15：** 现已支持 [PAL Robotics 的 Talos](https://pal-robotics.com/robot/talos/)，第 15 款人形机器人。
- **2025-10-14：** GMR 现已支持 [Nokov](https://www.nokov.com/) BVH 数据。
- **2025-10-14：** 新增 ik 配置文档。见 [DOC.md](DOC.md)。
- **2025-10-09：** RL 动作跟踪的开源代码见 [TWIST](https://github.com/YanjieZe/TWIST)。
- **2025-10-02：** GMR 技术报告已上线 [arXiv](https://arxiv.org/abs/2510.02252)。
- **2025-10-01：** GMR 现已支持将 GMR pickle 文件转换为 CSV（用于 beyondmimic），见 `scripts/batch_gmr_pkl_to_csv.py`。
- **2025-09-25：** GMR 介绍视频见 [Bilibili](https://www.bilibili.com/video/BV1p1nazeEzC/?share_source=copy_web&vd_source=c76e3ab14ac3f7219a9006b96b4b0f76)。
- **2025-09-16：** GMR 现支持使用 [GVHMR](https://github.com/zju3dv/GVHMR) 从**单目视频**提取人体姿态并重定向到机器人。
- **2025-09-12：** GMR 现支持 [Tienkung](https://github.com/Open-X-Humanoid/TienKung-Lab)，第 14 款人形机器人。
- **2025-08-30：** GMR 现支持 [Unitree H1 2](https://www.unitree.com/cn/h1) 与 [PND Adam Lite](https://pndbotics.com/)，第 12、13 款人形机器人。
- **2025-08-28：** GMR 现支持 [Booster T1](https://www.boosterobotics.com/) 的 23dof 与 29dof 版本。
- **2025-08-27：** GMR 现支持 [Berkeley Humanoid Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite-Assets)，第 11 款人形机器人。
- **2025-08-24：** GMR 现支持 [Unitree H1](https://www.unitree.com/h1/)，第 10 款人形机器人。
- **2025-08-24：** GMR 现支持电机速度限制，`GeneralMotionRetargeting` 类默认 `use_velocity_limit=True`（默认速度限制为 3*pi）；并默认打印机器人 DoF/Body/Motor 名称及其 ID，可通过 `robot_dof_names`、`robot_body_names`、`robot_motor_names` 属性访问。
- **2025-08-10：** GMR 现支持 [Booster K1](https://www.boosterobotics.com/)，第 9 款机器人。
- **2025-08-09：** GMR 现支持 *带 Dex31 灵巧手的 Unitree G1*。
- **2025-08-07：** GMR 现支持 [Galexea R1 Pro](https://galaxea-dynamics.com/)（这是一款轮式人形机器人！）与 [KUAVO](https://www.kuavo.ai/)，第 7、8 款人形机器人。
- **2025-08-06：** GMR 现支持 [HighTorque Hi](https://www.hightorquerobotics.com/hi/)，第 6 款人形机器人。
- **2025-08-04：** GMR 首发版本。见我们的 [twitter 推文](https://x.com/ZeYanjie/status/1952446745696469334)。

## 演示

<table>
  <tr>
    <td align="center" width="20%">
      <b>Demo 1</b><br>
      将 LAFAN1 舞蹈动作重定向到 5 款机器人。<br>
      <video src="https://github.com/user-attachments/assets/23566fa5-6335-46b9-957b-4b26aed11b9e" width="200" controls></video>
    </td>
    <td align="center" width="20%">
      <b>Demo 2</b><br>
      Galexea R1 Pro 机器人（视角 1）。<br>
      <video src="https://github.com/user-attachments/assets/903ed0b0-0ac5-4226-8f82-5a88631e9b7c" width="200" controls></video>
    </td>
    <td align="center" width="20%">
      <b>Demo 3</b><br>
      Galexea R1 Pro 机器人（视角 2）。<br>
      <video src="https://github.com/user-attachments/assets/deea0e64-f1c6-41bc-8661-351682006d5d" width="200" controls></video>
    </td>
    <td align="center" width="20%">
      <b>Demo 4</b><br>
      只改一个参数切换机器人。<br>
      <video src="https://github.com/user-attachments/assets/03f10902-c541-40b1-8104-715a5759fd5e" width="200" controls></video>
    </td>
    <td align="center" width="20%">
      <b>Demo 5</b><br>
      HighTorque 机器人跳 twist 舞。<br>
      <video src="https://github.com/user-attachments/assets/1d3e663b-f29e-41b1-8e15-5c0deb6a4a5c" width="200" controls></video>
    </td>
  </tr>

  <tr>
    <td align="center">
      <b>Demo 6</b><br>
      Kuavo 机器人搬箱子。<br>
      <video src="https://github.com/user-attachments/assets/02fc8f41-c363-484b-a329-4f4e83ed5b80" width="200" controls></video>
    </td>
    <td align="center">
      <b>Demo 7</b><br>
      Unitree H1 机器人跳 ChaCha 舞。<br>
      <video src="https://github.com/user-attachments/assets/28ee6f0f-be30-42bb-8543-cf1152d97724" width="200" controls></video>
    </td>
    <td align="center">
      <b>Demo 8</b><br>
      Booster T1 机器人跳跃（视角 1）。<br>
      <video src="https://github.com/user-attachments/assets/2c75a146-e28f-4327-930f-5281bfc2ca9c" width="200" controls></video>
    </td>
    <td align="center">
      <b>Demo 9</b><br>
      Booster T1 机器人跳跃（视角 2）。<br>
      <video src="https://github.com/user-attachments/assets/ff10c7ef-4357-4789-9219-23c6db8dba6d" width="200" controls></video>
    </td>
    <td align="center">
      <b>Demo 10</b><br>
      Unitree H1-2 机器人跳跃。<br>
      <video src="https://github.com/user-attachments/assets/2382d8ce-7902-432f-ab45-348a11eeb312" width="200" controls></video>
    </td>
  </tr>

  <tr>
    <td align="center">
      <b>Demo 11</b><br>
      PND Adam Lite 机器人。<br>
      <video src="https://github.com/user-attachments/assets/a8ef1409-88f1-4393-9cd0-d2b14216d2a4" width="200" controls></video>
    </td>
    <td align="center">
      <b>Demo 12</b><br>
      Tienkung 机器人行走。<br>
      <video src="https://github.com/user-attachments/assets/7a775ecc-4254-450c-a3eb-49e843b8e331" width="200" controls></video>
    </td>
    <td align="center">
      <b>Demo 13</b><br>
      提取人体姿态（GVHMR + GMR）。<br>
      <a href="https://www.bilibili.com/video/BV1Tnpmz9EaE">▶ 在 Bilibili 观看</a>
    </td>
    <td align="center">
      <b>Demo 14</b><br>
      PAL Robotics 的 Talos 机器人对战。<br>
      <video src="https://github.com/user-attachments/assets/3ec0bf80-80c1-4181-a623-dc2b072c2ca2" width="200" controls></video>
    </td>
    <td align="center">
      <b>Demo 15</b><br>
      （以后新增时的可选占位）<br>
      <i>敬请期待…</i>
    </td>
  </tr>
</table>


## 支持的机器人与数据格式



| 分配 ID | 机器人/数据格式 | 机器人 DoF | SMPLX ([AMASS](https://amass.is.tue.mpg.de/), [OMOMO](https://github.com/lijiaman/omomo_release)) | BVH [LAFAN1](https://github.com/ubisoft/ubisoft-laforge-animation-dataset)| FBX ([OptiTrack](https://www.optitrack.com/)) | BVH [Nokov](https://www.nokov.com/) | PICO ([XRoboToolkit](https://github.com/XR-Robotics/XRoboToolkit-PC-Service)) | 更多格式即将到来 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | Unitree G1 `unitree_g1` | Leg (2\*6) + Waist (3) + Arm (2\*7) = 29 | ✅ | ✅ | ✅ | ✅ | ✅ |
| 1 | Unitree G1 with Hands `unitree_g1_with_hands` | Leg (2\*6) + Waist (3) + Arm (2\*7) + Hand (2\*7) = 43 | ✅ | ✅ | ✅ | TBD | TBD |
| 2 | Unitree H1 `unitree_h1` | Leg (2\*5) + Waist (1) + Arm (2\*4) = 19 | ✅ | TBD | TBD | TBD | TBD |
| 3 | Unitree H1 2 `unitree_h1_2` | Leg (2\*6) + Waist (1) + Arm (2\*7) = 27 | ✅ | TBD | TBD | TBD | TBD |
| 4 | Booster T1 `booster_t1` | TBD | ✅ | TBD | TBD | TBD |
| 5 | Booster T1 29dof `booster_t1_29dof` | TBD | ✅ | ✅ | TBD | TBD |
| 6 | Booster K1 `booster_k1` | Neck (2) + Arm (2\*4) + Leg (2\*6) = 22 | ✅ | TBD | TBD | TBD |
| 7 | Stanford ToddlerBot `stanford_toddy` | TBD | ✅ | ✅ | TBD | TBD |
| 8 | Fourier N1 `fourier_n1` | TBD | ✅ | ✅ | TBD | TBD |
| 9 | ENGINEAI PM01 `engineai_pm01` | TBD | ✅ | ✅ | TBD | TBD |
| 10 | HighTorque Hi `hightorque_hi` | Head (2) + Arm (2\*5) + Waist (1) + Leg (2\*6) = 25 | ✅ | TBD | TBD | TBD |
| 11 | Galaxea R1 Pro `galaxea_r1pro`（这是一款轮式机器人！） | Base (6) + Torso (4) + Arm (2\*7) = 24 | ✅ | TBD | TBD | TBD |
| 12 | Kuavo `kuavo_s45` | Head (2) + Arm (2\*7) + Leg (2\*6) = 28 | ✅ | TBD | TBD | TBD |
| 13 | Berkeley Humanoid Lite `berkeley_humanoid_lite`（需进一步调优） | Leg (2\*6) + Arm (2\*5) = 22 | ✅ | TBD | TBD | TBD |
| 14 | PND Adam Lite `pnd_adam_lite` | Leg (2\*6) + Waist (3) + Arm (2\*5) = 25 | ✅ | TBD | TBD | TBD |
| 15 | Tienkung `tienkung` | Leg (2\*6) + Arm (2\*4) = 20 | ✅ | TBD | TBD | TBD |
| 16 | PAL Robotics' Talos `pal_talos` | Head (2) + Arm (2\*7) + Waist (2) + Leg (2\*6) = 30 | ✅ | TBD | TBD | TBD |
| 17 | Fourier GR3 `fourier_gr3` | Head (2) + Arm (2\*7) + Waist (3) + Leg (2\*6) = 31 | ✅ | TBD | TBD | TBD |
| 更多机器人即将到来！ |
| 18 | AgiBot A2 `agibot_a2` | TBD | TBD | TBD | TBD | TBD |
| 19 | OpenLoong `openloong` | TBD | TBD | TBD | TBD | TBD |




## 安装

> [!NOTE]
> 代码已在 Ubuntu 22.04/20.04 上测试。

首先创建 conda 环境：

```bash
conda create -n gmr python=3.10 -y
conda activate gmr
```

然后安装 GMR：

```bash
pip install -e .
```

SMPL-X 人体模型（`.pkl`）**不随仓库分发**；请从 [SMPL-X 官网](https://smpl-x.is.tue.mpg.de/) 下载并放入 `assets/body_models/smplx/`。无需修改 `site_packages`（`utils/smpl.py` 已传入 `ext="pkl"`）。

为解决一些可能的渲染问题：

```bash
conda install -c conda-forge libstdcxx-ng -y
```

## 数据准备

[[SMPLX](https://github.com/vchoutas/smplx) 人体模型] **不随仓库分发** —— 从 [SMPL-X 官网](https://smpl-x.is.tue.mpg.de/)（需注册）下载，把 `.pkl` 文件放入 `assets/body_models/smplx/`。请遵守 [SMPL-X license](https://smpl-x.is.tue.mpg.de/)。

[[AMASS](https://amass.is.tue.mpg.de/) 动作数据] 从 [AMASS](https://amass.is.tue.mpg.de/) 将原始 SMPL-X 数据下载到任意目录。AMASS 需要**免费注册**，并在其自身的**非商用数据集许可**下分发（见 [AMASS 站点](https://amass.is.tue.mpg.de/)）——使用前请遵守。注意：不要下载 SMPL+H 数据。

[[OMOMO](https://github.com/lijiaman/omomo_release) 动作数据] 从[这个 Google Drive 文件](https://drive.google.com/file/d/1tZVqLB7II0whI-Qjz-z-AU3ponSEyAmm/view?usp=sharing)（由 OMOMO 项目托管，其仓库为 [MIT 许可](https://github.com/lijiaman/omomo_release)；该 drive 链接可能需要申请访问权限）下载原始 OMOMO 数据到任意目录。然后用 `scripts/convert_omomo_to_smplx.py` 把数据处理成 SMPL-X 格式。

[[LAFAN1](https://github.com/ubisoft/ubisoft-laforge-animation-dataset) 动作数据] 从[官方仓库](https://github.com/ubisoft/ubisoft-laforge-animation-dataset)（即 [lafan1.zip](https://github.com/ubisoft/ubisoft-laforge-animation-dataset/blob/master/lafan1/lafan1.zip)）下载原始 LAFAN1 bvh 文件。


## 人体/机器人运动数据表示

为了更好地使用本库，你可以先了解我们使用的人体动作数据与得到的机器人动作数据。

**人体动作数据**的每一帧表示为一个 (human_body_name, 3d 全局平移 + 全局旋转) 的字典。旋转通常用四元数表示（默认 wxyz 顺序，以对齐 mujoco）。

**机器人动作数据**的每一帧可理解为一个 (robot_base_translation, robot_base_rotation, robot_joint_positions) 三元组。

## 使用

### [新] PICO 实时流式传输到机器人（TWIST2）

安装 PICO SDK：
1. 在你的 PICO 上安装 PICO SDK：见[这里](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases/)。
2. 在你的 PC 上：
    - 下载 [ubuntu 22.04 的 deb 包](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb)，或从[仓库源码](https://github.com/XR-Robotics/XRoboToolkit-PC-Service)构建。
    - 安装时使用命令
        ```bash
        sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
        ```
        随后你会在 APPs 中看到 `xrobotoolkit-pc-service`。记得在进行遥操作前启动这个 app。
    - 构建 PICO PC Service SDK 与 PICO 流式传输的 Python SDK：
        ```bash
        conda activate gmr

        git clone https://github.com/YanjieZe/XRoboToolkit-PC-Service-Pybind.git
        cd XRoboToolkit-PC-Service-Pybind

        mkdir -p tmp
        cd tmp
        git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git
        cd XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK
        bash build.sh
        cd ../../../..


        mkdir -p lib
        mkdir -p include
        cp tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/PXREARobotSDK.h include/
        cp -r tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/nlohmann include/nlohmann/
        cp tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so lib/
        # rm -rf tmp

        # 构建项目
        conda install -c conda-forge pybind11
        pip uninstall -y xrobotoolkit_sdk
        python setup.py install
        ```

一切就绪！

想试试，参考 [TWIST2 的这个脚本](https://github.com/amazon-far/TWIST2/blob/master/teleop.sh)：
```bash
bash teleop.sh
```
你应该能在 mujoco 窗口中看到重定向后的机器人动作。

### 从 SMPL-X（AMASS, OMOMO）重定向到机器人

> [!NOTE]
> 注意：安装 SMPL-X 后，如果你使用 SMPL-X pkl 文件，请把 `smplx/body_models.py` 中的 `ext` 从 `npz` 改为 `pkl`。

重定向单段动作：

```bash
python scripts/smplx_to_robot.py --smplx_file <path_to_smplx_data> --robot <path_to_robot_data> --save_path <path_to_save_robot_data.pkl> --rate_limit
```

默认你应在 mujoco 窗口中看到重定向后机器人动作的可视化。
想录制视频，加 `--record_video` 与 `--video_path <your_video_path,mp4>`。

- `--rate_limit` 用于限制重定向后机器人动作的速率，使其与人体动作保持一致。想尽量快，去掉 `--rate_limit`。

重定向一个文件夹的动作：

```bash
python scripts/smplx_to_robot_dataset.py --src_folder <path_to_dir_of_smplx_data> --tgt_folder <path_to_dir_to_save_robot_data> --robot <robot_name>
```

批处理重定向默认无可视化。

### 从 GVHMR 重定向到机器人

首先，按[官方说明](https://github.com/zju3dv/GVHMR/blob/main/docs/INSTALL.md)安装 GVHMR。

并运行他们的 demo，从单目视频提取人体姿态：

```bash
cd path/to/GVHMR
python tools/demo/demo.py --video=docs/example_video/tennis.mp4 -s
```

随后你应在 `GVHMR/outputs/demo/tennis/hmr4d_results.pt` 得到已保存的人体姿态数据。

然后运行下面的命令，把提取的人体姿态数据重定向到机器人：

```bash
python scripts/gvhmr_to_robot.py --gvhmr_pred_file <path_to_hmr4d_results.pt> --robot unitree_g1 --record_video
```



## 从 BVH（LAFAN1, Nokov）重定向到机器人

重定向单段动作：

```bash
# 单段动作
python scripts/bvh_to_robot.py --bvh_file <path_to_bvh_data> --robot <path_to_robot_data> --save_path <path_to_save_robot_data.pkl> --rate_limit --format <format>
```

默认你应在 mujoco 窗口中看到重定向后机器人动作的可视化。
- `--rate_limit` 用于限制重定向后机器人动作的速率，使其与人体动作保持一致。想尽量快，去掉 `--rate_limit`。
- `--format` 用于指定 BVH 数据格式。支持的格式为 `lafan1` 和 `nokov`。


重定向一个文件夹的动作：

```bash
python scripts/bvh_to_robot_dataset.py --src_folder <path_to_dir_of_bvh_data> --tgt_folder <path_to_dir_to_save_robot_data> --robot <robot_name>
```

批处理重定向默认无可视化。



## 从 Xsens 重定向到机器人

### 离线：Xsens BVH 到机器人

#### 用 MuJoCo 可视化 Xsens BVH 数据

安装 PyQt6：
```bash
pip install PyQt6 PyQt6-Qt6 PyQt6-sip
```


```bash
python general_motion_retargeting/utils/xsens_vendor/mujoco_xsens_bvh_view.py \
  --bvh_file <path_to_dir_of_bvh_data> \
  --scale <displacement scaling size> \
  --reset_to_zero
```
例如
```bash
python general_motion_retargeting/utils/xsens_vendor/mujoco_xsens_bvh_view.py \
  --scale 0.01 \
  --bvh_file assets/xsens_bvh_test/251021_04_boxing_120Hz_cm_3DsMax.bvh \
  --reset_to_zero
```

- `--start` 用于指定初始处理帧。若不输入，则默认从第一帧开始处理。

- `--end` 用于指定最终处理帧。若不输入，默认处理到最后一帧。

- `--reset_to_zero` 用于把位移与 Z 轴旋转归零。该功能与 `--start` 组合使用时，能很好地让数据回到初始零位。因为有些数据集的前一两帧与后续数据差异过大，这些数据需要丢弃。

- `--scale` 用于设置位移的缩放值，取决于数据集中位移所用的单位与米的关系。

- ##### 使用前必须安装 PyQt6。`pip install PyQt6`
- ##### 执行该命令后会启动一个 UI 界面，可调整每个关节 x、y、z 三个方向各通道的角度值。调整完成后点击 `"Apply and Preview"` 按钮，会在本地生成 `offset.json` 文件并做 MuJoCo 可视化回放。运行 `xsens_bvh_to_robot.py` 时会从这个 JSON 文件读取数据。因此你需要先执行 `mujoco_xsens_bvh_view.py`，再使用 `xsens_bvh_to_robot.py` 做动作重定向，以确保本地存在 `offset.json` 文件。

#### 重定向单段动作：
```bash
# 单段动作
python scripts/xsens_bvh_to_robot.py \
  --bvh_file <path_to_bvh_data> \
  --robot <path_to_robot_data> \
  --save_path <path_to_save_robot_data.pkl> \
  --rate_limit \
  --start <number of the first frame> \
  --scale <displacement scaling size> \
  --reset_to_zero \
  --bvh_format <exported bvh format>
```
例如
```bash
python scripts/xsens_bvh_to_robot.py  \
  --robot unitree_h1_2 \
  --scale 0.01 \
  --reset_to_zero \
  --bvh_format 3DSM \
  --bvh_file assets/xsens_bvh_test/251021_04_boxing_120Hz_cm_3DsMax.bvh \
  --save_path retargeting_data/h1/251021_04_boxing_120Hz_cm_3DsMax.pkl
```
##### 默认你应在 mujoco 窗口中看到重定向后机器人动作的可视化。
- `--rate_limit` 用于限制重定向后机器人动作的速率，使其与人体动作保持一致。想尽量快，去掉 `--rate_limit`。

- `--start` 用于指定初始处理帧。若不输入，则默认从第一帧开始处理。

- `--end` 用于指定最终处理帧。若不输入，默认处理到最后一帧。

- `--reset_to_zero` 用于把位移与 Z 轴旋转归零。该功能与 `--start` 组合使用时，能很好地让数据回到初始零位。因为有些数据集的前一两帧与后续数据差异过大，这些数据需要丢弃。

- `--scale` 用于设置位移的缩放值，取决于数据集中位移所用的单位与米的关系。

##### ！！！！！！！！！！！！！！！！！！ 注意 ！！！！！！！！！！！！！！！！！！！！
- `--bvh_format` 用于设置所解析 bvh 的格式。在 Xsens MVN 软件中，可以导出三种格式的 BVH 文件。不同格式的 BVH 文件会有一些差异。这里推荐使用 3D Studio Max 格式。（实际上我尚未完成对其他格式数据的解析。）

- 导出的 pkl 文件会以 `wxyz` 格式表示四元数。^ _ ^

---

### 在线流式传输（Xsens MVN）

把来自 **Xsens MVN Software** 的实时动作数据直接流式送入 GMR，进行实时机器人重定向。

#### 1. 安装 Xsens MVN UDP 数据解析器

`xsens_mvn_robot_python` 库把 Xsens MVN 网络数据报（Position + Orientation，四元数格式）解析为 Python 可访问的数据结构。请安装与你的 Python 版本匹配的 `.whl` 文件。

```bash
# 克隆解析器仓库
git clone https://github.com/jiminghe/xsens_mvn_robot_python.git
cd xsens_mvn_robot_python

# 安装与你的 Python 版本匹配的 wheel
# 以 Python 3.10 为例：
pip install xsens_mvn_robot_python-*-cp310-*.whl
```

> 选择文件名中包含你 Python 版本标签（如 Python 3.10 对应 `cp310`，Python 3.8 对应 `cp38`）的 `.whl` 文件。该库会自动处理 UDP socket 绑定与数据报解包。

#### 2. 配置 Xsens MVN Network Streamer

在 Windows 或 Linux 上启动 **Xsens MVN Software**。你可以边穿着 Xsens Link / Awinda 套装进行实时录制边流式传输，也可以回放事先录制的 `.mvn` 文件。

| 步骤 | 操作 |
|---|---|
| 1 | 点击 **Options → Network Streamer** |
| 2 | 在弹出窗口中点击 **Add** 新建一个流目标 |
| 3 | 设置 **Host Address**（见下表） |
| 4 | 在 Network Streamer Options 下，仅勾选 **Position + Orientation (Quaternion)** |
| 5 | GMR 重定向不需要其他数据源 |
| 6 | 点击 **OK** —— 确认 streamer 显示绿色状态 |

**Host Address 参考：**

| 场景 | Host Address 设置 |
|---|---|
| MVN 与 GMR 在同一台 Linux 机器（MVN Linux） | `127.0.0.1`（localhost） |
| MVN 在 Windows → 流式到 Ubuntu（同一局域网） | Ubuntu 的 IP 地址，如 `192.168.1.10` |

> **重要：** 从 Windows PC 流式到 Ubuntu 计算机时，确保两台机器在同一局域网。为 MVN 应用关闭 Windows 防火墙，或在 MVN 默认端口（`9763`）上创建入站 UDP 规则。

#### 3. 运行 GMR 实时流式脚本

在 Xsens MVN Network Streamer 激活且 conda 环境已加载的情况下，运行实时流式重定向脚本。会打开一个 MuJoCo 窗口，展示 Unitree G1 机器人实时镜像你的动作。

```bash
# 激活 GMR 环境
conda activate gmr

# 运行 Xsens 实时流式重定向脚本
python scripts/xsens_live_streaming.py
```

### OptiTrack 在线流式传输

我们提供了使用 OptiTrack MoCap 数据进行实时流式传输与重定向的脚本。

通常你会有两台计算机，一台是安装了 Motive（OptiTrack 的桌面 App）的服务器，另一台是安装了 GMR 的客户端。

找到服务器 IP（安装了 Motive 的机器）与客户端 IP（你的机器）。如下设置流式传输：

![OptiTrack Streaming](./assets/optitrack.png)

然后运行：

```bash
python scripts/optitrack_to_robot.py --server_ip <server_ip> --client_ip <client_ip> --use_multicast False --robot unitree_g1
```

你应该能在 mujoco 窗口中看到重定向后机器人动作的可视化。

### 可视化已保存的机器人动作

可视化单个动作：

```bash
python scripts/vis_robot_motion.py --robot <robot_name> --robot_motion_path <path_to_save_robot_data.pkl>
```

想录制视频，加 `--record_video` 与 `--video_path <your_video_path,mp4>`。

可视化一个文件夹的动作：

```bash
python scripts/vis_robot_motion_dataset.py --robot <robot_name> --robot_motion_folder <path_to_save_robot_data_folder>
```

启动 MuJoCo 可视化窗口并点击后，可用以下键盘控制：
* `[`：播放下一个动作
* `]`：播放下一个动作
* `space`：切换播放/暂停

## 速度基准

| CPU | 重定向速度 |
| --- | --- |
| AMD Ryzen Threadripper 7960X 24-Cores | 60~70 FPS |
| 13th Gen Intel Core i9-13900K 24-Cores | 35~45 FPS |
| TBD | TBD |

## 引用

如果你觉得我们的代码有用，请考虑引用我们的相关论文：

```bibtex
@article{joao2025gmr,
  title={Retargeting Matters: General Motion Retargeting for Humanoid Motion Tracking},
  author= {Joao Pedro Araujo and Yanjie Ze and Pei Xu and Jiajun Wu and C. Karen Liu},
  year= {2025},
  journal= {arXiv preprint arXiv:2510.02252}
}
```

```bibtex
@article{ze2025twist,
  title={TWIST: Teleoperated Whole-Body Imitation System},
  author= {Yanjie Ze and Zixuan Chen and João Pedro Araújo and Zi-ang Cao and Xue Bin Peng and Jiajun Wu and C. Karen Liu},
  year= {2025},
  journal= {arXiv preprint arXiv:2505.02833}
}
```

以及这个 GitHub 仓库：

```bibtex
@software{ze2025gmr,
  title={GMR: General Motion Retargeting},
  author= {Yanjie Ze and João Pedro Araújo and Jiajun Wu and C. Karen Liu},
  year= {2025},
  url= {https://github.com/YanjieZe/GMR},
  note= {GitHub repository}
}
```

## 已知问题

为所有不同的人设计单一配置并不容易。我们观察到有些动作的重定向结果可能不佳。如果你发现结果不佳，请告诉我们！我们现已把这些动作收集在 [TEST_MOTIONS.md](TEST_MOTIONS.md) 中。

## 致谢

我们的 IK 求解器基于 [mink](https://github.com/kevinzakka/mink) 与 [mujoco](https://github.com/google-deepmind/mujoco)。我们的可视化基于 [mujoco](https://github.com/google-deepmind/mujoco)。我们尝试的人体动作数据包括 [AMASS](https://amass.is.tue.mpg.de/)、[OMOMO](https://github.com/lijiaman/omomo_release) 与 [LAFAN1](https://github.com/ubisoft/ubisoft-laforge-animation-dataset)。

原始机器人模型可在以下位置找到：

* [Berkley Humanoid Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite-Assets)：CC-BY-SA-4.0 license
* [Booster K1](https://www.boosterobotics.com/)
* [Booster T1](https://booster.feishu.cn/wiki/UvowwBes1iNvvUkoeeVc3p5wnUg)（[English](https://booster.feishu.cn/wiki/DtFgwVXYxiBT8BksUPjcOwG4n4f)）
* [EngineAI PM01](https://github.com/engineai-robotics/engineai_ros2_workspace)：[文件链接](https://github.com/engineai-robotics/engineai_ros2_workspace/blob/community/src/simulation/mujoco/assets/resource)
* [Fourier N1](https://github.com/FFTAI/Wiki-GRx-Gym)：[文件链接](https://github.com/FFTAI/Wiki-GRx-Gym/tree/FourierN1/legged_gym/resources/robots/N1)
* [Galaxea R1 Pro](https://galaxea-dynamics.com/)：MIT license
* [HighToqure Hi](https://www.hightorquerobotics.com/hi/)
* [LEJU Kuavo S45](https://gitee.com/leju-robot/kuavo-ros-opensource/blob/master/LICENSE)：MIT license
* [PAL Robotics' Talos](https://github.com/google-deepmind/mujoco_menagerie)：[文件链接](https://github.com/google-deepmind/mujoco_menagerie/tree/main/pal_talos)
* [Toddlerbot](https://github.com/hshi74/toddlerbot)：[文件链接](https://github.com/hshi74/toddlerbot/tree/main/toddlerbot/descriptions/toddlerbot_active)
* [Unitree G1](https://github.com/unitreerobotics/unitree_ros)：[文件链接](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/g1_description)