# poselib (vendored)

This directory vendors `poselib`, a pose / motion utility library (quaternion
math, 3D skeleton retargeting, FBX import, and visualization) copied from the
`poselib` submodule of NVIDIA's **ASE** project (Adversarial Skill Embeddings).

## Origin

- Upstream repository: <https://github.com/nv-tlabs/ASE>
- Vendored from:
  - `ase/poselib/poselib/`          → `third_party/poselib/`
  - `ase/poselib/fbx_importer.py`   → `third_party/poselib/fbx_importer.py`
- Copyright: NVIDIA Corporation & Affiliates
- License: NVIDIA License (full text in [`LICENSE`](./LICENSE))

## Non-commercial use restriction

The vendored code is distributed under NVIDIA's license, whose **Section 3.3
("Use Limitation")** permits use only for **research or evaluation purposes**
and prohibits commercial use. This license is more restrictive than the
repository's overall MIT license and governs the files in this directory
regardless of the repository-level MIT license.