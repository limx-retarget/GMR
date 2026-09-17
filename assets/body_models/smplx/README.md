# SMPL-X body models (not bundled)

This directory is where GMR expects the three SMPL-X body model files. They are **not included** in this repository, because SMPL-X is distributed under a non-commercial license.

## Download

1. Register and agree to the terms at the [official SMPL-X website](https://smpl-x.is.tue.mpg.de/).
2. Download the SMPL-X Python models (the `.pkl` version).
3. Copy the following three files into this directory:

   - `SMPLX_NEUTRAL.pkl`
   - `SMPLX_FEMALE.pkl`
   - `SMPLX_MALE.pkl`

GMR loads them via `smplx.create(..., ext="pkl")` in `general_motion_retargeting/utils/smpl.py`.

## License

SMPL-X models are subject to the [SMPL-X Model License](https://smpl-x.is.tue.mpg.de/). Use only in compliance with that license (research / non-commercial terms apply by default). Do not redistribute the model files.