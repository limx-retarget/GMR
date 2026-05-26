# SMPL-X body models (vendored)

This directory ships the three SMPL-X body model files required by GMR:

- `SMPLX_NEUTRAL.pkl`
- `SMPLX_FEMALE.pkl`
- `SMPLX_MALE.pkl`

GMR loads them via `smplx.create(..., ext="pkl")` in `general_motion_retargeting/utils/smpl.py`. **No separate download is needed** after cloning this repository (ensure Git LFS is installed — see root `README.md`).

## License

SMPL-X models are subject to the [SMPL-X Model License](https://smpl-x.is.tue.mpg.de/). Use only in compliance with that license (research / non-commercial terms apply by default). Do not redistribute outside your organization if your license forbids it.
