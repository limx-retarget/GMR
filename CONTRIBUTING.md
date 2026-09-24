# Contributing to GMR (LimX fork)

Thanks for your interest in contributing! This repository is a **LimX Dynamics
fork** of [General Motion Retargeting (GMR)](https://github.com/YanjieZe/GMR).

## Scope of this fork

This fork adds LimX-specific robots and motion export:

- `limx_oli_edu` and `limx_luna` robot support and IK configs.
- Motion export aligned with
  [BeyondMimic](https://github.com/HybridRobotics/whole_body_tracking) / Isaac
  Lab reference-motion conventions.

Changes that are specific to upstream GMR behavior are best directed to the
[upstream repository](https://github.com/YanjieZe/GMR) so the whole community
benefits.

## Reporting issues

- **Bug or feature request for the LimX additions** → open an issue here.
- **General GMR usage questions** → consider the
  [upstream repository](https://github.com/YanjieZe/GMR) first.

Please include, where relevant: robot key (`limx_oli_edu` / `limx_luna`),
input format (SMPL-X / BVH / Xsens / OptiTrack), the exact command you ran, and
a short description of the expected vs. actual behavior.

## Pull requests

1. Fork the repository and create a branch from `limx`.
2. Make focused changes and keep commit messages clear.
3. If you add or modify third-party assets, update `THIRD_PARTY_NOTICES` and
   make sure the appropriate `LICENSE` file is present in the asset directory.
4. Open a pull request describing what and why.

By contributing, you agree that your contributions will be licensed under the
same [MIT License](LICENSE) as the rest of the repository.

## License

All contributions are licensed under the MIT license, unless otherwise stated
for vendored third-party assets (see `THIRD_PARTY_NOTICES`).