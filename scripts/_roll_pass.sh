#!/usr/bin/env bash
# Run one more roll pass over every LimX config. The roll search is greedy per body, so a second
# sweep can only improve on where the first one stopped.
set -euo pipefail

cd "$(dirname "$0")/.."
PY=${PY:-python}
IK=general_motion_retargeting/ik_configs

SMPLX=${SMPLX:-data/Amass/0df2f220d329_BAT20260106017_0005_Walking001_stageii.npz}
LAFAN=${LAFAN:-/home/shmin/projects/agmr2/agmr/test_samples/lafan/walk.bvh}
NOKOV=${NOKOV:-/home/shmin/projects/agmr2/agmr/test_samples/nokov/walk.bvh}
XSENS=${XSENS:-data/Xsens/猫步-002.bvh}
FZMOTION=${FZMOTION:-/home/shmin/projects/agmr2/agmr/test_samples/fzmotion/walk.bvh}
NOITOM=${NOITOM:-/home/shmin/projects/agmr2/agmr/test_samples/noitom_inertial/walk.bvh}

for robot in limx_oli_edu limx_luna; do
  short=$([ "$robot" = limx_oli_edu ] && echo oli_edu || echo luna)
  for spec in "smplx smplx_to_${short}.json ${SMPLX}" \
              "bvh_lafan1 bvh_lafan1_to_${short}.json ${LAFAN}" \
              "bvh_nokov bvh_nokov_to_${short}.json ${NOKOV}" \
              "bvh_xsens bvh_xsens_to_${short}.json ${XSENS}" \
              "bvh_fzmotion bvh_fzmotion_to_${short}.json ${FZMOTION}" \
              "bvh_noitom bvh_noitom_to_${short}.json ${NOITOM}"; do
    set -- $spec
    src=$1; config=$IK/$2; motion=$3
    [ -f "$config" ] || continue
    [[ "$src" = "smplx" ]] && continue
    printf '%-14s %-13s ' "$robot" "$src"
    $PY scripts/_calibrate_ik_offsets.py calibrate --robot "$robot" --src "$src" \
        --motion_file "$motion" --config "$config" --num_frames 15 --tune roll 2>/dev/null \
        | grep -E 'starting|final' | tr '\n' ' '
    echo
  done
done
