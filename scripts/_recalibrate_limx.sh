#!/usr/bin/env bash
# Re-derive every LimX IK config from geometry: chain scales, bone-aligned rotation offsets,
# left/right symmetry, then the root scale that puts the robot on the ground.
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

symmetrize() {
  $PY - "$1" <<'PYCODE'
import json, sys
path = sys.argv[1]
config = json.load(open(path))
table = config["human_scale_table"]
for name in list(table):
    mirror = "Right" + name[4:] if name.startswith("Left") else name.replace("left_", "right_", 1)
    if name.startswith(("Left", "left_")) and mirror in table and table[name] != table[mirror]:
        table[name] = table[mirror] = round((table[name] + table[mirror]) / 2, 3)
with open(path, "w") as f:
    json.dump(config, f, indent=4)
    f.write("\n")
PYCODE
}

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
    echo "### $robot / $src"
    $PY scripts/_calibrate_ik_offsets.py calibrate --robot "$robot" --src "$src" \
        --motion_file "$motion" --config "$config" --num_frames 20 --tune chain >/dev/null
    if [[ "$src" != "smplx" ]]; then
      $PY scripts/_calibrate_ik_offsets.py calibrate --robot "$robot" --src "$src" \
          --motion_file "$motion" --config "$config" --num_frames 20 --tune align >/dev/null
      symmetrize "$config"
      $PY scripts/_calibrate_ik_offsets.py calibrate --robot "$robot" --src "$src" \
          --motion_file "$motion" --config "$config" --num_frames 12 --tune roll | tail -1
    fi
    $PY scripts/_calibrate_ik_offsets.py calibrate --robot "$robot" --src "$src" \
        --motion_file "$motion" --config "$config" --num_frames 20 --tune ground | grep 'root scale'
    $PY scripts/_calibrate_ik_offsets.py score --robot "$robot" --src "$src" \
        --motion_file "$motion" --num_frames 40 2>/dev/null | tail -1
  done
done
