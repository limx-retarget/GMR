"""Regression test for the generic frozen_joints framework capability.

After moving LimX OLI EDU to a pure-serial XML, no registered robot uses
`frozen_joints` anymore. This test keeps the capability covered so future
regressions are caught:

1. Existing robots without `frozen_joints` (regression): GMR initializes
   with `frozen_qpos_addr == []` and a single ConfigurationLimit; output
   qpos remains identical to the pre-change behaviour.

2. Synthetic config with `frozen_joints` on `fourier_n1` (capability):
   load fourier_n1, monkey-patch the SMPL-X IK config in memory to declare
   two arbitrary hinge joints as frozen, run one retarget(), and verify
   those qpos slots are exactly the initial value.

If anyone re-introduces a robot that needs `frozen_joints`, the framework
hook (and this test) keeps working.
"""

import json
import os
import sys

import numpy as np

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting.params import IK_CONFIG_DICT, IK_CONFIG_ROOT


def fabricate_human_data(ik_cfg):
    bodies = set()
    for table_key in ("ik_match_table1", "ik_match_table2"):
        for _, entry in ik_cfg.get(table_key, {}).items():
            bodies.add(entry[0])
    bodies.update(ik_cfg.get("human_scale_table", {}).keys())
    bodies.add(ik_cfg["human_root_name"])
    human_data = {}
    for i, body in enumerate(sorted(bodies)):
        pos = np.array([0.0, 0.0, 1.0 + 0.01 * i], dtype=np.float64)
        quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        human_data[body] = (pos, quat)
    return human_data


def test_regression_no_frozen_joints():
    print("[REG] Existing robot (fourier_n1, no frozen_joints declared)")
    gmr = GMR(src_human="smplx", tgt_robot="fourier_n1", verbose=False)
    print(f"      frozen_qpos_addr: {gmr.frozen_qpos_addr} (expected [])")
    assert gmr.frozen_qpos_addr == []
    print(f"      ik_limits length: {len(gmr.ik_limits)} (expected 1)")
    assert len(gmr.ik_limits) == 1
    print("      OK")


def test_frozen_joints_capability():
    print()
    print("[CAP] Synthetic config: declare 2 frozen joints on fourier_n1")
    base_cfg_path = IK_CONFIG_DICT["smplx"]["fourier_n1"]
    with open(base_cfg_path) as f:
        ik_cfg = json.load(f)

    # Pick two arbitrary hinges that exist on fourier_n1 to freeze.
    ik_cfg["frozen_joints"] = ["left_knee_pitch_joint", "right_knee_pitch_joint"]

    # Write to a temp config and swap IK_CONFIG_DICT mapping for fourier_n1.
    tmp_path = str(IK_CONFIG_ROOT / "_tmp_frozen_test.json")
    with open(tmp_path, "w") as f:
        json.dump(ik_cfg, f)
    try:
        IK_CONFIG_DICT["smplx"]["fourier_n1"] = tmp_path
        gmr = GMR(src_human="smplx", tgt_robot="fourier_n1", verbose=False)
        print(f"      frozen_qpos_addr len: {len(gmr.frozen_qpos_addr)} (expected 2)")
        assert len(gmr.frozen_qpos_addr) == 2
        for qadr, dim, init in gmr.frozen_qpos_addr:
            assert dim == 1 and np.allclose(init, [0.0])
        print(f"      ik_limits length: {len(gmr.ik_limits)} (expected 2)")
        assert len(gmr.ik_limits) == 2

        qpos_before = gmr.configuration.data.qpos.copy()
        for qadr, dim, init in gmr.frozen_qpos_addr:
            assert np.allclose(qpos_before[qadr:qadr + dim], init, atol=1e-12)

        human_data = fabricate_human_data(ik_cfg)
        qpos = gmr.retarget(human_data)

        drift_max = 0.0
        for qadr, dim, init in gmr.frozen_qpos_addr:
            slice_val = qpos[qadr:qadr + dim]
            if not np.allclose(slice_val, init, atol=1e-12):
                print(f"      FAIL: qadr={qadr} got {slice_val} != {init}")
                sys.exit(1)
            drift_max = max(drift_max, float(np.max(np.abs(slice_val - init))))
        print(f"      drift after retarget: {drift_max:.2e} (<=1e-12)  OK")
    finally:
        IK_CONFIG_DICT["smplx"]["fourier_n1"] = base_cfg_path
        os.unlink(tmp_path)


if __name__ == "__main__":
    test_regression_no_frozen_joints()
    test_frozen_joints_capability()
    print()
    print("All tests passed.")
