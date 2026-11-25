#!/usr/bin/env python3
"""Test offline di hw/reach_check.py (cinematica del planner da URDF). Nessun robot."""
import copy
import os
import sys
import unittest

import numpy as np
import yaml

HW_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HW_DIR)
import reach_check as rc  # noqa: E402

URDF = os.path.join(HW_DIR, "..", "..", "fer_flat_effort.urdf")
CMD = os.path.join(HW_DIR, "examples", "grasp_command_hw_j1p90_w2_th0m10.yaml")
CFG = os.path.join(HW_DIR, "hw_config.yaml")


class ReachTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.arm = rc.Arm(URDF)
        cls.cmd = yaml.safe_load(open(CMD)); cls.cfg = yaml.safe_load(open(CFG))

    def test_fk_matches_planner_start_pose(self):
        R, p, _ = self.arm.fk(rc.Q_MEAS)
        s = self.cmd["start_pose"]
        self.assertLess(np.linalg.norm(p - np.array(s["position_m"])), 1e-6)
        q = rc.R_to_quat_wxyz(R)
        self.assertGreater(abs(float(q @ np.array(s["quaternion_wxyz"]))), 1 - 1e-8)

    def test_limits_from_urdf(self):
        self.assertAlmostEqual(self.arm.lo[3], -3.0718); self.assertAlmostEqual(self.arm.hi[5], 3.7525)

    def test_ik_returns_seed_for_seed_pose(self):
        R, p, _ = self.arm.fk(rc.Q_MEAS)
        q, ok, _, _ = self.arm.ik(R, p, rc.Q_MEAS)
        self.assertTrue(ok); self.assertLess(np.abs(q - rc.Q_MEAS).max(), 1e-3)

    def test_nominal_goal_and_follow(self):
        r = rc.analyse(self.arm, self.cmd, self.cfg)
        self.assertTrue(r["all_ik_ok"]); self.assertTrue(r["branch_ok"])
        self.assertAlmostEqual(r["d_full_deg"], -26.0); self.assertAlmostEqual(r["d_close_deg"], -12.0)
        self.assertGreater(r["min_margin_full_deg"], 10.0)
        self.assertLess(r["max_jump_deg"], rc.JUMP_DEG)

    def test_minus_q_goal_same_result(self):
        c2 = copy.deepcopy(self.cmd); c2["goal_pose"]["quaternion_wxyz"] = [-x for x in c2["goal_pose"]["quaternion_wxyz"]]
        a = rc.analyse(self.arm, self.cmd, self.cfg); b = rc.analyse(self.arm, c2, self.cfg)
        self.assertAlmostEqual(a["min_margin_full_deg"], b["min_margin_full_deg"], places=3)

    def test_unreachable_goal_rejected(self):
        c2 = copy.deepcopy(self.cmd); c2["goal_pose"]["position_m"] = [0.0, 1.5, 0.7]
        r = rc.analyse(self.arm, c2, self.cfg)
        self.assertFalse(r["goal_ik_ok"]); self.assertFalse(r["all_ik_ok"])


if __name__ == "__main__":
    unittest.main()
