#!/usr/bin/env python3
"""Test offline di hw.py: fattibilita' di 'plan', IK, protocollo UDP, parsing di echo_robot_state. Nessun robot."""

import copy
import math
import os
import shutil
import sys
import tempfile
import unittest

import yaml

HW_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HW_DIR)
import hw  # noqa: E402

CONFIG = os.path.join(HW_DIR, "hw_config.yaml")
CMD = os.path.join(HW_DIR, "examples", "grasp_command_nominale.yaml")
WEIGHTS = os.path.join(HW_DIR, "examples", "weights_sintetico_test.yaml")


def failed(checks):
    return sorted(name for name, ok, _d, blocking in checks if blocking and not ok)


class PlanTest(unittest.TestCase):
    def setUp(self):
        self.cfg = hw.load_config(CONFIG)
        with open(CMD) as f:
            self.cmd = yaml.safe_load(f)
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def plan(self, cfg=None, cmd=None, theta0=None, weights=WEIGHTS):
        return hw.plan_feasibility(cfg or self.cfg, cmd or self.cmd, weights, theta0)

    def test_nominale_ok(self):
        ok, checks, info = self.plan()
        self.assertTrue(ok, failed(checks))
        # contact_delay_s = 0 (come l'executor del simulatore): T_c = tau
        self.assertAlmostEqual(info["T_c"], 21.466, places=9)
        self.assertAlmostEqual(info["theta_trig"], -83.9 + 2.0 * 21.466, places=9)  # -40.968
        self.assertAlmostEqual(info["theta0"], -10.0, places=6)
        self.assertAlmostEqual(info["theta_end"], -109.9, places=9)  # T_close_max 6 + T_follow 5 + T_ramp 2 = 13 s
        # condizione A: |omega|*(ramp_s/2 + omega_window_s + margin_arm_s) = 2*(1 + 5 + 2) = 16 deg
        self.assertAlmostEqual(info["condition_a"], 16.0, places=9)
        lo, hi = info["theta0_interval"]
        self.assertAlmostEqual(lo, -40.968 + 16.0, places=6)
        self.assertAlmostEqual(hi, -5.0, places=9)
        # timeout = (theta_trig - theta0)/omega + ramp_s/2 + margine (come real_replay --mode task)
        self.assertAlmostEqual(info["trigger_timeout_s"], 30.968 / 2.0 + 1.0 + 5.0, places=6)
        r = info["robot1_reach"]
        self.assertIsNotNone(r)  # verifica cinematica (reach_check.py), NON "portata verificata": senza carico/collisioni
        self.assertTrue(r["goal_ik_ok"] and r["all_ik_ok"] and r["branch_ok"])
        self.assertGreater(r["min_margin_full_deg"], 5.0)
        names = [c[0] for c in checks]
        self.assertTrue(any(n.startswith("verifica cinematica (senza carico/collisioni)") for n in names))
        self.assertFalse(any("portata verificata" in n.lower() for n in names))

    def test_goal_irraggiungibile_blocca_il_plan(self):
        self.cmd["goal_pose"]["position_m"] = [0.0, 1.6, 0.7]
        ok, checks, info = self.plan()
        self.assertFalse(ok)
        self.assertTrue(any(n.startswith("verifica cinematica") for n in failed(checks)), failed(checks))
        self.assertFalse(info["robot1_reach"]["goal_ik_ok"])

    def test_start_senza_quaternione_e_accettato(self):
        del self.cmd["start_pose"]["quaternion_wxyz"]  # opzionale
        ok, checks, _ = self.plan()
        self.assertTrue(ok, failed(checks))

    def test_theta0_troppo_vicino_al_trigger(self):
        ok, checks, _ = self.plan(theta0=-35.0)  # B chiede theta0 >= -30.968, A >= -24.968
        self.assertFalse(ok)
        self.assertEqual(failed(checks), ["A: s*(theta_trig - theta0) >= |omega|*(ramp_s/2 + omega_window_s + margin_arm_s)", "B: s*(theta_trig - theta0) >= |omega|*T_arm_min"])

    def test_condizione_a_omega_non_ancora_valida(self):
        # theta0 = -28: s*(theta_trig - theta0) = 12.968 deg: B soddisfatto (>= 10), condizione A no (< 16)
        ok, checks, _ = self.plan(theta0=-28.0)
        self.assertFalse(ok)
        self.assertEqual(failed(checks), ["A: s*(theta_trig - theta0) >= |omega|*(ramp_s/2 + omega_window_s + margin_arm_s)"])
        ok, checks, _ = self.plan(theta0=-24.0)
        self.assertTrue(ok, failed(checks))

    def test_theta0_fuori_finestra(self):
        ok, checks, _ = self.plan(theta0=2.0)
        self.assertFalse(ok)
        self.assertEqual(failed(checks), ["C: theta0 dentro la finestra con margine"])
        ok, checks, _ = self.plan(theta0=-3.0)  # dentro la finestra ma nel margine di 5 deg
        self.assertIn("C: theta0 dentro la finestra con margine", failed(checks))

    def test_omega_zero(self):
        cmd = dict(self.cmd, omega_deg_s=0.0)
        ok, checks, info = self.plan(cmd=cmd)
        self.assertFalse(ok)
        self.assertIn("omega != 0", failed(checks))
        self.assertNotIn("theta_trig", info)

    def test_omega_positiva_theta_end_fuori_finestra(self):
        cfg = copy.deepcopy(self.cfg)
        cfg["scene"]["omega_deg_s"] = 2.0
        cmd = dict(self.cmd, omega_deg_s=2.0, theta_star_deg=-10.0)
        # theta_trig = -10 - 2*21.466 = -52.932; s=+1: theta0 <= -68.932 (condizione A)
        ok, checks, info = self.plan(cfg=cfg, cmd=cmd, theta0=-100.0)
        self.assertFalse(ok)
        self.assertAlmostEqual(info["theta_end"], -10.0 + 2.0 * 13.0, places=9)
        self.assertEqual(failed(checks), ["D: theta_end = theta* + omega*(T_close_max+T_follow+T_ramp) dentro la finestra con margine"])

    def test_schema_v2(self):
        cmd = dict(self.cmd, goal_pose=None)
        ok, checks, _ = self.plan(cmd=cmd)
        self.assertFalse(ok)
        self.assertEqual(failed(checks), ["start_pose e goal_pose presenti, finiti, quaternioni unitari"])
        cmd = dict(self.cmd, format="grasp_command_v1", frame_id="map")
        ok, checks, _ = self.plan(cmd=cmd)
        self.assertEqual(failed(checks), ["format grasp_command_v2", "frame_id world"])

    def test_sha256_diverso(self):
        other = os.path.join(self.tmp, "w.yaml")
        with open(WEIGHTS) as f, open(other, "w") as g:
            g.write(f.read() + "# modificato\n")
        ok, checks, _ = self.plan(weights=other)
        self.assertFalse(ok)
        self.assertEqual(failed(checks), ["sha256(weights) == weights_sha256"])

    def test_tau_diverso_e_omega_incoerente(self):
        cmd = dict(self.cmd, contact_time_s=21.4661, omega_deg_s=-2.2)
        ok, checks, _ = self.plan(cmd=cmd)
        self.assertFalse(ok)
        self.assertIn("contact_time_s == tau dei pesi", failed(checks))
        self.assertIn("omega comando coerente con scene.omega_deg_s", failed(checks))

    def test_omega_fuori_dominio_solo_avviso(self):
        cfg = copy.deepcopy(self.cfg)
        cfg["scene"]["omega_deg_s"] = -2.5
        cmd = dict(self.cmd, omega_deg_s=-2.5)
        ok, checks, _ = self.plan(cfg=cfg, cmd=cmd)
        warn = [n for n, good, _d, blocking in checks if not blocking and not good]
        self.assertEqual(len(warn), 1)
        self.assertNotIn(warn[0], failed(checks))


class IkTest(unittest.TestCase):
    def setUp(self):
        self.cfg = hw.load_config(CONFIG)

    def test_fk_posa_validata(self):
        qv = [x * hw.DEG for x in self.cfg["robot2"]["validated_q_deg"]]
        p = hw.fk_flange(qv)[:3, 3]
        self.assertLess(math.dist(p, (0.0, -0.255, 0.8035)), 1e-3)

    def test_ik_riproduce_posa_validata(self):
        r = hw.compute_ik(self.cfg, 0.0)
        self.assertTrue(r["ok"])
        qv = [x * hw.DEG for x in self.cfg["robot2"]["validated_q_deg"]]
        self.assertLess(max(abs(a - b) for a, b in zip(r["q"], qv)) / hw.DEG, 0.01)
        self.assertLess(r["pos_err"], 1e-4)

    def test_ik_psi0_config_solo_q7(self):
        r = hw.compute_ik(self.cfg)
        self.assertTrue(r["ok"])
        g = hw.sat_geom(self.cfg, r["q"])
        self.assertAlmostEqual(g["theta_flange"], -10.0, places=4)
        self.assertAlmostEqual(g["theta_q7"], -10.0, places=3)
        self.assertAlmostEqual(r["q"][6] / hw.DEG, -125.0, places=3)

    def test_ik_rifiuta_vicino_al_limite(self):
        # psi0 = +30 -> q7 = -165 deg, limite -166 deg: margine 1 deg < ik_limit_margin_deg (10)
        r = hw.compute_ik(self.cfg, 30.0)
        self.assertFalse(r["ok"])


class ProtocolTest(unittest.TestCase):
    def test_roundtrip_state(self):
        import struct
        buf = struct.pack(hw.STATE_FMT, hw.STATE_MAGIC, hw.PROTOCOL_VERSION, 120, 7, 2, 123, -1.0, float("nan"), -1.01, 0.5, -0.03,
                          0.0, 0.72, 0.7, 0.0, 0.0, 1.0, 0x30, hw.FLAG_OMEGA_VALID)
        self.assertEqual(buf[:4], b"SATS")
        p = hw.decode_state(buf)
        self.assertEqual((p["seq"], p["state"], p["faults"], p["flags"]), (7, 2, 0x30, 1))
        self.assertTrue(p["omega_valid"])
        old = struct.pack(hw.STATE_FMT, hw.STATE_MAGIC, 2, 120, 7, 2, 123, *([0.0] * 11), 0, 0)
        self.assertIsNone(hw.decode_state(old))  # versione 2 rifiutata (v3: STOP_RAMP)
        self.assertTrue(math.isnan(p["omega_meas"]))
        self.assertIsNone(hw.decode_state(buf[:-1]))
        self.assertIsNone(hw.decode_state(b"X" + buf[1:]))
        self.assertEqual(hw.encode_command(hw.CMD_START, 3)[:4], b"SATC")
        self.assertEqual(len(hw.encode_command(hw.CMD_STOP, 4)), 24)

    def test_conferma_carico(self):
        self.assertTrue(hw.mass_confirmation_ok(" 0.401\n", 0.4))
        self.assertFalse(hw.mass_confirmation_ok("0.402", 0.4))
        self.assertFalse(hw.mass_confirmation_ok("s", 0.4))

    def test_wrap(self):
        self.assertAlmostEqual(hw.wrap_branch(181.0), -179.0)
        self.assertAlmostEqual(hw.wrap_branch(90.0), -270.0)
        self.assertAlmostEqual(hw.wrap_pi_deg(-180.0), 180.0)

    def test_parse_echo_robot_state(self):
        line = ('{"O_T_EE": [1,0,0,0,0,-1,0,0,0,0,-1,0,0.3,0,0.5,1], "F_T_EE": [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0.1034,1], '
                '"m_ee": 0.73, "m_load": 0, "q": [0,-0.78,0,-2.35,0,1.57,0.78], "current_errors": [], '
                '"last_motion_errors": ["joint_position_limits_violation"], "robot_mode": "Idle", "time": 1234}')
        s = hw.parse_robot_state("connecting...\n" + line + "\n" + line + "\nDone.\n")
        self.assertEqual(s["robot_mode"], "Idle")
        OF = hw.flange_from_state(s)
        self.assertAlmostEqual(OF[2, 3], 0.5 + 0.1034, places=9)  # z EE = -z flangia: la flangia e' 0.1034 sopra


if __name__ == "__main__":
    unittest.main(verbosity=2)
