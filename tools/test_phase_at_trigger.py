#!/usr/bin/env python3
"""Test di phase_at_trigger.py: sequenza sintetica di quaternioni attorno a +z a 2 deg/s da yaw pi."""
import math
import sqlite3
import struct
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import phase_at_trigger as pt  # noqa: E402

RATE = math.radians(-2.0)   # CW visto da +z, come satellite_rotation_angular_velocity_deg_s
YAW0 = math.pi


def quat_z(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def odom_seq(rate, t_end_s=20.0, step_s=0.5, yaw0=YAW0, t0_s=100.0):
    n = int(round(t_end_s / step_s)) + 1
    return [(int(round((t0_s + i * step_s) * 1e9)), quat_z(yaw0 + rate * i * step_s)) for i in range(n)]


def joy_seq(t_start_s, t_trig_s, t0_s=100.0, step_s=0.1, t_end_s=20.0):
    out = []
    n = int(round(t_end_s / step_s)) + 1
    for i in range(n):
        t = t0_s + i * step_s
        b = [1 if t >= t_start_s - 1e-9 else 0, 0, 1 if t >= t_trig_s - 1e-9 else 0]
        out.append((int(round(t * 1e9)), b))
    return out


# ---- serializzazione CDR minima per il test del decoder / lettura bag ----
def _pad(buf, n):
    while (len(buf) - 4) % n:
        buf.append(0)


def _put(buf, fmt, n, *vals):
    _pad(buf, n)
    buf += struct.pack("<" + fmt, *vals)


def _str(buf, s):
    d = s.encode() + b"\0"
    _put(buf, "I", 4, len(d))
    buf += d


def cdr_joy(stamp_ns, buttons):
    b = bytearray(b"\x00\x01\x00\x00")
    _put(b, "iI", 4, stamp_ns // 10**9, stamp_ns % 10**9)
    _str(b, "")
    _put(b, "I", 4, 0)
    _put(b, "I", 4, len(buttons))
    for v in buttons:
        _put(b, "i", 4, v)
    return bytes(b)


def cdr_odom(stamp_ns, q):
    b = bytearray(b"\x00\x01\x00\x00")
    _put(b, "iI", 4, stamp_ns // 10**9, stamp_ns % 10**9)
    _str(b, "world")
    _str(b, "free_target_object")
    _put(b, "ddd", 8, 0.75, 0.0, 0.77)
    _put(b, "dddd", 8, *q)
    _put(b, "d" * 36, 8, *([0.0] * 36))
    return bytes(b)


def make_bag(dirpath, joy, odom):
    d = Path(dirpath)
    (d / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n  storage_identifier: sqlite3\n  relative_file_paths:\n    - bag_0.db3\n")
    con = sqlite3.connect(d / "bag_0.db3")
    con.execute("CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT, type TEXT)")
    con.execute("CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB)")
    con.execute("INSERT INTO topics VALUES(1, ?, 'sensor_msgs/msg/Joy')", (pt.BUTTONS_TOPIC,))
    con.execute("INSERT INTO topics VALUES(2, ?, 'nav_msgs/msg/Odometry')", (pt.ODOM_TOPIC,))
    for t, b in joy:
        con.execute("INSERT INTO messages(topic_id, timestamp, data) VALUES(1, ?, ?)", (t, cdr_joy(t, b)))
    for t, q in odom:
        con.execute("INSERT INTO messages(topic_id, timestamp, data) VALUES(2, ?, ?)", (t, cdr_odom(t, q)))
    con.commit()
    con.close()


class TestTheta(unittest.TestCase):
    def test_theta_at_t0_is_pi(self):
        th, off = pt.theta_from_quat(quat_z(YAW0))
        self.assertAlmostEqual(abs(math.degrees(th)), 180.0, places=6)
        self.assertLess(off, 1e-6)

    def test_theta_decreasing_from_pi(self):
        for t in (0.5, 5.0, 12.3, 20.0):
            th, _ = pt.theta_from_quat(quat_z(YAW0 + RATE * t))
            self.assertAlmostEqual(math.degrees(th), 180.0 - 2.0 * t, places=6)

    def test_wrap_past_pi(self):
        th, _ = pt.theta_from_quat(quat_z(YAW0 + math.radians(2.0) * 5.0))  # 190 deg
        self.assertAlmostEqual(math.degrees(th), -170.0, places=6)

    def test_range_is_half_open(self):
        for yaw in np_linspace(-7.0, 7.0, 141):
            th, _ = pt.theta_from_quat(quat_z(yaw))
            self.assertGreater(th, -math.pi - 1e-12)
            self.assertLessEqual(th, math.pi + 1e-12)

    def test_q_ref_aligned_gives_zero(self):
        th, _ = pt.theta_from_quat(quat_z(YAW0), q_ref=quat_z(YAW0))
        self.assertAlmostEqual(th, 0.0, places=9)
        th, _ = pt.theta_from_quat(quat_z(YAW0 + RATE * 10.0), q_ref=quat_z(YAW0))
        self.assertAlmostEqual(math.degrees(th), -20.0, places=6)

    def test_off_axis_reported(self):
        q = (math.sin(math.radians(5) / 2), 0.0, 0.0, math.cos(math.radians(5) / 2))  # 5 deg about x
        _, off = pt.theta_from_quat(q)
        self.assertAlmostEqual(off, 5.0, places=6)


def np_linspace(a, b, n):
    return [a + (b - a) * i / (n - 1) for i in range(n)]


class TestPipeline(unittest.TestCase):
    def test_phase_and_unwrapped_yaw_interpolated(self):
        # trigger a 10.25 s: tra due campioni di odometria (step 0.5 s) -> slerp
        r = pt.phase_at_trigger(joy_seq(2.0 + 100.0, 10.25 + 100.0), odom_seq(RATE))
        self.assertAlmostEqual(r["theta_deg"], 180.0 - 2.0 * 10.25, places=5)
        self.assertAlmostEqual(r["yaw_unwrapped_deg"], 180.0 - 2.0 * 10.25, places=5)
        self.assertAlmostEqual(r["dt_s"], 8.25, places=6)

    def test_wrap_and_unwrapped_yaw_differ(self):
        # +2 deg/s da yaw pi: a 10 s yaw non avvolto = 200 deg, theta = -160 deg
        r = pt.phase_at_trigger(joy_seq(100.0 + 1.0, 100.0 + 10.0), odom_seq(-RATE))
        self.assertAlmostEqual(r["theta_deg"], -160.0, places=5)
        self.assertAlmostEqual(r["yaw_unwrapped_deg"], 200.0, places=5)

    def test_zero_stamp_is_error(self):
        joy = joy_seq(101.0, 110.0)
        joy[3] = (0, joy[3][1])
        with self.assertRaises(pt.PhaseError):
            pt.phase_at_trigger(joy, odom_seq(RATE))

    def test_trigger_outside_odometry(self):
        with self.assertRaises(pt.PhaseError):
            pt.phase_at_trigger(joy_seq(101.0, 119.9), odom_seq(RATE, t_end_s=10.0))


class TestTau(unittest.TestCase):
    def _csv(self, tmp, t_trig):
        p = Path(tmp) / "demo_raw.csv"
        lines = ["t,x,y,z,qw,qx,qy,qz,gripper_trigger"]
        for i in range(0, 2001):
            t = i * 0.005
            lines.append(f"{t:.6f},0,0,0,1,0,0,0,{1 if abs(t - t_trig) < 1e-9 else 0}")
        p.write_text("\n".join(lines) + "\n")
        return p

    def test_tau_from_csv_and_check(self):
        with tempfile.TemporaryDirectory() as tmp:
            tau = pt.csv_tau(self._csv(tmp, 8.25))
        self.assertAlmostEqual(tau, 8.25, places=6)
        diff, warn = pt.tau_check(8.25 + 0.005, tau)
        self.assertIsNone(warn)
        self.assertAlmostEqual(diff, 0.005, places=9)
        _, warn = pt.tau_check(8.25 + 0.030, tau)
        self.assertIsNotNone(warn)


class TestBagEndToEnd(unittest.TestCase):
    def test_cdr_roundtrip_and_cli(self):
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "bag"
            bag.mkdir()
            make_bag(bag, joy_seq(101.0, 110.25), odom_seq(RATE))
            joy = pt.read_messages(bag, pt.BUTTONS_TOPIC, pt.decode_joy)
            self.assertEqual(joy[0][1], [0, 0, 0])
            odom = pt.read_messages(bag, pt.ODOM_TOPIC, pt.decode_odom)
            for (t, q), (t2, q2) in zip(odom, odom_seq(RATE)):
                self.assertEqual(t, t2)
                for a, b in zip(q, q2):
                    self.assertAlmostEqual(a, b, places=12)
            csv_path = Path(tmp) / "demo_raw.csv"
            csv_path.write_text("t,x,y,z,qw,qx,qy,qz,gripper_trigger\n0.0,0,0,0,1,0,0,0,0\n"
                                "9.25,0,0,0,1,0,0,0,1\n")
            self.assertEqual(pt.main(["--bag", str(bag), "--demo-csv", str(csv_path)]), 0)


if __name__ == "__main__":
    unittest.main()
