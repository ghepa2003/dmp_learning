import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import trim_demo_at_trigger as T

DEMO_HDR = "t,x,y,z,qw,qx,qy,qz,gripper_trigger"
JS_HDR = "t," + ",".join(f"fer_joint{i}" for i in range(1, 8))


def write_demo(d, n, trigger_idx, stem="demo"):
    rows = [f"{0.1 * i:.1f},{i},0,0,1,0,0,0,{1 if i == trigger_idx else 0}" for i in range(n)]
    (d / f"{stem}.csv").write_text("\n".join([DEMO_HDR] + rows) + "\n")


def write_js(d, n, stem="demo"):
    rows = [f"{0.1 * i:.1f}," + ",".join(["0.5"] * 7) for i in range(n)]
    (d / f"{stem}_joint_states.csv").write_text("\n".join([JS_HDR] + rows) + "\n")


class TrimTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.d = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def test_trigger_in_the_middle(self):
        write_demo(self.d, 10, 6)
        write_js(self.d, 10)
        out_demo, out_js, n_read, n_kept, t_trig, tau, is_last, js_read, js_kept = T.trim_demo(self.d / "demo.csv")
        self.assertEqual((n_read, n_kept, js_read, js_kept), (10, 7, 10, 7))
        self.assertAlmostEqual(t_trig, 0.6)
        self.assertAlmostEqual(tau, 0.6)
        self.assertFalse(is_last)
        self.assertEqual(out_demo.name, "demo_trim.csv")
        self.assertEqual(out_js.name, "demo_trim_joint_states.csv")
        demo_lines = out_demo.read_text().splitlines()
        self.assertEqual(demo_lines[0], DEMO_HDR)
        self.assertEqual(len(demo_lines), 8)
        self.assertTrue(demo_lines[-1].endswith(",1"))
        js_lines = out_js.read_text().splitlines()
        self.assertEqual(js_lines[0], JS_HDR)
        self.assertEqual(len(js_lines), 8)
        self.assertTrue(js_lines[-1].startswith("0.6,"))

    def test_trigger_on_last_row(self):
        write_demo(self.d, 5, 4)
        write_js(self.d, 5)
        _, _, n_read, n_kept, t_trig, tau, is_last, _, js_kept = T.trim_demo(self.d / "demo.csv")
        self.assertEqual((n_read, n_kept, js_kept), (5, 5, 5))
        self.assertTrue(is_last)
        self.assertAlmostEqual(t_trig, 0.4)
        self.assertAlmostEqual(tau, 0.4)

    def test_no_trigger(self):
        write_demo(self.d, 5, -1)
        write_js(self.d, 5)
        with self.assertRaisesRegex(T.TrimError, "no row with gripper_trigger"):
            T.trim_demo(self.d / "demo.csv")
        self.assertFalse((self.d / "demo_trim.csv").exists())

    def test_missing_joint_states(self):
        write_demo(self.d, 5, 3)
        with self.assertRaisesRegex(T.TrimError, "joint_states CSV not found"):
            T.trim_demo(self.d / "demo.csv")
        self.assertFalse((self.d / "demo_trim.csv").exists())

    def test_trigger_on_first_row(self):
        write_demo(self.d, 5, 0)
        write_js(self.d, 5)
        with self.assertRaisesRegex(T.TrimError, "first data row"):
            T.trim_demo(self.d / "demo.csv")

    def test_refuses_to_overwrite(self):
        write_demo(self.d, 5, 3)
        write_js(self.d, 5)
        (self.d / "demo_trim.csv").write_text("keep me\n")
        with self.assertRaisesRegex(T.TrimError, "already exists"):
            T.trim_demo(self.d / "demo.csv")
        self.assertEqual((self.d / "demo_trim.csv").read_text(), "keep me\n")
        self.assertFalse((self.d / "demo_trim_joint_states.csv").exists())


if __name__ == "__main__":
    unittest.main()
