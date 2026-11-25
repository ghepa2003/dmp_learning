"""Test del convertitore GraspCommand -> YAML v2 (senza ROS). pytest tools/test_make_hw_grasp_command.py"""
import math
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import make_hw_grasp_command as m  # noqa: E402


def msg(**kw):
    d = {"status": 1, "theta_star_rad": math.radians(-122.0), "contact_time_s": 21.466, "omega_rad_s": math.radians(-2.0),
         "weights_sha256": m.EXPECTED_SHA, "k": 2, "wrist_flip": True, "start_position": [0.0, 0.43, 0.77],
         "start_quat_wxyz": [0.5, -0.5, -0.5, -0.5], "goal_position": [-0.04, 0.5, 0.7],
         "goal_quat_xyzw": [0.1, 0.2, 0.3, 0.9273618495495704]}
    d.update(kw)
    return d


def test_quaternion_order_roundtrip():
    xyzw = [0.1, 0.2, 0.3, 0.9]
    wxyz = m.quat_xyzw_to_wxyz(xyzw)
    assert wxyz == [0.9, 0.1, 0.2, 0.3]
    assert m.quat_wxyz_to_xyzw(wxyz) == xyzw
    c = m.convert_command(msg(goal_quat_xyzw=xyzw))
    assert c["goal_pose"]["quaternion_wxyz"] == [0.9, 0.1, 0.2, 0.3]


@pytest.mark.parametrize("code,name", [(0, "UNSET"), (1, "SELECTED"), (2, "NO_FEASIBLE_CANDIDATE"), (3, "POSTHOC_FAILED")])
def test_status_mapping(code, name):
    assert m.convert_command(msg(status=code))["status"] == name


def test_status_out_of_range():
    with pytest.raises(ValueError):
        m.convert_command(msg(status=4))


@pytest.mark.parametrize("deg,exp", [(-83.9, -83.9), (90.0, -270.0), (-270.0, -270.0), (-271.0, 89.0), (269.0, -91.0), (-122.0, -122.0), (0.0, 0.0), (450.0, 90.0 - 360.0)])
def test_wrap_branch(deg, exp):
    assert m.wrap_branch_deg(deg) == pytest.approx(exp, abs=1e-9)
    assert -270.0 <= m.wrap_branch_deg(deg) < 90.0


def test_theta_wrap_in_command():
    assert m.convert_command(msg(theta_star_rad=math.radians(200.0)))["theta_star_deg"] == pytest.approx(-160.0)


def test_rad_to_deg():
    c = m.convert_command(msg(omega_rad_s=-0.034906585))
    assert c["omega_deg_s"] == pytest.approx(-2.0, abs=1e-6)
    assert c["theta_star_deg"] == pytest.approx(-122.0)


def test_sha_mismatch_rejected():
    with pytest.raises(ValueError, match="weights_sha256"):
        m.convert_command(msg(weights_sha256="0" * 64))


def test_yaml_keys_and_diag_as_comments():
    c = m.convert_command(msg(), now_s=1.0)
    text = m.dump_yaml(c, {"cost": 30.0, "evaluations": 720})
    d = yaml.safe_load(text)
    assert set(d) == {"format", "status", "frame_id", "stamp_unix_s", "theta_star_deg", "contact_time_s", "omega_deg_s",
                      "weights_sha256", "start_pose", "goal_pose", "k", "wrist_flip"}
    assert d["format"] == "grasp_command_v2" and d["wrist_flip"] is True and d["k"] == 2
    assert "cost: 30.0" in text and "cost" not in d
