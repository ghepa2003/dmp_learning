"""Test puri (senza ROS) dei bracci A/B della campagna: config/campaign_{A,B}.yaml e s1_run.py --arm A|B."""
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import s1_run  # noqa: E402

NODE_CPP = ROOT / "src/satellite_grasp_planner/src/ros/grasp_planner_node.cpp"


def as_dict(arm):
    return dict(s1_run.arm_planner_params(arm))


def test_arms_A_and_B_differ_only_in_use_dynamic_follow():
    a, b = as_dict("A"), as_dict("B")
    assert list(a) == list(b)                       # stesse chiavi, stesso ordine
    diff = {k for k in a if a[k] != b[k]}
    assert diff == {"s1.use_dynamic_follow"}
    assert a["s1.use_dynamic_follow"] == "false" and b["s1.use_dynamic_follow"] == "true"


def test_campaign_files_differ_in_one_line():
    la = (s1_run.CAMPAIGN_CONFIG_DIR / "campaign_A.yaml").read_text().splitlines()
    lb = (s1_run.CAMPAIGN_CONFIG_DIR / "campaign_B.yaml").read_text().splitlines()
    assert len(la) == len(lb)
    assert [i for i, (x, y) in enumerate(zip(la, lb)) if x != y] == [
        next(i for i, x in enumerate(la) if x.lstrip().startswith("s1.use_dynamic_follow:"))]


def test_arm_covers_every_s1_key_the_node_declares():
    text = NODE_CPP.read_text()
    node_keys = set(re.findall(r'"(s1\.[A-Za-z0-9_.]+)"', text))
    assert node_keys and node_keys == set(as_dict("A"))


def test_frozen_values_of_the_campaign():
    a = as_dict("A")
    assert a["s1.use_hold_end"] == "true" and a["s1.use_margin"] == "false"
    assert a["s1.posthoc_velocity_check"] == "false"
    assert a["s1.margin_follow_s"] == a["s1.hold_follow_s"] == "3.0"
    assert a["s1.follow_dt_s"] == "0.0005" and a["s1.margin_delta_deg"] == "0.0"
    assert a["s1.finalists_n"] == "40" and a["s1.use_clearance"] == "true" and a["s1.min_clearance_mm"] == "2.0"
    assert a["s1.branch_rule"] == "rotation"
    assert " " not in a["s1.rollout_q_ref"]          # -p vuole la lista senza spazi
    assert a["s1.rollout_q_ref"] == "[0.0,-0.785398163397,0.0,-2.356194490192,0.0,1.570796326795,0.785398163397]"


def test_arm_choices_and_unknown_arm():
    p = s1_run.make_parser()
    assert p.parse_args(["--weights-dir", "w", "--arm", "A"]).arm == "A"
    assert p.parse_args(["--weights-dir", "w", "--arm", "B"]).arm == "B"
    with pytest.raises(SystemExit):
        p.parse_args(["--weights-dir", "w", "--arm", "C"])
    assert s1_run.arm_planner_params("v2") == [("s1.branch_rule", "rotation")]   # invariato
    assert s1_run.arm_planner_params(None) == []


def test_repeating_an_arm_key_in_planner_param_is_an_error():
    pairs = s1_run.arm_planner_params("A") + [("s1.finalists_n", "12")]
    with pytest.raises(s1_run.RunError):
        s1_run.planner_param_dict(pairs)
