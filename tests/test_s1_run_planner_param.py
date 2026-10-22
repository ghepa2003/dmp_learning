"""pytest: parsing di --planner-param (s1_run.py) e comandi del planner."""
import argparse
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import s1_run  # noqa: E402


def test_parse_ok():
    assert s1_run.parse_planner_param("s1.force=true") == ("s1.force", "true")
    assert s1_run.parse_planner_param("s1.force_theta_deg=-163.5") == ("s1.force_theta_deg", "-163.5")
    assert s1_run.parse_planner_param("k=a=b") == ("k", "a=b")  # solo il primo '=' separa


@pytest.mark.parametrize("bad", ["", "=x", "  =x", "k", "k=", "k=  ", "a b=1", "a:b=1"])
def test_parse_errors(bad):
    with pytest.raises(argparse.ArgumentTypeError):
        s1_run.parse_planner_param(bad)


def test_parser_repeatable_and_error_exit():
    ap = s1_run.make_parser()
    a = ap.parse_args(["--weights-dir", "w", "--planner-param", "s1.force=true", "--planner-param", "s1.force_k=2"])
    assert a.planner_param == [("s1.force", "true"), ("s1.force_k", "2")]
    assert ap.parse_args(["--weights-dir", "w"]).planner_param == []
    with pytest.raises(SystemExit):
        ap.parse_args(["--weights-dir", "w", "--planner-param", "s1.force"])


def test_duplicate_key_is_error_and_dict_keeps_order():
    assert list(s1_run.planner_param_dict([("a", "1"), ("b", "2")]).items()) == [("a", "1"), ("b", "2")]
    with pytest.raises(s1_run.RunError):
        s1_run.planner_param_dict([("a", "1"), ("a", "2")])


def test_build_commands_adds_both_forms_after_defaults():
    a = s1_run.make_parser().parse_args(["--weights-dir", "w", "--planner-param", "s1.force_flip=1"])
    cmds = s1_run.build_commands(a, {"weights": "w.yaml", "demo_params": "d.yaml", "bag": Path("/tmp/b")})
    p = cmds["planner"]
    assert "grasp_planner_node:s1.force_flip:=1" in p and "-p s1.force_flip:=1" in p
    assert p.index("s1.force_flip") > p.index("s1.posthoc_velocity_check")
    base = s1_run.make_parser().parse_args(["--weights-dir", "w"])
    assert "force_flip" not in s1_run.build_commands(base, {"weights": "w", "demo_params": "d", "bag": Path("/tmp/b")})["planner"]
