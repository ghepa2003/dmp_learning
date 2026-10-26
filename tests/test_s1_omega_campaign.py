"""Test puri (senza ROS) di omega / yaw di spawn / centro / braccio in tools/s1_run.py e s1_campaign.py."""
import argparse
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import s1_campaign  # noqa: E402
import s1_run  # noqa: E402


# ------------------------------------------------------------ yaw di spawn "auto"

def test_auto_yaw_minus2():
    assert s1_run.auto_spawn_yaw_deg(-2.0) == pytest.approx(-73.1, abs=1e-9)
    assert abs(s1_run.auto_spawn_yaw_deg(-2.0) - (-73.05)) < 0.1


@pytest.mark.parametrize("om,expected", [(-3.0, -27.9), (-1.0, -118.3), (2.0, 106.1)])
def test_auto_yaw_values(om, expected):
    assert s1_run.auto_spawn_yaw_deg(om) == pytest.approx(expected, abs=1e-9)


def test_wrap_is_half_open():
    assert s1_run.wrap_deg_half_open(-180.0) == 180.0
    assert s1_run.wrap_deg_half_open(180.0) == 180.0
    assert s1_run.wrap_deg_half_open(540.0) == 180.0
    assert s1_run.wrap_deg_half_open(-181.0) == pytest.approx(179.0)
    assert -180.0 < s1_run.auto_spawn_yaw_deg(-7.3, 100.0, 77.0) <= 180.0


def test_parse_spawn_yaw():
    assert s1_run.parse_spawn_yaw("auto") == "auto" and s1_run.parse_spawn_yaw("AUTO") == "auto"
    assert s1_run.parse_spawn_yaw("-73") == -73.0
    with pytest.raises(argparse.ArgumentTypeError):
        s1_run.parse_spawn_yaw("boh")


# ------------------------------------------------------------ omega nei tre punti

def test_omega_values_default_is_unchanged_wz():
    v = s1_run.omega_values(-2.0)
    assert float(v["spawn_wz_rad_s"]) == pytest.approx(-0.034906585, abs=1e-9)
    assert v["spawn_wz_rad_s"] == "-0.034906585"
    assert v["executor_param"] == v["planner_param"] == f"{s1_run.OMEGA_PARAM}:=-2.000000"
    assert s1_run.OMEGA_PARAM == "satellite_rotation_angular_velocity_deg_s"


@pytest.mark.parametrize("om", [-2.0, -0.5, -3.7, 1.25, -10.0])
def test_omega_three_points_agree(om):
    v = s1_run.omega_values(om)
    wz = float(v["spawn_wz_rad_s"])
    for k in ("executor_param", "planner_param"):
        assert abs(math.radians(float(v[k].split(":=")[1])) - wz) < 1e-6 < s1_run.OMEGA_TOL_RAD_S
    s1_run.check_omega_consistent(om)


def _args(**kw):
    ap = s1_run.make_parser()
    a = ap.parse_args(["--weights-dir", "w"] + [x for k, v in kw.items() for x in (f"--{k.replace('_', '-')}", str(v))])
    return a


def test_build_commands_uses_run_omega_and_center():
    a = _args(omega_deg_s=-3.0, center_xyz="0.65,0.1,0.8", spawn_yaw_deg=-27.9)
    cmds = s1_run.build_commands(a, {"weights": "w.yaml", "demo_params": "d.yaml", "bag": Path("/b")})
    assert "wz:=-0.052359878" in cmds["spawn"]
    assert "x:=0.650 y:=0.100 z:=0.800" in cmds["spawn"]
    for k in ("executor", "planner"):
        assert f"{s1_run.OMEGA_PARAM}:=-3.000000" in cmds[k]
    assert "satellite_rotation_center:=[0.650,0.100,0.800]" in cmds["executor"]
    assert "satellite_rotation_center:=[0.650,0.100,0.800]" in cmds["planner"]
    assert "cube_geometry.center_world:=[0.650,0.100,0.800]" in cmds["planner"]
    assert "-2.000000" not in cmds["planner"] and "-2.000000" not in cmds["executor"]


def test_build_commands_default_matches_old_values():
    a = _args()
    cmds = s1_run.build_commands(a, {"weights": "w.yaml", "demo_params": "d.yaml", "bag": Path("/b")})
    assert "wz:=-0.034906585" in cmds["spawn"] and "x:=0.700 y:=0.000 z:=0.770" in cmds["spawn"]
    assert f"{s1_run.OMEGA_PARAM}:=-2.000000" in cmds["executor"]


def test_goal_gate_uses_center():
    # goal a 155.1 mm dal centro lungo theta*=0 -> x_c = 155.1 per qualunque centro
    assert s1_run.goal_x_c_mm((0.65 + 0.1551, 0.1), 0.0, center=(0.65, 0.1, 0.8)) == pytest.approx(155.1)
    assert s1_run.goal_x_c_mm((0.70 + 0.1551, 0.0), 0.0) == pytest.approx(155.1)


def test_center_and_arm_parsing():
    assert s1_run.parse_center_xyz("0.70,0.0,0.77") == (0.70, 0.0, 0.77)
    for bad in ("0.7,0", "a,b,c", "1,2,3,4", "nan,0,0"):
        with pytest.raises(argparse.ArgumentTypeError):
            s1_run.parse_center_xyz(bad)
    assert s1_run.arm_planner_params(None) == []
    assert s1_run.arm_planner_params("d_only") == [("s1.use_w", "false"), ("s1.branch_rule", "rotation")]
    assert s1_run.arm_planner_params("v2") == [("s1.branch_rule", "rotation")]


def test_legacy_arm_is_removed_and_s1_mode_is_never_passed():
    # the planner has no s1.mode any more (an undeclared -p is ignored silently): no arm may pass it
    assert set(s1_run.ARMS) == {"d_only", "v2"}
    for arm in s1_run.ARMS:
        assert all(k != "s1.mode" for k, _ in s1_run.arm_planner_params(arm))
    with pytest.raises(s1_run.RunError, match="legacy"):
        s1_run.arm_planner_params("legacy")
    with pytest.raises(s1_run.RunError):
        s1_run.arm_planner_params("boh")
    with pytest.raises(SystemExit):  # argparse of s1_run.py: choices=sorted(ARMS)
        s1_run.make_parser().parse_args(["--weights-dir", "w", "--arm", "legacy"])


def test_arm_never_passes_rollout_q_ref():
    for arm in s1_run.ARMS:
        assert all(k != "s1.rollout_q_ref" for k, _ in s1_run.arm_planner_params(arm))


# ------------------------------------------------------------ campagna

def _grid(**cli):
    full = {ax: None for ax in s1_campaign.AXES}
    full.update(cli)
    return s1_campaign.build_grid(full)


def test_default_grid_single_run_unchanged_axes():
    g = _grid()
    assert len(g) == 1
    c = g[0]
    assert c["omega_deg_s"] == -2.0 and c["center_xyz"] == (0.70, 0.0, 0.77) and c["arm"] is None
    assert c["spawn_yaw"] == 180.0


def test_spawn_yaw_auto_default_when_omega_varies():
    assert all(c["spawn_yaw"] == "auto" for c in _grid(omega_deg_s=["-2", "-3"]))
    assert _grid(omega_deg_s=["-3"])[0]["spawn_yaw"] == "auto"
    assert _grid(omega_deg_s=["-2"])[0]["spawn_yaw"] == 180.0
    # esplicito vince
    assert _grid(omega_deg_s=["-3"], spawn_yaw=["-30"])[0]["spawn_yaw"] == -30.0
    assert _grid(spawn_yaw=["auto"])[0]["spawn_yaw"] == "auto"


def test_grid_is_cases_times_arms():
    g = _grid(omega_deg_s=["-2", "-3"], center_xyz=["0.70,0.0,0.77", "0.65,0.0,0.77"],
              arm=["d_only", "v2"])
    assert len(g) == 2 * 2 * 2
    assert {c["arm"] for c in g} == {"d_only", "v2"}
    assert len({s1_campaign.make_tag(c) for c in g}) == len(g)   # tag univoci


def test_tag_has_omega_center_arm():
    c = _grid(omega_deg_s=["-3"], center_xyz=["0.65,0.0,0.77"], arm=["d_only"])[0]
    tag = s1_campaign.make_tag(c)
    assert "_wm3_" in tag and "_c0.65x0x0.77" in tag and tag.endswith("_ad_only") and "yauto" in tag
    assert "-" not in tag
    assert s1_campaign.make_tag(_grid()[0]) == "d12.5_kt2000_y180_f3_h120_qdef_wm2_c0.7x0x0.77"


def test_bad_values_exit():
    with pytest.raises(SystemExit):
        _grid(arm=["boh"])
    with pytest.raises(SystemExit):
        _grid(arm=["legacy"])
    with pytest.raises(SystemExit):
        _grid(center_xyz=["1,2"])


def test_dry_run_prints_full_grid(capsys):
    rc = s1_campaign.main(["--weights-dir", "w", "--outroot", "/nonexistent/x", "--dry-run",
                           "--omega-deg-s", "-2", "-3", "--arm", "d_only", "v2"])
    out = capsys.readouterr().out
    assert rc == 0 and out.startswith("4 run in griglia")
    assert out.count("yaw0=") == 4 and "yaw0=-73.10deg" in out and "yaw0=-27.90deg" in out
    assert "_ad_only" in out and "_av2" in out
