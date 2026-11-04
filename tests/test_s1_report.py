"""Test di tools/s1_report.py.

I test "golden" rianalizzano i bag di analysis_s1_1015 (skip se mancano, o se mancano rclpy /
pinocchio, cioe' fuori dal container). Quelli puri girano ovunque.
"""
import math
import sys
from pathlib import Path

import pytest

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "tools"))
import s1_report  # noqa: E402

AN = WS / "_data" / "analysis" / "analysis_s1_1015"


# ----------------------------------------------------------------- test puri

def test_csv_columns_fixed():
    assert s1_report.CSV_COLUMNS == [
        "tag", "depth", "kt", "follow_s", "hold_s", "squeeze", "spawn_yaw", "k", "theta_star", "d_pos",
        "cost", "wait_s", "yaw_trig_err", "yaw_end_err", "pad_straddle_2s", "f_sum_2s", "f_sum_5s",
        "f_sum_15s", "min_f_5s", "tcp_cube_var_3_8s", "tcp_cube_var_3_15s", "yaw_slope_6_12s",
        "max_err_tgt_0_15s", "cube_disp_max", "tilt_max", "sin_q6_min", "success",
        "stop_cube", "wz_after_stop_degps", "wait_s_actual",
        "tcp_cube_var_close_3_8s", "tcp_cube_var_tang_3_8s", "tcp_cube_var_axial_3_8s",
        "f1_f2_plateau_min_10_20s", "wz_cube_mean_4_8s",
        "tcp_cube_var_tang_3_15s", "tcp_cube_var_axial_3_15s", "pad_straddle_5s", "pad_straddle_15s",
        "f_sum_plateau_10_20s", "tcp_cube_var_close_3_15s",
        "window_end_rel_s", "gap_end_mean_last10s", "pad_straddle_end", "slip_tang_axial_3_end",
        "omega_deg_s", "center_xyz", "arm",
        "min_joint_margin_deg", "min_joint_margin_joint", "min_joint_margin_t_rel_s", "start_goal_deg",
        "criterion_11_ok", "success_1_11"]   # 02/11/2026: due colonne in coda (contratto aggiornato)


def test_wait_range_and_warning():
    assert s1_report.wait_range_s(-2.0) == (5.0, 185.0)
    assert s1_report.wait_range_s(-4.0) == (5.0, 95.0)
    assert s1_report.wait_range_s(0.0) is None
    assert not s1_report.wait_out_of_range(15.0, -2.0)
    assert not s1_report.wait_out_of_range(5.0, -2.0) and not s1_report.wait_out_of_range(185.0, -2.0)
    assert s1_report.wait_out_of_range(4.9, -2.0) and s1_report.wait_out_of_range(185.1, -2.0)
    assert s1_report.wait_out_of_range(100.0, -4.0)   # ammesso a -2, non a -4
    w = s1_report.run_warnings({}, -2.0, 300.0)
    assert len(w) == 1 and s1_report.WAIT_WARNING in w[0]
    assert s1_report.run_warnings({}, -2.0, 15.0) == []


def test_omega_meta_mismatch_warning():
    assert s1_report.run_warnings({"omega_deg_s": -2.0}, -2.0, 15.0) == []
    w = s1_report.run_warnings({"omega_deg_s": -3.0}, -2.0, 15.0)
    assert len(w) == 1 and "omega del meta" in w[0]


def test_csv_row_omega_center_arm():
    g = {k: 0.0 for k in ("tcp_cube_var_3_8s_mm", "tcp_cube_var_3_15s_mm", "yaw_slope_6_12s_deg_s",
                          "max_err_tgt_0_15s_mm", "cube_disp_max_mm", "tilt_max_deg", "sin_q6_min",
                          "tcp_cube_var_close_3_8s_mm", "tcp_cube_var_tang_3_8s_mm", "tcp_cube_var_axial_3_8s_mm",
                          "f1_f2_plateau_min_10_20s_mm", "wz_cube_mean_4_8s_degps", "tcp_cube_var_tang_3_15s_mm",
                          "tcp_cube_var_axial_3_15s_mm", "f_sum_plateau_10_20s_mm", "tcp_cube_var_close_3_15s_mm",
                          "window_end_rel_s", "gap_end_mean_last10s_mm", "slip_tang_axial_3_end_mm")}
    g["pad_straddle_end"] = True
    f = {"sum": 0.0, "min": 0.0}
    g["fingers_mm"] = {k: f for k in ("2", "5", "15")}
    g["pads"] = {k: {"straddle": True} for k in ("2", "5", "15")}
    sel = {"k": 0, "theta_star_deg": 0.0, "d_pos_mm": 0.0, "cost": 0.0, "provisional_delay_s": 0.0}
    meta = {"tag": "t", "omega_deg_s": -3.0, "center_xyz": [0.65, 0.0, 0.77], "arm": "v2"}
    la = {"yaw_trig_err_deg": 0.0, "yaw_end_err_deg": 0.0, "wait_actual_s": 10.0, "omega_deg_s": -2.0}
    row = s1_report.csv_row({"meta": meta, "selection": sel, "launch": la, "grasp": g, "success": True})
    assert (row["omega_deg_s"], row["center_xyz"], row["arm"]) == (-3.0, "0.65,0,0.77", "v2")
    # senza omega nel meta: si usa quello del GraspCommand
    row = s1_report.csv_row({"meta": {"tag": "t"}, "selection": sel, "launch": la, "grasp": g, "success": True})
    assert row["omega_deg_s"] == -2.0 and row["center_xyz"] is None and row["arm"] is None


def _values(**kw):
    v = {"f_sum_plateau_10_20s": 9.64, "min_f_plateau_10_20s": 4.68, "slip_tang_axial_3_15s": 0.1,
         "pad_straddle_2s": True, "pad_straddle_5s": True, "pad_straddle_15s": True,
         "cube_disp_pre_end_mm": 0.1,
         "gap_end_mean_last10s": 9.6, "pad_straddle_end": True, "slip_tang_axial_3_end": 0.1,
         "min_joint_margin_deg": 20.0}
    v.update(kw)
    return v


def _ok(**kw):
    return s1_report.evaluate_criteria(_values(**kw), s1_report.load_criteria())[1]


def test_criteria_names():
    names = set(s1_report.load_criteria()) - set(s1_report.PARAM_KEYS)
    assert names == set(_values())


def test_yaml_values():
    c = s1_report.load_criteria()
    assert c["slip_tang_axial_3_15s"] == {"max": 0.5}
    assert c["slip_tang_axial_3_end"] == {"max": 0.5}
    assert c["gap_end_mean_last10s"] == {"min": 8.0, "max": 11.0}
    assert c["pad_straddle_end"] is True
    assert c["min_window_after_at_end_s"] == 40
    assert c["min_joint_margin_deg"] == {"min": 10}


def test_yaml_fallback_parser(monkeypatch):
    """Senza PyYAML il parser minimale deve dare lo stesso risultato (numero scalare compreso)."""
    import builtins
    real = builtins.__import__

    def no_yaml(name, *a, **k):
        if name == "yaml":
            raise ImportError
        return real(name, *a, **k)

    expected = s1_report.load_criteria()
    monkeypatch.setattr(builtins, "__import__", no_yaml)
    got = s1_report.load_criteria()
    monkeypatch.undo()
    assert {k: v for k, v in got.items()} == {k: (float(v) if k in s1_report.PARAM_KEYS else v)
                                              for k, v in expected.items()}


def test_param_is_not_a_criterion():
    res, ok = s1_report.evaluate_criteria(_values(), s1_report.load_criteria())
    assert s1_report.MIN_WINDOW_KEY not in res and ok


def test_criteria_pass_and_boundaries():
    assert _ok()
    # estremi inclusivi: [8, 11], >= 1, <= 1.0; lt 0.3 escluso
    assert _ok(f_sum_plateau_10_20s=8.0)
    assert _ok(f_sum_plateau_10_20s=11.0)
    assert _ok(min_f_plateau_10_20s=1.0)
    assert _ok(slip_tang_axial_3_15s=0.5)
    assert _ok(gap_end_mean_last10s=8.0)
    assert _ok(gap_end_mean_last10s=11.0)
    assert _ok(slip_tang_axial_3_end=0.5)
    assert _ok(min_joint_margin_deg=10.0)
    assert not _ok(cube_disp_pre_end_mm=0.3)
    assert _ok(cube_disp_pre_end_mm=0.2999)


@pytest.mark.parametrize("over", [
    {"f_sum_plateau_10_20s": 7.99}, {"f_sum_plateau_10_20s": 11.01}, {"min_f_plateau_10_20s": 0.99},
    {"slip_tang_axial_3_15s": 0.51}, {"slip_tang_axial_3_15s": 1.0},
    {"gap_end_mean_last10s": 7.99}, {"gap_end_mean_last10s": 11.01}, {"gap_end_mean_last10s": float("nan")},
    {"gap_end_mean_last10s": None}, {"pad_straddle_end": False}, {"pad_straddle_end": None},
    {"slip_tang_axial_3_end": 0.51}, {"slip_tang_axial_3_end": float("nan")}, {"pad_straddle_2s": False}, {"pad_straddle_5s": False},
    {"pad_straddle_15s": False}, {"min_joint_margin_deg": 9.99}, {"min_joint_margin_deg": 0.0},
    {"min_joint_margin_deg": float("nan")}, {"min_joint_margin_deg": None},
     {"f_sum_plateau_10_20s": float("nan")},
    {"min_f_plateau_10_20s": float("nan")}, {"slip_tang_axial_3_15s": float("nan")},
])
def test_criteria_fail(over):
    assert not _ok(**over)


def test_removed_criteria_and_report_only():
    names = set(s1_report.load_criteria())
    for gone in ("min_f_5s_mm", "tcp_cube_var_3_8s_mm", "f_sum_5s_mm", "tcp_cube_var_close_3_8s",
                 "tcp_cube_var_close_3_15s", "wz_cube_mean_4_8s"):
        assert gone not in names


def test_yaw_slope_is_not_a_criterion():
    assert "yaw_slope_6_12s" not in " ".join(s1_report.load_criteria())


def test_helpers():
    assert s1_report.wrap_pi(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    R = s1_report.quat_to_matrix([0, 0, math.sin(0.3), math.cos(0.3)])
    assert math.atan2(R[1, 0], R[0, 0]) == pytest.approx(0.6)


# ----------------------------------------------------------------- golden

def _analyze(bag, log, **kw):
    pytest.importorskip("rclpy")
    pytest.importorskip("pinocchio")
    bag, log = AN / bag, AN / log
    if not (bag.exists() and log.exists()):
        pytest.skip(f"manca {bag} o {log}")
    return s1_report.analyze(bag, log, {"tag": bag.name}, **kw)


@pytest.fixture(scope="module")
def final2():
    return _analyze("bag_final2", "final2_executor.log")


@pytest.fixture(scope="module")
def final():
    return _analyze("bag_final", "final_executor.log")


def test_golden_final2(final2):
    r = final2["row"]
    assert r["d_pos"] == pytest.approx(7.171, abs=1e-3)
    assert r["f_sum_2s"] == pytest.approx(9.604, abs=1e-3)
    assert r["yaw_trig_err"] == pytest.approx(-0.002, abs=1e-3)
    assert list(r) == s1_report.CSV_COLUMNS


def test_golden_final(final):
    r = final["row"]
    assert r["d_pos"] == pytest.approx(17.387, abs=1e-3)
    assert r["f_sum_2s"] == pytest.approx(14.20, abs=1e-2)


def test_txt_layout(final2):
    txt = s1_report.render_txt(final2)
    lines = txt.splitlines()
    assert len(lines) <= s1_report.MAX_TXT_LINES
    heads = [l for l in lines if l.startswith("== ")]
    assert [h.split()[1] for h in heads] == ["1", "2", "3", "4"]


def test_json_roundtrip_and_csv(final2, tmp_path):
    import json
    s1_report.write_outputs(final2, tmp_path, tmp_path / "r.csv")
    assert json.loads((tmp_path / "report.json").read_text())["row"]["d_pos"] == final2["row"]["d_pos"]
    header, row = (tmp_path / "r.csv").read_text().splitlines()
    assert header.split(",") == s1_report.CSV_COLUMNS
    assert len(row.split(",")) == len(s1_report.CSV_COLUMNS)


def test_timeseries_csv(final2, tmp_path):
    s1_report.write_outputs(final2, tmp_path)
    lines = (tmp_path / "timeseries.csv").read_text().splitlines()
    assert lines[0].split(",") == s1_report.TIMESERIES_COLUMNS
    assert len(lines) > 2 and lines[1].startswith("0.0000,")
    assert "timeseries" not in __import__("json").loads((tmp_path / "report.json").read_text())


def test_end_window_columns_present(final2):
    r = final2["row"]
    cols = list(r)
    names = ["window_end_rel_s", "gap_end_mean_last10s", "pad_straddle_end", "slip_tang_axial_3_end"]
    i = cols.index(names[0])
    assert cols[i:i + 4] == names
    assert r["window_end_rel_s"] == pytest.approx(final2["bag_end_rel_s"])


def test_window_too_short_blocks_success():
    res = _analyze("bag_final2", "final2_executor.log", min_window_s=1e9)
    r = res["row"]
    assert math.isnan(r["gap_end_mean_last10s"]) and math.isnan(r["slip_tang_axial_3_end"])
    assert r["pad_straddle_end"] is None
    assert not res["success"]
    for k in ("gap_end_mean_last10s", "pad_straddle_end", "slip_tang_axial_3_end"):
        assert not res["criteria"][k]["ok"]
    assert "finestra registrata troppo corta" in s1_report.render_txt(res)


def test_window_override_computes_end_values():
    res = _analyze("bag_final2", "final2_executor.log", min_window_s=0.0)
    r = res["row"]
    assert math.isfinite(r["gap_end_mean_last10s"]) and math.isfinite(r["slip_tang_axial_3_end"])
    assert r["pad_straddle_end"] in (0, 1)
    assert "finestra registrata troppo corta" not in s1_report.render_txt(res)


def test_cli_min_window_option():
    import subprocess
    out = subprocess.run([sys.executable, str(WS / "tools/s1_report.py"), "--help"],
                         capture_output=True, text=True).stdout
    assert "--min-window-s" in out


# ----------------------------------------------------------------- criterio 11 e rotazione start-goal

def test_min_joint_margin_synthetic():
    import numpy as np
    lower, upper = -np.ones(7), np.ones(7)
    t = np.arange(0.0, 20.01, 0.1)
    q = np.zeros((len(t), 7))
    q[:, 5] = np.interp(t, [0, 12, 14, 20], [0.0, 0.0, 0.9, 0.9])   # giunto 6 verso +1: margine 0.1 rad a t>=14
    q[:, 2] = np.where(t < 5.0, 0.0, -0.95)                         # giunto 3: margine 0.05 rad da t=5 in poi
    r = s1_report.min_joint_margin(t, q, 4.0, 10.0, 3.0, lower, upper)
    # finestra [4, 13]: giunto 3 a 0.05 rad da t=5 in poi; giunto 6 a t=13 vale 0.45 (margine 0.55)
    assert r["joint"] == 3
    assert r["deg"] == pytest.approx(math.degrees(0.05), abs=1e-6)
    assert r["t_rel_s"] == pytest.approx(5.0 - 10.0, abs=0.02)
    # il minimo cade a fine finestra: giunto 6, t_rel = +3.0
    q[:, 2] = 0.0
    q[:, 5] = np.interp(t, [0, 12, 13, 20], [0.0, 0.0, 0.9, 0.9])
    r = s1_report.min_joint_margin(t, q, 4.0, 10.0, 3.0, lower, upper)
    assert r["joint"] == 6 and r["deg"] == pytest.approx(math.degrees(0.1), abs=1e-6)
    assert r["t_rel_s"] == pytest.approx(3.0, abs=1e-9)
    # limite inferiore: q = -0.98 sul giunto 1 -> margine 0.02
    q[:, 5] = 0.0
    q[:, 0] = -0.98
    r = s1_report.min_joint_margin(t, q, 4.0, 10.0, 3.0, lower, upper)
    assert r["joint"] == 1 and r["deg"] == pytest.approx(math.degrees(0.02), abs=1e-6)


def test_min_joint_margin_window_and_short_bag():
    import numpy as np
    lower, upper = -np.ones(7), np.ones(7)
    t = np.arange(0.0, 20.01, 0.1)
    q = np.zeros((len(t), 7))
    q[t > 14.0, 0] = 0.99            # fuori finestra [4, 13]: ignorato
    r = s1_report.min_joint_margin(t, q, 4.0, 10.0, 3.0, lower, upper)
    assert r["deg"] == pytest.approx(math.degrees(1.0))
    # bag che finisce prima di AT_END + follow_s: NaN, e il criterio 11 non e' soddisfatto
    r = s1_report.min_joint_margin(t[t <= 12.0], q[t <= 12.0], 4.0, 10.0, 3.0, lower, upper)
    assert math.isnan(r["deg"]) and r["joint"] is None
    assert not _ok(min_joint_margin_deg=r["deg"])


def test_criterion_11_decides_success():
    assert not _ok(min_joint_margin_deg=0.0)     # scaleA_A8
    assert not _ok(min_joint_margin_deg=9.9)
    assert _ok(min_joint_margin_deg=10.0)
    assert _ok(min_joint_margin_deg=30.0)
    res, ok = s1_report.evaluate_criteria(_values(min_joint_margin_deg=3.08), s1_report.load_criteria())
    assert not ok and res["min_joint_margin_deg"]["rule"] == ">= 10"


SELECTED = ("[planner] [INFO] [1.0]: grasp_planner_node: selected k=3 theta=12.3400 deg psi=0.0000 deg "
            "wrist_flip={flip} (force_flip=0) branch_rule=min_rot start_goal_deg[A]=41.50 "
            "start_goal_deg[B]=62.25 wait_s=15.0000\n")


def test_parse_selected_start_goal(tmp_path):
    f = tmp_path / "planner.log"
    f.write_text("altro\n" + SELECTED.format(flip=0))
    assert s1_report.parse_selected_start_goal(f) == pytest.approx(41.50)
    f.write_text(SELECTED.format(flip=1))
    assert s1_report.parse_selected_start_goal(f) == pytest.approx(62.25)


def test_parse_selected_start_goal_old_log(tmp_path):
    f = tmp_path / "planner.log"
    f.write_text("grasp_planner_node: selected k=3 theta=12.3400 deg psi=0.0000 deg wrist_flip=0 "
                 "(force_flip=0) wait_s=15.0000\n")
    assert s1_report.parse_selected_start_goal(f) is None
    f.write_text("nessuna riga qui\n")
    assert s1_report.parse_selected_start_goal(f) is None
    assert s1_report.parse_selected_start_goal(tmp_path / "manca.log") is None


def test_rotation_warning_over_58():
    assert s1_report.ROT_VERIFIED_MAX_DEG == 58.1
    assert s1_report.rotation_warnings(None) == [] and s1_report.rotation_warnings(float("nan")) == []
    assert s1_report.rotation_warnings(58.01) == []
    assert s1_report.rotation_warnings(58.1) == []
    w = s1_report.rotation_warnings(58.2)
    assert w == ["rotazione start-goal oltre il dominio verificato (58.1 gradi)"]
    # il successo non dipende dall'avviso
    assert _ok()


def test_csv_row_new_columns():
    g = {k: 0.0 for k in ("tcp_cube_var_3_8s_mm", "tcp_cube_var_3_15s_mm", "yaw_slope_6_12s_deg_s",
                          "max_err_tgt_0_15s_mm", "cube_disp_max_mm", "tilt_max_deg", "sin_q6_min",
                          "tcp_cube_var_close_3_8s_mm", "tcp_cube_var_tang_3_8s_mm", "tcp_cube_var_axial_3_8s_mm",
                          "f1_f2_plateau_min_10_20s_mm", "wz_cube_mean_4_8s_degps", "tcp_cube_var_tang_3_15s_mm",
                          "tcp_cube_var_axial_3_15s_mm", "f_sum_plateau_10_20s_mm", "tcp_cube_var_close_3_15s_mm",
                          "window_end_rel_s", "gap_end_mean_last10s_mm", "slip_tang_axial_3_end_mm")}
    g.update(pad_straddle_end=True, min_joint_margin_deg=3.08, min_joint_margin_joint=6,
             min_joint_margin_t_rel_s=3.0)
    f = {"sum": 0.0, "min": 0.0}
    g["fingers_mm"] = {k: f for k in ("2", "5", "15")}
    g["pads"] = {k: {"straddle": True} for k in ("2", "5", "15")}
    sel = {"k": 0, "theta_star_deg": 0.0, "d_pos_mm": 0.0, "cost": 0.0, "provisional_delay_s": 0.0}
    la = {"yaw_trig_err_deg": 0.0, "yaw_end_err_deg": 0.0, "wait_actual_s": 10.0, "omega_deg_s": -2.0}
    res = {"meta": {"tag": "t"}, "selection": sel, "launch": la, "grasp": g, "success": True}
    row = s1_report.csv_row(res)
    assert (row["min_joint_margin_deg"], row["min_joint_margin_joint"], row["min_joint_margin_t_rel_s"]) == (3.08, 6, 3.0)
    assert row["start_goal_deg"] is None and s1_report._fmt_cell(row["start_goal_deg"]) == ""
    sel["start_goal_deg"] = 61.5
    assert s1_report.csv_row(res)["start_goal_deg"] == 61.5


# ----------------------------------------------------------------- 02/11/2026: successo = criteri 1-10, criterio 11 a parte

def _split(**kw):
    crit, all_ok = s1_report.evaluate_criteria(_values(**kw), s1_report.load_criteria())
    return crit, all_ok, s1_report.split_success(crit)


def test_split_success_all_pass():
    _, _, (s10, c11, s111) = _split()
    assert (s10, c11, s111) == (True, True, True)


def test_split_success_criterion_11_fails_only():
    """criteri 1-10 ok, criterio 11 KO: success vero, criterion_11_ok falso, success_1_11 falso."""
    crit, all_ok, (s10, c11, s111) = _split(min_joint_margin_deg=3.08)
    assert s10 is True and c11 is False and s111 is False
    assert all_ok is False                      # evaluate_criteria resta l'AND di tutti (= success_1_11)
    assert crit["min_joint_margin_deg"]["ok"] is False and crit["min_joint_margin_deg"]["rule"] == ">= 10"
    assert s111 == all_ok


def test_split_success_criterion_11_nan_is_not_ok_but_main_success_stays():
    _, _, (s10, c11, s111) = _split(min_joint_margin_deg=float("nan"))
    assert (s10, c11, s111) == (True, False, False)


def test_split_success_a_criterion_1_10_fails():
    _, _, (s10, c11, s111) = _split(slip_tang_axial_3_15s=0.6)
    assert (s10, c11, s111) == (False, True, False)
    _, _, (s10, c11, s111) = _split(slip_tang_axial_3_15s=0.6, min_joint_margin_deg=0.0)
    assert (s10, c11, s111) == (False, False, False)


def test_slip_threshold_still_0_5_and_threshold_11_unchanged():
    c = s1_report.load_criteria()
    assert c["slip_tang_axial_3_15s"] == {"max": 0.5} and c["min_joint_margin_deg"] == {"min": 10}
    assert s1_report.SEPARATE_CRITERIA == ("min_joint_margin_deg",)


def test_split_success_without_criterion_11_in_yaml():
    crit = {"a": {"value": 1, "rule": "", "ok": True}}
    assert s1_report.split_success(crit) == (True, None, True)


def test_csv_row_has_separate_criterion_11_columns():
    g = {k: 0.0 for k in ("tcp_cube_var_3_8s_mm", "tcp_cube_var_3_15s_mm", "yaw_slope_6_12s_deg_s",
                          "max_err_tgt_0_15s_mm", "cube_disp_max_mm", "tilt_max_deg", "sin_q6_min",
                          "tcp_cube_var_close_3_8s_mm", "tcp_cube_var_tang_3_8s_mm", "tcp_cube_var_axial_3_8s_mm",
                          "f1_f2_plateau_min_10_20s_mm", "wz_cube_mean_4_8s_degps", "tcp_cube_var_tang_3_15s_mm",
                          "tcp_cube_var_axial_3_15s_mm", "f_sum_plateau_10_20s_mm", "tcp_cube_var_close_3_15s_mm",
                          "window_end_rel_s", "gap_end_mean_last10s_mm", "slip_tang_axial_3_end_mm")}
    g["pad_straddle_end"] = True
    f = {"sum": 0.0, "min": 0.0}
    g["fingers_mm"] = {k: f for k in ("2", "5", "15")}
    g["pads"] = {k: {"straddle": True} for k in ("2", "5", "15")}
    sel = {"k": 0, "theta_star_deg": 0.0, "d_pos_mm": 0.0, "cost": 0.0, "provisional_delay_s": 0.0}
    la = {"yaw_trig_err_deg": 0.0, "yaw_end_err_deg": 0.0, "wait_actual_s": 10.0, "omega_deg_s": -2.0}
    res = {"meta": {"tag": "t"}, "selection": sel, "launch": la, "grasp": g, "success": True,
           "criterion_11_ok": False, "success_1_11": False}
    row = s1_report.csv_row(res)
    assert (row["success"], row["criterion_11_ok"], row["success_1_11"]) == (1, 0, 0)
    assert list(row)[-2:] == ["criterion_11_ok", "success_1_11"]
    res2 = dict(res, criterion_11_ok=None)
    del res2["success_1_11"]
    row2 = s1_report.csv_row(res2)
    assert row2["criterion_11_ok"] is None and row2["success_1_11"] is None


def test_interrupted_run_is_neither_success():
    assert s1_report.apply_interrupt(True, {"interrupted": True}) is False


# ----------------------------------------------------------------- tempi per stadio: riga "s1 timing:" del planner.log

TIMING_LINE = ("[grasp_planner_node-1] [INFO] [100.0]: grasp_planner_node: s1 timing: total_s=1.250000 scan_ms=410.000 "
               "rollout_ms=300.500 clearance_ms=120.250 hold_end_ms=15.125 dynamic_ms={dyn} posthoc_ms=0.000 other_ms={other} finalists=12\n")


def test_parse_selection_timing_with_and_without_l2(tmp_path):
    f = tmp_path / "planner.log"
    f.write_text("altro\n" + TIMING_LINE.format(dyn="0.000", other="404.125"))
    t = s1_report.parse_selection_timing(f)
    assert t == {"scan_ms": 410.0, "rollout_ms": 300.5, "clearance_ms": 120.25, "hold_end_ms": 15.125,
                 "dynamic_ms": 0.0, "posthoc_ms": 0.0, "other_ms": 404.125, "finalists": 12}
    f.write_text(TIMING_LINE.format(dyn="350.000", other="54.125"))
    t = s1_report.parse_selection_timing(f)
    assert t["dynamic_ms"] == 350.0 and t["other_ms"] == 54.125


def test_parse_selection_timing_negative_other_is_kept_and_last_line_wins(tmp_path):
    f = tmp_path / "planner.log"
    f.write_text(TIMING_LINE.format(dyn="0.000", other="1.0") + TIMING_LINE.format(dyn="0.000", other="-0.012"))
    assert s1_report.parse_selection_timing(f)["other_ms"] == -0.012


def test_parse_selection_timing_after_a_refusal(tmp_path):
    """La riga e' stampata anche per status 2 / 3: un rifiuto la ha come una selezione riuscita."""
    f = tmp_path / "planner.log"
    f.write_text("grasp_planner_node: s1 status=2 evaluations=480 rollouts=12 note: v2 clearance: no finalist\n"
                 + TIMING_LINE.format(dyn="0.000", other="3.0"))
    t = s1_report.parse_selection_timing(f)
    assert t["scan_ms"] == 410.0 and t["finalists"] == 12


def test_parse_selection_timing_missing_line_file_or_field(tmp_path):
    none = {k: None for k in s1_report.TIMING_KEYS}
    none["finalists"] = None
    f = tmp_path / "planner.log"
    f.write_text("log vecchio senza la riga\n")
    assert s1_report.parse_selection_timing(f) == none
    assert s1_report.parse_selection_timing(tmp_path / "manca.log") == none
    assert s1_report.parse_selection_timing(None) == none
    f.write_text("grasp_planner_node: s1 timing: total_s=1.0 scan_ms=5.0 rollout_ms=abc finalists=x\n")
    t = s1_report.parse_selection_timing(f)
    assert t["scan_ms"] == 5.0 and t["rollout_ms"] is None and t["posthoc_ms"] is None and t["finalists"] is None


def test_csv_columns_unchanged_by_timing():
    """I tempi vanno in report.json["selection"], non in CSV_COLUMNS di s1_report."""
    assert not any(k in s1_report.CSV_COLUMNS for k in s1_report.TIMING_KEYS)
