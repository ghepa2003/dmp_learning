"""Test di tools/s1_kinematic_metrics.py.

Quelli puri usano dati sintetici e girano ovunque (nessun ROS, nessun pinocchio). I due test su
JacobianKin richiedono pinocchio e fer_flat_effort.urdf (skip altrimenti, cioe' fuori dal container).
"""
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "tools"))
import s1_kinematic_metrics as km  # noqa: E402

URDF = WS / "fer_flat_effort.urdf"
VEL_LIM = np.array([2.175] * 4 + [2.61] * 3)
T_TRIG, T_END, T_CONTACT = 2.0, 5.0, 3.5
V = 0.1 * np.arange(1, 8)             # rad/s in approccio; in follow 2*V


# ----------------------------------------------------------------- dati sintetici

def _bag(t_max=10.0, dt=0.001):
    """q_j(t) = V_j t per t < AT_END, poi pendenza 2 V_j: qdot = V (approccio), 2V (follow) esatte."""
    t = np.arange(0.0, t_max + dt / 2, dt)
    q = np.outer(np.minimum(t, T_END), V) + np.outer(np.maximum(t - T_END, 0.0), 2.0 * V)
    return t, q


def _jac_fn(q):
    """6x7 con w_full(q) = |q[0]| esatto: diagonale di 1 tranne J[0,0] = q[0]."""
    J = np.zeros((6, 7))
    for i in range(1, 6):
        J[i, i] = 1.0
    J[0, 0] = q[0]
    return J


def _rz(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


P_TCP = np.array([0.5, 0.1, 0.3])


def _pose_fn(q):
    return P_TCP.copy(), _rz(10.0)


def _metrics(follow_s=3.0, t_max=10.0, **kw):
    t, q = _bag(t_max)
    return km.compute_metrics(t, q, T_TRIG, T_END, T_CONTACT, follow_s, _jac_fn,
                              -10.0 * np.ones(7), 10.0 * np.ones(7), VEL_LIM, **kw)


def _known_J(s, seed=0):
    """6x7 con valori singolari esattamente s."""
    rng = np.random.default_rng(seed)
    U, _ = np.linalg.qr(rng.standard_normal((6, 6)))
    Vm, _ = np.linalg.qr(rng.standard_normal((7, 7)))
    S = np.zeros((6, 7))
    S[:6, :6] = np.diag(s)
    return U @ S @ Vm.T


# ----------------------------------------------------------------- w_full

def test_w_full_is_product_of_singular_values():
    s = [1.0, 2.0, 3.0, 0.5, 0.25, 4.0]
    J = _known_J(s)
    assert km.full_manipulability(J) == pytest.approx(3.0, rel=1e-12)
    assert km.full_manipulability(J) == pytest.approx(math.sqrt(np.linalg.det(J @ J.T)), rel=1e-9)


def test_w_full_rank_deficient_is_zero_and_scaling_law():
    J = _known_J([1.0, 2.0, 3.0, 0.5, 0.25, 4.0], seed=1)
    Jd = J.copy()
    Jd[5] = Jd[0] + Jd[1]
    assert km.full_manipulability(Jd) == pytest.approx(0.0, abs=1e-12)
    c = 1.7                                   # come test_s3_landscape_log.cpp: w_full -> c^3 w_full
    Jc = J * np.array([1, 1, 1, c, c, c])[:, None]
    assert km.full_manipulability(Jc) == pytest.approx(c ** 3 * km.full_manipulability(J), rel=1e-12)


def test_w_full_requires_six_rows():
    with pytest.raises(ValueError):
        km.full_manipulability(np.ones((3, 7)))


def test_rel_err():
    assert km.rel_err(0.35, 0.30) == pytest.approx(1.0 / 6.0)
    assert km.rel_err(0.25, 0.50) == pytest.approx(-0.5)
    assert math.isnan(km.rel_err(0.3, None)) and math.isnan(km.rel_err(0.3, 0.0))
    assert math.isnan(km.rel_err(0.3, float("nan")))


# ----------------------------------------------------------------- utilita'

def test_window_grid_ends_on_hi_and_empty_when_reversed():
    g = km.window_grid(2.0, 8.0, 0.01)
    assert g[0] == 2.0 and g[-1] == 8.0 and np.all(np.diff(g) > 0)
    assert len(km.window_grid(2.0, 2.0)) == 1
    assert len(km.window_grid(3.0, 2.0)) == 0


def test_interp_q_linear_and_clamped():
    t = np.array([0.0, 1.0, 2.0])
    q = np.outer(t, np.arange(1, 8))
    assert km.interp_q(0.5, t, q) == pytest.approx(0.5 * np.arange(1, 8))
    assert km.interp_q(5.0, t, q) == pytest.approx(2.0 * np.arange(1, 8))
    assert km.interp_q(np.array([0.5, 1.5]), t, q).shape == (2, 7)


def test_parse_urdf_limits_real_file():
    if not URDF.exists():
        pytest.skip("fer_flat_effort.urdf assente")
    lo, hi, vel = km.parse_urdf_limits(URDF)
    assert lo[3] == pytest.approx(-3.0718) and hi[3] == pytest.approx(-0.0698)
    assert lo[5] == pytest.approx(-0.0175) and hi[5] == pytest.approx(3.7525)
    assert vel == pytest.approx([2.175] * 4 + [2.61] * 3)


def test_parse_urdf_limits_missing_joint(tmp_path):
    p = tmp_path / "x.urdf"
    p.write_text('<robot><joint name="fer_joint1" type="revolute"><limit lower="-1" upper="1" velocity="2"/></joint></robot>')
    with pytest.raises(ValueError):
        km.parse_urdf_limits(p)


# ----------------------------------------------------------------- velocita'

def test_velocity_stats_known_peaks():
    qdot = np.zeros((3, 7))
    qdot[1] = [1.0, -3.0, 0, 0, 0, 0, 0.5]
    st = km.velocity_stats(qdot, VEL_LIM)
    assert st["norm_peak"] == pytest.approx(math.sqrt(10.25))
    assert st["norm_rms"] == pytest.approx(math.sqrt(10.25 / 3.0))
    assert st["peak"] == pytest.approx([1, 3, 0, 0, 0, 0, 0.5])
    assert st["ratio"][1] == pytest.approx(3.0 / 2.175) and st["ratio"][6] == pytest.approx(0.5 / 2.61)
    assert st["ratio_max"] == pytest.approx(3.0 / 2.175) and st["worst_joint"] == 2


def test_velocity_stats_empty_is_nan():
    st = km.velocity_stats(np.empty((0, 7)), VEL_LIM)
    assert math.isnan(st["norm_peak"]) and math.isnan(st["ratio_max"]) and st["worst_joint"] is None


def test_joint_velocity_constant_slope_exact():
    t, q = _bag()
    qd = km.joint_velocity(np.array([2.5, 3.0]), t, q)
    assert qd == pytest.approx(np.tile(V, (2, 1)), rel=1e-9)


def test_joint_velocity_clipped_to_window_does_not_leak_across_phases():
    t, q = _bag()
    # all'estremo AT_END la differenza centrata non ristretta darebbe 1.5 V
    unclipped = km.joint_velocity(np.array([T_END]), t, q)
    assert unclipped[0] == pytest.approx(1.5 * V, rel=1e-6)
    assert km.joint_velocity(np.array([T_END]), t, q, lo=T_TRIG, hi=T_END)[0] == pytest.approx(V, rel=1e-9)
    assert km.joint_velocity(np.array([T_END]), t, q, lo=T_END, hi=8.0)[0] == pytest.approx(2 * V, rel=1e-9)


# ----------------------------------------------------------------- compute_metrics

def test_compute_metrics_velocity_per_phase():
    m = _metrics()
    n2 = math.sqrt(float(np.sum(V ** 2)))
    assert m["qdot_norm_peak_approach"] == pytest.approx(n2, rel=1e-6)
    assert m["qdot_norm_rms_approach"] == pytest.approx(n2, rel=1e-6)
    assert m["qdot_norm_peak_follow"] == pytest.approx(2 * n2, rel=1e-6)
    for j in range(7):
        assert m[f"qdot_peak_j{j + 1}_approach"] == pytest.approx(V[j], rel=1e-6)
        assert m[f"qdot_peak_j{j + 1}_follow"] == pytest.approx(2 * V[j], rel=1e-6)
        assert m[f"qdot_ratio_j{j + 1}_approach"] == pytest.approx(V[j] / VEL_LIM[j], rel=1e-6)
    # il rapporto maggiore e' il giunto 7 (0.7/2.61 > 0.4/2.175)
    assert m["qdot_ratio_max_approach"] == pytest.approx(0.7 / 2.61, rel=1e-6)
    assert m["qdot_ratio_worst_joint_approach"] == 7
    assert m["qdot_ratio_max_follow"] == pytest.approx(1.4 / 2.61, rel=1e-6)


def test_compute_metrics_w_full_contact_min_and_follow_end():
    m = _metrics(predicted_w_full=0.30)
    assert m["w_full_contact"] == pytest.approx(0.1 * T_CONTACT, rel=1e-9)        # |q0(3.5)| = 0.35
    assert m["t_contact_rel_s"] == pytest.approx(T_CONTACT - T_END)
    assert m["w_full_contact_rel_err"] == pytest.approx((0.35 - 0.30) / 0.30, rel=1e-9)
    assert m["w_full_min"] == pytest.approx(0.1 * T_TRIG, rel=1e-9)               # q0 crescente: minimo al TRIGGER
    assert m["w_full_min_t_rel_s"] == pytest.approx(T_TRIG - T_END)
    assert m["w_full_follow_end"] == pytest.approx(0.1 * T_END + 0.2 * 3.0, rel=1e-9)   # 1.1


def test_compute_metrics_follow_s_changes_end_window():
    m = _metrics(follow_s=1.0)
    assert m["w_full_follow_end"] == pytest.approx(0.5 + 0.2, rel=1e-9)
    assert m["qdot_norm_peak_follow"] == pytest.approx(2 * math.sqrt(float(np.sum(V ** 2))), rel=1e-6)


def test_compute_metrics_no_follow_gives_nan_follow_velocity():
    m = _metrics(follow_s=0.0)
    assert math.isnan(m["qdot_norm_peak_follow"]) and m["qdot_ratio_worst_joint_follow"] is None
    assert m["w_full_follow_end"] == pytest.approx(0.5, rel=1e-9)                 # fine follow = AT_END


def test_compute_metrics_margin():
    m = _metrics(margin_pred_deg=15.0, max_err_tgt_mm=1.25)
    # giunto 7: q7(8) = 0.7*5 + 1.4*3 = 7.7 -> margine 10 - 7.7 = 2.3 rad, a fine follow (+3 s)
    assert m["margin_min_deg"] == pytest.approx(math.degrees(2.3), rel=1e-9)
    assert m["margin_min_joint"] == 7
    assert m["margin_min_t_rel_s"] == pytest.approx(3.0)
    # al contatto: q7(3.5) = 2.45 -> 7.55 rad
    assert m["margin_contact_deg"] == pytest.approx(math.degrees(7.55), rel=1e-9)
    assert m["margin_pred_deg"] == 15.0 and m["max_err_tgt_0_15s_mm"] == 1.25


def test_compute_metrics_margin_lower_limit_side():
    t, q = _bag()
    lower = np.zeros(7)
    lower[0] = 0.15                                  # q0(2) = 0.2 -> margine 0.05 rad al TRIGGER, giunto 1
    m = km.compute_metrics(t, q, T_TRIG, T_END, T_CONTACT, 3.0, _jac_fn, lower, 10.0 * np.ones(7), VEL_LIM)
    assert m["margin_min_joint"] == 1
    assert m["margin_min_deg"] == pytest.approx(math.degrees(0.05), rel=1e-6)
    assert m["margin_min_t_rel_s"] == pytest.approx(T_TRIG - T_END)


def test_w_full_at_end_and_after():
    m = _metrics()
    # |q0|: q0(5) = 0.5, poi pendenza 0.2 -> 0.7, 0.9, 1.1 a +1, +2, +3 s
    assert m["w_full_end"] == pytest.approx(0.5, rel=1e-9)
    assert m["w_full_end_p1s"] == pytest.approx(0.7, rel=1e-9)
    assert m["w_full_end_p2s"] == pytest.approx(0.9, rel=1e-9)
    assert m["w_full_end_p3s"] == pytest.approx(1.1, rel=1e-9)
    assert m["w_full_end"] != pytest.approx(m["w_full_contact"])      # AT_END e contatto sono istanti diversi


def test_w_full_end_independent_of_follow_s():
    assert _metrics(follow_s=0.0)["w_full_end_p3s"] == pytest.approx(1.1, rel=1e-9)


def test_w_full_after_end_is_nan_beyond_bag():
    m = _metrics(t_max=6.5)                          # bag fino a 6.5 s: +1 s (6.0) c'e', +2 s (7.0) no
    assert m["w_full_end_p1s"] == pytest.approx(0.7, rel=1e-9)
    assert math.isnan(m["w_full_end_p2s"]) and math.isnan(m["w_full_end_p3s"])


def test_w_full_end_rel_err_uses_at_end_not_contact():
    m = _metrics(predicted_w_full=0.4)
    assert m["w_full_end_rel_err"] == pytest.approx(0.25, rel=1e-9)                 # (0.5 - 0.4) / 0.4
    assert m["w_full_contact_rel_err"] == pytest.approx((0.35 - 0.4) / 0.4, rel=1e-9)
    assert math.isnan(_metrics()["w_full_end_rel_err"])


def test_q_end_is_measured_q_at_at_end():
    m = _metrics()
    for j in range(7):
        assert m[f"q_end_j{j + 1}"] == pytest.approx(V[j] * T_END, rel=1e-9)


def test_rotation_angle_deg():
    assert km.rotation_angle_deg(np.eye(3), np.eye(3)) == pytest.approx(0.0, abs=1e-9)
    assert km.rotation_angle_deg(np.eye(3), _rz(90.0)) == pytest.approx(90.0, rel=1e-9)
    assert km.rotation_angle_deg(_rz(30.0), _rz(-20.0)) == pytest.approx(50.0, rel=1e-9)
    Rx180 = np.diag([1.0, -1.0, -1.0])
    assert km.rotation_angle_deg(np.eye(3), Rx180) == pytest.approx(180.0, abs=1e-6)
    assert km.rotation_angle_deg(np.eye(3), _rz(0.01)) == pytest.approx(0.01, rel=1e-6)


def test_pose_error_known_values():
    pos, ori = km.pose_error(P_TCP, _rz(10.0), P_TCP + np.array([0.003, 0.004, 0.0]), _rz(40.0))
    assert pos == pytest.approx(5.0, rel=1e-9)                       # 3-4-5 mm
    assert ori == pytest.approx(30.0, rel=1e-9)


def test_pose_error_end_in_compute_metrics():
    m = _metrics(pose_fn=_pose_fn, goal_p=P_TCP + np.array([0.0, 0.0, 0.002]), goal_R=_rz(25.0))
    assert m["pose_err_end_pos_mm"] == pytest.approx(2.0, rel=1e-9)
    assert m["pose_err_end_ori_deg"] == pytest.approx(15.0, rel=1e-9)


def test_pose_error_end_nan_without_goal():
    m = _metrics()
    assert math.isnan(m["pose_err_end_pos_mm"]) and math.isnan(m["pose_err_end_ori_deg"])
    m = _metrics(pose_fn=_pose_fn)                   # pose ma niente goal
    assert math.isnan(m["pose_err_end_pos_mm"])


def test_compute_metrics_keys_are_columns():
    keys = set(_metrics())
    cols = set(km.KIN_COLUMNS)
    assert keys <= cols
    assert cols - keys == {"tag", "k", "theta_star_deg"}
    assert len(km.KIN_COLUMNS) == len(cols)


# ----------------------------------------------------------------- uscite

def _row():
    r = _metrics(predicted_w_full=0.30, margin_pred_deg=15.0, max_err_tgt_mm=1.25)
    r.update(tag="t1", k=2, theta_star_deg=12.5)
    return r


def test_json_line_is_single_valid_line_with_null_for_nonfinite():
    r = _metrics(follow_s=0.0)                       # NaN in follow, w_full_pred assente
    r.update(tag="t0", k=0, theta_star_deg=1.0)
    s = km.json_line(r)
    assert "\n" not in s and "NaN" not in s
    d = json.loads(s)
    assert list(d) == km.KIN_COLUMNS
    assert d["qdot_norm_peak_follow"] is None and d["w_full_pred"] is None and d["tag"] == "t0"


def test_write_outputs_and_csv_append(tmp_path):
    agg = tmp_path / "agg.csv"
    km.write_outputs(_row(), tmp_path / "run1", agg)
    km.write_outputs(_row(), tmp_path / "run2", agg)
    one = (tmp_path / "run1" / "kin_metrics.json").read_text()
    assert one.count("\n") == 1 and json.loads(one)["k"] == 2
    lines = (tmp_path / "run1" / "kin_metrics.csv").read_text().splitlines()
    assert lines[0].split(",") == km.KIN_COLUMNS and len(lines) == 2
    alines = agg.read_text().splitlines()
    assert len(alines) == 3 and alines[0] == lines[0]                    # un solo header
    assert len(alines[1].split(",")) == len(km.KIN_COLUMNS)


def test_csv_keeps_precision_of_small_w_full():
    r = _row()
    r["w_full_contact"] = 0.000312345678
    cells = dict(zip(km.KIN_COLUMNS, km.csv_cells(r)))
    assert float(cells["w_full_contact"]) == pytest.approx(0.000312345678, rel=1e-7)
    assert cells["w_full_pred"] == "0.3" and cells["tag"] == "t1" and cells["k"] == "2"


def test_render_summary_prints_gap_only_with_prediction():
    with_pred = km.render_summary(_row())
    assert "scarto rel a AT_END" in with_pred and "+66.67%" in with_pred          # (0.5 - 0.3) / 0.3
    assert "+16.67%" not in with_pred                                           # niente scarto al contatto
    assert "non confrontare col planner" in with_pred
    r = _metrics()
    r.update(tag="t", k=0, theta_star_deg=0.0)
    assert "scarto rel" not in km.render_summary(r)


# ----------------------------------------------------------------- cablaggio di analyze()

def _patch_analyze(monkeypatch, meta_follow=None):
    t, q = _bag()
    gc = SimpleNamespace(goal_pose=SimpleNamespace(
        position=SimpleNamespace(x=P_TCP[0] + 0.001, y=P_TCP[1], z=P_TCP[2]),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(math.radians(15.0)), w=math.cos(math.radians(15.0)))))
    rep = {"instants": {"first_contact_s": T_CONTACT},
           "grasp": {"max_err_tgt_0_15s_mm": 1.25},
           "selection": {"k": 2, "theta_star_deg": 10.0, "min_joint_margin_deg": 15.0}}
    calls = {}

    def fake_report_analyze(bag_dir, elog, meta=None, *a, **k):
        calls["report"] = (bag_dir, elog)
        return rep

    class FakeKin:
        def __init__(self, urdf):
            calls["urdf"] = urdf

        def jacobian(self, q7):
            return _jac_fn(q7)

        def pose(self, q7):
            return _pose_fn(q7)

    monkeypatch.setattr(km.s1_report, "parse_executor_log",
                        lambda p: {"t_trigger": T_TRIG, "t_at_end": T_END, "t_close_start": None, "t_close_done": None})
    monkeypatch.setattr(km.s1_report, "analyze", fake_report_analyze)
    monkeypatch.setattr(km.s1_report, "read_bag", lambda d: {"js_t": t, "js_q": q, "grasp_cmd": gc})
    monkeypatch.setattr(km, "parse_urdf_limits", lambda u: (-10 * np.ones(7), 10 * np.ones(7), VEL_LIM))
    monkeypatch.setattr(km, "JacobianKin", FakeKin)
    return calls


def test_analyze_wiring_uses_report_contact_and_error(monkeypatch):
    calls = _patch_analyze(monkeypatch)
    row = km.analyze("bagdir", "exec.log", {"tag": "r"}, urdf="u.urdf", predicted_w_full=0.30)
    assert calls["report"] == ("bagdir", "exec.log") and calls["urdf"] == "u.urdf"
    assert list(row) == km.KIN_COLUMNS
    assert (row["tag"], row["k"], row["theta_star_deg"]) == ("r", 2, 10.0)
    assert row["w_full_contact"] == pytest.approx(0.35, rel=1e-9)            # istante = first_contact_s di s1_report
    assert row["max_err_tgt_0_15s_mm"] == 1.25 and row["margin_pred_deg"] == 15.0
    assert row["w_full_follow_end"] == pytest.approx(1.1, rel=1e-9)          # follow_s assente -> 3.0 come s1_report
    assert row["w_full_end"] == pytest.approx(0.5, rel=1e-9) and row["w_full_end_rel_err"] == pytest.approx(2.0 / 3.0)
    assert row["q_end_j7"] == pytest.approx(3.5, rel=1e-9)
    assert row["pose_err_end_pos_mm"] == pytest.approx(1.0, rel=1e-6)        # goal dal GraspCommand del bag
    assert row["pose_err_end_ori_deg"] == pytest.approx(20.0, rel=1e-6)      # TCP Rz(10) vs goal Rz(30) (quaternione a 15 deg di semi-angolo)


def test_analyze_follow_s_from_meta(monkeypatch):
    _patch_analyze(monkeypatch)
    row = km.analyze("b", "e", {"tag": "r", "follow_s": 1.0}, urdf="u")
    assert row["w_full_follow_end"] == pytest.approx(0.7, rel=1e-9)


# ----------------------------------------------------------------- JacobianKin (pinocchio)

try:
    import pinocchio  # noqa: F401
    HAVE_PIN = True
except ImportError:
    HAVE_PIN = False
needs_urdf = pytest.mark.skipif(not (HAVE_PIN and URDF.exists()),
                                reason="pinocchio o fer_flat_effort.urdf assenti (fuori dal container)")
Q_READY = np.array([0.0, -math.pi / 4, 0.0, -3 * math.pi / 4, 0.0, math.pi / 2, math.pi / 4])


@needs_urdf
def test_jacobian_linear_block_matches_tcp_finite_difference():
    kin = km.JacobianKin(URDF)
    J = kin.jacobian(Q_READY)
    assert J.shape == (6, 7)
    eps = 1e-6
    fd = np.zeros((3, 7))
    for j in range(7):
        dq = np.zeros(7)
        dq[j] = eps
        fd[:, j] = (kin.tcp_position(Q_READY + dq) - kin.tcp_position(Q_READY - dq)) / (2 * eps)
    assert J[:3] == pytest.approx(fd, abs=1e-6)


@needs_urdf
def test_tcp_is_hand_plus_0p1034_along_hand_z_and_w_full_matches_det():
    kin = km.JacobianKin(URDF)
    kin.tcp_position(Q_READY)
    hand = kin.data.oMf[kin.model.getFrameId("fer_hand")]
    tcp = kin.data.oMf[kin.frame_id]
    assert tcp.translation == pytest.approx(hand.translation + hand.rotation @ np.array([0, 0, 0.1034]), abs=1e-9)
    assert tcp.rotation == pytest.approx(hand.rotation, abs=1e-9)
    J = kin.jacobian(Q_READY)
    w = km.full_manipulability(J)
    assert w > 0.0 and w == pytest.approx(math.sqrt(np.linalg.det(J @ J.T)), rel=1e-6)


@needs_urdf
def test_pose_is_consistent_with_tcp_position_and_rotation_is_orthonormal():
    kin = km.JacobianKin(URDF)
    p, R = kin.pose(Q_READY)
    assert p == pytest.approx(kin.tcp_position(Q_READY), abs=1e-12)
    assert R.T @ R == pytest.approx(np.eye(3), abs=1e-9) and np.linalg.det(R) == pytest.approx(1.0, abs=1e-9)
    assert km.pose_error(p, R, p, R) == pytest.approx((0.0, 0.0), abs=1e-6)
