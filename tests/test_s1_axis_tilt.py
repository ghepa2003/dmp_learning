"""pytest: asse di rotazione inclinato in tools/s1_run.py e tools/s1_report.py (B2).

Identita' a alpha = 0: i comandi sono confrontati con tests/data/s1_run_alpha0_commands.json (generato dal
runner PRIMA della modifica; il path di THESIS_WS e' sostituito da <WS>) e il report con i report.txt
esistenti in results/ (test saltati se mancano rclpy/pinocchio o le cartelle)."""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import s1_report  # noqa: E402
import s1_run  # noqa: E402

PATHS = {"weights": "/w/weights.yaml", "demo_params": "/w/demo.yaml", "bag": Path("/tmp/b")}
GOLDEN = ROOT / "tests" / "data" / "s1_run_alpha0_commands.json"
ALPHAS = [0.0, 0.5, 5.0, 10.0, 15.0, 19.99, 20.0]
PHIS = list(range(0, 360, 15)) + [359.999]
YAWS = [-180.0 + 1e-9, -135.0, -73.1, -1.0, 0.0, 33.3, 90.0, 179.0, 180.0]


def rz(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def ry(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])


def rx(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


def parse(argv):
    return s1_run.make_parser().parse_args(["--weights-dir", "w"] + argv)


# ------------------------------------------------------------------ geometria


def test_axis_formula_known_values():
    assert np.allclose(s1_run.tilt_axis(0.0, 0.0), [0, 0, 1], atol=0, rtol=0)
    assert np.allclose(s1_run.tilt_axis(0.0, 123.0), [0, 0, 1], atol=1e-15)
    a = s1_run.tilt_axis(20.0, -90.0 % 360.0)   # phi = -90 (G22 caso B, beta = 0)
    assert np.allclose(a, [0.0, -0.342020143, 0.939692621], atol=1e-9)
    assert np.allclose(s1_run.tilt_axis(20.0, -90.0), [0.0, -0.342020143, 0.939692621], atol=1e-9)


@pytest.mark.parametrize("alpha", ALPHAS)
@pytest.mark.parametrize("phi", PHIS)
def test_axis_unit_and_rtilt_z_is_axis(alpha, phi):
    a = s1_run.tilt_axis(alpha, phi)
    R = s1_run.tilt_rotation(alpha, phi)
    assert abs(np.linalg.norm(a) - 1.0) < 1e-15
    assert np.abs(R @ [0, 0, 1.0] - a).max() < 1e-15
    assert np.abs(R @ R.T - np.eye(3)).max() < 1e-15 and abs(np.linalg.det(R) - 1) < 1e-15
    # il quaternione e' la stessa rotazione
    assert np.abs(s1_run.quat_xyzw_to_matrix(s1_run.tilt_quat_xyzw(alpha, phi)) - R).max() < 1e-15


def test_phi_beta_relation_g22():
    # asse di inclinazione n_beta = (cos b, sin b, 0) <=> phi = beta - 90 deg; G22 caso B: beta = 0
    beta = 0.0
    phi = beta - 90.0
    n = s1_run.rot_axis_angle([0, 0, 1], 0) @ [math.cos(math.radians(beta)), math.sin(math.radians(beta)), 0]
    R_beta = s1_run.rot_axis_angle(n, math.radians(20.0))
    assert np.abs(R_beta - s1_run.tilt_rotation(20.0, phi)).max() < 1e-15


@pytest.mark.parametrize("alpha", ALPHAS)
@pytest.mark.parametrize("phi", PHIS)
def test_spawn_rpy_roundtrip(alpha, phi):
    """R_spawn = R_tilt.Rz(yaw) -> (roll, pitch, yaw) estrinseci -> Rz(y)Ry(p)Rx(r): errore < 1e-12."""
    R_tilt = s1_run.tilt_rotation(alpha, phi)
    for yaw_deg in YAWS:
        R_spawn = R_tilt @ rz(math.radians(yaw_deg))
        r, p, y = s1_run.rpy_extrinsic(R_spawn)
        assert np.abs(rz(y) @ ry(p) @ rx(r) - R_spawn).max() < 1e-12
        assert np.abs(s1_run.rpy_extrinsic_to_matrix(r, p, y) - R_spawn).max() < 1e-12
        if alpha > 0:   # i valori emessi (stringhe) fanno lo stesso giro
            tv = s1_run.tilt_values(alpha, phi, yaw_deg)
            r, p, y = (float(x) for x in tv["spawn_rpy_str"])
            assert np.abs(rz(y) @ ry(p) @ rx(r) - R_spawn).max() < 1e-12


@pytest.mark.parametrize("alpha", [a for a in ALPHAS if a > 0])
@pytest.mark.parametrize("phi", PHIS)
def test_emitted_normal_orthogonal_to_axis(alpha, phi):
    tv = s1_run.tilt_values(alpha, phi, 12.0)
    n, a = s1_run.parse_vec(tv["face_normal_str"]), s1_run.parse_vec(tv["axis_str"])
    assert abs(float(n @ a)) < 1e-12
    assert abs(np.linalg.norm(n) - 1) < 1e-12 and abs(np.linalg.norm(a) - 1) < 1e-12


def test_float_str_full_precision_and_yaml_float():
    assert float(s1_run.float_str(0.1 + 0.2)) == 0.1 + 0.2
    assert s1_run.float_str(1e-17) == "1.0e-17"
    assert s1_run.float_str(-6.28e-17) == "-6.28e-17"
    assert s1_run.float_str(0.0) == "0.0"


def test_tilt_none_at_zero_and_range_errors():
    assert s1_run.tilt_values(0.0, 0.0, 10.0) is None
    for bad in [(-0.1, 0.0), (20.01, 0.0), (float("nan"), 0.0), (5.0, 360.0), (5.0, -1.0), (5.0, float("inf"))]:
        with pytest.raises(s1_run.RunError):
            s1_run.validate_tilt(*bad)


# ------------------------------------------------------------------ identita' a alpha = 0


def test_alpha0_commands_identical_to_pre_change_runner():
    golden = json.loads(GOLDEN.read_text())
    assert golden
    for name, case in golden.items():
        a = parse(case["argv"])
        a.planner_param = s1_run.arm_planner_params(a.arm) + list(a.planner_param)
        cmds = s1_run.build_commands(a, PATHS)
        got = {k: v.replace(str(s1_run.WS_DIR), "<WS>") for k, v in cmds.items()}
        assert got == case["cmds"], name


def test_alpha0_emits_no_new_parameters():
    a = parse([])
    assert a.axis_tilt_deg == 0.0 and a.axis_lean_deg == 0.0
    cmds = s1_run.build_commands(a, PATHS)
    for k in ("executor", "planner", "spawn"):
        for word in ("satellite_rotation_axis", "satellite_q_ref", "axis_world", "face_normal_body", "roll:=", "pitch:="):
            assert word not in cmds[k], (k, word)
    # anche con phi != 0 ma alpha = 0 (phi non ha effetto)
    b = parse(["--axis-lean-deg", "77"])
    assert s1_run.build_commands(b, PATHS) == cmds


def test_alpha0_param_lists_equal_command_params():
    a = parse(["--arm", "v2", "--squeeze", "0.01"])
    a.planner_param = s1_run.arm_planner_params(a.arm) + list(a.planner_param)
    cmds = s1_run.build_commands(a, PATHS)
    assert s1_run.ros_args(s1_run.exec_param_list(a, PATHS)) in cmds["executor"]
    assert s1_run.ros_args(s1_run.planner_param_list(a, PATHS)) in cmds["planner"]


# ------------------------------------------------------------------ alpha > 0: comandi


def test_tilted_commands_consistent():
    a = parse(["--axis-tilt-deg", "20", "--axis-lean-deg", "270", "--spawn-yaw-deg", "-73.1", "--omega-deg-s", "-5"])
    cmds = s1_run.build_commands(a, PATHS)   # chiama check_axis_consistent
    tv = s1_run.args_tilt(a)
    assert "roll:=" in cmds["spawn"] and "pitch:=" in cmds["spawn"] and "wz:=-0.087266463" in cmds["spawn"]
    assert "wx:=" not in cmds["spawn"] and "wy:=" not in cmds["spawn"]
    for key, val in [("satellite_rotation_axis", tv["axis_str"]), ("satellite_q_ref", tv["q_ref_str"])]:
        assert f"{key}:={val}" in cmds["executor"]
    for key, val in [("satellite_rotation_axis", tv["axis_str"]), ("satellite_q_ref", tv["q_ref_str"]),
                     ("cube_geometry.axis_world", tv["axis_str"]),
                     ("cube_geometry.face_normal_body", tv["face_normal_str"])]:
        assert f"grasp_planner_node:{key}:={val}" in cmds["planner"] and f"-p {key}:={val}" in cmds["planner"].replace("'", "")
    # il resto dei parametri e' quello di alpha = 0
    base = s1_run.build_commands(parse(["--spawn-yaw-deg", "-73.1", "--omega-deg-s", "-5"]), PATHS)
    assert base["bringup"] == cmds["bringup"] and base["bag"] == cmds["bag"]


def test_tilted_params_cannot_be_overridden():
    a = parse(["--axis-tilt-deg", "5", "--planner-param", "satellite_q_ref=[0,0,0,1]"])
    with pytest.raises(s1_run.RunError):
        s1_run.build_commands(a, PATHS)
    b = parse(["--axis-tilt-deg", "5", "--extra-exec-param", "satellite_rotation_axis:=[0,0,1]"])
    with pytest.raises(s1_run.RunError):
        s1_run.build_commands(b, PATHS)
    # a alpha = 0 nessun vincolo (comportamento di oggi)
    s1_run.build_commands(parse(["--planner-param", "satellite_q_ref=[0,0,0,1]"]), PATHS)


# ------------------------------------------------------------------ check_axis_consistent


def _triple(alpha=15.0, phi=200.0, yaw=-40.0):
    tv = s1_run.tilt_values(alpha, phi, yaw)
    return (dict(rpy=tv["spawn_rpy_str"], yaw_rad=tv["yaw_rad"]),
            dict(axis=tv["axis_str"], q_ref=tv["q_ref_str"]),
            dict(axis=tv["axis_str"], q_ref=tv["q_ref_str"], axis_world=tv["axis_str"],
                 face_normal=tv["face_normal_str"]), tv)


def test_check_axis_consistent_ok():
    sp, ex, pl, _ = _triple()
    s1_run.check_axis_consistent(sp, ex, pl)


@pytest.mark.parametrize("which", ["spawn_rpy", "spawn_yaw", "exec_axis", "exec_q", "plan_axis", "plan_q",
                                   "plan_axis_world", "plan_normal"])
def test_check_axis_consistent_one_different(which):
    sp, ex, pl, _ = _triple()
    other = s1_run.tilt_values(15.0 + 0.01, 200.0, -40.0)   # 0.01 deg di tilt: piccolo ma >> 1e-9
    if which == "spawn_rpy":
        sp["rpy"] = other["spawn_rpy_str"]
    elif which == "spawn_yaw":
        sp["yaw_rad"] += 1e-4
    elif which == "exec_axis":
        ex["axis"] = other["axis_str"]
    elif which == "exec_q":
        ex["q_ref"] = other["q_ref_str"]
    elif which == "plan_axis":
        pl["axis"] = other["axis_str"]
    elif which == "plan_q":
        pl["q_ref"] = other["q_ref_str"]
    elif which == "plan_axis_world":
        pl["axis_world"] = other["axis_str"]
    elif which == "plan_normal":
        pl["face_normal"] = other["face_normal_str"]
    with pytest.raises(s1_run.RunError):
        s1_run.check_axis_consistent(sp, ex, pl)


# ------------------------------------------------------------------ twist, goal


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def _qrot(axis, angle):
    u = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    return (*(math.sin(angle / 2) * u), math.cos(angle / 2))


# omega da quaternioni (il twist.angular dell'odometria Gazebo e' d(rpy)/dt, non omega.a: G28)
RATE_TOL = 0.00174533   # rad/s, come omega_consistency_tol del planner


@pytest.mark.parametrize("alpha,phi", [(20.0, 0.0), (20.0, 270.0), (7.0, 33.0), (20.0, 359.0)])
def test_axis_rate_tilted_from_quaternions(alpha, phi):
    a = s1_run.tilt_axis(alpha, phi)
    om, dt = math.radians(-2.0), 0.005
    q0 = _qrot([0.3, -0.5, 0.2], 0.7)
    q1 = _qmul(_qrot(a, om * dt), q0)    # incremento WORLD attorno ad a
    assert abs(s1_run.axis_rate(q0, q1, dt, a) - om) < RATE_TOL
    assert abs(s1_run.axis_rate(q0, q1, dt, a) - om) < 1e-9


def test_axis_rate_regression_g28_twist_is_rpy_derivative():
    a = s1_run.tilt_axis(20.0, 0.0)
    om, dt = math.radians(-2.0), 0.005
    twist = np.array([-7.34e-5, -0.011939, -0.032802])      # G28 config B, regime
    assert abs(float(twist @ a) - (-0.030849)) < 1e-6        # vecchia proiezione
    assert abs(float(twist @ a) - om) > RATE_TOL             # ... fallirebbe
    q0 = _qrot([1.0, 0.0, 0.0], -0.349)
    assert abs(s1_run.axis_rate(q0, _qmul(_qrot(a, om * dt), q0), dt, a) - om) < RATE_TOL


def test_axis_rate_vertical_axis_same_as_wz():
    dt = 0.01
    for w in (-0.0349066, 0.0349066, 0.5, -1.2):
        q0, q1 = _qrot([0, 0, 1], 0.4), _qrot([0, 0, 1], 0.4 + w * dt)
        assert abs(s1_run.axis_rate(q0, q1, dt, None) - w) < 1e-12
        assert s1_run.axis_rate(q0, q1, dt, None) == pytest.approx(s1_run.axis_rate(q0, q1, dt, [0, 0, 1]), abs=1e-15)


def test_axis_rate_edge_cases():
    a = s1_run.tilt_axis(20.0, 33.0)
    om, dt = math.radians(-2.0), 0.005
    q0 = _qrot([0.1, 0.2, 0.3], 1.0)
    q1 = _qmul(_qrot(a, om * dt), q0)
    for bad_dt in (0.0, -dt, float("nan"), float("inf")):
        assert s1_run.axis_rate(q0, q1, bad_dt, a) is None
    neg = lambda q: tuple(-c for c in q)
    assert abs(s1_run.axis_rate(q0, neg(q1), dt, a) - om) < 1e-9      # segno opposto (doppia copertura)
    assert abs(s1_run.axis_rate(neg(q0), q1, dt, a) - om) < 1e-9
    assert abs(s1_run.axis_rate(tuple(0.25 * c for c in q0), tuple(3.0 * c for c in q1), dt, a) - om) < 1e-9
    assert s1_run.axis_rate((0, 0, 0, 0), q1, dt, a) is None
    assert s1_run.axis_rate(q0, (float("nan"), 0, 0, 1), dt, a) is None
    p0, p1 = _qrot(a, math.pi - 0.5 * abs(om) * dt), _qrot(a, math.pi + 0.5 * abs(om) * dt)   # wrap a pi
    assert abs(s1_run.axis_rate(p0, p1, dt, a) - abs(om)) < 1e-9
    assert abs(s1_run.axis_rate(p1, p0, dt, a) + abs(om)) < 1e-9
    assert abs(s1_run.axis_rate(q0, q0, dt, a)) < 1e-12
    with pytest.raises(ValueError):
        s1_run.axis_rate(q0, q1, dt, [0, 0, 0])


def _legacy_goal_x_c(goal_xy, th, center):
    dx, dy = goal_xy[0] - center[0], goal_xy[1] - center[1]
    return (dx * math.cos(th) + dy * math.sin(th)) * 1000.0


def test_goal_x_c_alpha0_identical_to_historic_formula():
    rng = np.random.default_rng(0)
    for _ in range(50):
        g = (rng.uniform(0.4, 1.0), rng.uniform(-0.3, 0.3))
        th = rng.uniform(-math.pi, math.pi)
        assert s1_run.goal_x_c_mm(g, th) == _legacy_goal_x_c(g, th, s1_run.SAT_CENTER)
        c = (0.65, 0.1, 0.8)
        assert s1_run.goal_x_c_mm(g, th, center=c) == _legacy_goal_x_c(g, th, c)
        # 3 componenti con asse None: la z e' ignorata come prima
        assert s1_run.goal_x_c_mm(g + (0.9,), th, center=c) == _legacy_goal_x_c(g, th, c)


def test_goal_x_c_tilted_projects_on_rotated_normal():
    alpha, phi, c = 20.0, 270.0, np.array(s1_run.SAT_CENTER)
    a, R = s1_run.tilt_axis(alpha, phi), s1_run.tilt_rotation(alpha, phi)
    for th in np.radians([-163.5, -73.1, 0.0, 45.0, 170.0]):
        n_w = s1_run.rot_axis_angle(a, th) @ (R @ [1.0, 0.0, 0.0])
        goal = c + 0.1551 * n_w + 0.02 * np.cross(a, n_w)   # componente tangenziale: non conta
        xc = s1_run.goal_x_c_mm(tuple(goal), th, center=tuple(c), axis=a, face_normal_ref=R[:, 0])
        assert abs(xc - 155.1) < 1e-6
    with pytest.raises(s1_run.RunError):   # il goal 2D non basta
        s1_run.goal_x_c_mm((0.8, 0.0), 0.0, axis=a, face_normal_ref=R[:, 0])
    # asse z e normale x: coincide con la formula storica (goal planare)
    g = (0.8, 0.05, c[2])
    th = 0.7
    assert abs(s1_run.goal_x_c_mm(g, th, axis=np.array([0, 0, 1.0]), face_normal_ref=[1, 0, 0])
               - _legacy_goal_x_c(g, th, s1_run.SAT_CENTER)) < 1e-9


def test_goal_from_executor_log_with_z():
    txt = "goal=[0.812,-0.034,0.771] ... theta_star=-163.5 deg"
    assert s1_run.goal_from_executor_log(txt) == ((0.812, -0.034), math.radians(-163.5))
    assert s1_run.goal_from_executor_log(txt, with_z=True) == ((0.812, -0.034, 0.771), math.radians(-163.5))


# ------------------------------------------------------------------ meta


def test_tilt_meta_and_git_state():
    m0 = s1_run.tilt_meta(parse([]))
    assert m0 == {"axis_tilt_deg": 0.0, "axis_lean_deg": 0.0, "axis": [0.0, 0.0, 1.0], "q_ref": [0.0, 0.0, 0.0, 1.0]}
    m = s1_run.tilt_meta(parse(["--axis-tilt-deg", "20", "--axis-lean-deg", "270"]))
    assert np.allclose(m["axis"], [0, -0.342020143, 0.939692621], atol=1e-9) and len(m["q_ref"]) == 4
    g = s1_run.git_state(s1_run.WS_DIR)
    assert set(g) == {"git_head", "git_dirty", "git_dirty_files"}
    if g["git_head"] is not None:
        assert len(g["git_head"]) == 40 and isinstance(g["git_dirty"], bool)
        assert not any(ln.startswith("??") for ln in g["git_dirty_files"])   # solo file tracciati
    assert s1_run.git_state("/nonexistent_dir_xyz") == {"git_head": None, "git_dirty": None, "git_dirty_files": None}


# ------------------------------------------------------------------ report: angolo attorno all'asse


def test_report_angles_legacy_branch_is_arctan2():
    rng = np.random.default_rng(1)
    R = np.array([s1_run.rot_axis_angle(rng.normal(size=3), rng.uniform(-3, 3)) for _ in range(40)])
    assert np.array_equal(s1_report.cube_angles(R, None), np.arctan2(R[:, 1, 0], R[:, 0, 0]))
    assert s1_report.tilt_from_meta({}) is None
    assert s1_report.tilt_from_meta({"axis": [0.0, 0.0, 1.0], "q_ref": [0.0, 0.0, 0.0, 1.0]}) is None
    assert s1_report.tilt_from_meta({"axis": [0, 0, 1]}) is None


@pytest.mark.parametrize("alpha,phi", [(20.0, 270.0), (12.0, 31.0), (3.0, 200.0)])
def test_report_angle_about_tilted_axis(alpha, phi):
    a, R_t = s1_run.tilt_axis(alpha, phi), s1_run.tilt_rotation(alpha, phi)
    meta = {"axis": list(a), "q_ref": list(s1_run.tilt_quat_xyzw(alpha, phi))}
    tilt = s1_report.tilt_from_meta(meta)
    assert tilt is not None
    th = np.radians(np.arange(-179.0, 180.0, 7.0))
    oR = np.array([s1_run.rot_axis_angle(a, t) @ R_t for t in th])   # R(t) = Rot(a, theta) R_tilt (= R_tilt Rz(theta))
    assert np.abs(oR[0] - R_t @ rz(th[0])).max() < 1e-12
    got = s1_report.cube_angles(oR, tilt)
    assert np.abs(np.angle(np.exp(1j * (got - th)))).max() < 1e-12
    # il vecchio yaw e' sbagliato (a 20 gradi di tilt di molti gradi, a 3 di ~0.04)
    old = np.arctan2(oR[:, 1, 0], oR[:, 0, 0])
    assert np.abs(np.angle(np.exp(1j * (old - th)))).max() > 1e-4


# ------------------------------------------------------------------ report su cartelle esistenti (container)

REF_DIRS = ["margin2_w-5.0", "clr_w-3.0", "forced_k1A_th-156"]


@pytest.mark.parametrize("name", REF_DIRS)
def test_report_alpha0_identical_on_existing_runs(name):
    pytest.importorskip("rclpy")
    pytest.importorskip("pinocchio")
    d = ROOT / "results" / name
    if not (d / "bag").is_dir() or not (d / "report.txt").exists():
        pytest.skip(f"{d} non disponibile")
    meta = json.loads((d / "run_meta.json").read_text())
    assert "axis" not in meta   # run storici: asse z / q_ref identita'
    res = s1_report.analyze(d / "bag", d / "executor.log", meta, planner_log=d / "planner.log")
    new, old = s1_report.render_txt(res).splitlines(), (d / "report.txt").read_text().splitlines()

    # dal 02/11/2026 il footer del successo e' cambiato (success = criteri 1-10, criterio 11 a parte): i report salvati
    # hanno la vecchia riga unica "SUCCESSO: SI|NO" (11 criteri). Tutte le altre righe devono restare identiche.
    def footer(lines):
        return [ln for ln in lines if ln.startswith("SUCCESSO")]

    def body(lines):
        return [ln for ln in lines if not ln.startswith("SUCCESSO")]

    assert body(new) == body(old)
    assert new[-1].startswith("SUCCESSO") and old[-1].startswith("SUCCESSO")   # il footer e' l'ultima riga in entrambi
    (new_f,), (old_f,) = footer(new), footer(old)
    assert old_f in ("SUCCESSO: SI", "SUCCESSO: NO")
    # relazione col vecchio esito (a 11 criteri): coincide con "SUCCESSO 1-11"; "criteri 1-10" puo' solo essere piu' largo
    if old_f == "SUCCESSO: SI":
        assert "SUCCESSO (criteri 1-10): SI" in new_f and "SUCCESSO 1-11: SI" in new_f
    else:
        assert "SUCCESSO 1-11: NO" in new_f
