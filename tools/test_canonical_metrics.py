#!/usr/bin/env python3
"""
Test di tools/canonical_metrics.py.

I quattro test `test_first_*` (numeri citati nella tesi) stanno in cima e vanno eseguiti PER
PRIMI: se uno fallisce NON si ritoccano le definizioni per farlo passare, si riporta il fallimento.

Esecuzione:
  pytest -x tools/test_canonical_metrics.py          # dove pytest e' installato
  python3 tools/test_canonical_metrics.py            # runner incorporato (fail-fast, ordine del file)

Le funzioni usano solo `assert`: nessuna dipendenza da pytest.
"""
from __future__ import annotations

import math
import sys
import tempfile
import traceback
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import canonical_metrics as cm  # noqa: E402

_CACHE: dict = {}


def _metrics():
    if "df" not in _CACHE:
        _CACHE["df"] = cm.compute_all(cm.DEFAULT_MANIFEST)
    return _CACHE["df"]


def _reg_row(variant: str, frame: str, instant: str, excluded: str = "none"):
    tab = cm.regression_table(_metrics())
    sel = tab[(tab.excluded_group == excluded) & (tab.axis_variant == variant)
              & (tab.frame == frame) & (tab.w_instant == instant)]
    assert len(sel) == 1
    return sel.iloc[0]


def _close(actual: float, cited: float, abs_tol: float, what: str):
    assert abs(actual - cited) <= abs_tol, f"{what}: calcolato {actual!r}, citato {cited!r}"


# ======================================================================================
# TEST DA ESEGUIRE PER PRIMI (numeri citati)
# ======================================================================================
def test_first_regression_as_scripts_link8_pre1s():
    row = _reg_row("as_scripts", "link8", "pre1s_mean")
    assert int(row.n) == 10
    _close(row.intercept, 9.6186, 6e-5, "intercetta")
    _close(row.slope, -48.5947, 6e-5, "pendenza")
    _close(row.r, -0.8670, 6e-5, "r")
    _close(row.r2, 0.7517, 6e-5, "R2")


def test_first_regression_aligned_link8_pre1s():
    row = _reg_row("aligned", "link8", "pre1s_mean")
    assert int(row.n) == 10
    _close(row.intercept, 9.0700, 6e-5, "intercetta")
    _close(row.slope, -43.3076, 6e-5, "pendenza")
    _close(row.r, -0.7967, 6e-5, "r")


def test_first_w_final_section_9_4():
    df = _metrics().set_index("run_id")
    cited = {
        "reach_task_satellite_rot_phase{P}_kt200": (0.1244, 0.1444, 0.1206, 0.0580),
        "reach_task_satellite_rot_phase{P}": (0.1234, 0.1446, 0.1208, 0.0585),
    }
    for pattern, values in cited.items():
        for phase, v in zip((30, 90, 180, 270), values):
            got = df.loc[pattern.format(P=phase), "w_link8_last_row_as_scripts"]
            assert abs(got - v) / v <= 1e-3, f"{pattern.format(P=phase)}: w_final {got!r} vs {v!r}"


def test_first_section_6_4():
    df = _metrics().set_index("run_id")
    # (run, media, max, ultimo, @tau) in mm, citati con 2 decimali
    cases = (("reach_task_baseline_velocity_delay1", 4.32, 7.74, 0.10, 5.57),
             ("reach_task_baseline_replay_prodmp_kt2000_delay1", 0.57, 5.86, 0.19, 0.46))
    for run, mean, mx, last, tau in cases:
        r = df.loc[run]
        got = (r.err_pos_mean_all_mm, r.err_pos_max_mm, r.err_pos_last_mm,
               r.err_pos_paired_at_t_settle_1e4_mm)
        assert tuple(round(float(v), 2) for v in got) == (mean, mx, last, tau), \
            f"{run}: calcolati {got!r}, citati {(mean, mx, last, tau)!r}"


# ======================================================================================
# Cinematica
# ======================================================================================
def test_jacobian_matches_finite_differences():
    kin = cm.PandaKinematics()
    rng = np.random.default_rng(12345)
    configs = [np.array([0.0, -0.786, 0.0, -2.372, 0.0, 1.571, 0.785]),
               np.array([0.3, -0.4, -0.2, -1.8, 0.5, 2.0, -0.6]),
               np.array([-1.0, 0.7, 1.2, -2.5, -0.9, 1.1, 0.3])]
    configs += [rng.uniform(-1.2, 1.2, 7) for _ in range(2)]
    assert len(configs) == 5
    h = 1e-6
    for q in configs:
        for ee in cm.FRAMES:
            jv = kin.jacobian_v(q, ee)
            fd = np.empty((3, 7))
            for i in range(7):
                dq = np.zeros(7)
                dq[i] = h
                fd[:, i] = (kin.position(q + dq, ee) - kin.position(q - dq, ee)) / (2 * h)
            assert np.max(np.abs(jv - fd)) < 1e-6, f"{ee}, q={q}: max |Jv-FD| = {np.max(np.abs(jv - fd))}"
            assert abs(kin.w_trans(q, ee) - math.prod(np.linalg.svd(jv, compute_uv=False))) < 1e-12


def test_tcp_offset_is_0_1034_along_tool_z():
    kin = cm.PandaKinematics()
    q = np.array([0.3, -0.4, -0.2, -1.8, 0.5, 2.0, -0.6])
    R8, p8, _, _ = kin._chain(q)
    d = kin.position(q, "tcp") - kin.position(q, "link8")
    assert abs(np.linalg.norm(d) - 0.1034) < 1e-12
    assert np.max(np.abs(d - R8[:, 2] * 0.1034)) < 1e-12


def test_actual_pose_matches_fk_tcp():
    """Il frame dichiarato nel manifest e' coerente con i dati: ultima riga actual_pose == FK(TCP)."""
    import pandas as pd
    kin = cm.PandaKinematics()
    for e in cm.load_manifest(cm.DEFAULT_MANIFEST):
        js = pd.read_csv(e["paths"]["joint_states"])
        ap = pd.read_csv(e["paths"]["actual_pose"])
        q = js[cm.JOINT_NAMES].to_numpy()[-1]
        a = ap[["x", "y", "z"]].to_numpy()[-1]
        d_tcp = np.linalg.norm(kin.position(q, "tcp") - a) * 1000
        d_l8 = np.linalg.norm(kin.position(q, "link8") - a) * 1000
        expected = e["frame_at_recording"]
        assert expected == "tcp"
        assert d_tcp < 0.01 and d_l8 > 100.0, f"{e['run_id']}: d_tcp={d_tcp} mm, d_link8={d_l8} mm"


# ======================================================================================
# Definizioni su dati sintetici
# ======================================================================================
def test_settle_and_step_indices_synthetic():
    # target: 0..9 con passo 1e-3 m lungo x, poi fermo a x=0.009 con un ultimo passo di 5e-5
    x = np.concatenate([np.arange(10) * 1e-3, [0.009] * 5])
    p = np.stack([x, np.zeros_like(x), np.zeros_like(x)], axis=1)
    assert cm.settle_index(p) == 9        # riga 8 dista 1e-3 > 1e-4; da riga 9 in poi entro 1e-4
    assert cm.step_index(p) == 9          # ultimo passo >= 1e-5 e' 8->9
    p2 = p.copy()
    p2[9:, 0] += 5e-5                      # ultimo passo 5e-5, dentro 1e-4 ma sopra 1e-5
    assert cm.settle_index(p2) == 9
    assert cm.step_index(p2) == 9
    p3 = p.copy()
    p3[8, 0] = 0.009 - 5e-5                # riga 8 entro 1e-4 dal finale
    assert cm.settle_index(p3) == 8 and cm.step_index(p3) == 9   # passo 8->9 = 5e-5 >= 1e-5
    flat = np.zeros((5, 3))
    assert cm.settle_index(flat) == 0 and cm.step_index(flat) == 0


def test_index_at_is_fail_loud_beyond_end():
    t = np.array([0.0, 1.0, 2.0])
    assert cm._index_at(t, 1.5, "x") == 2
    try:
        cm._index_at(t, 2.5, "x")
    except ValueError:
        return
    raise AssertionError("atteso ValueError oltre l'ultimo istante")


def test_paired_error_uses_bisect_left_with_clip():
    t_t = np.array([0.0, 1.0, 2.0])
    t_p = np.array([[0.0, 0, 0], [1.0, 0, 0], [2.0, 0, 0]])
    a_t = np.array([0.5, 1.0, 5.0])
    a_p = np.array([[1.0, 0, 0], [1.0, 0, 0], [2.0, 0, 0]])
    err = cm.paired_error_mm(a_t, a_p, t_t, t_p)
    assert np.allclose(err, [0.0, 0.0, 0.0])   # 0.5 -> riga 1, 1.0 -> riga 1, 5.0 -> clip riga 2
    a_p2 = np.array([[1.0, 0, 0], [1.5, 0, 0], [2.0, 0, 0]])
    assert np.allclose(cm.paired_error_mm(a_t, a_p2, t_t, t_p), [0.0, 500.0, 0.0])


def test_linfit_and_logo_synthetic():
    x = np.arange(8.0)
    y = 3.0 - 2.0 * x
    b0, b1, r = cm.linfit(x, y)
    assert abs(b0 - 3.0) < 1e-12 and abs(b1 + 2.0) < 1e-12 and abs(r + 1.0) < 1e-12
    groups = np.array(list("AABBCCDD"))
    assert abs(cm.r2_leave_one_group_out(x, y, groups) - 1.0) < 1e-12


# ======================================================================================
# Manifest fail-loud
# ======================================================================================
def _manifest_text(entries: str) -> str:
    return ("schema_version: 1\ndata_dir: " + str(cm.TOOLS_DIR / "gazebo_cartesian_eval/data")
            + "\nruns:\n" + entries)


def _entry(run="reach_task_baseline_velocity_delay1", kt="null", src="not_applicable",
           family="baseline", extra=""):
    return (f"  - run_id: {run}\n    family: {family}\n    files:\n"
            f"      target_aligned: target_aligned_{run}.csv\n"
            f"      actual_pose: actual_pose_{run}.csv\n"
            f"      joint_states: joint_states_{run}.csv\n"
            f"    kt: {kt}\n    kt_source: {src}\n"
            f'    recorded_utc: "2026-09-14T15:08:52+00:00"\n    frame_at_recording: tcp\n{extra}')


def _expect_error(text: str, exc):
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "m.yaml"
        p.write_text(text)
        try:
            cm.load_manifest(p)
        except exc:
            return
    raise AssertionError(f"atteso {exc.__name__}")


def test_manifest_valid_minimal():
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "m.yaml"
        p.write_text(_manifest_text(_entry()))
        assert len(cm.load_manifest(p)) == 1


def test_manifest_duplicate_run_id_fails():
    _expect_error(_manifest_text(_entry() + _entry()), ValueError)


def test_manifest_missing_file_fails():
    _expect_error(_manifest_text(_entry(run="reach_task_run_che_non_esiste")), FileNotFoundError)


def test_manifest_file_name_must_match_run_id():
    e1 = _entry()
    e2 = _entry(run="reach_task_baseline_impedance_kt200_delay1", kt="null", src="unknown")
    e2 = e2.replace("actual_pose_reach_task_baseline_impedance_kt200_delay1.csv",
                    "actual_pose_reach_task_baseline_velocity_delay1.csv")
    # nome file != <ruolo>_<run_id>.csv: rifiutato (il controllo 'file usato da due run_id' resta come difesa in piu')
    _expect_error(_manifest_text(e1 + e2), ValueError)


def test_manifest_bad_kt_source_and_kt_consistency_fail():
    _expect_error(_manifest_text(_entry(src="boh")), ValueError)
    _expect_error(_manifest_text(_entry(kt="200", src="unknown")), ValueError)
    _expect_error(_manifest_text(_entry(kt="null", src="script_argument")), ValueError)


def test_manifest_unknown_key_and_missing_group_fail():
    _expect_error(_manifest_text(_entry(extra="    inventato: 1\n")), ValueError)
    _expect_error(_manifest_text(_entry(family="regression_goal")), ValueError)


def test_repo_manifest_has_expected_runs():
    ents = cm.load_manifest(cm.DEFAULT_MANIFEST)
    fam = [e["family"] for e in ents]
    assert fam.count("regression_goal") == 10 and fam.count("satellite_phase") == 8
    assert fam.count("baseline") == 5
    assert len({e["run_id"] for e in ents}) == len(ents)


# ======================================================================================
# Lettura header.stamp dai bag (bag_time_probe) e variante sim_time
# ======================================================================================
def _blob(sec: int, nsec: int, little: bool = True, pad: int = 8) -> bytes:
    import struct
    head = b"\x00\x01\x00\x00" if little else b"\x00\x00\x00\x00"
    return head + struct.pack("<iI" if little else ">iI", sec, nsec) + b"\xaa" * pad


def _make_bag(root: Path, topics: dict, little: bool = True) -> Path:
    """topics: {nome: [(recv_ns, sec, nsec), ...]}; crea metadata.yaml + un .db3 sqlite."""
    import sqlite3
    root.mkdir(parents=True)
    (root / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n  storage_identifier: sqlite3\n  relative_file_paths:\n    - b_0.db3\n")
    con = sqlite3.connect(root / "b_0.db3")
    con.execute("CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT, type TEXT, "
                "serialization_format TEXT, offered_qos_profiles TEXT)")
    con.execute("CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB)")
    for tid, (name, msgs) in enumerate(topics.items(), start=1):
        con.execute("INSERT INTO topics VALUES(?,?,?,?,?)", (tid, name, "t", "cdr", ""))
        for recv, sec, nsec in reversed(msgs):   # inserite in ordine inverso: la lettura deve ordinare
            con.execute("INSERT INTO messages(topic_id, timestamp, data) VALUES(?,?,?)",
                        (tid, recv, _blob(sec, nsec, little)))
    con.commit()
    con.close()
    return root


def test_bag_reader_decodes_cdr_stamps_synthetic():
    with tempfile.TemporaryDirectory() as d:
        for little in (True, False):
            bag_dir = _make_bag(Path(d) / f"bag_{little}",
                                {"/c/target_pose_aligned": [(1_000_000_000 + i * 2_000_000, 5, 100_000_000 + i * 1_000_000)
                                                           for i in range(4)],
                                 "/joint_states": [(5, 0, 7)]}, little=little)
            bag = cm.btp.read_bag(bag_dir)
            tg = bag["target_pose_aligned"]
            assert tg.stamp_verified and tg.stamp_class == "sim_like"
            assert tg.note == ("little-endian" if little else "big-endian")
            assert list(tg.stamp_ns) == [5_100_000_000 + i * 1_000_000 for i in range(4)]
            assert list(tg.recv_ns) == [1_000_000_000 + i * 2_000_000 for i in range(4)]
            assert set(bag) == {"target_pose_aligned", "joint_states"}


def test_bag_reader_marks_implausible_stamp_not_verified():
    with tempfile.TemporaryDirectory() as d:
        bag_dir = _make_bag(Path(d) / "bag", {"/joint_states": [(1, 3, 2_000_000_000), (2, 3, 5)]})
        js = cm.btp.read_bag(bag_dir)["joint_states"]
        assert not js.stamp_verified and js.stamp_class == "NON VERIFICATO"
        row = cm.btp.topic_stats(js)
        assert row["d_sim_s"] == "NON VERIFICATO" and row["rtf_overall"] == "NON VERIFICATO"
    ok, _, note = cm.btp._decode_prefixes([b"\x01\x01\x00\x00" + b"\x00" * 8])
    assert not ok and "CDR" in note
    ok, _, note = cm.btp._decode_prefixes([b"\x00\x01\x00\x00" + b"\x00" * 4])
    assert not ok


def test_topic_stats_synthetic_rtf_and_anomalies():
    n = 3001
    recv = (np.arange(n) * 2_000_000).astype(np.int64)                  # 2 ms di ricezione
    stamp = (10 ** 9 + np.arange(n) * 1_000_000).astype(np.int64)       # 1 ms sim => RTF 0.5 (base 1 s: nessun nullo naturale)
    stamp[100] = stamp[99] - 5_000_000                                  # 1 non monotono
    stamp[200] = 0                                                      # 1 nullo (e 1 non monotono)
    ts = cm.btp.TopicStamps("joint_states", "/joint_states", recv, stamp, True, "sim_like")
    r = cm.btp.topic_stats(ts)
    assert abs(r["rtf_overall"] - 0.5) < 1e-9 and r["n"] == n
    assert abs(r["msg_per_s_recv"] - 500.0) < 1e-6
    assert r["n_null"] == 1 and r["n_non_monotonic"] == 2                # una differenza negativa per anomalia
    assert abs(r["dt_sim_median_ms"] - 1.0) < 1e-9
    assert r["n_windows"] == 0                                           # 6.0 s di ricezione: nessuna finestra da 10 s
    n2 = 40001
    ts2 = cm.btp.TopicStamps("x", "/x", (np.arange(n2) * 1_000_000).astype(np.int64),
                             (np.arange(n2) * 3_000_000).astype(np.int64), True, "sim_like")
    r2 = cm.btp.topic_stats(ts2)   # 40 s di ricezione => 4 finestre complete da 10 s, RTF 3
    assert r2["n_windows"] == 4 and abs(r2["rtf_win_min"] - 3.0) < 1e-9 and abs(r2["rtf_win_max"] - 3.0) < 1e-9


def test_all_manifest_bags_have_verified_sim_stamps_and_match_csv():
    import pandas as pd
    for e in cm.load_manifest(cm.DEFAULT_MANIFEST):
        assert e["bag_path"] is not None, e["run_id"]
        bag = cm.btp.read_bag(e["bag_path"])
        assert {"clock", "target_pose", "target_twist"}.isdisjoint(bag), e["run_id"]
        for role in ("target_pose_aligned", "actual_pose", "joint_states"):
            assert bag[role].stamp_verified and bag[role].stamp_class == "sim_like", (e["run_id"], role)
        for role in ("target_pose_aligned", "actual_pose"):
            assert (np.diff(bag[role].stamp_ns) >= 0).all() and not (bag[role].stamp_ns == 0).any()
        csv_t = {r: pd.read_csv(e["paths"][r])["t"].to_numpy()
                 for r in ("target_aligned", "actual_pose", "joint_states")}
        cm.btp.verify_csv_alignment(bag, csv_t)   # solleva ValueError se CSV e bag non coincidono


def test_verify_csv_alignment_is_fail_loud():
    with tempfile.TemporaryDirectory() as d:
        msgs = [(1_000_000_000 + i * 1_000_000, 1, i * 1_000_000) for i in range(5)]
        bag = cm.btp.read_bag(_make_bag(Path(d) / "b", {"/c/target_pose_aligned": msgs,
                                                        "/c/actual_pose": msgs, "/joint_states": msgs}))
        t = np.arange(5) * 1e-3
        cm.btp.verify_csv_alignment(bag, {"target_aligned": t, "actual_pose": t, "joint_states": t})
        for bad in (t[:4], t + 1e-3):
            try:
                cm.btp.verify_csv_alignment(bag, {"target_aligned": bad})
            except ValueError:
                continue
            raise AssertionError("atteso ValueError")


def test_sim_time_columns_and_last_message_simultaneity():
    df = _metrics()
    for col in ("t_settle_1e4_sim_s", cm.Y_COL_SIM, "w_link8_pre1s_mean_sim_time",
                "w_tcp_pre2s_mean_sim_time", "w_link8_at_t_settle_sim_time", "w_tcp_last_row_sim_time",
                "js_stamp_n_non_monotonic"):
        assert col in df.columns, col
    # dato osservato: ultimo js e ultimo target sono entro 10 ms in sim time in tutti i run
    assert (df["sim_js_minus_target_last_s"].abs() < 0.01).all()
    # l'ultima riga non dipende dall'asse
    assert np.allclose(df["w_link8_last_row_sim_time"], df["w_link8_last_row_as_scripts"])


def test_regression_table_has_sim_time_rows():
    tab = cm.regression_table(_metrics())
    sim = tab[tab.axis_variant == "sim_time"]
    assert set(sim.frame) == {"link8", "tcp"} and set(sim.w_instant) == set(cm.SIM_W_INSTANTS)
    assert set(sim.y_col) == {cm.Y_COL_SIM}
    assert len(sim[sim.excluded_group == "none"]) == 8 and (sim[sim.excluded_group == "none"].n == 10).all()


def test_logo_is_leave_one_goal_out_not_leave_one_run():
    ents = cm.load_manifest(cm.DEFAULT_MANIFEST)
    groups = [e["group"] for e in ents if e["family"] == "regression_goal"]
    folds = cm.logo_folds(groups)
    assert sorted(len(v) for v in folds.values()) == [1, 1, 2, 3, 3]        # 5 fold, non 10
    assert {g: len(v) for g, v in folds.items()} == {"Goal 1": 2, "Goal 2": 3, "Goal 3": 1,
                                                       "Goal 4": 3, "Goal 5": 1}
    # sintetico: tutte le repliche di un gruppo vengono escluse insieme dal fit
    x = np.array([0., 1., 2., 3., 4., 5., 6., 7.])
    y = np.array([0., 1., 2., 3., 4., 5., 6., 40.])
    groups = np.array(list("AABBCCDD"))
    pred = np.empty_like(y)
    for g in "ABCD":
        held = groups == g
        b0, b1, _ = cm.linfit(x[~held], y[~held])
        pred[held] = b0 + b1 * x[held]
    expected = 1 - np.sum((y - pred) ** 2) / np.sum((y - y.mean()) ** 2)
    assert abs(cm.r2_leave_one_group_out(x, y, groups) - expected) < 1e-12
    # leave-one-run darebbe un valore diverso (le repliche resterebbero nel training)
    pred_run = np.empty_like(y)
    for i in range(len(x)):
        keep = np.arange(len(x)) != i
        b0, b1, _ = cm.linfit(x[keep], y[keep])
        pred_run[i] = b0 + b1 * x[i]
    loro = 1 - np.sum((y - pred_run) ** 2) / np.sum((y - y.mean()) ** 2)
    assert abs(loro - expected) > 1e-6


# ======================================================================================
def _run_all() -> int:
    tests = [(n, f) for n, f in globals().items() if n.startswith("test_") and callable(f)]
    for name, fn in tests:
        try:
            fn()
        except Exception:
            print(f"FAIL {name}")
            traceback.print_exc()
            print("Fermato al primo fallimento (i test non vanno 'aggiustati' ritoccando le definizioni).")
            return 1
        print(f"ok   {name}")
    print(f"{len(tests)} test passati")
    return 0


if __name__ == "__main__":
    sys.exit(_run_all())
