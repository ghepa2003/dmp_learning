"""Tests of tools/s3_landscape_report.py on synthetic landscapes with a known answer (no ROS, numpy only)."""
import math
import sys
from pathlib import Path

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "tools"))
import s3_landscape_report as R  # noqa: E402

NAN = float("nan")


def row(k=0, theta=0.0, psi=0.0, d=10.0, w=1.0, accepted=1, mask=0, jcur=None, twait=100.0, mfollow=0.5, mroll=0.6,
        w_trans=1.0, w_rot=1.0, path_ok=1, follow_ik_ok=1):
    return {"k": float(k), "theta_deg": float(theta), "psi_deg": float(psi), "accepted": float(accepted),
            "reject_mask": float(mask), "d_pos_mm": float(d), "w_full": float(w),
            "J_current": float(d / 100.0 if jcur is None else jcur), "t_wait_s": float(twait),
            "margin_follow_rad": float(mfollow), "margin_rollout_rad": float(mroll), "margin_ik_rad": 0.5,
            "w_trans": float(w_trans), "w_rot": float(w_rot),
            "path_ok": float(path_ok), "follow_ik_ok": float(follow_ik_ok), "w_full_seedmin": NAN, "w_full_seedmax": NAN}


def test_bitmask_decoding_and_counts():
    assert R.decode_mask(0) == []
    assert R.decode_mask(1) == ["ik"]
    assert R.decode_mask(2 | 4) == ["w_hat", "margine"]
    assert R.decode_mask(1 | 2 | 4 | 8 | 16 | 32 | 64) == ["ik", "w_hat", "margine", "r_trust", "velocita", "percorso", "follow_ik"]
    rows = [row(mask=1, accepted=0), row(mask=1 | 8, accepted=0), row(mask=0), row(mask=16 | 32, accepted=1)]
    c = R.count_rejects(rows)
    assert c["ik"] == 2 and c["r_trust"] == 1 and c["velocita"] == 1 and c["percorso"] == 1 and c["w_hat"] == 0


def test_argmin_of_jnew_is_known():
    rows = [row(k=0, d=10, w=1.0), row(k=1, d=20, w=math.exp(2)), row(k=2, d=30, w=math.exp(5))]
    # kappa=1: A=10,B=18,C=25 -> A ; kappa=5: A=10,B=10,C=5 -> C ; kappa=10: A=10,B=0,C=-20 -> C
    # kappa=3: A=10,B=14,C=15 -> A
    for kappa, k in {0.0: 0, 1.0: 0, 3.0: 0, 5.0: 2, 10.0: 2}.items():
        assert R.ranked_jnew(rows, kappa)[0]["k"] == k, kappa


def test_exchange_point_kappa_star_analytic():
    A = row(k=0, d=10.0, w=1.0)               # ln w = 0
    B = row(k=1, d=20.0, w=math.exp(2.0))     # ln w = 2
    ks = R.exchange_kappa(A, B)
    assert abs(ks - 5.0) < 1e-12              # (10-20)/(0-2) = 5
    assert abs(R.j_new(A, ks) - R.j_new(B, ks)) < 1e-12
    assert R.j_new(A, 4.0) < R.j_new(B, 4.0) and R.j_new(A, 6.0) > R.j_new(B, 6.0)
    assert abs(R.exchange_kappa(B, A) - ks) < 1e-12          # symmetric
    assert math.isnan(R.exchange_kappa(row(d=1, w=2.0), row(d=2, w=2.0)))   # parallel lines


def test_pareto_front_five_points():
    pts = [(1.0, 5.0), (2.0, 3.0), (3.0, 4.0), (4.0, 1.0), (5.0, 2.0)]
    # (3,4) dominated by (2,3); (5,2) dominated by (4,1)
    assert R.pareto_front(pts) == [0, 1, 3]
    assert R.pareto_front([(1.0, 1.0), (1.0, 1.0)]) == [0, 1]      # duplicates kept
    assert R.pareto_front([(NAN, 1.0), (1.0, 1.0)]) == [1]


def test_psi_gain_on_a_known_function():
    # w_full(psi) = exp(0.01*(5 - |psi - 2|)) -> maximum at psi=2, gain over psi=0: 0.01*(5-3) ... ln w = .01(5-|psi-2|)
    rows = []
    for psi in range(-5, 6):
        rows.append(row(k=0, theta=10, psi=psi, w=math.exp(0.01 * (5 - abs(psi - 2)))))
    best = R.best_psi_rows(rows)[(0, 10.0)]
    assert best["psi_deg"] == 2.0
    gain = R.psi_gain(rows)
    assert len(gain) == 1 and abs(gain[0] - 0.01 * (5 - 0) + 0.01 * (5 - 2)) < 1e-12   # 0.05 - 0.03 = 0.02
    assert abs(gain[0] - 0.02) < 1e-12
    p = R.percentiles(gain)
    assert abs(p["max"] - 0.02) < 1e-12 and abs(p["p50"] - 0.02) < 1e-12


def test_psi_star_ignores_rejected_psi():
    rows = [row(psi=0, w=1.0), row(psi=3, w=10.0, accepted=0, mask=1), row(psi=-2, w=2.0)]
    assert R.best_psi_rows(rows)[(0, 0.0)]["psi_deg"] == -2.0


def test_all_k_equivalent_by_construction():
    rows = []
    for k in range(4):
        for th, d, w in [(0.0, 12.0, 0.5), (2.0, 7.0, 0.8), (4.0, 9.0, 0.9)]:
            rows.append(row(k=k, theta=th, d=d, w=w))
    pk = R.per_k_minimum(rows)
    assert all(v["d_pos_mm"] == 7.0 and v["theta_deg"] == 2.0 for v in pk.values())
    assert R.k_equivalence_mm(pk) == 0.0
    assert R.accepted_per_k(rows) == {0: 3, 1: 3, 2: 3, 3: 3}
    res = R.analyse(rows, [0.0, 10.0], [0.0], 2.0)
    ref = res["by_delta"]["6.0"]
    assert ref["k_equivalence_mm"] == 0.0
    # deterministic tie-break: smallest k, then theta
    assert ref["kappa"]["0.0"]["k"] == 0 and ref["kappa"]["0.0"]["theta_deg"] == 2.0
    nband, spread, _ = R.tie_band(rows, 0.0, 0.0)
    assert nband == 4 and spread == 0.0           # the four k tie exactly


def test_margin_follow_count_and_spearman_and_ratio():
    rows = [row(mfollow=math.radians(x)) for x in (0, 2, 4, 8, 12)] + [row(mfollow=NAN)]
    assert R.count_margin_follow(rows, 0.0) == 5
    assert R.count_margin_follow(rows, 3.0) == 3
    assert R.count_margin_follow(rows, 17.19) == 0
    assert abs(R.spearman([1, 2, 3, 4, 5], [10, 20, 30, 40, 50]) - 1.0) < 1e-12
    assert abs(R.spearman([1, 2, 3, 4, 5], [5, 4, 3, 2, 1]) + 1.0) < 1e-12
    assert math.isnan(R.spearman([1, 1, 1], [1, 2, 3]))
    s = row(); s["w_full_seedmin"] = 2.0; s["w_full_seedmax"] = 3.0
    assert R.seed_dependence([s]) == [1.5]


def test_end_to_end_csv_and_report_length(tmp_path=None):
    import csv
    import tempfile
    d = Path(tempfile.mkdtemp()) if tmp_path is None else Path(tmp_path)
    src = d / "l.csv"
    header = R.NUMERIC + ["reject_first"]
    with open(src, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=header)
        w.writeheader()
        for k in range(4):
            for th in range(-4, 6, 2):
                for psi in (-1, 0, 1):
                    r = {c: 0.0 for c in R.NUMERIC}
                    r.update(k=k, theta_deg=th, psi_deg=psi, accepted=1, path_ok=1, follow_ik_ok=1, d_pos_mm=10 + abs(th) + k * 0.0,
                             w_full=1.0 + 0.1 * psi, w_trans=1.0, w_rot=1.0, J_current=0.1, t_wait_s=50.0,
                             margin_follow_rad=0.4, margin_rollout_rad=0.5, margin_ik_rad=0.5,
                             w_full_seedmin="nan", w_full_seedmax="nan", reject_first="none")
                    w.writerow(r)
    rc = R.main(["--csv", str(src), "--outdir", str(d / "out")])
    assert rc == 0
    text = (d / "out" / "report.txt").read_text()
    assert len(text.splitlines()) <= 60
    assert (d / "out" / "summary.json").exists() and (d / "out" / "summary.csv").exists()


def test_new_rule_ignores_accepted_column_and_old_thresholds():
    ref_delta = 6.0
    ok_margin = math.radians(10.0)
    rows = [
        # A: w_hat < 0.5 (bit 2) so OLD rejects it, but NEW-admissible. Best J_new at kappa=0 (d=10).
        row(k=0, theta=0, d=10.0, w=1.0, accepted=0, mask=2, mfollow=ok_margin),
        # B: margin_follow below delta_m -> NOT admissible although accepted=1 (smallest d)
        row(k=1, theta=0, d=5.0, w=1.0, accepted=1, mask=0, mfollow=math.radians(2.0)),
        # C: |psi| = 7 > 5 -> NOT admissible (smallest d of all)
        row(k=2, theta=0, psi=7.0, d=4.0, w=1.0, accepted=1, mask=0, mfollow=ok_margin),
        # D: accepted, admissible
        row(k=3, theta=0, d=20.0, w=1.0, accepted=1, mask=0, mfollow=ok_margin, jcur=0.01),
        # E: accepted=1 but path_ok = 0 -> not admissible
        row(k=3, theta=10, d=1.0, w=1.0, accepted=1, mask=0, mfollow=ok_margin, path_ok=0),
        # F: margin bit 4 and R_trust bit 8 set (old rejects) but admissible, farther
        row(k=0, theta=10, d=30.0, w=1.0, accepted=0, mask=4 | 8, mfollow=ok_margin),
        # G: IK failure (bit 1) -> not admissible even with a margin_follow value
        row(k=1, theta=10, d=2.0, w=1.0, accepted=0, mask=1, mfollow=ok_margin),
        # H: NaN margin_follow and I: NaN w_full -> excluded, counted
        row(k=2, theta=10, d=3.0, w=1.0, accepted=1, mask=0, mfollow=NAN),
        row(k=2, theta=20, d=3.0, w=NAN, accepted=1, mask=0, mfollow=ok_margin),
    ]
    pred = R.make_pred(ref_delta, 5.0)
    names = {(int(r["k"]), r["theta_deg"], r["psi_deg"]) for r in rows if pred(r) and r["w_full"] == r["w_full"]}
    assert names == {(0, 0.0, 0.0), (3, 0.0, 0.0), (0, 10.0, 0.0)}
    res = R.analyse(rows, [0.0, 10.0], [0.0, 3.0], 2.0, delta_ref_deg=ref_delta, psi_max_deg=5.0)
    ov = res["old_vs_new"]
    assert ov["n_new_admissible_psi0"] == 3
    # OLD accepted psi0 rows with finite w_full: B, D, E, H (I has NaN w_full and is excluded) = 4
    assert ov["n_old_accepted_psi0"] == 4
    assert ov["new_admissible_psi0_violating_old"] == {"w_hat": 1, "margine": 1, "r_trust": 1, "any": 2}
    assert ov["old_winner"]["k"] == 3 and ov["old_winner"]["theta_deg"] == 0.0     # smallest J_current among accepted (D: 0.01)
    # new winner at the median kappa (5): J = d - 5 ln w with all w = 1 -> smallest d among admissible = A (10)
    assert ov["new_winner"]["k"] == 0 and ov["new_winner"]["theta_deg"] == 0.0
    assert ov["new_winner"]["violates_old"] == ["w_hat"]
    # NaN accounting per column
    nc = res["nan_counts"]
    assert nc["all_rows"] == {"w_full": 1, "margin_follow_rad": 1}
    # delta_m = 3 deg: B (2 deg) is still excluded, nothing new; at delta_m = 1 deg B enters
    assert res["by_delta"]["3.0"]["n_admissible_psi0"] == 3
    assert R.analyse(rows, [0.0], [1.0], 2.0, delta_ref_deg=ref_delta)["by_delta"]["1.0"]["n_admissible_psi0"] == 4
    # flipping the `accepted` column must change nothing in the new-rule results
    flipped = [dict(r, accepted=1.0 - r["accepted"]) for r in rows]
    res2 = R.analyse(flipped, [0.0, 10.0], [0.0, 3.0], 2.0, delta_ref_deg=ref_delta, psi_max_deg=5.0)
    strip = lambda kd: {k: {a: b for a, b in v.items() if a != "is_current_winner"} for k, v in kd.items()}  # noqa: E731
    import json
    assert json.dumps(strip(res2["by_delta"]["6.0"]["kappa"]), sort_keys=True) == \
        json.dumps(strip(res["by_delta"]["6.0"]["kappa"]), sort_keys=True)   # json: nan == nan
    assert res2["old_vs_new"]["n_new_admissible_psi0"] == 3
    assert res2["old_vs_new"]["new_winner"] == ov["new_winner"]
    assert len(R.render_report(res, [0.0, 10.0]).splitlines()) <= 60


# ---------------------------------------------------------------- resolution, psi*, map, tiered, kappa_t*

def test_theta_resolution_and_grid_counts():
    rows = [row(k=0, theta=t) for t in (0.0, 2.0, 2.1, 4.0)] + [dict(row(k=1, theta=0.0), grid="fine")]
    res = R.theta_resolution(rows)
    assert abs(res["min_dtheta_deg"] - 0.1) < 1e-9
    assert res["n_distinct_k_theta"] == 5
    assert res["rows_per_grid"] == {"coarse": 4, "fine": 1}


def test_psi_star_statistics_and_winner_sensitivity_to_psi_max():
    # four (k,theta) with psi* = +5, -5, 4.6, 0 (w peaks there)
    rows = []
    for k, peak in enumerate((5.0, -5.0, 4.6, 0.0)):
        for psi in (-5.0, -4.6, 0.0, 4.6, 5.0):
            rows.append(row(k=k, theta=0, psi=psi, d=10.0, w=math.exp(1.0 - 0.1 * abs(psi - peak))))
    st = R.psi_star_stats(rows, R.make_pred(0.0, 5.0), 5.0)
    assert st["n"] == 4 and st["frac_at_psi_max"] == 0.5 and st["frac_ge_90pct"] == 0.75
    assert math.isnan(R.psi_star_stats(rows, R.make_pred(0.0, 0.0), 0.0)["frac_at_psi_max"])
    # winner sensitivity: A flat (d=10), B (d=12) with ln w = 0.2*psi -> J_B(kappa=5) = 12 - psi
    rows = [row(k=0, theta=0, psi=p, d=10.0, w=1.0) for p in range(-5, 6)]
    rows += [row(k=1, theta=0, psi=p, d=12.0, w=math.exp(0.2 * p)) for p in range(-5, 6)]
    out = R.psi_max_sensitivity(rows, 5.0, 0.0, [0, 1, 3, 5, 8])
    assert [o["winner"]["k"] for o in out] == [0, 0, 1, 1, 1]      # J_B = 12, 11, 9, 7, 7 vs 10
    assert [o["capped"] for o in out] == [False, False, False, False, True]
    assert out[3]["winner"]["psi_deg"] == 5.0


def _k0_k2_rows():
    # k0: d=10, t=100 ; k2: d=12, t=20 ; w = 1 (ln w = 0) so kappa_w plays no role
    return [row(k=0, theta=0, d=10.0, w=1.0, twait=100.0), row(k=2, theta=0, d=12.0, w=1.0, twait=20.0)]


def test_winner_map_advantage_and_pair_flag():
    pool = R.psi_star_pool(_k0_k2_rows(), R.make_pred(0.0, 5.0))
    cells = R.winner_map(pool, [1.0], [0.0, 0.01, 0.05])
    by = {c["kappa_t"]: c for c in cells}
    assert by[0.0]["winner"]["k"] == 0 and abs(by[0.0]["advantage_other_k_mm"] - 2.0) < 1e-12
    assert by[0.01]["winner"]["k"] == 0 and abs(by[0.01]["advantage_other_k_mm"] - (12.2 - 11.0)) < 1e-12
    assert by[0.05]["winner"]["k"] == 2 and abs(by[0.05]["advantage_other_k_mm"] - (15.0 - 13.0)) < 1e-12
    assert all(c["pair_k0k2"] for c in cells)
    assert abs(by[0.05]["winner"]["J"] - 13.0) < 1e-12


def test_tiered_winner_picks_shortest_wait_inside_the_band():
    pool = R.psi_star_pool(_k0_k2_rows(), R.make_pred(0.0, 5.0))
    t1 = R.tiered_winner(pool, 1.0, 1.0)      # band excludes k2 (J0 = 12 > 10 + 1)
    assert t1["k"] == 0 and t1["n_in_band"] == 1
    t3 = R.tiered_winner(pool, 1.0, 3.0)      # both inside: k2 waits less
    assert t3["k"] == 2 and t3["n_in_band"] == 2 and t3["t_wait_s"] == 20.0


def test_kappa_t_critical_value_k0_k2():
    rows = _k0_k2_rows()
    pool = R.psi_star_pool(rows, R.make_pred(0.0, 5.0))
    res = R.kappa_t_star_pair(pool, 3.0)
    assert abs(res["kappa_t_star"] - 0.025) < 1e-12              # (12-10)/(100-20)
    assert res["valid"] and res["k0_wins_at_zero"]
    # the two lines really cross there
    a, b = rows
    assert abs(R.j_general(a, 3.0, 0.025) - R.j_general(b, 3.0, 0.025)) < 1e-12
    assert abs(R.kappa_t_star_envelope(pool, 3.0) - 0.025) < 1e-6
    # k0 better AND faster: no swap for kappa_t >= 0 -> negative / invalid
    pool2 = R.psi_star_pool([row(k=0, theta=0, d=10.0, w=1.0, twait=10.0), row(k=2, theta=0, d=12.0, w=1.0, twait=20.0)],
                            R.make_pred(0.0, 5.0))
    r2 = R.kappa_t_star_pair(pool2, 3.0)
    assert r2["kappa_t_star"] < 0 and not r2["valid"]
    assert math.isnan(R.kappa_t_star_envelope(pool2, 3.0))
    # ln w enters J0: k2 with w = e^2 at kappa_w = 5 gains 10 mm -> J0_k2 = 2, k2 wins at kappa_t = 0, kt* = (2-10)/(100-20) < 0
    pool3 = R.psi_star_pool([row(k=0, theta=0, d=10.0, w=1.0, twait=100.0),
                             row(k=2, theta=0, d=12.0, w=math.exp(2.0), twait=20.0)], R.make_pred(0.0, 5.0))
    r3 = R.kappa_t_star_pair(pool3, 5.0)
    assert abs(r3["kappa_t_star"] - (2.0 - 10.0) / 80.0) < 1e-12 and not r3["k0_wins_at_zero"]


def test_full_analysis_contains_new_sections_and_fits_60_lines(tmp_path=None):
    import json
    import tempfile
    d = Path(tempfile.mkdtemp()) if tmp_path is None else Path(tmp_path)
    rows = []
    for k in range(4):
        for th in (-4.0, 0.0, 4.0):
            for psi in (-5.0, 0.0, 5.0):
                rows.append(row(k=k, theta=th, psi=psi, d=10.0 + abs(th) + k, w=1.0 + 0.05 * psi + 0.01 * k,
                                twait=100.0 + 10 * k + th))
    res = R.analyse(rows, [0.0, 1.0, 5.0, 10.0], [0.0, 6.0], 2.0)
    assert set(res["map"]["kappas_w"]) == set(R.KAPPA_W_MAP) and len(res["map"]["cells"]) == 63
    text = R.render_report(res, [0.0, 1.0, 5.0, 10.0])
    assert len(text.splitlines()) <= 60
    out = d / "o"
    out.mkdir()
    R.write_summary_csv(str(out / "summary.csv"), res, [0.0, 1.0])
    kinds = {r["kind"] for r in __import__("csv").DictReader(open(out / "summary.csv"))}
    assert kinds == {"jnew", "map", "tiered", "kt_star"}
    json.dumps(res, allow_nan=True)
