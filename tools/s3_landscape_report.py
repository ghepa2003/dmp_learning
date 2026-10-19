#!/usr/bin/env python3
"""Offline analysis of the S3 landscape CSV written by `s3_landscape` (src/satellite_grasp_planner/analysis).

The new cost formulation lives ONLY here (it is not implemented in the planner):

    J_new(k, theta, psi) = d_mm + kappa_w * (-ln w_full)
    d_mm   = ||p(k,theta) - (p0 + delta_g_demo)|| in mm  (column d_pos_mm: the S3 distance, offset included)
    w_full = sqrt(det(J J^T)), J = 6x7 geometric Jacobian at the grasp pose (column w_full)
    psi*   = argmax of w_full over the ADMISSIBLE psi of each (k, theta)

Conventions (no hidden thresholds; every threshold is an argument):
  * OLD rule ("accepted"): the planner's a-priori constraints pass (CSV column accepted, reject_mask & 15 == 0:
    IK, w_hat >= 0.5, margin >= 0.3 rad, R_trust). Used only for the old-vs-new comparison and the current winner.
  * NEW rule (admissible_new, all the J_new analyses): IK ok on the selected branch (reject_mask bit 1 NOT set)
    AND path_ok == 1 AND follow_ik_ok == 1 AND margin_follow_rad >= delta_m AND |psi_deg| <= psi_max_deg
    (and finite w_full > 0). The bits 2, 4, 8 (w_hat, margin, R_trust) and the column `accepted` are NOT used.
    path_ok is 0 when the rollout was not computed (s3_landscape --rollout none/psi0), so use --rollout all.
  * psi* of a (k, theta) = argmax w_full over its admissible_new rows. Rows with NaN w_full or margin_follow_rad
    never enter any computation; their number is reported per column.
  * the "current winner" = the accepted psi = 0 row with the smallest J_current (= d + eps*t_wait of the planner).
  * rejection reason counts are over the psi = 0 rows (the real candidate set), all bits of reject_mask.
Usage:
  s3_landscape_report.py --csv landscape.csv --outdir DIR [--kappa-list 0,1,2,5,10,20,50,100]
      [--delta-m-list-deg 0,3,6,10,17.19] [--delta-m-ref-deg 6.0] [--psi-max-deg 5.0] [--tau-j-mm 2.0]
      [--map-kappa-w-list 0.5,1,2,5,10,20,50] [--map-kappa-t-list 0,0.01,0.05,0.1,0.2,0.5,1,2,5]
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

# reject_mask bits: 1..8 = planner a-priori constraints, 16..64 = diagnostics of the tool (see analysis/README.md)
REJECT_BITS = {1: "ik", 2: "w_hat", 4: "margine", 8: "r_trust", 16: "velocita", 32: "percorso", 64: "follow_ik"}
APRIORI_MASK = 1 | 2 | 4 | 8

NUMERIC = ["k", "theta_deg", "psi_deg", "accepted", "reject_mask", "ik_ok_A", "ik_ok_B", "branch_selected",
           "d_pos_mm", "t_wait_s", "J_current", "w_hat", "w_trans", "w_rot", "w_full", "smin_trans", "smin_rot",
           "margin_rollout_rad", "margin_follow_rad", "follow_ik_ok", "path_ok", "w_full_other", "margin_other",
           "margin_ik_rad", "dv_m_s", "w_full_seedmin", "w_full_seedmax", "margin_seedmin", "margin_seedmax",
           "n_seed_conv"]


# ------------------------------------------------------------------ I/O

def read_landscape(path):
    """Reads the CSV into a list of dicts; numeric columns -> float (nan stays nan), reject_first stays str."""
    rows = []
    with open(path, newline="") as f:
        for rec in csv.DictReader(f):
            row = {}
            for key, val in rec.items():
                if key in NUMERIC and val is not None:
                    row[key] = float(val)
                else:
                    row[key] = val
            rows.append(row)
    return rows


def read_meta(csv_path):
    """Scenario metadata written by s3_landscape next to the CSV (<csv>.meta, key=value lines); None if absent."""
    try:
        with open(csv_path + ".meta") as f:
            meta = dict(line.rstrip("\n").split("=", 1) for line in f if "=" in line)
    except OSError:
        return None
    return meta or None


def psi_rows_without_margins(rows):
    """(n_bad, n_psi_ik_ok): IK-ok rows with psi != 0 (n_psi_ik_ok) and, among them, those with BOTH margin_rollout_rad
    and margin_follow_rad NaN (n_bad). Typical of `s3_landscape --rollout psi0`."""
    ok = [r for r in rows if abs(r["psi_deg"]) > 1e-9 and (int(r["reject_mask"]) & 1) == 0]
    bad = [r for r in ok if r["margin_rollout_rad"] != r["margin_rollout_rad"] and r["margin_follow_rad"] != r["margin_follow_rad"]]
    return len(bad), len(ok)


# ------------------------------------------------------------------ pure functions

def decode_mask(mask):
    """Names of all the bits set in reject_mask (order of increasing bit)."""
    mask = int(mask)
    return [name for bit, name in sorted(REJECT_BITS.items()) if mask & bit]


def psi0_rows(rows, tol=1e-9):
    """Rows with psi = 0 (|psi_deg| <= tol)."""
    return [r for r in rows if abs(r["psi_deg"]) <= tol]


def accepted_rows(rows):
    """Rows whose a-priori constraints pass (accepted == 1)."""
    return [r for r in rows if r["accepted"] == 1]


def count_rejects(rows):
    """{reason: n_rows} over ALL bits of reject_mask of the given rows (a row can count for several reasons)."""
    out = {name: 0 for _, name in sorted(REJECT_BITS.items())}
    for r in rows:
        for name in decode_mask(r["reject_mask"]):
            out[name] += 1
    return out


def accepted_per_k(rows):
    """{k: number of accepted rows}, k = 0..3 always present."""
    out = {k: 0 for k in range(4)}
    for r in accepted_rows(rows):
        out[int(r["k"])] += 1
    return out


def ln_w(row):
    """ln(w_full) of a row (nan if w_full <= 0 or not finite)."""
    w = row["w_full"]
    return math.log(w) if (w == w and w > 0.0) else float("nan")


def j_new(row, kappa_w):
    """J_new = d_mm + kappa_w * (-ln w_full)."""
    return row["d_pos_mm"] + kappa_w * (-ln_w(row))


def _finite_pos(x):
    return x == x and x > 0.0


def old_pred(r):
    """OLD rule: the planner's a-priori constraints pass (column accepted)."""
    return r["accepted"] == 1


def make_pred(delta_m_deg, psi_max_deg=5.0):
    """admissible_new(row) for a given delta_m [deg] and |psi| bound [deg]: IK ok on the selected branch (bit 1 not
    set) AND path_ok AND follow_ik_ok AND margin_follow_rad >= delta_m AND |psi| <= psi_max AND finite w_full > 0.
    Bits 2, 4, 8 and `accepted` are ignored."""
    thr = math.radians(delta_m_deg)

    def pred(r):
        mf = r["margin_follow_rad"]
        return (int(r["reject_mask"]) & 1) == 0 and r["path_ok"] == 1 and r["follow_ik_ok"] == 1 \
            and mf == mf and mf >= thr and abs(r["psi_deg"]) <= psi_max_deg + 1e-9 and _finite_pos(r["w_full"])
    return pred


def admissible(rows, pred=old_pred):
    """Rows satisfying pred (default: OLD rule) with finite positive w_full."""
    return [r for r in rows if pred(r) and _finite_pos(r["w_full"])]


def nan_counts(rows):
    """Rows excluded from the calculations because of NaN, per column: {column: n} over all rows and over the rows
    with the IK ok (bit 1 not set)."""
    cols = ("w_full", "margin_follow_rad")
    allr = {c: sum(1 for r in rows if r[c] != r[c]) for c in cols}
    ok = [r for r in rows if (int(r["reject_mask"]) & 1) == 0]
    return {"all_rows": allr, "ik_ok_rows": {c: sum(1 for r in ok if r[c] != r[c]) for c in cols},
            "n_rows": len(rows), "n_ik_ok_rows": len(ok)}


def per_k_minimum(rows, pred=old_pred):
    """Per k: the psi=0 row satisfying pred of smallest d_pos_mm. Returns {k: dict or None}."""
    out = {}
    for k in range(4):
        cand = [r for r in admissible(psi0_rows(rows), pred) if int(r["k"]) == k]
        out[k] = min(cand, key=lambda r: (r["d_pos_mm"], r["theta_deg"])) if cand else None
    return out


def k_equivalence_mm(per_k):
    """max - min of d_min over the k that have an accepted candidate [mm] (0 = all k equivalent); nan if none."""
    d = [v["d_pos_mm"] for v in per_k.values() if v is not None]
    return (max(d) - min(d)) if d else float("nan")


def pareto_front(points):
    """Indices of the non-dominated points of [(x, y), ...] minimising both coordinates (strict domination:
    p dominates q if p <= q in both and < in at least one). Duplicates are all kept. NaN points are skipped."""
    idx = [i for i, p in enumerate(points) if p[0] == p[0] and p[1] == p[1]]
    front = []
    for i in idx:
        xi, yi = points[i]
        dominated = False
        for j in idx:
            if j == i:
                continue
            xj, yj = points[j]
            if xj <= xi and yj <= yi and (xj < xi or yj < yi):
                dominated = True
                break
        if not dominated:
            front.append(i)
    return front


def best_psi_rows(rows, pred=old_pred):
    """For every (k, theta) of the admissible rows, the row with the largest w_full (psi*). Ties: smallest |psi|, then psi."""
    best = {}
    for r in admissible(rows, pred):
        key = (int(r["k"]), r["theta_deg"])
        cur = best.get(key)
        if cur is None or (r["w_full"], -abs(r["psi_deg"]), -r["psi_deg"]) > (cur["w_full"], -abs(cur["psi_deg"]), -cur["psi_deg"]):
            best[key] = r
    return best


def psi_gain(rows, tol=1e-9, pred=old_pred):
    """For every (k, theta) admissible at psi = 0: max over admissible psi of ln w_full - ln w_full(psi = 0).
    Returns the list of gains (>= 0 because psi = 0 is itself admissible)."""
    zero = {(int(r["k"]), r["theta_deg"]): r for r in admissible(psi0_rows(rows, tol), pred)}
    best = best_psi_rows(rows, pred)
    return [ln_w(best[key]) - ln_w(z) for key, z in zero.items() if key in best]


def percentiles(values, qs=(50, 90, 99)):
    """{q: percentile} + 'max' of a list (nan-filled if empty)."""
    v = np.array([x for x in values if x == x], dtype=float)
    out = {f"p{q}": (float(np.percentile(v, q)) if v.size else float("nan")) for q in qs}
    out["max"] = float(v.max()) if v.size else float("nan")
    return out


def current_winner(rows):
    """The accepted psi=0 row with the smallest J_current (tie: k, theta). None if there is none."""
    cand = accepted_rows(psi0_rows(rows))
    return min(cand, key=lambda r: (r["J_current"], r["k"], r["theta_deg"])) if cand else None


def ranked_jnew(rows, kappa_w, use_psi_star=True, pred=old_pred):
    """Candidates sorted by J_new (tie: k, theta, |psi|). With use_psi_star each (k, theta) appears once with its
    psi* (the psi maximising w_full), otherwise the psi = 0 accepted rows."""
    if use_psi_star:
        pool = list(best_psi_rows(rows, pred).values())
    else:
        pool = admissible(psi0_rows(rows), pred)
    pool = [r for r in pool if j_new(r, kappa_w) == j_new(r, kappa_w)]
    return sorted(pool, key=lambda r: (j_new(r, kappa_w), r["k"], r["theta_deg"], abs(r["psi_deg"])))


def exchange_kappa(row_a, row_b):
    """kappa* at which J_new(A) = J_new(B), with J = d - kappa ln w:
         J_A - J_B = (d_A - d_B) - kappa (ln w_A - ln w_B) = 0
         => kappa* = (d_A - d_B) / (ln w_A - ln w_B)   [= (d_B - d_A) / (ln w_B - ln w_A)].
    Valid for kappa > 0 only if kappa* > 0. Returns nan if ln w_A == ln w_B (the lines never cross)."""
    dl = ln_w(row_a) - ln_w(row_b)
    if dl == 0.0 or dl != dl:
        return float("nan")
    return (row_a["d_pos_mm"] - row_b["d_pos_mm"]) / dl


def count_margin_follow(rows, delta_m_deg, pred=old_pred):
    """Number of rows satisfying pred with margin_follow_rad >= delta_m (given in degrees)."""
    thr = math.radians(delta_m_deg)
    return sum(1 for r in rows if pred(r) and r["margin_follow_rad"] == r["margin_follow_rad"]
               and r["margin_follow_rad"] >= thr)


def _ranks(x):
    """Average ranks (1-based) with ties."""
    x = np.asarray(x, dtype=float)
    order = np.argsort(x, kind="mergesort")
    ranks = np.empty(len(x), dtype=float)
    i = 0
    while i < len(x):
        j = i
        while j + 1 < len(x) and x[order[j + 1]] == x[order[i]]:
            j += 1
        ranks[order[i:j + 1]] = 0.5 * (i + j) + 1.0
        i = j + 1
    return ranks


def spearman(x, y):
    """Spearman rank correlation of two equally long sequences (pairs with a nan are dropped); nan if < 3 pairs
    or a constant sequence."""
    pairs = [(a, b) for a, b in zip(x, y) if a == a and b == b]
    if len(pairs) < 3:
        return float("nan")
    a = _ranks([p[0] for p in pairs])
    b = _ranks([p[1] for p in pairs])
    if np.std(a) == 0.0 or np.std(b) == 0.0:
        return float("nan")
    return float(np.corrcoef(a, b)[0, 1])


def tie_band(rows, kappa_w, tau_j_mm, pred=old_pred):
    """Candidates (with psi*) whose J_new <= J_new_min + tau_J [mm]: returns (n, spread_t_wait_s, list)."""
    ranked = ranked_jnew(rows, kappa_w, use_psi_star=True, pred=pred)
    if not ranked:
        return 0, float("nan"), []
    jmin = j_new(ranked[0], kappa_w)
    band = [r for r in ranked if j_new(r, kappa_w) <= jmin + tau_j_mm]
    waits = [r["t_wait_s"] for r in band]
    return len(band), float(max(waits) - min(waits)), band


def seed_dependence(rows):
    """w_full_seedmax / w_full_seedmin of the rows that have both (accepted psi=0 rows with --seed-spread)."""
    return [r["w_full_seedmax"] / r["w_full_seedmin"] for r in rows
            if r["w_full_seedmin"] == r["w_full_seedmin"] and r["w_full_seedmax"] == r["w_full_seedmax"]
            and r["w_full_seedmin"] > 0.0]


# ------------------------------------------------------------------ report

# ---------------------------------------------------------------- resolution, psi*, wait-aware winners

KAPPA_W_MAP = [0.5, 1, 2, 5, 10, 20, 50]                 # mm per unit of -ln w_full
KAPPA_T_MAP = [0, 0.01, 0.05, 0.1, 0.2, 0.5, 1, 2, 5]    # mm per second of waiting
PSI_MAX_SENSITIVITY = [0, 1, 3, 5, 8]


def theta_resolution(rows):
    """Effective theta resolution of the exported rows: per k the sorted distinct theta_deg; returns the smallest
    spacing over all k [deg] (nan if fewer than 2 distinct thetas), the number of distinct (k, theta) and the rows per
    grid label (rows without a `grid` column count as coarse)."""
    per_k = {}
    for r in rows:
        per_k.setdefault(int(r["k"]), set()).add(round(r["theta_deg"], 9))
    spacing = []
    for ths in per_k.values():
        v = sorted(ths)
        spacing += [b - a for a, b in zip(v, v[1:]) if b - a > 1e-9]
    grids = {}
    for r in rows:
        g = r.get("grid") or "coarse"
        grids[g] = grids.get(g, 0) + 1
    return {"min_dtheta_deg": min(spacing) if spacing else float("nan"),
            "n_distinct_k_theta": sum(len(v) for v in per_k.values()), "rows_per_grid": grids}


def psi_star_stats(rows, pred, psi_max_deg):
    """Over the (k, theta) that have at least one admissible row: fraction whose psi* has |psi*| == psi_max and
    |psi*| >= 0.9 psi_max (nan if psi_max == 0), and their number."""
    best = list(best_psi_rows(rows, pred).values())
    n = len(best)
    if n == 0 or not psi_max_deg > 0.0:
        return {"n": n, "frac_at_psi_max": float("nan"), "frac_ge_90pct": float("nan")}
    at = sum(1 for r in best if abs(abs(r["psi_deg"]) - psi_max_deg) <= 1e-9)
    ge = sum(1 for r in best if abs(r["psi_deg"]) >= 0.9 * psi_max_deg - 1e-9)
    return {"n": n, "frac_at_psi_max": at / n, "frac_ge_90pct": ge / n}


def j_general(row, kappa_w, kappa_t):
    """J = d_mm + kappa_w * (-ln w_full) + kappa_t * t_wait_s   [mm]."""
    return row["d_pos_mm"] + kappa_w * (-ln_w(row)) + kappa_t * row["t_wait_s"]


def psi_star_pool(rows, pred):
    """One row per admissible (k, theta): the one with psi* (max w_full). Because d and t_wait do not depend on psi,
    psi* also minimises J_general over psi for every kappa_w >= 0, kappa_t."""
    return list(best_psi_rows(rows, pred).values())


def ranked_general(pool, kappa_w, kappa_t):
    """pool sorted by J_general (tie: k, theta, |psi|); rows with a NaN J are dropped."""
    ok = [r for r in pool if j_general(r, kappa_w, kappa_t) == j_general(r, kappa_w, kappa_t)]
    return sorted(ok, key=lambda r: (j_general(r, kappa_w, kappa_t), r["k"], r["theta_deg"], abs(r["psi_deg"])))


def _brief_general(r, kappa_w, kappa_t):
    return {"k": int(r["k"]), "theta_deg": r["theta_deg"], "psi_deg": r["psi_deg"], "d_mm": r["d_pos_mm"],
            "w_full": r["w_full"], "t_wait_s": r["t_wait_s"], "margin_rollout": r["margin_rollout_rad"],
            "margin_follow": r["margin_follow_rad"], "J": j_general(r, kappa_w, kappa_t)}


def winner_map(pool, kappas_w, kappas_t):
    """Winner of J_general on the grid kappas_w x kappas_t. For each cell also: advantage_second_mm = J(2nd) - J(1st)
    (2nd = next in the ranking, usually a neighbouring theta of the same k), advantage_other_k_mm = J(best candidate of a
    different k) - J(1st), and pair_k0k2 = the winner and that other-k runner-up are exactly {k0, k2}."""
    cells = []
    for kw in kappas_w:
        for kt in kappas_t:
            ranked = ranked_general(pool, kw, kt)
            if not ranked:
                cells.append({"kappa_w": kw, "kappa_t": kt, "winner": None})
                continue
            a = ranked[0]
            ja = j_general(a, kw, kt)
            other = next((r for r in ranked[1:] if int(r["k"]) != int(a["k"])), None)
            cell = {"kappa_w": kw, "kappa_t": kt, "winner": _brief_general(a, kw, kt),
                    "advantage_second_mm": (j_general(ranked[1], kw, kt) - ja) if len(ranked) > 1 else float("nan"),
                    "advantage_other_k_mm": (j_general(other, kw, kt) - ja) if other is not None else float("nan"),
                    "other_k": None if other is None else int(other["k"]),
                    "pair_k0k2": bool(other is not None and {int(a["k"]), int(other["k"])} == {0, 2})}
            cells.append(cell)
    return cells


def tiered_winner(pool, kappa_w, tau_j_mm):
    """Two-level formulation: J0 = d_mm + kappa_w*(-ln w_full) (no waiting); among the candidates with
    J0 <= J0_min + tau_J the one with the smallest t_wait wins (tie: J0, k, theta)."""
    ranked = ranked_general(pool, kappa_w, 0.0)
    if not ranked:
        return None
    jmin = j_general(ranked[0], kappa_w, 0.0)
    band = [r for r in ranked if j_general(r, kappa_w, 0.0) <= jmin + tau_j_mm]
    w = min(band, key=lambda r: (r["t_wait_s"], j_general(r, kappa_w, 0.0), r["k"], r["theta_deg"]))
    out = _brief_general(w, kappa_w, 0.0)
    out["n_in_band"] = len(band)
    out["J0_min"] = jmin
    return out


def psi_max_sensitivity(rows, kappa_w, delta_m_deg, psi_maxes):
    """Winner of J_new (kappa_t = 0) for every psi_max of the list. `capped` = psi_max is larger than the largest |psi|
    present in the CSV, so the result cannot differ from the one at the largest exported |psi|."""
    data_max = max([abs(r["psi_deg"]) for r in rows] + [0.0])
    out = []
    for pm in psi_maxes:
        pool = psi_star_pool(rows, make_pred(delta_m_deg, pm))
        ranked = ranked_general(pool, kappa_w, 0.0)
        out.append({"psi_max_deg": pm, "capped": bool(pm > data_max + 1e-9), "n_admissible_kt": len(pool),
                    "winner": None if not ranked else _brief_general(ranked[0], kappa_w, 0.0)})
    return out


def kappa_t_star_pair(pool, kappa_w):
    """Critical kappa_t at which the best k0 and the best k2 candidate (each the argmin of J0 = d + kappa_w(-ln w), i.e.
    kappa_t = 0) swap. With J_k(kappa_t) = J0_k + kappa_t * t_k:
         J_k0 = J_k2   <=>   J0_k0 + kt t_k0 = J0_k2 + kt t_k2   <=>   kt* = (J0_k2 - J0_k0) / (t_k0 - t_k2).
    Signs: k0 wins at kappa_t = 0 iff J0_k0 < J0_k2. kt* > 0 only if (J0_k2 - J0_k0) and (t_k0 - t_k2) have the same
    sign; e.g. k0 better without waiting (J0_k2 - J0_k0 > 0) but waiting longer (t_k0 > t_k2) => kt* > 0 and k0 wins
    for kappa_t < kt*, k2 for kappa_t > kt*. kt* <= 0 (or nan) means there is no swap for kappa_t >= 0."""
    ranked = ranked_general(pool, kappa_w, 0.0)
    a = next((r for r in ranked if int(r["k"]) == 0), None)
    b = next((r for r in ranked if int(r["k"]) == 2), None)
    if a is None or b is None:
        return {"kappa_t_star": float("nan"), "valid": False, "reason": "missing k0 or k2"}
    ja, jb = j_general(a, kappa_w, 0.0), j_general(b, kappa_w, 0.0)
    dt = a["t_wait_s"] - b["t_wait_s"]
    ks = (jb - ja) / dt if dt != 0.0 else float("nan")
    return {"kappa_t_star": ks, "valid": bool(ks == ks and ks > 0.0), "J0_k0": ja, "J0_k2": jb,
            "t_k0": a["t_wait_s"], "t_k2": b["t_wait_s"], "k0_wins_at_zero": bool(ja < jb),
            "k0_theta_deg": a["theta_deg"], "k2_theta_deg": b["theta_deg"]}


def kappa_t_star_envelope(pool, kappa_w, kt_max=1000.0, n=2000):
    """Same swap, but between the BEST candidate of k0 and of k2 at every kappa_t (lower envelope E_k(kt) = min_c
    (J0_c + kt t_c)): first kappa_t > 0 where E_k0 - E_k2 changes sign (log grid + bisection); nan if it does not on
    [0, kt_max]. It can differ from kappa_t_star_pair when the best theta of a k changes with kappa_t."""
    lines = {0: [], 2: []}
    for r in ranked_general(pool, kappa_w, 0.0):
        if int(r["k"]) in lines:
            lines[int(r["k"])].append((j_general(r, kappa_w, 0.0), r["t_wait_s"]))
    if not lines[0] or not lines[2]:
        return float("nan")

    def f(kt):
        return min(a + kt * b for a, b in lines[0]) - min(a + kt * b for a, b in lines[2])

    grid = [0.0] + list(np.geomspace(1e-4, kt_max, n))
    prev = f(grid[0])
    for x0, x1 in zip(grid, grid[1:]):
        cur = f(x1)
        if prev == 0.0:
            return x0
        if prev * cur < 0.0:
            lo, hi = x0, x1
            for _ in range(80):
                mid = 0.5 * (lo + hi)
                if f(lo) * f(mid) <= 0.0:
                    hi = mid
                else:
                    lo = mid
            return 0.5 * (lo + hi)
        prev = cur
    return float("nan")


def _row_brief(r, kappa_w):
    return {"k": int(r["k"]), "theta_deg": r["theta_deg"], "psi_deg": r["psi_deg"], "d_mm": r["d_pos_mm"],
            "w_full": r["w_full"], "margin_rollout": r["margin_rollout_rad"], "margin_follow": r["margin_follow_rad"],
            "t_wait_s": r["t_wait_s"], "J_new": j_new(r, kappa_w)}


def old_vs_new(rows, pred, kappa_w):
    """Old (accepted) vs new (pred) rule on the psi = 0 rows, plus the winners of the two rules."""
    z = psi0_rows(rows)
    old = admissible(z, old_pred)
    new0 = admissible(z, pred)
    new_pairs = best_psi_rows(rows, pred)
    viol = {"w_hat": 0, "margine": 0, "r_trust": 0, "any": 0}
    for r in new0:
        m = int(r["reject_mask"])
        viol["w_hat"] += 1 if m & 2 else 0
        viol["margine"] += 1 if m & 4 else 0
        viol["r_trust"] += 1 if m & 8 else 0
        viol["any"] += 1 if m & 14 else 0
    ow = current_winner(rows)
    ranked = ranked_jnew(rows, kappa_w, True, pred)
    nw = ranked[0] if ranked else None
    return {"n_old_accepted_psi0": len(old), "n_new_admissible_psi0": len(new0),
            "n_new_admissible_kt_with_psi": len(new_pairs),
            "new_admissible_psi0_violating_old": viol,
            "old_winner": None if ow is None else {"k": int(ow["k"]), "theta_deg": ow["theta_deg"],
                                                   "d_mm": ow["d_pos_mm"], "w_full": ow["w_full"]},
            "new_winner_kappa": kappa_w,
            "new_winner": None if nw is None else {"k": int(nw["k"]), "theta_deg": nw["theta_deg"],
                                                   "psi_deg": nw["psi_deg"], "d_mm": nw["d_pos_mm"],
                                                   "w_full": nw["w_full"], "reject_mask": int(nw["reject_mask"]),
                                                   "violates_old": decode_mask(int(nw["reject_mask"]) & 14)}}


def analyse_delta(rows, kappas, delta_m_deg, psi_max_deg, tau_j_mm):
    """All the numbers of the NEW rule for one delta_m."""
    pred = make_pred(delta_m_deg, psi_max_deg)
    z = psi0_rows(rows)
    new0 = admissible(z, pred)
    out = {"delta_m_deg": delta_m_deg, "n_admissible_psi0": len(new0),
           "admissible_per_k_psi0": {k: sum(1 for r in new0 if int(r["k"]) == k) for k in range(4)},
           "n_admissible_all_psi": len(admissible(rows, pred))}
    pk = per_k_minimum(rows, pred)
    out["per_k_min"] = {str(k): (None if v is None else
                                 {"d_mm": v["d_pos_mm"], "theta_deg": v["theta_deg"], "w_full": v["w_full"],
                                  "margin_rollout": v["margin_rollout_rad"], "margin_follow": v["margin_follow_rad"]})
                        for k, v in pk.items()}
    out["k_equivalence_mm"] = k_equivalence_mm(pk)
    pts0 = [(r["d_pos_mm"], -ln_w(r)) for r in new0]
    best = list(best_psi_rows(rows, pred).values())
    ptss = [(r["d_pos_mm"], -ln_w(r)) for r in best]
    f0, fs = pareto_front(pts0), pareto_front(ptss)
    out["pareto_psi0"] = [{"d_mm": pts0[i][0], "neg_ln_w": pts0[i][1], "k": int(new0[i]["k"]),
                           "theta_deg": new0[i]["theta_deg"]} for i in f0]
    out["pareto_psistar"] = [{"d_mm": ptss[i][0], "neg_ln_w": ptss[i][1], "k": int(best[i]["k"]),
                              "theta_deg": best[i]["theta_deg"], "psi_deg": best[i]["psi_deg"]} for i in fs]
    out["psi_gain_ln_w"] = percentiles(psi_gain(rows, pred=pred))
    cw = current_winner(rows)
    out["kappa"] = {}
    for kw in kappas:
        ranked = ranked_jnew(rows, kw, True, pred)
        if not ranked:
            out["kappa"][str(kw)] = None
            continue
        a = ranked[0]
        entry = _row_brief(a, kw)
        entry["is_current_winner"] = bool(cw is not None and int(a["k"]) == int(cw["k"])
                                          and a["theta_deg"] == cw["theta_deg"])
        if len(ranked) > 1:
            b = ranked[1]
            entry["second"] = _row_brief(b, kw)
            entry["kappa_star_A_vs_B"] = exchange_kappa(a, b)
        out["kappa"][str(kw)] = entry
    out["spearman"] = {
        "ln_w_full_vs_margin_follow": spearman([ln_w(r) for r in new0], [r["margin_follow_rad"] for r in new0]),
        "w_trans_vs_w_rot": spearman([r["w_trans"] for r in new0], [r["w_rot"] for r in new0])}
    kmed = float(np.median(kappas)) if kappas else float("nan")
    n_band, spread, _ = tie_band(rows, kmed, tau_j_mm, pred)
    out["tie_band"] = {"kappa_median": kmed, "tau_J_mm": tau_j_mm, "n": n_band, "t_wait_spread_s": spread}
    out["seed_ratio"] = percentiles(seed_dependence(new0))
    out["seed_ratio_n"] = len(seed_dependence(new0))
    return out


def analyse(rows, kappas, deltas_deg, tau_j_mm, delta_ref_deg=6.0, psi_max_deg=5.0,
            kappas_w_map=None, kappas_t_map=None, meta=None):
    """All the numbers of the report as a JSON-serialisable dict. Points b-e, g-i and the old-vs-new comparison use
    delta_ref_deg; `by_delta` holds the full results for every delta_m of the list (and the reference)."""
    z = psi0_rows(rows)
    out = {"n_rows": len(rows), "n_psi0_rows": len(z), "n_accepted_psi0_old": len(admissible(z, old_pred)),
           "delta_ref_deg": delta_ref_deg, "psi_max_deg": psi_max_deg}
    out["nan_counts"] = nan_counts(rows)
    n_bad, n_psi = psi_rows_without_margins(rows)
    out["psi_invalid"] = n_bad > 0
    out["psi_rows_without_margins"] = {"n": n_bad, "n_psi_ik_ok": n_psi}
    out["meta"] = meta
    out["rejects_psi0"] = count_rejects(z)
    out["accepted_per_k_psi0_old"] = accepted_per_k(z)
    deltas = sorted(set(float(x) for x in list(deltas_deg) + [delta_ref_deg]))
    out["by_delta"] = {str(d): analyse_delta(rows, kappas, d, psi_max_deg, tau_j_mm) for d in deltas}
    kmed = float(np.median(kappas)) if kappas else 0.0
    out["old_vs_new"] = old_vs_new(rows, make_pred(delta_ref_deg, psi_max_deg), kmed)
    cw = current_winner(rows)
    out["current_winner"] = None if cw is None else {
        "k": int(cw["k"]), "theta_deg": cw["theta_deg"], "d_mm": cw["d_pos_mm"], "w_full": cw["w_full"],
        "J_current": cw["J_current"], "t_wait_s": cw["t_wait_s"], "margin_rollout": cw["margin_rollout_rad"]}

    kws = list(KAPPA_W_MAP if kappas_w_map is None else kappas_w_map)
    kts = list(KAPPA_T_MAP if kappas_t_map is None else kappas_t_map)
    pred = make_pred(delta_ref_deg, psi_max_deg)
    pool = psi_star_pool(rows, pred)
    out["resolution"] = theta_resolution(rows)
    out["psi_star_stats"] = psi_star_stats(rows, pred, psi_max_deg)
    out["psi_star_stats"]["winner_psi_deg"] = (ranked_general(pool, kmed, 0.0)[0]["psi_deg"]
                                              if ranked_general(pool, kmed, 0.0) else float("nan"))
    out["psi_max_sensitivity"] = {"kappa_w": kmed, "results": psi_max_sensitivity(rows, kmed, delta_ref_deg, PSI_MAX_SENSITIVITY)}
    out["map"] = {"kappas_w": kws, "kappas_t": kts, "cells": winner_map(pool, kws, kts)}
    out["tiered"] = {str(kw): tiered_winner(pool, kw, tau_j_mm) for kw in kws}
    out["kappa_t_star"] = {str(kw): dict(kappa_t_star_pair(pool, kw), envelope=kappa_t_star_envelope(pool, kw)) for kw in kws}
    return out


def _f(x, nd=3):
    return "nan" if x is None or x != x else f"{x:.{nd}f}"


def render_report(res, kappas):
    """report.txt text, at most 60 lines. Detail for the reference delta_m, one line per other delta_m."""
    ref = res["by_delta"][str(res["delta_ref_deg"])]
    L = []
    psi_bad = res.get("psi_invalid", False)
    if psi_bad:
        L.append("ATTENZIONE: righe con psi != 0 senza margini (rollout psi0): psi* = 0 forzato; "
                 "statistiche di psi* e sensibilita' a psi_max NON VALIDE")
    meta = res.get("meta")
    if meta:
        L.append("scenario: params=" + meta.get("params", "?") + " grasp_radius_m=" + meta.get("grasp_radius_m", "?")
                 + " theta_snap_rad=" + meta.get("theta_snap_rad", "?") + " t_snap_s=" + meta.get("t_snap_s", "?")
                 + " (altri: depth, omega, center, q_start, demo in <csv>.meta)")
    else:
        L.append("metadati assenti: non verificabile lo scenario")
    L.append(f"S3 landscape: rows={res['n_rows']} psi0_rows={res['n_psi0_rows']} | NEW rule: IK ok & path_ok & follow_ik_ok & "
             f"margin_follow>=delta_m & |psi|<={res['psi_max_deg']} (ref delta_m={res['delta_ref_deg']} deg)")
    nc = res["nan_counts"]
    L.append(f"NaN excluded (all rows / IK-ok rows of {nc['n_ik_ok_rows']}): " + " ".join(
        f"{c}={nc['all_rows'][c]}/{nc['ik_ok_rows'][c]}" for c in nc["all_rows"]))
    L.append("a) rejects (psi0 rows, all bits): " + " ".join(f"{k}={v}" for k, v in res["rejects_psi0"].items()))
    L.append("   OLD accepted per k (psi0): " + " ".join(f"k{k}={v}" for k, v in res["accepted_per_k_psi0_old"].items())
             + " | NEW admissible per k: " + " ".join(f"k{k}={v}" for k, v in ref["admissible_per_k_psi0"].items()))
    ov = res["old_vs_new"]
    L.append(f"OLD vs NEW (psi0, ref delta): old accepted={ov['n_old_accepted_psi0']} new admissible={ov['n_new_admissible_psi0']} "
             f"((k,theta) with some psi: {ov['n_new_admissible_kt_with_psi']})")
    v = ov["new_admissible_psi0_violating_old"]
    L.append(f"   new-admissible that violate old thresholds: w_hat<min={v['w_hat']} margine<min={v['margine']} "
             f"R_trust={v['r_trust']} any={v['any']}")
    ow, nw = ov["old_winner"], ov["new_winner"]
    L.append("   old winner (J_current): " + ("none" if ow is None else
             f"k{ow['k']} th={_f(ow['theta_deg'], 1)} d={_f(ow['d_mm'])} mm w_full={_f(ow['w_full'], 5)}"))
    L.append(f"   new winner (J_new, kappa={_f(ov['new_winner_kappa'], 2)} median, psi*): " + ("none" if nw is None else
             f"k{nw['k']} th={_f(nw['theta_deg'], 1)} psi={_f(nw['psi_deg'], 0)} d={_f(nw['d_mm'])} mm "
             f"w_full={_f(nw['w_full'], 5)} violates_old={nw['violates_old'] or 'no'}"))
    L.append("b) per k minimum d (NEW admissible, psi0):")
    for k, p in ref["per_k_min"].items():
        L.append(f"   k{k}: " + ("none" if p is None else
                 f"d={_f(p['d_mm'])} mm th={_f(p['theta_deg'], 1)} w_full={_f(p['w_full'], 5)} "
                 f"m_roll={_f(p['margin_rollout'])} m_fol={_f(p['margin_follow'])}"))
    L.append(f"   spread of d_min across k = {_f(ref['k_equivalence_mm'])} mm")
    L.append(f"c) Pareto (d_mm, -ln w_full): psi0 {len(ref['pareto_psi0'])} pts, psi* {len(ref['pareto_psistar'])} pts")
    for name in ("pareto_psi0", "pareto_psistar"):
        pts = sorted(ref[name], key=lambda p: p["d_mm"])[:4]
        L.append(f"   {name}: " + "; ".join(f"({_f(p['d_mm'], 2)}, {_f(p['neg_ln_w'], 3)} k{p['k']} th{_f(p['theta_deg'], 0)})"
                                            for p in pts))
    g = ref["psi_gain_ln_w"]
    L.append(f"d) psi gain ln w_full: p50={_f(g['p50'], 4)} p90={_f(g['p90'], 4)} p99={_f(g['p99'], 4)} max={_f(g['max'], 4)}")
    L.append("e) argmin J_new with psi* (cur = coincides with the old winner):")
    for kw in kappas:
        e = ref["kappa"].get(str(kw))
        if e is None:
            L.append(f"   kappa={kw}: no candidate")
            continue
        s = (f"   kappa={kw}: k{e['k']} th={_f(e['theta_deg'], 1)} psi={_f(e['psi_deg'], 0)} d={_f(e['d_mm'])} "
             f"w={_f(e['w_full'], 5)} m_roll={_f(e['margin_rollout'])} m_fol={_f(e['margin_follow'])} "
             f"tw={_f(e['t_wait_s'], 1)} cur={'Y' if e['is_current_winner'] else 'N'}")
        if "kappa_star_A_vs_B" in e:
            s += f" k*(A,B)={_f(e['kappa_star_A_vs_B'], 3)}"
        L.append(s)
    L.append("   k*(A,B): J_A=J_B at (d_A-d_B)/(lnw_A-lnw_B); A=1st, B=2nd of that kappa")
    L.append("f) per delta_m [deg]: n admissible psi0 / (k,theta) with some psi; argmin per kappa as k/theta/psi:")
    for dkey, dres in res["by_delta"].items():
        wins = " ".join(f"{kw}:{('-' if dres['kappa'][str(kw)] is None else '%d/%.0f/%+.0f' % (dres['kappa'][str(kw)]['k'], dres['kappa'][str(kw)]['theta_deg'], dres['kappa'][str(kw)]['psi_deg']))}"
                        for kw in kappas)
        L.append(f"   {dkey}: {dres['n_admissible_psi0']}/{dres['n_admissible_all_psi']}  {wins}")
    sp = ref["spearman"]
    L.append(f"g) Spearman ln w_full~margin_follow={_f(sp['ln_w_full_vs_margin_follow'])} w_trans~w_rot={_f(sp['w_trans_vs_w_rot'])}")
    t = ref["tie_band"]
    L.append(f"h) tie band kappa={_f(t['kappa_median'], 2)} tau_J={t['tau_J_mm']} mm: n={t['n']} t_wait spread={_f(t['t_wait_spread_s'], 1)} s")
    sr = ref["seed_ratio"]
    L.append(f"i) seed ratio w_full max/min (n={ref['seed_ratio_n']}): p50={_f(sr['p50'])} p90={_f(sr['p90'])} p99={_f(sr['p99'])} max={_f(sr['max'])}")
    rs, ps = res["resolution"], res["psi_star_stats"]
    L.append(f"j) resolution: min d_theta={_f(rs['min_dtheta_deg'], 3)} deg, {rs['n_distinct_k_theta']} distinct (k,theta), rows/grid {rs['rows_per_grid']}")
    inv = " [NON VALIDE]" if psi_bad else ""
    L.append(f"k) psi*{inv} (ref delta, {ps['n']} (k,theta)): |psi*|=psi_max {_f(ps['frac_at_psi_max'], 3)}, >=0.9 psi_max {_f(ps['frac_ge_90pct'], 3)}, winner psi*={_f(ps['winner_psi_deg'], 1)}")
    sens = res["psi_max_sensitivity"]
    L.append(f"   winner vs psi_max{inv} (kappa_w={_f(sens['kappa_w'], 2)}): " + "; ".join(
        f"{x['psi_max_deg']}:" + ("-" if x["winner"] is None else f"k{x['winner']['k']}/{_f(x['winner']['theta_deg'], 1)}/{_f(x['winner']['psi_deg'], 0)}")
        + ("(capped)" if x["capped"] else "") for x in sens["results"]))
    mp = res["map"]
    L.append("l) winner map k/theta/psi over kappa_w (rows, mm) x kappa_t (cols, mm/s " + ",".join(str(x) for x in mp["kappas_t"]) + "):")
    for kw in mp["kappas_w"]:
        cells = [c for c in mp["cells"] if c["kappa_w"] == kw]
        L.append(f"   kw={kw}: " + " ".join("-" if c["winner"] is None else
                 f"{c['winner']['k']}/{c['winner']['theta_deg']:.0f}/{c['winner']['psi_deg']:+.0f}" for c in cells))
    adv = [c["advantage_other_k_mm"] for c in mp["cells"] if c["winner"] is not None and c["advantage_other_k_mm"] == c["advantage_other_k_mm"]]
    n02 = sum(1 for c in mp["cells"] if c.get("pair_k0k2"))
    L.append(f"m) advantage over best other-k runner-up [mm]: min={_f(min(adv) if adv else float('nan'), 3)} median={_f(float(np.median(adv)) if adv else float('nan'), 3)}; "
             f"cells where winner/runner-up are {{k0,k2}}: {n02}/{len(mp['cells'])} (2nd-in-ranking advantage in summary.csv)")
    L.append("n) tiered (J0 w/o wait, then min t_wait within tau_J) per kappa_w: " + "; ".join(
        f"{kw}:" + ("-" if t is None else f"{t['k']}/{t['theta_deg']:.1f}/{t['psi_deg']:+.0f} tw={t['t_wait_s']:.0f}s n={t['n_in_band']}")
        for kw, t in res["tiered"].items()))
    L.append("o) kappa_t* k0<->k2 = (J0_k2-J0_k0)/(t_k0-t_k2) [pair | envelope] per kappa_w: " + "; ".join(
        f"{kw}:{_f(v['kappa_t_star'], 3)}|{_f(v['envelope'], 3)}" for kw, v in res["kappa_t_star"].items()))
    if len(L) > 60:
        L = L[:59] + ["... truncated at 60 lines, see summary.json"]
    return "\n".join(L) + "\n"


SUMMARY_COLS = ["kind", "delta_m_deg", "kappa_w", "kappa_t", "k", "theta_deg", "psi_deg", "d_mm", "w_full",
                "margin_rollout", "margin_follow", "t_wait_s", "J", "advantage_second_mm", "advantage_other_k_mm",
                "other_k", "pair_k0k2", "is_current_winner", "kappa_star", "kappa_t_star_pair", "kappa_t_star_envelope",
                "n_in_band"]


def write_summary_csv(path, res, kappas):
    """summary.csv, one row per case, `kind` says which:
       jnew   : argmin of J_new (kappa_t = 0, NEW rule) per (delta_m, kappa_w in --kappa-list); kappa_star = exchange
                point with the 2nd candidate;
       map    : winner of J = d + kappa_w(-ln w) + kappa_t t_wait per (kappa_w, kappa_t) cell, with the advantages;
       tiered : two-level winner per kappa_w (J without wait, then min t_wait within tau_J);
       kt_star: critical kappa_t k0<->k2 per kappa_w (pair formula and lower-envelope value)."""
    def blank():
        return {c: "" for c in SUMMARY_COLS}
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=SUMMARY_COLS)
        w.writeheader()
        for dkey, dres in res["by_delta"].items():
            for kw in kappas:
                e = dres["kappa"].get(str(kw))
                if e is None:
                    continue
                r = blank()
                r.update(kind="jnew", delta_m_deg=dres["delta_m_deg"], kappa_w=kw, kappa_t=0, k=e["k"],
                         theta_deg=e["theta_deg"], psi_deg=e["psi_deg"], d_mm=e["d_mm"], w_full=e["w_full"],
                         margin_rollout=e["margin_rollout"], margin_follow=e["margin_follow"], t_wait_s=e["t_wait_s"],
                         J=e["J_new"], is_current_winner=int(e["is_current_winner"]),
                         kappa_star=e.get("kappa_star_A_vs_B", float("nan")))
                w.writerow(r)
        for c in res["map"]["cells"]:
            r = blank()
            r.update(kind="map", delta_m_deg=res["delta_ref_deg"], kappa_w=c["kappa_w"], kappa_t=c["kappa_t"])
            if c["winner"] is not None:
                wn = c["winner"]
                r.update(k=wn["k"], theta_deg=wn["theta_deg"], psi_deg=wn["psi_deg"], d_mm=wn["d_mm"],
                         w_full=wn["w_full"], margin_rollout=wn["margin_rollout"], margin_follow=wn["margin_follow"],
                         t_wait_s=wn["t_wait_s"], J=wn["J"], advantage_second_mm=c["advantage_second_mm"],
                         advantage_other_k_mm=c["advantage_other_k_mm"],
                         other_k="" if c["other_k"] is None else c["other_k"], pair_k0k2=int(c["pair_k0k2"]))
            w.writerow(r)
        for kw, t in res["tiered"].items():
            r = blank()
            r.update(kind="tiered", delta_m_deg=res["delta_ref_deg"], kappa_w=kw, kappa_t=0)
            if t is not None:
                r.update(k=t["k"], theta_deg=t["theta_deg"], psi_deg=t["psi_deg"], d_mm=t["d_mm"], w_full=t["w_full"],
                         margin_rollout=t["margin_rollout"], margin_follow=t["margin_follow"], t_wait_s=t["t_wait_s"],
                         J=t["J"], n_in_band=t["n_in_band"])
            w.writerow(r)
        for kw, v in res["kappa_t_star"].items():
            r = blank()
            r.update(kind="kt_star", delta_m_deg=res["delta_ref_deg"], kappa_w=kw,
                     kappa_t_star_pair=v["kappa_t_star"], kappa_t_star_envelope=v["envelope"])
            w.writerow(r)


def _list(s, cast=float):
    return [cast(x) for x in s.split(",") if x.strip() != ""]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", required=True)
    ap.add_argument("--outdir", required=True)
    ap.add_argument("--kappa-list", default="0,1,2,5,10,20,50,100")
    ap.add_argument("--delta-m-list-deg", default="0,3,6,10,17.19")
    ap.add_argument("--delta-m-ref-deg", type=float, default=6.0)
    ap.add_argument("--psi-max-deg", type=float, default=5.0)
    ap.add_argument("--tau-j-mm", type=float, default=2.0)
    ap.add_argument("--map-kappa-w-list", default=",".join(str(x) for x in KAPPA_W_MAP))
    ap.add_argument("--map-kappa-t-list", default=",".join(str(x) for x in KAPPA_T_MAP))
    a = ap.parse_args(argv)
    rows = read_landscape(a.csv)
    if not rows:
        print("empty csv", file=sys.stderr)
        return 2
    kappas, deltas = _list(a.kappa_list), _list(a.delta_m_list_deg)
    res = analyse(rows, kappas, deltas, a.tau_j_mm, a.delta_m_ref_deg, a.psi_max_deg,
                  _list(a.map_kappa_w_list), _list(a.map_kappa_t_list), meta=read_meta(a.csv))
    os.makedirs(a.outdir, exist_ok=True)
    text = render_report(res, kappas)
    with open(os.path.join(a.outdir, "report.txt"), "w") as f:
        f.write(text)
    with open(os.path.join(a.outdir, "summary.json"), "w") as f:
        json.dump(res, f, indent=1, allow_nan=True)
    write_summary_csv(os.path.join(a.outdir, "summary.csv"), res, kappas)
    sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
