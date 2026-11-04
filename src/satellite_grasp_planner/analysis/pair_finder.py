#!/usr/bin/env python3
"""Pairs of candidates for the L2 validation, from the CSV written by `s3_landscape --all-candidates`.

Criteria (fixed before looking at any data); a pair is two rows of the same scenario, the same k and the same wrist branch with:
  1. V1 passed (v1_ok = 1) and V1' passed (v1p_ok = 1) for both. A blank v1p_ok (V1' not evaluated, e.g. use_hold_end = 0) does not pass.
  2. clearance_mm >= min_clearance_mm + --clearance-buffer-mm (default 2.5, printed in the report) for both. min_clearance_mm: the
     column of the row if filled, else `min_clearance_mm=` of <csv>.meta, else --min-clearance-mm (error if none of them exists).
  3. l2_diverged = 0 for both (a blank value, L2 not evaluated, does not pass).
  4. one with l2_total_deg >= --margin-pos-deg (default 10) and the other with l2_total_deg < --margin-neg-deg (default 0).
  5. |J_s(a) - J_s(b)| <= --j-diff-pct / 100 * min(J_s(a), J_s(b)) (default 20 %).

The script only lists pairs and the reason why every other pair was left out; which pairs to launch is not decided here.

Usage:
  pair_finder.py candidates.csv [more.csv ...] [--out pairs.csv] [--excluded-out excluded.csv]
                 [--clearance-buffer-mm 2.5] [--min-clearance-mm X] [--margin-pos-deg 10] [--margin-neg-deg 0] [--j-diff-pct 20]
"""
import argparse
import csv
import itertools
import math
import sys
from collections import Counter, OrderedDict


def num(row, key):
    v = row.get(key, "")
    if v is None or v == "" or v == "nan":
        return None
    try:
        x = float(v)
    except ValueError:
        return None
    return x if math.isfinite(x) else None


def member_reasons(row, args):
    """Reasons (criteria 1-3) for which this candidate cannot be in any pair."""
    out = []
    if num(row, "v1_ok") != 1:
        out.append("v1_not_passed")
    if num(row, "v1p_ok") != 1:
        out.append("v1p_not_passed")
    clr, thr = num(row, "clearance_mm"), row["_min_clr"]
    if clr is None or thr is None or clr < thr + args.clearance_buffer_mm:
        out.append("clearance_below_min_plus_margin")
    if num(row, "l2_diverged") != 0:
        out.append("l2_diverged_or_not_evaluated")
    return out


def pair_reasons(a, b, args):
    reasons = []
    for name, row in (("a", a), ("b", b)):
        reasons += [f"{name}:{r}" for r in member_reasons(row, args)]
    ta, tb = num(a, "l2_total_deg"), num(b, "l2_total_deg")
    if ta is None or tb is None:
        reasons.append("l2_total_missing")
    else:
        opposite = (ta >= args.margin_pos_deg and tb < args.margin_neg_deg) or (tb >= args.margin_pos_deg and ta < args.margin_neg_deg)
        if not opposite:
            reasons.append("margins_not_opposite")
    ja, jb = num(a, "J_s"), num(b, "J_s")
    if ja is None or jb is None:
        reasons.append("J_s_missing")
    elif abs(ja - jb) > args.j_diff_pct / 100.0 * min(ja, jb):
        reasons.append("J_s_difference_above_limit")
    return reasons


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+", help="CSV of s3_landscape --all-candidates")
    ap.add_argument("--out", help="accepted pairs (default: stdout)")
    ap.add_argument("--excluded-out", help="every excluded pair with its reasons")
    ap.add_argument("--clearance-buffer-mm", type=float, default=2.5, help="buffer added to min_clearance_mm (default 2.5)")
    ap.add_argument("--min-clearance-mm", type=float, default=None, help="used only if neither the CSV nor <csv>.meta has min_clearance_mm")
    ap.add_argument("--margin-pos-deg", type=float, default=10.0)
    ap.add_argument("--margin-neg-deg", type=float, default=0.0)
    ap.add_argument("--j-diff-pct", type=float, default=20.0)
    args = ap.parse_args()

    rows = []
    fields = None
    for path in args.csv:
        with open(path, newline="") as f:
            rd = csv.DictReader(f)
            if fields is None:
                fields = rd.fieldnames
            elif rd.fieldnames != fields:
                sys.exit(f"{path}: columns differ from {args.csv[0]}")
            meta_min = None
            try:
                with open(path + ".meta") as mf:
                    for line in mf:
                        if line.startswith("min_clearance_mm="):
                            meta_min = float(line.split("=", 1)[1])
            except OSError:
                pass
            for r in rd:
                r["_src"] = path
                v = num(r, "min_clearance_mm")
                v = v if v is not None else (meta_min if meta_min is not None else args.min_clearance_mm)
                if v is None:
                    sys.exit(f"{path}: min_clearance_mm is neither in the CSV nor in {path}.meta; pass --min-clearance-mm")
                r["_min_clr"] = v
                rows.append(r)
    if not rows:
        sys.exit("no rows")

    print(f"clearance buffer = {args.clearance_buffer_mm} mm (clearance_mm >= min_clearance_mm + buffer)", file=sys.stderr)
    if all(num(r, "v1p_ok") is None for r in rows):
        print("WARNING: use_hold_end non attivo: nessuna coppia valida (v1p_ok vuoto in tutte le righe)", file=sys.stderr)
    groups = OrderedDict()
    for r in rows:
        groups.setdefault((r["scenario"], r["k"], r["branch"]), []).append(r)

    n_pairs = 0
    accepted = []
    excluded = []
    first_reason = Counter()
    any_reason = Counter()
    for key, g in groups.items():
        for a, b in itertools.combinations(g, 2):
            n_pairs += 1
            reasons = pair_reasons(a, b, args)
            if reasons:
                excluded.append((key, a, b, reasons))
                first_reason[reasons[0].split(":")[-1]] += 1
                for r in set(x.split(":")[-1] for x in reasons):
                    any_reason[r] += 1
            else:
                accepted.append((key, a, b))

    def key_cols(row, prefix):
        return {f"{prefix}_{c}": row[c] for c in fields if c not in ("scenario", "k", "branch", "error")}

    out_fields = ["scenario", "k", "branch", "j_diff_pct"] + [f"{p}_{c}" for p in ("good", "bad") for c in fields if c not in ("scenario", "k", "branch", "error")]
    out = open(args.out, "w", newline="") if args.out else sys.stdout
    w = csv.DictWriter(out, fieldnames=out_fields)
    w.writeheader()
    for (scenario, k, branch), a, b in accepted:
        good, bad = (a, b) if num(a, "l2_total_deg") >= args.margin_pos_deg else (b, a)
        ja, jb = num(a, "J_s"), num(b, "J_s")
        row = {"scenario": scenario, "k": k, "branch": branch, "j_diff_pct": f"{100.0 * abs(ja - jb) / min(ja, jb):.3f}"}
        row.update(key_cols(good, "good"))
        row.update(key_cols(bad, "bad"))
        w.writerow(row)
    if args.out:
        out.close()

    if args.excluded_out:
        with open(args.excluded_out, "w", newline="") as f:
            ew = csv.writer(f)
            ew.writerow(["scenario", "k", "branch", "theta_a_deg", "theta_b_deg", "l2_total_a_deg", "l2_total_b_deg", "J_s_a", "J_s_b", "reasons"])
            for (scenario, k, branch), a, b, reasons in excluded:
                ew.writerow([scenario, k, branch, a["theta_deg"], b["theta_deg"], a["l2_total_deg"], b["l2_total_deg"], a["J_s"], b["J_s"], ";".join(reasons)])

    print(f"rows={len(rows)} groups(scenario,k,branch)={len(groups)} pairs={n_pairs} accepted={len(accepted)} excluded={len(excluded)}", file=sys.stderr)
    print("excluded pairs by FIRST reason:", file=sys.stderr)
    for r, c in first_reason.most_common():
        print(f"  {r}: {c}", file=sys.stderr)
    print("excluded pairs by EVERY reason that applies (a pair can count in several):", file=sys.stderr)
    for r, c in any_reason.most_common():
        print(f"  {r}: {c}", file=sys.stderr)


if __name__ == "__main__":
    main()
