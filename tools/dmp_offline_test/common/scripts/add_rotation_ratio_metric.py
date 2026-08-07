#!/usr/bin/env python3
"""Calcola il rapporto rotazione cumulativa/netta dai replay CSV prodotti
da run_filter_window_sweep.sh, e aggiunge questa colonna al CSV combinato
prodotto da plot_nbasis_study.py, per farla fluire nella stessa pipeline
di aggregazione/plot di aggregate_nbasis_seeds.py.

Uso:
    python3 add_rotation_ratio_metric.py \
        --combined-csv 04_basis_sweep/plots/05_filter_window_sweep/ridge/nbasis_all_metrics.csv \
        --replay-dir data
"""
import argparse
import csv
import math
import glob
import os
import re


def quat_angle(q1, q2):
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.degrees(math.acos(dot))


def cumulative_over_net(replay_csv_path):
    with open(replay_csv_path) as f:
        reader = csv.DictReader(f)
        quats = [(float(r["qw"]), float(r["qx"]), float(r["qy"]), float(r["qz"])) for r in reader]
    if len(quats) < 2:
        return None
    cum = sum(quat_angle(quats[i - 1], quats[i]) for i in range(1, len(quats)))
    net = quat_angle(quats[0], quats[-1])
    return cum / net if net > 1e-6 else None


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--combined-csv", required=True)
    p.add_argument("--replay-dir", default="data")
    args = p.parse_args()

    replay_dir = args.replay_dir
    if not os.path.exists(replay_dir) and os.path.exists("tools/dmp_offline_test/data"):
        replay_dir = "tools/dmp_offline_test/data"

    rows = list(csv.DictReader(open(args.combined_csv)))
    if not rows:
        print(f"No rows found in {args.combined_csv}")
        return

    fieldnames = list(rows[0].keys())
    if "rot_cum_over_net_ratio" not in fieldnames:
        fieldnames.append("rot_cum_over_net_ratio")

    for row in rows:
        row_series = row["series"]
        win_m = re.search(r"window=([\d.]+)", row_series) or re.search(r"w=([\d.]+)", row_series)
        traj_m = re.search(r"(traj[A-Z0-9]+|seed\d+)", row_series)
        mode_m = "ridge" if "Ridge" in row_series else ("noridge" if "LWR" in row_series or "noridge" in row_series else "")

        if not win_m:
            row["rot_cum_over_net_ratio"] = ""
            continue
        window = win_m.group(1)
        traj_id = traj_m.group(1) if traj_m else "trajA"
        pattern = os.path.join(replay_dir, f"replay_nbasis_{int(row['n_basis']):04d}_*{traj_id}*{mode_m}*w{window}*.csv")
        combo_guess = glob.glob(pattern)
        if not combo_guess:
            row["rot_cum_over_net_ratio"] = ""
            continue
        ratio = cumulative_over_net(combo_guess[0])
        row["rot_cum_over_net_ratio"] = f"{ratio:.4f}" if ratio is not None else ""

    with open(args.combined_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"Aggiunta colonna rot_cum_over_net_ratio a {args.combined_csv}")


if __name__ == "__main__":
    main()