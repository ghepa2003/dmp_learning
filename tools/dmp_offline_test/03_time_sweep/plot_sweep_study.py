#!/usr/bin/env python3
"""Reads the cumulative CSV produced by learn_and_test_dmp (via
appendToSummaryCsv, format from metrics.cpp/hpp) and produces study plots:

  1. Metric vs learning duration, one line per goal.
  2. Metric vs goal, at a fixed representative duration.

Expects trial labels to follow format produced by generate_sweep_matrix.py:
demo_synth_<duration>s_<goal_name> (e.g. demo_synth_45s_goalA_main).

Usage:
    python3 plot_sweep_study.py --summary-csv plot/sweep_results.csv \
        --plot-dir plot --representative-duration 60
"""
import argparse
import csv
import os
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LABEL_RE = re.compile(r"^demo_synth_(\d+(?:\.\d+)?)s_(.+)$")

METRICS_TO_PLOT = [
    ("rmse_overall_mm", "Total Position RMSE [mm]"),
    ("max_pos_error_mm", "Max Position Error [mm]"),
    ("endpoint_pos_error_mm", "Final Position Error [mm]"),
    ("mean_angular_error_deg", "Mean Angular Error [deg]"),
    ("max_angular_error_deg", "Max Angular Error [deg]"),
    ("endpoint_orient_error_deg", "Final Angular Error [deg]"),
]


def parse_label(label):
    m = LABEL_RE.match(label)
    if not m:
        return None, None
    duration = float(m.group(1))
    goal_name = m.group(2)
    return duration, goal_name


def load_summary(path):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            duration, goal_name = parse_label(row["trial"])
            if duration is None:
                print(f"  [warning] Unrecognized label, skipping: {row['trial']}")
                continue
            row["_duration"] = duration
            row["_goal"] = goal_name
            rows.append(row)
    return rows


def plot_metric_vs_duration(rows, metric_key, metric_label, plot_dir):
    goals = sorted(set(r["_goal"] for r in rows))
    fig, ax = plt.subplots(figsize=(8, 5))
    for goal in goals:
        pts = sorted(((r["_duration"], float(r[metric_key])) for r in rows if r["_goal"] == goal),
                     key=lambda p: p[0])
        if not pts:
            continue
        durations, values = zip(*pts)
        ax.plot(durations, values, marker="o", label=goal)
    ax.set_xlabel("Learning duration [s]")
    ax.set_ylabel(metric_label)
    ax.set_title(f"{metric_label} vs Duration (by goal)")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    out_path = os.path.join(plot_dir, f"sweep_{metric_key}_vs_duration.png")
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def plot_metric_vs_goal(rows, metric_key, metric_label, duration, plot_dir):
    available_durations = sorted(set(r["_duration"] for r in rows))
    if not available_durations:
        return None
    closest = min(available_durations, key=lambda d: abs(d - duration))

    subset = [r for r in rows if r["_duration"] == closest]
    subset.sort(key=lambda r: r["_goal"])
    goals = [r["_goal"] for r in subset]
    values = [float(r[metric_key]) for r in subset]

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.bar(goals, values, color="tab:red", alpha=0.8)
    ax.set_ylabel(metric_label)
    ax.set_title(f"{metric_label} vs Goal (duration = {closest:.0f}s)")
    ax.tick_params(axis="x", rotation=20)
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out_path = os.path.join(plot_dir, f"sweep_{metric_key}_vs_goal_{closest:.0f}s.png")
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path, closest


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--summary-csv", required=True, help="Cumulative CSV produced by learn_and_test_dmp")
    p.add_argument("--plot-dir", default="plot", help="output directory for study plots")
    p.add_argument("--representative-duration", type=float, default=60.0,
                    help="duration (in s) used for goal comparison; closest available duration will be used")
    args = p.parse_args()

    rows = load_summary(args.summary_csv)
    if not rows:
        print("No valid rows found in summary CSV.")
        return

    os.makedirs(args.plot_dir, exist_ok=True)

    n_goals = len(set(r["_goal"] for r in rows))
    n_durations = len(set(r["_duration"] for r in rows))
    print(f"Loaded {len(rows)} trials: {n_durations} durations x {n_goals} goals.")

    print("\n== Metric vs duration plots (per goal) ==")
    for metric_key, metric_label in METRICS_TO_PLOT:
        out_path = plot_metric_vs_duration(rows, metric_key, metric_label, args.plot_dir)
        print(f"  {out_path}")

    print(f"\n== Metric vs goal plots (requested representative duration: {args.representative_duration}s) ==")
    for metric_key, metric_label in METRICS_TO_PLOT:
        result = plot_metric_vs_goal(rows, metric_key, metric_label,
                                      args.representative_duration, args.plot_dir)
        if result:
            out_path, used_duration = result
            print(f"  {out_path}  (duration used: {used_duration:.0f}s)")


if __name__ == "__main__":
    main()

