#!/usr/bin/env python3
"""Reads one or more CSVs produced by run_nbasis_sweep.sh (format from
metrics.cpp/hpp, labels 'nbasis_<N>') and produces, for each metric:

  1. A plot with a line for each provided series (e.g. "clean" vs "noisy"),
     in logarithmic scale.
  2. A CSV table (n_basis per row, one column per series) for direct number
     inspection without opening raw CSVs separately.

Reports in text (not on the plot) if a metric INCREASES (>10%) relative to
its previous value in a series -- useful for spotting potential ill-conditioning.

Usage (single series):
    python3 plot_nbasis_study.py --summary-csv plot/nbasis_sweep_results.csv \
        --plot-dir plot

Usage (multiple overlaid series, e.g. clean vs noisy):
    python3 plot_nbasis_study.py \
        --summary-csv plot/nbasis_sweep_results_clean.csv --series-label clean \
        --summary-csv plot/nbasis_sweep_results_noisy.csv --series-label noisy \
        --plot-dir plot
"""
import argparse
import csv
import os
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LABEL_RE = re.compile(r"^nbasis_(\d+)(?:_.*)?$")

METRICS_TO_PLOT = [
    ("rmse_overall_mm", "Total Position RMSE [mm]"),
    ("max_pos_error_mm", "Max Position Error [mm]"),
    ("endpoint_pos_error_mm", "Final Position Error [mm]"),
    ("mean_angular_error_deg", "Mean Angular Error [deg]"),
    ("max_angular_error_deg", "Max Angular Error [deg]"),
    ("endpoint_orient_error_deg", "Final Angular Error [deg]"),
]

SERIES_COLORS = ["tab:blue", "tab:red", "tab:green", "tab:orange", "tab:purple",
                 "tab:brown", "tab:pink", "tab:gray", "tab:olive", "tab:cyan"]


def load_summary(path):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            m = LABEL_RE.match(row["trial"])
            if not m:
                print(f"  [warning] Unrecognized label, skipping: {row['trial']}")
                continue
            row["_n_basis"] = int(m.group(1))
            rows.append(row)
    rows.sort(key=lambda r: r["_n_basis"])
    return rows


def detect_instability(rows, metric_key):
    """Reports (text only) metric increases (>10%) relative to previous value."""
    values = [(r["_n_basis"], float(r[metric_key])) for r in rows]
    increases = []
    for i in range(1, len(values)):
        prev_n, prev_v = values[i - 1]
        n, v = values[i]
        if prev_v > 1e-12 and v > prev_v * 1.10:
            increases.append((prev_n, n, prev_v, v))
    return increases


def plot_metric_multi_series(series_data, metric_key, metric_label, plot_dir, title_suffix=""):
    fig, ax = plt.subplots(figsize=(9, 5.5))
    for i, (series_label, rows) in enumerate(series_data):
        color = SERIES_COLORS[i % len(SERIES_COLORS)]
        n_basis_vals = [r["_n_basis"] for r in rows]
        values = [float(r[metric_key]) for r in rows]
        ax.plot(n_basis_vals, values, marker="o", color=color, label=series_label)

    ax.set_xlabel("Number of basis functions (n_basis)")
    ax.set_ylabel(metric_label)
    ax.set_title(f"{metric_label} vs n_basis{title_suffix}")
    ax.set_yscale("log")
    ax.grid(alpha=0.3, which="both")
    ax.legend()
    fig.tight_layout()
    out_path = os.path.join(plot_dir, f"nbasis_{metric_key}.png")
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def write_metric_table(series_data, metric_key, plot_dir):
    """Writes a CSV with n_basis per row and one column per series."""
    all_n = sorted(set(r["_n_basis"] for _, rows in series_data for r in rows))
    lookup = {
        series_label: {r["_n_basis"]: r[metric_key] for r in rows}
        for series_label, rows in series_data
    }

    out_path = os.path.join(plot_dir, f"nbasis_table_{metric_key}.csv")
    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["n_basis"] + [label for label, _ in series_data])
        for n in all_n:
            row = [n] + [lookup[label].get(n, "") for label, _ in series_data]
            writer.writerow(row)
    return out_path


def write_combined_table(series_data, plot_dir):
    """Writes a single CSV with ALL metrics and ALL series together."""
    out_path = os.path.join(plot_dir, "nbasis_all_metrics.csv")
    metric_keys = [k for k, _ in METRICS_TO_PLOT]
    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["series", "n_basis"] + metric_keys)
        for series_label, rows in series_data:
            for r in rows:
                writer.writerow([series_label, r["_n_basis"]] + [r[k] for k in metric_keys])
    return out_path


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--summary-csv", action="append", required=True,
                    help="CSV produced by run_nbasis_sweep.sh (can be repeated for multiple series)")
    p.add_argument("--series-label", action="append", default=[],
                    help="label for each --summary-csv in the same order (default: filename if omitted)")
    p.add_argument("--plot-dir", default="plot", help="output directory for plots and tables")
    p.add_argument("--title-suffix", default="", help="suffix to append to plot titles (e.g. ' (Ridge Regression)')")
    args = p.parse_args()

    labels = list(args.series_label)
    while len(labels) < len(args.summary_csv):
        idx = len(labels)
        labels.append(os.path.splitext(os.path.basename(args.summary_csv[idx]))[0])

    series_data = []
    for csv_path, series_label in zip(args.summary_csv, labels):
        rows = load_summary(csv_path)
        if not rows:
            print(f"[{series_label}] No valid rows found in {csv_path}, skipping.")
            continue
        series_data.append((series_label, rows))
        n_basis_tested = [r["_n_basis"] for r in rows]
        print(f"[{series_label}] Loaded {len(rows)} trials, n_basis tested: {n_basis_tested}")

    if not series_data:
        print("No valid series to process.")
        return

    os.makedirs(args.plot_dir, exist_ok=True)

    print("\n== Plots and tables per metric ==")
    for metric_key, metric_label in METRICS_TO_PLOT:
        print(f"\n  {metric_label}:")
        for series_label, rows in series_data:
            increases = detect_instability(rows, metric_key)
            if increases:
                print(f"    [{series_label}] [WARNING] metric INCREASES (>10%) in these "
                      f"intervals (possible ill-conditioning, check corresponding YAML):")
                for prev_n, n, prev_v, v in increases:
                    print(f"      n_basis {prev_n} -> {n}: {prev_v:.4f} -> {v:.4f}")

        plot_path = plot_metric_multi_series(series_data, metric_key, metric_label, args.plot_dir, args.title_suffix)
        table_path = write_metric_table(series_data, metric_key, args.plot_dir)
        print(f"    plot: {plot_path}")
        print(f"    table: {table_path}")

    combined_path = write_combined_table(series_data, args.plot_dir)
    print(f"\nSingle combined table for all metrics/series: {combined_path}")


if __name__ == "__main__":
    main()
