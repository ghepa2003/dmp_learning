#!/usr/bin/env python3
"""Plot dello sweep 1D su ridge_lambda (summary CSV prodotto da
run_ridge_lambda_sweep.sh, label nel formato 'lambda_<valore>').

Uso:
    python3 plot_ridge_lambda_sweep.py \
        --summary-csv plots/06_ridge_lambda_sweep/ridge_lambda_results.csv \
        --plot-dir plots/06_ridge_lambda_sweep
"""
import argparse
import csv
import os
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LABEL_RE = re.compile(r"^lambda_(.+)$")

METRICS_TO_PLOT = [
    ("rmse_overall_mm", "Total Position RMSE [mm]"),
    ("max_pos_error_mm", "Max Position Error [mm]"),
    ("endpoint_pos_error_mm", "Final Position Error [mm]"),
    ("mean_angular_error_deg", "Mean Angular Error [deg]"),
    ("max_angular_error_deg", "Max Angular Error [deg]"),
    ("endpoint_orient_error_deg", "Final Angular Error [deg]"),
]


def load_summary(path):
    rows = []
    with open(path) as f:
        for row in csv.DictReader(f):
            m = LABEL_RE.match(row["trial"])
            if not m:
                continue
            row["_lambda"] = float(m.group(1))
            rows.append(row)
    rows.sort(key=lambda r: r["_lambda"])
    return rows


def plot_metric(rows, metric_key, metric_label, plot_dir):
    fig, ax = plt.subplots(figsize=(8, 5))
    lambdas = [r["_lambda"] for r in rows]
    values = [float(r[metric_key]) for r in rows]
    ax.plot(lambdas, values, marker="o", color="tab:blue")
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("ridge_lambda")
    ax.set_ylabel(metric_label)
    ax.set_title(f"{metric_label} vs ridge_lambda (n_basis=200, window=0.20s)")
    ax.grid(alpha=0.3, which="both")
    fig.tight_layout()
    out_path = os.path.join(plot_dir, f"ridge_lambda_{metric_key}.png")
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def write_table(rows, plot_dir):
    out_path = os.path.join(plot_dir, "ridge_lambda_table.csv")
    metric_keys = [k for k, _ in METRICS_TO_PLOT]
    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["ridge_lambda"] + metric_keys)
        for r in rows:
            writer.writerow([r["_lambda"]] + [r[k] for k in metric_keys])
    return out_path


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--summary-csv", required=True)
    p.add_argument("--plot-dir", default="plot")
    args = p.parse_args()

    rows = load_summary(args.summary_csv)
    if not rows:
        print("Nessuna riga valida trovata.")
        return

    os.makedirs(args.plot_dir, exist_ok=True)
    print(f"Lambda testati: {[r['_lambda'] for r in rows]}")

    for metric_key, metric_label in METRICS_TO_PLOT:
        plot_path = plot_metric(rows, metric_key, metric_label, args.plot_dir)
        print(f"  {metric_label}: {plot_path}")

    table_path = write_table(rows, args.plot_dir)
    print(f"\nTabella: {table_path}")


if __name__ == "__main__":
    main()