#!/usr/bin/env python3
"""Legge il CSV cumulativo prodotto da learn_and_test_dmp (via
appendToSummaryCsv, formato di metrics.cpp/hpp) e produce i grafici di
studio richiesti:

  1. metrica vs durata di apprendimento, una linea per ciascun goal
     (risponde a: "come cambia la traiettoria rispetto a diversi tempi")
  2. metrica vs goal, a una durata rappresentativa fissata
     (risponde a: "come cambia la traiettoria dando una serie di goal diversi")

Si aspetta che l'etichetta (colonna 'trial') segua il formato prodotto da
generate_sweep_matrix.py: demo_synth_<durata>s_<nome_goal>
(es. demo_synth_45s_goalA_main). Se usi un altro schema di nomi, adatta la
funzione parse_label() qui sotto.

Uso:
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

# Metriche da plottare nello studio vs durata / vs goal, con etichetta leggibile.
METRICS_TO_PLOT = [
    ("rmse_overall_mm", "RMSE posizione totale [mm]"),
    ("max_pos_error_mm", "Errore posizione massimo [mm]"),
    ("endpoint_pos_error_mm", "Errore posizione finale [mm]"),
    ("mean_angular_error_deg", "Errore angolare medio [deg]"),
    ("max_angular_error_deg", "Errore angolare massimo [deg]"),
    ("endpoint_orient_error_deg", "Errore angolare finale [deg]"),
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
                print(f"  [attenzione] etichetta non riconosciuta, salto: {row['trial']}")
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
    ax.set_xlabel("Durata di apprendimento [s]")
    ax.set_ylabel(metric_label)
    ax.set_title(f"{metric_label} vs durata (per goal)")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    out_path = os.path.join(plot_dir, f"sweep_{metric_key}_vs_duration.png")
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def plot_metric_vs_goal(rows, metric_key, metric_label, duration, plot_dir):
    # prende la durata disponibile piu' vicina a quella richiesta
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
    ax.set_title(f"{metric_label} vs goal (durata = {closest:.0f}s)")
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
    p.add_argument("--summary-csv", required=True, help="CSV cumulativo prodotto da learn_and_test_dmp")
    p.add_argument("--plot-dir", default="plot", help="cartella di output per i grafici di studio")
    p.add_argument("--representative-duration", type=float, default=60.0,
                    help="durata (in s) usata per il confronto tra goal diversi; "
                         "verra' usata la durata disponibile piu' vicina")
    args = p.parse_args()

    rows = load_summary(args.summary_csv)
    if not rows:
        print("Nessuna riga valida trovata nel summary CSV.")
        return

    os.makedirs(args.plot_dir, exist_ok=True)

    n_goals = len(set(r["_goal"] for r in rows))
    n_durations = len(set(r["_duration"] for r in rows))
    print(f"Caricate {len(rows)} prove: {n_durations} durate x {n_goals} goal.")

    print("\n== Grafici metrica vs durata (per goal) ==")
    for metric_key, metric_label in METRICS_TO_PLOT:
        out_path = plot_metric_vs_duration(rows, metric_key, metric_label, args.plot_dir)
        print(f"  {out_path}")

    print(f"\n== Grafici metrica vs goal (durata rappresentativa richiesta: {args.representative_duration}s) ==")
    for metric_key, metric_label in METRICS_TO_PLOT:
        result = plot_metric_vs_goal(rows, metric_key, metric_label,
                                      args.representative_duration, args.plot_dir)
        if result:
            out_path, used_duration = result
            print(f"  {out_path}  (durata usata: {used_duration:.0f}s)")


if __name__ == "__main__":
    main()
