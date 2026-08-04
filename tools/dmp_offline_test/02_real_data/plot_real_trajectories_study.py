#!/usr/bin/env python3
"""Grafici comparativi per l'analisi delle traiettorie reali (Traj A, B, C)
confrontando i due metodi di regressione in REGRESSION_VARIANTS (es. LWR vs Ridge).

Legge il CSV di riepilogo generato da run_real_trajectories_sweep.sh e produce:
  1. Grafici a barre raggruppate (Traiettoria A, B, C x Variante di regressione) per ciascuna metrica
  2. Tabella riassuntiva CSV per confronto diretto
"""

import argparse
import csv
import os
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

METRICS_TO_PLOT = [
    ("rmse_overall_mm", "RMSE posizione totale [mm]"),
    ("max_pos_error_mm", "Errore posizione massimo [mm]"),
    ("endpoint_pos_error_mm", "Errore posizione finale [mm]"),
    ("mean_angular_error_deg", "Errore angolare medio [deg]"),
    ("max_angular_error_deg", "Errore angolare massimo [deg]"),
    ("endpoint_orient_error_deg", "Errore angolare finale [deg]"),
]

# Formato trial previsto: real_traj<ID>_<variant> (es. real_trajA_lwr, real_trajB_ridge)
TRIAL_RE = re.compile(r"^real_(traj[A-Z0-9]+)_(.+)$")


def parse_trial(trial_str):
    m = TRIAL_RE.match(trial_str)
    if not m:
        return None, None
    return m.group(1), m.group(2)


def load_summary(path):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            traj_id, variant = parse_trial(row["trial"])
            if traj_id is None:
                # Prova fallback se trial ha un formato diverso (es. trajA_lwr)
                parts = row["trial"].split("_")
                if len(parts) >= 2:
                    traj_id = parts[0]
                    variant = "_".join(parts[1:])
                else:
                    continue
            row["_traj_id"] = traj_id
            row["_variant"] = variant
            rows.append(row)
    return rows


def plot_grouped_bar(rows, metric_key, metric_label, plot_dir):
    trajectories = sorted(list(set(r["_traj_id"] for r in rows)))
    variants = sorted(list(set(r["_variant"] for r in rows)))

    if not trajectories or not variants:
        return None

    # Mappa: traj -> variant -> val
    data_map = {}
    for r in rows:
        t_id = r["_traj_id"]
        v_id = r["_variant"]
        val = float(r[metric_key])
        data_map.setdefault(t_id, {})[v_id] = val

    x = np.arange(len(trajectories))
    width = 0.35 if len(variants) == 2 else 0.8 / len(variants)

    fig, ax = plt.subplots(figsize=(9, 5.5))
    colors = ["tab:blue", "tab:orange", "tab:green", "tab:purple", "tab:red"]

    for i, var_name in enumerate(variants):
        vals = [data_map.get(t_id, {}).get(var_name, 0.0) for t_id in trajectories]
        offset = (i - (len(variants) - 1) / 2) * width
        rects = ax.bar(x + offset, vals, width, label=var_name, color=colors[i % len(colors)], alpha=0.85)

        # Aggiungi etichette dei valori sopra le barre
        for rect in rects:
            height = rect.get_height()
            ax.annotate(f"{height:.3f}",
                        xy=(rect.get_x() + rect.get_width() / 2, height),
                        xytext=(0, 3),  # 3 points vertical offset
                        textcoords="offset points",
                        ha="center", va="bottom", fontsize=8)

    ax.set_xlabel("Traiettoria Reale")
    ax.set_ylabel(metric_label)
    ax.set_title(f"Confronto {metric_label} per Traiettoria Reale e Metodo")
    ax.set_xticks(x)
    ax.set_xticklabels(trajectories)
    ax.legend(title="Variante")
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()

    out_path = os.path.join(plot_dir, f"real_traj_comp_{metric_key}.png")
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def write_comparison_table(rows, plot_dir):
    trajectories = sorted(list(set(r["_traj_id"] for r in rows)))
    variants = sorted(list(set(r["_variant"] for r in rows)))

    out_path = os.path.join(plot_dir, "real_trajectories_comparison_table.csv")
    metric_keys = [k for k, _ in METRICS_TO_PLOT]

    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["trajectory", "variant"] + metric_keys)
        for t_id in trajectories:
            for v_id in variants:
                match = [r for r in rows if r["_traj_id"] == t_id and r["_variant"] == v_id]
                if match:
                    r = match[0]
                    writer.writerow([t_id, v_id] + [r[k] for k in metric_keys])
    return out_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary-csv", required=True, help="CSV riassuntivo prodotto dallo sweep")
    parser.add_argument("--plot-dir", default="plots/02_real_data", help="Cartella di output per i grafici")
    args = parser.parse_args()

    rows = load_summary(args.summary_csv)
    if not rows:
        print("Nessuna riga valida trovata nel summary CSV.")
        return

    os.makedirs(args.plot_dir, exist_ok=True)

    print("\n== Generazione grafici comparativi traiettorie reali ==")
    for metric_key, metric_label in METRICS_TO_PLOT:
        out_p = plot_grouped_bar(rows, metric_key, metric_label, args.plot_dir)
        if out_p:
            print(f"  Grafico salvato: {out_p}")

    tbl_p = write_comparison_table(rows, args.plot_dir)
    print(f"\nTabella riassuntiva salvata: {tbl_p}")


if __name__ == "__main__":
    main()
