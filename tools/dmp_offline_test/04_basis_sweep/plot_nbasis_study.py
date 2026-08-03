#!/usr/bin/env python3
"""Legge uno o piu' CSV prodotti da run_nbasis_sweep.sh (formato di
metrics.cpp/hpp, etichette 'nbasis_<N>') e produce, per ciascuna metrica:

  1. un grafico con una linea per ciascuna serie fornita (es. "pulito" vs
     "rumoroso"), in scala logaritmica -- nessuna individuazione automatica
     di gomito, solo la curva
  2. una tabella CSV (n_basis in riga, una colonna per serie) per
     l'ispezione diretta dei numeri, senza dover aprire i CSV grezzi di
     ciascuna serie separatamente

Segnala comunque a testo (non sul grafico) se una metrica RISALE (>10%)
rispetto al valore precedente in una serie -- utile per individuare a
occhio, incrociandolo con la tabella, dove eventualmente ispezionare i
pesi nello YAML corrispondente per un possibile ill-conditioning.

Uso (una sola serie):
    python3 plot_nbasis_study.py --summary-csv plot/nbasis_sweep_results.csv \
        --plot-dir plot

Uso (piu' serie sovrapposte, es. pulito vs rumoroso):
    python3 plot_nbasis_study.py \
        --summary-csv plot/nbasis_sweep_results_clean.csv --series-label pulito \
        --summary-csv plot/nbasis_sweep_results_noisy.csv --series-label rumoroso \
        --plot-dir plot
"""
import argparse
import csv
import os
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LABEL_RE = re.compile(r"^nbasis_(\d+)$")

METRICS_TO_PLOT = [
    ("rmse_overall_mm", "RMSE posizione totale [mm]"),
    ("max_pos_error_mm", "Errore posizione massimo [mm]"),
    ("endpoint_pos_error_mm", "Errore posizione finale [mm]"),
    ("mean_angular_error_deg", "Errore angolare medio [deg]"),
    ("max_angular_error_deg", "Errore angolare massimo [deg]"),
    ("endpoint_orient_error_deg", "Errore angolare finale [deg]"),
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
                print(f"  [attenzione] etichetta non riconosciuta, salto: {row['trial']}")
                continue
            row["_n_basis"] = int(m.group(1))
            rows.append(row)
    rows.sort(key=lambda r: r["_n_basis"])
    return rows


def detect_instability(rows, metric_key):
    """Segnala (solo a testo) le risalite (>10%) della metrica rispetto al
    valore precedente, senza calcolare alcun 'gomito'."""
    values = [(r["_n_basis"], float(r[metric_key])) for r in rows]
    increases = []
    for i in range(1, len(values)):
        prev_n, prev_v = values[i - 1]
        n, v = values[i]
        if prev_v > 1e-12 and v > prev_v * 1.10:
            increases.append((prev_n, n, prev_v, v))
    return increases


def plot_metric_multi_series(series_data, metric_key, metric_label, plot_dir):
    fig, ax = plt.subplots(figsize=(9, 5.5))
    for i, (series_label, rows) in enumerate(series_data):
        color = SERIES_COLORS[i % len(SERIES_COLORS)]
        n_basis_vals = [r["_n_basis"] for r in rows]
        values = [float(r[metric_key]) for r in rows]
        ax.plot(n_basis_vals, values, marker="o", color=color, label=series_label)

    ax.set_xlabel("Numero di basi (n_basis)")
    ax.set_ylabel(metric_label)
    ax.set_title(f"{metric_label} vs n_basis")
    ax.set_yscale("log")
    ax.grid(alpha=0.3, which="both")
    ax.legend()
    fig.tight_layout()
    out_path = os.path.join(plot_dir, f"nbasis_{metric_key}.png")
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def write_metric_table(series_data, metric_key, plot_dir):
    """Scrive un CSV con n_basis in riga e una colonna per serie, per
    l'ispezione diretta dei numeri."""
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
    """Scrive un unico CSV con TUTTE le metriche e TUTTE le serie insieme
    (formato lungo: n_basis, serie, metrica1, metrica2, ...), utile per chi
    preferisce un solo file da aprire invece di uno per metrica."""
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
                    help="CSV prodotto da run_nbasis_sweep.sh (puo' essere ripetuto per piu' serie)")
    p.add_argument("--series-label", action="append", default=[],
                    help="etichetta per ciascun --summary-csv, nello stesso ordine "
                         "(default: nome del file se omesso)")
    p.add_argument("--plot-dir", default="plot", help="cartella di output per grafici e tabelle")
    args = p.parse_args()

    labels = list(args.series_label)
    while len(labels) < len(args.summary_csv):
        idx = len(labels)
        labels.append(os.path.splitext(os.path.basename(args.summary_csv[idx]))[0])

    series_data = []
    for csv_path, series_label in zip(args.summary_csv, labels):
        rows = load_summary(csv_path)
        if not rows:
            print(f"[{series_label}] Nessuna riga valida trovata in {csv_path}, salto.")
            continue
        series_data.append((series_label, rows))
        n_basis_tested = [r["_n_basis"] for r in rows]
        print(f"[{series_label}] Caricate {len(rows)} prove, n_basis testati: {n_basis_tested}")

    if not series_data:
        print("Nessuna serie valida da elaborare.")
        return

    os.makedirs(args.plot_dir, exist_ok=True)

    print("\n== Grafici e tabelle per metrica ==")
    for metric_key, metric_label in METRICS_TO_PLOT:
        print(f"\n  {metric_label}:")
        for series_label, rows in series_data:
            increases = detect_instability(rows, metric_key)
            if increases:
                print(f"    [{series_label}] [ATTENZIONE] la metrica RISALE (>10%) in questi "
                      f"intervalli (possibile ill-conditioning, verifica lo YAML corrispondente):")
                for prev_n, n, prev_v, v in increases:
                    print(f"      n_basis {prev_n} -> {n}: {prev_v:.4f} -> {v:.4f}")

        plot_path = plot_metric_multi_series(series_data, metric_key, metric_label, args.plot_dir)
        table_path = write_metric_table(series_data, metric_key, args.plot_dir)
        print(f"    grafico: {plot_path}")
        print(f"    tabella: {table_path}")

    combined_path = write_combined_table(series_data, args.plot_dir)
    print(f"\nTabella unica con tutte le metriche/serie: {combined_path}")


if __name__ == "__main__":
    main()
