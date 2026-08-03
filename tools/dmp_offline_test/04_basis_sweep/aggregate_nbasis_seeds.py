#!/usr/bin/env python3
"""Aggrega, per ciascun n_basis, le serie generate da piu' seed di rumore
(es. "rumoroso (seed 42)", "rumoroso (seed 43)", ...) in media e deviazione
standard, cosi' da non dover leggere a occhio una riga per seed.

Legge il CSV combinato gia' prodotto da plot_nbasis_study.py
(nbasis_all_metrics.csv, formato lungo: series,n_basis,metrica1,metrica2,...).
Non ricalcola nulla di nuovo sul parsing delle etichette nbasis_<N> -- quel
lavoro e' gia' fatto a monte da plot_nbasis_study.py.

Convenzione di raggruppamento: le serie il cui nome contiene "(seed <N>)"
vengono raggruppate per il testo che precede la parentesi (es. tutte le
"rumoroso (seed N)" finiscono nel gruppo "rumoroso"); le altre serie (es.
"pulito", che ha un solo campione e non va aggregata) passano invariate,
riportate come singolo valore senza colonna di deviazione standard.

Uso:
    python3 aggregate_nbasis_seeds.py \
        --combined-csv plots/04_basis_sweep/nbasis_all_metrics.csv \
        --plot-dir plots/04_basis_sweep

Segnala a testo (non sul grafico) i punti dove la deviazione standard tra
seed supera una soglia relativa alla media -- utile per capire se il
minimo apparente nella curva "rumoroso" e' stabile o e' un artefatto del
singolo seed usato nel primo giro di sweep.
"""
import argparse
import csv
import os
import re
import statistics as stats

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SEED_GROUP_RE = re.compile(r"^(.*?)\s*\(seed\s+\d+\)\s*$")

METRICS = [
    ("rmse_overall_mm", "RMSE posizione totale [mm]"),
    ("max_pos_error_mm", "Errore posizione massimo [mm]"),
    ("endpoint_pos_error_mm", "Errore posizione finale [mm]"),
    ("mean_angular_error_deg", "Errore angolare medio [deg]"),
    ("max_angular_error_deg", "Errore angolare massimo [deg]"),
    ("endpoint_orient_error_deg", "Errore angolare finale [deg]"),
]

# Soglia (deviazione standard / media, in %) oltre la quale segnaliamo a
# testo che la stima del minimo in quel punto e' poco affidabile con i
# seed disponibili.
CV_WARN_THRESHOLD_PCT = 15.0


def load_combined(path):
    """Ritorna dict: group_label -> n_basis -> metric_key -> lista di valori
    (piu' di uno se il gruppo e' aggregato da piu' seed)."""
    groups = {}
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            series = row["series"]
            m = SEED_GROUP_RE.match(series)
            group_label = m.group(1) if m else series
            n_basis = int(row["n_basis"])
            groups.setdefault(group_label, {}).setdefault(n_basis, {})
            for metric_key, _ in METRICS:
                groups[group_label][n_basis].setdefault(metric_key, []).append(float(row[metric_key]))
    return groups


def aggregate(groups):
    """Per ciascun gruppo/n_basis/metrica: mean, std (None se un solo
    campione), n_seeds."""
    agg = {}
    for group_label, by_n in groups.items():
        agg[group_label] = {}
        for n_basis, metrics in by_n.items():
            agg[group_label][n_basis] = {}
            for metric_key, values in metrics.items():
                mean = stats.fmean(values)
                std = stats.pstdev(values) if len(values) > 1 else None
                agg[group_label][n_basis][metric_key] = (mean, std, len(values))
    return agg


def write_metric_table(agg, metric_key, plot_dir):
    all_n = sorted({n for by_n in agg.values() for n in by_n})
    group_labels = sorted(agg.keys())

    out_path = os.path.join(plot_dir, f"nbasis_aggregated_{metric_key}.csv")
    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        header = ["n_basis"]
        for label in group_labels:
            header += [f"{label}_mean", f"{label}_std", f"{label}_n_seeds"]
        writer.writerow(header)
        for n in all_n:
            row = [n]
            for label in group_labels:
                entry = agg[label].get(n, {}).get(metric_key)
                if entry is None:
                    row += ["", "", ""]
                else:
                    mean, std, n_seeds = entry
                    row += [f"{mean:.6g}", f"{std:.6g}" if std is not None else "", n_seeds]
            writer.writerow(row)
    return out_path


def plot_metric(agg, metric_key, metric_label, plot_dir):
    fig, ax = plt.subplots(figsize=(9, 5.5))
    colors = ["tab:blue", "tab:red", "tab:green", "tab:orange", "tab:purple"]
    for i, group_label in enumerate(sorted(agg.keys())):
        by_n = agg[group_label]
        n_sorted = sorted(by_n.keys())
        means = [by_n[n][metric_key][0] for n in n_sorted]
        stds = [by_n[n][metric_key][1] for n in n_sorted]
        color = colors[i % len(colors)]
        if any(s is not None for s in stds):
            # gruppo aggregato da piu' seed: error bar = deviazione standard
            stds_plot = [s if s is not None else 0.0 for s in stds]
            ax.errorbar(n_sorted, means, yerr=stds_plot, marker="o", color=color,
                        label=group_label, capsize=3)
        else:
            # serie a campione singolo (es. "pulito"): linea semplice
            ax.plot(n_sorted, means, marker="o", color=color, label=group_label)

    ax.set_xlabel("Numero di basi (n_basis)")
    ax.set_ylabel(metric_label)
    ax.set_title(f"{metric_label} vs n_basis (media \u00b1 std tra seed)")
    ax.set_yscale("log")
    ax.grid(alpha=0.3, which="both")
    ax.legend()
    fig.tight_layout()
    out_path = os.path.join(plot_dir, f"nbasis_aggregated_{metric_key}.png")
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def report_high_variance_points(agg, metric_key, metric_label):
    """Stampa a testo i punti dove std/mean supera CV_WARN_THRESHOLD_PCT,
    solo per gruppi aggregati da piu' di un seed."""
    flagged = []
    for group_label, by_n in agg.items():
        for n_basis, entry in sorted(by_n.items()):
            mean, std, n_seeds = entry[metric_key]
            if std is None or n_seeds < 2 or mean <= 1e-12:
                continue
            cv_pct = 100.0 * std / mean
            if cv_pct > CV_WARN_THRESHOLD_PCT:
                flagged.append((group_label, n_basis, cv_pct, n_seeds))
    if flagged:
        print(f"  [ATTENZIONE] variabilita' alta tra seed ({metric_label}):")
        for group_label, n_basis, cv_pct, n_seeds in flagged:
            print(f"    [{group_label}] n_basis={n_basis}: std/media = {cv_pct:.1f}% "
                  f"(su {n_seeds} seed) -- il valore puntuale a questo n_basis "
                  f"e' poco affidabile con i seed disponibili")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--combined-csv", required=True,
                    help="nbasis_all_metrics.csv prodotto da plot_nbasis_study.py")
    p.add_argument("--plot-dir", default="plot", help="cartella di output per tabelle e grafici")
    args = p.parse_args()

    groups = load_combined(args.combined_csv)
    for label, by_n in groups.items():
        n_seeds_seen = {len(v) for metrics in by_n.values() for v in metrics.values()}
        print(f"[{label}] n_basis testati: {sorted(by_n.keys())}, "
              f"campioni per punto: {sorted(n_seeds_seen)}")

    agg = aggregate(groups)
    os.makedirs(args.plot_dir, exist_ok=True)

    print("\n== Tabelle e grafici aggregati per metrica ==")
    for metric_key, metric_label in METRICS:
        print(f"\n  {metric_label}:")
        report_high_variance_points(agg, metric_key, metric_label)
        table_path = write_metric_table(agg, metric_key, args.plot_dir)
        plot_path = plot_metric(agg, metric_key, metric_label, args.plot_dir)
        print(f"    tabella: {table_path}")
        print(f"    grafico: {plot_path}")


if __name__ == "__main__":
    main()
