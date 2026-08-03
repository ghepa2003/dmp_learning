#!/usr/bin/env python3
"""Genera l'intera matrice di traiettorie sintetiche per lo sweep:
range di durate (default 30-90s) x un set di goal diversi (spostamenti
orizzontali/verticali diversi), tutte con lo stesso profilo minimum-jerk
"picking" (orizzontale poi verticale, transizione smussata).

Scrive i CSV in una cartella di output, piu' un manifest.csv con i
parametri di ciascun file generato, cosi' da poter incrociare
facilmente durata/goal con i risultati del fit una volta fatto lo sweep
sul lato C++.

Uso:
    python3 generate_sweep_matrix.py --outdir sweep_demos

Personalizza le liste DURATIONS e GOALS qui sotto secondo necessita'.
"""
import argparse
import csv
import os

from generate_picking_trajectory import generate, write_csv

# Range di durate da testare (in secondi). Modifica liberamente.
DURATIONS = [30, 45, 60, 75, 90]

# Goal diversi da testare (dx, dy, dz) in metri. Modifica liberamente.
# Il primo e' il caso "principale" (rappresentativo del task reale);
# gli altri servono a verificare la sensibilita' della DMP a goal
# diversi con la stessa durata.
GOALS = {
    "goalA_main":     (0.15,  0.00, -0.10),
    "goalB_wide":     (0.25,  0.05, -0.12),
    "goalC_narrow":   (0.08, -0.03, -0.06),
    "goalD_lateral":  (0.10,  0.15, -0.08),
}


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--outdir", required=True, help="cartella di output per i CSV generati")
    p.add_argument("--dt", type=float, default=0.001, help="passo di campionamento [s]")
    args = p.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    manifest_path = os.path.join(args.outdir, "manifest.csv")

    with open(manifest_path, "w", newline="") as mf:
        writer = csv.writer(mf)
        writer.writerow(["filename", "duration_s", "goal_name", "dx", "dy", "dz"])

        for duration in DURATIONS:
            for goal_name, (dx, dy, dz) in GOALS.items():
                fname = f"demo_synth_{int(duration)}s_{goal_name}.csv"
                fpath = os.path.join(args.outdir, fname)
                rows = generate(duration=duration, dx=dx, dy=dy, dz=dz, dt=args.dt)
                write_csv(rows, fpath)
                writer.writerow([fname, duration, goal_name, dx, dy, dz])
                print(f"  {fname}: {len(rows)} campioni")

    n_total = len(DURATIONS) * len(GOALS)
    print(f"\nGenerati {n_total} file in {args.outdir}/ (manifest: {manifest_path})")


if __name__ == "__main__":
    main()
