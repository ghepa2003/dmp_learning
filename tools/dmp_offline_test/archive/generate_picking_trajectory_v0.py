#!/usr/bin/env python3
"""Genera una traiettoria sintetica di tipo "picking" (traslazione
orizzontale seguita da traslazione verticale, con transizione smussata)
nello stesso formato CSV usato dalle demo reali (t,x,y,z,qw,qx,qy,qz),
cosi' da poter essere usata direttamente da learnFromDemonstration /
replay_saved_dmp senza modifiche alla pipeline esistente.

Metodologia di smussatura: profilo minimum-jerk (stessa famiglia
matematica di min_jerk_step.m, gia' usata come riferimento nel resto del
progetto) applicato indipendentemente a ciascun asse su una finestra
temporale propria. Le finestre orizzontale e verticale si sovrappongono
parzialmente (di default) per ottenere una transizione dolce invece di
uno spigolo netto tra le due fasi del movimento.

Uso:
    python3 generate_picking_trajectory.py --duration 45 \
        --dx 0.15 --dy 0.05 --dz -0.10 --output demo_synth_45s.csv

    # per goal diversi (stesso spostamento totale, direzioni diverse):
    python3 generate_picking_trajectory.py --duration 45 \
        --dx 0.20 --dy -0.05 --dz -0.12 --output demo_synth_45s_goalB.csv
"""
import argparse
import csv
import math


def min_jerk_profile(t, t0, t1, start, end):
    """Profilo posizione 1D minimum-jerk tra start ed end sulla finestra
    [t0, t1]. Costante = start prima di t0, costante = end dopo t1.
    Velocita' e accelerazione nulle a entrambi gli estremi della finestra
    (continuita' C2), stessa famiglia di min_jerk_step.m."""
    if t <= t0:
        return start
    if t >= t1:
        return end
    s = (t - t0) / (t1 - t0)
    smoothed = 10 * s**3 - 15 * s**4 + 6 * s**5
    return start + (end - start) * smoothed


def generate(duration, dx, dy, dz, dt=0.001,
             horizontal_window=(0.0, 0.6), vertical_window=(0.4, 1.0),
             y0=(0.0, 0.0, 0.0), q0=(1.0, 0.0, 0.0, 0.0)):
    """Genera la traiettoria completa.

    horizontal_window / vertical_window: frazioni di `duration` (0..1)
    che definiscono l'inizio/fine di ciascuna fase. Di default si
    sovrappongono tra 0.4 e 0.6 * duration per una transizione dolce
    (movimento diagonale nella fase di sovrapposizione, tipico di un
    reach-then-lift naturale, non di un percorso a gradino).
    """
    hx0, hx1 = horizontal_window[0] * duration, horizontal_window[1] * duration
    vz0, vz1 = vertical_window[0] * duration, vertical_window[1] * duration

    x0, y0_, z0 = y0
    qw, qx, qy, qz = q0  # orientamento costante per questo generatore

    n_samples = int(round(duration / dt)) + 1
    rows = []
    for k in range(n_samples):
        t = k * dt
        x = min_jerk_profile(t, hx0, hx1, x0, x0 + dx)
        y = min_jerk_profile(t, hx0, hx1, y0_, y0_ + dy)
        z = min_jerk_profile(t, vz0, vz1, z0, z0 + dz)
        rows.append((t, x, y, z, qw, qx, qy, qz))
    return rows


def write_csv(rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "x", "y", "z", "qw", "qx", "qy", "qz"])
        for row in rows:
            writer.writerow([f"{v:.6f}" for v in row])


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--duration", type=float, required=True,
                    help="durata totale della traiettoria in secondi")
    p.add_argument("--dx", type=float, default=0.15, help="spostamento orizzontale asse x [m]")
    p.add_argument("--dy", type=float, default=0.0, help="spostamento orizzontale asse y [m]")
    p.add_argument("--dz", type=float, default=-0.10, help="spostamento verticale asse z [m]")
    p.add_argument("--dt", type=float, default=0.001, help="passo di campionamento [s] (default 1kHz, come il Geomagic Touch)")
    p.add_argument("--h-start-frac", type=float, default=0.0, help="inizio finestra orizzontale, frazione di duration")
    p.add_argument("--h-end-frac", type=float, default=0.6, help="fine finestra orizzontale, frazione di duration")
    p.add_argument("--v-start-frac", type=float, default=0.4, help="inizio finestra verticale, frazione di duration")
    p.add_argument("--v-end-frac", type=float, default=1.0, help="fine finestra verticale, frazione di duration")
    p.add_argument("--output", required=True, help="percorso del CSV di output")
    args = p.parse_args()

    rows = generate(
        duration=args.duration, dx=args.dx, dy=args.dy, dz=args.dz, dt=args.dt,
        horizontal_window=(args.h_start_frac, args.h_end_frac),
        vertical_window=(args.v_start_frac, args.v_end_frac),
    )
    write_csv(rows, args.output)
    print(f"Scritto {args.output}: {len(rows)} campioni, durata {args.duration}s, "
          f"goal finale = ({rows[-1][1]:.4f}, {rows[-1][2]:.4f}, {rows[-1][3]:.4f})")


if __name__ == "__main__":
    main()
