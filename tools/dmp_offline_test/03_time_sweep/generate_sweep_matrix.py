#!/usr/bin/env python3
"""Generates the full matrix of synthetic trajectories for the sweep:
range of durations (default 30-90s) x a set of different goals, all with
the same minimum-jerk "picking" profile.

Writes CSVs into an output directory, plus a manifest.csv with parameters
of each generated file.

Usage:
    python3 generate_sweep_matrix.py --outdir sweep_demos
"""
import argparse
import csv
import os

from generate_picking_trajectory import generate, write_csv

# Range of durations to test (in seconds).
DURATIONS = [30, 45, 60, 75, 90]

# ABSOLUTE duration (in seconds) of horizontal+vertical movement.
TRANSITION_DURATION = 10.0

# Stylus rotation during motion.
ROT_AXIS = (0.0, 0.0, 1.0)   # yaw around z-axis
ROT_ANGLE_DEG = 15.0

# Different goals to test (dx, dy, dz) in meters.
GOALS = {
    "goalA_main":     (0.15,  0.00, -0.10),
    "goalB_wide":     (0.25,  0.05, -0.12),
    "goalC_narrow":   (0.08, -0.03, -0.06),
    "goalD_lateral":  (0.10,  0.15, -0.08),
}


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--outdir", required=True, help="output directory for generated CSVs")
    p.add_argument("--dt", type=float, default=0.001, help="sampling step [s]")
    args = p.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    manifest_path = os.path.join(args.outdir, "manifest.csv")

    with open(manifest_path, "w", newline="") as mf:
        writer = csv.writer(mf)
        writer.writerow(["filename", "duration_s", "transition_duration_s", "goal_name", "dx", "dy", "dz"])

        for duration in DURATIONS:
            for goal_name, (dx, dy, dz) in GOALS.items():
                fname = f"demo_synth_{int(duration)}s_{goal_name}.csv"
                fpath = os.path.join(args.outdir, fname)
                rows = generate(duration=duration, dx=dx, dy=dy, dz=dz, dt=args.dt,
                                 transition_duration=TRANSITION_DURATION,
                                 rot_axis=ROT_AXIS, rot_angle_deg=ROT_ANGLE_DEG)
                write_csv(rows, fpath)
                writer.writerow([fname, duration, TRANSITION_DURATION, goal_name, dx, dy, dz])
                print(f"  {fname}: {len(rows)} samples")

    n_total = len(DURATIONS) * len(GOALS)
    print(f"\nGenerated {n_total} files in {args.outdir}/ (manifest: {manifest_path})")


if __name__ == "__main__":
    main()

