#!/usr/bin/env python3
"""Generates PER-TRIAL plots (3D trajectory, position, angular error over time)
comparing a demo with its replay.

Numerical metrics (RMSE, mean/max angular error, scale_reliable) are computed
by metrics.cpp/hpp via learn_and_test_dmp, which writes the summary CSV.
This script handles visualization.

Typical usage (from build_and_run.sh):
    python3 plot_dmp_timesweep.py \
        --demo data/demo_synth_45s_goalA_main.csv \
        --replay data/replay_demo_synth_45s_goalA_main.csv \
        --plot-dir plot \
        --label demo_synth_45s_goalA_main
"""
import argparse
import csv
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_csv(path):
    t, x, y, z = [], [], [], []
    qw, qx, qy, qz = [], [], [], []
    has_quat = False
    with open(path) as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames or []
        has_quat = "qw" in fieldnames
        for row in reader:
            t.append(float(row["t"]))
            x.append(float(row["x"]))
            y.append(float(row["y"]))
            z.append(float(row["z"]))
            if has_quat:
                qw.append(float(row["qw"]))
                qx.append(float(row["qx"]))
                qy.append(float(row["qy"]))
                qz.append(float(row["qz"]))
    return t, x, y, z, qw, qx, qy, qz, has_quat


def angular_error_series(demo, replay):
    """Computes angular error series for error-vs-time plot."""
    if not (demo[8] and replay[8]):
        return None, None
    n = min(len(demo[0]), len(replay[0]))
    angular_errors = []
    for k in range(n):
        dot = abs(demo[4][k] * replay[4][k] + demo[5][k] * replay[5][k] +
                  demo[6][k] * replay[6][k] + demo[7][k] * replay[7][k])
        dot = max(-1.0, min(1.0, dot))
        angular_errors.append(2.0 * math.acos(dot) * 180.0 / math.pi)
    return angular_errors, demo[0][:n]


def make_plots(demo, replay, label, plot_dir):
    os.makedirs(plot_dir, exist_ok=True)

    fig = plt.figure(figsize=(20, 6))

    ax3d = fig.add_subplot(1, 3, 1, projection="3d")
    ax3d.plot(demo[1], demo[2], demo[3], label="Demo", linewidth=2)
    ax3d.plot(replay[1], replay[2], replay[3], "--", label="Replay DMP")
    ax3d.set_xlabel("x [m]"); ax3d.set_ylabel("y [m]"); ax3d.set_zlabel("z [m]")
    ax3d.set_title(f"3D Trajectory: {label}")
    ax3d.legend()

    ax_t = fig.add_subplot(1, 3, 2)
    for i, (lbl, color) in enumerate(zip(["x", "y", "z"], ["tab:blue", "tab:orange", "tab:green"])):
        ax_t.plot(demo[0], demo[i + 1], color=color, linestyle="-", label=f"demo {lbl}")
        ax_t.plot(replay[0], replay[i + 1], color=color, linestyle="--", alpha=0.7)
    ax_t.set_xlabel("t [s]"); ax_t.set_ylabel("Position [m]")
    ax_t.set_title("Demo (solid) vs Replay (dashed)")
    ax_t.legend()

    ax_q = fig.add_subplot(1, 3, 3)
    angular_errors, t_common = angular_error_series(demo, replay)
    if demo[8] and replay[8]:
        for i, (lbl, color) in enumerate(zip(["qw", "qx", "qy", "qz"],
                                              ["tab:purple", "tab:blue", "tab:orange", "tab:green"])):
            ax_q.plot(demo[0], demo[i + 4], color=color, linestyle="-", label=f"demo {lbl}")
            ax_q.plot(replay[0], replay[i + 4], color=color, linestyle="--", alpha=0.7)
        ax_q.set_xlabel("t [s]"); ax_q.set_ylabel("Quaternion Components")
        ax_q.set_title("Orientation: Demo (solid) vs Replay (dashed)")
        ax_q.legend()
    else:
        ax_q.text(0.5, 0.5, "No orientation data in CSV", ha="center", va="center")
        ax_q.set_axis_off()

    plt.tight_layout()
    traj_path = os.path.join(plot_dir, f"{label}_trajectory.png")
    plt.savefig(traj_path, dpi=150)
    plt.close(fig)

    err_path = None
    if angular_errors is not None:
        fig2, ax_err = plt.subplots(figsize=(10, 5))
        ax_err.plot(t_common, angular_errors, color="tab:red")
        idx_max = angular_errors.index(max(angular_errors))
        ax_err.axvline(t_common[idx_max], color="k", linestyle="--",
                        label=f"max = {angular_errors[idx_max]:.2f}\u00b0 @ t={t_common[idx_max]:.2f}s")
        ax_err.set_xlabel("t [s]"); ax_err.set_ylabel("Angular Error [deg]")
        ax_err.set_title(f"Angular Error over Time: {label}")
        ax_err.legend()
        fig2.tight_layout()
        err_path = os.path.join(plot_dir, f"{label}_angular_error.png")
        fig2.savefig(err_path, dpi=150)
        plt.close(fig2)

    return traj_path, err_path


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--demo", required=True, help="Demo CSV (synthetic or real)")
    p.add_argument("--replay", required=True, help="Replay CSV (written by learn_and_test_dmp)")
    p.add_argument("--plot-dir", default="plot", help="Output directory for plots")
    p.add_argument("--label", default=None,
                    help="Label used in file names (default: demo filename)")
    args = p.parse_args()

    label = args.label or os.path.splitext(os.path.basename(args.demo))[0]

    demo = load_csv(args.demo)
    replay = load_csv(args.replay)

    traj_path, err_path = make_plots(demo, replay, label, args.plot_dir)
    print(f"[{label}] Saved: {traj_path}" + (f", {err_path}" if err_path else ""))


if __name__ == "__main__":
    main()

