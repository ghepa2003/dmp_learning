#!/usr/bin/env python3
"""Plots 3D spatial trajectory and time series comparing:
  1. Traiettoria Insegnata (Real Demo recorded)
  2. Traiettoria Imparata (Original DMP Replay)
  3. Le 5 Traiettorie Eseguite per raggiungere i nuovi Goal
"""

import argparse
import csv
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


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


def load_goals_info(path):
    goals = []
    if not os.path.exists(path):
        return goals
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            goals.append({
                "id": int(row["goal_id"]),
                "name": row["name"],
                "gx": float(row["gx"]),
                "gy": float(row["gy"]),
                "gz": float(row["gz"]),
                "err_pos": float(row["err_pos_mm"]),
                "err_orient": float(row["err_orient_deg"]),
            })
    return goals


def make_generalization_plot(demo_csv, replay_orig_csv, goal_csvs, goals_info, label, out_dir):
    os.makedirs(out_dir, exist_ok=True)

    demo = load_csv(demo_csv)
    replay_orig = load_csv(replay_orig_csv)

    goal_replays = []
    for g_path in goal_csvs:
        if os.path.exists(g_path):
            goal_replays.append(load_csv(g_path))

    fig = plt.figure(figsize=(22, 7))
    fig.suptitle(f"Generalizzazione DMP su Nuovi Goal - Traiettoria {label}", fontsize=15, fontweight="bold")

    # Colors for the 5 new goals
    goal_colors = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3", "#ff7f00"]

    # --------------------------------------------------------------------------
    # Panel 1: 3D Trajectory Plot
    # --------------------------------------------------------------------------
    ax3d = fig.add_subplot(1, 3, 1, projection="3d")

    # 1. Taught (Insegnata)
    ax3d.plot(demo[1], demo[2], demo[3], label="Insegnata (Demo Reale)", color="blue", linewidth=2.5, alpha=0.8)

    # 2. Learned (Imparata)
    ax3d.plot(replay_orig[1], replay_orig[2], replay_orig[3], "--", label="Imparata (Goal Orig)", color="black", linewidth=2.0, alpha=0.9)

    # Start point marker
    ax3d.scatter([demo[1][0]], [demo[2][0]], [demo[3][0]], color="green", s=80, marker="o", label="Inizio (y0)", zorder=10)

    # Original goal marker
    ax3d.scatter([demo[1][-1]], [demo[2][-1]], [demo[3][-1]], color="black", s=80, marker="X", label="Goal Orig", zorder=10)

    # 3. Executed 5 New Goals
    for idx, (g_rep, color) in enumerate(zip(goal_replays, goal_colors)):
        g_id = idx + 1
        ax3d.plot(g_rep[1], g_rep[2], g_rep[3], ":", label=f"Eseguita Goal {g_id}", color=color, linewidth=2.0)
        # Mark final point executed
        ax3d.scatter([g_rep[1][-1]], [g_rep[2][-1]], [g_rep[3][-1]], color=color, s=70, marker="^", zorder=9)

    # Mark requested goals from info
    for g_info, color in zip(goals_info, goal_colors):
        ax3d.scatter([g_info["gx"]], [g_info["gy"]], [g_info["gz"]], color=color, s=100, marker="*", edgecolors="black", linewidths=0.5, zorder=11)

    ax3d.set_xlabel("X [m]")
    ax3d.set_ylabel("Y [m]")
    ax3d.set_zlabel("Z [m]")
    ax3d.set_title("Traiettorie 3D (Insegnata, Imparata, 5 Nuovi Goal)", fontweight="bold")
    ax3d.legend(fontsize=8, loc="upper left")

    # --------------------------------------------------------------------------
    # Panel 2: Position Components (x, y, z) over Time
    # --------------------------------------------------------------------------
    ax_t = fig.add_subplot(1, 3, 2)

    # Plot demo and original replay position norms or XYZ components
    # We plot position components for demo (solid gray) and goal replays (colored)
    ax_t.plot(demo[0], demo[1], color="gray", linestyle="-", linewidth=1.5, alpha=0.6, label="Demo X")
    ax_t.plot(demo[0], demo[2], color="gray", linestyle="--", linewidth=1.5, alpha=0.6, label="Demo Y")
    ax_t.plot(demo[0], demo[3], color="gray", linestyle=":", linewidth=1.5, alpha=0.6, label="Demo Z")

    for idx, (g_rep, color) in enumerate(zip(goal_replays, goal_colors)):
        g_id = idx + 1
        ax_t.plot(g_rep[0], g_rep[1], color=color, linestyle="-", alpha=0.8, linewidth=1.2, label=f"G{g_id} X")
        ax_t.plot(g_rep[0], g_rep[2], color=color, linestyle="--", alpha=0.8, linewidth=1.2)
        ax_t.plot(g_rep[0], g_rep[3], color=color, linestyle=":", alpha=0.8, linewidth=1.2)

    ax_t.set_xlabel("t [s]")
    ax_t.set_ylabel("Posizione [m]")
    ax_t.set_title("Evoluzione Posizione X/Y/Z nel Tempo per i 5 Goal", fontweight="bold")
    ax_t.grid(True, alpha=0.3)
    ax_t.legend(fontsize=7, ncol=2, loc="upper right")

    # --------------------------------------------------------------------------
    # Panel 3: Distance from Start to Target / Reaching Progression
    # --------------------------------------------------------------------------
    ax_dist = fig.add_subplot(1, 3, 3)

    import numpy as np

    p0 = np.array([demo[1][0], demo[2][0], demo[3][0]])
    d_demo = np.linalg.norm(np.column_stack((demo[1], demo[2], demo[3])) - p0, axis=1) * 1000.0
    d_orig = np.linalg.norm(np.column_stack((replay_orig[1], replay_orig[2], replay_orig[3])) - p0, axis=1) * 1000.0

    ax_dist.plot(demo[0], d_demo, color="blue", linewidth=2.0, label="Insegnata (Demo)")
    ax_dist.plot(replay_orig[0], d_orig, "--", color="black", linewidth=1.8, label="Imparata (Goal Orig)")

    for idx, (g_rep, color) in enumerate(zip(goal_replays, goal_colors)):
        g_id = idx + 1
        d_g = np.linalg.norm(np.column_stack((g_rep[1], g_rep[2], g_rep[3])) - p0, axis=1) * 1000.0
        err_txt = f" (err: {goals_info[idx]['err_pos']:.1f}mm)" if idx < len(goals_info) else ""
        ax_dist.plot(g_rep[0], d_g, ":", color=color, linewidth=1.8, label=f"Eseguita Goal {g_id}{err_txt}")

    ax_dist.set_xlabel("t [s]")
    ax_dist.set_ylabel("Distanza dal punto di partenza y0 [mm]")
    ax_dist.set_title("Progressione di Spostamento dal Start ai Nuovi Goal", fontweight="bold")
    ax_dist.grid(True, alpha=0.3)
    ax_dist.legend(fontsize=8, loc="lower right")

    plt.tight_layout()
    out_img = os.path.join(out_dir, f"{label}_goal_generalization.png")
    fig.savefig(out_img, dpi=150)
    plt.close(fig)
    print(f"[{label}] Plot salvato: {out_img}")
    return out_img


def main():
    parser = argparse.ArgumentParser(description="Plot goal generalization for DMP")
    parser.add_argument("--demo", required=True, help="Raw demo CSV")
    parser.add_argument("--replay-orig", required=True, help="Original replay CSV")
    parser.add_argument("--out-dir", default="plots/06_goal_generalization", help="Output directory")
    parser.add_argument("--label", required=True, help="Label e.g. trajA")
    args = parser.parse_args()

    data_dir = os.path.dirname(args.replay_orig)
    goal_csvs = [os.path.join(data_dir, f"{args.label}_replay_goal_{i}.csv") for i in range(1, 6)]
    goals_info_csv = os.path.join(data_dir, f"{args.label}_goals_info.csv")

    goals_info = load_goals_info(goals_info_csv)
    make_generalization_plot(args.demo, args.replay_orig, goal_csvs, goals_info, args.label, args.out_dir)


if __name__ == "__main__":
    main()
