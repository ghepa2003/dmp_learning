#!/usr/bin/env python3
"""Plot DMP Learning & Replay for Trajectory A (w=0.20s, n_basis=500, Ridge)
in the exact visual style of plot_dmp_test.py.
"""
import os
import csv
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def load_csv(path):
    t, x, y, z = [], [], [], []
    qw, qx, qy, qz = [], [], [], []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row["t"]))
            x.append(float(row["x"]))
            y.append(float(row["y"]))
            z.append(float(row["z"]))
            qw.append(float(row["qw"]))
            qx.append(float(row["qx"]))
            qy.append(float(row["qy"]))
            qz.append(float(row["qz"]))
    return t, x, y, z, qw, qx, qy, qz


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.abspath(os.path.join(script_dir, "..", ".."))

    candidate_demo_paths = [
        os.path.abspath(os.path.join(root_dir, "..", "demo_raw_trajA.csv")),
        os.path.abspath(os.path.join(root_dir, "data", "demo_raw_trajA.csv")),
        "/home/lorenzo/thesis_ws/demo_raw_trajA.csv"
    ]
    demo_path = None
    for p in candidate_demo_paths:
        if os.path.exists(p):
            demo_path = p
            break

    if not demo_path:
        raise FileNotFoundError(f"Cannot find demo_raw_trajA.csv in candidate paths: {candidate_demo_paths}")

    replay_path = os.path.join(root_dir, "data", "replay_nbasis_0500_trajA_ridge_w0.20.csv")
    if not os.path.exists(replay_path):
        raise FileNotFoundError(f"Cannot find replay CSV: {replay_path}")

    print(f"Loading Demo: {demo_path}")
    print(f"Loading Replay: {replay_path}")

    demo = load_csv(demo_path)
    replay = load_csv(replay_path)

    fig = plt.figure(figsize=(16, 6))
    fig.suptitle("DMP Learning & Replay: Trajectory A (Window=0.20s, n_basis=500, Ridge Active)", fontsize=14, fontweight="bold")

    # --- 1. 3D Position Trajectory ---
    ax3d = fig.add_subplot(1, 3, 1, projection="3d")
    ax3d.plot(demo[1], demo[2], demo[3], label="Original Demo (trajA)", linewidth=2, color="black")
    ax3d.plot(replay[1], replay[2], replay[3], "--", label="Replay (w=0.20s, n_basis=500, Ridge)", linewidth=1.5, color="tab:red")
    ax3d.set_xlabel("x [m]")
    ax3d.set_ylabel("y [m]")
    ax3d.set_zlabel("z [m]")
    ax3d.set_title("3D Position Trajectory")
    ax3d.legend(fontsize=9)

    # --- 2. Position Time Series ---
    ax_t = fig.add_subplot(1, 3, 2)
    axes_labels = ["x", "y", "z"]
    colors = ["tab:blue", "tab:orange", "tab:green"]
    for i, (label, color) in enumerate(zip(axes_labels, colors)):
        ax_t.plot(demo[0], demo[i + 1], color=color, linestyle="-", label=f"demo {label}")
        ax_t.plot(replay[0], replay[i + 1], color=color, linestyle="--", alpha=0.7, label=f"replay {label}")
    ax_t.set_xlabel("t [s]")
    ax_t.set_ylabel("position [m]")
    ax_t.set_title("Position: Demo (solid) vs Replay (dashed)")
    ax_t.legend(fontsize=8, ncol=2)
    ax_t.grid(True, alpha=0.3)

    # --- 3. Orientation Time Series (Quaternion) ---
    ax_q = fig.add_subplot(1, 3, 3)
    q_labels = ["qw", "qx", "qy", "qz"]
    q_colors = ["purple", "tab:blue", "tab:orange", "tab:green"]
    for i, (label, color) in enumerate(zip(q_labels, q_colors)):
        ax_q.plot(demo[0], demo[i + 4], color=color, linestyle="-", label=f"demo {label}")
        ax_q.plot(replay[0], replay[i + 4], color=color, linestyle="--", alpha=0.7, label=f"replay {label}")
    ax_q.set_xlabel("t [s]")
    ax_q.set_ylabel("quaternion components")
    ax_q.set_title("Orientation: Demo (solid) vs Replay (dashed)")
    ax_q.legend(fontsize=8, ncol=2)
    ax_q.grid(True, alpha=0.3)

    plt.tight_layout()

    out_dir = os.path.join(root_dir, "04_basis_sweep", "plots")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "dmp_learning_trajA_w0.20_nbasis500_ridge.png")
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"Plot saved to: {out_path}")


if __name__ == "__main__":
    main()
