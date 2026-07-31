#!/usr/bin/env python3
"""Visual comparison: original synthetic demo vs replay (Position + Quaternion DMP).

Usage:
    python3 plot_dmp_test.py [trajectory_name]

If no name is specified, tries "reach_lift_pitch" by default; if it does not
exist, lists available trajectories in the current folder.
"""
import sys
import os
import glob
import csv
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


def available_trajectories():
    files = glob.glob("data/demo_original_*.csv") or glob.glob("demo_original_*.csv")
    names = set()
    for f in files:
        basename = os.path.basename(f)
        names.add(basename.replace("demo_original_", "").replace(".csv", ""))
    return sorted(list(names))


def find_data_file(filename):
    path = os.path.join("data", filename)
    if os.path.exists(path):
        return path
    if os.path.exists(filename):
        return filename
    return path


if len(sys.argv) >= 2:
    traj_name = sys.argv[1]
else:
    trajs = available_trajectories()
    if not trajs:
        print("No trajectory found (data/demo_original_*.csv). Run build_and_run.sh first")
        sys.exit(1)
    traj_name = "reach_lift_pitch" if "reach_lift_pitch" in trajs else trajs[0]
    print(f"No name specified, using: {traj_name}")
    print(f"Available trajectories: {', '.join(trajs)}")

demo_path = find_data_file(f"demo_original_{traj_name}.csv")
replay_same_path = find_data_file(f"replay_same_goal_{traj_name}.csv")
replay_new_path = find_data_file(f"replay_new_goal_{traj_name}.csv")

try:
    demo = load_csv(demo_path)
    replay_same = load_csv(replay_same_path)
    replay_new = load_csv(replay_new_path)
except FileNotFoundError as e:
    print(f"File not found: {e}")
    print(f"Available trajectories: {', '.join(available_trajectories())}")
    sys.exit(1)

fig = plt.figure(figsize=(16, 6))
fig.suptitle(f"Trajectory: {traj_name}")

# --- 3D Position Trajectory ---
ax3d = fig.add_subplot(1, 3, 1, projection="3d")
ax3d.plot(demo[1], demo[2], demo[3], label="Original Demo (synthetic)", linewidth=2)
ax3d.plot(replay_same[1], replay_same[2], replay_same[3], "--", label="Replay (same goal)")
ax3d.plot(replay_new[1], replay_new[2], replay_new[3], ":", label="Replay (shifted goal)")
ax3d.set_xlabel("x [m]")
ax3d.set_ylabel("y [m]")
ax3d.set_zlabel("z [m]")
ax3d.set_title("3D Position Trajectory")
ax3d.legend()

# --- Position Time Series ---
ax_t = fig.add_subplot(1, 3, 2)
axes_labels = ["x", "y", "z"]
colors = ["tab:blue", "tab:orange", "tab:green"]
for i, (label, color) in enumerate(zip(axes_labels, colors)):
    ax_t.plot(demo[0], demo[i + 1], color=color, linestyle="-", label=f"demo {label}")
    ax_t.plot(replay_same[0], replay_same[i + 1], color=color, linestyle="--", alpha=0.7)
ax_t.set_xlabel("t [s]")
ax_t.set_ylabel("position [m]")
ax_t.set_title("Position: Demo (solid) vs Replay (dashed)")
ax_t.legend()

# --- Orientation Time Series (Quaternion) ---
if demo[8]:
    ax_q = fig.add_subplot(1, 3, 3)
    q_labels = ["qw", "qx", "qy", "qz"]
    q_colors = ["purple", "tab:blue", "tab:orange", "tab:green"]
    for i, (label, color) in enumerate(zip(q_labels, q_colors)):
        ax_q.plot(demo[0], demo[i + 4], color=color, linestyle="-", label=f"demo {label}")
        ax_q.plot(replay_same[0], replay_same[i + 4], color=color, linestyle="--", alpha=0.7)
    ax_q.set_xlabel("t [s]")
    ax_q.set_ylabel("quaternion components")
    ax_q.set_title("Orientation: Demo (solid) vs Replay (dashed)")
    ax_q.legend()

plt.tight_layout()
os.makedirs("plots", exist_ok=True)
out_path = os.path.join("plots", f"dmp_test_plot_{traj_name}.png")
plt.savefig(out_path, dpi=150)
print(f"Saved {out_path}")
plt.show()
