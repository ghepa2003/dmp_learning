#!/usr/bin/env python3
"""Compares REAL demo recorded from Geomagic Touch with DMP replay
(position + orientation) learned from it.

Usage:
    python3 plot_real_demo.py <path_to_dmp_demo_recorded.csv>

Expects 'replay_from_yaml.csv' to already be generated in the same folder
(via replay_build_and_run.sh).
Requires: matplotlib, numpy

NOTE (time alignment): the comparison below occurs per sample index k
(replay[k] vs demo[k]), not by interpolated timestamp. This is accurate only
if demo and replay are sampled at regular intervals and synchronized at t=0.
Geomagic Touch publishes at ~1000Hz with real jitter, so on long demos a small
misalignment can accumulate - keep this in mind if peak max error does not
visually match a flip in the quaternion plot.
"""
import sys
import os
import csv
import math
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

def slerp_arrays(t_src, qw, qx, qy, qz, t_query):
    """Interpolates quaternions onto a new time base with hemisphere continuity
    correction (nlerp + renormalization) to prevent sign flips."""
    qw2, qx2, qy2, qz2 = list(qw), list(qx), list(qy), list(qz)
    for k in range(1, len(qw2)):
        dot = qw2[k]*qw2[k-1] + qx2[k]*qx2[k-1] + qy2[k]*qy2[k-1] + qz2[k]*qz2[k-1]
        if dot < 0:
            qw2[k], qx2[k], qy2[k], qz2[k] = -qw2[k], -qx2[k], -qy2[k], -qz2[k]

    out = []
    for comp in (qw2, qx2, qy2, qz2):
        out.append(list(__import__("numpy").interp(t_query, t_src, comp)))
    qwq, qxq, qyq, qzq = out
    for k in range(len(qwq)):
        n = math.sqrt(qwq[k]**2 + qxq[k]**2 + qyq[k]**2 + qzq[k]**2)
        if n > 1e-9:
            qwq[k], qxq[k], qyq[k], qzq[k] = qwq[k]/n, qxq[k]/n, qyq[k]/n, qzq[k]/n
    return qwq, qxq, qyq, qzq


def resample_to_common_time(demo, replay):
    """Resamples replay onto demo's time base (~1kHz) so point-by-point
    comparison occurs at equal time instants rather than sample indices."""
    import numpy as np
    t_common = demo[0]
    rx = list(np.interp(t_common, replay[0], replay[1]))
    ry = list(np.interp(t_common, replay[0], replay[2]))
    rz = list(np.interp(t_common, replay[0], replay[3]))
    has_quat = demo[8] and replay[8]
    if has_quat:
        rqw, rqx, rqy, rqz = slerp_arrays(replay[0], replay[4], replay[5], replay[6], replay[7], t_common)
    else:
        rqw, rqx, rqy, rqz = [], [], [], []
    return (t_common, rx, ry, rz, rqw, rqx, rqy, rqz, has_quat)


def print_endpoint_metrics(demo, replay):
    if not demo[1] or not replay[1]:
        return
    # Last sample of real demo = goal
    dgx, dgy, dgz = demo[1][-1], demo[2][-1], demo[3][-1]
    rgx, rgy, rgz = replay[1][-1], replay[2][-1], replay[3][-1]
    dx, dy, dz = (rgx - dgx) * 1000.0, (rgy - dgy) * 1000.0, (rgz - dgz) * 1000.0
    pos_dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    print(f"  [Final Error - Position] dx={dx:.4f} dy={dy:.4f} dz={dz:.4f} mm "
          f"| distance to goal: {pos_dist:.4f} mm")

    if demo[8] and replay[8]:
        dqw = replay[4][-1] - demo[4][-1]
        dqx = replay[5][-1] - demo[5][-1]
        dqy = replay[6][-1] - demo[6][-1]
        dqz = replay[7][-1] - demo[7][-1]
        dot = abs(demo[4][-1] * replay[4][-1] + demo[5][-1] * replay[5][-1] +
                  demo[6][-1] * replay[6][-1] + demo[7][-1] * replay[7][-1])
        dot = max(-1.0, min(1.0, dot))
        ang_dist = 2.0 * math.acos(dot) * 180.0 / math.pi
        print(f"  [Final Error - Orientation] dqw={dqw:.5f} dqx={dqx:.5f} "
              f"dqy={dqy:.5f} dqz={dqz:.5f} | angular distance to goal: {ang_dist:.4f} deg")


def print_metrics(demo, replay):
    """Calculates positional RMSE and angular error, prints report and
    returns (angular_errors, t_common)."""
    n = min(len(demo[0]), len(replay[0]))
    if n == 0:
        return None, None
    sq = [0.0, 0.0, 0.0]
    max_err = 0.0
    for k in range(n):
        dx = (replay[1][k] - demo[1][k]) * 1000.0
        dy = (replay[2][k] - demo[2][k]) * 1000.0
        dz = (replay[3][k] - demo[3][k]) * 1000.0
        sq[0] += dx * dx
        sq[1] += dy * dy
        sq[2] += dz * dz
        err = math.sqrt(dx * dx + dy * dy + dz * dz)
        max_err = max(max_err, err)
    rmse = [math.sqrt(s / n) for s in sq]
    rmse_overall = math.sqrt(sum(r * r for r in rmse))
    print(f"  [Position] RMSE x/y/z: {rmse[0]:.4f} / {rmse[1]:.4f} / {rmse[2]:.4f} mm "
          f"| Total RMSE: {rmse_overall:.4f} mm | Max error: {max_err:.4f} mm")

    angular_errors = None
    t_common = None
    if demo[8] and replay[8]:
        angular_errors = []
        sum_ang, max_ang = 0.0, 0.0
        idx_max_ang = 0
        for k in range(n):
            dot = abs(demo[4][k] * replay[4][k] + demo[5][k] * replay[5][k] +
                      demo[6][k] * replay[6][k] + demo[7][k] * replay[7][k])
            dot = max(-1.0, min(1.0, dot))
            angle_deg = 2.0 * math.acos(dot) * 180.0 / math.pi
            angular_errors.append(angle_deg)
            sum_ang += angle_deg
            if angle_deg > max_ang:
                max_ang = angle_deg
                idx_max_ang = k
        t_common = demo[0][:n]
        print(f"  [Orientation] Mean angular error: {sum_ang / n:.4f} deg | max: {max_ang:.4f} deg "
              f"at time t={t_common[idx_max_ang]:.3f}s (sample {idx_max_ang}/{n})")

    return angular_errors, t_common


if len(sys.argv) >= 2:
    demo_path = sys.argv[1]
else:
    candidates = [
        "/home/lorenzo/thesis_ws/dmp_demo_recorded.csv",
        "/home/lorenzo/thesis_ws/demo_raw.csv",
        "dmp_demo_recorded.csv",
        "demo_raw.csv",
    ]
    demo_path = next((c for c in candidates if os.path.exists(c)), None)
    if demo_path is None:
        print("Usage: python3 plot_real_demo.py <path_to_dmp_demo_recorded.csv>")
        sys.exit(1)

demo = load_csv(demo_path)

replay_csv_candidates = [
    "data/replay_from_yaml.csv",
    "replay_from_yaml.csv",
]
replay_path = next((c for c in replay_csv_candidates if os.path.exists(c)), "data/replay_from_yaml.csv")
replay = load_csv(replay_path)
replay = resample_to_common_time(demo, replay)

print(f"Demo: {demo_path}")
angular_errors, t_common = print_metrics(demo, replay)
print_endpoint_metrics(demo, replay)

fig = plt.figure(figsize=(20, 6))

# --- 3D Trajectory ---
ax3d = fig.add_subplot(1, 3, 1, projection="3d")
ax3d.plot(demo[1], demo[2], demo[3], label="Recorded Demo (real)", linewidth=2)
ax3d.plot(replay[1], replay[2], replay[3], "--", label="DMP Replay")
ax3d.set_xlabel("x [m]")
ax3d.set_ylabel("y [m]")
ax3d.set_zlabel("z [m]")
ax3d.set_title("3D Trajectory: real vs replay")
ax3d.legend()

# --- Axis Time Series ---
ax_t = fig.add_subplot(1, 3, 2)
axes_labels = ["x", "y", "z"]
colors = ["tab:blue", "tab:orange", "tab:green"]
for i, (label, color) in enumerate(zip(axes_labels, colors)):
    ax_t.plot(demo[0], demo[i + 1], color=color, linestyle="-", label=f"demo {label}")
    ax_t.plot(replay[0], replay[i + 1], color=color, linestyle="--", alpha=0.7)

ax_t.set_xlabel("t [s]")
ax_t.set_ylabel("position [m]")
ax_t.set_title("Real Demo (solid) vs Replay (dashed)")
ax_t.legend()

# --- Orientation: Quaternion Components over Time ---
ax_q = fig.add_subplot(1, 3, 3)
if demo[8] and replay[8]:
    quat_labels = ["qw", "qx", "qy", "qz"]
    quat_colors = ["tab:purple", "tab:blue", "tab:orange", "tab:green"]
    for i, (label, color) in enumerate(zip(quat_labels, quat_colors)):
        ax_q.plot(demo[0], demo[i + 4], color=color, linestyle="-", label=f"demo {label}")
        ax_q.plot(replay[0], replay[i + 4], color=color, linestyle="--", alpha=0.7)
    ax_q.set_xlabel("t [s]")
    ax_q.set_ylabel("quaternion components")
    ax_q.set_title("Orientation: Demo (solid) vs Replay (dashed)")
    ax_q.legend()
else:
    ax_q.text(0.5, 0.5, "No orientation data in CSV", ha="center", va="center")
    ax_q.set_axis_off()

out_dir = os.path.join("plots", "02_real_data")
os.makedirs(out_dir, exist_ok=True)
plt.tight_layout()
out_plot1 = os.path.join(out_dir, "real_demo_plot.png")
plt.savefig(out_plot1, dpi=150)
print(f"Saved {out_plot1}")

# --- Angular Error over Time ---
if angular_errors is not None:
    fig2, ax_err = plt.subplots(figsize=(10, 5))
    ax_err.plot(t_common, angular_errors, color="tab:red")
    idx_max = angular_errors.index(max(angular_errors))
    ax_err.axvline(t_common[idx_max], color="k", linestyle="--",
                    label=f"max = {angular_errors[idx_max]:.2f}\u00b0 @ t={t_common[idx_max]:.2f}s")
    ax_err.set_xlabel("t [s]")
    ax_err.set_ylabel("angular error [deg]")
    ax_err.set_title("Angular Error over Time")
    ax_err.legend()
    fig2.tight_layout()
    out_plot2 = os.path.join(out_dir, "angular_error_over_time.png")
    fig2.savefig(out_plot2, dpi=150)
    print(f"Saved {out_plot2}")

plt.show()
