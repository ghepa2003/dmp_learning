#!/usr/bin/env python3
"""Versione headless di evaluate_cartesian_tracking.py per l'uso dentro gli
script di sweep automatizzati (nessuna finestra interattiva, salva solo il
PNG e stampa le metriche a console).

Uso:
    python3 evaluate_cartesian_tracking_headless.py <nome_run>
"""
import sys
import os
import csv
import math
import bisect

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
            x.append(float(row["x"])); y.append(float(row["y"])); z.append(float(row["z"]))
            qw.append(float(row["qw"])); qx.append(float(row["qx"]))
            qy.append(float(row["qy"])); qz.append(float(row["qz"]))
    return t, x, y, z, qw, qx, qy, qz


def quat_angle_between(q1, q2):
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.degrees(math.acos(dot))


def cumulative_and_net_angle(qw, qx, qy, qz):
    quats = list(zip(qw, qx, qy, qz))
    cum, max_step = 0.0, 0.0
    for i in range(1, len(quats)):
        ang = quat_angle_between(quats[i - 1], quats[i])
        cum += ang
        if ang > max_step:
            max_step = ang
    net = quat_angle_between(quats[0], quats[-1])
    return cum, net, max_step


def tracking_errors(target, actual):
    t_t, t_x, t_y, t_z, t_qw, t_qx, t_qy, t_qz = target
    a_t, a_x, a_y, a_z, a_qw, a_qx, a_qy, a_qz = actual

    pos_errs, ang_errs = [], []
    for i in range(len(a_t)):
        j = bisect.bisect_left(t_t, a_t[i])
        j = min(max(j, 0), len(t_t) - 1)
        dp = math.sqrt((a_x[i] - t_x[j]) ** 2 + (a_y[i] - t_y[j]) ** 2 + (a_z[i] - t_z[j]) ** 2)
        pos_errs.append(dp)
        da = quat_angle_between((a_qw[i], a_qx[i], a_qy[i], a_qz[i]),
                                 (t_qw[j], t_qx[j], t_qy[j], t_qz[j]))
        ang_errs.append(da)
    return pos_errs, ang_errs


def find_clamp_index(t_x, t_y, t_z, tol_m=1e-5):
    """Index of the first target sample already frozen at the goal (t ~= tau):
    scans backward from the end and returns the first index after which every
    consecutive step stays below tol_m (default 0.01 mm)."""
    n = len(t_x)
    for i in range(n - 1, 0, -1):
        d = math.sqrt((t_x[i] - t_x[i - 1]) ** 2 +
                       (t_y[i] - t_y[i - 1]) ** 2 +
                       (t_z[i] - t_z[i - 1]) ** 2)
        if d >= tol_m:
            return i
    return 0


if len(sys.argv) < 2:
    print("Uso: python3 evaluate_cartesian_tracking_headless.py <nome_run>")
    sys.exit(1)

run_name = sys.argv[1]
data_dir = os.path.join(os.path.dirname(__file__), "..", "data")
target_path = os.path.join(data_dir, f"target_aligned_{run_name}.csv")
actual_path = os.path.join(data_dir, f"actual_pose_{run_name}.csv")

try:
    target = load_csv(target_path)
    actual = load_csv(actual_path)
except FileNotFoundError as e:
    print(f"File non trovato: {e}")
    print("RESULT_LINE,NA,NA,NA,NA,NA,NA,NA")
    sys.exit(1)

if len(target[0]) < 2 or len(actual[0]) < 2:
    print("Dati insufficienti (probabilmente 0 messaggi registrati).")
    print("RESULT_LINE,NA,NA,NA,NA,NA,NA,NA")
    sys.exit(1)

t_cum, t_net, t_max = cumulative_and_net_angle(*target[4:8])
a_cum, a_net, a_max = cumulative_and_net_angle(*actual[4:8])
pos_errs, ang_errs = tracking_errors(target, actual)

ratio_actual = a_cum / a_net if a_net > 1e-6 else float('nan')
mean_pos_mm = sum(pos_errs) / len(pos_errs) * 1000
max_pos_mm = max(pos_errs) * 1000
mean_ang_deg = sum(ang_errs) / len(ang_errs)
max_ang_deg = max(ang_errs)

final_pos_mm = pos_errs[-1] * 1000
final_ang_deg = ang_errs[-1]

t_t, t_x, t_y, t_z = target[0], target[1], target[2], target[3]
clamp_idx = find_clamp_index(t_x, t_y, t_z)
clamp_t = t_t[clamp_idx]
a_t = actual[0]
j_tau = bisect.bisect_left(a_t, clamp_t)
j_tau = min(max(j_tau, 0), len(a_t) - 1)
final_tau_pos_mm = pos_errs[j_tau] * 1000
final_tau_ang_deg = ang_errs[j_tau]

print(f"\n=== {run_name} ===")
print(f"[target] rotazione cumulativa={t_cum:.2f} deg, netta={t_net:.2f} deg, rapporto={t_cum/t_net:.3f}")
print(f"[actual] rotazione cumulativa={a_cum:.2f} deg, netta={a_net:.2f} deg, rapporto={ratio_actual:.3f}")
print(f"errore posizione: media={mean_pos_mm:.2f}mm, max={max_pos_mm:.2f}mm")
print(f"errore orientamento: media={mean_ang_deg:.3f} deg, max={max_ang_deg:.3f} deg")
print(f"Final Error [last sample]: pos={final_pos_mm:.2f}mm, ang={final_ang_deg:.3f} deg "
      f"(t={a_t[-1]:.3f}s, ultimo campione del bag)")
print(f"Final Error [@tau]:        pos={final_tau_pos_mm:.2f}mm, ang={final_tau_ang_deg:.3f} deg "
      f"(target congelato a partire da t={clamp_t:.3f}s)")

# Riga machine-readable per lo sweep script (evita parsing fragile su testo colloquiale)
print(f"RESULT_LINE,{a_cum:.4f},{a_net:.4f},{ratio_actual:.4f},{mean_pos_mm:.4f},{max_pos_mm:.4f},{mean_ang_deg:.4f},{max_ang_deg:.4f},"
      f"{final_pos_mm:.4f},{final_ang_deg:.4f},{final_tau_pos_mm:.4f},{final_tau_ang_deg:.4f}")

# Plot (salvato, non mostrato)
fig = plt.figure(figsize=(16, 6))
fig.suptitle(f"Cartesian tracking: {run_name}")

ax3d = fig.add_subplot(1, 3, 1, projection="3d")
ax3d.plot(target[1], target[2], target[3], label="Target (comandato)", linewidth=2)
ax3d.plot(actual[1], actual[2], actual[3], "--", label="Actual (Gazebo)")
ax3d.set_xlabel("x [m]"); ax3d.set_ylabel("y [m]"); ax3d.set_zlabel("z [m]")
ax3d.set_title("3D Position Trajectory")
ax3d.legend()

ax_t = fig.add_subplot(1, 3, 2)
axes_labels = ["x", "y", "z"]
colors = ["tab:blue", "tab:orange", "tab:green"]
for i, (label, color) in enumerate(zip(axes_labels, colors)):
    ax_t.plot(target[0], target[i + 1], color=color, linestyle="-", label=f"target {label}")
    ax_t.plot(actual[0], actual[i + 1], color=color, linestyle="--", alpha=0.7)
ax_t.set_xlabel("t [s]"); ax_t.set_ylabel("position [m]")
ax_t.set_title("Position: Target (solid) vs Actual (dashed)")
ax_t.legend()

ax_q = fig.add_subplot(1, 3, 3)
q_labels = ["qw", "qx", "qy", "qz"]
q_colors = ["purple", "tab:blue", "tab:orange", "tab:green"]
for i, (label, color) in enumerate(zip(q_labels, q_colors)):
    ax_q.plot(target[0], target[i + 4], color=color, linestyle="-", label=f"target {label}")
    ax_q.plot(actual[0], actual[i + 4], color=color, linestyle="--", alpha=0.7)
ax_q.set_xlabel("t [s]"); ax_q.set_ylabel("quaternion components")
ax_q.set_title("Orientation: Target (solid) vs Actual (dashed)")
ax_q.legend()

plt.tight_layout()
out_dir = os.path.join(os.path.dirname(__file__), "..", "plots", "03_gain_sweep")
os.makedirs(out_dir, exist_ok=True)
out_path = os.path.join(out_dir, f"cartesian_tracking_{run_name}.png")
plt.savefig(out_path, dpi=150)
print(f"Saved {out_path}")
