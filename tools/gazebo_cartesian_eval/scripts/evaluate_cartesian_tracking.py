#!/usr/bin/env python3
"""Confronto visivo: target cartesiano comandato vs posa effettivamente
raggiunta dal robot in Gazebo (Position + Quaternion), più metriche di
errore di tracking. Stesso stile grafico di plot_dmp_test.py.

Uso:
    python3 evaluate_cartesian_tracking.py [nome_run]

Se non specificato, prova a listare i run disponibili in data/.
"""
import sys
import os
import glob
import csv
import math
import bisect
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


def available_runs(data_dir):
    files = glob.glob(os.path.join(data_dir, "target_aligned_*.csv"))
    return sorted(os.path.basename(f).replace("target_aligned_", "").replace(".csv", "") for f in files)


def find_data_file(data_dir, filename):
    path = os.path.join(data_dir, filename)
    return path


def quat_angle_between(q1, q2):
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.degrees(math.acos(dot))


def cumulative_and_net_angle(qw, qx, qy, qz):
    quats = list(zip(qw, qx, qy, qz))
    cum, max_step, max_step_i = 0.0, 0.0, None
    for i in range(1, len(quats)):
        ang = quat_angle_between(quats[i - 1], quats[i])
        cum += ang
        if ang > max_step:
            max_step, max_step_i = ang, i
    net = quat_angle_between(quats[0], quats[-1])
    return cum, net, max_step, max_step_i


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


if len(sys.argv) >= 2:
    run_name = sys.argv[1]
else:
    data_dir = os.path.join(os.path.dirname(__file__), "..", "data")
    runs = available_runs(data_dir)
    if not runs:
        print("Nessun run trovato (data/target_aligned_*.csv). Esegui prima extract_bag_to_csv.py")
        sys.exit(1)
    run_name = runs[-1]
    print(f"Nessun nome specificato, uso l'ultimo run: {run_name}")
    print(f"Run disponibili: {', '.join(runs)}")

data_dir = os.path.join(os.path.dirname(__file__), "..", "data")
target_path = find_data_file(data_dir, f"target_aligned_{run_name}.csv")
actual_path = find_data_file(data_dir, f"actual_pose_{run_name}.csv")

try:
    target = load_csv(target_path)
    actual = load_csv(actual_path)
except FileNotFoundError as e:
    print(f"File non trovato: {e}")
    sys.exit(1)

# --- Metriche numeriche (stampate a console, come diagnostica testuale) ---
t_cum, t_net, t_max, _ = cumulative_and_net_angle(*target[4:8])
a_cum, a_net, a_max, _ = cumulative_and_net_angle(*actual[4:8])
pos_errs, ang_errs = tracking_errors(target, actual)

print(f"\n=== {run_name} ===")
print(f"[target] rotazione cumulativa={t_cum:.2f} deg, netta={t_net:.2f} deg, "
      f"rapporto={t_cum/t_net:.3f}, max step singolo={t_max:.4f} deg")
print(f"[actual] rotazione cumulativa={a_cum:.2f} deg, netta={a_net:.2f} deg, "
      f"rapporto={a_cum/a_net:.3f}, max step singolo={a_max:.4f} deg")
print(f"errore posizione: media={sum(pos_errs)/len(pos_errs)*1000:.2f}mm, "
      f"max={max(pos_errs)*1000:.2f}mm, "
      f"finale={pos_errs[-1]*1000:.2f}mm")
print(f"errore orientamento: media={sum(ang_errs)/len(ang_errs):.3f} deg, "
      f"max={max(ang_errs):.3f} deg, "
      f"finale={ang_errs[-1]:.3f} deg")

# --- Plot, stesso layout/stile di plot_dmp_test.py ---
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
out_dir = os.path.join(os.path.dirname(__file__), "..", "plots", "02_gazebo_tracking")
os.makedirs(out_dir, exist_ok=True)
out_path = os.path.join(out_dir, f"cartesian_tracking_{run_name}.png")
plt.savefig(out_path, dpi=150)
print(f"\nSaved {out_path}")
plt.show()