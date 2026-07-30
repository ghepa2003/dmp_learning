#!/usr/bin/env python3
"""Confronto visivo: demo sintetica originale vs replay (Posizione + Quaternion DMP).

Uso:
    python3 plot_dmp_test.py [nome_traiettoria]

Se non specifichi il nome, prova "reach_lift_pitch" come default e, se non
esiste, elenca le traiettorie trovate nella cartella corrente.
"""
import sys
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
    files = glob.glob("demo_original_*.csv")
    return sorted(f.replace("demo_original_", "").replace(".csv", "") for f in files)


if len(sys.argv) >= 2:
    traj_name = sys.argv[1]
else:
    trajs = available_trajectories()
    if not trajs:
        print("Nessuna traiettoria trovata (demo_original_*.csv). Esegui prima build_and_run.sh")
        sys.exit(1)
    traj_name = "reach_lift_pitch" if "reach_lift_pitch" in trajs else trajs[0]
    print(f"Nessun nome specificato, uso: {traj_name}")
    print(f"Traiettorie disponibili: {', '.join(trajs)}")

demo_path = f"demo_original_{traj_name}.csv"
replay_same_path = f"replay_same_goal_{traj_name}.csv"
replay_new_path = f"replay_new_goal_{traj_name}.csv"

try:
    demo = load_csv(demo_path)
    replay_same = load_csv(replay_same_path)
    replay_new = load_csv(replay_new_path)
except FileNotFoundError as e:
    print(f"File non trovato: {e}")
    print(f"Traiettorie disponibili: {', '.join(available_trajectories())}")
    sys.exit(1)

fig = plt.figure(figsize=(16, 6))
fig.suptitle(f"Traiettoria: {traj_name}")

# --- Traiettoria 3D Posizione ---
ax3d = fig.add_subplot(1, 3, 1, projection="3d")
ax3d.plot(demo[1], demo[2], demo[3], label="Demo originale (sintetica)", linewidth=2)
ax3d.plot(replay_same[1], replay_same[2], replay_same[3], "--", label="Replay (stesso goal)")
ax3d.plot(replay_new[1], replay_new[2], replay_new[3], ":", label="Replay (goal spostato)")
ax3d.set_xlabel("x [m]")
ax3d.set_ylabel("y [m]")
ax3d.set_zlabel("z [m]")
ax3d.set_title("Traiettoria 3D Posizione")
ax3d.legend()

# --- Serie temporali Posizione ---
ax_t = fig.add_subplot(1, 3, 2)
axes_labels = ["x", "y", "z"]
colors = ["tab:blue", "tab:orange", "tab:green"]
for i, (label, color) in enumerate(zip(axes_labels, colors)):
    ax_t.plot(demo[0], demo[i + 1], color=color, linestyle="-", label=f"demo {label}")
    ax_t.plot(replay_same[0], replay_same[i + 1], color=color, linestyle="--", alpha=0.7)
ax_t.set_xlabel("t [s]")
ax_t.set_ylabel("posizione [m]")
ax_t.set_title("Posizione: Demo (continua) vs Replay (tratteggiata)")
ax_t.legend()

# --- Serie temporali Orientamento (Quaternione) ---
if demo[8]:
    ax_q = fig.add_subplot(1, 3, 3)
    q_labels = ["qw", "qx", "qy", "qz"]
    q_colors = ["purple", "tab:blue", "tab:orange", "tab:green"]
    for i, (label, color) in enumerate(zip(q_labels, q_colors)):
        ax_q.plot(demo[0], demo[i + 4], color=color, linestyle="-", label=f"demo {label}")
        ax_q.plot(replay_same[0], replay_same[i + 4], color=color, linestyle="--", alpha=0.7)
    ax_q.set_xlabel("t [s]")
    ax_q.set_ylabel("componenti quaternione")
    ax_q.set_title("Orientamento: Demo (continua) vs Replay (tratteggiata)")
    ax_q.legend()

plt.tight_layout()
out_path = f"dmp_test_plot_{traj_name}.png"
plt.savefig(out_path, dpi=150)
print(f"Salvato {out_path}")
plt.show()
