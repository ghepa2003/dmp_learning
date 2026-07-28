#!/usr/bin/env python3
"""Confronta la demo REALE registrata dal Geomagic Touch con il replay della
DMP appresa da essa.

Uso:
    python3 plot_real_demo.py <percorso_dmp_demo_recorded.csv>

Si aspetta che 'replay_from_yaml.csv' sia gia' stato generato nella stessa
cartella (con replay_build_and_run.sh).
Richiede: matplotlib
"""
import sys
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def load_csv(path):
    t, x, y, z = [], [], [], []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row["t"]))
            x.append(float(row["x"]))
            y.append(float(row["y"]))
            z.append(float(row["z"]))
    return t, x, y, z


if len(sys.argv) >= 2:
    demo_path = sys.argv[1]
else:
    import os
    if os.path.exists("/home/lorenzo/thesis_ws/demo_raw.csv"):
        demo_path = "/home/lorenzo/thesis_ws/demo_raw.csv"
    elif os.path.exists("demo_raw.csv"):
        demo_path = "demo_raw.csv"
    elif os.path.exists("dmp_demo_recorded.csv"):
        demo_path = "dmp_demo_recorded.csv"
    else:
        print("Uso: python3 plot_real_demo.py <percorso_demo_raw.csv>")
        sys.exit(1)

demo = load_csv(demo_path)
replay = load_csv("replay_from_yaml.csv")

fig = plt.figure(figsize=(14, 6))

# --- Traiettoria 3D ---
ax3d = fig.add_subplot(1, 2, 1, projection="3d")
ax3d.plot(demo[1], demo[2], demo[3], label="Demo registrata (reale)", linewidth=2)
ax3d.plot(replay[1], replay[2], replay[3], "--", label="Replay DMP")
ax3d.set_xlabel("x [m]")
ax3d.set_ylabel("y [m]")
ax3d.set_zlabel("z [m]")
ax3d.set_title("Traiettoria 3D: reale vs replay")
ax3d.legend()

# --- Serie temporali per asse ---
ax_t = fig.add_subplot(1, 2, 2)
axes_labels = ["x", "y", "z"]
colors = ["tab:blue", "tab:orange", "tab:green"]
for i, (label, color) in enumerate(zip(axes_labels, colors)):
    ax_t.plot(demo[0], demo[i + 1], color=color, linestyle="-", label=f"demo {label}")
    ax_t.plot(replay[0], replay[i + 1], color=color, linestyle="--", alpha=0.7)

ax_t.set_xlabel("t [s]")
ax_t.set_ylabel("posizione [m]")
ax_t.set_title("Demo reale (continua) vs Replay (tratteggiata)")
ax_t.legend()

plt.tight_layout()
plt.savefig("real_demo_plot.png", dpi=150)
print("Salvato real_demo_plot.png")
plt.show()
