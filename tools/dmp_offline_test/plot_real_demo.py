#!/usr/bin/env python3
"""Confronta la demo REALE registrata dal Geomagic Touch con il replay della
DMP (posizione + orientamento) appresa da essa.

Uso:
    python3 plot_real_demo.py <percorso_dmp_demo_recorded.csv>

Si aspetta che 'replay_from_yaml.csv' sia gia' stato generato nella stessa
cartella (con replay_build_and_run.sh).
Richiede: matplotlib, numpy

NOTA (allineamento temporale): il confronto qui sotto avviene per indice di
campione k (replay[k] vs demo[k]), non per timestamp interpolato. Questo e'
corretto solo se demo e replay sono campionati a intervalli regolari e
sincronizzati a t=0. Il Geomagic Touch pubblica a ~1000Hz con jitter reale,
quindi su demo lunghe un piccolo disallineamento puo' accumularsi - da tenere
presente se l'istante di errore massimo individuato qui sotto non corrisponde
visivamente a un'inversione nel grafico dei quaternioni.
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
    """Interpola i quaternioni su una nuova base temporale, con correzione di
    continuità di emisfero (nlerp + rinormalizzazione) per evitare che un
    'nearest' cada dal lato sbagliato del segno durante un flip veloce."""
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
    """Ricampiona replay sulla base temporale di demo (che ha risoluzione
    comparabile, ~1kHz) cosi' il confronto punto-a-punto avviene a parita' di
    istante temporale, non di indice di campione."""
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
    # Ultimo campione della demo reale = goal (sul sistema reale non c'e'
    # variazione di goal a runtime, quindi il goal coincide col punto finale demo).
    dgx, dgy, dgz = demo[1][-1], demo[2][-1], demo[3][-1]
    rgx, rgy, rgz = replay[1][-1], replay[2][-1], replay[3][-1]
    dx, dy, dz = rgx - dgx, rgy - dgy, rgz - dgz
    pos_dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    print(f"  [Errore finale - Posizione] dx={dx:.5f} dy={dy:.5f} dz={dz:.5f} m "
          f"| distanza dal goal: {pos_dist:.5f} m")

    if demo[8] and replay[8]:
        dqw = replay[4][-1] - demo[4][-1]
        dqx = replay[5][-1] - demo[5][-1]
        dqy = replay[6][-1] - demo[6][-1]
        dqz = replay[7][-1] - demo[7][-1]
        dot = abs(demo[4][-1] * replay[4][-1] + demo[5][-1] * replay[5][-1] +
                  demo[6][-1] * replay[6][-1] + demo[7][-1] * replay[7][-1])
        dot = max(-1.0, min(1.0, dot))
        ang_dist = 2.0 * math.acos(dot) * 180.0 / math.pi
        print(f"  [Errore finale - Orientamento] dqw={dqw:.5f} dqx={dqx:.5f} "
              f"dqy={dqy:.5f} dqz={dqz:.5f} | distanza angolare dal goal: {ang_dist:.4f} deg")


def print_metrics(demo, replay):
    """Calcola RMSE posizionale ed errore angolare, stampa il report e
    restituisce (angular_errors, t_common) per la diagnostica Step 1
    (localizzazione temporale dell'errore massimo). Se non ci sono dati di
    orientamento restituisce (None, None)."""
    n = min(len(demo[0]), len(replay[0]))
    if n == 0:
        return None, None
    sq = [0.0, 0.0, 0.0]
    max_err = 0.0
    for k in range(n):
        dx = replay[1][k] - demo[1][k]
        dy = replay[2][k] - demo[2][k]
        dz = replay[3][k] - demo[3][k]
        sq[0] += dx * dx
        sq[1] += dy * dy
        sq[2] += dz * dz
        err = math.sqrt(dx * dx + dy * dy + dz * dz)
        max_err = max(max_err, err)
    rmse = [math.sqrt(s / n) for s in sq]
    rmse_overall = math.sqrt(sum(r * r for r in rmse))
    print(f"  [Posizione] RMSE x/y/z: {rmse[0]:.5f} / {rmse[1]:.5f} / {rmse[2]:.5f} m "
          f"| RMSE totale: {rmse_overall:.5f} m | Errore max: {max_err:.5f} m")

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
        print(f"  [Orientamento] Errore angolare medio: {sum_ang / n:.4f} deg | massimo: {max_ang:.4f} deg "
              f"al tempo t={t_common[idx_max_ang]:.3f}s (campione {idx_max_ang}/{n})")

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
        print("Uso: python3 plot_real_demo.py <percorso_dmp_demo_recorded.csv>")
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

# --- Traiettoria 3D ---
ax3d = fig.add_subplot(1, 3, 1, projection="3d")
ax3d.plot(demo[1], demo[2], demo[3], label="Demo registrata (reale)", linewidth=2)
ax3d.plot(replay[1], replay[2], replay[3], "--", label="Replay DMP")
ax3d.set_xlabel("x [m]")
ax3d.set_ylabel("y [m]")
ax3d.set_zlabel("z [m]")
ax3d.set_title("Traiettoria 3D: reale vs replay")
ax3d.legend()

# --- Serie temporali per asse ---
ax_t = fig.add_subplot(1, 3, 2)
axes_labels = ["x", "y", "z"]
colors = ["tab:blue", "tab:orange", "tab:green"]
for i, (label, color) in enumerate(zip(axes_labels, colors)):
    ax_t.plot(demo[0], demo[i + 1], color=color, linestyle="-", label=f"demo {label}")
    ax_t.plot(replay[0], replay[i + 1], color=color, linestyle="--", alpha=0.7)

ax_t.set_xlabel("t [s]")
ax_t.set_ylabel("posizione [m]")
ax_t.set_title("Demo reale (continua) vs Replay (tratteggiata)")
ax_t.legend()

# --- Orientamento: componenti quaternione nel tempo ---
ax_q = fig.add_subplot(1, 3, 3)
if demo[8] and replay[8]:
    quat_labels = ["qw", "qx", "qy", "qz"]
    quat_colors = ["tab:purple", "tab:blue", "tab:orange", "tab:green"]
    for i, (label, color) in enumerate(zip(quat_labels, quat_colors)):
        ax_q.plot(demo[0], demo[i + 4], color=color, linestyle="-", label=f"demo {label}")
        ax_q.plot(replay[0], replay[i + 4], color=color, linestyle="--", alpha=0.7)
    ax_q.set_xlabel("t [s]")
    ax_q.set_ylabel("componenti quaternione")
    ax_q.set_title("Orientamento: Demo (continua) vs Replay (tratteggiata)")
    ax_q.legend()
else:
    ax_q.text(0.5, 0.5, "Nessun dato di orientamento nel CSV", ha="center", va="center")
    ax_q.set_axis_off()

os.makedirs("plots", exist_ok=True)
plt.tight_layout()
out_plot1 = os.path.join("plots", "real_demo_plot.png")
plt.savefig(out_plot1, dpi=150)
print(f"Salvato {out_plot1}")

# --- Errore angolare nel tempo (diagnostica Step 1: localizzazione del picco) ---
if angular_errors is not None:
    fig2, ax_err = plt.subplots(figsize=(10, 5))
    ax_err.plot(t_common, angular_errors, color="tab:red")
    idx_max = angular_errors.index(max(angular_errors))
    ax_err.axvline(t_common[idx_max], color="k", linestyle="--",
                    label=f"max = {angular_errors[idx_max]:.2f}\u00b0 @ t={t_common[idx_max]:.2f}s")
    ax_err.set_xlabel("t [s]")
    ax_err.set_ylabel("errore angolare [deg]")
    ax_err.set_title("Errore angolare nel tempo")
    ax_err.legend()
    fig2.tight_layout()
    out_plot2 = os.path.join("plots", "angular_error_over_time.png")
    fig2.savefig(out_plot2, dpi=150)
    print(f"Salvato {out_plot2}")

plt.show()
