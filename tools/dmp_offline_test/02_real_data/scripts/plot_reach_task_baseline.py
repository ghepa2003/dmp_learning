#!/usr/bin/env python3
import os
import csv
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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
    return (np.array(t), np.array(x), np.array(y), np.array(z),
            np.array(qw), np.array(qx), np.array(qy), np.array(qz), has_quat)

def calculate_metrics(demo, replay):
    n = min(len(demo[0]), len(replay[0]))
    t = demo[0][:n]
    
    # Position errors in mm
    dx = (replay[1][:n] - demo[1][:n]) * 1000.0
    dy = (replay[2][:n] - demo[2][:n]) * 1000.0
    dz = (replay[3][:n] - demo[3][:n]) * 1000.0
    
    pos_err = np.sqrt(dx**2 + dy**2 + dz**2)
    rmse_x = np.sqrt(np.mean(dx**2))
    rmse_y = np.sqrt(np.mean(dy**2))
    rmse_z = np.sqrt(np.mean(dz**2))
    rmse_total = np.sqrt(np.mean(pos_err**2))
    max_pos_err = np.max(pos_err)
    
    # Endpoint position error in mm
    ep_dx = (replay[1][-1] - demo[1][-1]) * 1000.0
    ep_dy = (replay[2][-1] - demo[2][-1]) * 1000.0
    ep_dz = (replay[3][-1] - demo[3][-1]) * 1000.0
    ep_pos_err = np.sqrt(ep_dx**2 + ep_dy**2 + ep_dz**2)
    
    # Angular errors in deg
    ang_errors = []
    for k in range(n):
        q_demo = np.array([demo[4][k], demo[5][k], demo[6][k], demo[7][k]])
        q_rep = np.array([replay[4][k], replay[5][k], replay[6][k], replay[7][k]])
        dot = abs(np.dot(q_demo, q_rep) / (np.linalg.norm(q_demo) * np.linalg.norm(q_rep)))
        dot = np.clip(dot, -1.0, 1.0)
        ang_errors.append(2.0 * math.acos(dot) * 180.0 / math.pi)
    ang_errors = np.array(ang_errors)
    
    mean_ang_err = np.mean(ang_errors)
    max_ang_err = np.max(ang_errors)
    
    # Endpoint angular error
    q_demo_end = np.array([demo[4][-1], demo[5][-1], demo[6][-1], demo[7][-1]])
    q_rep_end = np.array([replay[4][-1], replay[5][-1], replay[6][-1], replay[7][-1]])
    dot_end = abs(np.dot(q_demo_end, q_rep_end) / (np.linalg.norm(q_demo_end) * np.linalg.norm(q_rep_end)))
    dot_end = np.clip(dot_end, -1.0, 1.0)
    ep_ang_err = 2.0 * math.acos(dot_end) * 180.0 / math.pi
    
    return {
        "rmse_x_mm": rmse_x,
        "rmse_y_mm": rmse_y,
        "rmse_z_mm": rmse_z,
        "rmse_total_mm": rmse_total,
        "max_pos_err_mm": max_pos_err,
        "endpoint_pos_err_mm": ep_pos_err,
        "mean_ang_err_deg": mean_ang_err,
        "max_ang_err_deg": max_ang_err,
        "endpoint_ang_err_deg": ep_ang_err,
        "pos_errors_mm": pos_err,
        "ang_errors_deg": ang_errors,
        "t": t
    }

def main():
    demo_path = "/home/lorenzo/thesis_ws/real_demo/reach_task_baseline.csv"
    replay_path = "/home/lorenzo/thesis_ws/tools/dmp_offline_test/data/replay_reach_task_baseline_nbasis200.csv"
    
    demo = load_csv(demo_path)
    replay = load_csv(replay_path)
    
    metrics = calculate_metrics(demo, replay)
    
    print("=" * 60)
    print("RISULTATI APPRENDIMENTO DMP - reach_task_baseline (n_basis=200)")
    print("=" * 60)
    print(f"Campioni: {len(metrics['t'])}, Durata: {metrics['t'][-1] - metrics['t'][0]:.3f} s")
    print("\n--- ERRORI DI POSIZIONE ---")
    print(f"RMSE X:           {metrics['rmse_x_mm']:.4f} mm")
    print(f"RMSE Y:           {metrics['rmse_y_mm']:.4f} mm")
    print(f"RMSE Z:           {metrics['rmse_z_mm']:.4f} mm")
    print(f"RMSE Totale:      {metrics['rmse_total_mm']:.4f} mm")
    print(f"Errore Max Pos:   {metrics['max_pos_err_mm']:.4f} mm")
    print(f"Errore Finale Ep: {metrics['endpoint_pos_err_mm']:.4f} mm")
    print("\n--- ERRORI DI ORIENTAMENTO ---")
    print(f"Errore Angolare Medio:  {metrics['mean_ang_err_deg']:.4f}° ({metrics['mean_ang_err_deg']*math.pi/180.0:.6f} rad)")
    print(f"Errore Angolare Max:    {metrics['max_ang_err_deg']:.4f}° ({metrics['max_ang_err_deg']*math.pi/180.0:.6f} rad)")
    print(f"Errore Angolare Finale: {metrics['endpoint_ang_err_deg']:.4f}° ({metrics['endpoint_ang_err_deg']*math.pi/180.0:.6f} rad)")
    print("=" * 60)
    
    # -------------------------------------------------------------
    # PLOT CLASSICO A 3 FIGURE (3 SUBPLOTS)
    # -------------------------------------------------------------
    fig = plt.figure(figsize=(19, 6))
    fig.suptitle("DMP Learning & Replay: reach_task_baseline (n_basis = 200, Window = 0.20 s, Ridge $\\lambda=10^{-6}$)", 
                 fontsize=14, fontweight="bold", y=0.98)
    
    # 1. 3D Trajectory
    ax3d = fig.add_subplot(1, 3, 1, projection="3d")
    ax3d.plot(demo[1], demo[2], demo[3], label="Dimostrazione Reale", linewidth=2.0, color="#1f77b4")
    ax3d.plot(replay[1], replay[2], replay[3], "--", label="Replay DMP (n_basis=200)", linewidth=1.8, color="#d62728")
    ax3d.scatter([demo[1][0]], [demo[2][0]], [demo[3][0]], color="green", s=40, label="Start", zorder=5)
    ax3d.scatter([demo[1][-1]], [demo[2][-1]], [demo[3][-1]], color="black", s=40, marker="x", label="Goal", zorder=5)
    ax3d.set_xlabel("X [m]", labelpad=8)
    ax3d.set_ylabel("Y [m]", labelpad=8)
    ax3d.set_zlabel("Z [m]", labelpad=8)
    ax3d.set_title("1. Traiettoria 3D nello Spazio Operativo", pad=10, fontweight="semibold")
    ax3d.legend(loc="upper right", fontsize=8, framealpha=0.9)
    ax3d.grid(True, linestyle=":", alpha=0.6)
    
    # 2. Position Time Series
    ax_t = fig.add_subplot(1, 3, 2)
    t_demo = demo[0] - demo[0][0]
    t_rep = replay[0] - replay[0][0]
    ax_t.plot(t_demo, demo[1], color="tab:blue", label="Demo x", linewidth=1.5)
    ax_t.plot(t_rep, replay[1], color="tab:blue", linestyle="--", alpha=0.85, label="Replay x")
    ax_t.plot(t_demo, demo[2], color="tab:orange", label="Demo y", linewidth=1.5)
    ax_t.plot(t_rep, replay[2], color="tab:orange", linestyle="--", alpha=0.85, label="Replay y")
    ax_t.plot(t_demo, demo[3], color="tab:green", label="Demo z", linewidth=1.5)
    ax_t.plot(t_rep, replay[3], color="tab:green", linestyle="--", alpha=0.85, label="Replay z")
    ax_t.set_xlabel("Tempo [s]")
    ax_t.set_ylabel("Posizione [m]")
    ax_t.set_title("2. Posizione nel Tempo (Reale vs Replay)", pad=10, fontweight="semibold")
    ax_t.legend(loc="upper right", fontsize=8, ncol=3, framealpha=0.9)
    ax_t.grid(True, linestyle=":", alpha=0.6)
    
    # 3. Orientation Time Series (Quaternion components)
    ax_q = fig.add_subplot(1, 3, 3)
    quat_labels = ["qw", "qx", "qy", "qz"]
    quat_colors = ["tab:purple", "tab:blue", "tab:orange", "tab:green"]
    for i, (lbl, color) in enumerate(zip(quat_labels, quat_colors)):
        ax_q.plot(t_demo, demo[i + 4], color=color, label=f"Demo {lbl}", linewidth=1.5)
        ax_q.plot(t_rep, replay[i + 4], color=color, linestyle="--", alpha=0.85, label=f"Replay {lbl}")
    ax_q.set_xlabel("Tempo [s]")
    ax_q.set_ylabel("Componenti Quaternione")
    ax_q.set_title("3. Orientamento nel Tempo (Reale vs Replay)", pad=10, fontweight="semibold")
    ax_q.legend(loc="upper right", fontsize=8, ncol=2, framealpha=0.9)
    ax_q.grid(True, linestyle=":", alpha=0.6)
    
    plt.tight_layout()
    
    # Save plots
    out_dir1 = "/home/lorenzo/thesis_ws/tools/dmp_offline_test/02_real_data/plots"
    out_dir2 = "/home/lorenzo/thesis_ws/real_demo"
    os.makedirs(out_dir1, exist_ok=True)
    os.makedirs(out_dir2, exist_ok=True)
    
    plot_path1 = os.path.join(out_dir1, "reach_task_baseline_nbasis200_3plot.png")
    plot_path2 = os.path.join(out_dir2, "reach_task_baseline_nbasis200_3plot.png")
    fig.savefig(plot_path1, dpi=200, bbox_inches="tight")
    fig.savefig(plot_path2, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"Plot salvato in:\n  - {plot_path1}\n  - {plot_path2}")
    
    # -------------------------------------------------------------
    # PLOT DETTAGLIATO DEGLI ERRORI (POSIZIONE + ORIENTAMENTO NEL TEMPO)
    # -------------------------------------------------------------
    fig_err, (ax_epos, ax_eang) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    fig_err.suptitle("Errori di Riproduzione nel Tempo: reach_task_baseline (n_basis = 200)", 
                     fontsize=13, fontweight="bold")
    
    t_eval = metrics["t"] - metrics["t"][0]
    ax_epos.plot(t_eval, metrics["pos_errors_mm"], color="#d62728", linewidth=1.5, label="Errore Posizione Euclideo")
    ax_epos.axhline(metrics["rmse_total_mm"], color="black", linestyle="--", alpha=0.7, 
                    label=f"RMSE Totale = {metrics['rmse_total_mm']:.3f} mm")
    ax_epos.set_ylabel("Errore Posizione [mm]")
    ax_epos.set_title("Errore di Posizione Istantaneo")
    ax_epos.legend(loc="upper right", fontsize=9)
    ax_epos.grid(True, linestyle=":", alpha=0.6)
    
    ax_eang.plot(t_eval, metrics["ang_errors_deg"], color="#9467bd", linewidth=1.5, label="Errore Angolare Geodetico")
    ax_eang.axhline(metrics["mean_ang_err_deg"], color="black", linestyle="--", alpha=0.7, 
                    label=f"Errore Medio = {metrics['mean_ang_err_deg']:.3f}°")
    ax_eang.set_xlabel("Tempo [s]")
    ax_eang.set_ylabel("Errore Angolare [deg]")
    ax_eang.set_title("Errore di Orientamento Istantaneo")
    ax_eang.legend(loc="upper right", fontsize=9)
    ax_eang.grid(True, linestyle=":", alpha=0.6)
    
    plt.tight_layout()
    err_plot_path1 = os.path.join(out_dir1, "reach_task_baseline_nbasis200_errors.png")
    err_plot_path2 = os.path.join(out_dir2, "reach_task_baseline_nbasis200_errors.png")
    fig_err.savefig(err_plot_path1, dpi=200, bbox_inches="tight")
    fig_err.savefig(err_plot_path2, dpi=200, bbox_inches="tight")
    plt.close(fig_err)
    print(f"Plot errori salvato in:\n  - {err_plot_path1}\n  - {err_plot_path2}")

if __name__ == "__main__":
    main()
