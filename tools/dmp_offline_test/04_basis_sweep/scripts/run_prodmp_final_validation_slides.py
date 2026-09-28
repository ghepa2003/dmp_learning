#!/usr/bin/env python3
"""
Master runner for ProDMP final validation and slide figures generation.
Covers:
  - Task 1: n_basis sweep with final config (lambda=1e-10, window=0.05s, fix-goal=true)
  - Task 2: window sweep with final config (n_basis=80, lambda=1e-10, fix-goal=true)
  - Task 3: lambda sweep reformatting with final config highlighted (from existing data)
  - Task 4: Final trajectory replay (demo vs replay, config definitiva on Trajectory C)
  - Task 5: ProDMP Goal generalization (retarget to 5 new goals) + Table
All figures saved with _slidestyle.png styling.
"""

import os
import sys
import math
import subprocess
import tempfile
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# Matplotlib global style for slides
plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["DejaVu Sans", "Helvetica", "Arial"],
    "figure.facecolor": "#FFFFFF",
    "axes.facecolor": "#FFFFFF",
    "savefig.facecolor": "#FFFFFF",
    "axes.edgecolor": "#CCCCCC",
    "axes.linewidth": 1.0,
    "grid.color": "#CCCCCC",
    "grid.linestyle": ":",
    "grid.alpha": 0.35,
    "legend.frameon": True,
    "legend.facecolor": "#FFFFFF",
    "legend.edgecolor": "#DDDDDD",
    "legend.framealpha": 0.95,
    "text.color": "#222222",
    "axes.labelcolor": "#222222",
    "xtick.color": "#333333",
    "ytick.color": "#333333",
    "font.size": 10.5,
    "axes.titlesize": 12,
    "axes.labelsize": 11,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "legend.fontsize": 9.5,
})

WORKSPACE_DIR = "/home/lorenzo/thesis_ws"
SLIDES_DIR = os.path.join(WORKSPACE_DIR, "slides_material")
OFFLINE_DIR = os.path.join(WORKSPACE_DIR, "tools/dmp_offline_test")
LEARN_TEST_BIN = os.path.join(OFFLINE_DIR, "build/learn_and_test_prodmp")
GOAL_GEN_BIN = os.path.join(OFFLINE_DIR, "06_goal_generalization/plots/build/run_prodmp_goal_generalization")

DEMOS = {
    "Trajectory C": os.path.join(WORKSPACE_DIR, "demo_raw_trajC.csv"),
    "Trajectory A": os.path.join(WORKSPACE_DIR, "demo_raw_trajA.csv"),
    "Reach Task": os.path.join(WORKSPACE_DIR, "reach_task_baseline.csv"),
}

COLORS = {
    "Trajectory C": "#1f77b4",  # Blue
    "Trajectory A": "#ff7f0e",  # Orange
    "Reach Task": "#2ca02c",    # Green
}

MARKERS = {
    "Trajectory C": "o",
    "Trajectory A": "s",
    "Reach Task": "^",
}

def run_prodmp_single(demo_path, n_basis, ridge_lambda, window_sec, fix_goal=True, label="prodmp_run"):
    with tempfile.TemporaryDirectory() as tmp_dir:
        weights_path = os.path.join(tmp_dir, f"weights_{label}.yaml")
        replay_path = os.path.join(tmp_dir, f"replay_{label}.csv")
        summary_csv = os.path.join(tmp_dir, f"summary_{label}.csv")

        cmd = [
            LEARN_TEST_BIN,
            demo_path,
            weights_path,
            replay_path,
            summary_csv,
            label,
            str(n_basis),
            "-", "-", "-",
            "--lambda", str(ridge_lambda),
            "--window", str(window_sec),
        ]
        if fix_goal:
            cmd.append("--fix-goal")

        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if res.returncode != 0:
            print(f"Error executing ProDMP ({label}):\n{res.stderr}")
            raise RuntimeError(f"ProDMP execution failed for {label}")

        df = pd.read_csv(summary_csv)
        row = df.iloc[-1].to_dict()
        
        # Load replay CSV if needed
        replay_df = pd.read_csv(replay_path)
        return row, replay_df

def run_task1_nbasis_sweep():
    print("\n" + "="*70)
    print("TASK 1: n_basis sweep (lambda=1e-10, window=0.05s, fix_goal=true)")
    print("="*70)
    
    csv_out = os.path.join(SLIDES_DIR, "data_fig12_nbasis_sweep_final_config.csv")
    if os.path.exists(csv_out):
        print(f"Loading cached Task 1 data from {csv_out}")
        df_res = pd.read_csv(csv_out)
    else:
        n_basis_grid = [10, 20, 30, 50, 65, 80, 100, 150, 200, 300]
        ridge_lambda = 1e-10
        window_sec = 0.05
        
        results = []
        for demo_name, demo_path in DEMOS.items():
            print(f"\n--- Running sweep on {demo_name} ---")
            for nb in n_basis_grid:
                lbl = f"{demo_name}_nb{nb}"
                row, _ = run_prodmp_single(demo_path, nb, ridge_lambda, window_sec, fix_goal=True, label=lbl)
                rmse = row.get("rmse_overall_mm", row.get("rmse_total_mm", float("nan")))
                final_err = row.get("endpoint_pos_error_mm", float("nan"))
                print(f"  n_basis={nb:3d} | RMSE={rmse:.4f} mm | Final Err={final_err:.4f} mm")
                results.append({
                    "demo": demo_name,
                    "n_basis": nb,
                    "rmse_overall_mm": rmse,
                    "final_position_error_mm": final_err,
                })
                
        df_res = pd.DataFrame(results)
        df_res.to_csv(csv_out, index=False)
        print(f"\nSaved Task 1 data to: {csv_out}")

    # Plotting Fig 12
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5.5), dpi=300)
    
    for demo_name in DEMOS.keys():
        sub = df_res[df_res["demo"] == demo_name].sort_values("n_basis")
        color = COLORS[demo_name]
        marker = MARKERS[demo_name]
        
        x_vals = sub["n_basis"].to_numpy()
        y_rmse = sub["rmse_overall_mm"].to_numpy()
        y_err = sub["final_position_error_mm"].to_numpy()

        # Panel 1: RMSE overall
        ax1.plot(x_vals, y_rmse, label=demo_name, color=color,
                 marker=marker, markersize=5, linewidth=1.8, alpha=0.9)
        
        # Highlight n_basis = 80
        pt_80 = sub[sub["n_basis"] == 80]
        if not pt_80.empty:
            ax1.scatter(pt_80["n_basis"].to_numpy(), pt_80["rmse_overall_mm"].to_numpy(), color=color,
                        edgecolor="#222222", s=90, linewidth=1.5, zorder=6)
        
        # Panel 2: Final Error
        ax2.plot(x_vals, y_err, label=demo_name, color=color,
                 marker=marker, markersize=5, linewidth=1.8, alpha=0.9)
        if not pt_80.empty:
            ax2.scatter(pt_80["n_basis"].to_numpy(), pt_80["final_position_error_mm"].to_numpy(), color=color,
                        edgecolor="#222222", s=90, linewidth=1.5, zorder=6)

    # Annotate n_basis = 80 choice
    ax1.axvline(80, color="#d62728", linestyle="--", linewidth=1.2, alpha=0.7, label=r"Scelta Finale ($n=80$)")
    ax2.axvline(80, color="#d62728", linestyle="--", linewidth=1.2, alpha=0.7, label=r"Scelta Finale ($n=80$)")

    ax1.set_title(r"Fidelity Tracking: RMSE Overall vs $n_{\mathrm{basis}}$", fontweight="semibold")
    ax1.set_xlabel(r"Numero Funzioni Base ($n_{\mathrm{basis}}$)")
    ax1.set_ylabel("RMSE Overall [mm]")
    ax1.grid(True)
    ax1.legend(loc="upper right")

    ax2.set_title(r"Errore di Posizionamento Finale vs $n_{\mathrm{basis}}$", fontweight="semibold")
    ax2.set_xlabel(r"Numero Funzioni Base ($n_{\mathrm{basis}}$)")
    ax2.set_ylabel("Errore Finale di Posizione [mm]")
    ax2.grid(True)
    ax2.legend(loc="upper right")

    plt.tight_layout()
    fig12_path = os.path.join(SLIDES_DIR, "fig12_nbasis_sweep_final_config_slidestyle.png")
    fig.savefig(fig12_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved Fig 12 to: {fig12_path}")
    return df_res

def run_task2_window_sweep():
    print("\n" + "="*70)
    print("TASK 2: window sweep (n_basis=80, lambda=1e-10, fix_goal=true)")
    print("="*70)
    
    window_grid = [0.01, 0.02, 0.03, 0.05, 0.08, 0.10, 0.15, 0.20]
    n_basis = 80
    ridge_lambda = 1e-10
    
    results = []
    for demo_name, demo_path in DEMOS.items():
        print(f"\n--- Running sweep on {demo_name} ---")
        for w in window_grid:
            lbl = f"{demo_name}_w{w:.2f}"
            row, _ = run_prodmp_single(demo_path, n_basis, ridge_lambda, w, fix_goal=True, label=lbl)
            rmse = row.get("rmse_overall_mm", row.get("rmse_total_mm", float("nan")))
            final_err = row.get("endpoint_pos_error_mm", float("nan"))
            print(f"  window={w:0.2f}s | RMSE={rmse:.4f} mm | Final Err={final_err:.4f} mm")
            results.append({
                "demo": demo_name,
                "window_sec": w,
                "rmse_overall_mm": rmse,
                "final_position_error_mm": final_err,
            })
            
    df_res = pd.DataFrame(results)
    csv_out = os.path.join(SLIDES_DIR, "data_fig13_window_sweep_final_config.csv")
    df_res.to_csv(csv_out, index=False)
    print(f"\nSaved Task 2 data to: {csv_out}")

    # Plotting Fig 13
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5.5), dpi=300)
    
    for demo_name in DEMOS.keys():
        sub = df_res[df_res["demo"] == demo_name].sort_values("window_sec")
        color = COLORS[demo_name]
        marker = MARKERS[demo_name]
        
        x_vals = sub["window_sec"].to_numpy()
        y_rmse = sub["rmse_overall_mm"].to_numpy()
        y_err = sub["final_position_error_mm"].to_numpy()

        # Panel 1: RMSE overall
        ax1.plot(x_vals, y_rmse, label=demo_name, color=color,
                 marker=marker, markersize=5, linewidth=1.8, alpha=0.9)
        
        # Highlight window = 0.05
        pt_05 = sub[sub["window_sec"] == 0.05]
        if not pt_05.empty:
            ax1.scatter(pt_05["window_sec"].to_numpy(), pt_05["rmse_overall_mm"].to_numpy(), color=color,
                        edgecolor="#222222", s=90, linewidth=1.5, zorder=6)
        
        # Panel 2: Final Error
        ax2.plot(x_vals, y_err, label=demo_name, color=color,
                 marker=marker, markersize=5, linewidth=1.8, alpha=0.9)
        if not pt_05.empty:
            ax2.scatter(pt_05["window_sec"].to_numpy(), pt_05["final_position_error_mm"].to_numpy(), color=color,
                        edgecolor="#222222", s=90, linewidth=1.5, zorder=6)

    ax1.axvline(0.05, color="#d62728", linestyle="--", linewidth=1.2, alpha=0.7, label=r"Scelta Finale ($w=0.05\,\mathrm{s}$)")
    ax2.axvline(0.05, color="#d62728", linestyle="--", linewidth=1.2, alpha=0.7, label=r"Scelta Finale ($w=0.05\,\mathrm{s}$)")

    ax1.set_title("Fidelity Tracking: RMSE Overall vs Filtro Posizione", fontweight="semibold")
    ax1.set_xlabel("Finestra Filtro Posizione [s]")
    ax1.set_ylabel("RMSE Overall [mm]")
    ax1.grid(True)
    ax1.legend(loc="upper left")

    ax2.set_title("Errore di Posizionamento Finale vs Filtro Posizione", fontweight="semibold")
    ax2.set_xlabel("Finestra Filtro Posizione [s]")
    ax2.set_ylabel("Errore Finale di Posizione [mm]")
    ax2.grid(True)
    ax2.legend(loc="upper right")

    plt.tight_layout()
    fig13_path = os.path.join(SLIDES_DIR, "fig13_window_sweep_final_config_slidestyle.png")
    fig.savefig(fig13_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved Fig 13 to: {fig13_path}")
    return df_res

def run_task3_lambda_sweep():
    print("\n" + "="*70)
    print("TASK 3: lambda sweep reformatting (from data_fig8_lambda_sweep_fixed_goal.csv)")
    print("="*70)
    
    csv_in = os.path.join(SLIDES_DIR, "data_fig8_lambda_sweep_fixed_goal.csv")
    df_res = pd.read_csv(csv_in)

    # Plotting Fig 14
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5.5), dpi=300)
    
    for demo_name in DEMOS.keys():
        sub = df_res[df_res["demo"] == demo_name].sort_values("ridge_lambda")
        color = COLORS[demo_name]
        marker = MARKERS[demo_name]
        
        x_vals = sub["ridge_lambda"].to_numpy()
        y_rmse = sub["rmse_overall_mm"].to_numpy()
        y_err = sub["final_position_error_mm"].to_numpy()

        # Panel 1: RMSE overall
        ax1.plot(x_vals, y_rmse, label=demo_name, color=color,
                 marker=marker, markersize=5, linewidth=1.8, alpha=0.9)
        
        # Highlight lambda = 1e-10
        pt_10 = sub[sub["ridge_lambda"] == 1e-10]
        if not pt_10.empty:
            ax1.scatter(pt_10["ridge_lambda"].to_numpy(), pt_10["rmse_overall_mm"].to_numpy(), color=color,
                        edgecolor="#222222", s=90, linewidth=1.5, zorder=6)
        
        # Panel 2: Final Error
        ax2.plot(x_vals, y_err, label=demo_name, color=color,
                 marker=marker, markersize=5, linewidth=1.8, alpha=0.9)
        if not pt_10.empty:
            ax2.scatter(pt_10["ridge_lambda"].to_numpy(), pt_10["final_position_error_mm"].to_numpy(), color=color,
                        edgecolor="#222222", s=90, linewidth=1.5, zorder=6)

    ax1.axvline(1e-10, color="#d62728", linestyle="--", linewidth=1.2, alpha=0.7, label=r"Scelta Finale ($\lambda=10^{-10}$)")
    ax2.axvline(1e-10, color="#d62728", linestyle="--", linewidth=1.2, alpha=0.7, label=r"Scelta Finale ($\lambda=10^{-10}$)")

    ax1.set_xscale("log")
    ax2.set_xscale("log")

    ax1.set_title(r"Fidelity Tracking: RMSE Overall vs Regolarizzazione $\lambda$", fontweight="semibold")
    ax1.set_xlabel(r"Parametro Ridge $\lambda$")
    ax1.set_ylabel("RMSE Overall [mm]")
    ax1.grid(True, which="both")
    ax1.legend(loc="upper left")

    ax2.set_title(r"Errore di Posizionamento Finale vs Regolarizzazione $\lambda$", fontweight="semibold")
    ax2.set_xlabel(r"Parametro Ridge $\lambda$")
    ax2.set_ylabel("Errore Finale di Posizione [mm]")
    ax2.grid(True, which="both")
    ax2.legend(loc="upper left")

    plt.tight_layout()
    fig14_path = os.path.join(SLIDES_DIR, "fig14_lambda_sweep_final_config_slidestyle.png")
    fig.savefig(fig14_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved Fig 14 to: {fig14_path}")

def run_task4_final_trajectory_replay():
    print("\n" + "="*70)
    print("TASK 4: Final trajectory replay on Trajectory C")
    print("="*70)
    
    demo_path = DEMOS["Trajectory C"]
    n_basis = 80
    ridge_lambda = 1e-10
    window_sec = 0.05
    fix_goal = True
    
    row, replay_df = run_prodmp_single(demo_path, n_basis, ridge_lambda, window_sec, fix_goal, label="final_replay_trajC")
    demo_df = pd.read_csv(demo_path)

    # Compute detailed metrics
    dx = (replay_df["x"].to_numpy() - demo_df["x"].to_numpy()) * 1000.0
    dy = (replay_df["y"].to_numpy() - demo_df["y"].to_numpy()) * 1000.0
    dz = (replay_df["z"].to_numpy() - demo_df["z"].to_numpy()) * 1000.0
    pos_err = np.sqrt(dx**2 + dy**2 + dz**2)
    rmse_x = np.sqrt(np.mean(dx**2))
    rmse_y = np.sqrt(np.mean(dy**2))
    rmse_z = np.sqrt(np.mean(dz**2))
    rmse_total = np.sqrt(np.mean(pos_err**2))
    max_pos_err = np.max(pos_err)
    final_pos_err = np.sqrt(dx[-1]**2 + dy[-1]**2 + dz[-1]**2)

    # Orientation metrics from QuaternionDMP
    ang_errors = []
    for k in range(len(demo_df)):
        qd = np.array([demo_df["qw"].iloc[k], demo_df["qx"].iloc[k], demo_df["qy"].iloc[k], demo_df["qz"].iloc[k]])
        qr = np.array([replay_df["qw"].iloc[k], replay_df["qx"].iloc[k], replay_df["qy"].iloc[k], replay_df["qz"].iloc[k]])
        dot = abs(np.dot(qd, qr) / (np.linalg.norm(qd) * np.linalg.norm(qr)))
        dot = np.clip(dot, -1.0, 1.0)
        ang_errors.append(2.0 * math.acos(dot) * 180.0 / math.pi)
    ang_errors = np.array(ang_errors)
    mean_ang_err = np.mean(ang_errors)
    final_ang_err = ang_errors[-1]

    print("\n" + "="*50)
    print("METRICHE REPLAY PRODMP DEFINITIVO (Trajectory C)")
    print("="*50)
    print(f"[Final Error - Position]: {final_pos_err:.4f} mm")
    print(f"[Position] RMSE Total:    {rmse_total:.4f} mm (X: {rmse_x:.4f}, Y: {rmse_y:.4f}, Z: {rmse_z:.4f})")
    print(f"[Position] Max Pos Error: {max_pos_err:.4f} mm")
    print(f"[Final Error - Orientation (QuatDMP)]: {final_ang_err:.4f}°")
    print(f"[Orientation] Mean Ang Error:          {mean_ang_err:.4f}°")
    print("="*50 + "\n")

    # Plotting 2-panel figure
    fig = plt.figure(figsize=(14, 5.5), dpi=300)
    
    # Subplot 1: 3D Trajectory
    ax3d = fig.add_subplot(1, 2, 1, projection="3d")
    ax3d.plot(demo_df["x"].to_numpy(), demo_df["y"].to_numpy(), demo_df["z"].to_numpy(),
              label="Dimostrazione Reale", color="#1f77b4", linewidth=2.2, alpha=0.85)
    ax3d.plot(replay_df["x"].to_numpy(), replay_df["y"].to_numpy(), replay_df["z"].to_numpy(),
              "--", label=r"Replay ProDMP Finale ($n=80$)", color="#d62728", linewidth=1.8, alpha=0.95)
    
    # Start and Goal markers
    ax3d.scatter([demo_df["x"].iloc[0]], [demo_df["y"].iloc[0]], [demo_df["z"].iloc[0]],
                 color="#2ca02c", s=60, marker="o", label="Start ($y_0$)", zorder=10)
    ax3d.scatter([demo_df["x"].iloc[-1]], [demo_df["y"].iloc[-1]], [demo_df["z"].iloc[-1]],
                 color="#222222", s=70, marker="X", label="Goal ($g$)", zorder=10)

    ax3d.set_title("1. Traiettoria 3D Spazio Operativo", pad=10, fontweight="semibold")
    ax3d.set_xlabel("X [m]", labelpad=6)
    ax3d.set_ylabel("Y [m]", labelpad=6)
    ax3d.set_zlabel("Z [m]", labelpad=6)
    ax3d.legend(loc="upper right", fontsize=8.5)
    ax3d.grid(True)

    # Subplot 2: Position vs Time
    ax_t = fig.add_subplot(1, 2, 2)
    t = (demo_df["t"] - demo_df["t"].iloc[0]).to_numpy()
    
    ax_t.plot(t, demo_df["x"].to_numpy(), color="#1f77b4", label="Demo x", linewidth=1.6)
    ax_t.plot(t, replay_df["x"].to_numpy(), color="#1f77b4", linestyle="--", linewidth=1.4, alpha=0.85, label="ProDMP x")
    
    ax_t.plot(t, demo_df["y"].to_numpy(), color="#ff7f0e", label="Demo y", linewidth=1.6)
    ax_t.plot(t, replay_df["y"].to_numpy(), color="#ff7f0e", linestyle="--", linewidth=1.4, alpha=0.85, label="ProDMP y")
    
    ax_t.plot(t, demo_df["z"].to_numpy(), color="#2ca02c", label="Demo z", linewidth=1.6)
    ax_t.plot(t, replay_df["z"].to_numpy(), color="#2ca02c", linestyle="--", linewidth=1.4, alpha=0.85, label="ProDMP z")

    ax_t.set_title("2. Posizione nel Tempo (Reale vs Replay)", pad=10, fontweight="semibold")
    ax_t.set_xlabel("Tempo [s]")
    ax_t.set_ylabel("Posizione [m]")
    ax_t.grid(True)
    ax_t.legend(loc="upper right", ncol=3, fontsize=8.5)

    plt.tight_layout()
    fig15_path = os.path.join(SLIDES_DIR, "fig15_final_trajectory_replay_slidestyle.png")
    fig.savefig(fig15_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved Fig 15 to: {fig15_path}")

def run_task5_goal_generalization():
    print("\n" + "="*70)
    print("TASK 5: ProDMP Goal Generalization on Trajectory C (5 new target goals)")
    print("="*70)
    
    out_dir = os.path.join(OFFLINE_DIR, "06_goal_generalization/plots_prodmp")
    os.makedirs(out_dir, exist_ok=True)
    
    demo_path = DEMOS["Trajectory C"]
    cmd = [
        GOAL_GEN_BIN,
        demo_path,
        out_dir,
        "trajC",
        "80",
        "1e-10",
        "0.05"
    ]
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print(res.stdout)
    if res.returncode != 0:
        print(f"Error executing Goal Generalization:\n{res.stderr}")
        raise RuntimeError("Goal Generalization execution failed.")

    # Load data for plotting and table
    demo_df = pd.read_csv(demo_path)
    replay_orig_df = pd.read_csv(os.path.join(out_dir, "data/trajC_replay_orig.csv"))
    goals_info_df = pd.read_csv(os.path.join(out_dir, "data/trajC_goals_info.csv"))

    # 1. Generate 3D Plot
    fig = plt.figure(figsize=(10, 7), dpi=300)
    ax3d = fig.add_subplot(1, 1, 1, projection="3d")

    # Insegnata (Demo Reale)
    ax3d.plot(demo_df["x"].to_numpy(), demo_df["y"].to_numpy(), demo_df["z"].to_numpy(),
              label="Insegnata (Demo Reale)", color="#1f77b4", linewidth=2.5, alpha=0.85)

    # Imparata (Replay Goal Orig)
    ax3d.plot(replay_orig_df["x"].to_numpy(), replay_orig_df["y"].to_numpy(), replay_orig_df["z"].to_numpy(),
              "--", label="Imparata (Goal Orig)", color="#222222", linewidth=2.0, alpha=0.9)

    # Start & Goal Orig markers
    ax3d.scatter([demo_df["x"].iloc[0]], [demo_df["y"].iloc[0]], [demo_df["z"].iloc[0]],
                 color="#2ca02c", s=70, marker="o", label="Inizio ($y_0$)", zorder=10)
    ax3d.scatter([demo_df["x"].iloc[-1]], [demo_df["y"].iloc[-1]], [demo_df["z"].iloc[-1]],
                 color="#222222", s=80, marker="X", label="Goal Orig", zorder=10)

    goal_colors = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3", "#ff7f00"]
    for idx, row in goals_info_df.iterrows():
        gid = int(row["goal_id"])
        color = goal_colors[(gid - 1) % len(goal_colors)]
        g_rep_path = os.path.join(out_dir, f"data/trajC_replay_goal_{gid}.csv")
        g_rep_df = pd.read_csv(g_rep_path)
        
        # Executed trajectory
        ax3d.plot(g_rep_df["x"].to_numpy(), g_rep_df["y"].to_numpy(), g_rep_df["z"].to_numpy(),
                  ":", label=f"Eseguita Goal {gid}", color=color, linewidth=2.0)
        # Reached final position marker
        ax3d.scatter([g_rep_df["x"].iloc[-1]], [g_rep_df["y"].iloc[-1]], [g_rep_df["z"].iloc[-1]],
                     color=color, s=60, marker="^", zorder=9)
        # Requested target marker
        ax3d.scatter([row["gx"]], [row["gy"]], [row["gz"]],
                     color=color, s=90, marker="*", edgecolors="#222222", linewidths=0.5, zorder=11)

    ax3d.set_title("Generalizzazione Spaziale ProDMP su 5 Nuovi Goal (Traiettoria C)", pad=12, fontweight="semibold")
    ax3d.set_xlabel("X [m]", labelpad=8)
    ax3d.set_ylabel("Y [m]", labelpad=8)
    ax3d.set_zlabel("Z [m]", labelpad=8)
    ax3d.legend(loc="upper right", fontsize=8.5, ncol=2)
    ax3d.grid(True)

    plt.tight_layout()
    fig16_path = os.path.join(SLIDES_DIR, "fig16_goal_generalization_slidestyle.png")
    fig.savefig(fig16_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved Fig 16 to: {fig16_path}")

    # 2. Generate Slide-Style Table Image & Markdown
    table_data = []
    for _, row in goals_info_df.iterrows():
        table_data.append([
            f"Goal {int(row['goal_id'])}",
            row["name"].replace(f"Goal {int(row['goal_id'])} (", "").rstrip(")"),
            f"{row['err_pos_mm']:.3f} mm"
        ])
    mean_err = goals_info_df["err_pos_mm"].mean()
    table_data.append(["Media", "Tutti i 5 nuovi goal", f"{mean_err:.3f} mm"])

    col_labels = ["Goal", "Descrizione (Offset)", "Err Posizione Finale [mm]"]
    
    fig_tbl, ax_tbl = plt.subplots(figsize=(10, 3.2), dpi=300)
    ax_tbl.axis("off")

    table = ax_tbl.table(
        cellText=table_data,
        colLabels=col_labels,
        cellLoc="center",
        loc="center",
        colWidths=[0.18, 0.52, 0.30]
    )
    table.auto_set_font_size(False)
    table.set_fontsize(10.5)
    table.scale(1.0, 1.85)

    # Style header and rows
    for (r, c), cell in table.get_celld().items():
        cell.set_edgecolor("#D0D0D0")
        cell.set_linewidth(1.0)
        if r == 0:
            cell.set_facecolor("#2B4C7E")  # Elegant dark blue
            cell.get_text().set_color("#FFFFFF")
            cell.get_text().set_fontweight("bold")
        elif r == len(table_data):
            cell.set_facecolor("#EAEFF8")  # Highlight mean row
            cell.get_text().set_fontweight("bold")
        else:
            cell.set_facecolor("#F9FBFD" if r % 2 == 1 else "#FFFFFF")

    plt.tight_layout()
    table_img_path = os.path.join(SLIDES_DIR, "table_goal_generalization_slidestyle.png")
    fig_tbl.savefig(table_img_path, dpi=300, bbox_inches="tight")
    plt.close(fig_tbl)
    print(f"Saved Table Image to: {table_img_path}")

    # Print markdown table to stdout
    print("\n" + "="*50)
    print("TABELLA GENERALIZZAZIONE SU NUOVI GOAL (ProDMP)")
    print("="*50)
    print(f"| {'Goal':<8} | {'Descrizione (Offset)':<36} | {'Err Posizione Finale [mm]':<26} |")
    print(f"|{'-'*10}|{'-'*38}|{'-'*28}|")
    for r in table_data:
        print(f"| {r[0]:<8} | {r[1]:<36} | {r[2]:<26} |")
    print("="*50 + "\n")

def main():
    os.makedirs(SLIDES_DIR, exist_ok=True)
    df_task1 = run_task1_nbasis_sweep()
    df_task2 = run_task2_window_sweep()
    run_task3_lambda_sweep()
    run_task4_final_trajectory_replay()
    run_task5_goal_generalization()
    print("\n" + "#"*70)
    print("ALL 5 TASKS COMPLETED SUCCESSFULLY!")
    print("#"*70)

if __name__ == "__main__":
    main()
