#!/usr/bin/env python3
"""
Temporal holdout validation study for ProDMP on Trajectory C and Trajectory A.
"""

import os
import subprocess
import tempfile
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

WORKSPACE_DIR = "/home/lorenzo/thesis_ws"
OFFLINE_DIR = os.path.join(WORKSPACE_DIR, "tools/dmp_offline_test")
EXECUTABLE = os.path.join(OFFLINE_DIR, "build/learn_and_test_prodmp_holdout")

DEMO_C = os.path.join(WORKSPACE_DIR, "demo_raw_trajC.csv")
DEMO_A = os.path.join(WORKSPACE_DIR, "demo_raw_trajA.csv")

PLOT_DIR = os.path.join(OFFLINE_DIR, "plots/07_holdout_prodmp")
os.makedirs(PLOT_DIR, exist_ok=True)

CSV_C = os.path.join(PLOT_DIR, "prodmp_holdout_trajC.csv")
CSV_A = os.path.join(PLOT_DIR, "prodmp_holdout_trajA.csv")

N_BASIS_LIST = [30, 50, 80, 150, 200, 250, 300]
RIDGE_LAMBDA_LIST = [1e-6, 1e-8, 1e-9, 1e-10, 1e-11, 1e-12]
WINDOW_SEC = 0.01
HOLDOUT_FRACTION = 0.2

def run_grid(demo_path, output_csv, traj_name):
    if os.path.exists(output_csv):
        os.remove(output_csv)
    
    tmp_dir = tempfile.mkdtemp()
    
    # 42 grid combinations
    configs = []
    for nb in N_BASIS_LIST:
        for lam in RIDGE_LAMBDA_LIST:
            configs.append((nb, lam, WINDOW_SEC, f"{traj_name}_nb{nb:04d}_lam{lam:.1e}_w{WINDOW_SEC:.2f}"))
            
    # Also add baseline post-fix (n=200, lam=1e-6, w=0.20) for direct side-by-side comparison
    configs.append((200, 1e-6, 0.20, f"{traj_name}_baseline_post_fix_n200_lam1e-6_w0.20"))

    print(f"Running {len(configs)} holdout configurations on {traj_name}...")
    for nb, lam, w, label in configs:
        weights_yaml = os.path.join(tmp_dir, f"weights_{label}.yaml")
        cmd = [
            EXECUTABLE,
            demo_path,
            weights_yaml,
            output_csv,
            label,
            str(nb),
            str(lam),
            str(w),
            str(HOLDOUT_FRACTION)
        ]
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if res.returncode != 0:
            print(f"Error on {label}: {res.stderr}")
            raise RuntimeError(f"Execution failed for {label}")

    df = pd.read_csv(output_csv)
    print(f"Done for {traj_name}. Total rows: {len(df)}")
    return df

print("=== Starting Holdout Study ===")
df_c = run_grid(DEMO_C, CSV_C, "trajC")
df_a = run_grid(DEMO_A, CSV_A, "trajA")

# Generate Scatter Plot: RMSE in-sample vs RMSE holdout for Trajectory C
plt.figure(figsize=(10, 7), dpi=150)
grid_c = df_c[~df_c["label"].str.contains("baseline")].copy()

# Color by n_basis, marker/size by ridge_lambda
scatter = plt.scatter(
    grid_c["rmse_in_sample_mm"],
    grid_c["rmse_holdout_mm"],
    c=grid_c["n_basis"],
    cmap="viridis",
    s=70,
    edgecolors="black",
    linewidths=0.8,
    alpha=0.9
)
cbar = plt.colorbar(scatter)
cbar.set_label("n_basis", fontsize=11)

# Annotate specific key configurations
for _, row in grid_c.iterrows():
    nb = int(row["n_basis"])
    lam = row["ridge_lambda"]
    # Annotate extremes / key points
    if (nb == 300 and lam == 1e-12) or (nb == 30 and lam == 1e-6) or (nb == 200 and lam == 1e-6):
        plt.annotate(
            f"n={nb}, $\\lambda$={lam:.0e}",
            (row["rmse_in_sample_mm"], row["rmse_holdout_mm"]),
            textcoords="offset points",
            xytext=(7, -2),
            fontsize=9,
            weight="bold"
        )

# Plot reference y=x (no overfitting line)
max_val = min(max(grid_c["rmse_in_sample_mm"].max() * 1.5, 2.0), 5.0)
diag_x = np.linspace(0, max_val, 100)
plt.plot(diag_x, diag_x, 'k--', alpha=0.5, label="y = x (zero generalization gap)")
plt.plot(diag_x, 3.0 * diag_x, 'r:', alpha=0.6, label="y = 3x (overfitting threshold > 3x)")

plt.xlabel("In-Sample RMSE [mm] (train: 80% demo)", fontsize=12)
plt.ylabel("Held-Out RMSE [mm] (test: 20% future extrapolation)", fontsize=12)
plt.title("ProDMP Temporal Holdout Generalization on Trajectory C\nIn-Sample vs Held-Out Error across 42 Hyperparameter Sets", fontsize=13)
plt.grid(True, linestyle="--", alpha=0.5)
plt.legend(loc="upper left")

plot_path = os.path.join(PLOT_DIR, "rmse_in_sample_vs_holdout_trajC.png")
plt.tight_layout()
plt.savefig(plot_path)
plt.close()
print(f"Saved scatter plot to: {plot_path}")

