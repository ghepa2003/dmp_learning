#!/usr/bin/env python3
"""
Joint 3D grid search for ProDMP hyperparameters on Trajectory C.
Explores combinations of (n_basis, ridge_lambda, position_filter_window).
"""

import os
import subprocess
import tempfile
import pandas as pd
import numpy as np

WORKSPACE_DIR = "/home/lorenzo/thesis_ws"
OFFLINE_DIR = os.path.join(WORKSPACE_DIR, "tools/dmp_offline_test")
DEMO_CSV = os.path.join(WORKSPACE_DIR, "demo_raw_trajC.csv")
EXECUTABLE = os.path.join(OFFLINE_DIR, "build/learn_and_test_prodmp")
OUTPUT_CSV = os.path.join(OFFLINE_DIR, "plots/prodmp_joint_grid_search_trajC.csv")

# Grid definition centered around 1D winners:
# n_basis candidates: 30, 40, 50, 60, 80, 100, 150, 200
# ridge_lambda candidates: 1e-10, 1e-9, 1e-8, 1e-7, 1e-6
# window candidates: 0.01, 0.05, 0.10, 0.20

n_basis_list = [30, 40, 50, 60, 80, 100, 150, 200]
ridge_lambda_list = [1e-10, 1e-9, 1e-8, 1e-7, 1e-6]
window_list = [0.01, 0.05, 0.10, 0.20]

os.makedirs(os.path.dirname(OUTPUT_CSV), exist_ok=True)
if os.path.exists(OUTPUT_CSV):
    os.remove(OUTPUT_CSV)

tmp_dir = tempfile.mkdtemp(prefix="prodmp_grid_")

print(f"Starting joint grid search ({len(n_basis_list)} x {len(ridge_lambda_list)} x {len(window_list)} = {len(n_basis_list)*len(ridge_lambda_list)*len(window_list)} configs)...")

configs_to_run = []
for nb in n_basis_list:
    for rl in ridge_lambda_list:
        for w in window_list:
            configs_to_run.append((nb, rl, w))

for i, (nb, rl, w) in enumerate(configs_to_run, 1):
    cfg_content = f"""num_basis: {nb}
ridge_lambda: {rl}
position_filter:
  enabled: true
  window_sec: {w}
"""
    cfg_path = os.path.join(tmp_dir, f"cfg_{i}.yaml")
    with open(cfg_path, "w") as f:
        f.write(cfg_content)
    
    label = f"nb{nb:04d}_lam{rl:.1e}_w{w:.2f}"
    weights_path = os.path.join(tmp_dir, f"weights_{label}.yaml")
    replay_path = os.path.join(tmp_dir, f"replay_{label}.csv")

    cmd = [
        EXECUTABLE,
        DEMO_CSV,
        weights_path,
        replay_path,
        OUTPUT_CSV,
        label,
        str(nb),
        "-", "-", "-",
        cfg_path
    ]
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if res.returncode != 0:
        print(f"Error on {label}: {res.stderr}")
        raise RuntimeError(f"Failed run: {label}")
    if i % 20 == 0 or i == len(configs_to_run):
        print(f"Completed {i}/{len(configs_to_run)} runs...")

print(f"Joint grid search complete! Saved to {OUTPUT_CSV}")

df = pd.read_csv(OUTPUT_CSV)
df = df.sort_values(by="rmse_overall_mm")
print("\nTop 15 Configurations by RMSE overall [mm]:")
print(df[["trial", "rmse_overall_mm", "max_pos_error_mm", "endpoint_pos_error_mm", "mean_angular_error_deg"]].head(15).to_string(index=False))
