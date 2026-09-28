#!/usr/bin/env python3
"""Heatmap 2D del rapporto rotazione cumulativa/netta (indicatore di
stabilita') per gli sweep di guadagni traslazione/rotazione.

Uso:
    python3 plot_gain_sweep_heatmap.py translation_sweep_summary.csv trans_stiffness trans_damping
    python3 plot_gain_sweep_heatmap.py rotation_sweep_summary.csv rot_stiffness rot_damping
"""
import sys
import csv
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

if len(sys.argv) != 4:
    print("Uso: python3 plot_gain_sweep_heatmap.py <csv> <col_k> <col_d>")
    sys.exit(1)

csv_path, col_k, col_d = sys.argv[1], sys.argv[2], sys.argv[3]

rows = list(csv.DictReader(open(csv_path)))
k_vals = sorted(set(float(r[col_k]) for r in rows))
d_vals = sorted(set(float(r[col_d]) for r in rows))

ratio_grid = np.full((len(d_vals), len(k_vals)), np.nan)
for r in rows:
    if r["ratio"] == "NA":
        continue
    ki = k_vals.index(float(r[col_k]))
    di = d_vals.index(float(r[col_d]))
    ratio_grid[di, ki] = float(r["ratio"])

fig, ax = plt.subplots(figsize=(9, 7))
im = ax.imshow(ratio_grid, origin="lower", aspect="auto", cmap="RdYlGn_r",
               vmin=1.0, vmax=min(50, np.nanmax(ratio_grid)) if np.nanmax(ratio_grid) > 1 else 10)
ax.set_xticks(range(len(k_vals))); ax.set_xticklabels(k_vals)
ax.set_yticks(range(len(d_vals))); ax.set_yticklabels(d_vals)
ax.set_xlabel(col_k)
ax.set_ylabel(col_d)
ax.set_title(f"Rapporto rotazione cumulativa/netta (verde=stabile, rosso=oscilla)\n{os.path.basename(csv_path)}")

for di in range(len(d_vals)):
    for ki in range(len(k_vals)):
        val = ratio_grid[di, ki]
        if not np.isnan(val):
            ax.text(ki, di, f"{val:.1f}", ha="center", va="center", fontsize=8)

fig.colorbar(im, label="ratio (target~2.08)")
fig.tight_layout()
out_path = csv_path.replace(".csv", "_heatmap.png")
fig.savefig(out_path, dpi=150)
print(f"Saved {out_path}")
