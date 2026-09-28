#!/usr/bin/env python3
"""
Generates reformatted, slide-style figures (300 DPI, clean sans-serif typography,
sober palettes, 16:9 / 2:1 slide box proportions) for Chapter 07:
"07 · PRODMP: OPTIMIZATION & VALIDATION".

Generates:
1. fig1_column_scaling_before_after_slidestyle.png
2. fig3_trajectory_overlay_dmp_vs_prodmp_slidestyle.png
3. fig4_overfitting_scatter_annotated_slidestyle.png
4. fig8_lambda_sweep_fixed_goal_slidestyle.png
5. fig11_perturbation_stability_colored_noise_slidestyle.png
6. table_final_summary_slidestyle.png
"""

import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

WORKSPACE_DIR = "/home/lorenzo/thesis_ws"
SLIDES_DIR = os.path.join(WORKSPACE_DIR, "slides_material")
os.makedirs(SLIDES_DIR, exist_ok=True)

# Common clean slide aesthetic configuration
plt.rcParams.update({
    'font.family': 'sans-serif',
    'font.sans-serif': ['DejaVu Sans', 'Arial', 'Helvetica', 'Liberation Sans'],
    'font.size': 11,
    'axes.labelsize': 12,
    'axes.titlesize': 13,
    'xtick.labelsize': 10.5,
    'ytick.labelsize': 10.5,
    'legend.fontsize': 10.5,
    'figure.titlesize': 14,
    'lines.linewidth': 2.0,
    'lines.markersize': 7,
    'grid.alpha': 0.35,
    'grid.linestyle': '--',
    'figure.facecolor': '#FFFFFF',
    'axes.facecolor': '#FFFFFF',
    'savefig.facecolor': '#FFFFFF',
    'savefig.edgecolor': 'none',
    'figure.autolayout': False
})

# Sober color palette
C_BLUE   = "#1f77b4"
C_ORANGE = "#e66101"
C_GREEN  = "#2ca02c"
C_PURPLE = "#7570b3"
C_RED    = "#d62728"
C_GRAY   = "#555555"
C_LIGHTGRAY = "#888888"

print("================================================================================")
print("GENERATING SLIDE-STYLE FIGURES & SUMMARY TABLE (300 DPI, Pure White BG)")
print("================================================================================\n")

# ==============================================================================
# 1. FIG 1: Column Scaling Before / After
# ==============================================================================
print(">>> 1. Generating fig1_column_scaling_before_after_slidestyle.png...")
csv_fig1 = os.path.join(SLIDES_DIR, "data_fig1_column_norms.csv")
df_f1 = pd.read_csv(csv_fig1)

min_pre = float(df_f1["norm_before_maxabs"].min())
max_pre = float(df_f1["norm_before_maxabs"].max())
ratio_pre = max_pre / min_pre

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.5, 5.2), dpi=300, sharey=True)

rbf_mask = (df_f1["is_goal_col"] == 0).values
goal_mask = (df_f1["is_goal_col"] == 1).values
scaled_mask = (df_f1["scaled_applied"] == 1).values
unscaled_mask = ((df_f1["scaled_applied"] == 0) & (df_f1["is_goal_col"] == 0)).values

# Left: Before
ax1.scatter(df_f1.loc[rbf_mask, "col_idx"].values, df_f1.loc[rbf_mask, "norm_before_maxabs"].values,
            color=C_BLUE, s=28, alpha=0.85, label="Colonne RBF forma ($w_i$)")
ax1.scatter(df_f1.loc[goal_mask, "col_idx"].values, df_f1.loc[goal_mask, "norm_before_maxabs"].values,
            color=C_RED, s=75, marker="^", label="Colonna Step Goal ($g$)")

ax1.set_yscale("log")
ax1.set_ylim(1e-8, 5.0)
ax1.set_xlabel("Indice Colonna $i$ di $H$ ($0 \\dots 200$)")
ax1.set_ylabel("Norma Picco Colonna $\\|H_{:, i}\\|_{\\infty}$ [scala log]")
ax1.set_title("PRIMA: Matrice $H$ Naturale\n(Spread patologico di $\\sim 7$ ordini di grandezza)", fontsize=11.5, pad=8)
ax1.grid(True)
ax1.legend(loc="upper left", framealpha=0.9)

ax1.annotate(f"Spread max/min: {ratio_pre:.1e}\nGoal domina di $10^4 \\times$",
             xy=(200, max_pre), xytext=(65, 8e-3),
             arrowprops=dict(facecolor='black', shrink=0.08, width=1, headwidth=5),
             bbox=dict(boxstyle="round,pad=0.35", fc="#fff9db", ec="#f59f00", alpha=0.9),
             fontsize=9.5)

# Right: After
ax2.scatter(df_f1.loc[scaled_mask, "col_idx"].values, df_f1.loc[scaled_mask, "norm_after_maxabs"].values,
            color=C_GREEN, s=28, alpha=0.85, label="Colonne Scalate a 1.0 ($> 5\\times 10^{-6}$)")
ax2.scatter(df_f1.loc[unscaled_mask, "col_idx"].values, df_f1.loc[unscaled_mask, "norm_after_maxabs"].values,
            color=C_GRAY, s=24, alpha=0.6, marker="x", label="Sotto Floor Relativo (non scalate)")
ax2.scatter(df_f1.loc[goal_mask, "col_idx"].values, df_f1.loc[goal_mask, "norm_after_maxabs"].values,
            color=C_RED, s=75, marker="^", label="Colonna Goal Scalata a 1.0")

floor_val = 5e-6 * max_pre
ax2.axhline(floor_val, color=C_ORANGE, linestyle=":", linewidth=1.6, label="Floor relativo ($5\\times 10^{-6}$)")

ax2.set_yscale("log")
ax2.set_xlabel("Indice Colonna $i$ di $H$ ($0 \\dots 200$)")
ax2.set_title("DOPO: Column-Scale Preconditioning\n(Floor relativo $k_{\\mathrm{floor}} = 5\\times 10^{-6}$)", fontsize=11.5, pad=8)
ax2.grid(True)
ax2.legend(loc="lower right", framealpha=0.9)

ax2.annotate("148 colonne equalizzate a 1.0\nRidge agisce uniformemente",
             xy=(100, 1.0), xytext=(25, 1.5e-2),
             arrowprops=dict(facecolor='black', shrink=0.08, width=1, headwidth=5),
             bbox=dict(boxstyle="round,pad=0.35", fc="#e6fcf5", ec="#20c997", alpha=0.9),
             fontsize=9.5)

plt.tight_layout()
out_fig1 = os.path.join(SLIDES_DIR, "fig1_column_scaling_before_after_slidestyle.png")
plt.savefig(out_fig1, dpi=300)
plt.close()
print(f"  -> Salvato: {out_fig1}\n")


# ==============================================================================
# 2. FIG 3: Trajectory Overlay (Demo vs DMP vs ProDMP)
# ==============================================================================
print(">>> 2. Generating fig3_trajectory_overlay_dmp_vs_prodmp_slidestyle.png...")
demo_csv = os.path.join(WORKSPACE_DIR, "demo_raw_trajC.csv")
replay_dmp_csv = os.path.join(SLIDES_DIR, "replay_dmp_classic_trajC.csv")
replay_prodmp_csv = os.path.join(SLIDES_DIR, "replay_prodmp_baseline_trajC.csv")

df_demo = pd.read_csv(demo_csv)
df_dmp = pd.read_csv(replay_dmp_csv)
df_prodmp = pd.read_csv(replay_prodmp_csv)

t_demo = (df_demo["t"] - df_demo["t"].iloc[0]).values
t_dmp = df_dmp["t"].values
t_prodmp = df_prodmp["t"].values

diff_dmp = (df_dmp[["x", "y", "z"]].values - df_demo[["x", "y", "z"]].values) * 1000.0
rmse_dmp = np.sqrt(np.mean(np.sum(diff_dmp**2, axis=1)))

diff_prodmp = (df_prodmp[["x", "y", "z"]].values - df_demo[["x", "y", "z"]].values) * 1000.0
rmse_prodmp = np.sqrt(np.mean(np.sum(diff_prodmp**2, axis=1)))

fig, axes = plt.subplots(3, 1, figsize=(11.0, 5.8), dpi=300, sharex=True)
coords = ['x', 'y', 'z']
labels_axis = ['X [m]', 'Y [m]', 'Z [m]']

for ax, c, lbl in zip(axes, coords, labels_axis):
    ax.plot(t_demo, df_demo[c].values, color="#222222", linestyle='-', linewidth=2.0, label="Dimostrazione Grezza")
    ax.plot(t_dmp, df_dmp[c].values, color=C_BLUE, linestyle='--', linewidth=1.8, label=f"DMP Classico (RMSE = {rmse_dmp:.3f} mm)")
    ax.plot(t_prodmp, df_prodmp[c].values, color=C_GREEN, linestyle=':', linewidth=2.0, label=f"ProDMP Post-Fix (RMSE = {rmse_prodmp:.3f} mm)")
    ax.set_ylabel(lbl, fontsize=11)
    ax.grid(True)

axes[2].set_xlabel("Tempo [s]", fontsize=11.5)
axes[0].set_title("Overlay Traiettoria su Trajectory C (DMP Classico vs ProDMP Post-Fix)", fontsize=12, pad=10)

handles, labels = axes[0].get_legend_handles_labels()
fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, 0.96), ncol=3, framealpha=0.95, fontsize=10)

plt.tight_layout(rect=[0, 0, 1, 0.90])
out_fig3 = os.path.join(SLIDES_DIR, "fig3_trajectory_overlay_dmp_vs_prodmp_slidestyle.png")
plt.savefig(out_fig3, dpi=300)
plt.close()
print(f"  -> Salvato: {out_fig3}\n")


# ==============================================================================
# 3. FIG 4: Overfitting Scatter Annotated
# ==============================================================================
print(">>> 3. Generating fig4_overfitting_scatter_annotated_slidestyle.png...")
csv_holdout_c = os.path.join(WORKSPACE_DIR, "tools/dmp_offline_test/plots/07_holdout_prodmp/prodmp_holdout_trajC.csv")
df_hc = pd.read_csv(csv_holdout_c)

df_grid = df_hc[~df_hc["label"].str.contains("baseline")].copy()
row_base = df_hc[df_hc["label"].str.contains("baseline")].iloc[0]

row_in_winner = df_grid[(df_grid["n_basis"] == 300) & (df_grid["ridge_lambda"] == 1e-12)].iloc[0]
row_out_winner = df_grid[(df_grid["n_basis"] == 80) & (df_grid["ridge_lambda"] == 1e-9)].iloc[0]

val_in_in = float(row_in_winner['rmse_in_sample_mm'])
val_in_out = float(row_in_winner['rmse_holdout_mm'])
gap_in = val_in_out / val_in_in

val_out_in = float(row_out_winner['rmse_in_sample_mm'])
val_out_out = float(row_out_winner['rmse_holdout_mm'])
gap_out = val_out_out / val_out_in

val_base_in = float(row_base['rmse_in_sample_mm'])
val_base_out = float(row_base['rmse_holdout_mm'])
gap_base = val_base_out / val_base_in

fig, ax = plt.subplots(figsize=(11.0, 5.8), dpi=300)

sc = ax.scatter(
    df_grid["rmse_in_sample_mm"].values,
    df_grid["rmse_holdout_mm"].values,
    c=df_grid["n_basis"].values,
    cmap="viridis",
    s=55,
    alpha=0.85,
    edgecolors="k",
    linewidths=0.5,
    label="Configurazioni ($n \\in [30..300], \\lambda \\in [10^{-12}..10^{-6}]$)"
)
cbar = plt.colorbar(sc, ax=ax, pad=0.02)
cbar.set_label("Numero Funzioni di Base $n_{\\mathrm{basis}}$", fontsize=11)

x_vals = np.linspace(0.05, 2.5, 100)
ax.plot(x_vals, x_vals, 'k--', linewidth=1.3, alpha=0.5, label="$y = x$ (Zero Gap)")
ax.plot(x_vals, 3.0 * x_vals, 'r:', linewidth=1.5, alpha=0.7, label="$y = 3x$ (Soglia Overfitting)")

# Annotate points
ax.scatter(val_in_in, val_in_out, s=180, marker="*", color=C_RED, edgecolors="black", linewidth=1.0, zorder=5)
ax.annotate(f"Vincitore In-Sample (OVERFITTING)\n$n=300, \\lambda=10^{-12}$ | Gap: {gap_in:.0f}x\nTrain: {val_in_in:.2f} mm $\\rightarrow$ Holdout: {val_in_out:.1f} mm",
            xy=(val_in_in, val_in_out),
            xytext=(0.42, 40),
            arrowprops=dict(facecolor=C_RED, shrink=0.08, width=1.2, headwidth=5),
            bbox=dict(boxstyle="round,pad=0.35", fc="#ffe3e3", ec=C_RED, alpha=0.9),
            fontsize=9.5)

ax.scatter(val_out_in, val_out_out, s=140, marker="o", color=C_GREEN, edgecolors="black", linewidth=1.2, zorder=5)
ax.annotate(f"Miglior Generalizzazione Held-Out\n$n=80, \\lambda=10^{-9}$ | Gap: {gap_out:.1f}x\nTrain: {val_out_in:.2f} mm $\\rightarrow$ Holdout: {val_out_out:.2f} mm",
            xy=(val_out_in, val_out_out),
            xytext=(0.58, 8),
            arrowprops=dict(facecolor=C_GREEN, shrink=0.08, width=1.2, headwidth=5),
            bbox=dict(boxstyle="round,pad=0.35", fc="#d3f9d8", ec=C_GREEN, alpha=0.9),
            fontsize=9.5)

ax.scatter(val_base_in, val_base_out, s=120, marker="D", color=C_BLUE, edgecolors="black", linewidth=1.0, zorder=5)
ax.annotate(f"Baseline di Produzione\n$n=200, \\lambda=10^{{-6}}, w=0.20\\,\\mathrm{{s}}$\nTrain: {val_base_in:.2f} mm $\\rightarrow$ Holdout: {val_base_out:.1f} mm",
            xy=(val_base_in, val_base_out),
            xytext=(1.05, 62),
            arrowprops=dict(facecolor=C_BLUE, shrink=0.08, width=1.2, headwidth=5),
            bbox=dict(boxstyle="round,pad=0.35", fc="#e7f5ff", ec=C_BLUE, alpha=0.9),
            fontsize=9.5)

ax.set_xlabel("In-Sample RMSE [mm] (Train: primi 80% campioni)")
ax.set_ylabel("Held-Out RMSE [mm] (Test: ultimi 20% estrapolazione)")
ax.set_title("Validazione Held-Out Temporale: In-Sample vs Held-Out RMSE (Trajectory C)", fontsize=12.5, pad=10)
ax.set_xlim(0.08, 2.2)
ax.set_ylim(0, 115)
ax.grid(True)
ax.legend(loc="upper left", framealpha=0.95, fontsize=10)

plt.tight_layout()
out_fig4 = os.path.join(SLIDES_DIR, "fig4_overfitting_scatter_annotated_slidestyle.png")
plt.savefig(out_fig4, dpi=300)
plt.close()
print(f"  -> Salvato: {out_fig4}\n")


# ==============================================================================
# 4. FIG 8: Lambda Sweep with Fixed Goal
# ==============================================================================
print(">>> 4. Generating fig8_lambda_sweep_fixed_goal_slidestyle.png...")
csv_fig8 = os.path.join(SLIDES_DIR, "data_fig8_lambda_sweep_fixed_goal.csv")
df_f8 = pd.read_csv(csv_fig8)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.5, 5.2), dpi=300)

demo_names = ["Trajectory C", "Trajectory A", "Reach Task"]
demo_colors = {"Trajectory C": C_BLUE, "Trajectory A": C_ORANGE, "Reach Task": C_GREEN}
demo_markers = {"Trajectory C": "o", "Trajectory A": "s", "Reach Task": "^"}

for d in demo_names:
    df_d = df_f8[df_f8["demo"] == d]
    ax1.plot(df_d["ridge_lambda"].values, df_d["rmse_overall_mm"].values,
             marker=demo_markers[d], color=demo_colors[d], label=d)
    ax2.plot(df_d["ridge_lambda"].values, df_d["final_position_error_mm"].values,
             marker=demo_markers[d], color=demo_colors[d], label=d)

ax1.set_xscale("log")
ax1.set_xlabel("Parametro Ridge $\\lambda$ [scala log]")
ax1.set_ylabel("RMSE Complessivo [mm]")
ax1.set_title("Fedeltà Traiettoria (RMSE vs $\\lambda$)", fontsize=11.5, pad=8)
ax1.grid(True)
ax1.axvline(1e-9, color=C_LIGHTGRAY, linestyle=":", linewidth=1.5, label="Valore candidato ($\\lambda=10^{-9}$)")
ax1.legend(loc="upper left", framealpha=0.9, fontsize=9.5)

ax2.set_xscale("log")
ax2.set_xlabel("Parametro Ridge $\\lambda$ [scala log]")
ax2.set_ylabel("Errore Finale di Posizione [mm]")
ax2.set_title("Accuratezza al Target (Errore Finale vs $\\lambda$)", fontsize=11.5, pad=8)
ax2.grid(True)
ax2.axvline(1e-9, color=C_LIGHTGRAY, linestyle=":", linewidth=1.5, label="Valore candidato ($\\lambda=10^{-9}$)")
ax2.legend(loc="upper left", framealpha=0.9, fontsize=9.5)

fig.suptitle("ProDMP con Vincolo Goal Esplicito (--fix-goal, $n_{\\mathrm{basis}}=80$, $w=0.05\\,\\mathrm{s}$)", fontsize=13, y=0.98)
plt.tight_layout(rect=[0, 0, 1, 0.94])
out_fig8 = os.path.join(SLIDES_DIR, "fig8_lambda_sweep_fixed_goal_slidestyle.png")
plt.savefig(out_fig8, dpi=300)
plt.close()
print(f"  -> Salvato: {out_fig8}\n")


# ==============================================================================
# 5. FIG 11: Perturbation Stability Colored vs White Noise
# ==============================================================================
print(">>> 5. Generating fig11_perturbation_stability_colored_noise_slidestyle.png...")
csv_colored = os.path.join(SLIDES_DIR, "data_fig11_perturbation_stability_colored.csv")
csv_white = os.path.join(SLIDES_DIR, "data_fig10_perturbation_stability.csv")
df_col = pd.read_csv(csv_colored)
df_w = pd.read_csv(csv_white)

fig, ax = plt.subplots(figsize=(11.0, 5.5), dpi=300)

for d in demo_names:
    df_c_sub = df_col[df_col["demo"] == d]
    sigma_val = df_c_sub["sigma_rumore_colorato_mm"].iloc[0] * 1000.0
    ax.plot(df_c_sub["ridge_lambda"].values, df_c_sub["variabilita_rollout_rms_mm"].values,
            marker=demo_markers[d], color=demo_colors[d], linestyle="-",
            label=f"{d} — Rumore Colorato ($f_c=0.5\\,\\mathrm{{Hz}}$, $\\sigma={sigma_val:.1f}\\,\\mu\\mathrm{{m}}$)")

for d in demo_names:
    df_w_sub = df_w[df_w["demo"] == d]
    ax.plot(df_w_sub["ridge_lambda"].values, df_w_sub["variabilita_rollout_rms_mm"].values,
            marker=demo_markers[d], color=demo_colors[d], linestyle="--", alpha=0.5, fillstyle="none",
            label=f"{d} — Rumore Bianco ($1\\,\\mathrm{{kHz}}$)")

ax.set_xscale("log")
ax.set_xlabel("Parametro Ridge $\\lambda$ [scala log]")
ax.set_ylabel("Variabilità Rollout Tra Realizzazioni (RMS) [mm]")
ax.set_title("Test di Stabilità per Perturbazione: Rumore Correlato vs Rumore Bianco\n($n_{\\mathrm{basis}}=80$, $w=0.05\\,\\mathrm{s}$, --fix-goal, 8 realizzazioni i.i.d.)", fontsize=12.5, pad=10)
ax.set_ylim(0.0, 0.035)
ax.grid(True)

ax.text(0.03, 0.18, 
        "• Rumore colorato: variabilità $\\sim 2.7-3.4\\times$ maggiore del bianco (cade nella banda delle RBF)\n"
        "• Risposta al ridge $\\lambda$ perfettamente piatta: $\\mathrm{Var}(10^{-12})/\\mathrm{Var}(10^{-7}) \\approx 1.004\\times$\n"
        "• Variabilità rollout ($21-25\\,\\mu\\mathrm{m}$) $\\ll \\Delta\\mathrm{RMSE}$ in-sample ($110-180\\,\\mu\\mathrm{m}$)\n"
        "• Conferma indipendente: nessun overfitting a deviazioni lente della singola demo",
        transform=ax.transAxes,
        bbox=dict(boxstyle="round,pad=0.4", fc="#f8f9fa", ec="#cccccc", alpha=0.95),
        fontsize=9.5)

ax.legend(loc="upper right", framealpha=0.95, fontsize=9.5)
plt.tight_layout()
out_fig11 = os.path.join(SLIDES_DIR, "fig11_perturbation_stability_colored_noise_slidestyle.png")
plt.savefig(out_fig11, dpi=300)
plt.close()
print(f"  -> Salvato: {out_fig11}\n")


# ==============================================================================
# 6. SUMMARY TABLE: table_final_summary_slidestyle.png
# ==============================================================================
print(">>> 6. Generating table_final_summary_slidestyle.png...")

table_data = [
    ["DMP Classico (LWR)", "200", "0.20 s", "1.0e-06", "0.297", "0.000*", "42.6", "N/A (fit locale)"],
    ["ProDMP Baseline (Post-Fix)", "200", "0.20 s", "1.0e-06", "0.713", "3.099", "4,821", "1.27 × 10¹²"],
    ["ProDMP Candidato (Holdout Opt)", "80", "0.05 s", "1.0e-09", "0.405", "1.189", "1,481", "2.40 × 10¹⁰"],
    ["ProDMP + Fix-Goal", "80", "0.05 s", "1.0e-09", "0.404", "0.893", "1,475", "5.05 × 10⁸"],
    ["ProDMP + Fix-Goal + λ Ottimizzato", "80", "0.05 s", "1.0e-12", "0.382", "0.063", "2,804", "5.05 × 10⁸"]
]

col_labels = [
    "Modello / Configurazione",
    "n_basis",
    "Filtro",
    "ridge_λ",
    "RMSE [mm]",
    "Err. Finale [mm]",
    "max |w|",
    "cond(H)"
]

fig, ax = plt.subplots(figsize=(11.5, 3.8), dpi=300)
ax.axis('off')

col_widths = [0.28, 0.08, 0.08, 0.09, 0.11, 0.13, 0.10, 0.13]

table = ax.table(
    cellText=table_data,
    colLabels=col_labels,
    colWidths=col_widths,
    loc='center',
    cellLoc='center'
)

table.auto_set_font_size(False)
table.set_fontsize(10.0)
table.scale(1.0, 1.9)

# Style header and rows
for col_idx in range(len(col_labels)):
    cell = table[(0, col_idx)]
    cell.set_facecolor('#2c3e50')
    cell.set_text_props(color='white', weight='bold', fontsize=10.5)
    cell.set_edgecolor('#1a252f')
    cell.set_linewidth(1.2)

# Color alternating rows and align text
for row_idx in range(1, len(table_data) + 1):
    bg_color = '#f8f9fa' if row_idx % 2 == 1 else '#ffffff'
    # Highlight final row (winner configuration) with subtle mint green
    if row_idx == 5:
        bg_color = '#e6fcf5'
    for col_idx in range(len(col_labels)):
        cell = table[(row_idx, col_idx)]
        cell.set_facecolor(bg_color)
        cell.set_edgecolor('#dee2e6')
        cell.set_linewidth(0.8)
        # Left-align model name with indentation
        if col_idx == 0:
            cell.set_text_props(ha='left', weight='bold' if (row_idx == 1 or row_idx == 5) else 'normal')
        if row_idx == 5 and col_idx in [4, 5, 7]:
            cell.set_text_props(weight='bold', color='#087f5b')

plt.title("Confronto Configurazioni Chiave DMP vs ProDMP (Trajectory C, N=68 245 campioni)",
          fontsize=12, weight='bold', pad=12)
plt.text(0.01, 0.02, "* In DMP classico il goal finale è vincolato analiticamente a convergenza esponenziale.",
         fontsize=8.5, style='italic', color='#666666', transform=fig.transFigure)

plt.tight_layout()
out_table = os.path.join(SLIDES_DIR, "table_final_summary_slidestyle.png")
plt.savefig(out_table, dpi=300, bbox_inches='tight')
plt.close()
print(f"  -> Salvato: {out_table}\n")

print("================================================================================")
print("TUTTI I 6 FILE SLIDESTYLE SONO STATI GENERATI CON SUCCESSO!")
print("================================================================================")
