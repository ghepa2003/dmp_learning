#!/usr/bin/env python3
"""
Generates high-resolution slide figures (300 DPI) for thesis / presentation
covering ProDMP column scaling fix, DMP vs ProDMP fidelity, and holdout validation.
"""

import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Setup output directory
OUTPUT_DIR = "/home/lorenzo/thesis_ws/slides_material"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Set global matplotlib style for high readability on projectors
plt.rcParams.update({
    'font.sans-serif': 'DejaVu Sans',
    'font.family': 'sans-serif',
    'font.size': 12,
    'axes.labelsize': 13,
    'axes.titlesize': 14,
    'xtick.labelsize': 11,
    'ytick.labelsize': 11,
    'legend.fontsize': 11,
    'figure.titlesize': 15,
    'lines.linewidth': 2.0,
    'lines.markersize': 7,
    'grid.alpha': 0.5,
    'grid.linestyle': '--'
})

print("================================================================================")
print("GENERATING SLIDE MATERIAL FIGURES (300 DPI)")
print("================================================================================\n")

# ==============================================================================
# FIGURA 1: Column Norms of H (Before vs After Preconditioning)
# ==============================================================================
print(">>> FIGURA 1: Column-norm della matrice di design H")
csv_fig1 = os.path.join(OUTPUT_DIR, "data_fig1_column_norms.csv")
df_f1 = pd.read_csv(csv_fig1)

min_pre = float(df_f1["norm_before_maxabs"].min())
max_pre = float(df_f1["norm_before_maxabs"].max())
ratio_pre = max_pre / min_pre

min_post = float(df_f1["norm_after_maxabs"].min())
max_post = float(df_f1["norm_after_maxabs"].max())
ratio_post = max_post / min_post

print(f"  [Prima] Min col norm: {min_pre:.4e} | Max col norm: {max_pre:.4f} | Ratio: {ratio_pre:.2e}")
print(f"  [Dopo]  Min col norm: {min_post:.4e} | Max col norm: {max_post:.4f} | Ratio: {ratio_post:.2e}")
print(f"  [Info]  Colonne scalate ad 1.0: {int(df_f1['scaled_applied'].sum())} su {len(df_f1)}")

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5.5), dpi=300, sharey=True)

c_rbf = "#1f77b4"
c_goal = "#d62728"
c_scaled = "#2ca02c"
c_floor = "#7f7f7f"

# Subplot 1: Prima
rbf_mask = (df_f1["is_goal_col"] == 0).values
goal_mask = (df_f1["is_goal_col"] == 1).values

ax1.scatter(df_f1.loc[rbf_mask, "col_idx"].values, df_f1.loc[rbf_mask, "norm_before_maxabs"].values,
            color=c_rbf, s=35, alpha=0.85, label="Colonne RBF forma (w_i)")
ax1.scatter(df_f1.loc[goal_mask, "col_idx"].values, df_f1.loc[goal_mask, "norm_before_maxabs"].values,
            color=c_goal, s=90, marker="^", label="Colonna Step Goal (g)")

ax1.set_yscale("log")
ax1.set_ylim(1e-8, 5.0)
ax1.set_xlabel("Indice Colonna i di H (0 .. 200)")
ax1.set_ylabel("Norma Picco Colonna ||H_{:, i}||_inf [scala log]")
ax1.set_title("PRIMA: Matrice H Naturale\n(Spread patologico di ~7 ordini di grandezza)")
ax1.grid(True)
ax1.legend(loc="upper left")

ax1.annotate(f"Spread max/min: {ratio_pre:.1e}\nGoal domina di 10^4 x",
             xy=(200, max_pre), xytext=(80, 1e-2),
             arrowprops=dict(facecolor='black', shrink=0.08, width=1, headwidth=6),
             bbox=dict(boxstyle="round,pad=0.4", fc="yellow", alpha=0.25, ec="orange"),
             fontsize=10.5, weight="bold")

# Subplot 2: Dopo
scaled_mask = (df_f1["scaled_applied"] == 1).values
unscaled_mask = ((df_f1["scaled_applied"] == 0) & (df_f1["is_goal_col"] == 0)).values

ax2.scatter(df_f1.loc[scaled_mask, "col_idx"].values, df_f1.loc[scaled_mask, "norm_after_maxabs"].values,
            color=c_scaled, s=35, alpha=0.9, label="Colonne Scalate a 1.0 (> 5e-6)")
ax2.scatter(df_f1.loc[unscaled_mask, "col_idx"].values, df_f1.loc[unscaled_mask, "norm_after_maxabs"].values,
            color=c_floor, s=30, alpha=0.6, marker="x", label="Sotto Floor Relativo (non scalate)")
ax2.scatter(df_f1.loc[goal_mask, "col_idx"].values, df_f1.loc[goal_mask, "norm_after_maxabs"].values,
            color=c_goal, s=90, marker="^", label="Colonna Goal Scalata a 1.0")

floor_val = 5e-6 * max_pre
ax2.axhline(floor_val, color="darkorange", linestyle=":", linewidth=2, label="Floor relativo (5e-6)")

ax2.set_yscale("log")
ax2.set_xlabel("Indice Colonna i di H (0 .. 200)")
ax2.set_title("DOPO: Column-Scale Preconditioning\n(Floor relativo k_floor = 5e-6)")
ax2.grid(True)
ax2.legend(loc="lower right")

ax2.annotate("148 colonne equalizzate a 1.0\nRidge agisce uniformemente",
             xy=(100, 1.0), xytext=(30, 2e-2),
             arrowprops=dict(facecolor='black', shrink=0.08, width=1, headwidth=6),
             bbox=dict(boxstyle="round,pad=0.4", fc="lightgreen", alpha=0.35, ec="green"),
             fontsize=10.5, weight="bold")

plt.tight_layout()
fig1_path = os.path.join(OUTPUT_DIR, "fig1_column_scaling_before_after.png")
plt.savefig(fig1_path, dpi=300)
plt.close()
print(f"  -> Salvato: {fig1_path}\n")


# ==============================================================================
# FIGURA 2: Ridge Lambda Sweep (Before vs After Fix)
# ==============================================================================
print(">>> FIGURA 2: Sweep di ridge_lambda prima e dopo il fix")
csv_fig2 = os.path.join(OUTPUT_DIR, "data_fig2_ridge_sweep_before_after.csv")
df_f2 = pd.read_csv(csv_fig2)

min_idx_pre = df_f2["rmse_prefix_mm"].idxmin()
min_lam_pre = float(df_f2.loc[min_idx_pre, "ridge_lambda"])
min_rmse_pre = float(df_f2.loc[min_idx_pre, "rmse_prefix_mm"])

min_idx_post = df_f2["rmse_postfix_mm"].idxmin()
min_lam_post = float(df_f2.loc[min_idx_post, "ridge_lambda"])
min_rmse_post = float(df_f2.loc[min_idx_post, "rmse_postfix_mm"])

val_prefix_1e6 = float(df_f2.loc[df_f2['ridge_lambda']==1e-6, 'rmse_prefix_mm'].values[0])
val_postfix_1e6 = float(df_f2.loc[df_f2['ridge_lambda']==1e-6, 'rmse_postfix_mm'].values[0])

print(f"  [Pre-fix]  Min RMSE: {min_rmse_pre:.4f} mm a lambda={min_lam_pre:.1e} | a lambda=1e-6: {val_prefix_1e6:.4f} mm")
print(f"  [Post-fix] Min RMSE: {min_rmse_post:.4f} mm a lambda={min_lam_post:.1e} | a lambda=1e-6: {val_postfix_1e6:.4f} mm")

plt.figure(figsize=(9.5, 6.0), dpi=300)
plt.plot(df_f2["ridge_lambda"].values, df_f2["rmse_prefix_mm"].values, 'r--o', label="Pre-Fix (Unscaled H, patologico)", linewidth=2.2, markersize=8)
plt.plot(df_f2["ridge_lambda"].values, df_f2["rmse_postfix_mm"].values, 'b-s', label="Post-Fix (Column-Scaled H, corretto)", linewidth=2.5, markersize=8)

plt.xscale("log")
plt.xlabel("Parametro di Regolarizzazione Ridge lambda [scala log]", fontsize=13, weight="bold")
plt.ylabel("Posizione RMSE complessivo [mm]", fontsize=13, weight="bold")
plt.title("Impatto del Column Preconditioning sullo Sweep di lambda\n(Trajectory C, n_basis=200, finestra filtro 0.20 s)", fontsize=13, weight="bold")
plt.grid(True)

# Highlight baseline value 1e-6
plt.axvline(1e-6, color="gray", linestyle=":", linewidth=1.5, alpha=0.8)
plt.text(1.2e-6, 40, "Baseline adottata\n(lambda = 1e-6)", color="dimgray", fontsize=10.5, weight="bold")

# Annotations
plt.annotate(f"Pre-Fix a 1e-6:\nRMSE = {val_prefix_1e6:.3f} mm\n(errore 4.2x maggiore)",
             xy=(1e-6, val_prefix_1e6),
             xytext=(3e-5, 12),
             arrowprops=dict(facecolor='crimson', shrink=0.08, width=1.2, headwidth=6),
             bbox=dict(boxstyle="round,pad=0.3", fc="#ffe6e6", ec="red"),
             fontsize=10, weight="bold")

plt.annotate(f"Post-Fix a 1e-6:\nRMSE = {val_postfix_1e6:.3f} mm\n(miglioramento netto)",
             xy=(1e-6, val_postfix_1e6),
             xytext=(3e-9, 15),
             arrowprops=dict(facecolor='blue', shrink=0.08, width=1.2, headwidth=6),
             bbox=dict(boxstyle="round,pad=0.3", fc="#e6f0ff", ec="blue"),
             fontsize=10, weight="bold")

plt.legend(loc="upper left", fontsize=11)
plt.tight_layout()
fig2_path = os.path.join(OUTPUT_DIR, "fig2_ridge_lambda_sweep_before_after.png")
plt.savefig(fig2_path, dpi=300)
plt.close()
print(f"  -> Salvato: {fig2_path}\n")


# ==============================================================================
# FIGURA 3: Trajectory Overlay (Demo vs DMP vs ProDMP)
# ==============================================================================
print(">>> FIGURA 3: Overlay traiettoria Demo vs DMP classico vs ProDMP")
demo_csv = "/home/lorenzo/thesis_ws/demo_raw_trajC.csv"
replay_dmp_csv = os.path.join(OUTPUT_DIR, "replay_dmp_classic_trajC.csv")
replay_prodmp_csv = os.path.join(OUTPUT_DIR, "replay_prodmp_baseline_trajC.csv")

df_demo = pd.read_csv(demo_csv)
df_dmp = pd.read_csv(replay_dmp_csv)
df_prodmp = pd.read_csv(replay_prodmp_csv)

t_demo = (df_demo["t"] - df_demo["t"].iloc[0]).values
t_dmp = df_dmp["t"].values
t_prodmp = df_prodmp["t"].values

diff_dmp = (df_dmp[["x", "y", "z"]].values - df_demo[["x", "y", "z"]].values) * 1000.0
rmse_dmp = np.sqrt(np.sum(diff_dmp**2) / len(diff_dmp))

diff_prodmp = (df_prodmp[["x", "y", "z"]].values - df_demo[["x", "y", "z"]].values) * 1000.0
rmse_prodmp = np.sqrt(np.sum(diff_prodmp**2) / len(diff_prodmp))

print(f"  [Overlay] DMP classico RMSE:    {rmse_dmp:.4f} mm")
print(f"  [Overlay] ProDMP post-fix RMSE: {rmse_prodmp:.4f} mm")

fig, axes = plt.subplots(3, 1, figsize=(11, 8.5), dpi=300, sharex=True)
coords = ['x', 'y', 'z']
labels_axis = ['Posizione X [m]', 'Posizione Y [m]', 'Posizione Z [m]']

c_gt = "black"
c_classic = "#1f77b4"
c_pro = "#2ca02c"

for ax, c, lbl in zip(axes, coords, labels_axis):
    ax.plot(t_demo, df_demo[c].values, color=c_gt, linestyle='-', linewidth=2.2, label="Ground Truth (demo_raw_trajC)")
    ax.plot(t_dmp, df_dmp[c].values, color=c_classic, linestyle='--', linewidth=2.0, label=f"DMP Classico (RMSE = {rmse_dmp:.3f} mm)")
    ax.plot(t_prodmp, df_prodmp[c].values, color=c_pro, linestyle=':', linewidth=2.2, label=f"ProDMP Post-Fix (RMSE = {rmse_prodmp:.3f} mm)")
    ax.set_ylabel(lbl, fontsize=12, weight="bold")
    ax.grid(True)

axes[2].set_xlabel("Tempo [s]", fontsize=13, weight="bold")
axes[0].set_title("Confronto Fedeltà di Replay su Trajectory C (Durata: 68.3 s)\nDMP Classico vs ProDMP (Baseline n=200, lambda=1e-6, w=0.20 s)", fontsize=13, weight="bold")

handles, labels = axes[0].get_legend_handles_labels()
fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, 0.94), ncol=3, frameon=True, fontsize=11)

plt.tight_layout(rect=[0, 0, 1, 0.90])
fig3_path = os.path.join(OUTPUT_DIR, "fig3_trajectory_overlay_dmp_vs_prodmp.png")
plt.savefig(fig3_path, dpi=300)
plt.close()
print(f"  -> Salvato: {fig3_path}\n")


# ==============================================================================
# FIGURA 4: Scatter Overfitting Annotated (In-Sample vs Held-Out)
# ==============================================================================
print(">>> FIGURA 4: Scatter Overfitting In-Sample vs Held-Out (Trajectory C)")
csv_holdout_c = "/home/lorenzo/thesis_ws/tools/dmp_offline_test/plots/07_holdout_prodmp/prodmp_holdout_trajC.csv"
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

print(f"  [In-sample Winner]  n=300, lam=1e-12 -> in-sample: {val_in_in:.4f} mm | holdout: {val_in_out:.4f} mm | gap: {gap_in:.1f}x")
print(f"  [Holdout Winner]    n=80,  lam=1e-9  -> in-sample: {val_out_in:.4f} mm | holdout: {val_out_out:.4f} mm | gap: {gap_out:.1f}x")
print(f"  [Baseline Adottato] n=200, lam=1e-6  -> in-sample: {val_base_in:.4f} mm | holdout: {val_base_out:.4f} mm | gap: {gap_base:.1f}x")

plt.figure(figsize=(10.5, 7.2), dpi=300)

sc = plt.scatter(
    df_grid["rmse_in_sample_mm"].values,
    df_grid["rmse_holdout_mm"].values,
    c=df_grid["n_basis"].values,
    cmap="plasma",
    s=65,
    alpha=0.75,
    edgecolors="k",
    linewidths=0.6,
    label="Configurazioni Griglia (n in [30..300], lambda in [10^-12..10^-6])"
)
cbar = plt.colorbar(sc, pad=0.02)
cbar.set_label("Numero Funzioni di Base n_basis", fontsize=12, weight="bold")

x_vals = np.linspace(0.05, 2.5, 100)
plt.plot(x_vals, x_vals, 'k--', linewidth=1.5, alpha=0.6, label="y = x (Zero Gap di Generalizzazione)")
plt.plot(x_vals, 3.0 * x_vals, 'r:', linewidth=1.8, alpha=0.7, label="y = 3x (Soglia Segnalazione Overfitting)")

# 1. In-sample winner
plt.scatter(val_in_in, val_in_out, s=220, marker="*", color="#d62728", edgecolors="black", linewidth=1.2, zorder=5)
plt.annotate(f"Vincitore In-Sample (OVERFITTING)\nn=300, lambda=1e-12\nIn-sample: {val_in_in:.2f} mm -> Held-out: {val_in_out:.1f} mm (Gap {gap_in:.0f}x)",
             xy=(val_in_in, val_in_out),
             xytext=(0.40, 42),
             arrowprops=dict(facecolor='#d62728', shrink=0.08, width=1.5, headwidth=7),
             bbox=dict(boxstyle="round,pad=0.4", fc="#ffe6e6", ec="red", alpha=0.9),
             fontsize=10.5, weight="bold")

# 2. Holdout winner
plt.scatter(val_out_in, val_out_out, s=180, marker="o", color="#2ca02c", edgecolors="black", linewidth=1.5, zorder=5)
plt.annotate(f"Miglior Generalizzazione Held-Out\nn=80, lambda=1e-9\nIn-sample: {val_out_in:.2f} mm -> Held-out: {val_out_out:.2f} mm (Gap {gap_out:.1f}x)",
             xy=(val_out_in, val_out_out),
             xytext=(0.60, 8),
             arrowprops=dict(facecolor='#2ca02c', shrink=0.08, width=1.5, headwidth=7),
             bbox=dict(boxstyle="round,pad=0.4", fc="#e6ffe6", ec="green", alpha=0.9),
             fontsize=10.5, weight="bold")

# 3. Baseline post-fix
plt.scatter(val_base_in, val_base_out, s=160, marker="D", color="#1f77b4", edgecolors="black", linewidth=1.2, zorder=5)
plt.annotate(f"Baseline Adottata in Produzione\nn=200, lambda=1e-6, w=0.20 s\nIn-sample: {val_base_in:.2f} mm -> Held-out: {val_base_out:.1f} mm",
             xy=(val_base_in, val_base_out),
             xytext=(1.05, 65),
             arrowprops=dict(facecolor='#1f77b4', shrink=0.08, width=1.5, headwidth=7),
             bbox=dict(boxstyle="round,pad=0.4", fc="#e6f0ff", ec="blue", alpha=0.9),
             fontsize=10.5, weight="bold")

plt.xlabel("In-Sample RMSE [mm] (Train: primi 80% campioni)", fontsize=13, weight="bold")
plt.ylabel("Held-Out RMSE [mm] (Test: ultimi 20% estrapolazione)", fontsize=13, weight="bold")
plt.title("Validazione Held-Out Temporale: Scoperta e Quantificazione dell'Overfitting\n(Trajectory C, N=68 245 campioni)", fontsize=13, weight="bold")
plt.xlim(0.08, 2.2)
plt.ylim(0, 115)
plt.grid(True)
plt.legend(loc="upper left", fontsize=10.5)

plt.tight_layout()
fig4_path = os.path.join(OUTPUT_DIR, "fig4_overfitting_scatter_annotated.png")
plt.savefig(fig4_path, dpi=300)
plt.close()
print(f"  -> Salvato: {fig4_path}\n")


# ==============================================================================
# FIGURA 5: Gap Ratio Comparison Bar Chart
# ==============================================================================
print(">>> FIGURA 5: Bar chart del Gap Ratio per le 3 configurazioni chiave su 3 demo")

demos = ["Trajectory C\n(demo_raw_trajC)", "Reach Task Baseline\n(reach_task_baseline)", "Trajectory A\n(demo_raw_trajA)"]

gap_in_winner = [193.6, 332.9, 181.7]
gap_baseline  = [24.1,  36.0,  21.7]
gap_out_winner= [14.8,  87.8,  21.6]

print("  [Gap Ratio Table]")
for d, g_in, g_base, g_out in zip(demos, gap_in_winner, gap_baseline, gap_out_winner):
    d_clean = d.replace('\n', ' ')
    print(f"    {d_clean:40s} | In-sample opt: {g_in:6.1f}x | Baseline: {g_base:5.1f}x | Holdout opt: {g_out:5.1f}x")

x = np.arange(len(demos))
width = 0.26

fig, ax = plt.subplots(figsize=(11, 6.2), dpi=300)

rects1 = ax.bar(x - width, gap_in_winner, width, label='Vincitore In-Sample (n=300, lambda=1e-12, w=0.01)',
                color='#d62728', edgecolor='black', alpha=0.9)
rects2 = ax.bar(x, gap_baseline, width, label='Baseline Adottata (n=200, lambda=1e-6, w=0.20)',
                color='#1f77b4', edgecolor='black', alpha=0.9)
rects3 = ax.bar(x + width, gap_out_winner, width, label='Vincitore Holdout (n=80, lambda=1e-9, w=0.01)',
                color='#2ca02c', edgecolor='black', alpha=0.9)

ax.set_yscale('log')
ax.set_ylabel('Generalization Gap Ratio (RMSE Held-Out / RMSE In-Sample) [log]', fontsize=12, weight="bold")
ax.set_title('Confronto del Gap di Generalizzazione tra Configurazioni Chiave\n(Valutazione su 3 Traiettorie Indipendenti)', fontsize=13, weight="bold")
ax.set_xticks(x)
ax.set_xticklabels(demos, fontsize=11, weight="bold")
ax.set_ylim(5, 700)
ax.grid(True, which="both", axis="y", alpha=0.4)
ax.legend(loc='upper right', fontsize=11)

def autolabel(rects):
    for rect in rects:
        height = rect.get_height()
        ax.annotate(f'{height:.1f}x',
                    xy=(rect.get_x() + rect.get_width() / 2, height),
                    xytext=(0, 4),
                    textcoords="offset points",
                    ha='center', va='bottom', fontsize=11, weight='bold')

autolabel(rects1)
autolabel(rects2)
autolabel(rects3)

plt.tight_layout()
fig5_path = os.path.join(OUTPUT_DIR, "fig5_gap_ratio_comparison.png")
plt.savefig(fig5_path, dpi=300)
plt.close()
print(f"  -> Salvato: {fig5_path}\n")


# ==============================================================================
# TABELLA RIASSUNTIVA (summary_table.md)
# ==============================================================================
print(">>> TABELLA RIASSUNTIVA (summary_table.md)")
summary_table_content = """# Tabella Riassuntiva: Confronto Metriche Chiave DMP vs ProDMP

| Modello / Configurazione | $n_{\\mathrm{basis}}$ | $\\text{ridge\\_}\\lambda$ | Finestra Filtro | RMSE In-Sample [mm] | RMSE Held-Out (20%) [mm] | $\\max |w|$ | Note e Valutazione |
|:---|:---:|:---:|:---:|:---:|:---:|:---:|:---|
| **DMP Classico (LWR/Ridge)** | 200 | $1.0\\times 10^{-6}$ | $0.20\\,\\mathrm{s}$ | **0.2970** | *N/A* | 42.6 | Baseline di riferimento, privo di forcing analitico |
| **ProDMP Baseline Post-Fix** | 200 | $1.0\\times 10^{-6}$ | $0.20\\,\\mathrm{s}$ | **0.7130** | **46.725** | 23,497 | Adottato in produzione, stabile, precondizionato ($k_{\\mathrm{floor}}=5\\cdot 10^{-6}$) |
| **ProDMP "Ottimo In-Sample"** | 300 | $1.0\\times 10^{-12}$ | $0.01\\,\\mathrm{s}$ | **0.1487** | **28.787** | 5,546 | **Overfitting severo** (Gap $194\\times$ su TrajC, $333\\times$ su Reach) |
| **ProDMP "Ottimo Held-Out"** | 80 | $1.0\\times 10^{-9}$ | $0.01\\,\\mathrm{s}$ | **0.3724** | **5.520** | 3,210 | Minimo errore di estrapolazione su TrajC e TrajA |

> **Dati di Riferimento**: Trajectory C (`demo_raw_trajC.csv`, $N=68\\,245$ campioni, durata $68.325\\,\\mathrm{s}$).
"""

summary_md_path = os.path.join(OUTPUT_DIR, "summary_table.md")
with open(summary_md_path, "w") as f:
    f.write(summary_table_content)
print(f"  -> Salvato: {summary_md_path}\n")

print("================================================================================")
print("TUTTE LE FIGURE E LA TABELLA SONO STATE GENERATE CON SUCCESSO IN:")
print(OUTPUT_DIR)
print("================================================================================")
