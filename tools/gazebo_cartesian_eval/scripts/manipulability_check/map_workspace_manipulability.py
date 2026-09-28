#!/usr/bin/env python3
"""
Mappatura dello Spazio di Lavoro Locale e Sensibilità all'Orientamento
per Franka Emika Panda (URDF fer_flat_effort.urdf, frame fer_hand_tcp).

Livello 1:
- Mappatura 3D attorno alla posa baseline della demo: g_baseline = (0.3907, -0.1159, 0.2920) m.
- Raggio ±10 cm su x, y, z con passo 1 cm (griglia 21x21x21 = 9261 punti).
- Orientamento EE fisso a quello della demo baseline.
- Calcolo w_trans = sqrt(det(J_p J_p^T)) per ciascun punto.
- Sezioni 2D ortogonali (x-y, x-z, y-z) passanti per Goal 2 e per la Baseline.
- Sovrapposizione dei 5 goal testati.

Livello 2:
- Posizione fissa a Goal 2 (dx=-5cm, dy=+4cm, dz=-3cm).
- Variazione dell'orientamento attorno agli assi locali EE (Roll X, Pitch Y, Yaw Z) da -20° a +20°.
- Calcolo di w_trans vs delta angolo per ciascun asse.
"""

import os
import math
import numpy as np
import pinocchio as pin
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

eval_root = "/root/thesis_ws/tools/gazebo_cartesian_eval"
plots_dir = os.path.join(eval_root, "plots/goal_generalization")
os.makedirs(plots_dir, exist_ok=True)
urdf_path = "/root/thesis_ws/fer_flat_effort.urdf"

model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()
frame_id = model.getFrameId("fer_hand_tcp")


def build_q_index(model, joint_names):
    """Resolves each joint name to its Pinocchio q-vector index (model.idx_qs[joint_id]),
    mirroring RobotModel::update's q_index resolution in robot_model.cpp. Needed because
    model.nq (9 with hand:=true: 7 arm joints + fer_finger_joint1/2) no longer matches the
    7-element q vectors this script solves for."""
    return [model.idx_qs[model.getJointId(name)] for name in joint_names]


def build_v_index(model, joint_names):
    """Resolves each joint name to its Pinocchio v-vector (tangent/velocity) index
    (model.idx_vs[joint_id]), mirroring RobotModel::update's v_index resolution in
    robot_model.cpp. Needed to slice a 6 x model.nv Jacobian back down to the 6x7 arm-only
    shape solve_ik()'s Newton-Raphson update (dq = J^T (JJ^T)^-1 err, q = q - dq) expects -
    that step is NOT invariant to the extra (structurally zero) finger-joint columns
    model.nq/nv now carry under hand:=true."""
    return [model.idx_vs[model.getJointId(name)] for name in joint_names]


def pad_q(q):
    """Zero-pads a 7-element arm q into Pinocchio's full model.nq configuration vector, at
    the name-resolved indices in Q_INDEX. The extra DOFs (gripper finger prismatic joints
    under hand:=true) stay at 0.0: fer_hand_tcp is reached via a fixed joint upstream of the
    fingers, so their value does not affect its position."""
    q_full = np.zeros(model.nq)
    for i, idx in enumerate(Q_INDEX):
        q_full[idx] = q[i]
    return q_full


JOINT_NAMES = [f"fer_joint{i}" for i in range(1, 8)]
Q_INDEX = build_q_index(model, JOINT_NAMES)
V_INDEX = build_v_index(model, JOINT_NAMES)
# Position limits restricted to the 7 arm joints, in JOINT_NAMES order (Q_INDEX), so shapes
# stay compatible with the 7-dim q solve_ik() works with - model.lowerPositionLimit/
# upperPositionLimit are now 9-dim (with hand:=true) and NOT usable as-is against a 7-dim q.
q_min = model.lowerPositionLimit[Q_INDEX] + 1e-3
q_max = model.upperPositionLimit[Q_INDEX] - 1e-3

# Baseline configuration (seed naturale)
q_base = np.array([-0.0125, -0.4610, -0.2486, -2.6409, -0.2534, 2.2794, 1.0082])
pin.forwardKinematics(model, data, pad_q(q_base))
pin.updateFramePlacements(model, data)
T_base = data.oMf[frame_id].copy()
p_base = T_base.translation.copy()
R_des = T_base.rotation.copy()

# 5 Goals relative offsets [m]
goals = {
    "Baseline": np.array([0.00, 0.00, 0.00]),
    "Goal 1": np.array([+0.04, +0.03, +0.02]),
    "Goal 2": np.array([-0.05, +0.04, -0.03]),
    "Goal 3": np.array([+0.03, -0.05, +0.04]),
    "Goal 4": np.array([-0.04, -0.03, +0.05]),
    "Goal 5": np.array([+0.05, -0.04, -0.03]),
}

def solve_ik(p_target, R_target, q_init):
    q = q_init.copy()
    T_target = pin.SE3(R_target, p_target)
    for _ in range(30):
        pin.forwardKinematics(model, data, pad_q(q))
        pin.updateFramePlacements(model, data)
        curr = data.oMf[frame_id]
        err = pin.log6(T_target.actInv(curr)).vector
        if np.linalg.norm(err[:3]) < 1e-4 and np.linalg.norm(err[3:]) < 1e-3:
            if np.all(q >= q_min) and np.all(q <= q_max):
                pin.computeJointJacobians(model, data, pad_q(q))
                pin.framesForwardKinematics(model, data, pad_q(q))
                J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:, V_INDEX]
                Jp = J[:3, :]
                wt = math.sqrt(max(0.0, np.linalg.det(Jp @ Jp.T)))
                return True, wt, q
            else:
                return False, np.nan, q
        pin.computeJointJacobians(model, data, pad_q(q))
        pin.framesForwardKinematics(model, data, pad_q(q))
        # getFrameJacobian returns 6 x model.nv (9 with hand:=true); slice back to the 7
        # arm-DOF columns via V_INDEX (mirroring RobotModel::update's Jacobian column
        # extraction) - required for the Newton-Raphson step below, not just a shape fix:
        # dq must stay 7-dim to match q.
        J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL)[:, V_INDEX]
        JJt = J @ J.T + 1e-4 * np.eye(6)
        dq = J.T @ np.linalg.solve(JJt, err)
        q = q - dq
    return False, np.nan, q

# =========================================================================
# LIVELLO 1: Griglia 3D (21x21x21)
# =========================================================================
print("Calcolo griglia 3D 21x21x21 (passo 1cm, raggio +-10cm)...")
dx = np.linspace(-0.10, 0.10, 21)
dy = np.linspace(-0.10, 0.10, 21)
dz = np.linspace(-0.10, 0.10, 21)

grid_wt = np.full((21, 21, 21), np.nan)
n_conv = 0

for ix, x_off in enumerate(dx):
    for iy, y_off in enumerate(dy):
        for iz, z_off in enumerate(dz):
            p_tgt = p_base + np.array([x_off, y_off, z_off])
            ok, wt, _ = solve_ik(p_tgt, R_des, q_base)
            if ok:
                n_conv += 1
                grid_wt[ix, iy, iz] = wt

print(f"Completato: {n_conv}/9261 punti convergenti ({n_conv/9261*100:.1f}%)")
print(f"w_trans min = {np.nanmin(grid_wt):.5f}, max = {np.nanmax(grid_wt):.5f}, media = {np.nanmean(grid_wt):.5f}")

# Valori nei punti dei Goal
print("\n--- Valori w_trans estratti nei 5 Goal sulla griglia cinematica ---")
goal_grid_vals = {}
for g_name, g_off in goals.items():
    p_tgt = p_base + g_off
    ok, wt, _ = solve_ik(p_tgt, R_des, q_base)
    goal_grid_vals[g_name] = wt
    print(f"{g_name:<10}: offset = ({g_off[0]:+.2f}, {g_off[1]:+.2f}, {g_off[2]:+.2f}) m -> w_trans = {wt:.5f}")

# =========================================================================
# PLOT LIVELLO 1: Sezioni Ortogonali 2D passanti per Goal 2
# =========================================================================
# Indici Goal 2: dx=-0.05 (idx=5), dy=+0.04 (idx=14), dz=-0.03 (idx=7)
idx_g2_x = np.argmin(np.abs(dx - (-0.05)))
idx_g2_y = np.argmin(np.abs(dy - (+0.04)))
idx_g2_z = np.argmin(np.abs(dz - (-0.03)))

fig, axes = plt.subplots(1, 3, figsize=(18, 5.5), dpi=150)
fig.suptitle(r"Sezioni Ortogonali di Manipolabilità Traslazionale $w_{trans}$ passanti per Goal 2 ($\Delta x=-5, \Delta y=+4, \Delta z=-3$ cm)", fontsize=13, fontweight='bold')

cmap = 'viridis'
vmin, vmax = 0.03, 0.12

# 1. Sezione X-Y (fissato Z a Goal 2: dz = -3cm)
im0 = axes[0].imshow(grid_wt[:, :, idx_g2_z].T, origin='lower', extent=[-10, 10, -10, 10], cmap=cmap, vmin=vmin, vmax=vmax, aspect='equal')
axes[0].set_title(f"Piano $\Delta x - \Delta y$ (a $\Delta z = {dz[idx_g2_z]*100:.0f}$ cm)", fontsize=11)
axes[0].set_xlabel(r"$\Delta x$ [cm]", fontsize=10)
axes[0].set_ylabel(r"$\Delta y$ [cm]", fontsize=10)
axes[0].grid(True, linestyle=':', alpha=0.6)

# 2. Sezione X-Z (fissato Y a Goal 2: dy = +4cm)
im1 = axes[1].imshow(grid_wt[:, idx_g2_y, :].T, origin='lower', extent=[-10, 10, -10, 10], cmap=cmap, vmin=vmin, vmax=vmax, aspect='equal')
axes[1].set_title(f"Piano $\Delta x - \Delta z$ (a $\Delta y = {dy[idx_g2_y]*100:.0f}$ cm)", fontsize=11)
axes[1].set_xlabel(r"$\Delta x$ [cm]", fontsize=10)
axes[1].set_ylabel(r"$\Delta z$ [cm]", fontsize=10)
axes[1].grid(True, linestyle=':', alpha=0.6)

# 3. Sezione Y-Z (fissato X a Goal 2: dx = -5cm)
im2 = axes[2].imshow(grid_wt[idx_g2_x, :, :].T, origin='lower', extent=[-10, 10, -10, 10], cmap=cmap, vmin=vmin, vmax=vmax, aspect='equal')
axes[2].set_title(f"Piano $\Delta y - \Delta z$ (a $\Delta x = {dx[idx_g2_x]*100:.0f}$ cm)", fontsize=11)
axes[2].set_xlabel(r"$\Delta y$ [cm]", fontsize=10)
axes[2].set_ylabel(r"$\Delta z$ [cm]", fontsize=10)
axes[2].grid(True, linestyle=':', alpha=0.6)

# Overlay dei Goal su ciascun asse
goal_colors = {'Baseline': 'black', 'Goal 1': 'blue', 'Goal 2': 'red', 'Goal 3': 'green', 'Goal 4': 'purple', 'Goal 5': 'orange'}
goal_markers = {'Baseline': 'X', 'Goal 1': 'o', 'Goal 2': 's', 'Goal 3': '^', 'Goal 4': 'D', 'Goal 5': 'v'}

for g_name, off in goals.items():
    gx, gy, gz = off * 100.0
    c = goal_colors[g_name]
    m = goal_markers[g_name]
    # In ax0 (x-y)
    axes[0].scatter(gx, gy, color=c, marker=m, s=80, edgecolors='white', linewidths=1.2, label=g_name if g_name in ['Baseline', 'Goal 2'] else None)
    axes[0].annotate(g_name, (gx+0.4, gy+0.4), fontsize=8, color='white', fontweight='bold', bbox=dict(boxstyle='round,pad=0.15', facecolor='black', alpha=0.5))
    # In ax1 (x-z)
    axes[1].scatter(gx, gz, color=c, marker=m, s=80, edgecolors='white', linewidths=1.2)
    axes[1].annotate(g_name, (gx+0.4, gz+0.4), fontsize=8, color='white', fontweight='bold', bbox=dict(boxstyle='round,pad=0.15', facecolor='black', alpha=0.5))
    # In ax2 (y-z)
    axes[2].scatter(gy, gz, color=c, marker=m, s=80, edgecolors='white', linewidths=1.2)
    axes[2].annotate(g_name, (gy+0.4, gz+0.4), fontsize=8, color='white', fontweight='bold', bbox=dict(boxstyle='round,pad=0.15', facecolor='black', alpha=0.5))

fig.subplots_adjust(right=0.88)
cbar_ax = fig.add_axes([0.90, 0.15, 0.018, 0.7])
cbar = fig.colorbar(im0, cax=cbar_ax)
cbar.set_label(r"Translational Manipulability $w_{trans}$", fontsize=11)

plot_slices_path = os.path.join(plots_dir, "heatmap_w_trans_3d_slices_goal2.png")
plt.savefig(plot_slices_path, bbox_inches='tight')
print(f"Salvato plot sezioni 3D: {plot_slices_path}")
plt.close()


# =========================================================================
# LIVELLO 2: Sensibilità all'Orientamento a Posizione Goal 2 Fissa
# =========================================================================
print("\n--- LIVELLO 2: Sensibilità all'Orientamento a Posizione Goal 2 Fissa ---")
p_g2 = p_base + goals["Goal 2"]
angles_deg = np.linspace(-20, 20, 21)

# Assi locali di rotazione (frame fer_hand_tcp): X_local (Roll), Y_local (Pitch), Z_local (Yaw)
w_trans_rot_x = []
w_trans_rot_y = []
w_trans_rot_z = []

q_g2_seed = q_base.copy()

for ang in angles_deg:
    rad = np.radians(ang)
    # Roll (rot attorno a X locale)
    Rx = R_des @ pin.exp3(np.array([rad, 0.0, 0.0]))
    ok_x, wt_x, _ = solve_ik(p_g2, Rx, q_g2_seed)
    w_trans_rot_x.append(wt_x if ok_x else np.nan)

    # Pitch (rot attorno a Y locale)
    Ry = R_des @ pin.exp3(np.array([0.0, rad, 0.0]))
    ok_y, wt_y, _ = solve_ik(p_g2, Ry, q_g2_seed)
    w_trans_rot_y.append(wt_y if ok_y else np.nan)

    # Yaw (rot attorno a Z locale)
    Rz = R_des @ pin.exp3(np.array([0.0, 0.0, rad]))
    ok_z, wt_z, _ = solve_ik(p_g2, Rz, q_g2_seed)
    w_trans_rot_z.append(wt_z if ok_z else np.nan)

fig_rot, ax_rot = plt.subplots(figsize=(8, 5.5), dpi=150)
ax_rot.plot(angles_deg, w_trans_rot_x, 'o-', color='tab:red', label=r'Roll (rot attorno a $X_{local}$)', linewidth=2)
ax_rot.plot(angles_deg, w_trans_rot_y, 's-', color='tab:green', label=r'Pitch (rot attorno a $Y_{local}$)', linewidth=2)
ax_rot.plot(angles_deg, w_trans_rot_z, '^-', color='tab:blue', label=r'Yaw (rot attorno a $Z_{local}$)', linewidth=2)

ax_rot.axvline(0, color='gray', linestyle='--', alpha=0.7, label='Orientamento Demo Originale (0°)')
ax_rot.axhline(goal_grid_vals["Goal 2"], color='red', linestyle=':', alpha=0.7, label=f'Goal 2 Nominale ({goal_grid_vals["Goal 2"]:.4f})')
ax_rot.set_xlabel(r'Deviazione Angolare $\Delta \theta$ dall\'Orientamento Demo [deg]', fontsize=11)
ax_rot.set_ylabel(r'Translational Manipulability $w_{trans}$', fontsize=11)
ax_rot.set_title('Sensibilità di $w_{trans}$ alla Variazione dell\'Orientamento a Posizione Goal 2 Fissa', fontsize=12, fontweight='bold')
ax_rot.grid(True, linestyle='--', alpha=0.6)
ax_rot.legend(fontsize=9.5)
plt.tight_layout()

plot_rot_path = os.path.join(plots_dir, "w_trans_orientation_sensitivity_goal2.png")
plt.savefig(plot_rot_path)
print(f"Salvato plot sensibilità orientamento: {plot_rot_path}")
plt.close()

# Stampa tabella di sensibilità all'orientamento
print("\nTabella Sensibilità Orientamento su Goal 2:")
print(f"{'Angolo [deg]':<12} | {'w_trans (Roll X)':<18} | {'w_trans (Pitch Y)':<18} | {'w_trans (Yaw Z)':<18}")
print("-" * 72)
for i in range(0, len(angles_deg), 2):
    ang = angles_deg[i]
    print(f"{ang:>+8.1f}°   | {w_trans_rot_x[i]:<18.5f} | {w_trans_rot_y[i]:<18.5f} | {w_trans_rot_z[i]:<18.5f}")
