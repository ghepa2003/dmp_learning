#!/usr/bin/env python3
import os, sys, csv, math
import numpy as np
import pinocchio as pin
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

eval_root = '/root/thesis_ws/tools/gazebo_cartesian_eval'
data_dir = os.path.join(eval_root, 'data')
plots_dir = os.path.join(eval_root, 'plots/goal_generalization')
os.makedirs(plots_dir, exist_ok=True)
urdf_path = '/root/thesis_ws/fer_flat_effort.urdf'

model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()
frame_id = model.getFrameId('fer_hand_tcp')
JOINT_NAMES = [f'fer_joint{i}' for i in range(1, 8)]


def build_q_index(model, joint_names):
    """Resolves each joint name to its Pinocchio q-vector index (model.idx_qs[joint_id]),
    mirroring RobotModel::update's q_index resolution in robot_model.cpp. Needed because
    model.nq (9 with hand:=true: 7 arm joints + fer_finger_joint1/2) no longer matches the
    7-element q vectors read from joint_states_*.csv."""
    return [model.idx_qs[model.getJointId(name)] for name in joint_names]


Q_INDEX = build_q_index(model, JOINT_NAMES)

runs_all = [
    ('reach_task_goal_1_prodmp', 'Goal 1 (Run 1)', 'Goal 1', 'tab:blue', 'o'),
    ('reach_task_goal_1_rep2_prodmp', 'Goal 1 (Run 2)', 'Goal 1', 'tab:blue', 's'),
    ('reach_task_goal_2_prodmp', 'Goal 2 (Run 1)', 'Goal 2', 'tab:red', 'o'),
    ('reach_task_goal_2_rep2_prodmp', 'Goal 2 (Run 2)', 'Goal 2', 'tab:red', 's'),
    ('reach_task_goal_2_rep3_prodmp', 'Goal 2 (Run 3)', 'Goal 2', 'tab:red', '^'),
    ('reach_task_goal_3_prodmp', 'Goal 3 (orig)', 'Goal 3', 'tab:green', 'o'),
    ('reach_task_goal_4_prodmp', 'Goal 4 (Run 1)', 'Goal 4', 'tab:purple', 'o'),
    ('reach_task_goal_4_rep2_prodmp', 'Goal 4 (Run 2)', 'Goal 4', 'tab:purple', 's'),
    ('reach_task_goal_4_rep3_prodmp', 'Goal 4 (Run 3)', 'Goal 4', 'tab:purple', '^'),
    ('reach_task_goal_5_prodmp', 'Goal 5 (orig)', 'Goal 5', 'tab:orange', 'o'),
]

def eval_at_tau(target_path, actual_path):
    t_t, t_x, t_y, t_z = [], [], [], []
    with open(target_path) as f:
        for row in csv.DictReader(f):
            t_t.append(float(row['t']))
            x, y, z = float(row['x']), float(row['y']), float(row['z'])
            t_x.append(x); t_y.append(y); t_z.append(z)
    a_t, a_x, a_y, a_z = [], [], [], []
    with open(actual_path) as f:
        for row in csv.DictReader(f):
            a_t.append(float(row['t']))
            x, y, z = float(row['x']), float(row['y']), float(row['z'])
            a_x.append(x); a_y.append(y); a_z.append(z)

    final_tgt = (t_x[-1], t_y[-1], t_z[-1])
    idx_clamp = 0
    for i in range(len(t_t)-1, -1, -1):
        if math.dist((t_x[i], t_y[i], t_z[i]), final_tgt) > 1e-4:
            idx_clamp = i + 1
            break
    tau_t = t_t[idx_clamp]
    import bisect
    j = bisect.bisect_left(a_t, tau_t)
    j = min(max(j, 0), len(a_t)-1)
    err_tau = math.dist((a_x[j], a_y[j], a_z[j]), (t_x[idx_clamp], t_y[idx_clamp], t_z[idx_clamp]))
    return tau_t, err_tau * 1000

def load_joint_states(js_path):
    t_list, q_list = [], []
    with open(js_path, 'r') as f:
        for row in csv.DictReader(f):
            t_list.append(float(row['t']))
            q_list.append([float(row[jn]) for jn in JOINT_NAMES])
    return np.array(t_list), np.array(q_list)

results = []
for r_key, r_label, group, color, marker in runs_all:
    tp = os.path.join(data_dir, f'target_aligned_{r_key}.csv')
    ap = os.path.join(data_dir, f'actual_pose_{r_key}.csv')
    jp = os.path.join(data_dir, f'joint_states_{r_key}.csv')
    
    tau_t, err_tau = eval_at_tau(tp, ap)
    t_js, q_js = load_joint_states(jp)
    
    w_trans, w_rot, w_full = [], [], []
    for q in q_js:
        # Zero-pad the 7 actuated arm values into Pinocchio's full model.nq configuration
        # vector, at the name-resolved indices in Q_INDEX. The extra DOFs (gripper finger
        # prismatic joints under hand:=true) stay at 0.0: fer_hand_tcp is reached via a
        # fixed joint upstream of the fingers, so their value does not affect its position,
        # and their Jacobian columns are structurally zero - harmless to the determinants
        # below.
        q_full = np.zeros(model.nq)
        for i, idx in enumerate(Q_INDEX):
            q_full[idx] = q[i]
        pin.computeJointJacobians(model, data, q_full)
        pin.framesForwardKinematics(model, data, q_full)
        J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        w_trans.append(math.sqrt(max(0.0, np.linalg.det(J[:3, :] @ J[:3, :].T))))
        w_rot.append(math.sqrt(max(0.0, np.linalg.det(J[3:, :] @ J[3:, :].T))))
        w_full.append(math.sqrt(max(0.0, np.linalg.det(J @ J.T))))
    
    w_trans = np.array(w_trans)
    w_rot = np.array(w_rot)
    w_full = np.array(w_full)
    
    # Save manipulability csv
    out_csv = os.path.join(data_dir, f'manipulability_{r_key}.csv')
    with open(out_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['t', 'w_full', 'w_trans', 'w_rot'] + JOINT_NAMES)
        for i in range(len(t_js)):
            writer.writerow([t_js[i], w_full[i], w_trans[i], w_rot[i]] + list(q_js[i]))
            
    mask_pre = (t_js >= (tau_t - 1.0)) & (t_js <= tau_t)
    wt_pre = np.mean(w_trans[mask_pre])
    wr_pre = np.mean(w_rot[mask_pre])
    wf_pre = np.mean(w_full[mask_pre])
    
    results.append({
        'key': r_key, 'label': r_label, 'group': group,
        'color': color, 'marker': marker,
        'tau_t': tau_t, 'err_tau': err_tau,
        'wt_pre': wt_pre, 'wr_pre': wr_pre, 'wf_pre': wf_pre
    })

print(f'=== RESULTS FOR ALL {len(results)} RUNS ===')
for r in results:
    print(f"{r['label']:<20}: err@tau={r['err_tau']:6.3f} mm | w_trans={r['wt_pre']:.5f} | w_rot={r['wr_pre']:.5f} | w_full={r['wf_pre']:.5f}")

# Correlation on N=9 (excluding Goal 1 rep 2)
n9_res = [r for r in results if r['key'] != 'reach_task_goal_1_rep2_prodmp']
wt_n9 = [r['wt_pre'] for r in n9_res]
err_n9 = [r['err_tau'] for r in n9_res]
r_n9 = np.corrcoef(wt_n9, err_n9)[0, 1]

# Correlation on N=10 (all)
wt_n10 = [r['wt_pre'] for r in results]
err_n10 = [r['err_tau'] for r in results]
r_n10 = np.corrcoef(wt_n10, err_n10)[0, 1]

print(f'\nPearson r (w_trans vs err@tau, n=9):  r = {r_n9:.4f}')
print(f'Pearson r (w_trans vs err@tau, n=10): r = {r_n10:.4f}')

# Generate Scatter Plot
fig, ax = plt.subplots(figsize=(8, 6), dpi=150)
for r in results:
    ax.scatter(r['wt_pre'], r['err_tau'], color=r['color'], marker=r['marker'], s=80, alpha=0.85, label=r['label'])

ax.set_xlabel('Translational Manipulability $w_{trans}$ (pre-clamp [$\\tau-1s, \\tau$])', fontsize=11)
ax.set_ylabel('Residual Tracking Error [@ $\\tau$] [mm]', fontsize=11)
ax.set_title(f'Translational Manipulability vs Residual Error [@ $\\tau$]\n(Pearson r = {r_n10:.3f}, N=10)', fontsize=12, fontweight='bold')
ax.grid(True, linestyle='--', alpha=0.5)
ax.legend(bbox_to_anchor=(1.04, 1), loc='upper left', fontsize=9)
plt.tight_layout()

plot_path = os.path.join(plots_dir, 'scatter_w_trans_vs_error.png')
plt.savefig(plot_path)
print(f'Saved scatter plot: {plot_path}')
