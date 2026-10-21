import math
import numpy as np
import pinocchio as pin
import csv
import os
from scipy.stats import spearmanr

urdf_path = "/root/thesis_ws/fer_flat_effort.urdf"
model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()
frame_name = "fer_hand_tcp"
frame_id = model.getFrameId(frame_name)
JOINT_NAMES = [f"fer_joint{i}" for i in range(1, 8)]


def build_q_index(model, joint_names):
    """Resolves each joint name to its Pinocchio q-vector index (model.idx_qs[joint_id]),
    mirroring RobotModel::update's q_index resolution in robot_model.cpp. Needed because
    model.nq (9 with hand:=true: 7 arm joints + fer_finger_joint1/2) no longer matches the
    7-element q vectors read from joint_states_*.csv."""
    return [model.idx_qs[model.getJointId(name)] for name in joint_names]


def build_v_index(model, joint_names):
    """Resolves each joint name to its Pinocchio v-vector (tangent/velocity) index
    (model.idx_vs[joint_id]), mirroring RobotModel::update's v_index resolution in
    robot_model.cpp. Needed to slice a 6 x model.nv Jacobian back down to the 6x7 arm-only
    shape simulate_hold_final()'s DLS pseudoinverse / nullspace projector expect - that 7x7
    linear algebra is NOT invariant to the extra (structurally zero) finger-joint columns
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


Q_INDEX = build_q_index(model, JOINT_NAMES)
V_INDEX = build_v_index(model, JOINT_NAMES)


def compute_w_trans(q):
    q_full = pad_q(q)
    pin.computeJointJacobians(model, data, q_full)
    pin.framesForwardKinematics(model, data, q_full)
    J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    J_p = J[:3, :]
    sv = np.linalg.svd(J_p, compute_uv=False)
    return float(np.prod(sv))

data_dir = "/root/thesis_ws/tools/gazebo_cartesian_eval/data"

ALL_RUNS = [
    ("Baseline", "reach_task_baseline_impedance_kt200_delay1"),
    ("Goal 1", "reach_task_goal_1_prodmp"),
    ("Goal 2 rep1", "reach_task_goal_2_prodmp"),
    ("Goal 2 rep2", "reach_task_goal_2_rep2_prodmp"),
    ("Goal 2 rep3", "reach_task_goal_2_rep3_prodmp"),
    ("Goal 3", "reach_task_goal_3_prodmp"),
    ("Goal 4 rep1", "reach_task_goal_4_prodmp"),
    ("Goal 4 rep2", "reach_task_goal_4_rep2_prodmp"),
    ("Goal 4 rep3", "reach_task_goal_4_rep3_prodmp"),
    ("Goal 5", "reach_task_goal_5_prodmp"),
]

def simulate_hold_final(goal_pos, goal_quat, q0, k_ns=0.0, j1_scale=1.0, duration=8.0, dt=0.005, lam=0.05):
    q = np.copy(q0)
    n_steps = int(duration / dt)
    q_target = pin.Quaternion(goal_quat[3], goal_quat[0], goal_quat[1], goal_quat[2])
    q_target.normalize()
    R_target = q_target.toRotationMatrix()
    
    k_vec = np.ones(7) * k_ns
    k_vec[0] *= j1_scale
    
    for _ in range(n_steps):
        q_full = pad_q(q)
        pin.computeJointJacobians(model, data, q_full)
        pin.framesForwardKinematics(model, data, q_full)
        # getFrameJacobian returns 6 x model.nv (9 with hand:=true); slice back to the 7
        # arm-DOF columns via V_INDEX (mirroring RobotModel::update's Jacobian column
        # extraction) - required for the DLS pseudoinverse / nullspace projector below, which
        # are NOT invariant to the extra (structurally zero) finger-joint columns: keeping
        # them would turn N's 1D nullspace into 3D and break every 7-dim shape downstream
        # (q, k_vec, np.eye(7)).
        J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:, V_INDEX]
        oMf = data.oMf[frame_id]
        cur_pos = oMf.translation
        cur_rot = oMf.rotation
        
        e_pos = goal_pos - cur_pos
        R_err = R_target @ cur_rot.T
        e_rot = pin.log3(R_err)
        
        twist = np.zeros(6)
        twist[:3] = 1.0 * e_pos
        twist[3:] = 1.0 * e_rot
        
        JJt = J @ J.T + (lam**2) * np.eye(6)
        J_pinv = J.T @ np.linalg.inv(JJt)
        dq_primary = J_pinv @ twist
        
        N = np.eye(7) - J_pinv @ J
        dq_ns = N @ (k_vec * (q0 - q))
        dq = dq_primary + dq_ns
        q += dq * dt
        
    return q, compute_w_trans(q)

results = []

for label, run_key in ALL_RUNS:
    target_file = os.path.join(data_dir, f"target_aligned_{run_key}.csv")
    js_file = os.path.join(data_dir, f"joint_states_{run_key}.csv")
    manip_file = os.path.join(data_dir, f"manipulability_{run_key}.csv")
    
    if not os.path.exists(target_file):
        print(f"[WARN] Missing {target_file}")
        continue
        
    with open(target_file) as f:
        target_rows = list(csv.DictReader(f))
    with open(js_file) as f:
        js_rows = list(csv.DictReader(f))
    with open(manip_file) as f:
        manip_rows = list(csv.DictReader(f))
        
    q0 = np.array([float(js_rows[0][f"fer_joint{i}"]) for i in range(1, 8)])
    
    tail_w_trans = [float(r["w_trans"]) for r in manip_rows[-100:]]
    w_trans_real = float(np.mean(tail_w_trans))
    
    last_t = target_rows[-1]
    goal_pos = np.array([float(last_t["x"]), float(last_t["y"]), float(last_t["z"])])
    goal_quat = [float(last_t["qx"]), float(last_t["qy"]), float(last_t["qz"]), float(last_t["qw"])]
    
    _, w_dls = simulate_hold_final(goal_pos, goal_quat, q0, k_ns=0.0)
    _, w_ns = simulate_hold_final(goal_pos, goal_quat, q0, k_ns=0.2236, j1_scale=10.0)
    
    gap_dls_pct = (w_dls - w_trans_real) / w_trans_real * 100.0
    gap_ns_pct = (w_ns - w_trans_real) / w_trans_real * 100.0
    
    results.append({
        "run": label,
        "run_key": run_key,
        "w_trans_real": w_trans_real,
        "w_trans_dls": w_dls,
        "w_trans_nullspace_biased": w_ns,
        "gap_dls_pct": gap_dls_pct,
        "gap_ns_pct": gap_ns_pct,
    })

print("\n=== CSV PURO ===")
print("run,w_trans_real,w_trans_dls,w_trans_nullspace_biased,gap_dls_pct,gap_ns_pct")
for r in results:
    name = r["run"]
    wr = r["w_trans_real"]
    wd = r["w_trans_dls"]
    wns = r["w_trans_nullspace_biased"]
    gd = r["gap_dls_pct"]
    gns = r["gap_ns_pct"]
    print(f"{name},{wr:.4f},{wd:.4f},{wns:.4f},{gd:+.2f}%,{gns:+.2f}%")

print("\n=== RANKING COMPARISON ===")
sorted_by_real = sorted(results, key=lambda x: x["w_trans_real"], reverse=True)
sorted_by_ns = sorted(results, key=lambda x: x["w_trans_nullspace_biased"], reverse=True)
sorted_by_dls = sorted(results, key=lambda x: x["w_trans_dls"], reverse=True)

header = f"{'Rank':<5} | {'Sorted by Real':<24} | {'Sorted by NullspaceBiased':<30} | {'Sorted by MinNormDls':<27}"
print(header)
print("-" * len(header))
for i in range(len(results)):
    r_real = sorted_by_real[i]
    r_ns = sorted_by_ns[i]
    r_dls = sorted_by_dls[i]
    name_r = f"{r_real['run']} ({r_real['w_trans_real']:.4f})"
    name_ns = f"{r_ns['run']} ({r_ns['w_trans_nullspace_biased']:.4f})"
    name_dls = f"{r_dls['run']} ({r_dls['w_trans_dls']:.4f})"
    print(f"{i+1:<5} | {name_r:<24} | {name_ns:<30} | {name_dls:<27}")

ranks_real = [r["w_trans_real"] for r in results]
ranks_ns = [r["w_trans_nullspace_biased"] for r in results]
ranks_dls = [r["w_trans_dls"] for r in results]
rho_ns, p_ns = spearmanr(ranks_real, ranks_ns)
rho_dls, p_dls = spearmanr(ranks_real, ranks_dls)
print(f"\nSpearman Rank Correlation (Real vs NullspaceBiased): rho = {rho_ns:.4f} (p = {p_ns:.4e})")
print(f"Spearman Rank Correlation (Real vs MinNormDls):       rho = {rho_dls:.4f} (p = {p_dls:.4e})")
