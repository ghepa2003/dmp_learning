import csv
import numpy as np
import os
import pinocchio as pin

urdf_path = "/root/thesis_ws/fer_flat_effort.urdf"
model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()
frame_name = "fer_hand_tcp"
frame_id = model.getFrameId(frame_name)


def build_q_index(model, joint_names):
    """Resolves each joint name to its Pinocchio q-vector index (model.idx_qs[joint_id]),
    mirroring RobotModel::update's q_index resolution in robot_model.cpp. Needed because
    model.nq (9 with hand:=true: 7 arm joints + fer_finger_joint1/2) no longer matches the
    7-element q vectors read from joint_states_*.csv."""
    return [model.idx_qs[model.getJointId(name)] for name in joint_names]


JOINT_NAMES = [f"fer_joint{i}" for i in range(1, 8)]
Q_INDEX = build_q_index(model, JOINT_NAMES)


def compute_w_trans(q):
    # Zero-pad the 7 actuated arm values into Pinocchio's full model.nq configuration
    # vector, at the name-resolved indices in Q_INDEX. The extra DOFs (gripper finger
    # prismatic joints under hand:=true) stay at 0.0: fer_hand_tcp is reached via a fixed
    # joint upstream of the fingers, so their value does not affect its position, and their
    # Jacobian columns are structurally zero - harmless to the determinant/SVD below.
    q_full = np.zeros(model.nq)
    for i, idx in enumerate(Q_INDEX):
        q_full[idx] = q[i]
    pin.computeJointJacobians(model, data, q_full)
    pin.framesForwardKinematics(model, data, q_full)
    J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    J_p = J[:3, :]
    sv = np.linalg.svd(J_p, compute_uv=False)
    return float(np.prod(sv))

data_dir = "/root/thesis_ws/tools/gazebo_cartesian_eval/data"

def load_js(run_key):
    js_file = os.path.join(data_dir, f"joint_states_{run_key}.csv")
    with open(js_file) as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r['t']) for r in rows])
    q = np.array([[float(r[f'fer_joint{i}']) for i in range(1, 8)] for r in rows])
    return t, q

# 1. Goal 2 analysis
t1, q1 = load_js("reach_task_goal_2_prodmp")
t2, q2 = load_js("reach_task_goal_2_rep2_prodmp")
t3, q3 = load_js("reach_task_goal_2_rep3_prodmp")

print("=== GOAL 2: DIVERGENCE BETWEEN REPLICATES ===")
for target_t in [0.0, 0.2, 0.5, 0.8, 1.0, 1.5, 2.0, 3.0, 5.0, 10.0, 20.0]:
    idx1 = np.argmin(np.abs(t1 - target_t))
    idx2 = np.argmin(np.abs(t2 - target_t))
    idx3 = np.argmin(np.abs(t3 - target_t))
    
    q1_deg = np.rad2deg(q1[idx1])
    q2_deg = np.rad2deg(q2[idx2])
    q3_deg = np.rad2deg(q3[idx3])
    
    w1 = compute_w_trans(q1[idx1])
    w2 = compute_w_trans(q2[idx2])
    w3 = compute_w_trans(q3[idx3])
    
    diff_3_vs_1 = q3_deg - q1_deg
    diff_2_vs_1 = q2_deg - q1_deg
    max_j = np.argmax(np.abs(diff_3_vs_1)) + 1
    max_diff = diff_3_vs_1[max_j - 1]
    
    print(f"t={target_t:4.1f}s | w1={w1:.4f}, w2={w2:.4f}, w3={w3:.4f} | Max diff on J{max_j}: {max_diff:+5.1f}° | J4: {q1_deg[3]:5.1f} vs {q3_deg[3]:5.1f} | J2: {q1_deg[1]:5.1f} vs {q3_deg[1]:5.1f} | J6: {q1_deg[5]:5.1f} vs {q3_deg[5]:5.1f}")

# 2. Goal 4 analysis
t4_1, q4_1 = load_js("reach_task_goal_4_prodmp")
t4_2, q4_2 = load_js("reach_task_goal_4_rep2_prodmp")
t4_3, q4_3 = load_js("reach_task_goal_4_rep3_prodmp")

print("\n=== GOAL 4: DIVERGENCE BETWEEN REPLICATES ===")
for target_t in [0.0, 0.2, 0.5, 0.8, 1.0, 1.5, 2.0, 3.0, 5.0, 10.0, 20.0]:
    idx1 = np.argmin(np.abs(t4_1 - target_t))
    idx2 = np.argmin(np.abs(t4_2 - target_t))
    idx3 = np.argmin(np.abs(t4_3 - target_t))
    
    q1_deg = np.rad2deg(q4_1[idx1])
    q2_deg = np.rad2deg(q4_2[idx2])
    q3_deg = np.rad2deg(q4_3[idx3])
    
    w1 = compute_w_trans(q4_1[idx1])
    w2 = compute_w_trans(q4_2[idx2])
    w3 = compute_w_trans(q4_3[idx3])
    
    diff_2_vs_1 = q2_deg - q1_deg
    max_j = np.argmax(np.abs(diff_2_vs_1)) + 1
    max_diff = diff_2_vs_1[max_j - 1]
    
    print(f"t={target_t:4.1f}s | w1={w1:.4f}, w2={w2:.4f}, w3={w3:.4f} | Max diff on J{max_j}: {max_diff:+5.1f}° | J4: {q1_deg[3]:5.1f} vs {q2_deg[3]:5.1f} | J2: {q1_deg[1]:5.1f} vs {q2_deg[1]:5.1f} | J6: {q1_deg[5]:5.1f} vs {q2_deg[5]:5.1f}")
