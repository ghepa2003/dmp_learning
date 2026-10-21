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

def print_final(run_name, file_key):
    js_file = os.path.join(data_dir, f"joint_states_{file_key}.csv")
    with open(js_file) as f:
        rows = list(csv.DictReader(f))
    q_final = np.array([float(rows[-1][f'fer_joint{i}']) for i in range(1, 8)])
    w = compute_w_trans(q_final)
    print(f"{run_name:<15} | w_trans={w:.4f} | q(deg): {np.round(np.rad2deg(q_final), 1)}")

print("=== FINAL STEADY-STATE JOINT CONFIGURATIONS ===")
print_final("Goal 2 rep1", "reach_task_goal_2_prodmp")
print_final("Goal 2 rep2", "reach_task_goal_2_rep2_prodmp")
print_final("Goal 2 rep3", "reach_task_goal_2_rep3_prodmp")
print()
print_final("Goal 4 rep1", "reach_task_goal_4_prodmp")
print_final("Goal 4 rep2", "reach_task_goal_4_rep2_prodmp")
print_final("Goal 4 rep3", "reach_task_goal_4_rep3_prodmp")
