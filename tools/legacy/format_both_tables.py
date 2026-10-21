#!/usr/bin/env python3
import os
import math
import csv
import bisect
import numpy as np
import pinocchio as pin

WS_ROOT = "/root/thesis_ws"
DATA_DIR = os.path.join(WS_ROOT, "tools/gazebo_cartesian_eval/data")
URDF_PATH = os.path.join(WS_ROOT, "fer_flat_effort.urdf")
JOINT_NAMES = [f"fer_joint{i}" for i in range(1, 8)]
PHASES = [30.0, 90.0, 180.0, 270.0]

model = pin.buildModelFromUrdf(URDF_PATH)
data = model.createData()
frame_id = model.getFrameId("fer_hand_tcp")


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
    shape simulate_proxy()'s DLS pseudoinverse / nullspace projector expect - that 7x7 linear
    algebra is NOT invariant to the extra (structurally zero) finger-joint columns model.nq/nv
    now carry under hand:=true."""
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
    return math.sqrt(max(0.0, np.linalg.det(J_p @ J_p.T)))

def simulate_proxy(target_samples, q0):
    dt = 0.005
    k_ns = 0.2236
    joint1_scale = 10.0
    damping_lambda = 0.05
    q_posture = np.array([0.0, -45.0, 0.0, -135.0, 0.0, 90.0, 45.0]) * (np.pi / 180.0)
    q = np.copy(q0)
    for k in range(len(target_samples) - 1):
        pos_next = target_samples[k+1][:3]
        quat_next = target_samples[k+1][3:]
        q_full = pad_q(q)
        pin.computeJointJacobians(model, data, q_full)
        pin.framesForwardKinematics(model, data, q_full)
        ee_pos = data.oMf[frame_id].translation
        ee_quat = pin.Quaternion(data.oMf[frame_id].rotation)
        pos_err = pos_next - ee_pos
        q_target = pin.Quaternion(quat_next[0], quat_next[1], quat_next[2], quat_next[3])
        if ee_quat.dot(q_target) < 0:
            q_target.coeffs()[:] = -q_target.coeffs()
        q_err = ee_quat.inverse() * q_target
        rot_err_local = pin.log3(q_err.toRotationMatrix())
        rot_err_world = data.oMf[frame_id].rotation @ rot_err_local
        v_task = np.zeros(6)
        v_task[:3] = pos_err / dt
        v_task[3:] = rot_err_world / dt
        # getFrameJacobian returns 6 x model.nv (9 with hand:=true); slice back to the 7
        # arm-DOF columns via V_INDEX (mirroring RobotModel::update's Jacobian column
        # extraction) - required for the nullspace projector N below, not just a shape fix:
        # keeping the finger columns would turn N's 1D nullspace into 3D.
        J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:, V_INDEX]
        JJT = J @ J.T + (damping_lambda**2) * np.eye(6)
        J_pinv = J.T @ np.linalg.inv(JJT)
        qdot_task = J_pinv @ v_task
        N = np.eye(7) - J_pinv @ J
        k_ns_vec = np.array([k_ns * joint1_scale, k_ns, k_ns, k_ns, k_ns, k_ns, k_ns])
        qdot_null = N @ (k_ns_vec * (q_posture - q))
        q = q + (qdot_task + qdot_null) * dt
    return q

def load_run(suffix):
    res = {}
    for p in PHASES:
        p_int = int(p)
        run_name = f"reach_task_satellite_rot_phase{p_int}{suffix}"
        tp_path = os.path.join(DATA_DIR, f"target_aligned_{run_name}.csv")
        ap_path = os.path.join(DATA_DIR, f"actual_pose_{run_name}.csv")
        js_path = os.path.join(DATA_DIR, f"joint_states_{run_name}.csv")
        
        t_t, t_x, t_y, t_z, t_qw, t_qx, t_qy, t_qz = [], [], [], [], [], [], [], []
        with open(tp_path) as f:
            for r in csv.DictReader(f):
                t_t.append(float(r["t"]))
                t_x.append(float(r["x"])); t_y.append(float(r["y"])); t_z.append(float(r["z"]))
                t_qw.append(float(r["qw"])); t_qx.append(float(r["qx"]))
                t_qy.append(float(r["qy"])); t_qz.append(float(r["qz"]))
        
        a_t, a_x, a_y, a_z, a_qw, a_qx, a_qy, a_qz = [], [], [], [], [], [], [], []
        with open(ap_path) as f:
            for r in csv.DictReader(f):
                a_t.append(float(r["t"]))
                a_x.append(float(r["x"])); a_y.append(float(r["y"])); a_z.append(float(r["z"]))
                a_qw.append(float(r["qw"])); a_qx.append(float(r["qx"]))
                a_qy.append(float(r["qy"])); a_qz.append(float(r["qz"]))
                
        js_t, js_q = [], []
        with open(js_path) as f:
            for r in csv.DictReader(f):
                js_t.append(float(r["t"]))
                js_q.append([float(r[jn]) for jn in JOINT_NAMES])
        
        final_pos = (t_x[-1], t_y[-1], t_z[-1])
        clamp_idx = 0
        for i in range(len(t_x) - 1, -1, -1):
            d = math.sqrt((t_x[i] - final_pos[0])**2 + (t_y[i] - final_pos[1])**2 + (t_z[i] - final_pos[2])**2)
            if d > 1e-4:
                clamp_idx = i + 1
                break
        t_clamp = t_t[clamp_idx]
        
        pos_errs, ori_errs = [], []
        for i in range(len(a_t)):
            j = bisect.bisect_left(t_t, a_t[i])
            j = min(max(j, 0), len(t_t) - 1)
            dp = math.sqrt((a_x[i] - t_x[j])**2 + (a_y[i] - t_y[j])**2 + (a_z[i] - t_z[j])**2) * 1000.0
            pos_errs.append(dp)
            dot = abs(a_qw[i]*t_qw[j] + a_qx[i]*t_qx[j] + a_qy[i]*t_qy[j] + a_qz[i]*t_qz[j])
            dot = max(-1.0, min(1.0, dot))
            da = 2.0 * math.degrees(math.acos(dot))
            ori_errs.append(da)
            
        j_tau = bisect.bisect_left(a_t, t_clamp)
        j_tau = min(max(j_tau, 0), len(a_t) - 1)
        
        err_pos_tau = pos_errs[j_tau]
        err_ori_tau = ori_errs[j_tau]
        err_pos_final = pos_errs[-1]
        err_ori_final = ori_errs[-1]
        
        q_final = np.array(js_q[-1])
        w_final = compute_w_trans(q_final)
        
        traj_samples = []
        for i in range(clamp_idx + 1):
            traj_samples.append(np.array([t_x[i], t_y[i], t_z[i], t_qw[i], t_qx[i], t_qy[i], t_qz[i]]))
        q_proxy = simulate_proxy(traj_samples, np.array(js_q[0]))
        w_proxy = compute_w_trans(q_proxy)
        gap_pct = 100.0 * (w_proxy - w_final) / w_final
        
        res[p] = {
            "phase": p,
            "err_pos_tau": err_pos_tau,
            "err_ori_tau": err_ori_tau,
            "err_pos_final": err_pos_final,
            "err_ori_final": err_ori_final,
            "w_final": w_final,
            "w_proxy": w_proxy,
            "gap_pct": gap_pct,
            "q_final_deg": [math.degrees(v) for v in q_final]
        }
    return res

res_kt2000 = load_run("")
res_kt200 = load_run("_kt200")

print("=== TABLE KT=200 ===")
print("frozen_phase_deg,err_pos_at_tau_mm,err_ori_at_tau_deg,err_pos_at_final_mm,err_ori_at_final_deg,w_trans_at_final,w_trans_proxy,gap_pct")
for p in PHASES:
    r = res_kt200[p]
    p_val = r["phase"]
    ept = r["err_pos_tau"]
    eot = r["err_ori_tau"]
    epf = r["err_pos_final"]
    eof = r["err_ori_final"]
    wf = r["w_final"]
    wp = r["w_proxy"]
    gp = r["gap_pct"]
    print(f"{p_val:.1f},{ept:.3f},{eot:.3f},{epf:.3f},{eof:.3f},{wf:.4f},{wp:.4f},{gp:+.2f}%")

print("\n=== TABLE KT=2000 ===")
print("frozen_phase_deg,err_pos_at_tau_mm,err_ori_at_tau_deg,err_pos_at_final_mm,err_ori_at_final_deg,w_trans_at_final,w_trans_proxy,gap_pct")
for p in PHASES:
    r = res_kt2000[p]
    p_val = r["phase"]
    ept = r["err_pos_tau"]
    eot = r["err_ori_tau"]
    epf = r["err_pos_final"]
    eof = r["err_ori_final"]
    wf = r["w_final"]
    wp = r["w_proxy"]
    gp = r["gap_pct"]
    print(f"{p_val:.1f},{ept:.3f},{eot:.3f},{epf:.3f},{eof:.3f},{wf:.4f},{wp:.4f},{gp:+.2f}%")
