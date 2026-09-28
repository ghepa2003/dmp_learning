#!/usr/bin/env python3
"""
compare_proxy_vs_real_phases.py
Compares predicted manipulability (JointPathSimulator with NullspaceBiasedResolution)
vs real manipulability measured in Gazebo (cartesian_impedance_controller Kt=2000)
at the end of the reach (at tau / t_clamp) for the 4 rotated phases.
"""

import os
import math
import csv
import bisect
import numpy as np
import pinocchio as pin

WS_ROOT = "/root/thesis_ws"
DATA_DIR = os.path.join(WS_ROOT, "tools/gazebo_cartesian_eval/data")
URDF_PATH = os.path.join(WS_ROOT, "fer_flat_effort.urdf")

PHASES = [30.0, 90.0, 180.0, 270.0]
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
    shape that simulate_proxy_nullspace()'s DLS pseudoinverse / nullspace projector expect -
    unlike a scalar manipulability index, that 7x7 linear algebra is NOT invariant to the
    extra (structurally zero) finger-joint columns model.nq/nv now carry under hand:=true."""
    return [model.idx_vs[model.getJointId(name)] for name in joint_names]


def pad_q(model, q_index, q):
    """Zero-pads a 7-element arm q into Pinocchio's full model.nq configuration vector, at
    the name-resolved indices in q_index. The extra DOFs (gripper finger prismatic joints
    under hand:=true) stay at 0.0: fer_hand_tcp is reached via a fixed joint upstream of the
    fingers, so their value does not affect its position."""
    q_full = np.zeros(model.nq)
    for i, idx in enumerate(q_index):
        q_full[idx] = q[i]
    return q_full

def load_data(run_name):
    tp_path = os.path.join(DATA_DIR, f"target_aligned_{run_name}.csv")
    js_path = os.path.join(DATA_DIR, f"joint_states_{run_name}.csv")

    def load_pose_csv(path):
        t, x, y, z, qw, qx, qy, qz = [], [], [], [], [], [], [], []
        with open(path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                t.append(float(row["t"]))
                x.append(float(row["x"])); y.append(float(row["y"])); z.append(float(row["z"]))
                qw.append(float(row["qw"])); qx.append(float(row["qx"]))
                qy.append(float(row["qy"])); qz.append(float(row["qz"]))
        return np.array(t), np.array(x), np.array(y), np.array(z), np.array(qw), np.array(qx), np.array(qy), np.array(qz)

    def load_js_csv(path):
        t, q = [], []
        with open(path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                t.append(float(row["t"]))
                q.append([float(row[jn]) for jn in JOINT_NAMES])
        return np.array(t), np.array(q)

    target = load_pose_csv(tp_path)
    js = load_js_csv(js_path)
    return target, js

def find_clamp_index(t_x, t_y, t_z, tol_m=1e-4):
    n = len(t_x)
    final_pos = (t_x[-1], t_y[-1], t_z[-1])
    for i in range(n - 1, -1, -1):
        d = math.sqrt((t_x[i] - final_pos[0])**2 + (t_y[i] - final_pos[1])**2 + (t_z[i] - final_pos[2])**2)
        if d > tol_m:
            return i + 1
    return 0

def compute_w_trans(model, data, frame_id, q_index, q):
    q_full = pad_q(model, q_index, q)
    pin.computeJointJacobians(model, data, q_full)
    pin.framesForwardKinematics(model, data, q_full)
    J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    J_p = J[:3, :]
    return math.sqrt(max(0.0, np.linalg.det(J_p @ J_p.T)))

def simulate_proxy_nullspace(model, data, frame_id, q_index, v_index, target_trajectory, q0, dt=0.005, damping_lambda=0.05, k_ns=0.2236, joint1_scale=10.0):
    """
    Exact simulation of JointPathSimulator with NullspaceBiasedResolution
    matching core/joint_path_simulator.cpp.
    """
    q_posture = np.array([0.0, -45.0, 0.0, -135.0, 0.0, 90.0, 45.0]) * (np.pi / 180.0)
    q = np.copy(q0)

    steps_q = [np.copy(q)]

    for k in range(len(target_trajectory) - 1):
        pos_curr_target = target_trajectory[k][:3]
        pos_next_target = target_trajectory[k+1][:3]
        quat_curr_target = target_trajectory[k][3:] # w, x, y, z
        quat_next_target = target_trajectory[k+1][3:]

        # Current EE state from FK
        q_full = pad_q(model, q_index, q)
        pin.computeJointJacobians(model, data, q_full)
        pin.framesForwardKinematics(model, data, q_full)
        ee_pos = data.oMf[frame_id].translation
        ee_quat = pin.Quaternion(data.oMf[frame_id].rotation) # x, y, z, w internally in pin
        
        # Position error
        pos_err = pos_next_target - ee_pos
        
        # Orientation error (quaternion log / axis angle)
        q_target = pin.Quaternion(quat_next_target[0], quat_next_target[1], quat_next_target[2], quat_next_target[3])
        # Shortest path hemisphere alignment
        if ee_quat.dot(q_target) < 0:
            q_target.coeffs()[:] = -q_target.coeffs()
        
        # Relative rotation
        q_err = ee_quat.inverse() * q_target
        # Rotation vector in world frame
        # Using LocalWorldAligned conventions
        rot_err_local = pin.log3(q_err.toRotationMatrix())
        rot_err_world = data.oMf[frame_id].rotation @ rot_err_local
        
        # Task velocity (linear + angular)
        v_task = np.zeros(6)
        v_task[:3] = pos_err / dt
        v_task[3:] = rot_err_world / dt
        
        # Clamp Cartesian speed if desired, or pure Kp
        # Full 6x7 Jacobian. getFrameJacobian returns 6 x model.nv (9 with hand:=true); slice
        # back to the 7 arm-DOF columns via v_index (mirroring RobotModel::update's Jacobian
        # column extraction) - the finger columns are structurally zero (fer_hand_tcp is
        # upstream of them) but MUST be dropped, not just ignored, because keeping them would
        # change the nullspace projector N below from 1D (7-7+6=... true arm self-motion) to a
        # 3D nullspace that also contains the two trivial finger directions, and would break
        # the shape of every 7-dim quantity downstream (q, k_ns_vec, np.eye(7)).
        J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:, v_index]

        # DLS pseudoinverse: J^# = J^T (J J^T + lambda^2 I)^-1
        JJT = J @ J.T + (damping_lambda**2) * np.eye(6)
        J_pinv = J.T @ np.linalg.inv(JJT)
        
        # Primary velocity: qdot_task = J_pinv @ v_task
        qdot_task = J_pinv @ v_task
        
        # Nullspace projector: N = I - J_pinv @ J
        N = np.eye(7) - J_pinv @ J
        
        # Secondary objective: qdot_null = N @ (K_ns_diag @ (q_posture - q))
        k_ns_vec = np.array([k_ns * joint1_scale, k_ns, k_ns, k_ns, k_ns, k_ns, k_ns])
        qdot_null = N @ (k_ns_vec * (q_posture - q))
        
        qdot = qdot_task + qdot_null
        
        # Euler integration
        q = q + qdot * dt
        steps_q.append(np.copy(q))
        
    return steps_q

def main():
    model = pin.buildModelFromUrdf(URDF_PATH)
    data = model.createData()
    frame_id = model.getFrameId("fer_hand_tcp")
    q_index = build_q_index(model, JOINT_NAMES)
    v_index = build_v_index(model, JOINT_NAMES)

    results = []
    
    for p in PHASES:
        run_name = f"reach_task_satellite_rot_phase{int(p)}"
        target, js = load_data(run_name)
        t_t, t_x, t_y, t_z, t_qw, t_qx, t_qy, t_qz = target
        t_js, q_js = js
        
        # Find clamp / tau index
        clamp_idx = find_clamp_index(t_x, t_y, t_z)
        t_clamp = t_t[clamp_idx]
        
        # Real joint state at tau / t_clamp
        j_tau = bisect.bisect_left(t_js, t_clamp)
        j_tau = min(max(j_tau, 0), len(t_js) - 1)
        q_real_tau = q_js[j_tau]
        w_real_tau = compute_w_trans(model, data, frame_id, q_index, q_real_tau)

        # Final real joint state
        q_real_final = q_js[-1]
        w_real_final = compute_w_trans(model, data, frame_id, q_index, q_real_final)

        # Build target trajectory up to clamp_idx
        traj_samples = []
        for i in range(clamp_idx + 1):
            traj_samples.append(np.array([t_x[i], t_y[i], t_z[i], t_qw[i], t_qx[i], t_qy[i], t_qz[i]]))

        q0 = q_js[0]
        # Simulate proxy
        sim_q = simulate_proxy_nullspace(model, data, frame_id, q_index, v_index, traj_samples, q0)
        q_proxy_tau = sim_q[-1]
        w_proxy_tau = compute_w_trans(model, data, frame_id, q_index, q_proxy_tau)
        
        gap_pct_tau = 100.0 * (w_proxy_tau - w_real_tau) / w_real_tau
        gap_pct_final = 100.0 * (w_proxy_tau - w_real_final) / w_real_final
        
        results.append({
            "phase": p,
            "w_real_tau": w_real_tau,
            "w_proxy_tau": w_proxy_tau,
            "gap_pct_tau": gap_pct_tau,
            "w_real_final": w_real_final,
            "gap_pct_final": gap_pct_final,
            "q_real_tau_deg": [math.degrees(v) for v in q_real_tau],
            "q_proxy_tau_deg": [math.degrees(v) for v in q_proxy_tau],
        })

    print("\n" + "="*80)
    print("CONFRONTO PROXY (NullspaceBiased) vs REALE (Impedance Kt=2000) A TAU")
    print("="*80)
    print("frozen_phase_deg,w_trans_real_tau,w_trans_proxy,gap_pct_at_tau")
    for r in results:
        print(f"{r['phase']:.1f},{r['w_real_tau']:.4f},{r['w_proxy_tau']:.4f},{r['gap_pct_tau']:+.2f}%")

    # Check ranking
    real_ranking = sorted(results, key=lambda x: x["w_real_tau"], reverse=True)
    proxy_ranking = sorted(results, key=lambda x: x["w_proxy_tau"], reverse=True)
    
    real_order = [r["phase"] for r in real_ranking]
    proxy_order = [r["phase"] for r in proxy_ranking]
    
    print("\n" + "="*80)
    print("RANKING COMPARISON (Best -> Worst)")
    print("="*80)
    print("Real ranking at tau :", [f"{p:.0f}° ({r['w_real_tau']:.4f})" for p, r in zip(real_order, real_ranking)])
    print("Proxy ranking at tau:", [f"{p:.0f}° ({r['w_proxy_tau']:.4f})" for p, r in zip(proxy_order, proxy_ranking)])
    print(f"Ranking matches exactly: {real_order == proxy_order}")
    print("="*80)

if __name__ == "__main__":
    main()
