#!/usr/bin/env python3
"""
sweep_free_roll_verification.py
Deterministic offline analysis of 2-DOF grasp with free roll optimization
around the approach axis for Franka Emika Panda.
Satellite rotation axis is fixed to x = [1, 0, 0].
"""

import sys
import os
import math
import numpy as np
import pinocchio as pin
import yaml

# 1. Joint Limits for Franka Emika Panda (rad & deg)
JOINT_LIMITS = [
    (-2.8973, 2.8973),   # q1: [-166.0, 166.0] deg
    (-1.7628, 1.7628),   # q2: [-101.0, 101.0] deg
    (-2.8973, 2.8973),   # q3: [-166.0, 166.0] deg
    (-3.0718, -0.0698),  # q4: [-176.0, -4.0] deg
    (-2.8973, 2.8973),   # q5: [-166.0, 166.0] deg
    (-0.0175, 3.7525),   # q6: [-1.0, 215.0] deg
    (-2.8973, 2.8973),   # q7: [-166.0, 166.0] deg
]

READY_POSE = np.array([0.0, -np.pi/4, 0.0, -3*np.pi/4, 0.0, np.pi/2, np.pi/4, 0.0, 0.0]) # 9 dof
SAT_CENTER = np.array([0.45, -0.05, 0.35])
SAT_AXIS = np.array([1.0, 0.0, 0.0]) # CORRECTED: x-axis
DEMO_DISPLACEMENT = np.array([0.085913, -0.114969, -0.188120])

URDF_PATH = "/root/thesis_ws/fer_flat_effort.urdf"
WEIGHTS_PATH = "/root/thesis_ws/runs/20260915_090358_fit_reach_task_baseline_prodmp/weights.yaml"

def build_robot():
    model = pin.buildModelFromUrdf(URDF_PATH)
    data = model.createData()
    tcp_frame_id = model.getFrameId("fer_hand_tcp")
    return model, data, tcp_frame_id

def get_ee_pose(model, data, tcp_frame_id, q_7dof):
    q_full = np.zeros(9)
    q_full[:7] = q_7dof
    pin.forwardKinematics(model, data, q_full)
    pin.updateFramePlacements(model, data)
    pos = data.oMf[tcp_frame_id].translation.copy()
    rot = data.oMf[tcp_frame_id].rotation.copy()
    return pos, rot

def get_jacobian(model, data, tcp_frame_id, q_7dof):
    q_full = np.zeros(9)
    q_full[:7] = q_7dof
    pin.computeJointJacobians(model, data, q_full)
    pin.updateFramePlacements(model, data)
    J6 = pin.getFrameJacobian(model, data, tcp_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    return J6[:6, :7]

def compute_w_trans(J6):
    J_pos = J6[:3, :7]
    JJt = J_pos @ J_pos.T
    det = max(0.0, np.linalg.det(JJt))
    return math.sqrt(det)

def rot_axis_angle(axis, angle_rad):
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    ux, uy, uz = axis / np.linalg.norm(axis)
    return np.array([
        [c + ux*ux*(1-c),    ux*uy*(1-c) - uz*s, ux*uz*(1-c) + uy*s],
        [uy*ux*(1-c) + uz*s, c + uy*uy*(1-c),    uy*uz*(1-c) - ux*s],
        [uz*ux*(1-c) - uy*s, uz*uy*(1-c) + ux*s, c + uz*uz*(1-c)]
    ])

def rot_z(psi_rad):
    c = np.cos(psi_rad)
    s = np.sin(psi_rad)
    return np.array([
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0]
    ])

def quat_slerp(q0, q1, s):
    q0_pin = pin.Quaternion(q0)
    q1_pin = pin.Quaternion(q1)
    if q0_pin.dot(q1_pin) < 0:
        q1_pin = pin.Quaternion(-q1_pin.coeffs())
    return q0_pin.slerp(s, q1_pin)

class ProDMPTrajectory:
    def __init__(self, weights_path):
        with open(weights_path, "r") as f:
            w = yaml.safe_load(f)
        self.num_basis = w["num_basis"]
        self.alpha = float(w["alpha"])
        self.alpha_x = float(w["alpha_x"])
        self.tau = float(w["tau"])
        self.centers = np.array(w["centers"], dtype=np.float64)
        self.widths = np.array(w["widths"], dtype=np.float64)
        self.weights = np.array([item["values"] for item in w["weights"]], dtype=np.float64)

    def generate_positions(self, p_init, p_goal, num_samples=100):
        times = np.linspace(0, self.tau, num_samples)
        y1_0 = 1.0
        y2_0 = 0.0
        dy1_0 = -0.5 * self.alpha * y1_0
        dy2_0 = y1_0 - 0.5 * self.alpha * y2_0
        det = y1_0 * dy2_0 - y2_0 * dy1_0

        c1 = (dy2_0 * (p_init - p_goal) - 0.0) / det
        c2 = (-dy1_0 * (p_init - p_goal) + 0.0) / det

        pos_list = []
        p1_accum = np.zeros(3)
        p2_accum = np.zeros(3)

        x0 = np.exp(-self.alpha_x * 0.0)
        dist0 = (x0 - self.centers)**2
        psi0 = np.exp(-0.5 * self.widths * dist0)
        psi0 = psi0 / np.sum(psi0)
        dp1_prev = np.zeros(3)
        dp2_prev = self.weights @ (x0 * psi0)

        for i, t in enumerate(times):
            s = t / self.tau
            if i == 0:
                pos_list.append(p_init.copy())
                continue

            dt_s = (times[i] - times[i-1]) / self.tau
            xs = np.exp(-self.alpha_x * s)
            dist = (xs - self.centers)**2
            psi = np.exp(-0.5 * self.widths * dist)
            psi = psi / np.sum(psi)

            f = self.weights @ (xs * psi)
            exp_term = np.exp(0.5 * self.alpha * s)
            dp1_curr = s * exp_term * f
            dp2_curr = exp_term * f

            p1_accum += 0.5 * (dp1_prev + dp1_curr) * dt_s
            p2_accum += 0.5 * (dp2_prev + dp2_curr) * dt_s

            dp1_prev = dp1_curr
            dp2_prev = dp2_curr

            y1_s = (1.0 + 0.5 * self.alpha * s) * np.exp(-0.5 * self.alpha * s)
            y2_s = s * np.exp(-0.5 * self.alpha * s)

            f_part = y2_s * p1_accum - y1_s * p2_accum
            p_s = y1_s * c1 + y2_s * c2 + p_goal + f_part
            pos_list.append(p_s)

        return np.array(pos_list)

def simulate_joint_path(model, data, tcp_frame_id, trajectory_pos, trajectory_rot, q_init_7dof, q0_ready_7dof, nullspace_gain=0.2236, j1_scale=10.0):
    q = q_init_7dof.copy()
    w_trans_history = []
    joint_limit_violation_max = 0.0
    q6_violation_max = 0.0

    n_steps = len(trajectory_pos)
    dt = 60.9729 / (n_steps - 1)

    kp_pos = 1.0
    kp_rot = 1.0
    damping_sq = 1e-4
    ns_damping_sq = 1e-4

    for i in range(n_steps):
        pos_ee, rot_ee = get_ee_pose(model, data, tcp_frame_id, q)
        J6 = get_jacobian(model, data, tcp_frame_id, q)

        w = compute_w_trans(J6)
        w_trans_history.append(w)

        # Check limits
        for j in range(7):
            min_l, max_l = JOINT_LIMITS[j]
            if q[j] < min_l:
                v = min_l - q[j]
                if v > joint_limit_violation_max:
                    joint_limit_violation_max = v
                if j == 5 and v > q6_violation_max:
                    q6_violation_max = v
            elif q[j] > max_l:
                v = q[j] - max_l
                if v > joint_limit_violation_max:
                    joint_limit_violation_max = v
                if j == 5 and v > q6_violation_max:
                    q6_violation_max = v

        if i == n_steps - 1:
            break

        target_p = trajectory_pos[i+1] # track forward point
        target_R = trajectory_rot[i+1]

        err_pos = target_p - pos_ee
        R_diff = target_R @ rot_ee.T
        trace = np.trace(R_diff)
        cos_theta = np.clip(0.5 * (trace - 1.0), -1.0, 1.0)
        theta = np.arccos(cos_theta)
        if theta < 1e-6:
            err_rot = np.zeros(3)
        else:
            err_rot = (theta / (2.0 * np.sin(theta))) * np.array([
                R_diff[2, 1] - R_diff[1, 2],
                R_diff[0, 2] - R_diff[2, 0],
                R_diff[1, 0] - R_diff[0, 1]
            ])

        twist_desired = np.zeros(6)
        twist_desired[:3] = kp_pos * (err_pos / dt)
        twist_desired[3:] = kp_rot * (err_rot / dt)

        JJt = J6 @ J6.T + damping_sq * np.eye(6)
        J_pinv = J6.T @ np.linalg.inv(JJt)
        dq_primary = J_pinv @ twist_desired

        JJt_ns = J6 @ J6.T + ns_damping_sq * np.eye(6)
        J_pinv_ns = J6.T @ np.linalg.inv(JJt_ns)
        N = np.eye(7) - J_pinv_ns @ J6

        qe = q0_ready_7dof - q
        qe[0] *= j1_scale
        dq_ns = N @ (nullspace_gain * qe)

        dq_cmd = dq_primary + dq_ns
        q = q + dq_cmd * dt

    pos_ee_final, rot_ee_final = get_ee_pose(model, data, tcp_frame_id, q)
    final_pos_err = np.linalg.norm(pos_ee_final - trajectory_pos[-1])

    return {
        "q_final": q,
        "w_trans_final": w_trans_history[-1],
        "joint_limit_violation_max_rad": joint_limit_violation_max,
        "q6_violation_max_rad": q6_violation_max,
        "q_deg": np.degrees(q),
        "q6_final_deg": np.degrees(q[5]),
        "q4_final_deg": np.degrees(q[3]),
        "final_pos_err_mm": final_pos_err * 1000.0,
    }

def main():
    model, data, tcp_frame_id = build_robot()
    q0_ready_7dof = READY_POSE[:7]
    p_init, R_init = get_ee_pose(model, data, tcp_frame_id, q0_ready_7dof)
    q_init_quat = pin.Quaternion(R_init)

    p_grasp_demo = p_init + DEMO_DISPLACEMENT
    p_grasp_body = p_grasp_demo - SAT_CENTER
    R_grasp_demo = R_init.copy()

    prodmp = ProDMPTrajectory(WEIGHTS_PATH)
    phases_deg = list(range(0, 361, 15)) # 25 angles

    print("=" * 140)
    print("DETERMINISTIC 2-DOF GRASP + FREE ROLL ANALYSIS (SATELLITE ROTATION AXIS = X)")
    print(f"Sat Center: {SAT_CENTER.tolist()}, Sat Axis: {SAT_AXIS.tolist()}")
    print(f"p_init: {np.round(p_init, 4).tolist()}, p_grasp_demo: {np.round(p_grasp_demo, 4).tolist()}")
    print("=" * 140)

    results = []

    for phase_deg in phases_deg:
        theta = np.radians(phase_deg)
        R_sat = rot_axis_angle(SAT_AXIS, theta)
        p_goal = SAT_CENTER + R_sat @ p_grasp_body
        R_goal_rigid = R_sat @ R_grasp_demo
        approach_dir = R_goal_rigid[:, 2] # 3rd column is approach axis

        traj_pos = prodmp.generate_positions(p_init, p_goal, num_samples=100)
        n_steps = len(traj_pos)

        # Baseline: psi = 0 (Rigid rotation convention)
        traj_rot_0 = []
        for i in range(n_steps):
            s = i / (n_steps - 1)
            q_s = quat_slerp(q_init_quat, pin.Quaternion(R_goal_rigid), s)
            traj_rot_0.append(q_s.toRotationMatrix())
        res_0 = simulate_joint_path(model, data, tcp_frame_id, traj_pos, traj_rot_0, q0_ready_7dof, q0_ready_7dof)
        feas_0 = (res_0["joint_limit_violation_max_rad"] < 1e-4) and (res_0["final_pos_err_mm"] < 2.0)

        # Free Roll Optimization: search psi in [-180, 180] deg (step 2 deg)
        psi_grid = np.linspace(-180, 180, 181)
        best_psi_deg = 0.0
        min_violation = 1e9
        best_res = None
        feasible_psi_list = []

        for psi_deg in psi_grid:
            psi_rad = np.radians(psi_deg)
            R_goal_psi = R_goal_rigid @ rot_z(psi_rad)
            traj_rot_psi = []
            for i in range(n_steps):
                s = i / (n_steps - 1)
                q_s = quat_slerp(q_init_quat, pin.Quaternion(R_goal_psi), s)
                traj_rot_psi.append(q_s.toRotationMatrix())
            res_psi = simulate_joint_path(model, data, tcp_frame_id, traj_pos, traj_rot_psi, q0_ready_7dof, q0_ready_7dof)
            
            viol = res_psi["joint_limit_violation_max_rad"]
            if viol < 1e-4 and res_psi["final_pos_err_mm"] < 2.0:
                feasible_psi_list.append(psi_deg)

            # Score: prioritize zero violation, then distance of q6 from center of range (107 deg)
            if viol < min_violation:
                min_violation = viol
                best_psi_deg = psi_deg
                best_res = res_psi
            elif abs(viol - min_violation) < 1e-6 and min_violation < 1e-4:
                # If both feasible, pick the one with q6 closest to center of range
                q6_center = (JOINT_LIMITS[5][0] + JOINT_LIMITS[5][1]) / 2.0
                if abs(res_psi["q_final"][5] - q6_center) < abs(best_res["q_final"][5] - q6_center):
                    best_psi_deg = psi_deg
                    best_res = res_psi

        feas_opt = (best_res["joint_limit_violation_max_rad"] < 1e-4) and (best_res["final_pos_err_mm"] < 2.0)
        psi_min_feas = min(feasible_psi_list) if feasible_psi_list else None
        psi_max_feas = max(feasible_psi_list) if feasible_psi_list else None
        psi_width = (psi_max_feas - psi_min_feas) if feasible_psi_list else 0.0

        results.append({
            "phase_deg": phase_deg,
            "approach_dir": approach_dir,
            "res_0": res_0,
            "feas_0": feas_0,
            "best_psi_deg": best_psi_deg,
            "best_res": best_res,
            "feas_opt": feas_opt,
            "psi_min_feas": psi_min_feas,
            "psi_max_feas": psi_max_feas,
            "psi_width": psi_width,
            "num_feasible_samples": len(feasible_psi_list)
        })

        f0_str = "OK" if feas_0 else "FAIL"
        fopt_str = "OK" if feas_opt else "FAIL"
        feas_range_str = f"[{psi_min_feas:+4.0f}°, {psi_max_feas:+4.0f}°] (span {psi_width:3.0f}°)" if psi_min_feas is not None else "NONE"
        app_str = f"[{approach_dir[0]:+.3f}, {approach_dir[1]:+.3f}, {approach_dir[2]:+.3f}]"

        print(f"θ={phase_deg:3d}° | AppDir={app_str} | Base(ψ=0°): Viol={res_0['joint_limit_violation_max_rad']:.3f} q6={res_0['q6_final_deg']:5.1f}° [{f0_str:<4}] | Opt: ψ*={best_psi_deg:+5.0f}° Viol={best_res['joint_limit_violation_max_rad']:.3f} q6={best_res['q6_final_deg']:5.1f}° [{fopt_str:<4}] | Feas ψ range: {feas_range_str}")

    print("\n" + "=" * 140)
    print("DETAILED JOINT ANGLES AT OPTIMAL ROLL ψ*")
    print("=" * 140)
    print(f"{'Phase θ':<8} | {'ψ* (roll)':<10} | {'q1 (°)':<7} {'q2 (°)':<7} {'q3 (°)':<7} {'q4 (°)':<7} {'q5 (°)':<7} {'q6 (°)':<7} {'q7 (°)':<7} | {'w_trans':<8} | {'Feas?'}")
    print("-" * 140)
    for r in results:
        q = r["best_res"]["q_deg"]
        f_str = "OK" if r["feas_opt"] else "FAIL"
        print(f"{r['phase_deg']:3d}°     | {r['best_psi_deg']:+5.0f}°      | {q[0]:+6.1f}  {q[1]:+6.1f}  {q[2]:+6.1f}  {q[3]:+6.1f}  {q[4]:+6.1f}  {q[5]:+6.1f}  {q[6]:+6.1f}  | {r['best_res']['w_trans_final']:6.4f}   | {f_str}")

    # Summary metrics
    n_total = len(results)
    n_feas_0 = sum(1 for r in results if r["feas_0"])
    n_feas_opt = sum(1 for r in results if r["feas_opt"])

    # Range of previously infeasible (195° - 300°)
    infeas_yesterday_phases = [p for p in phases_deg if 195 <= p <= 300]
    res_infeas_yesterday = [r for r in results if r["phase_deg"] in infeas_yesterday_phases]
    
    psi_opt_all = [r["best_psi_deg"] for r in results if r["feas_opt"]]
    psi_opt_infeas_yesterday = [r["best_psi_deg"] for r in res_infeas_yesterday if r["feas_opt"]]

    print("\n" + "=" * 140)
    print("SUMMARY COMPARISON & EXCURSION STATISTICS")
    print("=" * 140)
    print(f"Feasible phases with rigid rotation (ψ=0, axis=x): {n_feas_0} / {n_total} ({n_feas_0/n_total*100:.1f}%)")
    print(f"Feasible phases with FREE ROLL (ψ*, axis=x)       : {n_feas_opt} / {n_total} ({n_feas_opt/n_total*100:.1f}%)")
    print(f"Phases gained by 3rd DOF optimization             : +{n_feas_opt - n_feas_0}")
    if psi_opt_all:
        print(f"Total roll excursion across ALL 0-360° phases: min={min(psi_opt_all):+.1f}°, max={max(psi_opt_all):+.1f}°, Excursion = {max(psi_opt_all) - min(psi_opt_all):.1f}°")
    if psi_opt_infeas_yesterday:
        print(f"Roll excursion in [195°, 300°] (previously failing): min={min(psi_opt_infeas_yesterday):+.1f}°, max={max(psi_opt_infeas_yesterday):+.1f}°, Excursion = {max(psi_opt_infeas_yesterday) - min(psi_opt_infeas_yesterday):.1f}°")

if __name__ == "__main__":
    main()
