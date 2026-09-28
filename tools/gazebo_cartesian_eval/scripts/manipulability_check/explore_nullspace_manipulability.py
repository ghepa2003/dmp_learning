#!/usr/bin/env python3
"""
Esplorazione cinematica del nullspace per Franka Emika Panda (7-DOF)
su task cartesiano 6D (EE frame fer_hand_tcp).

Per ciascun Goal (Goal 2, Goal 4, Goal 1):
1. Estrae la configurazione q_orig raggiunta al clamp (t_clamp) dal run originale.
2. Calcola la posa cartesiana target 6D X_des = (p_des, R_des) tramite cinematica diretta (FK).
3. Esplora la curva di self-motion 1D (varietà di nullspace) con integrazione continua
   lungo il vettore di kernel v_null(q) = ker(J(q)) e correzione Newton-Raphson ad alta precisione
   (errore posizionale < 1e-6 m, errore orientamento < 1e-6 rad), nel rispetto dei limiti di giunto.
4. Esegue inoltre una minimizzazione non lineare vincolata (SLSQP) per verificare globalmente l'ottimo.
"""

import os
import csv
import math
import bisect
import numpy as np
import pinocchio as pin
from scipy.optimize import minimize

eval_root = "/root/thesis_ws/tools/gazebo_cartesian_eval"
data_dir = os.path.join(eval_root, "data")
urdf_path = "/root/thesis_ws/fer_flat_effort.urdf"

model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()
frame_id = model.getFrameId("fer_hand_tcp")
JOINT_NAMES = [f"fer_joint{i}" for i in range(1, 8)]


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
    shape trace_nullspace()'s SVD null-space step and Newton-Raphson IK correction expect.
    This one is NOT just a shape fix: with the extra (structurally zero) finger-joint columns
    left in, J's null space becomes 3D (1 true arm self-motion direction + 2 trivial
    finger-only directions) instead of 1D, and SVD offers no guarantee of which of the 3
    degenerate (zero singular value) directions ends up as Vt[-1, :] - the traced curve could
    silently start following a finger's own DOF instead of the arm's self-motion manifold."""
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
# Position limits restricted to the 7 arm joints, in JOINT_NAMES order (Q_INDEX), so shapes
# stay compatible with the 7-dim q this script works with - model.lowerPositionLimit/
# upperPositionLimit are now 9-dim (with hand:=true) and NOT usable as-is against a 7-dim q.
q_min = model.lowerPositionLimit[Q_INDEX]
q_max = model.upperPositionLimit[Q_INDEX]

runs = [
    ("reach_task_goal_2_prodmp", "Goal 2"),
    ("reach_task_goal_4_prodmp", "Goal 4"),
    ("reach_task_goal_1_prodmp", "Goal 1"),
]


def find_clamp_time(target_path, tol_m=1e-4):
    t_list, x_list, y_list, z_list = [], [], [], []
    with open(target_path, "r") as f:
        for row in csv.DictReader(f):
            t_list.append(float(row["t"]))
            x_list.append(float(row["x"]))
            y_list.append(float(row["y"]))
            z_list.append(float(row["z"]))
    n = len(t_list)
    clamp_idx = 0
    final_pos = (x_list[-1], y_list[-1], z_list[-1])
    for i in range(n - 1, -1, -1):
        d = math.dist((x_list[i], y_list[i], z_list[i]), final_pos)
        if d > tol_m:
            clamp_idx = i + 1
            break
    return t_list[clamp_idx]


def get_q_at_time(js_path, t_target):
    t_list, q_list = [], []
    with open(js_path, "r") as f:
        for row in csv.DictReader(f):
            t_list.append(float(row["t"]))
            q_list.append([float(row[jn]) for jn in JOINT_NAMES])
    idx = bisect.bisect_left(t_list, t_target)
    idx = min(max(idx, 0), len(t_list) - 1)
    return np.array(q_list[idx])


def calc_w_trans(q):
    q_full = pad_q(q)
    pin.computeJointJacobians(model, data, q_full)
    pin.framesForwardKinematics(model, data, q_full)
    J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    Jp = J[:3, :]
    return math.sqrt(max(0.0, np.linalg.det(Jp @ Jp.T)))


def calc_w_rot(q):
    q_full = pad_q(q)
    pin.computeJointJacobians(model, data, q_full)
    pin.framesForwardKinematics(model, data, q_full)
    J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    Jr = J[3:, :]
    return math.sqrt(max(0.0, np.linalg.det(Jr @ Jr.T)))


def get_pose(q):
    pin.forwardKinematics(model, data, pad_q(q))
    pin.updateFramePlacements(model, data)
    return data.oMf[frame_id].copy()


def pose_error_6d(q, target_pose):
    pin.forwardKinematics(model, data, pad_q(q))
    pin.updateFramePlacements(model, data)
    current_pose = data.oMf[frame_id]
    dM = target_pose.actInv(current_pose)
    err = pin.log6(dM).vector  # 6D error in local frame
    return err


def trace_nullspace(q_init, target_pose, step_size=0.005, max_steps=2000):
    manifold_points = [(q_init.copy(), calc_w_trans(q_init))]

    for direction in [+1, -1]:
        q_curr = q_init.copy()
        v_prev = None
        for step in range(max_steps):
            pin.computeJointJacobians(model, data, pad_q(q_curr))
            pin.framesForwardKinematics(model, data, pad_q(q_curr))
            # getFrameJacobian returns 6 x model.nv (9 with hand:=true); slice back to the 7
            # arm-DOF columns via V_INDEX (mirroring RobotModel::update's Jacobian column
            # extraction) BEFORE the SVD below - see build_v_index()'s doc comment for why
            # this is required for correctness (not just shape), not optional.
            J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL)[:, V_INDEX]

            # SVD to get 1D nullspace vector
            U, S, Vt = np.linalg.svd(J)
            v_null = Vt[-1, :]

            # Keep orientation of nullspace tangent consistent
            if v_prev is not None and np.dot(v_null, v_prev) < 0:
                v_null = -v_null
            v_prev = v_null.copy()

            # Step in nullspace
            q_next = q_curr + direction * step_size * v_null

            # Project back onto exact pose manifold using damped Newton-Raphson IK
            ik_ok = False
            for it in range(15):
                err = pose_error_6d(q_next, target_pose)
                if np.linalg.norm(err) < 1e-7:
                    ik_ok = True
                    break
                pin.computeJointJacobians(model, data, pad_q(q_next))
                pin.framesForwardKinematics(model, data, pad_q(q_next))
                # Same V_INDEX restriction as above: J_pinv must map a 6D pose error back to
                # a 7-dim dq, not model.nv=9.
                J_next = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL)[:, V_INDEX]
                J_pinv = np.linalg.pinv(J_next)
                q_next = q_next - J_pinv @ err

            # Check joint limits
            if not ik_ok or np.any(q_next < (q_min + 1e-3)) or np.any(q_next > (q_max - 1e-3)):
                break

            w_val = calc_w_trans(q_next)
            manifold_points.append((q_next.copy(), w_val))
            q_curr = q_next.copy()

    return manifold_points


def optimize_slsqp(q_init, target_pose):
    def objective(q):
        return -calc_w_trans(q)

    def constraint_eq(q):
        return pose_error_6d(q, target_pose)

    bounds = [(q_min[i], q_max[i]) for i in range(7)]
    constraints = [{'type': 'eq', 'fun': constraint_eq}]

    res = minimize(
        objective, q_init, method='SLSQP',
        bounds=bounds, constraints=constraints,
        options={'ftol': 1e-7, 'maxiter': 300}
    )
    return res


def main():
    print("=" * 110)
    print("ESPLORAZIONE CINEMATICA DEL NULLSPACE (TASK 6D EE fer_hand_tcp)")
    print("=" * 110)

    summary_rows = []

    for r_key, label in runs:
        tp = os.path.join(data_dir, f"target_aligned_{r_key}.csv")
        jp = os.path.join(data_dir, f"joint_states_{r_key}.csv")
        t_clamp = find_clamp_time(tp)
        q_orig = get_q_at_time(jp, t_clamp)
        target_pose = get_pose(q_orig)
        w_orig = calc_w_trans(q_orig)
        w_rot_orig = calc_w_rot(q_orig)

        # 1. Trace 1D continuous manifold
        pts = trace_nullspace(q_orig, target_pose)
        w_vals = [p[1] for p in pts]
        best_idx = np.argmax(w_vals)
        q_best_trace, w_best_trace = pts[best_idx]
        dist_trace = np.linalg.norm(q_best_trace - q_orig)

        # 2. SLSQP optimization
        res_opt = optimize_slsqp(q_best_trace, target_pose)
        if res_opt.success and -res_opt.fun > w_best_trace:
            q_best = res_opt.x
            w_best = -res_opt.fun
        else:
            q_best = q_best_trace
            w_best = w_best_trace

        dist_rad = np.linalg.norm(q_best - q_orig)
        delta_pct = ((w_best - w_orig) / w_orig) * 100.0
        delta_abs = w_best - w_orig

        err_best = pose_error_6d(q_best, target_pose)
        pos_err_mm = np.linalg.norm(err_best[:3]) * 1000.0
        rot_err_deg = np.linalg.norm(err_best[3:]) * 180.0 / math.pi

        summary_rows.append({
            "goal": label,
            "w_orig": w_orig,
            "w_max": w_best,
            "delta_abs": delta_abs,
            "delta_pct": delta_pct,
            "dist_rad": dist_rad,
            "q_orig": q_orig,
            "q_best": q_best,
            "pos_err_mm": pos_err_mm,
            "rot_err_deg": rot_err_deg,
            "w_min_manifold": min(w_vals),
            "w_max_manifold": max(w_vals),
            "n_samples": len(pts),
        })

    print(f"{'Goal':<10} | {'w_trans orig':<14} | {'w_trans max (nullspace)':<24} | {'Δw_trans (assoluto)':<20} | {'Δw_trans (%)':<14} | {'Distanza q [rad]':<16}")
    print("-" * 110)
    for row in summary_rows:
        print(f"{row['goal']:<10} | {row['w_orig']:<14.5f} | {row['w_max']:<24.5f} | {row['delta_abs']:>+19.5f} | {row['delta_pct']:>+12.2f}% | {row['dist_rad']:>14.4f} rad")
    print("=" * 110)

    print("\n--- DETTAGLI DELLE CONFIGURAZIONI DI GIUNTO ---")
    for row in summary_rows:
        print(f"\n{row['goal']}:")
        print(f"  - q_orig: {[round(float(x), 4) for x in row['q_orig']]}")
        print(f"  - q_best: {[round(float(x), 4) for x in row['q_best']]}")
        print(f"  - Delta q (giunto per giunto) [rad]: {[round(float(b - a), 4) for a, b in zip(row['q_orig'], row['q_best'])]}")
        print(f"  - Norma spostamento: {row['dist_rad']:.4f} rad ({row['dist_rad']*180.0/math.pi:.2f} deg)")
        print(f"  - Range w_trans lungo l'intera varietà di nullspace: [{row['w_min_manifold']:.5f}, {row['w_max_manifold']:.5f}] (campionati {row['n_samples']} punti)")
        print(f"  - Errore residuo pose rispetto a target: posizione = {row['pos_err_mm']:.4e} mm, orientamento = {row['rot_err_deg']:.4e} deg")


if __name__ == "__main__":
    main()
