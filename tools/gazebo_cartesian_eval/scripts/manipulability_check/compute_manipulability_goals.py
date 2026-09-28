#!/usr/bin/env python3
"""
Calcola l'indice di manipolabilità di Yoshikawa per i test di goal-change:
- Goal 2 (3 run: orig, rep2, rep3)
- Goal 4 (3 run: orig, rep2, rep3)
- Goal 1, 3, 5 (run singoli)

Confronta:
- Intervallo di crociera (t in [0.35*t_clamp - 0.5, 0.35*t_clamp + 0.5] s, ovvero attorno al 35% del moto)
- Ultimo secondo prima del clamp (t in [t_clamp - 1.0, t_clamp] s)
"""

import os
import sys
import csv
import math
import numpy as np
import pinocchio as pin

RUNS = [
    ("reach_task_goal_1_prodmp", "Goal 1 (orig)", "Goal 1"),
    ("reach_task_goal_2_prodmp", "Goal 2 (Run 1)", "Goal 2"),
    ("reach_task_goal_2_rep2_prodmp", "Goal 2 (Run 2)", "Goal 2"),
    ("reach_task_goal_2_rep3_prodmp", "Goal 2 (Run 3)", "Goal 2"),
    ("reach_task_goal_3_prodmp", "Goal 3 (orig)", "Goal 3"),
    ("reach_task_goal_4_prodmp", "Goal 4 (Run 1)", "Goal 4"),
    ("reach_task_goal_4_rep2_prodmp", "Goal 4 (Run 2)", "Goal 4"),
    ("reach_task_goal_4_rep3_prodmp", "Goal 4 (Run 3)", "Goal 4"),
    ("reach_task_goal_5_prodmp", "Goal 5 (orig)", "Goal 5"),
]

JOINT_NAMES = [f"fer_joint{i}" for i in range(1, 8)]


def build_q_index(model, joint_names):
    """Resolves each joint name to its Pinocchio q-vector index (model.idx_qs[joint_id]),
    mirroring RobotModel::update's q_index resolution in robot_model.cpp. Needed because
    model.nq (9 with hand:=true: 7 arm joints + fer_finger_joint1/2) no longer matches the
    7 columns in joint_states_*.csv, and because index-by-name (not a hardcoded 0-6 range)
    is required for correct ordering regardless of URDF joint declaration order."""
    return [model.idx_qs[model.getJointId(name)] for name in joint_names]


def find_clamp_time(target_path, tol_m=1e-4):
    """Trova il timestamp t in cui il target si ferma al goal."""
    t_list, x_list, y_list, z_list = [], [], [], []
    with open(target_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t_list.append(float(row["t"]))
            x_list.append(float(row["x"]))
            y_list.append(float(row["y"]))
            z_list.append(float(row["z"]))

    n = len(t_list)
    clamp_idx = 0
    final_pos = (x_list[-1], y_list[-1], z_list[-1])
    for i in range(n - 1, -1, -1):
        d = math.sqrt(
            (x_list[i] - final_pos[0]) ** 2 +
            (y_list[i] - final_pos[1]) ** 2 +
            (z_list[i] - final_pos[2]) ** 2
        )
        if d > tol_m:
            clamp_idx = i + 1
            break

    t_clamp = t_list[clamp_idx]
    return t_clamp, clamp_idx, t_list[-1]


def load_joint_states(js_path):
    t_list = []
    q_list = []
    with open(js_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t_list.append(float(row["t"]))
            q = [float(row[jn]) for jn in JOINT_NAMES]
            q_list.append(q)
    return np.array(t_list), np.array(q_list)


def compute_manipulability_series(model, data, frame_id, q_array, q_index):
    w_full = []
    w_trans = []
    w_rot = []

    for q in q_array:
        # Zero-pad the 7 actuated arm values into Pinocchio's full model.nq configuration
        # vector, at the name-resolved indices in q_index - see build_q_index(). The extra
        # DOFs (gripper finger prismatic joints under hand:=true) stay at 0.0: fer_hand_tcp
        # is reached via a fixed joint upstream of the fingers, so their value does not
        # affect its position.
        q_full = np.zeros(model.nq)
        for i, idx in enumerate(q_index):
            q_full[idx] = q[i]
        pin.computeJointJacobians(model, data, q_full)
        pin.framesForwardKinematics(model, data, q_full)
        J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

        # Full (6x7)
        det_full = np.linalg.det(J @ J.T)
        w_f = math.sqrt(max(0.0, det_full))

        # Translational (3x7)
        J_p = J[:3, :]
        det_trans = np.linalg.det(J_p @ J_p.T)
        w_t = math.sqrt(max(0.0, det_trans))

        # Rotational (3x7)
        J_r = J[3:, :]
        det_rot = np.linalg.det(J_r @ J_r.T)
        w_r = math.sqrt(max(0.0, det_rot))

        w_full.append(w_f)
        w_trans.append(w_t)
        w_rot.append(w_r)

    return np.array(w_full), np.array(w_trans), np.array(w_rot)


def compute_stats(arr):
    if len(arr) == 0:
        return {"mean": float("nan"), "std": float("nan"), "min": float("nan"), "max": float("nan"), "count": 0, "start": float("nan"), "end": float("nan")}
    return {
        "mean": float(np.mean(arr)),
        "std": float(np.std(arr)),
        "min": float(np.min(arr)),
        "max": float(np.max(arr)),
        "count": len(arr),
        "start": float(arr[0]),
        "end": float(arr[-1]),
    }


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    eval_root = os.path.abspath(os.path.join(script_dir, "../.."))
    data_dir = os.path.join(eval_root, "data")
    ws_root = os.path.abspath(os.path.join(eval_root, "../.."))
    urdf_path = os.path.join(ws_root, "fer_flat_effort.urdf")

    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()
    frame_name = "fer_hand_tcp"
    frame_id = model.getFrameId(frame_name)
    q_index = build_q_index(model, JOINT_NAMES)

    results = []

    for run_key, run_label, goal_group in RUNS:
        target_path = os.path.join(data_dir, f"target_aligned_{run_key}.csv")
        js_path = os.path.join(data_dir, f"joint_states_{run_key}.csv")

        if not os.path.exists(target_path) or not os.path.exists(js_path):
            print(f"[WARN] File mancante per {run_key}")
            continue

        t_clamp, clamp_idx, t_max = find_clamp_time(target_path)
        t_js, q_js = load_joint_states(js_path)

        w_full, w_trans, w_rot = compute_manipulability_series(model, data, frame_id, q_js, q_index)

        # Salva serie temporale
        out_csv = os.path.join(data_dir, f"manipulability_{run_key}.csv")
        with open(out_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["t", "w_full", "w_trans", "w_rot"] + JOINT_NAMES)
            for i in range(len(t_js)):
                writer.writerow([t_js[i], w_full[i], w_trans[i], w_rot[i]] + list(q_js[i]))

        # Crociera: intorno a 35% del tempo di clamp (1s window)
        t_mid = 0.35 * t_clamp
        mask_cruise = (t_js >= (t_mid - 0.5)) & (t_js <= (t_mid + 0.5))
        # Pre-clamp: [t_clamp - 1.0, t_clamp]
        mask_preclamp = (t_js >= (t_clamp - 1.0)) & (t_js <= t_clamp)

        res = {
            "key": run_key,
            "label": run_label,
            "group": goal_group,
            "t_clamp": t_clamp,
            "rot_cruise": compute_stats(w_rot[mask_cruise]),
            "rot_preclamp": compute_stats(w_rot[mask_preclamp]),
            "trans_cruise": compute_stats(w_trans[mask_cruise]),
            "trans_preclamp": compute_stats(w_trans[mask_preclamp]),
            "full_cruise": compute_stats(w_full[mask_cruise]),
            "full_preclamp": compute_stats(w_full[mask_preclamp]),
        }
        results.append(res)

    print("\n" + "=" * 115)
    print(f"{'RUN':<25} | {'t_clamp [s]':<11} | {'w_rot Crociera':<18} | {'w_rot Pre-Clamp (1s)':<22} | {'Calo vs Crociera':<17} | {'Trend Ult. Sec':<15}")
    print("=" * 115)

    for r in results:
        rc = r["rot_cruise"]
        rp = r["rot_preclamp"]
        delta_pct = ((rp["mean"] - rc["mean"]) / rc["mean"]) * 100.0 if rc["mean"] > 0 else 0
        trend_sec = ((rp["end"] - rp["start"]) / rp["start"]) * 100.0 if rp["start"] > 0 else 0
        c_str = f"{rc['mean']:.4f} ± {rc['std']:.4f}"
        p_str = f"{rp['mean']:.4f} (min {rp['min']:.4f})"
        print(f"{r['label']:<25} | {r['t_clamp']:<11.2f} | {c_str:<18} | {p_str:<22} | {delta_pct:>+15.2f}% | {trend_sec:>+13.2f}%")
    print("=" * 115)


if __name__ == "__main__":
    main()
