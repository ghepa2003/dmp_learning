#!/usr/bin/env python3
"""
Calcola l'indice di manipolabilita' di Yoshikawa w(q) = sqrt(det(J(q) * J(q)^T))
per i 4 run Gazebo registrati:
1. reach_task_baseline_velocity_delay1
2. reach_task_baseline_impedance_kt200_delay1
3. reach_task_baseline_impedance_kt2000_delay1
4. reach_task_baseline_replay_prodmp_kt200_delay1

Esegue il confronto tra:
- Intervallo di crociera (t in [20.0, 21.0] s)
- Ultimo secondo prima del clamp del target (t in [t_clamp - 1.0, t_clamp] s)
"""

import os
import sys
import csv
import math
import numpy as np
import pinocchio as pin

RUNS = [
    ("reach_task_baseline_velocity_delay1", "Velocity (delay 1s)"),
    ("reach_task_baseline_impedance_kt200_delay1", "Impedance Kt=200 (delay 1s)"),
    ("reach_task_baseline_impedance_kt2000_delay1", "Impedance Kt=2000 (delay 1s)"),
    ("reach_task_baseline_replay_prodmp_kt200_delay1", "ProDMP Replay Kt=200 (delay 1s)"),
]

JOINT_NAMES = [f"fer_joint{i}" for i in range(1, 8)]


def build_q_index(model, joint_names):
    """Resolves each joint name to its Pinocchio q-vector index (model.idx_qs[joint_id]),
    mirroring RobotModel::update's q_index resolution in robot_model.cpp. Needed because
    model.nq (9 with hand:=true: 7 arm joints + fer_finger_joint1/2) no longer matches the
    7 columns in joint_states_*.csv, and because index-by-name (not a hardcoded 0-6 range)
    is required for correct ordering regardless of URDF joint declaration order."""
    return [model.idx_qs[model.getJointId(name)] for name in joint_names]


def find_clamp_time(target_path, tol_m=1e-5):
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
    for i in range(n - 1, 0, -1):
        d = math.sqrt(
            (x_list[i] - x_list[i - 1]) ** 2 +
            (y_list[i] - y_list[i - 1]) ** 2 +
            (z_list[i] - z_list[i - 1]) ** 2
        )
        if d >= tol_m:
            clamp_idx = i
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
    """Calcola w(q) per tutte le configurazioni q nell'array."""
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
        
        # Manipolabilità spaziale completa (6x7)
        JJt = J @ J.T
        det_full = np.linalg.det(JJt)
        w_f = math.sqrt(max(0.0, det_full))

        # Manipolabilità traslazionale (3x7)
        J_p = J[:3, :]
        det_trans = np.linalg.det(J_p @ J_p.T)
        w_t = math.sqrt(max(0.0, det_trans))

        # Manipolabilità rotazionale (3x7)
        J_r = J[3:, :]
        det_rot = np.linalg.det(J_r @ J_r.T)
        w_r = math.sqrt(max(0.0, det_rot))

        w_full.append(w_f)
        w_trans.append(w_t)
        w_rot.append(w_r)

    return np.array(w_full), np.array(w_trans), np.array(w_rot)


def compute_stats(arr):
    if len(arr) == 0:
        return {"mean": float("nan"), "std": float("nan"), "min": float("nan"), "max": float("nan"), "count": 0}
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

    print(f"Caricamento modello Pinocchio da: {urdf_path}")
    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()
    frame_name = "fer_hand_tcp"
    frame_id = model.getFrameId(frame_name)
    q_index = build_q_index(model, JOINT_NAMES)
    print(f"Modello caricato: {model.name} (nq={model.nq}, nv={model.nv}), frame: {frame_name} (id={frame_id})\n")

    results = []

    for run_key, run_label in RUNS:
        target_path = os.path.join(data_dir, f"target_aligned_{run_key}.csv")
        js_path = os.path.join(data_dir, f"joint_states_{run_key}.csv")

        if not os.path.exists(target_path):
            print(f"[ERRORE] File target non trovato: {target_path}")
            continue
        if not os.path.exists(js_path):
            print(f"[ERRORE] File joint_states non trovato: {js_path}")
            continue

        t_clamp, clamp_idx, t_target_max = find_clamp_time(target_path)
        t_js, q_js = load_joint_states(js_path)

        w_full, w_trans, w_rot = compute_manipulability_series(model, data, frame_id, q_js, q_index)

        # Salva serie temporale manipolabilità
        out_csv = os.path.join(data_dir, f"manipulability_{run_key}.csv")
        with open(out_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["t", "w_full", "w_trans", "w_rot"] + JOINT_NAMES)
            for i in range(len(t_js)):
                writer.writerow([t_js[i], w_full[i], w_trans[i], w_rot[i]] + list(q_js[i]))

        # Finestra crociera: [20.0, 21.0] s
        mask_cruise = (t_js >= 20.0) & (t_js <= 21.0)
        # Finestra pre-clamp: [t_clamp - 1.0, t_clamp] s
        mask_preclamp = (t_js >= (t_clamp - 1.0)) & (t_js <= t_clamp)

        stats_cruise = compute_stats(w_full[mask_cruise])
        stats_preclamp = compute_stats(w_full[mask_preclamp])

        stats_trans_cruise = compute_stats(w_trans[mask_cruise])
        stats_trans_preclamp = compute_stats(w_trans[mask_preclamp])

        res = {
            "key": run_key,
            "label": run_label,
            "t_clamp": t_clamp,
            "t_max": t_js[-1],
            "stats_cruise": stats_cruise,
            "stats_preclamp": stats_preclamp,
            "stats_trans_cruise": stats_trans_cruise,
            "stats_trans_preclamp": stats_trans_preclamp,
        }
        results.append(res)

    print("=" * 105)
    print(f"{'RUN':<40} | {'T_CLAMP (s)':<11} | {'CROCIERA (20-21s) [mean ± std (min..max)]':<32} | {'PRE-CLAMP (clamp-1s..clamp)':<32}")
    print("=" * 105)

    for r in results:
        sc = r["stats_cruise"]
        sp = r["stats_preclamp"]
        c_str = f"{sc['mean']:.4f} ± {sc['std']:.4f} ({sc['min']:.4f}..{sc['max']:.4f})"
        p_str = f"{sp['mean']:.4f} ± {sp['std']:.4f} ({sp['min']:.4f}..{sp['max']:.4f})"
        print(f"{r['label']:<40} | {r['t_clamp']:<11.3f} | {c_str:<32} | {p_str:<32}")
    print("=" * 105)

    print("\n--- DETTAGLIO VARIAZIONE MANIPOLABILITÀ NELL'ULTIMO SECONDO ---")
    for r in results:
        sc = r["stats_cruise"]
        sp = r["stats_preclamp"]
        delta_pct = ((sp['mean'] - sc['mean']) / sc['mean']) * 100.0
        sec_trend = ((sp['end'] - sp['start']) / sp['start']) * 100.0
        print(f"\nRun: {r['label']} (t_clamp = {r['t_clamp']:.3f} s)")
        print(f"  - Crociera w(q):   Media = {sc['mean']:.5f} [Min = {sc['min']:.5f}, Max = {sc['max']:.5f}]")
        print(f"  - Pre-clamp w(q):  Media = {sp['mean']:.5f} [Min = {sp['min']:.5f}, Max = {sp['max']:.5f}]")
        print(f"  - Trend nell'ultimo secondo (da clamp-1s a clamp): {sp['start']:.5f} -> {sp['end']:.5f} ({sec_trend:+.2f}%)")
        print(f"  - Differenza media rispetto a crociera: {delta_pct:+.2f}%")


if __name__ == "__main__":
    main()
