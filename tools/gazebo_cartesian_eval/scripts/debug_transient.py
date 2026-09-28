#!/usr/bin/env python3
import csv
import math
import numpy as np
import bisect

def load_csv(path):
    t, pos, quat = [], [], []
    with open(path) as f:
        reader = csv.DictReader(f)
        for r in reader:
            t.append(float(r["t"]))
            pos.append([float(r["x"]), float(r["y"]), float(r["z"])])
            quat.append([float(r["qw"]), float(r["qx"]), float(r["qy"]), float(r["qz"])])
    return np.array(t), np.array(pos), np.array(quat)

def quat_angle(q1, q2):
    dot = abs(np.dot(q1, q2))
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.degrees(math.acos(dot))

for run in ["reach_task_baseline_replay_prodmp_kt200_delay1", "reach_task_baseline_replay_prodmp_kt2000_delay1"]:
    print("=" * 70)
    print("RUN:", run)
    print("=" * 70)
    tp_file = f"tools/gazebo_cartesian_eval/data/target_aligned_{run}.csv"
    ap_file = f"tools/gazebo_cartesian_eval/data/actual_pose_{run}.csv"
    
    t_tgt, pos_tgt, q_tgt = load_csv(tp_file)
    t_act, pos_act, q_act = load_csv(ap_file)
    
    print(f"Target count: {len(t_tgt)}, t_0={t_tgt[0]:.6f}, t_end={t_tgt[-1]:.6f}")
    print(f"Actual count: {len(t_act)}, t_0={t_act[0]:.6f}, t_end={t_act[-1]:.6f}")
    
    print("First 5 target samples:")
    for i in range(5):
        print(f"  t={t_tgt[i]:.4f}s: pos={pos_tgt[i]} quat={q_tgt[i]}")
        
    print("First 5 actual samples:")
    for i in range(5):
        print(f"  t={t_act[i]:.4f}s: pos={pos_act[i]} quat={q_act[i]}")
        
    # Match at the exact same physical time
    # Notice: what is actual pose when target FIRST appears?
    t0_tgt = t_tgt[0]
    idx_act_at_t0_tgt = bisect.bisect_left(t_act, t0_tgt)
    idx_act_at_t0_tgt = min(idx_act_at_t0_tgt, len(t_act)-1)
    
    pos_act_at_t0 = pos_act[idx_act_at_t0_tgt]
    pos_tgt_at_t0 = pos_tgt[0]
    diff_at_t0_tgt = np.linalg.norm(pos_act_at_t0 - pos_tgt_at_t0) * 1000.0
    ori_diff_at_t0_tgt = quat_angle(q_act[idx_act_at_t0_tgt], q_tgt[0])
    
    print(f"\n--- AT MOMENT TARGET FIRST PUBLISHED (t={t0_tgt:.4f}s) ---")
    print(f"Target pos: {pos_tgt_at_t0}")
    print(f"Actual pos (at t={t_act[idx_act_at_t0_tgt]:.4f}s): {pos_act_at_t0}")
    print(f"Difference: pos={diff_at_t0_tgt:.3f} mm, ori={ori_diff_at_t0_tgt:.4f} deg")
    
    # Also check target speed in [0.0, 1.0]s
    print(f"\n--- TARGET MOTION IN [0, 1.5]s ---")
    mask = (t_tgt >= t0_tgt) & (t_tgt <= t0_tgt + 1.5)
    t_sub = t_tgt[mask] - t0_tgt
    pos_sub = pos_tgt[mask]
    
    for dt_check in [0.05, 0.1, 0.2, 0.5, 1.0]:
        k = bisect.bisect_left(t_sub, dt_check)
        k = min(k, len(t_sub)-1)
        disp = np.linalg.norm(pos_sub[k] - pos_sub[0]) * 1000.0
        v_tgt = disp / dt_check if dt_check > 0 else 0
        print(f"  dt={dt_check:.2f}s: target displacement from start = {disp:.2f} mm (avg v_tgt = {v_tgt:.2f} mm/s)")
