import os
import csv
import numpy as np

runs = [
    "reach_task_baseline_velocity_delay1",
    "reach_task_baseline_impedance_kt200_delay1",
    "reach_task_baseline_impedance_kt2000_delay1",
    "reach_task_baseline_replay_prodmp_kt200_delay1",
    "reach_task_baseline_replay_prodmp_kt2000_delay1"
]

data_dir = "/root/thesis_ws/tools/gazebo_cartesian_eval/data"

print("run,n_campioni,t_sim_ultimo_campione_s,tau_nominale_s,frazione_completata,err_pos_ultimo_campione_mm,err_pos_a_tau_mm,troncata")

for r in runs:
    tp_file = os.path.join(data_dir, f"target_aligned_{r}.csv")
    ap_file = os.path.join(data_dir, f"actual_pose_{r}.csv")
    if not os.path.exists(tp_file) or not os.path.exists(ap_file):
        continue
    
    t_tgt, x_t, y_t, z_t = [], [], [], []
    with open(tp_file) as f:
        reader = csv.DictReader(f)
        for row in reader:
            t_tgt.append(float(row["t"]))
            x_t.append(float(row["x"]))
            y_t.append(float(row["y"]))
            z_t.append(float(row["z"]))
            
    t_act, x_a, y_a, z_a = [], [], [], []
    with open(ap_file) as f:
        reader = csv.DictReader(f)
        for row in reader:
            t_act.append(float(row["t"]))
            x_a.append(float(row["x"]))
            y_a.append(float(row["y"]))
            z_a.append(float(row["z"]))
            
    t_tgt = np.array(t_tgt)
    t_act = np.array(t_act)
    pos_t = np.column_stack([x_t, y_t, z_t])
    pos_a = np.column_stack([x_a, y_a, z_a])
    
    # Tau nominale: da definizione canonica, istante in cui target si congela (settle_1e4)
    dist_from_end = np.linalg.norm(pos_t - pos_t[-1], axis=1)
    settle_indices = np.where(dist_from_end > 1e-4)[0]
    tau_nom = t_tgt[settle_indices[-1] + 1] if len(settle_indices) > 0 else t_tgt[-1]
    
    t_last = t_tgt[-1]
    n_pts = len(t_tgt)
    frazione = min(1.0, t_last / tau_nom) if tau_nom > 0 else 1.0
    
    err_last = np.linalg.norm(pos_a[-1] - pos_t[-1]) * 1000.0
    
    idx_tau_tgt = np.argmin(np.abs(t_tgt - tau_nom))
    idx_tau_act = np.argmin(np.abs(t_act - tau_nom))
    err_tau = np.linalg.norm(pos_a[idx_tau_act] - pos_t[idx_tau_tgt]) * 1000.0
    
    troncata = "si" if t_last < 0.98 * tau_nom else "no"
    
    print(f"{r},{n_pts},{t_last:.3f},{tau_nom:.3f},{frazione:.3f},{err_last:.2f},{err_tau:.2f},{troncata}")
