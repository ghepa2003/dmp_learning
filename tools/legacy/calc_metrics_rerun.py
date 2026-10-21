import os
import sys
import csv
import bisect
import math
import glob
import sqlite3
import numpy as np

WS_ROOT = "/root/thesis_ws"
DATA_DIR = os.path.join(WS_ROOT, "tools/gazebo_cartesian_eval/data")
BAGS_DIR = os.path.join(WS_ROOT, "tools/gazebo_cartesian_eval/bags")

run_name = sys.argv[1] if len(sys.argv) > 1 else "reach_task_baseline_replay_prodmp_kt2000_delay1_rerun1007b"

tp_path = os.path.join(DATA_DIR, f"target_aligned_{run_name}.csv")
ap_path = os.path.join(DATA_DIR, f"actual_pose_{run_name}.csv")

t_tgt, pos_tgt, quat_tgt = [], [], []
with open(tp_path) as f:
    reader = csv.DictReader(f)
    for row in reader:
        t_tgt.append(float(row["t"]))
        pos_tgt.append([float(row["x"]), float(row["y"]), float(row["z"])])
        quat_tgt.append([float(row["qw"]), float(row["qx"]), float(row["qy"]), float(row["qz"])])

t_act, pos_act, quat_act = [], [], []
with open(ap_path) as f:
    reader = csv.DictReader(f)
    for row in reader:
        t_act.append(float(row["t"]))
        pos_act.append([float(row["x"]), float(row["y"]), float(row["z"])])
        quat_act.append([float(row["qw"]), float(row["qx"]), float(row["qy"]), float(row["qz"])])

t_tgt = np.array(t_tgt)
t_act = np.array(t_act)
pos_tgt = np.array(pos_tgt)
pos_act = np.array(pos_act)

dts = np.diff(t_tgt)
dt_mediano = np.median(dts)
n_gaps = int(np.sum(dts > 3.0 * dt_mediano))

# Tau nominale: da definizione canonica, istante in cui target si congela (settle_1e4)
dist_from_end = np.linalg.norm(pos_tgt - pos_tgt[-1], axis=1)
settle_indices = np.where(dist_from_end > 1e-4)[0]
tau_nom = t_tgt[settle_indices[-1] + 1] if len(settle_indices) > 0 else t_tgt[-1]
t_sim_ultimo = t_tgt[-1]
frazione = min(1.0, t_sim_ultimo / tau_nom)

pos_errs = []
ori_errs = []
times_matched = []

for i, ta in enumerate(t_act):
    if ta < 0:
        continue
    idx = bisect.bisect_left(t_tgt, ta)
    if idx >= len(t_tgt):
        idx = len(t_tgt) - 1
    dp = np.linalg.norm(pos_act[i] - pos_tgt[idx]) * 1000.0
    
    q1 = quat_act[i]
    q2 = quat_tgt[idx]
    dot = abs(q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3])
    dot = min(1.0, max(-1.0, dot))
    do = 2.0 * math.acos(dot) * 180.0 / math.pi
    
    pos_errs.append(dp)
    ori_errs.append(do)
    times_matched.append(ta)

pos_errs = np.array(pos_errs)
ori_errs = np.array(ori_errs)
times_matched = np.array(times_matched)

z_t0 = pos_act[0][2]
err_t0 = pos_errs[0]

pos_media = np.mean(pos_errs)
pos_mediana = np.median(pos_errs)
pos_p95 = np.percentile(pos_errs, 95)
pos_max = np.max(pos_errs)
idx_max = np.argmax(pos_errs)
t_max = times_matched[idx_max]

mask_1s = times_matched >= 1.0
pos_max_1s = np.max(pos_errs[mask_1s]) if np.any(mask_1s) else pos_max

pos_finale_ultimo = pos_errs[-1]

idx_tau = np.argmin(np.abs(times_matched - tau_nom))
pos_finale_tau = pos_errs[idx_tau]

ori_media = np.mean(ori_errs)
ori_max = np.max(ori_errs)
ori_finale = ori_errs[-1]

# RTF from clock
bag_dir = os.path.join(BAGS_DIR, run_name)
db_path = glob.glob(f"{bag_dir}/*.db3")[0]
conn = sqlite3.connect(db_path)
cur = conn.cursor()
cur.execute("SELECT id FROM topics WHERE name = '/clock'")
clk_row = cur.fetchone()
if clk_row:
    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC", (clk_row[0],))
    c_rows = cur.fetchall()
    from rclpy.serialization import deserialize_message
    from rosgraph_msgs.msg import Clock
    c_first = deserialize_message(c_rows[0][1], Clock).clock
    c_last = deserialize_message(c_rows[-1][1], Clock).clock
    t_sim_dur = (c_last.sec + c_last.nanosec * 1e-9) - (c_first.sec + c_first.nanosec * 1e-9)
    t_wall_dur = (c_rows[-1][0] - c_rows[0][0]) * 1e-9
    rtf = t_sim_dur / t_wall_dur if t_wall_dur > 0 else 1.0
else:
    rtf = 1.0

# Milliseconds between first actual_pose and first target_pose_aligned
cur.execute("SELECT id FROM topics WHERE name = '/cartesian_impedance_controller/actual_pose'")
ap_tid = cur.fetchone()[0]
cur.execute("SELECT timestamp FROM messages WHERE topic_id = ? ORDER BY timestamp ASC LIMIT 1", (ap_tid,))
t_first_act_ns = cur.fetchone()[0]

cur.execute("SELECT id FROM topics WHERE name = '/cartesian_impedance_controller/target_pose_aligned'")
tp_tid = cur.fetchone()[0]
cur.execute("SELECT timestamp FROM messages WHERE topic_id = ? ORDER BY timestamp ASC LIMIT 1", (tp_tid,))
t_first_tgt_ns = cur.fetchone()[0]

dt_ms_first_act_to_tgt = (t_first_tgt_ns - t_first_act_ns) * 1e-6
conn.close()

# Error z at first sample: target_z - actual_z
err_z_t0 = (pos_tgt[0][2] - pos_act[0][2]) * 1000.0

cfg = "ProDMP_impedance_kt2000" if "kt2000" in run_name else "ProDMP_impedance_kt200"

print(f"=== METRICHE PER {run_name} ===")
print(f"tau nominale: {tau_nom:.3f} s")
print(f"t_sim_ultimo: {t_sim_ultimo:.3f} s (frazione = {frazione:.3f})")
print(f"n_campioni: {len(t_tgt)}")
print(f"dt_mediano: {dt_mediano:.6f} s")
print(f"n_gaps_gt3dt: {n_gaps}")
print(f"rtf_medio: {rtf:.4f}")
print(f"z_t0: {z_t0:.6f} m")
print(f"err_t0: {err_t0:.2f} mm")
print(f"pos_media: {pos_media:.2f} mm")
print(f"pos_mediana: {pos_mediana:.2f} mm")
print(f"pos_p95: {pos_p95:.2f} mm")
print(f"pos_max: {pos_max:.2f} mm (a t={t_max:.3f} s)")
print(f"pos_max_dopo_1s: {pos_max_1s:.2f} mm")
print(f"pos_finale_ultimo: {pos_finale_ultimo:.2f} mm")
print(f"pos_finale_a_tau: {pos_finale_tau:.2f} mm")
print(f"ori_media: {ori_media:.3f} deg")
print(f"ori_max: {ori_max:.3f} deg")
print(f"ori_finale: {ori_finale:.3f} deg")
print(f"Delta t primo actual -> primo target: {dt_ms_first_act_to_tgt:.2f} ms")
print(f"Errore z a t0 (target_z - actual_z): {err_z_t0:.2f} mm")

print("\n--- CSV ROW ---")
print("run,config,t_sim_ultimo_s,frazione_completata,n_gaps_gt3dt,rtf_gz_stats,z_t0_m,err_t0_mm,pos_media_mm,pos_mediana_mm,pos_p95_mm,pos_max_mm,pos_max_dopo_1s_mm,pos_finale_ultimo_mm,pos_finale_a_tau_mm,ori_media_deg,ori_max_deg,ori_finale_deg")
print(f"{run_name},{cfg},{t_sim_ultimo:.3f},{frazione:.3f},{n_gaps},{rtf:.4f},{z_t0:.6f},{err_t0:.2f},{pos_media:.2f},{pos_mediana:.2f},{pos_p95:.2f},{pos_max:.2f},{pos_max_1s:.2f},{pos_finale_ultimo:.2f},{pos_finale_tau:.2f},{ori_media:.3f},{ori_max:.3f},{ori_finale:.3f}")
