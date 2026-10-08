import sqlite3
import glob
import numpy as np
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
import math

def quat_diff_deg(q1, q2):
    # q1, q2: [w, x, y, z]
    dot = abs(q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3])
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot) * 180.0 / math.pi

bag_dir = "/root/thesis_ws/tools/gazebo_cartesian_eval/bags/reach_task_baseline_impedance_kt200_delay1_fix1007"
db_path = glob.glob(f"{bag_dir}/*.db3")[0]
conn = sqlite3.connect(db_path)
cur = conn.cursor()

# Get topic ids
cur.execute("SELECT id, name FROM topics")
topics = dict(cur.fetchall())
topics_rev = {v: k for k, v in topics.items()}

# Read actual_pose
cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC", (topics_rev["/cartesian_impedance_controller/actual_pose"],))
actual_rows = cur.fetchall()

# Read target_pose_aligned
cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC", (topics_rev["/cartesian_impedance_controller/target_pose_aligned"],))
target_rows = cur.fetchall()

# Read clock
cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC", (topics_rev["/clock"],))
clk_rows = cur.fetchall()

conn.close()

# RTF
if len(clk_rows) >= 2:
    t_wall_first = clk_rows[0][0] * 1e-9
    t_wall_last = clk_rows[-1][0] * 1e-9
    c_first = deserialize_message(clk_rows[0][1], Clock).clock
    c_last = deserialize_message(clk_rows[-1][1], Clock).clock
    t_sim_first = c_first.sec + c_first.nanosec * 1e-9
    t_sim_last = c_last.sec + c_last.nanosec * 1e-9
    rtf = (t_sim_last - t_sim_first) / (t_wall_last - t_wall_first) if (t_wall_last > t_wall_first) else 1.0
else:
    rtf = 1.0

# Parse target and actual
target_t, target_pos, target_quat = [], [], []
for ts_ns, data in target_rows:
    p = deserialize_message(data, PoseStamped)
    t = p.header.stamp.sec + p.header.stamp.nanosec * 1e-9
    target_t.append(t)
    target_pos.append([p.pose.position.x, p.pose.position.y, p.pose.position.z])
    target_quat.append([p.pose.orientation.w, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z])

actual_t, actual_pos, actual_quat = [], [], []
for ts_ns, data in actual_rows:
    p = deserialize_message(data, PoseStamped)
    t = p.header.stamp.sec + p.header.stamp.nanosec * 1e-9
    actual_t.append(t)
    actual_pos.append([p.pose.position.x, p.pose.position.y, p.pose.position.z])
    actual_quat.append([p.pose.orientation.w, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z])

target_t = np.array(target_t)
actual_t = np.array(actual_t)
target_pos = np.array(target_pos)
actual_pos = np.array(actual_pos)

# Align time to t0 of target
t0_tgt = target_t[0]
target_t_rel = target_t - t0_tgt
actual_t_rel = actual_t - t0_tgt

# Tau duration
tau = target_t_rel[-1]
n_campioni = len(target_t)

# dt and gaps
dts = np.diff(target_t_rel)
dt_mediano = np.median(dts)
n_gaps = int(np.sum(dts > 3.0 * dt_mediano))

# Calculate errors by matching nearest/interpolated or synchronized target/actual
# Controller publishes actual and target at same rate
pos_errs_mm = []
ori_errs_deg = []
times_matched = []

for i, t_act in enumerate(actual_t_rel):
    if t_act < 0:
        continue
    # find nearest target
    idx_tgt = np.searchsorted(target_t_rel, t_act)
    if idx_tgt >= len(target_t_rel):
        idx_tgt = len(target_t_rel) - 1
    
    dp = np.linalg.norm(actual_pos[i] - target_pos[idx_tgt]) * 1000.0
    do = quat_diff_deg(actual_quat[i], target_quat[idx_tgt])
    pos_errs_mm.append(dp)
    ori_errs_deg.append(do)
    times_matched.append(t_act)

pos_errs_mm = np.array(pos_errs_mm)
ori_errs_deg = np.array(ori_errs_deg)
times_matched = np.array(times_matched)

# z at t0 of actual pose
z_t0 = actual_pos[0][2]
err_t0 = pos_errs_mm[0]

pos_media = np.mean(pos_errs_mm)
pos_max = np.max(pos_errs_mm)

# pos_max after 1s
mask_after_1s = times_matched >= 1.0
pos_max_dopo_1s = np.max(pos_errs_mm[mask_after_1s]) if np.any(mask_after_1s) else pos_max
pos_finale = pos_errs_mm[-1]

ori_media = np.mean(ori_errs_deg)
ori_max = np.max(ori_errs_deg)
ori_finale = ori_errs_deg[-1]

run_name = "reach_task_baseline_impedance_kt200_delay1_fix1007"
config = "DMP_impedance_kt200_friction0.2"

print(f"=== METRICHE EVALUATE PER {run_name} ===")
print(f"tau: {tau:.3f} s")
print(f"n_campioni: {n_campioni}")
print(f"dt_mediano: {dt_mediano:.6f} s")
print(f"n_gaps_gt3dt: {n_gaps}")
print(f"rtf_medio: {rtf:.4f}")
print(f"z_t0: {z_t0:.6f} m")
print(f"err_t0: {err_t0:.2f} mm")
print(f"pos_media: {pos_media:.2f} mm")
print(f"pos_max: {pos_max:.2f} mm")
print(f"pos_max_dopo_1s: {pos_max_dopo_1s:.2f} mm")
print(f"pos_finale: {pos_finale:.2f} mm")
print(f"ori_media: {ori_media:.2f} deg")
print(f"ori_max: {ori_max:.2f} deg")
print(f"ori_finale: {ori_finale:.2f} deg")

print("\n--- CSV ROW ---")
print("run,config,tau_s,n_campioni,dt_mediano_s,n_gaps_gt3dt,rtf_medio,z_t0_m,err_t0_mm,pos_media_mm,pos_max_mm,pos_max_dopo_1s_mm,pos_finale_mm,ori_media_deg,ori_max_deg,ori_finale_deg")
print(f"{run_name},{config},{tau:.3f},{n_campioni},{dt_mediano:.6f},{n_gaps},{rtf:.4f},{z_t0:.6f},{err_t0:.2f},{pos_media:.2f},{pos_max:.2f},{pos_max_dopo_1s:.2f},{pos_finale:.2f},{ori_media:.2f},{ori_max:.2f},{ori_finale:.2f}")
