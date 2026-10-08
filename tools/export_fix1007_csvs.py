import sqlite3
import glob
import os
import csv
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

bag_dir = "/root/thesis_ws/tools/gazebo_cartesian_eval/bags/reach_task_baseline_impedance_kt200_delay1_fix1007"
out_dir = "/root/thesis_ws/tools/gazebo_cartesian_eval/data"
db_path = glob.glob(f"{bag_dir}/*.db3")[0]
conn = sqlite3.connect(db_path)
cur = conn.cursor()

cur.execute("SELECT id, name FROM topics")
topics = dict(cur.fetchall())
topics_rev = {v: k for k, v in topics.items()}

# 1. Target pose aligned
cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC", (topics_rev["/cartesian_impedance_controller/target_pose_aligned"],))
rows = cur.fetchall()
t0 = rows[0][0] * 1e-9

with open(os.path.join(out_dir, "target_aligned_reach_task_baseline_impedance_kt200_delay1_fix1007.csv"), "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["t", "x", "y", "z", "qw", "qx", "qy", "qz"])
    for ts_ns, data in rows:
        p = deserialize_message(data, PoseStamped)
        t = ts_ns * 1e-9 - t0
        writer.writerow([f"{t:.6f}", p.pose.position.x, p.pose.position.y, p.pose.position.z, p.pose.orientation.w, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z])

# 2. Actual pose
cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC", (topics_rev["/cartesian_impedance_controller/actual_pose"],))
rows = cur.fetchall()

with open(os.path.join(out_dir, "actual_pose_reach_task_baseline_impedance_kt200_delay1_fix1007.csv"), "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["t", "x", "y", "z", "qw", "qx", "qy", "qz"])
    for ts_ns, data in rows:
        p = deserialize_message(data, PoseStamped)
        t = ts_ns * 1e-9 - t0
        writer.writerow([f"{t:.6f}", p.pose.position.x, p.pose.position.y, p.pose.position.z, p.pose.orientation.w, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z])

# 3. Joint states
cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC", (topics_rev["/joint_states"],))
rows = cur.fetchall()
joint_names = [f"fer_joint{i+1}" for i in range(7)]

with open(os.path.join(out_dir, "joint_states_reach_task_baseline_impedance_kt200_delay1_fix1007.csv"), "w", newline="") as f:
    writer = csv.writer(f)
    header = ["t"] + [f"q{i+1}" for i in range(7)] + [f"dq{i+1}" for i in range(7)] + [f"effort{i+1}" for i in range(7)]
    writer.writerow(header)
    for ts_ns, data in rows:
        js = deserialize_message(data, JointState)
        t = ts_ns * 1e-9 - t0
        q, dq, eff = [], [], []
        for jn in joint_names:
            if jn in js.name:
                idx = js.name.index(jn)
                q.append(js.position[idx])
                dq.append(js.velocity[idx] if len(js.velocity) > idx else 0.0)
                eff.append(js.effort[idx] if len(js.effort) > idx else 0.0)
            else:
                q.append(0.0)
                dq.append(0.0)
                eff.append(0.0)
        writer.writerow([f"{t:.6f}"] + q + dq + eff)

conn.close()
print("CSVs exported successfully.")
