import sqlite3
import glob
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock

bag_dir = "/root/thesis_ws/bags/verify_20s_friction02_1007"
db_path = glob.glob(f"{bag_dir}/*.db3")[0]
conn = sqlite3.connect(db_path)
cur = conn.cursor()

cur.execute("SELECT id FROM topics WHERE name = '/joint_states'")
js_tid = cur.fetchone()[0]

cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC", (js_tid,))
js_rows = cur.fetchall()

cur.execute("SELECT id FROM topics WHERE name = '/clock'")
clk_row = cur.fetchone()
if clk_row:
    clk_tid = clk_row[0]
    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC", (clk_tid,))
    clk_rows = cur.fetchall()
else:
    clk_rows = []

conn.close()

def fk_panda(q):
    DH = [
        (0.0,      0.333,  0.0,     -np.pi/2),
        (0.0,      0.0,    0.0,      np.pi/2),
        (0.0,      0.316,  0.0825,   np.pi/2),
        (0.0,      0.0,   -0.0825,  -np.pi/2),
        (0.0,      0.384,  0.0,      np.pi/2),
        (0.0,      0.0,    0.088,    np.pi/2),
        (0.0,      0.107,  0.0,      0.0)
    ]
    T = np.eye(4)
    for i in range(7):
        theta = q[i]
        d = DH[i][1]
        a = DH[i][2]
        alpha = DH[i][3]
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        Ti = np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,   sa,     ca,    d],
            [0,   0,      0,     1]
        ])
        T = T @ Ti
    T_tcp = T @ np.array([
        [1, 0, 0, 0.0],
        [0, 1, 0, 0.0],
        [0, 0, 1, 0.1034],
        [0, 0, 0, 1.0]
    ])
    return T[:3, 3], T_tcp[:3, 3]

arm_joints = [f"fer_joint{i+1}" for i in range(7)]

t0_wall_ns = js_rows[0][0]
t_1s_ns = t0_wall_ns + int(1.0 * 1e9)

z_tcp_list = []
times_rel = []
max_dq_1s = []
max_dq_all = []

for ts_ns, data in js_rows:
    t_rel = (ts_ns - t0_wall_ns) * 1e-9
    js = deserialize_message(data, JointState)
    q = []
    dq = []
    for jn in arm_joints:
        if jn in js.name:
            idx = js.name.index(jn)
            q.append(js.position[idx])
            dq.append(abs(js.velocity[idx]) if len(js.velocity) > idx else 0.0)
        else:
            q.append(0.0)
            dq.append(0.0)
    
    pos_l8, pos_tcp = fk_panda(q)
    z_tcp_list.append(pos_tcp[2])
    times_rel.append(t_rel)
    
    if ts_ns <= t_1s_ns:
        max_dq_1s.append(max(dq))
    max_dq_all.append(max(dq))

z_t0 = z_tcp_list[0]
z_min = min(z_tcp_list)
z_fine = z_tcp_list[-1]
dq_max_1s_val = max(max_dq_1s) if max_dq_1s else 0.0
dq_max_all_val = max(max_dq_all) if max_dq_all else 0.0

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

# Detect t_primo_moto: first time max_dq > 0.05 rad/s
t_moto = "none"
for i, val in enumerate(max_dq_all):
    if val > 0.05:
        t_moto = f"{times_rel[i]:.3f}"
        break

print(f"Total JS samples: {len(js_rows)}")
print(f"Duration: {times_rel[-1]:.2f} s")
print(f"z_t0: {z_t0:.6f} m")
print(f"z_min: {z_min:.6f} m")
print(f"z_fine: {z_fine:.6f} m")
print(f"dq_max_1s: {dq_max_1s_val:.6f} rad/s")
print(f"dq_max_all: {dq_max_all_val:.6f} rad/s")
print(f"t_primo_moto_s: {t_moto}")
print(f"RTF: {rtf:.4f}")

print("\n--- z(t) progression ---")
for t_tgt in range(0, int(times_rel[-1]) + 1, 2):
    idx = np.argmin(np.abs(np.array(times_rel) - t_tgt))
    print(f"t={times_rel[idx]:5.2f} s: z={z_tcp_list[idx]:.6f} m, max_dq={max_dq_all[idx]:.6f} rad/s")

print("\nCSV row:")
print("test,friction,t_primo_moto_s,z_min_m,z_fine_m,dq_max_1s,rtf")
print(f"verify_20s_friction02,0.2,{t_moto},{z_min:.6f},{z_fine:.6f},{dq_max_1s_val:.6f},{rtf:.4f}")
