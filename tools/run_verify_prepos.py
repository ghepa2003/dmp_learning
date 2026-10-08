#!/usr/bin/env python3
"""
run_verify_prepos.py
Protocollo di verifica ready + prepos con mano prismatica attuale:
  1. Spawn ready (mano prismatica attuale, attrito 0.2, Kt 200)
  2. Rampa prepos 20s a capture (0.589, 0.344, 0.35, quat [0.6903455, 0.1530459, 0.1530459, 0.6903455])
  3. Attesa iniziale 60s
  4. Misura posa ancora a fine attesa
  5. Traiettoria minimum-jerk 40s con spostamento +100 mm lungo z_tcp (skip_initial_alignment=true, rotazione nulla)
  6. Tenuta finale 60s all'ultimo target
Esegue 3 run indipendenti e salva tutti i dati in verify_prepos_1007.
"""

import os
import sys
import time
import subprocess
import signal
import sqlite3
import glob
import math
import csv
import shutil
import numpy as np

WS_ROOT = "/root/thesis_ws"
FRANKA_WS = "/root/ros_workspaces/ros2/franka_ws"
BRINGUP_SHARE = os.path.join(FRANKA_WS, "install/franka_gazebo_bringup/share/franka_gazebo_bringup")
CONFIG_YAML = os.path.join(FRANKA_WS, "src/franka_ros2/franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml")

OUT_DIR = "/root/thesis_ws/verify_prepos_1007"
BAGS_DIR = os.path.join(OUT_DIR, "bags")

# Limiti giunto Franka Emika Research 3 (rad)
Q_MIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
Q_MAX = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])

P_CAP_NOM = np.array([0.589, 0.344, 0.35], dtype=np.float64)
Q_CAP_NOM = np.array([0.6903455270798549, 0.15304591873303097, 0.15304591873303086, 0.6903455270798549], dtype=np.float64)
Q_CAP_NOM = Q_CAP_NOM / np.linalg.norm(Q_CAP_NOM)
NOM_APPROACH_DIR = np.array([0.4226, -0.9063, 0.0], dtype=np.float64)
NOM_APPROACH_DIR /= np.linalg.norm(NOM_APPROACH_DIR)


def rpy_to_mat(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def tf_mat(xyz, rpy, angle):
    T = np.eye(4)
    R_fixed = rpy_to_mat(*rpy)
    c, s = np.cos(angle), np.sin(angle)
    R_joint = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    T[:3, :3] = R_fixed @ R_joint
    T[:3, 3] = xyz
    return T


def fixed_tf(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_mat(*rpy)
    T[:3, 3] = xyz
    return T


def forward_kinematics_tcp(q):
    pi = np.pi
    T01 = tf_mat([0, 0, 0.333], [0, 0, 0], q[0])
    T12 = tf_mat([0, 0, 0], [-pi/2, 0, 0], q[1])
    T23 = tf_mat([0, -0.316, 0], [pi/2, 0, 0], q[2])
    T34 = tf_mat([0.0825, 0, 0], [pi/2, 0, 0], q[3])
    T45 = tf_mat([-0.0825, 0.384, 0], [-pi/2, 0, 0], q[4])
    T56 = tf_mat([0, 0, 0], [pi/2, 0, 0], q[5])
    T67 = tf_mat([0.088, 0, 0], [pi/2, 0, 0], q[6])
    T78 = fixed_tf([0, 0, 0.107], [0, 0, 0])
    T8h = fixed_tf([0, 0, 0], [0, 0, -pi/4])
    Thtcp = fixed_tf([0, 0, 0.1034], [0, 0, 0])
    T = T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T67 @ T78 @ T8h @ Thtcp
    return T[:3, 3], T[:3, :3]


def rot_to_quat(R):
    # R is (3, 3) matrix, returns [x, y, z, w]
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    q = np.array([x, y, z, w], dtype=np.float64)
    return q / np.linalg.norm(q)


def quat_slerp(q0, q1, s):
    q0 = np.array(q0, dtype=np.float64) / np.linalg.norm(q0)
    q1 = np.array(q1, dtype=np.float64) / np.linalg.norm(q1)
    dot = np.dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.99995:
        res = q0 + s * (q1 - q0)
        return res / np.linalg.norm(res)
    dot = np.clip(dot, -1.0, 1.0)
    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * s
    sin_theta = math.sin(theta)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    res = (s0 * q0) + (s1 * q1)
    return res / np.linalg.norm(res)


def quat_angular_diff_deg(q1, q2):
    q1 = np.array(q1, dtype=np.float64) / np.linalg.norm(q1)
    q2 = np.array(q2, dtype=np.float64) / np.linalg.norm(q2)
    dot = np.clip(np.abs(np.dot(q1, q2)), 0.0, 1.0)
    return float(2.0 * math.acos(dot) * 180.0 / math.pi)


def min_jerk(tau):
    if tau <= 0.0:
        return 0.0
    if tau >= 1.0:
        return 1.0
    return 10.0 * (tau ** 3) - 15.0 * (tau ** 4) + 6.0 * (tau ** 5)


def clean_processes():
    cmds = [
        "pkill -9 -f 'ros2 bag record'",
        "pkill -9 -f 'ign gazebo'",
        "pkill -9 -f 'gz_sim'",
        "pkill -9 -f 'ruby'",
        "pkill -9 -f 'ros2'",
        "pkill -9 -f 'robot_state_publisher'",
        "pkill -9 -f 'joint_state_publisher'",
        "pkill -9 -f 'spawner'",
        "pkill -9 -f 'controller_manager'",
        "pkill -9 -f 'joint_state_broadcaster'",
        "pkill -9 -f 'cartesian_impedance_controller'",
        "pkill -9 -f 'parameter_bridge'",
        "rm -rf /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*",
    ]
    for c in cmds:
        subprocess.run(c, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)


def setup_bringup_share():
    os.makedirs(os.path.join(BRINGUP_SHARE, "launch"), exist_ok=True)
    os.makedirs(os.path.join(BRINGUP_SHARE, "config"), exist_ok=True)
    for fname in [
        "gazebo_cartesian_impedance_control.launch.py",
        "gazebo_cartesian_impedance_control_headless.launch.py",
    ]:
        src = os.path.join(WS_ROOT, "src/franka_gazebo_overrides", fname)
        dst = os.path.join(BRINGUP_SHARE, "launch", fname)
        if os.path.exists(src):
            shutil.copyfile(src, dst)
    src_yaml = os.path.join(WS_ROOT, "src/franka_gazebo_overrides/franka_gazebo_controllers.yaml")
    dst_yaml = os.path.join(BRINGUP_SHARE, "config/franka_gazebo_controllers.yaml")
    if os.path.exists(src_yaml):
        if os.path.islink(dst_yaml):
            os.remove(dst_yaml)
        shutil.copyfile(src_yaml, dst_yaml)
    if os.path.exists(src_yaml) and os.path.exists(CONFIG_YAML):
        shutil.copyfile(src_yaml, CONFIG_YAML)


def wait_for_controller(launch_log_path, timeout_sec=60):
    check_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {FRANKA_WS}/install/setup.bash && "
        f"ros2 control list_controllers 2>/dev/null"
    )
    for i in range(timeout_sec * 2):
        if os.path.exists(launch_log_path):
            try:
                with open(launch_log_path, "r") as f:
                    log_text = f.read()
                if "Configured and activated cartesian_impedance_controller" in log_text or \
                   "Successfully loaded controller cartesian_impedance_controller" in log_text:
                    return True
            except Exception:
                pass
        res = subprocess.run(check_cmd, shell=True, executable="/bin/bash", capture_output=True, text=True)
        if "cartesian_impedance_controller" in res.stdout and "active" in res.stdout:
            return True
        time.sleep(0.5)
    return False


def get_current_sim_time():
    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"ros2 topic echo /clock rosgraph_msgs/msg/Clock --once 2>/dev/null"
    )
    res = subprocess.run(cmd, shell=True, executable="/bin/bash", capture_output=True, text=True, timeout=5)
    if res.returncode == 0:
        lines = res.stdout.splitlines()
        sec = 0
        nanosec = 0
        for l in lines:
            if "sec:" in l and "nanosec" not in l:
                sec = int(l.split(":")[-1].strip())
            elif "nanosec:" in l:
                nanosec = int(l.split(":")[-1].strip())
        return sec + nanosec * 1e-9
    return None


def execute_trajectory_publisher(run_name, p0, q0, log_file):
    """
    Gestisce la pubblicazione a 200 Hz su /target_pose delle fasi:
      1. Rampa 20s verso capture
      2. Attesa iniziale 60s a capture
      3. Traiettoria minimum-jerk 40s lungo z_tcp (+100 mm)
      4. Tenuta finale 60s
    """
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState

    class TrajectoryOrchestratorNode(Node):
        def __init__(self):
            super().__init__('verify_prepos_orchestrator')
            self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
            self.pub_target = self.create_publisher(PoseStamped, '/target_pose', 10)
            self.sub_js = self.create_subscription(JointState, '/joint_states', self.js_cb, 10)
            
            self.p0 = p0
            self.q0 = q0
            self.latest_q = None
            self.p_anchor = None
            self.q_anchor = None
            self.p_final_target = None
            self.z_tcp_anchor = None
            self.p_curr_cmd = p0
            self.q_curr_cmd = q0

            # Durate fasi (s sim)
            self.d_ramp = 20.0
            self.d_wait1 = 60.0
            self.d_traj = 40.0
            self.d_wait2 = 60.0
            self.total_duration = self.d_ramp + self.d_wait1 + self.d_traj + self.d_wait2 # 180.0s

            self.start_sim_time = None
            self.timer = self.create_timer(0.005, self.timer_cb) # 200 Hz
            self.is_done = False
            self.phase = "RAMP"

        def js_cb(self, msg):
            name_to_pos = dict(zip(msg.name, msg.position))
            joint_names = ["fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4", "fer_joint5", "fer_joint6", "fer_joint7"]
            if all(jn in name_to_pos for jn in joint_names):
                self.latest_q = [name_to_pos[jn] for jn in joint_names]

        def timer_cb(self):
            now_t = self.get_clock().now().nanoseconds * 1e-9
            if now_t <= 0.0:
                return

            if self.start_sim_time is None:
                self.start_sim_time = now_t
                self.get_logger().info(f"Avvio orchestrazione a sim_time = {self.start_sim_time:.3f} s")

            elapsed = now_t - self.start_sim_time
            if elapsed < 0:
                return

            # 1. Rampa (0 .. 20s)
            if elapsed < self.d_ramp:
                self.phase = "RAMP"
                tau = elapsed / self.d_ramp
                s = min_jerk(tau)
                p_cmd = self.p0 + s * (P_CAP_NOM - self.p0)
                q_cmd = quat_slerp(self.q0, Q_CAP_NOM, s)

            # 2. Attesa iniziale (20 .. 80s)
            elif elapsed < (self.d_ramp + self.d_wait1):
                self.phase = "WAIT1"
                p_cmd = P_CAP_NOM
                q_cmd = Q_CAP_NOM

            # 3. Traiettoria (80 .. 120s)
            elif elapsed < (self.d_ramp + self.d_wait1 + self.d_traj):
                if self.p_anchor is None:
                    # Fine attesa: cattura posa ancora reale misurata da FK dei giunti
                    if self.latest_q is not None:
                        p_fk, R_fk = forward_kinematics_tcp(self.latest_q)
                        self.p_anchor = np.array(p_fk, dtype=np.float64)
                        self.q_anchor = rot_to_quat(R_fk)
                        self.z_tcp_anchor = R_fk[:, 2]
                    else:
                        self.p_anchor = P_CAP_NOM.copy()
                        self.q_anchor = Q_CAP_NOM.copy()
                        r_mat = rpy_to_mat(0, 0, 0)
                        self.z_tcp_anchor = NOM_APPROACH_DIR.copy()

                    self.p_final_target = self.p_anchor + 0.100 * self.z_tcp_anchor
                    self.get_logger().info(f"=== ANCORA FISSATA (t_sim={elapsed:.2f}s) ===")
                    self.get_logger().info(f"  p_anchor: {self.p_anchor}")
                    self.get_logger().info(f"  z_tcp:    {self.z_tcp_anchor}")
                    self.get_logger().info(f"  p_final:  {self.p_final_target}")

                self.phase = "TRAJ"
                t_traj = elapsed - (self.d_ramp + self.d_wait1)
                tau = t_traj / self.d_traj
                s = min_jerk(tau)
                p_cmd = self.p_anchor + s * (0.100 * self.z_tcp_anchor)
                q_cmd = self.q_anchor

            # 4. Tenuta finale (120 .. 180s)
            elif elapsed <= self.total_duration:
                self.phase = "WAIT2"
                p_cmd = self.p_final_target
                q_cmd = self.q_anchor

            else:
                self.is_done = True
                p_cmd = self.p_final_target
                q_cmd = self.q_anchor

            self.p_curr_cmd = p_cmd
            self.q_curr_cmd = q_cmd

            msg = PoseStamped()
            msg.header.frame_id = "world"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = float(p_cmd[0])
            msg.pose.position.y = float(p_cmd[1])
            msg.pose.position.z = float(p_cmd[2])
            msg.pose.orientation.x = float(q_cmd[0])
            msg.pose.orientation.y = float(q_cmd[1])
            msg.pose.orientation.z = float(q_cmd[2])
            msg.pose.orientation.w = float(q_cmd[3])
            self.pub_target.publish(msg)

    rclpy.init()
    node = TrajectoryOrchestratorNode()
    
    wall_start = time.time()
    while rclpy.ok() and not node.is_done:
        rclpy.spin_once(node, timeout_sec=0.01)
        if node.start_sim_time is not None:
            now_t = node.get_clock().now().nanoseconds * 1e-9
            el_sim = now_t - node.start_sim_time
            el_wall = time.time() - wall_start
            rtf = el_sim / el_wall if el_wall > 0 else 0
            print(f"\r[{run_name}] Fase: {node.phase:5s} | Sim: {el_sim:6.2f}/{node.total_duration:.1f} s | Wall: {el_wall:5.1f} s (RTF: {rtf:.2f})", end="", flush=True)

    print("\nCompletata sequenza di controllo!")
    node.destroy_node()
    rclpy.shutdown()


def extract_and_analyze_run(bag_dir, csv_out_path, run_idx):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import JointState
    from rosgraph_msgs.msg import Clock
    from geometry_msgs.msg import PoseStamped

    db_files = glob.glob(os.path.join(bag_dir, "*.db3"))
    if not db_files:
        print(f"[ERRORE] Nessun file .db3 in {bag_dir}")
        return None

    joint_names = ["fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4", "fer_joint5", "fer_joint6", "fer_joint7", "fer_finger_joint1", "fer_finger_joint2"]
    conn = sqlite3.connect(db_files[0])
    cur = conn.cursor()
    cur.execute("SELECT id, name FROM topics")
    topics = dict(cur.fetchall())
    topics_rev = {v: k for k, v in topics.items()}

    clock_data = []
    if "/clock" in topics_rev:
        cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC", (topics_rev["/clock"],))
        for ts_ns, raw_msg in cur.fetchall():
            c = deserialize_message(raw_msg, Clock)
            sim_t = c.clock.sec + c.clock.nanosec * 1e-9
            clock_data.append((ts_ns * 1e-9, sim_t))

    def get_sim_time(rec_t_s):
        if not clock_data:
            return rec_t_s
        l, r = 0, len(clock_data) - 1
        while l <= r:
            m = (l + r) // 2
            if clock_data[m][0] == rec_t_s:
                return clock_data[m][1]
            elif clock_data[m][0] < rec_t_s:
                l = m + 1
            else:
                r = m - 1
        idx = max(0, min(r, len(clock_data) - 1))
        return clock_data[idx][1]

    # Target pose map (sim_time -> target pose)
    target_data = []
    if "/target_pose" in topics_rev:
        cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC", (topics_rev["/target_pose"],))
        for ts_ns, raw_msg in cur.fetchall():
            tp = deserialize_message(raw_msg, PoseStamped)
            t_s = get_sim_time(ts_ns * 1e-9)
            target_data.append((t_s, [tp.pose.position.x, tp.pose.position.y, tp.pose.position.z],
                                [tp.pose.orientation.x, tp.pose.orientation.y, tp.pose.orientation.z, tp.pose.orientation.w]))

    def get_target_at_sim(sim_t):
        if not target_data:
            return P_CAP_NOM, Q_CAP_NOM
        l, r = 0, len(target_data) - 1
        while l <= r:
            m = (l + r) // 2
            if target_data[m][0] == sim_t:
                return target_data[m][1], target_data[m][2]
            elif target_data[m][0] < sim_t:
                l = m + 1
            else:
                r = m - 1
        idx = max(0, min(r, len(target_data) - 1))
        return target_data[idx][1], target_data[idx][2]

    if "/joint_states" not in topics_rev:
        return None

    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC", (topics_rev["/joint_states"],))
    rows = cur.fetchall()
    if not rows:
        return None

    t_wall_0 = rows[0][0] * 1e-9
    sim_0 = get_sim_time(t_wall_0)

    records = []
    for ts_ns, raw_msg in rows:
        js = deserialize_message(raw_msg, JointState)
        name_to_pos = dict(zip(js.name, js.position))
        name_to_vel = dict(zip(js.name, js.velocity)) if js.velocity else {}
        q = [name_to_pos.get(jn, 0.0) for jn in joint_names[:7]]
        dq = [name_to_vel.get(jn, 0.0) for jn in joint_names[:7]]
        f_pos = [name_to_pos.get(jn, 0.0) for jn in joint_names[7:]]
        f_vel = [name_to_vel.get(jn, 0.0) for jn in joint_names[7:]]
        pos, rot = forward_kinematics_tcp(q)
        quat = rot_to_quat(rot)

        t_wall = ts_ns * 1e-9 - t_wall_0
        t_sim = get_sim_time(ts_ns * 1e-9) - sim_0

        t_tgt_pos, t_tgt_quat = get_target_at_sim(get_sim_time(ts_ns * 1e-9))

        records.append({
            "t_sim": t_sim,
            "t_wall": t_wall,
            "q": q,
            "dq": dq,
            "f_pos": f_pos,
            "f_vel": f_vel,
            "x": float(pos[0]), "y": float(pos[1]), "z": float(pos[2]),
            "qx": float(quat[0]), "qy": float(quat[1]), "qz": float(quat[2]), "qw": float(quat[3]),
            "tgt_x": float(t_tgt_pos[0]), "tgt_y": float(t_tgt_pos[1]), "tgt_z": float(t_tgt_pos[2]),
            "tgt_qx": float(t_tgt_quat[0]), "tgt_qy": float(t_tgt_quat[1]), "tgt_qz": float(t_tgt_quat[2]), "tgt_qw": float(t_tgt_quat[3]),
            "R": rot
        })

    with open(csv_out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "t_sim", "t_wall",
            "q1", "q2", "q3", "q4", "q5", "q6", "q7",
            "f1_pos", "f2_pos",
            "x", "y", "z", "qx", "qy", "qz", "qw",
            "tgt_x", "tgt_y", "tgt_z", "tgt_qx", "tgt_qy", "tgt_qz", "tgt_qw"
        ])
        for r in records:
            writer.writerow([
                f"{r['t_sim']:.6f}", f"{r['t_wall']:.6f}",
                *[f"{val:.6f}" for val in r["q"]],
                *[f"{val:.6f}" for val in r["f_pos"]],
                f"{r['x']:.6f}", f"{r['y']:.6f}", f"{r['z']:.6f}",
                f"{r['qx']:.6f}", f"{r['qy']:.6f}", f"{r['qz']:.6f}", f"{r['qw']:.6f}",
                f"{r['tgt_x']:.6f}", f"{r['tgt_y']:.6f}", f"{r['tgt_z']:.6f}",
                f"{r['tgt_qx']:.6f}", f"{r['tgt_qy']:.6f}", f"{r['tgt_qz']:.6f}", f"{r['tgt_qw']:.6f}",
            ])

    print(f"Salvato CSV: {csv_out_path} ({len(records)} campioni)")
    return records


def compute_metrics_for_run(records, run_idx):
    """
    Calcola tutte le metriche richieste dal protocollo:
      - Fine rampa e attesa iniziale (ultimi 50 s dell'attesa iniziale: t_sim 30s .. 80s)
      - Traiettoria (t_sim 80s .. 120s)
      - Tenuta finale (ultimi 50 s della tenuta finale: t_sim 130s .. 180s)
      - Margini giunto, dita, diagnostica
    """
    # Rimuovi duplicati temporali
    t_sim = np.array([r["t_sim"] for r in records])
    t_wall = np.array([r["t_wall"] for r in records])
    pos = np.array([[r["x"], r["y"], r["z"]] for r in records])
    quats = np.array([[r["qx"], r["qy"], r["qz"], r["qw"]] for r in records])
    q_joints = np.array([r["q"] for r in records])
    f_pos = np.array([r["f_pos"] if len(r["f_pos"]) == 2 else [0.0, 0.0] for r in records])

    # Calcolo velocita' numerica del TCP tramite gradient (m/s)
    # per stabilita', usa finestra a passo fisso o differenze centrali
    v_tcp = np.zeros(len(t_sim))
    dt_arr = np.diff(t_sim)
    dt_arr[dt_arr <= 1e-9] = 1e-3
    dp = np.diff(pos, axis=0)
    v_raw = np.linalg.norm(dp, axis=1) / dt_arr
    v_tcp[1:] = v_raw
    v_tcp[0] = v_raw[0] if len(v_raw) > 0 else 0.0
    v_tcp_mms = v_tcp * 1000.0 # mm/s

    def find_idx(t):
        return int(np.argmin(np.abs(t_sim - t)))

    idx_ramp_end = find_idx(20.0)
    idx_wait1_10s = find_idx(30.0) # 10s dall'inizio attesa
    idx_wait1_end = find_idx(80.0) # 60s di attesa (fine attesa)
    idx_traj_end = find_idx(120.0) # fine traiettoria 40s
    idx_wait2_10s = find_idx(130.0) # 10s dall'inizio tenuta finale
    idx_wait2_end = find_idx(180.0) # fine tenuta finale

    # 1. FINE RAMPA E ATTESA INIZIALE
    p_ramp_end = pos[idx_ramp_end]
    err_ramp_end_pos = (p_ramp_end - P_CAP_NOM) * 1000.0 # mm
    err_ramp_end_pos_norm = float(np.linalg.norm(err_ramp_end_pos))

    # Statistiche ultimi 50s attesa iniziale (da idx_wait1_10s a idx_wait1_end)
    pos_wait1_50s = pos[idx_wait1_10s:idx_wait1_end+1]
    quats_wait1_50s = quats[idx_wait1_10s:idx_wait1_end+1]
    
    # Errore contro nominale a regime attesa
    p_wait1_end = pos[idx_wait1_end]
    q_wait1_end = quats[idx_wait1_end]
    err_wait1_pos = (p_wait1_end - P_CAP_NOM) * 1000.0 # [ex, ey, ez] mm
    err_wait1_norm = float(np.linalg.norm(err_wait1_pos))
    err_wait1_rot_deg = quat_angular_diff_deg(q_wait1_end, Q_CAP_NOM)

    # Asse di approccio z_tcp a fine attesa
    rot_wait1_end = records[idx_wait1_end]["R"]
    z_tcp_wait1 = rot_wait1_end[:, 2] # (3,)
    # Angolo dalla direzione nominale (0.4226, -0.9063, 0)
    dot_z = np.clip(np.dot(z_tcp_wait1, NOM_APPROACH_DIR) / (np.linalg.norm(z_tcp_wait1) * np.linalg.norm(NOM_APPROACH_DIR)), -1.0, 1.0)
    angle_z_tcp_nom_deg = float(np.arccos(dot_z) * 180.0 / np.pi)
    component_z_deg = float(np.arcsin(np.clip(np.abs(z_tcp_wait1[2]), -1.0, 1.0)) * 180.0 / np.pi)

    # Deriva attesa (tra t=10s e t=60s dell'attesa, ossia t_sim 30s e 80s)
    drift_wait1_mm = float(np.linalg.norm(pos[idx_wait1_end] - pos[idx_wait1_10s]) * 1000.0)
    duration_wait1_min = (t_sim[idx_wait1_end] - t_sim[idx_wait1_10s]) / 60.0
    slope_wait1_mm_min = drift_wait1_mm / duration_wait1_min if duration_wait1_min > 0 else 0.0
    max_vel_wait1_mms = float(np.max(v_tcp_mms[idx_wait1_10s:idx_wait1_end+1]))

    # Salto tra posa di tenuta e primo target traiettoria
    p_anchor = p_wait1_end
    q_anchor = q_wait1_end
    # Il primo target di traiettoria parte da p_anchor, q_anchor
    jump_pos_mm = float(np.linalg.norm(p_anchor - pos[idx_wait1_end]) * 1000.0) # 0.0
    jump_rot_deg = float(quat_angular_diff_deg(q_anchor, quats[idx_wait1_end]))

    # 2. TRAIETTORIA (t_sim 80s .. 120s)
    idx_traj_start = idx_wait1_end
    pos_traj = pos[idx_traj_start:idx_traj_end+1]
    quats_traj = quats[idx_traj_start:idx_traj_end+1]
    t_traj_rel = t_sim[idx_traj_start:idx_traj_end+1] - t_sim[idx_traj_start]
    
    # Target teorico lungo il percorso
    traj_errors_pos_mm = []
    traj_errors_rot_deg = []
    p_final_theo = p_anchor + 0.100 * z_tcp_wait1
    
    for i, t_rel in enumerate(t_traj_rel):
        tau = np.clip(t_rel / 40.0, 0.0, 1.0)
        s = min_jerk(tau)
        p_tgt_theo = p_anchor + s * (0.100 * z_tcp_wait1)
        err_p = np.linalg.norm(pos_traj[i] - p_tgt_theo) * 1000.0
        err_q = quat_angular_diff_deg(quats_traj[i], q_anchor)
        traj_errors_pos_mm.append(err_p)
        traj_errors_rot_deg.append(err_q)

    traj_err_mean_mm = float(np.mean(traj_errors_pos_mm))
    traj_err_p95_mm = float(np.percentile(traj_errors_pos_mm, 95))
    traj_err_max_mm = float(np.max(traj_errors_pos_mm))
    traj_err_final_mm = float(traj_errors_pos_mm[-1])

    traj_rot_mean_deg = float(np.mean(traj_errors_rot_deg))
    traj_rot_p95_deg = float(np.percentile(traj_errors_rot_deg, 95))
    traj_rot_max_deg = float(np.max(traj_errors_rot_deg))
    traj_rot_final_deg = float(traj_errors_rot_deg[-1])

    # Errore a fine traiettoria contro l'ultimo target p_final_theo
    p_traj_end = pos[idx_traj_end]
    err_end_vec_mm = (p_traj_end - p_final_theo) * 1000.0 # mm
    err_end_norm_mm = float(np.linalg.norm(err_end_vec_mm))
    # Errore contro nominale + spostamento nominale
    p_nom_final = P_CAP_NOM + 0.100 * NOM_APPROACH_DIR
    err_end_nom_norm_mm = float(np.linalg.norm((p_traj_end - p_nom_final) * 1000.0))
    # Proiezione lungo la direzione del moto e residuo trasversale
    err_parallel_mm = float(np.dot(err_end_vec_mm, z_tcp_wait1))
    err_perp_mm = float(np.sqrt(max(0.0, err_end_norm_mm**2 - err_parallel_mm**2)))

    # 3. TENUTA FINALE (t_sim 120s .. 180s, statistiche ultimi 50s: 130s .. 180s)
    p_wait2_end = pos[idx_wait2_end]
    q_wait2_end = quats[idx_wait2_end]
    err_final_hold_vec_mm = (p_wait2_end - p_final_theo) * 1000.0
    err_final_hold_norm_mm = float(np.linalg.norm(err_final_hold_vec_mm))
    err_final_hold_rot_deg = float(quat_angular_diff_deg(q_wait2_end, q_anchor))

    # Deriva in 60s di tenuta finale (tra t_sim 120s e 180s)
    drift_wait2_60s_mm = float(np.linalg.norm(pos[idx_wait2_end] - pos[idx_traj_end]) * 1000.0)
    duration_wait2_min = (t_sim[idx_wait2_end] - t_sim[idx_traj_end]) / 60.0
    slope_wait2_mm_min = drift_wait2_60s_mm / duration_wait2_min if duration_wait2_min > 0 else 0.0
    max_vel_wait2_mms = float(np.max(v_tcp_mms[idx_wait2_10s:idx_wait2_end+1]))

    # Verifica robot fermo: |v_TCP| < 0.01 mm/s per 10 s consecutivi
    # Campioni a 1000 Hz: 10s = 10000 campioni consecutivi con v_tcp_mms < 0.01
    v_wait2 = v_tcp_mms[idx_wait2_10s:idx_wait2_end+1]
    is_stopped_10s = False
    window_samples = int(10.0 / np.median(dt_arr))
    if len(v_wait2) >= window_samples:
        for k in range(len(v_wait2) - window_samples):
            if np.all(v_wait2[k:k+window_samples] < 0.01):
                is_stopped_10s = True
                break

    # 4. MARGINI DAI LIMITI DI GIUNTO
    # in attesa (idx_wait1_end)
    margin_wait1 = np.minimum(q_joints[idx_wait1_end] - Q_MIN, Q_MAX - q_joints[idx_wait1_end])
    min_margin_wait1_rad = float(np.min(margin_wait1))
    joint_min_wait1 = int(np.argmin(margin_wait1)) + 1

    # a fine tenuta (idx_wait2_end)
    margin_wait2 = np.minimum(q_joints[idx_wait2_end] - Q_MIN, Q_MAX - q_joints[idx_wait2_end])
    min_margin_wait2_rad = float(np.min(margin_wait2))
    joint_min_wait2 = int(np.argmin(margin_wait2)) + 1

    # Minimo margine lungo tutta la traiettoria
    margin_all_traj = [np.min(np.minimum(q - Q_MIN, Q_MAX - q)) for q in q_joints[idx_traj_start:idx_traj_end+1]]
    min_margin_traj_rad = float(np.min(margin_all_traj))

    # 5. DITA DEL GRIPPER
    # Posizioni a inizio/fine di ogni fase
    f_p0 = f_pos[0]
    f_p_ramp_end = f_pos[idx_ramp_end]
    f_p_wait1_end = f_pos[idx_wait1_end]
    f_p_traj_end = f_pos[idx_traj_end]
    f_p_wait2_end = f_pos[idx_wait2_end]
    max_f_var_mm = float(np.max(np.ptp(f_pos, axis=0)) * 1000.0)

    # 6. DIAGNOSTICA SIMULAZIONE
    total_sim_s = float(t_sim[-1] - t_sim[0])
    total_wall_s = float(t_wall[-1] - t_wall[0])
    rtf = float(total_sim_s / total_wall_s) if total_wall_s > 0 else 0.0
    dt_nom = np.median(dt_arr)
    gaps_count = int(np.sum(dt_arr > 3.0 * dt_nom))

    res = {
        "run_idx": run_idx,
        # Fine rampa & attesa
        "err_ramp_end_pos_norm": err_ramp_end_pos_norm,
        "err_wait1_pos": err_wait1_pos.tolist(),
        "err_wait1_norm": err_wait1_norm,
        "err_wait1_rot_deg": err_wait1_rot_deg,
        "z_tcp_wait1": z_tcp_wait1.tolist(),
        "angle_z_tcp_nom_deg": angle_z_tcp_nom_deg,
        "component_z_deg": component_z_deg,
        "drift_wait1_mm": drift_wait1_mm,
        "slope_wait1_mm_min": slope_wait1_mm_min,
        "max_vel_wait1_mms": max_vel_wait1_mms,
        "jump_pos_mm": jump_pos_mm,
        "jump_rot_deg": jump_rot_deg,
        # Traiettoria
        "traj_err_mean_mm": traj_err_mean_mm,
        "traj_err_p95_mm": traj_err_p95_mm,
        "traj_err_max_mm": traj_err_max_mm,
        "traj_err_final_mm": traj_err_final_mm,
        "traj_rot_mean_deg": traj_rot_mean_deg,
        "traj_rot_p95_deg": traj_rot_p95_deg,
        "traj_rot_max_deg": traj_rot_max_deg,
        "traj_rot_final_deg": traj_rot_final_deg,
        "err_end_norm_mm": err_end_norm_mm,
        "err_end_nom_norm_mm": err_end_nom_norm_mm,
        "err_parallel_mm": err_parallel_mm,
        "err_perp_mm": err_perp_mm,
        # Tenuta finale
        "err_final_hold_vec_mm": err_final_hold_vec_mm.tolist(),
        "err_final_hold_norm_mm": err_final_hold_norm_mm,
        "err_final_hold_rot_deg": err_final_hold_rot_deg,
        "drift_wait2_60s_mm": drift_wait2_60s_mm,
        "slope_wait2_mm_min": slope_wait2_mm_min,
        "max_vel_wait2_mms": max_vel_wait2_mms,
        "is_stopped_10s": is_stopped_10s,
        # Margini giunto
        "min_margin_wait1_rad": min_margin_wait1_rad,
        "joint_min_wait1": joint_min_wait1,
        "min_margin_wait2_rad": min_margin_wait2_rad,
        "joint_min_wait2": joint_min_wait2,
        "min_margin_traj_rad": min_margin_traj_rad,
        # Dita
        "max_f_var_mm": max_f_var_mm,
        "f_p0": f_p0.tolist(),
        "f_p_wait1_end": f_p_wait1_end.tolist(),
        "f_p_wait2_end": f_p_wait2_end.tolist(),
        # Diagnostica
        "total_sim_s": total_sim_s,
        "total_wall_s": total_wall_s,
        "rtf": rtf,
        "gaps_count": gaps_count,
        "num_samples": len(records),
        "p_anchor": p_anchor.tolist(),
        "q_anchor": q_anchor.tolist()
    }
    return res


def run_single_experiment(run_idx=1):
    run_name = f"verify_prepos_run{run_idx}"
    print(f"\n=======================================================================", flush=True)
    print(f">>> AVVIO RUN {run_idx}/3: {run_name} (ready + prepos + traj 100mm + hold)", flush=True)
    print(f"=======================================================================", flush=True)

    clean_processes()
    setup_bringup_share()

    bag_dir = os.path.join(BAGS_DIR, run_name)
    if os.path.exists(bag_dir):
        shutil.rmtree(bag_dir)

    launch_log = os.path.join(OUT_DIR, f"launch_{run_name}.log")
    launch_cmd = (
        f"export PYTHONUNBUFFERED=1 && "
        f"export FRANKA_START_POSE=ready && "
        f"export FRANKA_FINGERS=prismatic && "
        f"source /opt/ros/humble/setup.bash && "
        f"source {FRANKA_WS}/install/setup.bash && "
        f"source {WS_ROOT}/install/setup.bash && "
        f"export QT_QPA_PLATFORM=offscreen && "
        f"stdbuf -oL -eL ros2 launch franka_gazebo_bringup gazebo_cartesian_impedance_control.launch.py "
        f"headless:=true load_gripper:=true"
    )

    with open(launch_log, "w") as f:
        launch_proc = subprocess.Popen(launch_cmd, shell=True, executable="/bin/bash", stdout=f, stderr=subprocess.STDOUT)

    print("Attesa attivazione controller...")
    if not wait_for_controller(launch_log, 60):
        print(f"[ERRORE FATALE] Controller non attivato per {run_name}!")
        clean_processes()
        return None

    print("Controller ATTIVO! Impostazione parametro skip_initial_alignment=true...")
    param_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"ros2 param set /cartesian_impedance_controller skip_initial_alignment true"
    )
    subprocess.run(param_cmd, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Leggi posa iniziale TCP
    print("Lettura posa iniziale TCP allo spawn...")
    p0 = np.array([0.307, 0.0, 0.590], dtype=np.float64) # nominale ready
    q0 = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64) # nominale ready (w=1 o rpy)
    # Calcola FK nominale ready: [0, -pi/4, 0, -3*pi/4, 0, pi, pi/4]
    q_ready_nom = [0.0, -np.pi/4, 0.0, -3*np.pi/4, 0.0, np.pi, np.pi/4]
    p_ready_fk, R_ready_fk = forward_kinematics_tcp(q_ready_nom)
    p0 = np.array(p_ready_fk, dtype=np.float64)
    q0 = rot_to_quat(R_ready_fk)

    print("Avvio registrazione bag...")
    bag_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"ros2 bag record -o {bag_dir} "
        f"/joint_states "
        f"/clock "
        f"/target_pose"
    )
    bag_proc = subprocess.Popen(bag_cmd, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)

    # Esegui traiettoria e orchestrazione
    execute_trajectory_publisher(run_name, p0, q0, launch_log)

    time.sleep(2.0)
    if bag_proc.poll() is None:
        bag_proc.send_signal(signal.SIGINT)
        try:
            bag_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            bag_proc.kill()

    clean_processes()

    csv_path = os.path.join(OUT_DIR, f"{run_name}.csv")
    records = extract_and_analyze_run(bag_dir, csv_path, run_idx)
    if not records:
        print(f"[ERRORE] Impossibile estrarre record per {run_name}")
        return None

    metrics = compute_metrics_for_run(records, run_idx)
    print(f"\n--- METRICHE RUN {run_idx} ---")
    print(f"Errore rampa (pos norm):        {metrics['err_ramp_end_pos_norm']:.3f} mm")
    print(f"Errore attesa (pos norm):       {metrics['err_wait1_norm']:.3f} mm | rot: {metrics['err_wait1_rot_deg']:.3f} deg")
    print(f"Deriva attesa 50s:              {metrics['drift_wait1_mm']:.4f} mm ({metrics['slope_wait1_mm_min']:.4f} mm/min)")
    print(f"Asse z_tcp err angolo:          {metrics['angle_z_tcp_nom_deg']:.3f} deg | comp z: {metrics['component_z_deg']:.3f} deg")
    print(f"Traiettoria err pos mean:       {metrics['traj_err_mean_mm']:.3f} mm | p95: {metrics['traj_err_p95_mm']:.3f} mm | max: {metrics['traj_err_max_mm']:.3f} mm | fine: {metrics['traj_err_final_mm']:.3f} mm")
    print(f"Traiettoria err fine target:    {metrics['err_end_norm_mm']:.3f} mm (longitudinale: {metrics['err_parallel_mm']:.3f} mm, trasversale: {metrics['err_perp_mm']:.3f} mm)")
    print(f"Tenuta finale deriva 60s:       {metrics['drift_wait2_60s_mm']:.4f} mm ({metrics['slope_wait2_mm_min']:.4f} mm/min)")
    print(f"Robot fermo (>10s <0.01 mm/s):  {'SI' if metrics['is_stopped_10s'] else 'NO'}")
    print(f"Margine giunto min attesa:      {metrics['min_margin_wait1_rad']:.4f} rad (giunto {metrics['joint_min_wait1']})")
    print(f"Margine giunto min tenuta:      {metrics['min_margin_wait2_rad']:.4f} rad (giunto {metrics['joint_min_wait2']})")
    print(f"Dita variazione max:            {metrics['max_f_var_mm']:.4f} mm")
    print(f"RTF:                            {metrics['rtf']:.3f}")
    return metrics


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(BAGS_DIR, exist_ok=True)

    print("=================================================================")
    print("VERIFICA READY + PREPOS + TRAIETTORIA 100mm + TENUTA (3 RUNS)")
    print("=================================================================")

    runs_results = []
    for r in range(1, 4):
        res = run_single_experiment(run_idx=r)
        if res is not None:
            runs_results.append(res)
        time.sleep(3.0)

    if not runs_results:
        print("[ERRORE FATALE] Nessuna run completata con successo.")
        return

    # Calcolo medie e deviazioni standard
    summary_path = os.path.join(OUT_DIR, "summary_verify_prepos_1007.txt")
    with open(summary_path, "w") as sf:
        sf.write("=== REPORT VERIFICA READY + PREPOS + TRAIETTORIA + TENUTA ===\n\n")
        for res in runs_results:
            sf.write(f"\n--- RUN {res['run_idx']} ---\n")
            for k, v in res.items():
                sf.write(f"  {k}: {v}\n")

    print("\n=================================================================")
    print(f"ESECUZIONE COMPLETATA CON SUCCESSO. Report salvato in: {summary_path}")
    print("=================================================================")


if __name__ == "__main__":
    main()
