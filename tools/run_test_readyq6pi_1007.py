#!/usr/bin/env python3
"""
run_test_readyq6pi_1007.py
Test di 30s simulati con FRANKA_START_POSE=ready_q6pi, nessun target.
Verifica assestamento (t >= 5s e |v_TCP| < 0.01 mm/s per 2s)
e stato di equilibrio vs valori attesi misurati il 07/10.
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

OUT_DIR = os.path.join(WS_ROOT, "readyq6pi_1007")
BAG_DIR = os.path.join(OUT_DIR, "bag_readyq6pi_1007")
CSV_PATH = os.path.join(OUT_DIR, "records_readyq6pi_1007.csv")


def rpy_to_mat(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def tf_mat(xyz, rpy, axis, angle):
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
    T01 = tf_mat([0, 0, 0.333], [0, 0, 0], [0, 0, 1], q[0])
    T12 = tf_mat([0, 0, 0], [-pi/2, 0, 0], [0, 0, 1], q[1])
    T23 = tf_mat([0, -0.316, 0], [pi/2, 0, 0], [0, 0, 1], q[2])
    T34 = tf_mat([0.0825, 0, 0], [pi/2, 0, 0], [0, 0, 1], q[3])
    T45 = tf_mat([-0.0825, 0.384, 0], [-pi/2, 0, 0], [0, 0, 1], q[4])
    T56 = tf_mat([0, 0, 0], [pi/2, 0, 0], [0, 0, 1], q[5])
    T67 = tf_mat([0.088, 0, 0], [pi/2, 0, 0], [0, 0, 1], q[6])
    T78 = fixed_tf([0, 0, 0.107], [0, 0, 0])
    T8h = fixed_tf([0, 0, 0], [0, 0, -pi/4])
    Thtcp = fixed_tf([0, 0, 0.1034], [0, 0, 0])
    T = T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T67 @ T78 @ T8h @ Thtcp
    return T[:3, 3], T[:3, :3]


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


def extract_joint_and_tcp_records(bag_dir, csv_out_path):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import JointState
    from rosgraph_msgs.msg import Clock
    from geometry_msgs.msg import PoseStamped

    db_files = glob.glob(os.path.join(bag_dir, "*.db3"))
    if not db_files:
        print(f"[ERRORE] Nessun file .db3 trovato in {bag_dir}")
        return None

    joint_names = ["fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4", "fer_joint5", "fer_joint6", "fer_joint7"]

    conn = sqlite3.connect(db_files[0])
    cur = conn.cursor()
    cur.execute("SELECT id, name FROM topics")
    topics = dict(cur.fetchall())
    topics_rev = {v: k for k, v in topics.items()}

    # Clock
    clock_data = []
    if "/clock" in topics_rev:
        cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC",
                    (topics_rev["/clock"],))
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

    if "/joint_states" not in topics_rev:
        print(f"[ERRORE] /joint_states non trovato in {bag_dir}")
        return None

    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC",
                (topics_rev["/joint_states"],))
    rows = cur.fetchall()
    if not rows:
        print(f"[ERRORE] Nessun messaggio in /joint_states per {bag_dir}")
        return None

    t_wall_0 = rows[0][0] * 1e-9
    sim_0 = get_sim_time(t_wall_0)

    records = []
    for ts_ns, raw_msg in rows:
        js = deserialize_message(raw_msg, JointState)
        name_to_pos = dict(zip(js.name, js.position))
        q = [name_to_pos.get(jn, 0.0) for jn in joint_names]
        pos, rot = forward_kinematics_tcp(q)

        t_wall = ts_ns * 1e-9 - t_wall_0
        t_sim = get_sim_time(ts_ns * 1e-9) - sim_0

        records.append({
            "t_sim": t_sim,
            "t_wall": t_wall,
            "q": q,
            "x": float(pos[0]), "y": float(pos[1]), "z": float(pos[2]),
            "R": rot.tolist()
        })

    with open(csv_out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_sim", "t_wall", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "x", "y", "z"])
        for r in records:
            writer.writerow([
                f"{r['t_sim']:.6f}",
                f"{r['t_wall']:.6f}",
                *[f"{val:.6f}" for val in r["q"]],
                f"{r['x']:.6f}",
                f"{r['y']:.6f}",
                f"{r['z']:.6f}",
            ])

    print(f"Salvato CSV: {csv_out_path} ({len(records)} campioni)")
    return records


def analyze_records(records, target_sim=30.0):
    t_sim = np.array([r["t_sim"] for r in records])
    t_wall = np.array([r["t_wall"] for r in records])
    q_arr = np.array([r["q"] for r in records])
    pos_arr = np.array([[r["x"], r["y"], r["z"]] for r in records])

    # Gap analysis
    dt_sim = np.diff(t_sim)
    valid_dt = dt_sim > 1e-6
    median_dt = float(np.median(dt_sim[valid_dt]))
    gaps_count = int(np.sum(dt_sim > 3 * median_dt))

    # Total times
    total_sim = float(t_sim[-1] - t_sim[0])
    total_wall = float(t_wall[-1] - t_wall[0])
    rtf = total_sim / total_wall if total_wall > 0 else 0.0
    completed_frac = total_sim / target_sim

    # Velocity |v_TCP| (mm/s)
    dpos = np.diff(pos_arr, axis=0)
    vel_norm_mms = np.zeros(len(pos_arr))
    vel_norm_mms[1:][valid_dt] = (np.linalg.norm(dpos[valid_dt], axis=1) / dt_sim[valid_dt]) * 1000.0

    # Settle criterion: t >= 5.0s and |v_TCP| < 0.01 mm/s for 2s continuously
    # Window of 2.0s
    settle_t = None
    window_duration = 2.0
    for i in range(len(t_sim)):
        t_i = t_sim[i]
        if t_i < 5.0:
            continue
        idx_end = np.where((t_sim >= t_i) & (t_sim <= t_i + window_duration))[0]
        if len(idx_end) > 0 and (t_sim[idx_end[-1]] - t_i) >= 1.95:
            # Check displacement rate over window
            pts = pos_arr[idx_end]
            disp_mm = np.linalg.norm(pts - pts[0], axis=1) * 1000.0
            max_disp_rate = np.max(disp_mm) / (t_sim[idx_end[-1]] - t_i)
            if max_disp_rate < 0.01:
                settle_t = float(t_i)
                break

    if settle_t is not None:
        ss_mask = t_sim >= settle_t
        settle_str = f"t = {settle_t:.3f} s"
    else:
        ss_mask = t_sim >= 5.0
        settle_str = "NON TROVATO (usati campioni t >= 5.0 s)"

    q_ss = q_arr[ss_mask]
    pos_ss = pos_arr[ss_mask]

    q_mean = np.mean(q_ss, axis=0)
    q_std = np.std(q_ss, axis=0)

    p_mean = np.mean(pos_ss, axis=0)
    p_std = np.std(pos_ss, axis=0)

    # Expected values measured on 07/10:
    # joints [0, -0.7866, 0, -2.3789, 0, 3.1416, 0.7854] entro 0.002 rad
    # TCP (0.4325, 0, 0.7722) entro 1 mm
    q_exp = np.array([0.0, -0.7866, 0.0, -2.3789, 0.0, 3.1416, 0.7854])
    p_exp = np.array([0.4325, 0.0, 0.7722])

    delta_q = np.abs(q_mean - q_exp)
    delta_p_mm = np.abs(p_mean - p_exp) * 1000.0
    delta_p_norm_mm = np.linalg.norm(p_mean - p_exp) * 1000.0

    tol_q = 0.002
    tol_p_mm = 1.0

    q_pass = np.all(delta_q <= tol_q)
    p_pass = delta_p_norm_mm <= tol_p_mm

    print("\n=======================================================")
    print("RISULTATI RUN _readyq6pi_1007")
    print("=======================================================")
    print(f"Tempo simulato effettivo: {total_sim:.3f} s (target: {target_sim:.1f} s, frazione: {completed_frac*100:.1f}%)")
    print(f"Tempo wall-clock:         {total_wall:.2f} s")
    print(f"RTF:                      {rtf:.3f}")
    print(f"Buchi dt > 3*dt:          {gaps_count} (dt mediano: {median_dt*1000:.2f} ms)")
    print(f"Assestamento:             {settle_str} (criterio: t >= 5s, |v_TCP| < 0.01 mm/s per 2s)")
    print(f"Campioni steady-state:    {np.sum(ss_mask)} / {len(t_sim)}")
    print("\nSTATISTICHE ALL'EQUILIBRIO (Steady State):")
    print("Joints misurati (mean +- std) [rad]:")
    for j in range(7):
        print(f"  q{j+1}: {q_mean[j]:.6f} +- {q_std[j]:.6f} rad | Atteso: {q_exp[j]:.4f} rad | Delta: {delta_q[j]:.6f} rad (max {tol_q}) -> {'OK' if delta_q[j] <= tol_q else 'FAIL'}")

    print("\nTCP misurato (mean +- std) [m]:")
    print(f"  x: {p_mean[0]:.6f} +- {p_std[0]:.6f} m | Atteso: {p_exp[0]:.4f} m | Delta: {delta_p_mm[0]:.3f} mm")
    print(f"  y: {p_mean[1]:.6f} +- {p_std[1]:.6f} m | Atteso: {p_exp[1]:.4f} m | Delta: {delta_p_mm[1]:.3f} mm")
    print(f"  z: {p_mean[2]:.6f} +- {p_std[2]:.6f} m | Atteso: {p_exp[2]:.4f} m | Delta: {delta_p_mm[2]:.3f} mm")
    print(f"  Norma errore TCP: {delta_p_norm_mm:.3f} mm (max {tol_p_mm} mm) -> {'OK' if p_pass else 'FAIL'}")
    print(f"\nEsito complessivo: Joints {'PASS' if q_pass else 'FAIL'}, TCP {'PASS' if p_pass else 'FAIL'}")


def run_experiment():
    os.makedirs(OUT_DIR, exist_ok=True)
    clean_processes()
    setup_bringup_share()

    if os.path.exists(BAG_DIR):
        shutil.rmtree(BAG_DIR)

    launch_log = os.path.join(OUT_DIR, "launch_readyq6pi_1007.log")
    launch_cmd = (
        f"export PYTHONUNBUFFERED=1 && "
        f"export FRANKA_START_POSE=ready_q6pi && "
        f"source /opt/ros/humble/setup.bash && "
        f"source {FRANKA_WS}/install/setup.bash && "
        f"source {WS_ROOT}/install/setup.bash && "
        f"export QT_QPA_PLATFORM=offscreen && "
        f"stdbuf -oL -eL ros2 launch franka_gazebo_bringup gazebo_cartesian_impedance_control.launch.py "
        f"headless:=true load_gripper:=true"
    )

    print("\n=======================================================")
    print("AVVIO RUN: FRANKA_START_POSE=ready_q6pi (30s sim)")
    print("=======================================================")

    with open(launch_log, "w") as f:
        launch_proc = subprocess.Popen(launch_cmd, shell=True, executable="/bin/bash", stdout=f, stderr=subprocess.STDOUT)

    print("Attesa attivazione controller...")
    if not wait_for_controller(launch_log, 60):
        print("[ERRORE FATALE] Controller non attivato!")
        clean_processes()
        return None

    print("Controller ATTIVO! Avvio registrazione bag...")
    bag_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"ros2 bag record -o {BAG_DIR} "
        f"/joint_states "
        f"/clock"
    )
    bag_proc = subprocess.Popen(bag_cmd, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)

    sim_start = None
    wall_start = time.time()
    while sim_start is None:
        sim_start = get_current_sim_time()
        time.sleep(0.2)

    target_sim = 30.0
    print(f"Misura avviata a sim_time = {sim_start:.3f} s. Target = {target_sim:.1f} s sim.")
    while True:
        now_sim = get_current_sim_time()
        if now_sim is not None:
            elapsed_sim = now_sim - sim_start
            elapsed_wall = time.time() - wall_start
            rtf = elapsed_sim / elapsed_wall if elapsed_wall > 0 else 0
            print(f"\r[ready_q6pi] Sim: {elapsed_sim:.2f}/{target_sim:.1f} s (Wall: {elapsed_wall:.1f} s, RTF: {rtf:.2f})", end="", flush=True)
            if elapsed_sim >= target_sim:
                print(f"\nRaggiunti {elapsed_sim:.2f} s sim! Termine run...")
                break
        time.sleep(0.4)

    if bag_proc.poll() is None:
        bag_proc.send_signal(signal.SIGINT)
        try:
            bag_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            bag_proc.kill()

    clean_processes()

    records = extract_joint_and_tcp_records(BAG_DIR, CSV_PATH)
    if not records:
        print("[ERRORE] Impossibile estrarre record.")
        return None

    analyze_records(records, target_sim)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--analyze-only":
        records = extract_joint_and_tcp_records(BAG_DIR, CSV_PATH)
        if records:
            analyze_records(records, 30.0)
    else:
        run_experiment()
