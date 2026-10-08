#!/usr/bin/env python3
"""
run_spawnq6_1007.py
Valutazione dello spawn diretto in orientamento orizzontale (readyPose con q6 = pi).
Esperimenti:
  - S0: Spawn nominale (readyPose), 10 s sim.
  - S1: Spawn con q6 = pi, 60 s sim, 3 ripetizioni indipendenti.
"""

import os
import sys
import time
import subprocess
import signal
import shutil
import sqlite3
import glob
import math
import csv
import numpy as np

WS_ROOT = "/root/thesis_ws"
FRANKA_WS = "/root/ros_workspaces/ros2/franka_ws"
BRINGUP_SHARE = os.path.join(FRANKA_WS, "install/franka_gazebo_bringup/share/franka_gazebo_bringup")
CONFIG_YAML = os.path.join(FRANKA_WS, "src/franka_ros2/franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml")

DESC_SHARE_DIR = os.path.join(FRANKA_WS, "install/franka_description/share/franka_description/robots/common")
ORIG_ARM_XACRO = os.path.join(DESC_SHARE_DIR, "franka_arm.ros2_control.xacro")
ORIG_ARM_XACRO_BAK = os.path.join(DESC_SHARE_DIR, "franka_arm.ros2_control.xacro.bak_spawnq6_1007")

SRC_XACRO_OVERRIDE = os.path.join(WS_ROOT, "src/franka_description_overrides/franka_arm.ros2_control.xacro_spawnq6_1007")

OUT_DIR = os.path.join(WS_ROOT, "spawnq6_1007")
BAGS_DIR = os.path.join(OUT_DIR, "bags")

URDF_PATH = os.path.join(WS_ROOT, "fer_flat_effort.urdf")


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


def setup_xacro_override():
    if not os.path.exists(ORIG_ARM_XACRO_BAK) and os.path.exists(ORIG_ARM_XACRO):
        shutil.copyfile(ORIG_ARM_XACRO, ORIG_ARM_XACRO_BAK)
    shutil.copyfile(SRC_XACRO_OVERRIDE, ORIG_ARM_XACRO)


def restore_xacro_override():
    if os.path.exists(ORIG_ARM_XACRO_BAK):
        shutil.copyfile(ORIG_ARM_XACRO_BAK, ORIG_ARM_XACRO)
        os.remove(ORIG_ARM_XACRO_BAK)


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
    import pinocchio as pin
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import JointState
    from rosgraph_msgs.msg import Clock

    db_files = glob.glob(os.path.join(bag_dir, "*.db3"))
    if not db_files:
        print(f"[ERRORE] Nessun file .db3 trovato in {bag_dir}")
        return None

    model = pin.buildModelFromUrdf(URDF_PATH)
    data = model.createData()
    frame_id = model.getFrameId("fer_hand_tcp")
    joint_names = ["fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4", "fer_joint5", "fer_joint6", "fer_joint7"]
    q_indices = [model.idx_qs[model.getJointId(jn)] for jn in joint_names]

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
        q = np.zeros(model.nq)
        for i, jn in enumerate(joint_names):
            q[q_indices[i]] = name_to_pos[jn]
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        pos = data.oMf[frame_id].translation
        rot = data.oMf[frame_id].rotation
        quat = pin.Quaternion(rot)

        t_wall = ts_ns * 1e-9 - t_wall_0
        t_sim = get_sim_time(ts_ns * 1e-9) - sim_0

        records.append({
            "t_sim": t_sim,
            "t_wall": t_wall,
            "q": [name_to_pos[jn] for jn in joint_names],
            "x": pos[0], "y": pos[1], "z": pos[2],
            "qw": quat.w, "qx": quat.x, "qy": quat.y, "qz": quat.z,
            "R": rot.tolist()
        })

    with open(csv_out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_sim", "t_wall", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "x", "y", "z", "qw", "qx", "qy", "qz"])
        for r in records:
            writer.writerow([
                f"{r['t_sim']:.6f}",
                f"{r['t_wall']:.6f}",
                *[f"{val:.6f}" for val in r["q"]],
                f"{r['x']:.6f}",
                f"{r['y']:.6f}",
                f"{r['z']:.6f}",
                f"{r['qw']:.6f}",
                f"{r['qx']:.6f}",
                f"{r['qy']:.6f}",
                f"{r['qz']:.6f}",
            ])

    print(f"Salvato CSV: {csv_out_path} ({len(records)} campioni)")
    return records


def run_s0():
    print("\n=======================================================")
    print("ESECUZIONE ESPERIMENTO S0: Spawn nominale (readyPose), 10 s sim")
    print("=======================================================")
    clean_processes()
    setup_bringup_share()
    setup_xacro_override()

    bag_dir = os.path.join(BAGS_DIR, "s0_ready_nominal_spawnq6_1007")
    if os.path.exists(bag_dir):
        shutil.rmtree(bag_dir)

    launch_log = "/tmp/launch_s0_spawnq6_1007.log"
    launch_cmd = (
        f"export PYTHONUNBUFFERED=1 && "
        f"export FRANKA_START_POSE=ready && "
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
        print("[ERRORE FATALE] Controller non attivato per S0!")
        clean_processes()
        return None

    print("Controller ATTIVO! Avvio registrazione bag...")
    bag_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"ros2 bag record -o {bag_dir} "
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

    print(f"Misura S0 avviata a sim_time = {sim_start:.3f} s. Target = 10.0 s sim.")
    while True:
        now_sim = get_current_sim_time()
        if now_sim is not None:
            elapsed_sim = now_sim - sim_start
            elapsed_wall = time.time() - wall_start
            rtf = elapsed_sim / elapsed_wall if elapsed_wall > 0 else 0
            print(f"\r[S0] Sim: {elapsed_sim:.2f}/10.00 s (Wall: {elapsed_wall:.1f} s, RTF: {rtf:.2f})", end="", flush=True)
            if elapsed_sim >= 10.0:
                print(f"\nRaggiunti {elapsed_sim:.2f} s sim! Termine S0...")
                break
        time.sleep(0.3)

    if bag_proc.poll() is None:
        bag_proc.send_signal(signal.SIGINT)
        try:
            bag_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            bag_proc.kill()

    clean_processes()

    csv_path = os.path.join(OUT_DIR, "s0_ready_nominal_spawnq6_1007.csv")
    records = extract_joint_and_tcp_records(bag_dir, csv_path)
    if not records:
        return None

    t_sim = np.array([r["t_sim"] for r in records])
    t_wall = np.array([r["t_wall"] for r in records])
    q_arr = np.array([r["q"] for r in records])
    pos_arr = np.array([[r["x"], r["y"], r["z"]] for r in records])

    # Sample indices
    idx_0 = 0
    idx_5s = int(np.argmin(np.abs(t_sim - 5.0)))
    idx_10s = int(np.argmin(np.abs(t_sim - 10.0)))

    q_nom = [0.0, -math.pi/4, 0.0, -3*math.pi/4, 0.0, math.pi/2, math.pi/4]

    dt_sim = np.diff(t_sim)
    valid_dt = dt_sim > 1e-6
    median_dt = np.median(dt_sim[valid_dt])
    gaps_count = int(np.sum(dt_sim > 3 * median_dt))

    res = {
        "q_nominal": q_nom,
        "q_0": q_arr[idx_0].tolist(),
        "p_0": pos_arr[idx_0].tolist(),
        "q_5s": q_arr[idx_5s].tolist(),
        "p_5s": pos_arr[idx_5s].tolist(),
        "q_10s": q_arr[idx_10s].tolist(),
        "p_10s": pos_arr[idx_10s].tolist(),
        "diff_q0_nom_rad": (q_arr[idx_0] - np.array(q_nom)).tolist(),
        "diff_q10s_nom_rad": (q_arr[idx_10s] - np.array(q_nom)).tolist(),
        "total_sim_s": float(t_sim[-1] - t_sim[0]),
        "total_wall_s": float(t_wall[-1] - t_wall[0]),
        "rtf": float((t_sim[-1] - t_sim[0]) / (t_wall[-1] - t_wall[0])),
        "completed_frac": float((t_sim[-1] - t_sim[0]) / 10.0),
        "gaps_gt_3dt": gaps_count,
        "num_samples": len(records),
        "csv_path": csv_path
    }

    print("\n--- RISULTATI S0 ---")
    print("Giunti nominali (rad):  ", [round(v, 4) for v in q_nom])
    print("Giunti t=0s (rad):      ", [round(v, 4) for v in res["q_0"]])
    print("Posa TCP t=0s (m):      ", [round(v, 4) for v in res["p_0"]])
    print("Giunti t=5s (rad):      ", [round(v, 4) for v in res["q_5s"]])
    print("Posa TCP t=5s (m):      ", [round(v, 4) for v in res["p_5s"]])
    print("Giunti t=10s (rad):     ", [round(v, 4) for v in res["q_10s"]])
    print("Posa TCP t=10s (m):     ", [round(v, 4) for v in res["p_10s"]])
    print("Diff (q10s - q_nom) rad:", [round(v, 6) for v in res["diff_q10s_nom_rad"]])
    return res


def run_s1_single(run_idx=1):
    run_name = f"s1_run{run_idx}_q6pi_spawnq6_1007"
    print(f"\n=======================================================")
    print(f"ESECUZIONE ESPERIMENTO S1 (Run {run_idx}/3): Spawn q6=pi (60s sim)")
    print(f"=======================================================")
    clean_processes()
    setup_bringup_share()
    setup_xacro_override()

    bag_dir = os.path.join(BAGS_DIR, run_name)
    if os.path.exists(bag_dir):
        shutil.rmtree(bag_dir)

    launch_log = f"/tmp/launch_{run_name}.log"
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
    with open(launch_log, "w") as f:
        launch_proc = subprocess.Popen(launch_cmd, shell=True, executable="/bin/bash", stdout=f, stderr=subprocess.STDOUT)

    print("Attesa attivazione controller...")
    if not wait_for_controller(launch_log, 60):
        print(f"[ERRORE FATALE] Controller non attivato per S1 Run {run_idx}!")
        clean_processes()
        return None

    print("Controller ATTIVO! Avvio registrazione bag...")
    bag_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"ros2 bag record -o {bag_dir} "
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

    target_sim = 60.0
    print(f"Misura S1 Run {run_idx} avviata a sim_time = {sim_start:.3f} s. Target = {target_sim:.1f} s sim.")
    while True:
        now_sim = get_current_sim_time()
        if now_sim is not None:
            elapsed_sim = now_sim - sim_start
            elapsed_wall = time.time() - wall_start
            rtf = elapsed_sim / elapsed_wall if elapsed_wall > 0 else 0
            print(f"\r[{run_name}] Sim: {elapsed_sim:.2f}/{target_sim:.1f} s (Wall: {elapsed_wall:.1f} s, RTF: {rtf:.2f})", end="", flush=True)
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

    csv_path = os.path.join(OUT_DIR, f"{run_name}.csv")
    records = extract_joint_and_tcp_records(bag_dir, csv_path)
    if not records:
        return None

    t_sim = np.array([r["t_sim"] for r in records])
    t_wall = np.array([r["t_wall"] for r in records])
    q_arr = np.array([r["q"] for r in records])
    pos_arr = np.array([[r["x"], r["y"], r["z"]] for r in records])
    rot_arr = np.array([r["R"] for r in records])  # (N, 3, 3)

    idx_0 = 0
    idx_5s = int(np.argmin(np.abs(t_sim - 5.0)))
    idx_60s = int(np.argmin(np.abs(t_sim - 60.0)))

    p_0 = pos_arr[idx_0]
    p_5s = pos_arr[idx_5s]
    p_60s = pos_arr[idx_60s]

    q_0 = q_arr[idx_0]
    q_5s = q_arr[idx_5s]
    q_60s = q_arr[idx_60s]

    # Cedimento = p(5s) - p(t=0) (mm)
    cedimento_vec_mm = (p_5s - p_0) * 1000.0
    cedimento_norm_mm = float(np.linalg.norm(cedimento_vec_mm))

    # Deriva tra 5s e 60s (rispetto a p(5s))
    mask_5_60 = (t_sim >= 5.0) & (t_sim <= t_sim[idx_60s])
    t_5_60 = t_sim[mask_5_60] - 5.0
    pos_5_60 = pos_arr[mask_5_60]

    drift_5_60_vec = (pos_5_60 - p_5s) * 1000.0
    drift_5_60_norm = np.linalg.norm(drift_5_60_vec, axis=1)

    max_drift_5_60_mm = float(np.max(drift_5_60_norm))
    final_drift_5_60_mm = float(drift_5_60_norm[-1])

    if len(t_5_60) > 1:
        slope_mm_s, _ = np.polyfit(t_5_60, drift_5_60_norm, 1)
        slope_mm_min = float(slope_mm_s * 60.0)
    else:
        slope_mm_min = 0.0

    # Velocità massima (mm/s)
    dt_sim = np.diff(t_sim)
    valid_dt = dt_sim > 1e-6
    dpos = np.diff(pos_arr, axis=0)
    vel_norm = np.zeros(len(pos_arr))
    vel_norm[1:][valid_dt] = (np.linalg.norm(dpos[valid_dt], axis=1) / dt_sim[valid_dt]) * 1000.0
    max_vel_mms = float(np.max(vel_norm))

    # Cinematica attesa: TCP = (0.4293, 0, 0.7853) m, asse tool = (1, 0, 0)
    expected_tcp = np.array([0.42929057, 0.0, 0.78528205])
    expected_tool_axis = np.array([1.0, 0.0, 0.0])

    diff_tcp_0_mm = (p_0 - expected_tcp) * 1000.0
    diff_tcp_0_norm_mm = float(np.linalg.norm(diff_tcp_0_mm))

    tool_axis_0 = rot_arr[idx_0][:, 2]  # TCP z-axis in base frame
    tool_axis_err_deg = float(math.acos(np.clip(np.dot(tool_axis_0, expected_tool_axis), -1.0, 1.0)) * 180.0 / math.pi)

    total_sim = float(t_sim[-1] - t_sim[0])
    total_wall = float(t_wall[-1] - t_wall[0])
    rtf = total_sim / total_wall if total_wall > 0 else 0
    completed_frac = total_sim / 60.0
    median_dt = np.median(dt_sim[valid_dt])
    gaps_count = int(np.sum(dt_sim > 3 * median_dt))

    res = {
        "run_idx": run_idx,
        "run_name": run_name,
        "q_0": q_0.tolist(),
        "p_0": p_0.tolist(),
        "q_5s": q_5s.tolist(),
        "p_5s": p_5s.tolist(),
        "q_60s": q_60s.tolist(),
        "p_60s": p_60s.tolist(),
        "cedimento_x_mm": float(cedimento_vec_mm[0]),
        "cedimento_y_mm": float(cedimento_vec_mm[1]),
        "cedimento_z_mm": float(cedimento_vec_mm[2]),
        "cedimento_norm_mm": cedimento_norm_mm,
        "max_drift_5_60_mm": max_drift_5_60_mm,
        "final_drift_5_60_mm": final_drift_5_60_mm,
        "slope_mm_min": slope_mm_min,
        "max_vel_mms": max_vel_mms,
        "diff_tcp_0_x_mm": float(diff_tcp_0_mm[0]),
        "diff_tcp_0_y_mm": float(diff_tcp_0_mm[1]),
        "diff_tcp_0_z_mm": float(diff_tcp_0_mm[2]),
        "diff_tcp_0_norm_mm": diff_tcp_0_norm_mm,
        "tool_axis_0": tool_axis_0.tolist(),
        "tool_axis_err_deg": tool_axis_err_deg,
        "total_sim_s": total_sim,
        "total_wall_s": total_wall,
        "rtf": rtf,
        "completed_frac": completed_frac,
        "gaps_gt_3dt": gaps_count,
        "median_dt_ms": float(median_dt * 1000.0),
        "num_samples": len(records),
        "csv_path": csv_path
    }

    print(f"\n--- RISULTATI S1 RUN {run_idx} ---")
    print(f"Posa TCP t=0s:            {p_0.round(4).tolist()} m")
    print(f"Posa TCP t=5s:            {p_5s.round(4).tolist()} m")
    print(f"Posa TCP t=60s:           {p_60s.round(4).tolist()} m")
    print(f"Cedimento (5s - 0s):      dx={cedimento_vec_mm[0]:.4f} mm, dy={cedimento_vec_mm[1]:.4f} mm, dz={cedimento_vec_mm[2]:.4f} mm | norma={cedimento_norm_mm:.4f} mm")
    print(f"Deriva 5s-60s:            max={max_drift_5_60_mm:.4f} mm, finale={final_drift_5_60_mm:.4f} mm")
    print(f"Pendenza deriva 5s-60s:   {slope_mm_min:.4f} mm/min")
    print(f"Velocità max:             {max_vel_mms:.4f} mm/s")
    print(f"Diff vs Atteso (t=0s):    dx={diff_tcp_0_mm[0]:.4f} mm, dy={diff_tcp_0_mm[1]:.4f} mm, dz={diff_tcp_0_mm[2]:.4f} mm | norma={diff_tcp_0_norm_mm:.4f} mm")
    print(f"Asse tool z (approccio):  {tool_axis_0.round(4).tolist()} (errore angolare vs [1,0,0]: {tool_axis_err_deg:.4f} deg)")
    print(f"Tempo sim:                {total_sim:.3f} s (RTF: {rtf:.3f})")
    print(f"Buchi > 3*dt:             {gaps_count}")
    return res


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(BAGS_DIR, exist_ok=True)

    print("=================================================================")
    print("AVVIO VALUTAZIONE SPAWN Q6=PI (_spawnq6_1007)")
    print("=================================================================")

    # 1. S0: Nominal spawn
    res_s0 = run_s0()

    # 2. S1: q6=pi spawn (3 runs)
    runs_s1 = []
    for r in range(1, 4):
        res_r = run_s1_single(run_idx=r)
        if res_r is not None:
            runs_s1.append(res_r)
        time.sleep(2.0)

    # Restore original xacro
    restore_xacro_override()
    print("\nRipristinato file xacro originale in franka_description.")

    # Summary
    summary_path = os.path.join(OUT_DIR, "summary_spawnq6_1007.txt")
    with open(summary_path, "w") as sf:
        sf.write("=== REPORT VALUTAZIONE SPAWN Q6=PI (_spawnq6_1007) ===\n\n")

        sf.write("--- ESPERIMENTO S0: Spawn nominale readyPose (10s sim) ---\n")
        if res_s0:
            for k, v in res_s0.items():
                sf.write(f"  {k}: {v}\n")
        sf.write("\n")

        sf.write("--- ESPERIMENTO S1: Spawn readyPose con q6=pi (60s sim, 3 RUNS) ---\n")
        for res in runs_s1:
            sf.write(f"\n[Run {res['run_idx']}]\n")
            for k, v in res.items():
                sf.write(f"  {k}: {v}\n")

        if runs_s1:
            sf.write("\n--- STATISTICHE AGGREGATE S1 (MEDIA +- STD) ---\n")
            keys_to_agg = [
                "cedimento_x_mm", "cedimento_y_mm", "cedimento_z_mm", "cedimento_norm_mm",
                "max_drift_5_60_mm", "final_drift_5_60_mm", "slope_mm_min",
                "max_vel_mms", "diff_tcp_0_norm_mm", "tool_axis_err_deg",
                "rtf", "total_sim_s"
            ]
            for k in keys_to_agg:
                vals = [res[k] for res in runs_s1]
                m = np.mean(vals)
                s = np.std(vals)
                sf.write(f"  {k}: media = {m:.4f}, std = {s:.4f}\n")

    print("\n=================================================================")
    print(f"ESECUZIONE COMPLETATA! Riepilogo salvato in: {summary_path}")
    print("=================================================================")


if __name__ == "__main__":
    main()
