#!/usr/bin/env python3
"""
run_drift_baseline1007.py
Misura della deriva di base per cartesian_impedance_controller su Franka FER in Gazebo.
Esperimenti:
  - B1: Spawn in readyPose, 60s sim time, nessun target.
  - B2: Prepos capture (ramp 20s a [0.589, 0.344, 0.35]) + 60s sim hold.
  - B3: Ripetizione di B2 per 3 volte totali (Run 1, Run 2, Run 3) con calcolo di media e std dev.
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

OUT_DIR = os.path.join(WS_ROOT, "baseline1007")
BAGS_DIR = os.path.join(OUT_DIR, "bags")


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
        "pkill -9 -f 'prepos_capture'",
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
    """Reads current /clock sim time via a quick ros2 topic echo with timeout."""
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


def extract_bag_to_csv(bag_dir, csv_out_path):
    db_files = glob.glob(os.path.join(bag_dir, "*.db3"))
    if not db_files:
        print(f"[ERRORE] Nessun file .db3 trovato in {bag_dir}")
        return None

    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import PoseStamped
    from rosgraph_msgs.msg import Clock

    conn = sqlite3.connect(db_files[0])
    cur = conn.cursor()
    cur.execute("SELECT id, name FROM topics")
    topics = dict(cur.fetchall())
    topics_rev = {v: k for k, v in topics.items()}

    # Extract clock map (timestamp_ns -> sim_time_s)
    clock_data = []
    if "/clock" in topics_rev:
        cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC",
                    (topics_rev["/clock"],))
        for ts_ns, raw_msg in cur.fetchall():
            c = deserialize_message(raw_msg, Clock)
            sim_t = c.clock.sec + c.clock.nanosec * 1e-9
            clock_data.append((ts_ns * 1e-9, sim_t))

    # Function to interpolate sim_time given record timestamp
    def get_sim_time(rec_t_s):
        if not clock_data:
            return rec_t_s
        # Linear search or bisect
        idx = bisect_search(clock_data, rec_t_s)
        return clock_data[idx][1]

    def bisect_search(cd, t):
        l, r = 0, len(cd) - 1
        while l <= r:
            m = (l + r) // 2
            if cd[m][0] == t:
                return m
            elif cd[m][0] < t:
                l = m + 1
            else:
                r = m - 1
        idx = max(0, min(r, len(cd) - 1))
        return idx

    actual_pose_topic = "/cartesian_impedance_controller/actual_pose"
    records = []

    if actual_pose_topic in topics_rev:
        cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC",
                    (topics_rev[actual_pose_topic],))
        rows = cur.fetchall()
        if rows:
            t_wall_0 = rows[0][0] * 1e-9
            t_sim_0 = get_sim_time(t_wall_0)
            for ts_ns, raw_msg in rows:
                p = deserialize_message(raw_msg, PoseStamped)
                t_wall = ts_ns * 1e-9 - t_wall_0
                t_sim = get_sim_time(ts_ns * 1e-9)
                px = p.pose.position.x
                py = p.pose.position.y
                pz = p.pose.position.z
                qw = p.pose.orientation.w
                qx = p.pose.orientation.x
                qy = p.pose.orientation.y
                qz = p.pose.orientation.z
                records.append({
                    "t_wall": t_wall,
                    "t_sim": t_sim,
                    "x": px, "y": py, "z": pz,
                    "qw": qw, "qx": qx, "qy": qy, "qz": qz
                })

    if not records and "/joint_states" in topics_rev:
        # Fallback to computing TCP pose from joint_states via Pinocchio FK
        import pinocchio as pin
        from sensor_msgs.msg import JointState
        urdf_path = "/root/thesis_ws/fer_flat_effort.urdf"
        model = pin.buildModelFromUrdf(urdf_path)
        data = model.createData()
        frame_id = model.getFrameId("fer_hand_tcp")
        joint_names = ["fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4", "fer_joint5", "fer_joint6", "fer_joint7"]
        q_indices = [model.idx_qs[model.getJointId(jn)] for jn in joint_names]

        cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC",
                    (topics_rev["/joint_states"],))
        rows = cur.fetchall()
        if rows:
            t_wall_0 = rows[0][0] * 1e-9
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
                t_sim = get_sim_time(ts_ns * 1e-9)
                records.append({
                    "t_wall": t_wall,
                    "t_sim": t_sim,
                    "x": pos[0], "y": pos[1], "z": pos[2],
                    "qw": quat.w, "qx": quat.x, "qy": quat.y, "qz": quat.z
                })

    if not records:
        print(f"[ERRORE] Nessun messaggio valido trovato nella bag {bag_dir}")
        return None

    # Adjust t_sim relative to start
    sim_0 = records[0]["t_sim"]
    for r in records:
        r["t_sim_rel"] = r["t_sim"] - sim_0

    with open(csv_out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_sim", "t_wall", "x", "y", "z", "qw", "qx", "qy", "qz"])
        for r in records:
            writer.writerow([
                f"{r['t_sim_rel']:.6f}",
                f"{r['t_wall']:.6f}",
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


def run_b1():
    print("\n=======================================================")
    print("ESECUZIONE ESPERIMENTO B1: Drift in readyPose (60s sim, no target)")
    print("=======================================================")
    clean_processes()
    setup_bringup_share()

    bag_dir = os.path.join(BAGS_DIR, "b1_ready_drift_baseline1007")
    if os.path.exists(bag_dir):
        shutil.rmtree(bag_dir)

    launch_log = "/tmp/launch_b1_baseline1007.log"
    launch_cmd = (
        f"export PYTHONUNBUFFERED=1 && "
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
        print("[ERRORE FATALE] Controller non attivato per B1!")
        clean_processes()
        return None

    print("Controller ATTIVO! Avvio registrazione bag...")
    bag_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"ros2 bag record -o {bag_dir} "
        f"/cartesian_impedance_controller/actual_pose "
        f"/joint_states "
        f"/clock"
    )
    bag_proc = subprocess.Popen(bag_cmd, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)

    # Monitor sim time for 60.0 seconds
    sim_start = None
    wall_start = time.time()
    while sim_start is None:
        sim_start = get_current_sim_time()
        time.sleep(0.2)

    print(f"Inizio misura drift a sim_time = {sim_start:.3f} s. Target durata = 60.0 s sim.")
    while True:
        now_sim = get_current_sim_time()
        if now_sim is not None:
            elapsed_sim = now_sim - sim_start
            elapsed_wall = time.time() - wall_start
            rtf = elapsed_sim / elapsed_wall if elapsed_wall > 0 else 0
            print(f"\r[B1 Progresso] Sim: {elapsed_sim:.2f}/60.00 s (Wall: {elapsed_wall:.1f} s, RTF: {rtf:.2f})", end="", flush=True)
            if elapsed_sim >= 60.0:
                print(f"\nRaggiunti {elapsed_sim:.2f} s sim! Arresto simulazione...")
                break
        time.sleep(0.5)

    # Stop recording
    if bag_proc.poll() is None:
        bag_proc.send_signal(signal.SIGINT)
        try:
            bag_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            bag_proc.kill()

    clean_processes()

    # Process and analyze B1
    csv_path = os.path.join(OUT_DIR, "b1_ready_drift_baseline1007.csv")
    records = extract_bag_to_csv(bag_dir, csv_path)
    if not records:
        return None

    # Compute metrics
    t_sim = np.array([r["t_sim_rel"] for r in records])
    t_wall = np.array([r["t_wall"] for r in records])
    pos = np.array([[r["x"], r["y"], r["z"]] for r in records])

    p0 = pos[0]
    drift_vec = pos - p0
    drift_norm_mm = np.linalg.norm(drift_vec, axis=1) * 1000.0

    # Numerical velocity (mm/s)
    dt_sim = np.diff(t_sim)
    valid_dt = dt_sim > 1e-6
    dpos = np.diff(pos, axis=0)
    vel_norm_mms = np.zeros(len(pos))
    vel_norm_mms[1:][valid_dt] = (np.linalg.norm(dpos[valid_dt], axis=1) / dt_sim[valid_dt]) * 1000.0

    max_drift_mm = float(np.max(drift_norm_mm))
    final_drift_mm = float(drift_norm_mm[-1])
    max_vel_mms = float(np.max(vel_norm_mms))

    total_sim = float(t_sim[-1] - t_sim[0])
    total_wall = float(t_wall[-1] - t_wall[0])
    rtf = total_sim / total_wall if total_wall > 0 else 0
    completed_frac = total_sim / 60.0

    # Gaps > 3*dt (using median dt)
    median_dt = np.median(dt_sim[valid_dt]) if np.any(valid_dt) else 0.001
    gaps_count = int(np.sum(dt_sim > 3 * median_dt))

    res = {
        "run": "B1",
        "p0": p0.tolist(),
        "p_end": pos[-1].tolist(),
        "max_drift_mm": max_drift_mm,
        "final_drift_mm": final_drift_mm,
        "max_vel_mms": max_vel_mms,
        "total_sim_s": total_sim,
        "total_wall_s": total_wall,
        "rtf": rtf,
        "completed_frac": completed_frac,
        "gaps_gt_3dt": gaps_count,
        "median_dt_ms": float(median_dt * 1000.0),
        "num_samples": len(records),
        "csv_path": csv_path
    }

    print("\n--- RISULTATI B1 ---")
    print(f"Posa iniziale p0:          {p0.round(4).tolist()} m")
    print(f"Posa finale p_end:         {pos[-1].round(4).tolist()} m")
    print(f"Deriva massima |p - p0|:  {max_drift_mm:.4f} mm")
    print(f"Deriva finale:            {final_drift_mm:.4f} mm")
    print(f"Velocità massima:         {max_vel_mms:.4f} mm/s")
    print(f"Tempo simulato effettivo: {total_sim:.3f} s (frazione: {completed_frac*100:.1f}%)")
    print(f"RTF:                      {rtf:.3f}")
    print(f"Buchi > 3*dt (dt={median_dt*1000:.2f}ms): {gaps_count}")
    return res


def run_prepos_and_hold(run_index=1):
    run_name = f"b2_run{run_index}_baseline1007"
    print(f"\n=======================================================")
    print(f"ESECUZIONE ESPERIMENTO B2 / B3 (Run {run_index}/3): Prepos Ramp (20s) + Hold (60s sim)")
    print(f"=======================================================")
    clean_processes()
    setup_bringup_share()

    bag_dir = os.path.join(BAGS_DIR, run_name)
    if os.path.exists(bag_dir):
        shutil.rmtree(bag_dir)

    launch_log = f"/tmp/launch_{run_name}.log"
    launch_cmd = (
        f"export PYTHONUNBUFFERED=1 && "
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
        print(f"[ERRORE FATALE] Controller non attivato per Run {run_index}!")
        clean_processes()
        return None

    print("Controller ATTIVO! Avvio registrazione bag...")
    bag_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"ros2 bag record -o {bag_dir} "
        f"/cartesian_impedance_controller/actual_pose "
        f"/cartesian_impedance_controller/target_pose_aligned "
        f"/joint_states "
        f"/clock"
    )
    bag_proc = subprocess.Popen(bag_cmd, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)

    # Launch prepos_capture.py in background
    prepos_log = f"/tmp/prepos_{run_name}.log"
    prepos_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {FRANKA_WS}/install/setup.bash && "
        f"source {WS_ROOT}/install/setup.bash && "
        f"python3 {WS_ROOT}/tools/prepos_capture.py "
        f"--anchor-pos 0.589 0.344 0.35 "
        f"--duration 20.0 "
        f"--rate 200.0"
    )
    with open(prepos_log, "w") as f:
        prepos_proc = subprocess.Popen(prepos_cmd, shell=True, executable="/bin/bash", stdout=f, stderr=subprocess.STDOUT)

    # Monitor sim time: 20s ramp + 60s hold = 80s total sim time
    sim_start = None
    wall_start = time.time()
    while sim_start is None:
        sim_start = get_current_sim_time()
        time.sleep(0.2)

    total_target_sim = 80.0
    print(f"Prepos avviato a sim_time = {sim_start:.3f} s. Target totale = {total_target_sim:.1f} s sim (20s ramp + 60s hold).")

    while True:
        now_sim = get_current_sim_time()
        if now_sim is not None:
            elapsed_sim = now_sim - sim_start
            elapsed_wall = time.time() - wall_start
            rtf = elapsed_sim / elapsed_wall if elapsed_wall > 0 else 0
            phase = "RAMP" if elapsed_sim < 20.0 else f"HOLD ({elapsed_sim - 20.0:.1f}/60.0s)"
            print(f"\r[{run_name} {phase}] Sim: {elapsed_sim:.2f}/{total_target_sim:.1f} s (Wall: {elapsed_wall:.1f} s, RTF: {rtf:.2f})", end="", flush=True)
            if elapsed_sim >= total_target_sim:
                print(f"\nRaggiunti {elapsed_sim:.2f} s sim! Termine run...")
                break
        time.sleep(0.5)

    # Terminate prepos_capture and bag
    if prepos_proc.poll() is None:
        prepos_proc.send_signal(signal.SIGINT)
        try:
            prepos_proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            prepos_proc.kill()

    if bag_proc.poll() is None:
        bag_proc.send_signal(signal.SIGINT)
        try:
            bag_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            bag_proc.kill()

    clean_processes()

    # Process and analyze data
    csv_path = os.path.join(OUT_DIR, f"{run_name}.csv")
    records = extract_bag_to_csv(bag_dir, csv_path)
    if not records:
        return None

    t_sim = np.array([r["t_sim_rel"] for r in records])
    t_wall = np.array([r["t_wall"] for r in records])
    pos = np.array([[r["x"], r["y"], r["z"]] for r in records])

    target_pos = np.array([0.589, 0.344, 0.35])

    # Find index at capture (t_sim_rel closest to 20.0s)
    idx_capture = int(np.argmin(np.abs(t_sim - 20.0)))
    p_capture = pos[idx_capture]
    err_capture_vec = (p_capture - target_pos) * 1000.0  # mm
    err_capture_norm = float(np.linalg.norm(err_capture_vec))

    # Hold phase: t_sim_rel >= 20.0
    hold_mask = t_sim >= 20.0
    t_hold = t_sim[hold_mask] - 20.0  # relative to hold start in seconds
    pos_hold = pos[hold_mask]

    drift_hold_vec = (pos_hold - p_capture) * 1000.0  # mm
    drift_hold_norm = np.linalg.norm(drift_hold_vec, axis=1)

    max_drift_hold_mm = float(np.max(drift_hold_norm))
    final_drift_hold_mm = float(drift_hold_norm[-1])

    # Drift slope: linear regression on drift_hold_norm vs t_hold (mm/min)
    if len(t_hold) > 1:
        slope_mm_s, intercept = np.polyfit(t_hold, drift_hold_norm, 1)
        slope_mm_min = float(slope_mm_s * 60.0)
    else:
        slope_mm_min = 0.0

    # Oscillations during hold: Peak-to-peak amplitude (max - min) in x,y,z and norm
    p2p_x = float(np.ptp(pos_hold[:, 0]) * 1000.0)
    p2p_y = float(np.ptp(pos_hold[:, 1]) * 1000.0)
    p2p_z = float(np.ptp(pos_hold[:, 2]) * 1000.0)
    p2p_norm = float(math.sqrt(p2p_x**2 + p2p_y**2 + p2p_z**2))
    std_residual = float(np.std(drift_hold_norm - (slope_mm_s * t_hold + intercept)))

    total_sim = float(t_sim[-1] - t_sim[0])
    total_wall = float(t_wall[-1] - t_wall[0])
    rtf = total_sim / total_wall if total_wall > 0 else 0
    completed_frac = total_sim / total_target_sim

    dt_sim = np.diff(t_sim)
    valid_dt = dt_sim > 1e-6
    median_dt = np.median(dt_sim[valid_dt]) if np.any(valid_dt) else 0.001
    gaps_count = int(np.sum(dt_sim > 3 * median_dt))

    res = {
        "run_index": run_index,
        "run_name": run_name,
        "p_capture": p_capture.tolist(),
        "err_cap_x_mm": float(err_capture_vec[0]),
        "err_cap_y_mm": float(err_capture_vec[1]),
        "err_cap_z_mm": float(err_capture_vec[2]),
        "err_cap_norm_mm": err_capture_norm,
        "max_drift_hold_mm": max_drift_hold_mm,
        "final_drift_hold_mm": final_drift_hold_mm,
        "slope_mm_min": slope_mm_min,
        "p2p_x_mm": p2p_x,
        "p2p_y_mm": p2p_y,
        "p2p_z_mm": p2p_z,
        "p2p_norm_mm": p2p_norm,
        "std_residual_mm": std_residual,
        "hold_duration_sim_s": float(t_hold[-1] - t_hold[0]) if len(t_hold) > 0 else 0.0,
        "total_sim_s": total_sim,
        "total_wall_s": total_wall,
        "rtf": rtf,
        "completed_frac": completed_frac,
        "gaps_gt_3dt": gaps_count,
        "median_dt_ms": float(median_dt * 1000.0),
        "num_samples": len(records),
        "csv_path": csv_path
    }

    print(f"\n--- RISULTATI RUN {run_index} ---")
    print(f"Posa alla cattura (t=20s):   {p_capture.round(4).tolist()} m")
    print(f"Errore cattura vs target:    dx={err_capture_vec[0]:.3f} mm, dy={err_capture_vec[1]:.3f} mm, dz={err_capture_vec[2]:.3f} mm | norma={err_capture_norm:.3f} mm")
    print(f"Deriva durante hold (60s):   max={max_drift_hold_mm:.4f} mm, finale={final_drift_hold_mm:.4f} mm")
    print(f"Pendenza deriva nell'hold:   {slope_mm_min:.4f} mm/min")
    print(f"Oscillazioni peak-to-peak:   dx={p2p_x:.4f} mm, dy={p2p_y:.4f} mm, dz={p2p_z:.4f} mm (std residua: {std_residual:.4f} mm)")
    print(f"Tempo simulato effettivo:    {total_sim:.3f} s (frazione: {completed_frac*100:.1f}%)")
    print(f"RTF:                         {rtf:.3f}")
    print(f"Buchi > 3*dt:                {gaps_count}")

    return res


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(BAGS_DIR, exist_ok=True)

    print("=================================================================")
    print("AVVIO SUITE COMPLETA MISURAZIONE DERIVA BASELINE (_baseline1007)")
    print("=================================================================")

    # 1. Run B1
    res_b1 = run_b1()

    # 2. Run B2 & B3 (3 runs of prepos + hold)
    runs_b2_b3 = []
    for r in range(1, 4):
        res_r = run_prepos_and_hold(run_index=r)
        if res_r is not None:
            runs_b2_b3.append(res_r)
        time.sleep(2.0)

    # Summary calculations
    summary_path = os.path.join(OUT_DIR, "summary_baseline1007.txt")
    with open(summary_path, "w") as sf:
        sf.write("=== REPORT MISURE DI DERIVA BASELINE (_baseline1007) ===\n\n")

        sf.write("--- ESPERIMENTO B1: Spawn readyPose (nessun target, 60s sim) ---\n")
        if res_b1:
            for k, v in res_b1.items():
                sf.write(f"  {k}: {v}\n")
        else:
            sf.write("  FALLITO\n")
        sf.write("\n")

        sf.write("--- ESPERIMENTO B2 / B3: Prepos Capture (20s ramp) + 60s Hold (3 RUNS) ---\n")
        for res in runs_b2_b3:
            sf.write(f"\n[Run {res['run_index']}]\n")
            for k, v in res.items():
                sf.write(f"  {k}: {v}\n")

        if runs_b2_b3:
            sf.write("\n--- STATISTICHE AGGREGATE B3 (MEDIA +- STD) ---\n")
            keys_to_agg = [
                "err_cap_x_mm", "err_cap_y_mm", "err_cap_z_mm", "err_cap_norm_mm",
                "max_drift_hold_mm", "final_drift_hold_mm", "slope_mm_min",
                "p2p_norm_mm", "std_residual_mm", "rtf", "total_sim_s"
            ]
            for k in keys_to_agg:
                vals = [r[k] for r in runs_b2_b3]
                m = np.mean(vals)
                s = np.std(vals)
                sf.write(f"  {k}: media = {m:.4f}, std = {s:.4f}\n")

    print("\n=================================================================")
    print(f"TUTTE LE ESECUZIONI COMPLETATE! Riepilogo salvato in: {summary_path}")
    print("=================================================================")


if __name__ == "__main__":
    main()
