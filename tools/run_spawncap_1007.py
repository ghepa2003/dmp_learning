#!/usr/bin/env python3
"""
run_spawncap_1007.py
Valutazione dello spawn diretto nella posa di cattura (FRANKA_START_POSE=capture).
Esperimento: 3 ripetizioni indipendenti (stack nuovo ogni volta), 60 s simulati ciascuna.
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
ORIG_ARM_XACRO_BAK = os.path.join(DESC_SHARE_DIR, "franka_arm.ros2_control.xacro.bak_spawncap_1007")

SRC_XACRO = os.path.join(WS_ROOT, "src/franka_description_overrides/franka_arm.ros2_control.xacro")

OUT_DIR = os.path.join(WS_ROOT, "spawncap_1007")
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
    shutil.copyfile(SRC_XACRO, ORIG_ARM_XACRO)


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
        t_sim_abs = get_sim_time(ts_ns * 1e-9)
        t_sim = t_sim_abs - sim_0

        records.append({
            "t_sim": t_sim,
            "t_sim_abs": t_sim_abs,
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


def quat_angular_diff_deg(q1_xyzw, q2_xyzw):
    q1 = np.array(q1_xyzw, dtype=np.float64) / np.linalg.norm(q1_xyzw)
    q2 = np.array(q2_xyzw, dtype=np.float64) / np.linalg.norm(q2_xyzw)
    dot = np.clip(np.abs(np.dot(q1, q2)), 0.0, 1.0)
    return float(2.0 * math.acos(dot) * 180.0 / math.pi)


def run_spawncap_single(run_idx=1):
    run_name = f"spawncap_run{run_idx}_spawncap_1007"
    print(f"\n=======================================================")
    print(f"ESECUZIONE ESPERIMENTO SPAWN CAPTURE (Run {run_idx}/3): FRANKA_START_POSE=capture (60s sim)")
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
        f"export FRANKA_START_POSE=capture && "
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
        print(f"[ERRORE FATALE] Controller non attivato per Run {run_idx}!")
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
    print(f"Misura Run {run_idx} avviata a sim_time = {sim_start:.3f} s. Target = {target_sim:.1f} s sim.")
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
    quat_arr = np.array([[r["qx"], r["qy"], r["qz"], r["qw"]] for r in records])  # xyzw

    # Save first 2s at max frequency
    first2s_mask = t_sim <= 2.0
    first2s_csv = os.path.join(OUT_DIR, f"{run_name}_first2s.csv")
    with open(first2s_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_sim", "q4", "tcp_x", "tcp_y", "tcp_z", "qw", "qx", "qy", "qz"])
        for r in records:
            if r["t_sim"] <= 2.0:
                writer.writerow([
                    f"{r['t_sim']:.6f}",
                    f"{r['q'][3]:.6f}",
                    f"{r['x']:.6f}",
                    f"{r['y']:.6f}",
                    f"{r['z']:.6f}",
                    f"{r['qw']:.6f}",
                    f"{r['qx']:.6f}",
                    f"{r['qy']:.6f}",
                    f"{r['qz']:.6f}",
                ])
    print(f"Salvato CSV primi 2s: {first2s_csv}")

    idx_0 = 0
    idx_5s = int(np.argmin(np.abs(t_sim - 5.0)))
    idx_60s = int(np.argmin(np.abs(t_sim - 60.0)))

    sim_t_0 = records[idx_0]["t_sim_abs"]

    q_0 = q_arr[idx_0]
    q_5s = q_arr[idx_5s]
    q_60s = q_arr[idx_60s]

    p_0 = pos_arr[idx_0]
    p_5s = pos_arr[idx_5s]
    p_60s = pos_arr[idx_60s]

    tool_axis_0 = rot_arr[idx_0][:, 2]
    tool_axis_5s = rot_arr[idx_5s][:, 2]
    tool_axis_60s = rot_arr[idx_60s][:, 2]

    # Expected target pose
    expected_pos = np.array([0.589, 0.344, 0.350])
    expected_quat_xyzw = np.array([0.6903455270798549, 0.15304591873303097, 0.15304591873303086, 0.6903455270798549])
    expected_tool_axis = np.array([0.42261826, -0.90630779, 0.0])

    # Error at 5s vs expected
    diff_p5s_vec = (p_5s - expected_pos) * 1000.0  # mm
    diff_p5s_norm = float(np.linalg.norm(diff_p5s_vec))
    diff_q5s_deg = quat_angular_diff_deg(quat_arr[idx_5s], expected_quat_xyzw)

    # Error at 0s vs expected
    diff_p0_vec = (p_0 - expected_pos) * 1000.0
    diff_p0_norm = float(np.linalg.norm(diff_p0_vec))
    diff_q0_deg = quat_angular_diff_deg(quat_arr[idx_0], expected_quat_xyzw)

    # Error at 60s vs expected
    diff_p60s_vec = (p_60s - expected_pos) * 1000.0
    diff_p60s_norm = float(np.linalg.norm(diff_p60s_vec))
    diff_q60s_deg = quat_angular_diff_deg(quat_arr[idx_60s], expected_quat_xyzw)

    # Drift between 5s and 60s (relative to p_5s)
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

    # Max velocity across the whole run (mm/s)
    dt_sim = np.diff(t_sim)
    valid_dt = dt_sim > 1e-6
    dpos = np.diff(pos_arr, axis=0)
    vel_norm = np.zeros(len(pos_arr))
    vel_norm[1:][valid_dt] = (np.linalg.norm(dpos[valid_dt], axis=1) / dt_sim[valid_dt]) * 1000.0
    max_vel_mms = float(np.max(vel_norm))

    total_sim = float(t_sim[-1] - t_sim[0])
    total_wall = float(t_wall[-1] - t_wall[0])
    rtf = total_sim / total_wall if total_wall > 0 else 0
    completed_frac = total_sim / 60.0
    median_dt = np.median(dt_sim[valid_dt])
    gaps_count = int(np.sum(dt_sim > 3 * median_dt))

    res = {
        "run_idx": run_idx,
        "run_name": run_name,
        "sim_t_0": float(sim_t_0),
        "q_0": q_0.tolist(),
        "p_0": p_0.tolist(),
        "axis_0": tool_axis_0.tolist(),
        "diff_p0_norm_mm": diff_p0_norm,
        "diff_q0_deg": diff_q0_deg,
        "q_5s": q_5s.tolist(),
        "p_5s": p_5s.tolist(),
        "axis_5s": tool_axis_5s.tolist(),
        "diff_p5s_x_mm": float(diff_p5s_vec[0]),
        "diff_p5s_y_mm": float(diff_p5s_vec[1]),
        "diff_p5s_z_mm": float(diff_p5s_vec[2]),
        "diff_p5s_norm_mm": diff_p5s_norm,
        "diff_q5s_deg": diff_q5s_deg,
        "q_60s": q_60s.tolist(),
        "p_60s": p_60s.tolist(),
        "axis_60s": tool_axis_60s.tolist(),
        "diff_p60s_norm_mm": diff_p60s_norm,
        "diff_q60s_deg": diff_q60s_deg,
        "max_drift_5_60_mm": max_drift_5_60_mm,
        "final_drift_5_60_mm": final_drift_5_60_mm,
        "slope_mm_min": slope_mm_min,
        "max_vel_mms": max_vel_mms,
        "total_sim_s": total_sim,
        "total_wall_s": total_wall,
        "rtf": rtf,
        "completed_frac": completed_frac,
        "gaps_gt_3dt": gaps_count,
        "median_dt_ms": float(median_dt * 1000.0),
        "num_samples": len(records),
        "csv_path": csv_path,
        "first2s_csv": first2s_csv
    }

    print(f"\n--- RISULTATI RUN {run_idx} ---")
    print(f"Tempo sim primo campione:     {sim_t_0:.3f} s")
    print(f"Giunti t=0s (rad):            {[round(v,4) for v in q_0]}")
    print(f"Posa TCP t=0s (m):            {p_0.round(4).tolist()} | asse: {tool_axis_0.round(4).tolist()}")
    print(f"Giunti t=5s (rad):            {[round(v,4) for v in q_5s]}")
    print(f"Posa TCP t=5s (m):            {p_5s.round(4).tolist()} | asse: {tool_axis_5s.round(4).tolist()}")
    print(f"Giunti t=60s (rad):           {[round(v,4) for v in q_60s]}")
    print(f"Posa TCP t=60s (m):           {p_60s.round(4).tolist()} | asse: {tool_axis_60s.round(4).tolist()}")
    print(f"Errore posa t=5s vs Target:   dx={diff_p5s_vec[0]:.2f} mm, dy={diff_p5s_vec[1]:.2f} mm, dz={diff_p5s_vec[2]:.2f} mm | norma={diff_p5s_norm:.2f} mm")
    print(f"Errore orientamento t=5s:     {diff_q5s_deg:.2f} deg")
    print(f"Deriva 5s-60s:                max={max_drift_5_60_mm:.4f} mm, finale={final_drift_5_60_mm:.4f} mm")
    print(f"Pendenza deriva 5s-60s:       {slope_mm_min:.4f} mm/min")
    print(f"Velocità max:                 {max_vel_mms:.2f} mm/s")
    print(f"Tempo sim effettivo:          {total_sim:.3f} s (RTF: {rtf:.3f})")
    print(f"Buchi > 3*dt:                 {gaps_count}")

    return res


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(BAGS_DIR, exist_ok=True)

    print("=================================================================")
    print("AVVIO VALUTAZIONE SPAWN CAPTURE POSE (_spawncap_1007)")
    print("=================================================================")

    runs_data = []
    for r in range(1, 4):
        res = run_spawncap_single(run_idx=r)
        if res is not None:
            runs_data.append(res)
        time.sleep(2.0)

    restore_xacro_override()
    print("\nRipristinato file xacro originale in franka_description.")

    summary_path = os.path.join(OUT_DIR, "summary_spawncap_1007.txt")
    with open(summary_path, "w") as sf:
        sf.write("=== REPORT VALUTAZIONE SPAWN CAPTURE POSE (_spawncap_1007) ===\n\n")

        for res in runs_data:
            sf.write(f"\n[Run {res['run_idx']}]\n")
            for k, v in res.items():
                sf.write(f"  {k}: {v}\n")

        if runs_data:
            sf.write("\n--- STATISTICHE ALL'EQUILIBRIO (t >= 5 s) (MEDIA +- STD) ---\n")
            keys_to_agg = [
                "diff_p5s_x_mm", "diff_p5s_y_mm", "diff_p5s_z_mm", "diff_p5s_norm_mm",
                "diff_q5s_deg", "diff_p60s_norm_mm", "diff_q60s_deg",
                "max_drift_5_60_mm", "final_drift_5_60_mm", "slope_mm_min",
                "max_vel_mms", "rtf", "total_sim_s"
            ]
            for k in keys_to_agg:
                vals = [res[k] for res in runs_data]
                m = np.mean(vals)
                s = np.std(vals)
                sf.write(f"  {k}: media = {m:.4f}, std = {s:.4f}\n")

    print("\n=================================================================")
    print(f"ESECUZIONE COMPLETATA! Riepilogo salvato in: {summary_path}")
    print("=================================================================")


if __name__ == "__main__":
    main()
