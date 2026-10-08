#!/usr/bin/env python3
"""
run_skipalign1007_experiment.py
Esecuzione e confronto A/B per ProDMP Kt=200 con:
  - Run 1: skip_initial_alignment=false
  - Run 2: skip_initial_alignment=true

Registra:
  /cartesian_impedance_controller/actual_pose
  /cartesian_impedance_controller/target_pose_aligned
  /target_pose
  /joint_states
  /clock
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
EVAL_DIR = os.path.join(WS_ROOT, "tools/gazebo_cartesian_eval")
DATA_DIR = os.path.join(WS_ROOT, "skipalign1007")
BAGS_DIR = os.path.join(DATA_DIR, "bags")

FRANKA_WS = "/root/ros_workspaces/ros2/franka_ws"
CONFIG_YAML = os.path.join(
    FRANKA_WS, "src/franka_ros2/franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml"
)
BRINGUP_SHARE = os.path.join(
    FRANKA_WS, "install/franka_gazebo_bringup/share/franka_gazebo_bringup"
)

WEIGHTS_PRODMP = os.path.join(WS_ROOT, "runs/20260915_090358_fit_reach_task_baseline_prodmp/weights.yaml")
ORIENTATION_PATH = os.path.join(WS_ROOT, "reach_task_baseline_nbasis200.yaml")


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
        "pkill -9 -f 'prodmp_gazebo_executor_node'",
        "pkill -9 -f 'dmp_gazebo_executor_node'",
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


def set_impedance_gains(kt=200.0, dt=10.0, kr=150.0, dr=1.0):
    for path in [
        CONFIG_YAML,
        os.path.join(BRINGUP_SHARE, "config/franka_gazebo_controllers.yaml"),
        os.path.join(WS_ROOT, "src/franka_gazebo_overrides/franka_gazebo_controllers.yaml"),
    ]:
        if not os.path.exists(path):
            continue
        with open(path, "r") as f:
            lines = f.readlines()
        new_lines = []
        for line in lines:
            if "translational_stiffness:" in line:
                new_lines.append(f"    translational_stiffness: {kt:.1f}\n")
            elif "translational_damping:" in line:
                new_lines.append(f"    translational_damping: {dt:.1f}\n")
            elif "rotational_stiffness:" in line:
                new_lines.append(f"    rotational_stiffness: {kr:.1f}\n")
            elif "rotational_damping:" in line:
                new_lines.append(f"    rotational_damping: {dr:.1f}\n")
            else:
                new_lines.append(line)
        with open(path, "w") as f:
            f.writelines(new_lines)


def run_single_experiment(run_name, skip_initial_alignment=False, kt=200.0):
    print(f"\n=======================================================================", flush=True)
    print(f">>> RUN: {run_name} (skip_initial_alignment={skip_initial_alignment}, Kt={kt})", flush=True)
    print(f"=======================================================================", flush=True)

    clean_processes()
    setup_bringup_share()
    set_impedance_gains(kt=kt, dt=10.0, kr=150.0, dr=1.0)

    os.makedirs(DATA_DIR, exist_ok=True)
    os.makedirs(BAGS_DIR, exist_ok=True)
    bag_dir = os.path.join(BAGS_DIR, run_name)
    if os.path.exists(bag_dir):
        shutil.rmtree(bag_dir)

    launch_log_path = f"/tmp/launch_{run_name}.log"
    launch_cmd = (
        f"export PYTHONUNBUFFERED=1 && "
        f"source /opt/ros/humble/setup.bash && "
        f"source {FRANKA_WS}/install/setup.bash && "
        f"source {WS_ROOT}/install/setup.bash && "
        f"export QT_QPA_PLATFORM=offscreen && "
        f"stdbuf -oL -eL ros2 launch franka_gazebo_bringup gazebo_cartesian_impedance_control.launch.py "
        f"headless:=true load_gripper:=true"
    )
    with open(launch_log_path, "w") as log_f:
        launch_proc = subprocess.Popen(
            launch_cmd, shell=True, executable="/bin/bash", stdout=log_f, stderr=subprocess.STDOUT
        )

    # Wait for controller activation (max 60s)
    activated = False
    check_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {FRANKA_WS}/install/setup.bash && "
        f"ros2 control list_controllers 2>/dev/null"
    )
    for _ in range(60):
        if os.path.exists(launch_log_path):
            try:
                with open(launch_log_path, "r") as f:
                    log_text = f.read()
                if (
                    "Configured and activated cartesian_impedance_controller" in log_text
                    or "Successfully loaded controller cartesian_impedance_controller" in log_text
                ):
                    activated = True
                    break
            except Exception:
                pass
        res = subprocess.run(check_cmd, shell=True, executable="/bin/bash", capture_output=True, text=True)
        if "cartesian_impedance_controller" in res.stdout and "active" in res.stdout:
            activated = True
            break
        time.sleep(1.0)

    if not activated:
        print(f"[ERRORE] Controller non attivato per {run_name}", flush=True)
        clean_processes()
        return None

    print(f"Controller active. Impostazione parametro skip_initial_alignment={skip_initial_alignment}...", flush=True)
    time.sleep(2.0)

    # Set parameter explicitly via ros2 param set
    param_val_str = "true" if skip_initial_alignment else "false"
    param_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"ros2 param set /cartesian_impedance_controller skip_initial_alignment {param_val_str}"
    )
    res_param = subprocess.run(param_cmd, shell=True, executable="/bin/bash", capture_output=True, text=True)
    print(f"Param set output: {res_param.stdout.strip()} (stderr: {res_param.stderr.strip()})", flush=True)

    time.sleep(1.0)

    # Start bag recording
    bag_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"ros2 bag record -o {bag_dir} "
        f"/cartesian_impedance_controller/target_pose_aligned "
        f"/cartesian_impedance_controller/actual_pose "
        f"/target_pose "
        f"/joint_states "
        f"/clock"
    )
    bag_proc = subprocess.Popen(
        bag_cmd, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    time.sleep(2.0)

    # Run ProDMP executor with timeout 150s
    exec_log_path = f"/tmp/exec_{run_name}.log"
    exec_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {FRANKA_WS}/install/setup.bash && "
        f"source {WS_ROOT}/install/setup.bash && "
        f"timeout 150 ros2 run haptic_dmp_learning prodmp_gazebo_executor_node "
        f"--ros-args "
        f"-p target_pose_topic:=/target_pose "
        f"-p target_twist_topic:=/target_twist "
        f"-p weights_yaml_path:={WEIGHTS_PRODMP} "
        f"-p orientation_weights_yaml_path:={ORIENTATION_PATH} "
        f"-p target_odom_required:=false "
        f"-p use_sim_time:=true"
    )
    print(f"Avvio ProDMP executor...", flush=True)
    with open(exec_log_path, "w") as exec_f:
        subprocess.run(exec_cmd, shell=True, executable="/bin/bash", stdout=exec_f, stderr=subprocess.STDOUT)

    print(f"ProDMP executor terminato. Attesa assestamento (3s)...", flush=True)
    time.sleep(3.0)

    # Stop recording cleanly
    if bag_proc.poll() is None:
        bag_proc.send_signal(signal.SIGINT)
        try:
            bag_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            bag_proc.kill()

    clean_processes()

    # Extract Bag to CSV
    extract_bag_to_csv(bag_dir, run_name)

    # Analyze CSVs
    return analyze_run_csvs(run_name, skip_initial_alignment)


def extract_bag_to_csv(bag_dir, run_name):
    print(f"Estrazione bag {bag_dir} in CSV...", flush=True)
    extract_script = f"""
import sys, os, glob, sqlite3, csv
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped

bag_dir = "{bag_dir}"
out_dir = "{DATA_DIR}"
db_paths = glob.glob(os.path.join(bag_dir, "*.db3"))
if not db_paths:
    sys.exit(1)
db_path = db_paths[0]

conn = sqlite3.connect(db_path)
cur = conn.cursor()
cur.execute("SELECT id, name FROM topics")
topic_map = {{name: tid for tid, name in cur.fetchall()}}

# 1. target_pose_aligned
t_aligned_csv = os.path.join(out_dir, "target_aligned_{run_name}.csv")
if "/cartesian_impedance_controller/target_pose_aligned" in topic_map:
    tid = topic_map["/cartesian_impedance_controller/target_pose_aligned"]
    cur.execute("SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC", (tid,))
    rows = cur.fetchall()
    with open(t_aligned_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "z", "qw", "qx", "qy", "qz"])
        t0 = None
        for (blob,) in rows:
            msg = deserialize_message(blob, PoseStamped)
            t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if t0 is None:
                t0 = t_sec
            p = msg.pose.position
            q = msg.pose.orientation
            w.writerow([t_sec - t0, p.x, p.y, p.z, q.w, q.x, q.y, q.z])

# 2. raw target_pose
t_raw_csv = os.path.join(out_dir, "target_raw_{run_name}.csv")
if "/target_pose" in topic_map:
    tid = topic_map["/target_pose"]
    cur.execute("SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC", (tid,))
    rows = cur.fetchall()
    with open(t_raw_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "z", "qw", "qx", "qy", "qz"])
        t0 = None
        for (blob,) in rows:
            msg = deserialize_message(blob, PoseStamped)
            t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if t0 is None:
                t0 = t_sec
            p = msg.pose.position
            q = msg.pose.orientation
            w.writerow([t_sec - t0, p.x, p.y, p.z, q.w, q.x, q.y, q.z])

# 3. actual_pose
act_csv = os.path.join(out_dir, "actual_pose_{run_name}.csv")
if "/cartesian_impedance_controller/actual_pose" in topic_map:
    tid = topic_map["/cartesian_impedance_controller/actual_pose"]
    cur.execute("SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC", (tid,))
    rows = cur.fetchall()
    with open(act_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "z", "qw", "qx", "qy", "qz"])
        t0 = None
        for (blob,) in rows:
            msg = deserialize_message(blob, PoseStamped)
            t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if t0 is None:
                t0 = t_sec
            p = msg.pose.position
            q = msg.pose.orientation
            w.writerow([t_sec - t0, p.x, p.y, p.z, q.w, q.x, q.y, q.z])

conn.close()
"""
    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"python3 -c '{extract_script}'"
    )
    subprocess.run(cmd, shell=True, executable="/bin/bash", check=True)
    print(f"Estrazione completata per {run_name}.", flush=True)


def analyze_run_csvs(run_name, skip_initial_alignment):
    t_aligned_csv = os.path.join(DATA_DIR, f"target_aligned_{run_name}.csv")
    t_raw_csv = os.path.join(DATA_DIR, f"target_raw_{run_name}.csv")
    act_csv = os.path.join(DATA_DIR, f"actual_pose_{run_name}.csv")

    def load_pose_csv(path):
        t, pos, quat = [], [], []
        if not os.path.exists(path):
            return np.array([]), np.array([]), np.array([])
        with open(path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                t.append(float(row["t"]))
                pos.append([float(row["x"]), float(row["y"]), float(row["z"])])
                quat.append([float(row["qw"]), float(row["qx"]), float(row["qy"]), float(row["qz"])])
        return np.array(t), np.array(pos), np.array(quat)

    t_tgt_aligned, pos_tgt_aligned, quat_tgt_aligned = load_pose_csv(t_aligned_csv)
    t_tgt_raw, pos_tgt_raw, quat_tgt_raw = load_pose_csv(t_raw_csv)
    t_act, pos_act, quat_act = load_pose_csv(act_csv)

    if len(t_act) == 0 or len(pos_tgt_aligned) == 0:
        print(f"[ERRORE] Dati mancanti per {run_name}")
        return None

    # Primo target raw e primo target aligned
    first_target_raw = pos_tgt_raw[0] if len(pos_tgt_raw) > 0 else pos_tgt_aligned[0]
    first_target_aligned = pos_tgt_aligned[0]

    # Posa TCP al momento del primo target
    # Il primo target arriva a t_act[0]
    first_tcp = pos_act[0]

    # Differenza al momento del primo target (vs target_aligned che è ciò che comanda il controller)
    diff_aligned_mm = (first_tcp - first_target_aligned) * 1000.0
    diff_aligned_norm_mm = np.linalg.norm(diff_aligned_mm)

    # Differenza vs target_raw puro pubblicato da ProDMP
    diff_raw_mm = (first_tcp - first_target_raw) * 1000.0
    diff_raw_norm_mm = np.linalg.norm(diff_raw_mm)

    # Tau nominale: istante di arrivo/clamp
    dist_from_end = np.linalg.norm(pos_tgt_aligned - pos_tgt_aligned[-1], axis=1)
    settle_indices = np.where(dist_from_end > 1e-4)[0]
    tau_nom = t_tgt_aligned[settle_indices[-1] + 1] if len(settle_indices) > 0 else t_tgt_aligned[-1]
    t_sim_end = t_tgt_aligned[-1]
    completion_frac = min(1.0, t_sim_end / tau_nom) if tau_nom > 0 else 1.0

    # Calcolo errori istante per istante (confrontando actual_pose con target_aligned interpolato/matchato)
    import bisect
    pos_errs = []
    times_matched = []
    for i, ta in enumerate(t_act):
        idx = bisect.bisect_left(t_tgt_aligned, ta)
        if idx >= len(t_tgt_aligned):
            idx = len(t_tgt_aligned) - 1
        dp = np.linalg.norm(pos_act[i] - pos_tgt_aligned[idx]) * 1000.0
        pos_errs.append(dp)
        times_matched.append(ta)

    pos_errs = np.array(pos_errs)
    times_matched = np.array(times_matched)

    err_max_t0 = pos_errs[0]

    mask_1s = times_matched >= 1.0
    err_max_after_1s = np.max(pos_errs[mask_1s]) if np.any(mask_1s) else np.max(pos_errs)

    err_mean = np.mean(pos_errs)
    err_final = pos_errs[-1]

    idx_tau = np.argmin(np.abs(times_matched - tau_nom))
    err_at_tau = pos_errs[idx_tau]

    results = {
        "run_name": run_name,
        "skip_initial_alignment": skip_initial_alignment,
        "first_target_raw_xyz_m": first_target_raw.tolist(),
        "first_target_aligned_xyz_m": first_target_aligned.tolist(),
        "first_tcp_xyz_m": first_tcp.tolist(),
        "diff_raw_mm": {
            "dx": diff_raw_mm[0],
            "dy": diff_raw_mm[1],
            "dz": diff_raw_mm[2],
            "norm": diff_raw_norm_mm,
        },
        "diff_aligned_mm": {
            "dx": diff_aligned_mm[0],
            "dy": diff_aligned_mm[1],
            "dz": diff_aligned_mm[2],
            "norm": diff_aligned_norm_mm,
        },
        "err_max_t0_mm": err_max_t0,
        "err_max_after_1s_mm": err_max_after_1s,
        "err_mean_mm": err_mean,
        "err_final_mm": err_final,
        "err_at_tau_mm": err_at_tau,
        "tau_nom_s": tau_nom,
        "t_sim_end_s": t_sim_end,
        "completion_frac": completion_frac,
        "num_target_samples": len(t_tgt_aligned),
        "num_act_samples": len(t_act),
    }

    print(f"\n--- RISULTATI {run_name} ---")
    print(f"  Primo target raw: [{first_target_raw[0]:.5f}, {first_target_raw[1]:.5f}, {first_target_raw[2]:.5f}] m")
    print(f"  Primo target aligned: [{first_target_aligned[0]:.5f}, {first_target_aligned[1]:.5f}, {first_target_aligned[2]:.5f}] m")
    print(f"  TCP a t=0: [{first_tcp[0]:.5f}, {first_tcp[1]:.5f}, {first_tcp[2]:.5f}] m")
    print(f"  Diff raw vs TCP: dx={diff_raw_mm[0]:.2f} mm, dy={diff_raw_mm[1]:.2f} mm, dz={diff_raw_mm[2]:.2f} mm, norm={diff_raw_norm_mm:.2f} mm")
    print(f"  Diff aligned vs TCP: dx={diff_aligned_mm[0]:.2f} mm, dy={diff_aligned_mm[1]:.2f} mm, dz={diff_aligned_mm[2]:.2f} mm, norm={diff_aligned_norm_mm:.2f} mm")
    print(f"  Errore max a t=0: {err_max_t0:.2f} mm")
    print(f"  Errore max dopo 1 s: {err_max_after_1s:.2f} mm")
    print(f"  Errore medio: {err_mean:.2f} mm")
    print(f"  Errore finale: {err_final:.2f} mm (a tau: {err_at_tau:.2f} mm)")
    print(f"  Frazione completata: {completion_frac:.3f} (t_end={t_sim_end:.2f}s / tau={tau_nom:.2f}s)")

    return results


def main():
    res_false = run_single_experiment("prodmp_kt200_skipalign_false_1007", skip_initial_alignment=False, kt=200.0)
    res_true = run_single_experiment("prodmp_kt200_skipalign_true_1007", skip_initial_alignment=True, kt=200.0)

    # Save summary report
    report_path = os.path.join(DATA_DIR, "summary_skipalign1007.txt")
    with open(report_path, "w") as f:
        f.write("=== CONFRONTO SPERIMENTALE ProDMP Kt=200 (_skipalign1007) ===\n\n")
        for res in [res_false, res_true]:
            if not res:
                continue
            f.write(f"--- RUN: {res['run_name']} (skip_initial_alignment={res['skip_initial_alignment']}) ---\n")
            f.write(f"  Primo target raw (xyz): {res['first_target_raw_xyz_m']} m\n")
            f.write(f"  Primo target aligned (xyz): {res['first_target_aligned_xyz_m']} m\n")
            f.write(f"  Posa TCP al primo target: {res['first_tcp_xyz_m']} m\n")
            f.write(f"  Differenza raw vs TCP: dx={res['diff_raw_mm']['dx']:.3f} mm, dy={res['diff_raw_mm']['dy']:.3f} mm, dz={res['diff_raw_mm']['dz']:.3f} mm, norm={res['diff_raw_mm']['norm']:.3f} mm\n")
            f.write(f"  Differenza aligned vs TCP: dx={res['diff_aligned_mm']['dx']:.3f} mm, dy={res['diff_aligned_mm']['dy']:.3f} mm, dz={res['diff_aligned_mm']['dz']:.3f} mm, norm={res['diff_aligned_mm']['norm']:.3f} mm\n")
            f.write(f"  Errore max t=0: {res['err_max_t0_mm']:.3f} mm\n")
            f.write(f"  Errore max t>=1s: {res['err_max_after_1s_mm']:.3f} mm\n")
            f.write(f"  Errore medio: {res['err_mean_mm']:.3f} mm\n")
            f.write(f"  Errore finale: {res['err_final_mm']:.3f} mm (a tau: {res['err_at_tau_mm']:.3f} mm)\n")
            f.write(f"  Frazione completata: {res['completion_frac']:.4f} (t_end={res['t_sim_end_s']:.3f} s / tau={res['tau_nom_s']:.3f} s)\n\n")

    print(f"\nReport salvato in {report_path}")


if __name__ == "__main__":
    main()
