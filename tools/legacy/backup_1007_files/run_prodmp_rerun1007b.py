#!/usr/bin/env python3
"""
run_prodmp_rerun1007b.py — Esecuzione e valutazione della Reach Baseline ProDMP con suffisso _rerun1007b e timeout 150s.
"""

import os
import sys
import time
import subprocess
import signal
import math
import csv
import bisect
import shutil
import numpy as np

WS_ROOT = "/root/thesis_ws"
EVAL_DIR = os.path.join(WS_ROOT, "tools/gazebo_cartesian_eval")
DATA_DIR = os.path.join(EVAL_DIR, "data")
BAGS_DIR = os.path.join(EVAL_DIR, "bags")

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


def set_impedance_gains(kt=2000.0, dt=10.0, kr=150.0, dr=1.0):
    for path in [CONFIG_YAML, os.path.join(BRINGUP_SHARE, "config/franka_gazebo_controllers.yaml"), os.path.join(WS_ROOT, "src/franka_gazebo_overrides/franka_gazebo_controllers.yaml")]:
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


def restore_yaml_backups():
    for orig, bak in [
        (os.path.join(WS_ROOT, "src/franka_gazebo_overrides/franka_gazebo_controllers.yaml"),
         os.path.join(WS_ROOT, "src/franka_gazebo_overrides/franka_gazebo_controllers.yaml.bak_prodmp1007")),
        (os.path.join(FRANKA_WS, "src/franka_ros2/franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml"),
         os.path.join(FRANKA_WS, "src/franka_ros2/franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml.bak_prodmp1007")),
        (os.path.join(FRANKA_WS, "install/franka_gazebo_bringup/share/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml"),
         os.path.join(FRANKA_WS, "install/franka_gazebo_bringup/share/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml.bak_prodmp1007"))
    ]:
        if os.path.exists(bak):
            shutil.copyfile(bak, orig)


def run_simulation(run_name="reach_task_baseline_replay_prodmp_kt2000_delay1_rerun1007b", kt=2000.0, dt=10.0, kr=150.0, dr=1.0):
    print(f"\n>>> AVVIO SIMULAZIONE: {run_name} (ProDMP + Impedance Kt={kt})", flush=True)
    clean_processes()
    setup_bringup_share()
    set_impedance_gains(kt, dt, kr, dr)

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

    # Wait for controller activation (max 45s)
    activated = False
    check_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {FRANKA_WS}/install/setup.bash && "
        f"ros2 control list_controllers 2>/dev/null"
    )
    for _ in range(45):
        if os.path.exists(launch_log_path):
            with open(launch_log_path, "r") as f:
                log_text = f.read()
            if "Configured and activated cartesian_impedance_controller" in log_text or "Successfully loaded controller cartesian_impedance_controller" in log_text:
                activated = True
                break
        res = subprocess.run(check_cmd, shell=True, executable="/bin/bash", capture_output=True, text=True)
        if "cartesian_impedance_controller" in res.stdout and "active" in res.stdout:
            activated = True
            break
        time.sleep(1.0)

    if not activated:
        print(f"[ERRORE] Controller non attivato per {run_name}", flush=True)
        clean_processes()
        restore_yaml_backups()
        return None

    time.sleep(2.0)

    # Start bag recording
    bag_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"ros2 bag record -o {bag_dir} "
        f"/cartesian_impedance_controller/target_pose_aligned "
        f"/cartesian_impedance_controller/actual_pose "
        f"/joint_states "
        f"/clock"
    )
    bag_proc = subprocess.Popen(bag_cmd, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
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
    with open(exec_log_path, "w") as exec_f:
        subprocess.run(exec_cmd, shell=True, executable="/bin/bash", stdout=exec_f, stderr=subprocess.STDOUT)

    time.sleep(2.0)

    # Stop recording cleanly
    if bag_proc.poll() is None:
        bag_proc.send_signal(signal.SIGINT)
        try:
            bag_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            bag_proc.kill()

    clean_processes()
    restore_yaml_backups()

    # Extract Bag to CSV using python script
    export_py = f"""
import sqlite3, glob, os, csv
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

bag_dir = "{bag_dir}"
out_dir = "{DATA_DIR}"
db_path = glob.glob(f"{{bag_dir}}/*.db3")[0]
conn = sqlite3.connect(db_path)
cur = conn.cursor()

cur.execute("SELECT id, name FROM topics")
topics = dict(cur.fetchall())
topics_rev = {{v: k for k, v in topics.items()}}

if "/cartesian_impedance_controller/target_pose_aligned" in topics_rev:
    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC", (topics_rev["/cartesian_impedance_controller/target_pose_aligned"],))
    rows = cur.fetchall()
    t0 = rows[0][0] * 1e-9
    with open(os.path.join(out_dir, "target_aligned_{run_name}.csv"), "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "x", "y", "z", "qw", "qx", "qy", "qz"])
        for ts_ns, data in rows:
            p = deserialize_message(data, PoseStamped)
            t = ts_ns * 1e-9 - t0
            writer.writerow([f"{{t:.6f}}", p.pose.position.x, p.pose.position.y, p.pose.position.z, p.pose.orientation.w, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z])

if "/cartesian_impedance_controller/actual_pose" in topics_rev:
    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC", (topics_rev["/cartesian_impedance_controller/actual_pose"],))
    rows = cur.fetchall()
    with open(os.path.join(out_dir, "actual_pose_{run_name}.csv"), "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "x", "y", "z", "qw", "qx", "qy", "qz"])
        for ts_ns, data in rows:
            p = deserialize_message(data, PoseStamped)
            t = ts_ns * 1e-9 - t0
            writer.writerow([f"{{t:.6f}}", p.pose.position.x, p.pose.position.y, p.pose.position.z, p.pose.orientation.w, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z])

if "/joint_states" in topics_rev:
    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC", (topics_rev["/joint_states"],))
    rows = cur.fetchall()
    joint_names = [f"fer_joint{{i+1}}" for i in range(7)]
    with open(os.path.join(out_dir, "joint_states_{run_name}.csv"), "w", newline="") as f:
        writer = csv.writer(f)
        header = ["t"] + [f"q{{i+1}}" for i in range(7)] + [f"dq{{i+1}}" for i in range(7)] + [f"effort{{i+1}}" for i in range(7)]
        writer.writerow(header)
        for ts_ns, data in rows:
            js = deserialize_message(data, JointState)
            t = ts_ns * 1e-9 - t0
            q, dq, eff = [], [], []
            for jn in joint_names:
                if jn in js.name:
                    idx = js.name.index(jn)
                    q.append(js.position[idx])
                    dq.append(abs(js.velocity[idx]) if len(js.velocity) > idx else 0.0)
                    eff.append(js.effort[idx] if len(js.effort) > idx else 0.0)
                else:
                    q.append(0.0); dq.append(0.0); eff.append(0.0)
            writer.writerow([f"{{t:.6f}}"] + q + dq + eff)

conn.close()
"""
    subprocess.run(["python3", "-c", export_py], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return evaluate_metrics(run_name)


def evaluate_metrics(run_name):
    tp_path = os.path.join(DATA_DIR, f"target_aligned_{run_name}.csv")
    ap_path = os.path.join(DATA_DIR, f"actual_pose_{run_name}.csv")

    if not os.path.exists(tp_path) or not os.path.exists(ap_path):
        print(f"[ERRORE] File dati non trovati per {run_name}", flush=True)
        return None

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

    # Tau nominale
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
        from rosgraph_msgs.msg import Clock
        c_first = deserialize_message(c_rows[0][1], Clock).clock
        c_last = deserialize_message(c_rows[-1][1], Clock).clock
        t_sim_dur = (c_last.sec + c_last.nanosec * 1e-9) - (c_first.sec + c_first.nanosec * 1e-9)
        t_wall_dur = (c_rows[-1][0] - c_rows[0][0]) * 1e-9
        rtf = t_sim_dur / t_wall_dur if t_wall_dur > 0 else 1.0
    else:
        rtf = 1.0
    conn.close()

    return {
        "run": run_name,
        "t_sim_ultimo": t_sim_ultimo,
        "frazione": frazione,
        "n_gaps": n_gaps,
        "rtf": rtf,
        "z_t0": z_t0,
        "err_t0": err_t0,
        "pos_media": pos_media,
        "pos_mediana": pos_mediana,
        "pos_p95": pos_p95,
        "pos_max": pos_max,
        "t_max": t_max,
        "pos_max_1s": pos_max_1s,
        "pos_finale_ultimo": pos_finale_ultimo,
        "pos_finale_tau": pos_finale_tau,
        "ori_media": ori_media,
        "ori_max": ori_max,
        "ori_finale": ori_finale,
        "tau_nom": tau_nom
    }


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else "all"
    
    if target in ["2000", "all"]:
        res_2000 = run_simulation("reach_task_baseline_replay_prodmp_kt2000_delay1_rerun1007b", kt=2000.0, dt=10.0, kr=150.0, dr=1.0)
    else:
        res_2000 = None

    if target in ["200", "all"]:
        res_200 = run_simulation("reach_task_baseline_replay_prodmp_kt200_delay1_rerun1007b", kt=200.0, dt=10.0, kr=150.0, dr=1.0)
    else:
        res_200 = None

    print("\n" + "=" * 120)
    print("CSV OUTPUT RERUN 1007b:")
    print("run,config,t_sim_ultimo_s,frazione_completata,n_gaps_gt3dt,rtf_gz_stats,z_t0_m,err_t0_mm,pos_media_mm,pos_mediana_mm,pos_p95_mm,pos_max_mm,t_max_s,pos_max_dopo_1s_mm,pos_finale_ultimo_mm,pos_finale_a_tau_mm,ori_media_deg,ori_max_deg,ori_finale_deg")
    for res in [res_2000, res_200]:
        if res:
            cfg = "ProDMP_impedance_kt2000" if "kt2000" in res["run"] else "ProDMP_impedance_kt200"
            print(f"{res['run']},{cfg},{res['t_sim_ultimo']:.3f},{res['frazione']:.3f},{res['n_gaps']},{res['rtf']:.4f},{res['z_t0']:.6f},{res['err_t0']:.2f},{res['pos_media']:.2f},{res['pos_mediana']:.2f},{res['pos_p95']:.2f},{res['pos_max']:.2f},{res['t_max']:.3f},{res['pos_max_1s']:.2f},{res['pos_finale_ultimo']:.2f},{res['pos_finale_tau']:.2f},{res['ori_media']:.3f},{res['ori_max']:.3f},{res['ori_finale']:.3f}")


if __name__ == "__main__":
    main()
