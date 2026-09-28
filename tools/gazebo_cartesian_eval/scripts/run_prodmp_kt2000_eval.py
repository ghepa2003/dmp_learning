#!/usr/bin/env python3
"""
run_prodmp_kt2000_eval.py — Esecuzione e valutazione della Reach Baseline con ProDMP + Impedance Kt=2000.
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
            elif "rotational_stiffness:" in line and "rotational_stiffness:" in line:
                new_lines.append(f"    rotational_stiffness: {kr:.1f}\n")
            elif "rotational_damping:" in line:
                new_lines.append(f"    rotational_damping: {dr:.1f}\n")
            else:
                new_lines.append(line)
        with open(path, "w") as f:
            f.writelines(new_lines)


def run_simulation(run_name="reach_task_baseline_replay_prodmp_kt2000_delay1", kt=2000.0, dt=10.0, kr=150.0, dr=1.0):
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
        return None

    time.sleep(2.0)

    # Start bag recording
    bag_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"ros2 bag record -o {bag_dir} "
        f"/cartesian_impedance_controller/target_pose_aligned "
        f"/cartesian_impedance_controller/actual_pose "
        f"/joint_states"
    )
    bag_proc = subprocess.Popen(bag_cmd, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)

    # Run ProDMP executor
    exec_log_path = f"/tmp/exec_{run_name}.log"
    exec_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {FRANKA_WS}/install/setup.bash && "
        f"source {WS_ROOT}/install/setup.bash && "
        f"timeout 75 ros2 run haptic_dmp_learning prodmp_gazebo_executor_node "
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

    # Extract Bag to CSV
    extract_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"python3 {EVAL_DIR}/scripts/extract_bag_to_csv.py {bag_dir} {run_name} cartesian_impedance_controller && "
        f"python3 {EVAL_DIR}/scripts/manipulability_check/extract_joint_states_to_csv.py {bag_dir} {run_name} {DATA_DIR}"
    )
    subprocess.run(extract_cmd, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Generate headless 3-panel plot
    eval_cmd = (
        f"python3 {EVAL_DIR}/scripts/evaluate_cartesian_tracking_headless.py {run_name}"
    )
    subprocess.run(eval_cmd, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Copy plot to 02_gazebo_tracking as well
    src_plot = os.path.join(EVAL_DIR, f"plots/03_gain_sweep/cartesian_tracking_{run_name}.png")
    dst_plot = os.path.join(EVAL_DIR, f"plots/02_gazebo_tracking/cartesian_tracking_{run_name}.png")
    if os.path.exists(src_plot):
        shutil.copyfile(src_plot, dst_plot)

    # Calculate metrics at tau and final
    return evaluate_metrics(run_name)


def evaluate_metrics(run_name):
    tp_path = os.path.join(DATA_DIR, f"target_aligned_{run_name}.csv")
    ap_path = os.path.join(DATA_DIR, f"actual_pose_{run_name}.csv")

    def load_csv(path):
        t, x, y, z = [], [], [], []
        qw, qx, qy, qz = [], [], [], []
        with open(path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                t.append(float(row["t"]))
                x.append(float(row["x"])); y.append(float(row["y"])); z.append(float(row["z"]))
                qw.append(float(row["qw"])); qx.append(float(row["qx"]))
                qy.append(float(row["qy"])); qz.append(float(row["qz"]))
        return np.array(t), np.array(x), np.array(y), np.array(z), np.array(qw), np.array(qx), np.array(qy), np.array(qz)

    def quat_angle_between(q1, q2):
        dot = abs(sum(a * b for a, b in zip(q1, q2)))
        dot = max(-1.0, min(1.0, dot))
        return 2.0 * math.degrees(math.acos(dot))

    def find_clamp_index(t_x, t_y, t_z, tol_m=1e-5):
        n = len(t_x)
        for i in range(n - 1, 0, -1):
            d = math.sqrt((t_x[i] - t_x[i - 1]) ** 2 +
                           (t_y[i] - t_y[i - 1]) ** 2 +
                           (t_z[i] - t_z[i - 1]) ** 2)
            if d >= tol_m:
                return i
        return 0

    t_t, t_x, t_y, t_z, t_qw, t_qx, t_qy, t_qz = load_csv(tp_path)
    a_t, a_x, a_y, a_z, a_qw, a_qx, a_qy, a_qz = load_csv(ap_path)

    pos_errs, ang_errs = [], []
    for i in range(len(a_t)):
        j = bisect.bisect_left(t_t, a_t[i])
        j = min(max(j, 0), len(t_t) - 1)
        dp = math.sqrt((a_x[i] - t_x[j]) ** 2 + (a_y[i] - t_y[j]) ** 2 + (a_z[i] - t_z[j]) ** 2) * 1000.0
        pos_errs.append(dp)
        da = quat_angle_between((a_qw[i], a_qx[i], a_qy[i], a_qz[i]),
                                 (t_qw[j], t_qx[j], t_qy[j], t_qz[j]))
        ang_errs.append(da)

    clamp_idx = find_clamp_index(t_x, t_y, t_z)
    t_clamp = t_t[clamp_idx]
    j_tau = bisect.bisect_left(a_t, t_clamp)
    j_tau = min(max(j_tau, 0), len(a_t) - 1)

    t_final = a_t[-1]
    err_pos_tau = pos_errs[j_tau]
    err_ori_tau = ang_errs[j_tau]
    err_pos_final = pos_errs[-1]
    err_ori_final = ang_errs[-1]

    mask_cruise = (a_t >= 2.0) & (a_t <= (a_t[-1] - 3.0))
    err_pos_mean = np.mean(np.array(pos_errs)[mask_cruise])
    err_ori_mean = np.mean(np.array(ang_errs)[mask_cruise])

    return {
        "run": run_name,
        "t_clamp_s": t_clamp,
        "err_pos_at_tau_mm": err_pos_tau,
        "err_ori_at_tau_deg": err_ori_tau,
        "t_final_s": t_final,
        "err_pos_at_final_mm": err_pos_final,
        "err_ori_at_final_deg": err_ori_final,
        "err_pos_mean_cruise_mm": err_pos_mean,
        "err_ori_mean_cruise_deg": err_ori_mean,
    }


def main():
    res_200 = run_simulation("reach_task_baseline_replay_prodmp_kt200_delay1", kt=200.0, dt=10.0, kr=150.0, dr=1.0)
    res_2000 = run_simulation("reach_task_baseline_replay_prodmp_kt2000_delay1", kt=2000.0, dt=10.0, kr=150.0, dr=1.0)
    
    print("\n" + "=" * 110)
    print("RISULTATI FINALI CON RESET PULITO (CSV PURO)")
    print("=" * 110)
    print("run,t_clamp_s,err_pos_at_tau_mm,err_ori_at_tau_deg,t_final_s,err_pos_at_final_mm,err_ori_at_final_deg")
    for res in [res_200, res_2000]:
        if res:
            print(f"{res['run']},{res['t_clamp_s']:.3f},{res['err_pos_at_tau_mm']:.2f},{res['err_ori_at_tau_deg']:.3f},{res['t_final_s']:.3f},{res['err_pos_at_final_mm']:.2f},{res['err_ori_at_final_deg']:.3f}")
            print(f"   -> Media in crociera: err_pos_mean={res['err_pos_mean_cruise_mm']:.2f}mm, err_ori_mean={res['err_ori_mean_cruise_deg']:.3f}deg")


if __name__ == "__main__":
    main()
