#!/usr/bin/env python3
"""
run_baseline_ab_fixed.py — Esecuzione e confronto A/B post-fix sulla sola Reach Baseline.
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
WEIGHTS_DMP = os.path.join(WS_ROOT, "reach_task_baseline_nbasis200.yaml")
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
        "pkill -9 -f 'velocity_cartesian_controller'",
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
        "gazebo_velocity_cartesian_control.launch.py",
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


def set_feedforward_yaml(enabled: bool):
    for path in [CONFIG_YAML, os.path.join(BRINGUP_SHARE, "config/franka_gazebo_controllers.yaml")]:
        if not os.path.exists(path):
            continue
        with open(path, "r") as f:
            content = f.read()
        if enabled:
            content = content.replace("feedforward_enabled: false", "feedforward_enabled: true")
        else:
            content = content.replace("feedforward_enabled: true", "feedforward_enabled: false")
        with open(path, "w") as f:
            f.write(content)


def run_single_simulation(run_name, ff_enabled, formulation):
    print(f"\n>>> AVVIO: {run_name} (FF={ff_enabled}, Form={formulation})", flush=True)
    clean_processes()
    set_feedforward_yaml(ff_enabled)

    bag_dir = os.path.join(BAGS_DIR, run_name)
    if os.path.exists(bag_dir):
        shutil.rmtree(bag_dir)

    launch_log_path = f"/tmp/launch_{run_name}.log"
    launch_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {FRANKA_WS}/install/setup.bash && "
        f"source {WS_ROOT}/install/setup.bash && "
        f"export QT_QPA_PLATFORM=offscreen && "
        f"ros2 launch franka_gazebo_bringup gazebo_velocity_cartesian_control.launch.py "
        f"headless:=true load_gripper:=true"
    )
    with open(launch_log_path, "w") as log_f:
        launch_proc = subprocess.Popen(
            launch_cmd, shell=True, executable="/bin/bash", stdout=log_f, stderr=subprocess.STDOUT
        )

    # Wait for controller activation (max 45s)
    activated = False
    for _ in range(45):
        if os.path.exists(launch_log_path):
            with open(launch_log_path, "r") as f:
                log_text = f.read()
            if "Configured and activated velocity_cartesian_controller" in log_text or "Velocity feedforward:" in log_text:
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
        f"/velocity_cartesian_controller/target_pose_aligned "
        f"/velocity_cartesian_controller/actual_pose "
        f"/joint_states"
    )
    bag_proc = subprocess.Popen(bag_cmd, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)

    # Run executor
    exec_log_path = f"/tmp/exec_{run_name}.log"
    if formulation == "prodmp":
        exec_cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"source {FRANKA_WS}/install/setup.bash && "
            f"source {WS_ROOT}/install/setup.bash && "
            f"timeout 32 ros2 run haptic_dmp_learning prodmp_gazebo_executor_node "
            f"--ros-args "
            f"-p target_pose_topic:=/target_pose "
            f"-p target_twist_topic:=/target_twist "
            f"-p weights_yaml_path:={WEIGHTS_PRODMP} "
            f"-p orientation_weights_yaml_path:={ORIENTATION_PATH} "
            f"-p target_odom_required:=false "
            f"-p use_sim_time:=true"
        )
    else:
        exec_cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"source {FRANKA_WS}/install/setup.bash && "
            f"source {WS_ROOT}/install/setup.bash && "
            f"timeout 32 ros2 run haptic_dmp_learning dmp_gazebo_executor_node "
            f"--ros-args "
            f"-p target_pose_topic:=/target_pose "
            f"-p target_twist_topic:=/target_twist "
            f"-p weights_yaml_path:={WEIGHTS_DMP} "
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
        f"python3 {EVAL_DIR}/scripts/extract_bag_to_csv.py {bag_dir} {run_name} velocity_cartesian_controller && "
        f"python3 {EVAL_DIR}/scripts/manipulability_check/extract_joint_states_to_csv.py {bag_dir} {run_name} {DATA_DIR}"
    )
    subprocess.run(extract_cmd, shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    res, series = evaluate_metrics(run_name, return_series=True)
    if res:
        print(f"  --> Risultato: err_pos_mean={res['err_pos_mean_mm']:.2f}mm, err_pos_final={res['err_pos_final_mm']:.2f}mm, err_ori_mean={res['err_ori_mean_deg']:.3f}deg, err_ori_final={res['err_ori_final_deg']:.3f}deg", flush=True)
    return res, series


def evaluate_metrics(run_name, return_series=False):
    tp_path = os.path.join(DATA_DIR, f"target_aligned_{run_name}.csv")
    ap_path = os.path.join(DATA_DIR, f"actual_pose_{run_name}.csv")

    if not (os.path.exists(tp_path) and os.path.exists(ap_path)):
        print(f"[ERRORE] CSV mancanti per {run_name}", flush=True)
        return (None, None) if return_series else None

    def read_csv(p):
        t, pos, quat = [], [], []
        with open(p, "r") as f:
            for r in csv.DictReader(f):
                t.append(float(r["t"]))
                pos.append([float(r["x"]), float(r["y"]), float(r["z"])])
                quat.append([float(r["qx"]), float(r["qy"]), float(r["qz"]), float(r["qw"])])
        return np.array(t), np.array(pos), np.array(quat)

    t_tgt, pos_tgt, q_tgt = read_csv(tp_path)
    t_act, pos_act, q_act = read_csv(ap_path)

    if len(t_tgt) == 0 or len(t_act) == 0:
        print(f"[ERRORE] CSV vuoti per {run_name}", flush=True)
        return (None, None) if return_series else None

    dt = 0.05
    t_min = max(t_tgt[0], t_act[0])
    t_max = min(t_tgt[-1], t_act[-1])
    sample_t = np.arange(t_min, t_max, dt)

    pos_errs = []
    ori_errs = []

    for t in sample_t:
        it = min(max(bisect.bisect_left(t_tgt, t), 0), len(t_tgt) - 1)
        ia = min(max(bisect.bisect_left(t_act, t), 0), len(t_act) - 1)

        ep = np.linalg.norm(pos_tgt[it] - pos_act[ia]) * 1000.0  # mm
        pos_errs.append(ep)

        dot = min(1.0, abs(np.dot(q_tgt[it], q_act[ia])))
        eo = 2.0 * math.acos(dot) * 180.0 / math.pi  # deg
        ori_errs.append(eo)

    pos_errs = np.array(pos_errs)
    ori_errs = np.array(ori_errs)

    # Cruise window: from t=2s to t=t_max-3s
    mask_cruise = (sample_t >= 2.0) & (sample_t <= (sample_t[-1] - 3.0))

    metrics = {
        "err_pos_mean_mm": float(np.mean(pos_errs[mask_cruise])),
        "err_pos_final_mm": float(pos_errs[-1]),
        "err_ori_mean_deg": float(np.mean(ori_errs[mask_cruise])),
        "err_ori_final_deg": float(ori_errs[-1]),
        "n_samples": len(sample_t),
    }
    series = {
        "t": sample_t,
        "err_pos_mm": pos_errs,
        "err_ori_deg": ori_errs,
    }
    return (metrics, series) if return_series else metrics


def main():
    print("=" * 110)
    print("ESECUZIONE CONFRONTO A/B SULLA BASELINE CON FIX AL VELOCITY FEEDFORWARD")
    print("=" * 110)

    setup_bringup_share()

    # 1. Kp-only (feedforward_enabled: false, prodmp)
    res_noff, series_noff = run_single_simulation("vel_eval_Baseline_noff", False, "prodmp")

    # 2. ProDMP with Feedforward (feedforward_enabled: true, prodmp)
    res_ff_prodmp, series_ff_prodmp = run_single_simulation("vel_eval_Baseline_ff_prodmp", True, "prodmp")

    # 3. Standard DMP with Feedforward (feedforward_enabled: true, dmp)
    res_ff_dmp, series_ff_dmp = run_single_simulation("vel_eval_Baseline_ff_dmp", True, "dmp")

    # Calculate reductions vs no-ff
    red_pos_prodmp = ((res_noff["err_pos_mean_mm"] - res_ff_prodmp["err_pos_mean_mm"]) / res_noff["err_pos_mean_mm"]) * 100.0
    red_ori_prodmp = ((res_noff["err_ori_mean_deg"] - res_ff_prodmp["err_ori_mean_deg"]) / res_noff["err_ori_mean_deg"]) * 100.0

    red_pos_dmp = ((res_noff["err_pos_mean_mm"] - res_ff_dmp["err_pos_mean_mm"]) / res_noff["err_pos_mean_mm"]) * 100.0
    red_ori_dmp = ((res_noff["err_ori_mean_deg"] - res_ff_dmp["err_ori_mean_deg"]) / res_noff["err_ori_mean_deg"]) * 100.0

    print("\n" + "=" * 110)
    print("TABELLA COMPARATIVA POST-FIX (CSV PURO)")
    print("=" * 110)
    print("run,feedforward_enabled,mp_formulation,err_pos_mean_mm,err_pos_final_mm,err_ori_mean_deg,err_ori_final_deg,red_pos_mean_pct,red_ori_mean_pct")
    print(f"vel_eval_Baseline_noff,false,prodmp,{res_noff['err_pos_mean_mm']:.2f},{res_noff['err_pos_final_mm']:.2f},{res_noff['err_ori_mean_deg']:.3f},{res_noff['err_ori_final_deg']:.3f},0.0%,0.0%")
    print(f"vel_eval_Baseline_ff_prodmp,true,prodmp,{res_ff_prodmp['err_pos_mean_mm']:.2f},{res_ff_prodmp['err_pos_final_mm']:.2f},{res_ff_prodmp['err_ori_mean_deg']:.3f},{res_ff_prodmp['err_ori_final_deg']:.3f},{red_pos_prodmp:+.1f}%,{red_ori_prodmp:+.1f}%")
    print(f"vel_eval_Baseline_ff_dmp,true,dmp,{res_ff_dmp['err_pos_mean_mm']:.2f},{res_ff_dmp['err_pos_final_mm']:.2f},{res_ff_dmp['err_ori_mean_deg']:.3f},{res_ff_dmp['err_ori_final_deg']:.3f},{red_pos_dmp:+.1f}%,{red_ori_dmp:+.1f}%")

    # Time series comparison: windows across the trajectory for ProDMP
    if series_ff_prodmp:
        t_arr = series_ff_prodmp["t"]
        err_arr = series_ff_prodmp["err_pos_mm"]
        print("\n" + "=" * 110)
        print("SERIE TEMPORALE ERRORE POSIZIONE [mm] (ProDMP FF POST-FIX)")
        print("=" * 110)
        print("finestra_temporale_s,err_pos_mean_mm,err_pos_min_mm,err_pos_max_mm")
        windows = [(2.0, 5.0), (5.0, 10.0), (10.0, 15.0), (15.0, 18.0), (18.0, 21.0)]
        for w_start, w_end in windows:
            mask = (t_arr >= w_start) & (t_arr < w_end)
            if np.any(mask):
                sub = err_arr[mask]
                print(f"[{w_start:.1f} - {w_end:.1f}],{np.mean(sub):.2f},{np.min(sub):.2f},{np.max(sub):.2f}")


if __name__ == "__main__":
    main()
