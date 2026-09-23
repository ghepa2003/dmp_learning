#!/usr/bin/env python3
"""
run_satellite_rotation_experiment.py
Runs prodmp_gazebo_executor_node with satellite_rotation_enabled in Gazebo,
records bags, extracts CSVs, and computes:
- err_pos_at_tau_mm, err_ori_at_tau_deg
- err_pos_at_final_mm, err_ori_at_final_deg
- w_trans_at_final (Pinocchio)
- q_final_deg
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
import pinocchio as pin

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
URDF_PATH = os.path.join(WS_ROOT, "fer_flat_effort.urdf")

# tau del ProDMP in WEIGHTS_PRODMP (vedi weights.yaml: tau: 60.972930046).
TAU_SEC = 60.972930046

# RTF misurato su 3 run identiche con record_bag.sh + --max-cache-size 100MB
# (verifica del 22/09/2026): stabile a ~0.678. RTF_REFERENCE e' tenuto un filo
# SOTTO il valore misurato (mai sopra) per avere margine di sicurezza: usare un
# RTF di riferimento piu' alto di quello reale sottostimerebbe il timeout e
# rischierebbe di troncare il rollout come nel bug osservato con "timeout 75".
RTF_REFERENCE = 0.65

# Margine moltiplicativo sul tempo wall-clock stimato, per assorbire variazioni
# di RTF tra run diverse (fasi diverse del rollout possono avere carico
# leggermente differente, es. picchi di calcolo/IO non catturati dalla media
# misurata) oltre al tempo fisso di startup_delay_sec del nodo (default 1.0s,
# non sovrascritto in questo script) prima che il DMP inizi a muoversi.
TIMEOUT_MARGIN_FACTOR = 1.3

# startup_delay_sec di prodmp_gazebo_executor_node: non e' passato come
# parametro in exec_cmd, quindi il nodo usa il suo default (vedi
# prodmp_gazebo_executor_node.cpp: declare_parameter<double>("startup_delay_sec", 1.0)).
STARTUP_DELAY_SEC = 1.0


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


def run_test(run_name, phase_deg, axis, center, kt=2000.0):
    print(f"\n=======================================================", flush=True)
    print(f">>> START TEST: {run_name} (Kt={kt})", flush=True)
    print(f"    phase: {phase_deg} deg, axis: {axis}, center: {center}", flush=True)
    print(f"=======================================================", flush=True)

    clean_processes()
    setup_bringup_share()
    set_impedance_gains(kt=kt, dt=10.0, kr=150.0, dr=1.0)

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
        print(f"[ERROR] Controller not activated for {run_name}", flush=True)
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

    # Format axis and center parameters for ros2 run
    axis_str = f"[{axis[0]}, {axis[1]}, {axis[2]}]"
    center_str = f"[{center[0]}, {center[1]}, {center[2]}]"

    # Sia tau che startup_delay_sec avanzano in sim time (use_sim_time:=true),
    # quindi il tempo wall-clock necessario a coprirli e' (tau + startup_delay_sec)
    # / RTF_REFERENCE; TIMEOUT_MARGIN_FACTOR aggiunge margine per la variabilita'
    # di RTF osservabile tra run/fasi diverse.
    timeout_s = math.ceil((TAU_SEC + STARTUP_DELAY_SEC) / RTF_REFERENCE * TIMEOUT_MARGIN_FACTOR)

    exec_log_path = f"/tmp/exec_{run_name}.log"
    exec_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {FRANKA_WS}/install/setup.bash && "
        f"source {WS_ROOT}/install/setup.bash && "
        f"timeout {timeout_s} ros2 run haptic_dmp_learning prodmp_gazebo_executor_node "
        f"--ros-args "
        f"-p target_pose_topic:=/target_pose "
        f"-p target_twist_topic:=/target_twist "
        f"-p weights_yaml_path:={WEIGHTS_PRODMP} "
        f"-p orientation_weights_yaml_path:={ORIENTATION_PATH} "
        f"-p target_odom_required:=false "
        f"-p satellite_rotation_enabled:=true "
        f"-p satellite_rotation_frozen_phase_deg:={phase_deg} "
        f"-p satellite_rotation_axis:=\"{axis_str}\" "
        f"-p satellite_rotation_center:=\"{center_str}\" "
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

    return evaluate_metrics(run_name, phase_deg)


def evaluate_metrics(run_name, phase_deg):
    tp_path = os.path.join(DATA_DIR, f"target_aligned_{run_name}.csv")
    ap_path = os.path.join(DATA_DIR, f"actual_pose_{run_name}.csv")
    js_path = os.path.join(DATA_DIR, f"joint_states_{run_name}.csv")

    if not os.path.exists(tp_path) or not os.path.exists(ap_path) or not os.path.exists(js_path):
        print(f"[ERROR] Missing CSV files for {run_name}", flush=True)
        return None

    def load_pose_csv(path):
        t, x, y, z, qw, qx, qy, qz = [], [], [], [], [], [], [], []
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

    def find_clamp_index(t_x, t_y, t_z, tol_m=1e-4):
        n = len(t_x)
        final_pos = (t_x[-1], t_y[-1], t_z[-1])
        for i in range(n - 1, -1, -1):
            d = math.sqrt((t_x[i] - final_pos[0])**2 + (t_y[i] - final_pos[1])**2 + (t_z[i] - final_pos[2])**2)
            if d > tol_m:
                return i + 1
        return 0

    t_t, t_x, t_y, t_z, t_qw, t_qx, t_qy, t_qz = load_pose_csv(tp_path)
    a_t, a_x, a_y, a_z, a_qw, a_qx, a_qy, a_qz = load_pose_csv(ap_path)

    pos_errs, ang_errs = [], []
    for i in range(len(a_t)):
        j = bisect.bisect_left(t_t, a_t[i])
        j = min(max(j, 0), len(t_t) - 1)
        dp = math.sqrt((a_x[i] - t_x[j])**2 + (a_y[i] - t_y[j])**2 + (a_z[i] - t_z[j])**2) * 1000.0
        pos_errs.append(dp)
        da = quat_angle_between((a_qw[i], a_qx[i], a_qy[i], a_qz[i]),
                                 (t_qw[j], t_qx[j], t_qy[j], t_qz[j]))
        ang_errs.append(da)

    clamp_idx = find_clamp_index(t_x, t_y, t_z)
    t_clamp = t_t[clamp_idx]
    j_tau = bisect.bisect_left(a_t, t_clamp)
    j_tau = min(max(j_tau, 0), len(a_t) - 1)

    err_pos_tau = pos_errs[j_tau]
    err_ori_tau = ang_errs[j_tau]
    err_pos_final = pos_errs[-1]
    err_ori_final = ang_errs[-1]

    # Pinocchio w_trans at final joint state
    model = pin.buildModelFromUrdf(URDF_PATH)
    data = model.createData()
    frame_id = model.getFrameId("fer_link8")

    joint_names = [f"fer_joint{i}" for i in range(1, 8)]
    q_final = []
    with open(js_path) as f:
        rows = list(csv.DictReader(f))
        last_row = rows[-1]
        q_final = [float(last_row[jn]) for jn in joint_names]

    q_arr = np.array(q_final)
    pin.computeJointJacobians(model, data, q_arr)
    pin.framesForwardKinematics(model, data, q_arr)
    J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    J_p = J[:3, :]
    w_trans = math.sqrt(max(0.0, np.linalg.det(J_p @ J_p.T)))
    q_deg = [math.degrees(v) for v in q_final]

    res = {
        "run": run_name,
        "phase_deg": phase_deg,
        "t_clamp": t_clamp,
        "err_pos_at_tau_mm": err_pos_tau,
        "err_ori_at_tau_deg": err_ori_tau,
        "err_pos_at_final_mm": err_pos_final,
        "err_ori_at_final_deg": err_ori_final,
        "w_trans_at_final": w_trans,
        "q_deg": q_deg,
    }

    print(f"Results for {run_name}:")
    print(f"  err_pos_at_tau_mm  : {err_pos_tau:.3f} mm")
    print(f"  err_ori_at_tau_deg : {err_ori_tau:.3f} deg")
    print(f"  err_pos_at_final_mm: {err_pos_final:.3f} mm")
    print(f"  err_ori_at_final_deg: {err_ori_final:.3f} deg")
    print(f"  w_trans_at_final   : {w_trans:.4f}")
    print(f"  q_deg              : {[round(x, 2) for x in q_deg]}")

    return res


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "stage_a"
    if mode == "stage_a":
        # STAGE A: Sanity check at phase 0
        run_name = "reach_task_satellite_rot_phase0"
        axis = [0.0, 0.0, 1.0]
        center = [0.0, 0.0, 0.0]
        res = run_test(run_name, 0.0, axis, center)
    elif mode == "stage_b":
        # STAGE B: Non-zero phases: 30, 90, 180, 270 (with safe reachable workspace geometry at Kt=2000)
        axis = [0.0, 1.0, 0.0]
        center = [0.45, -0.05, 0.35]
        phases = [30.0, 90.0, 180.0, 270.0]
        all_res = []
        for p in phases:
            run_name = f"reach_task_satellite_rot_phase{int(p)}"
            r = run_test(run_name, p, axis, center, kt=2000.0)
            if r:
                all_res.append(r)
        
        print("\n" + "="*80)
        print("SUMMARY CSV (Kt=2000)")
        print("="*80)
        print("frozen_phase_deg,err_pos_at_tau_mm,err_ori_at_tau_deg,err_pos_at_final_mm,err_ori_at_final_deg,w_trans_at_final,q1_deg,q2_deg,q3_deg,q4_deg,q5_deg,q6_deg,q7_deg")
        for r in all_res:
            q = r["q_deg"]
            print(f"{r['phase_deg']:.1f},{r['err_pos_at_tau_mm']:.3f},{r['err_ori_at_tau_deg']:.3f},{r['err_pos_at_final_mm']:.3f},{r['err_ori_at_final_deg']:.3f},{r['w_trans_at_final']:.4f},{q[0]:.2f},{q[1]:.2f},{q[2]:.2f},{q[3]:.2f},{q[4]:.2f},{q[5]:.2f},{q[6]:.2f}")
    elif mode == "stage_b_kt200":
        # STAGE B: Non-zero phases: 30, 90, 180, 270 (at Kt=200)
        axis = [0.0, 1.0, 0.0]
        center = [0.45, -0.05, 0.35]
        phases = [30.0, 90.0, 180.0, 270.0]
        all_res = []
        for p in phases:
            run_name = f"reach_task_satellite_rot_phase{int(p)}_kt200"
            r = run_test(run_name, p, axis, center, kt=200.0)
            if r:
                all_res.append(r)
        
        print("\n" + "="*80)
        print("SUMMARY CSV (Kt=200)")
        print("="*80)
        print("frozen_phase_deg,err_pos_at_tau_mm,err_ori_at_tau_deg,err_pos_at_final_mm,err_ori_at_final_deg,w_trans_at_final,q1_deg,q2_deg,q3_deg,q4_deg,q5_deg,q6_deg,q7_deg")
        for r in all_res:
            q = r["q_deg"]
            print(f"{r['phase_deg']:.1f},{r['err_pos_at_tau_mm']:.3f},{r['err_ori_at_tau_deg']:.3f},{r['err_pos_at_final_mm']:.3f},{r['err_ori_at_final_deg']:.3f},{r['w_trans_at_final']:.4f},{q[0]:.2f},{q[1]:.2f},{q[2]:.2f},{q[3]:.2f},{q[4]:.2f},{q[5]:.2f},{q[6]:.2f}")

