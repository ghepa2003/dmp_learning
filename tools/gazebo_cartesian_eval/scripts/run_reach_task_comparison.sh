#!/bin/bash
# run_reach_task_comparison.sh — Confronto tra Cartesian Velocity Control e Cartesian Impedance Control
# su una traiettoria DMP specificata.

set -euo pipefail

THESIS_WS="${THESIS_WS:-/root/thesis_ws}"
FRANKA_WS="${FRANKA_WS:-/root/ros_workspaces/ros2/franka_ws}"
TOOLS_DIR="$THESIS_WS/tools/gazebo_cartesian_eval"
DMP_WEIGHTS="${1:-$THESIS_WS/real_trajA_ridge_filter.yaml}"
ROLLOUT_TIMEOUT=75

# Nome base della traiettoria per i file di output
TRAJ_NAME=$(basename "$DMP_WEIGHTS" .yaml)

mkdir -p "$TOOLS_DIR/bags" "$TOOLS_DIR/data" "$TOOLS_DIR/plots/02_gazebo_tracking"

kill_everything_hard () {
    pkill -9 -f "ign gazebo" 2>/dev/null || true
    pkill -9 -f "gz_sim" 2>/dev/null || true
    pkill -9 -f "ros2 launch franka_gazebo_bringup" 2>/dev/null || true
    pkill -9 -f "robot_state_publisher" 2>/dev/null || true
    pkill -9 -f "joint_state_publisher" 2>/dev/null || true
    pkill -9 -f "dmp_gazebo_executor_node" 2>/dev/null || true
    pkill -9 -f "log_nullspace_leak.py" 2>/dev/null || true
    pkill -9 -f "ros2 bag record" 2>/dev/null || true
    pkill -9 -f rviz2 2>/dev/null || true
    sleep 3
}

wait_for_clean_ros2_graph () {
    local max_wait=40
    local waited=0
    while true; do
        local node_count
        node_count=$(ros2 node list 2>/dev/null | wc -l)
        if [ "$node_count" -eq 0 ]; then
            return 0
        fi
        sleep 1
        waited=$((waited + 1))
        if [ "$waited" -ge "$max_wait" ]; then
            echo "  [WARNING] Graph non pulito dopo ${max_wait}s, riavvio daemon ROS 2..."
            ros2 daemon stop > /dev/null 2>&1 || true
            sleep 2
            ros2 daemon start > /dev/null 2>&1 || true
            sleep 3
            return 1
        fi
    done
}

run_simulation () {
    local controller_type="$1"       # "velocity" o "impedance"
    local controller_name="$2"       # "velocity_cartesian_controller" o "cartesian_impedance_controller"
    local launch_file="$3"           # nome del file launch
    local run_name="${TRAJ_NAME}_${controller_type}"
    local launch_log="/tmp/launch_log_${run_name}.txt"
    local bag_dir="$TOOLS_DIR/bags/${run_name}"

    echo ""
    echo "=================================================================="
    echo "=== AVVIO SIMULAZIONE: ${controller_type} (${controller_name}) ==="
    echo "=== Traiettoria: ${DMP_WEIGHTS} ==="
    echo "=================================================================="

    kill_everything_hard
    wait_for_clean_ros2_graph

    # 1. Launch Gazebo Headless
    ros2 launch franka_gazebo_bringup "$launch_file" headless:=true > "$launch_log" 2>&1 &
    local launch_pid=$!

    # 2. Attendi caricamento controller
    local waited=0
    while ! grep -q "Successfully loaded controller ${controller_name}" "$launch_log" 2>/dev/null; do
        if ! kill -0 "$launch_pid" 2>/dev/null; then
            echo "  [ERROR] Processo launch terminato prematuramente per ${run_name}"
            kill_everything_hard
            return 1
        fi
        sleep 1
        waited=$((waited + 1))
        if [ "$waited" -ge 45 ]; then
            echo "  [ERROR] Timeout attesa avvio controller per ${run_name}"
            kill -9 "$launch_pid" 2>/dev/null || true
            kill_everything_hard
            return 1
        fi
    done
    echo "  [OK] Controller ${controller_name} caricato e attivo."
    sleep 3

    # 3. Avvia registrazione ROS2 Bag
    rm -rf "$bag_dir"
    ros2 bag record \
        "/${controller_name}/target_pose_aligned" \
        "/${controller_name}/actual_pose" \
        /joint_states \
        -o "$bag_dir" > /dev/null 2>&1 &
    local bag_pid=$!
    sleep 1

    # 4. Esegui la traiettoria DMP
    echo "  [INFO] Esecuzione DMP ($DMP_WEIGHTS)..."
    timeout "${ROLLOUT_TIMEOUT}" ros2 run haptic_dmp_learning dmp_gazebo_executor_node \
        --ros-args -p weights_yaml_path:="$DMP_WEIGHTS" > "/tmp/executor_log_${run_name}.txt" 2>&1 || true

    sleep 2
    echo "  [INFO] Fermo registrazione bag..."
    kill -INT "$bag_pid" 2>/dev/null || true
    wait "$bag_pid" 2>/dev/null || true

    # 5. Estrai dati dal Bag
    echo "  [INFO] Estrazione dati bag in CSV..."
    python3 "$TOOLS_DIR/scripts/extract_bag_to_csv.py" \
        "$bag_dir" "$run_name" "$controller_name" > "/tmp/extract_log_${run_name}.txt" 2>&1

    # 6. Valuta il tracking ed elabora plot + metriche
    echo "  [INFO] Valutazione tracking..."
    python3 - "$run_name" "$TOOLS_DIR" <<'PYEOF'
import sys, os, csv, math, bisect
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

run_name = sys.argv[1]
tools_dir = sys.argv[2]
data_dir = os.path.join(tools_dir, "data")
target_path = os.path.join(data_dir, f"target_aligned_{run_name}.csv")
actual_path = os.path.join(data_dir, f"actual_pose_{run_name}.csv")

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
    return t, x, y, z, qw, qx, qy, qz

def quat_angle_between(q1, q2):
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.degrees(math.acos(dot))

def cumulative_and_net_angle(qw, qx, qy, qz):
    quats = list(zip(qw, qx, qy, qz))
    cum, max_step = 0.0, 0.0
    for i in range(1, len(quats)):
        ang = quat_angle_between(quats[i - 1], quats[i])
        cum += ang
        if ang > max_step:
            max_step = ang
    net = quat_angle_between(quats[0], quats[-1])
    return cum, net, max_step

def tracking_errors(target, actual):
    t_t, t_x, t_y, t_z, t_qw, t_qx, t_qy, t_qz = target
    a_t, a_x, a_y, a_z, a_qw, a_qx, a_qy, a_qz = actual
    pos_errs, ang_errs = [], []
    for i in range(len(a_t)):
        j = bisect.bisect_left(t_t, a_t[i])
        j = min(max(j, 0), len(t_t) - 1)
        dp = math.sqrt((a_x[i] - t_x[j]) ** 2 + (a_y[i] - t_y[j]) ** 2 + (a_z[i] - t_z[j]) ** 2)
        pos_errs.append(dp)
        da = quat_angle_between((a_qw[i], a_qx[i], a_qy[i], a_qz[i]),
                                 (t_qw[j], t_qx[j], t_qy[j], t_qz[j]))
        ang_errs.append(da)
    return pos_errs, ang_errs

target = load_csv(target_path)
actual = load_csv(actual_path)

t_cum, t_net, t_max = cumulative_and_net_angle(*target[4:8])
a_cum, a_net, a_max = cumulative_and_net_angle(*actual[4:8])
pos_errs, ang_errs = tracking_errors(target, actual)

mean_pos_mm = sum(pos_errs) / len(pos_errs) * 1000
max_pos_mm = max(pos_errs) * 1000
final_pos_mm = pos_errs[-1] * 1000
mean_ang_deg = sum(ang_errs) / len(ang_errs)
max_ang_deg = max(ang_errs)
final_ang_deg = ang_errs[-1]

print(f"\n================ METRICHE: {run_name} ================")
print(f"[Target] Rotazione cumulativa = {t_cum:.2f}° | Netta = {t_net:.2f}° | Rapporto = {t_cum/t_net:.3f}")
print(f"[Actual] Rotazione cumulativa = {a_cum:.2f}° | Netta = {a_net:.2f}° | Rapporto = {a_cum/a_net:.3f}")
print(f"Errore Posizione:  Media = {mean_pos_mm:.2f} mm | Max = {max_pos_mm:.2f} mm | Finale = {final_pos_mm:.2f} mm")
print(f"Errore Angolare:   Media = {mean_ang_deg:.3f}°  | Max = {max_ang_deg:.3f}°  | Finale = {final_ang_deg:.3f}°")
print("=========================================================\n")

# Plot 3 subplot
fig = plt.figure(figsize=(16, 6))
fig.suptitle(f"Cartesian Tracking: {run_name}", fontsize=14)

ax3d = fig.add_subplot(1, 3, 1, projection="3d")
ax3d.plot(target[1], target[2], target[3], label="Target (DMP comandato)", linewidth=2, color="blue")
ax3d.plot(actual[1], actual[2], actual[3], "--", label="Actual (Gazebo)", linewidth=2, color="orange")
ax3d.set_xlabel("x [m]"); ax3d.set_ylabel("y [m]"); ax3d.set_zlabel("z [m]")
ax3d.set_title("3D Position Trajectory")
ax3d.legend()

ax_t = fig.add_subplot(1, 3, 2)
axes_labels = ["x", "y", "z"]
colors = ["tab:blue", "tab:orange", "tab:green"]
for i, (label, color) in enumerate(zip(axes_labels, colors)):
    ax_t.plot(target[0], target[i + 1], color=color, linestyle="-", label=f"target {label}")
    ax_t.plot(actual[0], actual[i + 1], color=color, linestyle="--", alpha=0.7)
ax_t.set_xlabel("t [s]"); ax_t.set_ylabel("position [m]")
ax_t.set_title("Position: Target (solid) vs Actual (dashed)")
ax_t.legend()

ax_q = fig.add_subplot(1, 3, 3)
q_labels = ["qw", "qx", "qy", "qz"]
q_colors = ["purple", "tab:blue", "tab:orange", "tab:green"]
for i, (label, color) in enumerate(zip(q_labels, q_colors)):
    ax_q.plot(target[0], target[i + 4], color=color, linestyle="-", label=f"target {label}")
    ax_q.plot(actual[0], actual[i + 4], color=color, linestyle="--", alpha=0.7)
ax_q.set_xlabel("t [s]"); ax_q.set_ylabel("quaternion components")
ax_q.set_title("Orientation: Target (solid) vs Actual (dashed)")
ax_q.legend()

plt.tight_layout()
out_dir = os.path.join(tools_dir, "plots", "02_gazebo_tracking")
os.makedirs(out_dir, exist_ok=True)
out_path = os.path.join(out_dir, f"cartesian_tracking_{run_name}.png")
plt.savefig(out_path, dpi=150)
print(f"Plot salvato in: {out_path}")
PYEOF

    # 7. Cleanup
    kill -TERM "$launch_pid" 2>/dev/null || true
    sleep 2
    kill_everything_hard
    wait_for_clean_ros2_graph
}

# --- 1. ESECUZIONE CARTESIAN VELOCITY CONTROL ---
run_simulation "cartesian_velocity" "velocity_cartesian_controller" "gazebo_velocity_cartesian_control.launch.py"

# --- 2. ESECUZIONE CARTESIAN IMPEDANCE CONTROL ---
run_simulation "cartesian_impedance" "cartesian_impedance_controller" "gazebo_cartesian_impedance_control_headless.launch.py"

echo ""
echo "=================================================================="
echo "Confronto completato con successo per entrambi i controller!"
echo "=================================================================="
