#!/usr/bin/env bash
set -uo pipefail

# Punti falliti nello sweep principale, da rilanciare isolatamente.
MISSING=("100 30" "400 50" "800 50" "1400 50")

FIXED_ROT_STIFF=20.0
FIXED_ROT_DAMP=3.0
WEIGHTS_YAML="/root/thesis_ws/real_trajA_ridge_filter.yaml"
ROLLOUT_DURATION=70
CONTROLLERS_YAML="/root/ros_workspaces/ros2/franka_ws/src/franka_ros2/franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml"
CONTROLLERS_YAML_BACKUP="/root/thesis_ws/src/franka_gazebo_overrides/franka_gazebo_controllers.yaml"
TOOLS_DIR="$HOME/thesis_ws/tools/gazebo_cartesian_eval"
SUMMARY_CSV="${TOOLS_DIR}/data/translation_sweep_summary.csv"   # APPEND, non sovrascrive

write_gains_to_yaml () {
    local ts="$1" td="$2" rs="$3" rd="$4"
    cp "$CONTROLLERS_YAML_BACKUP" "$CONTROLLERS_YAML"
    python3 << PYEOF
import re
path = "$CONTROLLERS_YAML"
with open(path) as f:
    content = f.read()
content = re.sub(r'(translational_stiffness:\s*)[\d.]+', r'\g<1>' + str(float("$ts")), content)
content = re.sub(r'(translational_damping:\s*)[\d.]+', r'\g<1>' + str(float("$td")), content)
content = re.sub(r'(rotational_stiffness:\s*)[\d.]+', r'\g<1>' + str(float("$rs")), content)
content = re.sub(r'(rotational_damping:\s*)[\d.]+', r'\g<1>' + str(float("$rd")), content)
with open(path, "w") as f:
    f.write(content)
PYEOF
}

kill_everything_hard () {
    pkill -9 -f "ign gazebo" 2>/dev/null
    pkill -9 -f "ros2 launch franka_gazebo_bringup" 2>/dev/null
    pkill -9 -f "robot_state_publisher" 2>/dev/null
    pkill -9 -f "joint_state_publisher" 2>/dev/null
    pkill -9 -f "dmp_gazebo_executor_node" 2>/dev/null
    pkill -9 -f "ros2 bag record" 2>/dev/null
    pkill -9 -f rviz2 2>/dev/null
    sleep 3
}

wait_for_clean_ros2_graph () {
    local max_wait=40 waited=0
    while true; do
        local node_count
        node_count=$(ros2 node list 2>/dev/null | wc -l)
        [ "$node_count" -eq 0 ] && return 0
        sleep 1
        waited=$((waited + 1))
        if [ "$waited" -ge "$max_wait" ]; then
            ros2 daemon stop > /dev/null 2>&1; sleep 2
            ros2 daemon start > /dev/null 2>&1; sleep 3
            return 1
        fi
    done
}

for combo in "${MISSING[@]}"; do
    read -r ts td <<< "$combo"
    run_name="trans_K${ts}_D${td}"
    echo ""
    echo "== ${run_name} =="

    kill_everything_hard
    wait_for_clean_ros2_graph
    write_gains_to_yaml "$ts" "$td" "$FIXED_ROT_STIFF" "$FIXED_ROT_DAMP"

    cd /root/ros_workspaces/ros2/franka_ws
    colcon build --packages-select franka_gazebo_bringup --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release > /dev/null 2>&1
    set +u; source install/setup.bash; set -u

    ros2 launch franka_gazebo_bringup gazebo_cartesian_impedance_control_headless.launch.py \
        > "/tmp/launch_log_${run_name}.txt" 2>&1 &
    launch_pid=$!

    waited=0
    while ! grep -q "Successfully loaded controller cartesian_impedance_controller" "/tmp/launch_log_${run_name}.txt" 2>/dev/null; do
        if ! kill -0 "$launch_pid" 2>/dev/null; then
            echo "  [ERROR] launch morto"; echo "${ts},${td},NA,NA,NA,NA,NA,NA,NA" >> "$SUMMARY_CSV"; kill_everything_hard; continue 2
        fi
        sleep 1; waited=$((waited + 1))
        if [ "$waited" -ge 40 ]; then
            echo "  [ERROR] timeout"; kill -9 "$launch_pid" 2>/dev/null; kill_everything_hard
            echo "${ts},${td},NA,NA,NA,NA,NA,NA,NA" >> "$SUMMARY_CSV"; continue 2
        fi
    done
    sleep 3

    rm -rf "${TOOLS_DIR}/bags/${run_name}"
    ros2 bag record /cartesian_impedance_controller/target_pose_aligned \
        /cartesian_impedance_controller/actual_pose /joint_states \
        -o "${TOOLS_DIR}/bags/${run_name}" > /dev/null 2>&1 &
    bag_pid=$!
    sleep 1

    timeout "${ROLLOUT_DURATION}" ros2 run haptic_dmp_learning dmp_gazebo_executor_node \
        --ros-args -p weights_yaml_path:="$WEIGHTS_YAML" > /tmp/executor_log.txt 2>&1
    sleep 2
    kill -INT "$bag_pid" 2>/dev/null; wait "$bag_pid" 2>/dev/null

    python3 "${TOOLS_DIR}/scripts/extract_bag_to_csv.py" "${TOOLS_DIR}/bags/${run_name}" "$run_name" cartesian_impedance_controller > /tmp/extract_log.txt 2>&1
    eval_out=$(python3 "${TOOLS_DIR}/scripts/evaluate_cartesian_tracking_headless.py" "$run_name" 2>&1)
    result_line=$(echo "$eval_out" | grep "^RESULT_LINE" | sed 's/^RESULT_LINE,//')

    if [ -z "$result_line" ]; then
        echo "  [WARNING] ancora fallito"
        echo "${ts},${td},NA,NA,NA,NA,NA,NA,NA" >> "$SUMMARY_CSV"
    else
        echo "  -> ${result_line}"
        echo "${ts},${td},${result_line}" >> "$SUMMARY_CSV"
    fi

    kill -TERM "$launch_pid" 2>/dev/null; sleep 2
    kill_everything_hard; wait_for_clean_ros2_graph
done

echo "Fatto. Ricontrolla il CSV per eventuali NA residui."
