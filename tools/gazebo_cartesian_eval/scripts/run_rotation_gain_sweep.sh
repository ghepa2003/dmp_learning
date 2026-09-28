#!/usr/bin/env bash
set -uo pipefail

# Range scelto per bracket-are sia il punto stabile trovato (20/3) sia i
# valori SERL originali (150/7) che avevano causato l'instabilita' di ieri -
# per testare direttamente l'ipotesi che la rotazione fosse la vera causa.
ROT_K_LIST=(10 20 40 70 110 150)
ROT_D_LIST=(1 2 3 4 5 7)

FIXED_TRANS_STIFF=200.0
FIXED_TRANS_DAMP=30.0

WEIGHTS_YAML="/root/thesis_ws/real_trajA_ridge_filter.yaml"
ROLLOUT_DURATION=70

CONTROLLERS_YAML="/root/ros_workspaces/ros2/franka_ws/src/franka_ros2/franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml"
CONTROLLERS_YAML_BACKUP="/root/thesis_ws/src/franka_gazebo_overrides/franka_gazebo_controllers.yaml"

TOOLS_DIR="$HOME/thesis_ws/tools/gazebo_cartesian_eval"
SUMMARY_CSV="${TOOLS_DIR}/data/rotation_sweep_summary.csv"
mkdir -p "${TOOLS_DIR}/data" "${TOOLS_DIR}/bags"
echo "rot_stiffness,rot_damping,rot_cum_deg,rot_net_deg,ratio,mean_pos_mm,max_pos_mm,mean_ang_deg,max_ang_deg" > "$SUMMARY_CSV"

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
            echo "  [WARNING] Grafo non pulito dopo ${max_wait}s, forzo riavvio demone ROS2"
            ros2 daemon stop > /dev/null 2>&1
            sleep 2
            ros2 daemon start > /dev/null 2>&1
            sleep 3
            return 1
        fi
    done
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

run_one_combo () {
    local rs="$1" rd="$2"
    local run_name="rot_K${rs}_D${rd}"

    echo ""
    echo "== ${run_name}: rot_stiffness=${rs}, rot_damping=${rd} =="

    kill_everything_hard
    wait_for_clean_ros2_graph

    write_gains_to_yaml "$FIXED_TRANS_STIFF" "$FIXED_TRANS_DAMP" "$rs" "$rd"
    cd /root/ros_workspaces/ros2/franka_ws
    colcon build --packages-select franka_gazebo_bringup --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release > /dev/null 2>&1
    set +u
    source install/setup.bash
    set -u

    ros2 launch franka_gazebo_bringup gazebo_cartesian_impedance_control_headless.launch.py \
        > "/tmp/launch_log_${run_name}.txt" 2>&1 &
    local launch_pid=$!

    local waited=0
    while ! grep -q "Successfully loaded controller cartesian_impedance_controller" \
            "/tmp/launch_log_${run_name}.txt" 2>/dev/null; do
        if ! kill -0 "$launch_pid" 2>/dev/null; then
            echo "  [ERROR] Processo launch morto prematuramente per ${run_name}"
            echo "${rs},${rd},NA,NA,NA,NA,NA,NA,NA" >> "$SUMMARY_CSV"
            kill_everything_hard
            return
        fi
        sleep 1
        waited=$((waited + 1))
        if [ "$waited" -ge 40 ]; then
            echo "  [ERROR] Timeout aspettando l'avvio del controller per ${run_name}"
            kill -9 "$launch_pid" 2>/dev/null
            kill_everything_hard
            echo "${rs},${rd},NA,NA,NA,NA,NA,NA,NA" >> "$SUMMARY_CSV"
            return
        fi
    done

    local logged_rs
    logged_rs=$(grep "rotational_stiffness" "/tmp/launch_log_${run_name}.txt" | grep -oE '=\s*[0-9.]+' | grep -oE '[0-9.]+' | head -1)
    if [ "$logged_rs" != "${rs}.000" ] && [ "$logged_rs" != "$rs" ]; then
        echo "  [ERROR] Valore caricato (${logged_rs}) non coincide con quello atteso (${rs})!"
        echo "${rs},${rd},NA,NA,NA,NA,NA,NA,NA" >> "$SUMMARY_CSV"
        kill -TERM "$launch_pid" 2>/dev/null
        kill_everything_hard
        return
    fi

    sleep 3

    rm -rf "${TOOLS_DIR}/bags/${run_name}"
    ros2 bag record \
        /cartesian_impedance_controller/target_pose_aligned \
        /cartesian_impedance_controller/actual_pose \
        /joint_states \
        -o "${TOOLS_DIR}/bags/${run_name}" > /dev/null 2>&1 &
    local bag_pid=$!
    sleep 1

    timeout "${ROLLOUT_DURATION}" ros2 run haptic_dmp_learning dmp_gazebo_executor_node \
        --ros-args -p weights_yaml_path:="$WEIGHTS_YAML" > /tmp/executor_log.txt 2>&1

    sleep 2
    kill -INT "$bag_pid" 2>/dev/null
    wait "$bag_pid" 2>/dev/null

    python3 "${TOOLS_DIR}/scripts/extract_bag_to_csv.py" \
        "${TOOLS_DIR}/bags/${run_name}" "$run_name" cartesian_impedance_controller > /tmp/extract_log.txt 2>&1

    local eval_out result_line
    eval_out=$(python3 "${TOOLS_DIR}/scripts/evaluate_cartesian_tracking_headless.py" "$run_name" 2>&1)
    result_line=$(echo "$eval_out" | grep "^RESULT_LINE" | sed 's/^RESULT_LINE,//')

    if [ -z "$result_line" ]; then
        echo "  [WARNING] Nessun risultato valido per ${run_name}"
        echo "${rs},${rd},NA,NA,NA,NA,NA,NA,NA" >> "$SUMMARY_CSV"
    else
        echo "${rs},${rd},${result_line}" >> "$SUMMARY_CSV"
        echo "  -> ${result_line}"
    fi

    kill -TERM "$launch_pid" 2>/dev/null
    sleep 2
    kill_everything_hard
    wait_for_clean_ros2_graph
}

for rs in "${ROT_K_LIST[@]}"; do
    for rd in "${ROT_D_LIST[@]}"; do
        run_one_combo "$rs" "$rd"
    done
done

echo ""
echo "== Sweep rotazione completato. Risultati in ${SUMMARY_CSV} =="
