#!/usr/bin/env bash
set -uo pipefail

TRANS_K_LIST=(100 200 400 800 1400 2000)
TRANS_D_LIST=(10 20 30 50 70 89)

FIXED_ROT_STIFF=20.0
FIXED_ROT_DAMP=3.0

WEIGHTS_YAML="/root/thesis_ws/real_trajA_ridge_filter.yaml"
ROLLOUT_DURATION=70

TOOLS_DIR="$HOME/thesis_ws/tools/gazebo_cartesian_eval"
SUMMARY_CSV="${TOOLS_DIR}/data/translation_sweep_summary.csv"
mkdir -p "${TOOLS_DIR}/data" "${TOOLS_DIR}/bags"
echo "trans_stiffness,trans_damping,rot_cum_deg,rot_net_deg,ratio,mean_pos_mm,max_pos_mm,mean_ang_deg,max_ang_deg" > "$SUMMARY_CSV"

run_one_combo () {
    local ts="$1" td="$2"
    local run_name="trans_K${ts}_D${td}"

    echo ""
    echo "== ${run_name}: trans_stiffness=${ts}, trans_damping=${td} =="

    # Lancia Gazebo da zero per questa combinazione
    ros2 launch franka_gazebo_bringup gazebo_cartesian_impedance_control.launch.py \
        > /tmp/launch_log_${run_name}.txt 2>&1 &
    local launch_pid=$!

    # Aspetta che il controller sia effettivamente attivo, invece di un
    # semplice sleep fisso - poll sul log fino al messaggio di successo.
    local waited=0
    while ! grep -q "Successfully loaded controller cartesian_impedance_controller" \
            /tmp/launch_log_${run_name}.txt 2>/dev/null; do
        sleep 1
        waited=$((waited + 1))
        if [ "$waited" -ge 30 ]; then
            echo "  [ERROR] Timeout aspettando l'avvio del controller per ${run_name}"
            kill -9 "$launch_pid" 2>/dev/null
            pkill -9 -f "ign gazebo" 2>/dev/null
            echo "${ts},${td},NA,NA,NA,NA,NA,NA,NA" >> "$SUMMARY_CSV"
            return
        fi
    done
    sleep 2  # margine extra dopo il caricamento

    ros2 param set /cartesian_impedance_controller translational_stiffness "$ts" > /dev/null
    ros2 param set /cartesian_impedance_controller translational_damping "$td" > /dev/null
    ros2 param set /cartesian_impedance_controller rotational_stiffness "$FIXED_ROT_STIFF" > /dev/null
    ros2 param set /cartesian_impedance_controller rotational_damping "$FIXED_ROT_DAMP" > /dev/null

    ros2 control set_controller_state cartesian_impedance_controller inactive > /dev/null
    sleep 1
    ros2 control set_controller_state cartesian_impedance_controller active > /dev/null
    sleep 1

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
        echo "${ts},${td},NA,NA,NA,NA,NA,NA,NA" >> "$SUMMARY_CSV"
    else
        echo "${ts},${td},${result_line}" >> "$SUMMARY_CSV"
        echo "  -> ${result_line}"
    fi

    # Chiudi completamente Gazebo prima della prossima combinazione
    kill -TERM "$launch_pid" 2>/dev/null
    sleep 2
    pkill -9 -f "ign gazebo" 2>/dev/null
    pkill -9 -f "ros2 launch franka_gazebo_bringup" 2>/dev/null
    sleep 3
}

for ts in "${TRANS_K_LIST[@]}"; do
    for td in "${TRANS_D_LIST[@]}"; do
        run_one_combo "$ts" "$td"
    done
done

echo ""
echo "== Sweep traslazione completato. Risultati in ${SUMMARY_CSV} =="
