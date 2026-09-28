#!/bin/bash
# run_missing_nullspace_points.sh — Rilancio isolato dei soli punti mancanti/falliti
# dello sweep nullspace stiffness, in append al file CSV esistente.

set -euo pipefail

# --- Percorsi Workspace e File di Configurazione ---
THESIS_WS="${THESIS_WS:-/root/thesis_ws}"
FRANKA_WS="${FRANKA_WS:-/root/ros_workspaces/ros2/franka_ws}"
CONFIG_YAML="$FRANKA_WS/src/franka_ros2/franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml"
LAUNCH_PACKAGE="franka_gazebo_bringup"
LAUNCH_FILE="gazebo_cartesian_impedance_control_headless.launch.py"
DMP_WEIGHTS="$THESIS_WS/real_trajA_ridge_filter.yaml"
LOG_SCRIPT="$THESIS_WS/tools/gazebo_cartesian_eval/scripts/log_nullspace_leak.py"
OUTPUT_CSV="$THESIS_WS/tools/gazebo_cartesian_eval/results/nullspace_stiffness_sweep.csv"

LEAK_TOPIC="/cartesian_impedance_controller/nullspace_leak"
TARGET_POSE_TOPIC="/target_pose"
JOINT1_NULLSPACE_STIFFNESS=100.0   # fisso: ablation già chiusa

# Punti da rilanciare: passa argomenti o usa il default dei punti falliti (5,2), (20,2), (20,3)
if [ $# -gt 0 ]; then
    MISSING=("$@")
else
    MISSING=("5 2" "20 2" "20 3")
fi

# Assicura che la directory esista e inizializza l'header solo se il file non esiste
mkdir -p "$(dirname "$OUTPUT_CSV")"
if [ ! -f "$OUTPUT_CSV" ]; then
    echo "nullspace_stiffness,repeat,n_samples,leak_lin_mean_m_s2,leak_lin_max_m_s2,leak_ang_mean_rad_s2,leak_ang_max_rad_s2,rot_cum_deg,rot_net_deg,ratio,mean_pos_mm,max_pos_mm,mean_ang_deg,max_ang_deg" > "$OUTPUT_CSV"
fi

kill_everything_hard () {
    pkill -9 -f "ign gazebo" 2>/dev/null || true
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

for combo in "${MISSING[@]}"; do
    read -r K R <<< "$combo"
    run_name="nullspace_K${K}_rep${R}"
    launch_log="/tmp/launch_log_${run_name}.txt"

    echo ""
    echo "=================================================================="
    echo "=== nullspace_stiffness=$K, repeat=$R (missing point) ==="
    echo "=================================================================="

    kill_everything_hard
    wait_for_clean_ros2_graph

    # 1. Scrivi il gain nello YAML sorgente PRIMA del launch
    python3 - "$CONFIG_YAML" "$K" <<'PYEOF'
import sys, re
path, value = sys.argv[1], sys.argv[2]
with open(path) as f:
    content = f.read()

# Aggiorna solo nullspace_stiffness (escludendo joint1_nullspace_stiffness)
content = re.sub(
    r'(?<!joint1_)(nullspace_stiffness:\s*)[\d.]+',
    r'\g<1>' + str(float(value)),
    content
)

# Assicura che la diagnostica leak sia abilitata
if 'enable_nullspace_leak_diagnostics:' in content:
    content = re.sub(r'enable_nullspace_leak_diagnostics:\s*(true|false)', 'enable_nullspace_leak_diagnostics: true', content)
else:
    content += "\n    enable_nullspace_leak_diagnostics: true\n"

with open(path, 'w') as f:
    f.write(content)
PYEOF

    # 2. Relaunch completo di Gazebo Headless, con retry su fallimento avvio
    MAX_LAUNCH_ATTEMPTS=3
    launch_ok=false
    for attempt in $(seq 1 $MAX_LAUNCH_ATTEMPTS); do
        ros2 launch "$LAUNCH_PACKAGE" "$LAUNCH_FILE" > "$launch_log" 2>&1 &
        GAZEBO_PID=$!
        waited=0
        while ! grep -q "Successfully loaded controller cartesian_impedance_controller" "$launch_log" 2>/dev/null; do
            if ! kill -0 "$GAZEBO_PID" 2>/dev/null; then
                echo "  [WARN] Processo launch terminato prematuramente (tentativo $attempt/$MAX_LAUNCH_ATTEMPTS)"
                break
            fi
            sleep 1
            waited=$((waited + 1))
            if [ "$waited" -ge 45 ]; then
                echo "  [WARN] Timeout avvio controller (tentativo $attempt/$MAX_LAUNCH_ATTEMPTS)"
                kill -9 "$GAZEBO_PID" 2>/dev/null || true
                break
            fi
        done
        if grep -q "Successfully loaded controller cartesian_impedance_controller" "$launch_log" 2>/dev/null; then
            launch_ok=true
            break
        fi
        kill_everything_hard
        wait_for_clean_ros2_graph
    done
    if [ "$launch_ok" != "true" ]; then
        echo "  [ERROR] Fallito avvio controller per ${run_name} dopo $MAX_LAUNCH_ATTEMPTS tentativi"
        echo "$K,$R,0,NA,NA,NA,NA,NA,NA,NA,NA,NA,NA,NA" >> "$OUTPUT_CSV"
        kill_everything_hard
        continue
    fi

    # 3. Verifica esplicita dal log che il valore caricato sia quello atteso
    logged_val=$(grep -oE 'nullspace_stiffness[[:space:]]*=[[:space:]]*[0-9.]+' "$launch_log" | grep -oE '[0-9.]+$' | head -1 || true)
    if [ -n "$logged_val" ]; then
        match=$(python3 -c "print(abs(float('$logged_val') - float('$K')) < 1e-3)")
        if [ "$match" != "True" ]; then
            echo "  [ERROR] Mismatch gain: atteso $K, caricato dal nodo $logged_val"
            kill -TERM "$GAZEBO_PID" 2>/dev/null || true
            kill_everything_hard
            echo "$K,$R,0,NA,NA,NA,NA,NA,NA,NA,NA,NA,NA,NA" >> "$OUTPUT_CSV"
            continue
        else
            echo "  [OK] Parametro confermato dal controller: nullspace_stiffness = $logged_val"
        fi
    fi

    sleep 2

    # 3bis. Avvia registrazione bag per valutazione tracking cartesiano in parallelo
    bag_dir="$THESIS_WS/tools/gazebo_cartesian_eval/bags/${run_name}"
    rm -rf "$bag_dir"
    ros2 bag record \
        /cartesian_impedance_controller/target_pose_aligned \
        /cartesian_impedance_controller/actual_pose \
        /joint_states \
        -o "$bag_dir" > /dev/null 2>&1 &
    BAG_PID=$!

    # 4. Avvia il logger diagnostico del leak in background
    leak_log="/tmp/leak_${run_name}.txt"
    python3 "$LOG_SCRIPT" "$LEAK_TOPIC" "$TARGET_POSE_TOPIC" 0.5 180.0 > "$leak_log" 2>&1 &
    LOGGER_PID=$!

    sleep 1

    # 5. Lancia la traiettoria DMP
    ros2 run haptic_dmp_learning dmp_gazebo_executor_node \
        --ros-args -p weights_yaml_path:="$DMP_WEIGHTS" > /tmp/executor_log.txt 2>&1 &
    DMP_PID=$!

    # Attendi che il logger finisca (si chiuderà 0.5s dopo la fine del rollout, con timeout di sicurezza a 80s)
    waited_log=0
    while kill -0 "$LOGGER_PID" 2>/dev/null; do
        sleep 1
        waited_log=$((waited_log + 1))
        if [ "$waited_log" -ge 80 ]; then
            echo "  [WARNING] Timeout attesa logger (${waited_log}s), forzo chiusura..."
            kill -9 "$LOGGER_PID" 2>/dev/null || true
            break
        fi
    done

    # Ferma la registrazione bag e valuta il tracking
    kill -INT "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true

    python3 "$THESIS_WS/tools/gazebo_cartesian_eval/scripts/extract_bag_to_csv.py" \
        "$bag_dir" "$run_name" cartesian_impedance_controller > /tmp/extract_log_${run_name}.txt 2>&1

    eval_out=$(python3 "$THESIS_WS/tools/gazebo_cartesian_eval/scripts/evaluate_cartesian_tracking_headless.py" "$run_name" 2>&1)
    tracking_result=$(echo "$eval_out" | grep "^RESULT_LINE" | sed 's/^RESULT_LINE,//')
    if [ -z "$tracking_result" ]; then
        tracking_result="NA,NA,NA,NA,NA,NA,NA"
    fi
    echo "  [Tracking] $tracking_result"

    RESULT=$(grep "^RESULT_LINE" "$leak_log" 2>/dev/null || echo "RESULT_LINE no_samples=1")
    echo "  $RESULT"

    N_SAMPLES=$(grep -oP 'n_samples=\K[0-9]+' <<< "$RESULT" || echo "0")
    L_LIN_MEAN=$(grep -oP 'leak_lin_mean_m_s2=\K[0-9.eE+-]+' <<< "$RESULT" || echo "NA")
    L_LIN_MAX=$(grep -oP 'leak_lin_max_m_s2=\K[0-9.eE+-]+' <<< "$RESULT" || echo "NA")
    L_ANG_MEAN=$(grep -oP 'leak_ang_mean_rad_s2=\K[0-9.eE+-]+' <<< "$RESULT" || echo "NA")
    L_ANG_MAX=$(grep -oP 'leak_ang_max_rad_s2=\K[0-9.eE+-]+' <<< "$RESULT" || echo "NA")

    echo "$K,$R,$N_SAMPLES,$L_LIN_MEAN,$L_LIN_MAX,$L_ANG_MEAN,$L_ANG_MAX,$tracking_result" >> "$OUTPUT_CSV"

    # 6. Cleanup completo prima del prossimo punto
    kill "$DMP_PID" 2>/dev/null || true
    kill -TERM "$GAZEBO_PID" 2>/dev/null || true
    sleep 2
    kill_everything_hard
    wait_for_clean_ros2_graph
done

echo ""
echo "=================================================================="
echo "Completati punti mancanti. Risultati aggiunti in: $OUTPUT_CSV"
echo "=================================================================="
