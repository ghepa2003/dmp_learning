#!/usr/bin/env bash
# Registra i topic rilevanti per la valutazione del tracking cartesiano.
# Uso: ./record_bag.sh <nome_run> [nome_controller]
# nome_controller default: cartesian_impedance_controller
set -e

RUN_NAME="${1:-run_$(date +%Y%m%d_%H%M%S)}"
CONTROLLER_NAME="${2:-cartesian_impedance_controller}"
BAG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/bags/${RUN_NAME}"

if [ -d "$BAG_DIR" ]; then
    echo "Errore: esiste già un bag con nome '${RUN_NAME}' in ${BAG_DIR}"
    exit 1
fi

echo "Registrazione bag: ${RUN_NAME} (controller: ${CONTROLLER_NAME})"
echo "Output: ${BAG_DIR}"
echo "Premi Ctrl+C per fermare la registrazione al termine del rollout."
echo ""

# --- Aggiunte per satellite_rotation_mode="continuous" (i topic sopra restano invariati) ---
# /clock e ~/continuous_status dell'executor sono sempre registrati; l'odometria del satellite solo se
# SATELLITE_ODOM_TOPIC e' impostata (es. SATELLITE_ODOM_TOPIC=/free_target_object/odometry): senza,
# l'assenza viene dichiarata esplicitamente qui sotto. --use-sim-time: timestamp di registrazione in sim time.
EXTRA_TOPICS=(/clock /prodmp_gazebo_executor_node/continuous_status)
if [ -n "${SATELLITE_ODOM_TOPIC:-}" ]; then
    EXTRA_TOPICS+=("$SATELLITE_ODOM_TOPIC")
else
    echo "[ATTENZIONE] SATELLITE_ODOM_TOPIC non impostata: l'odometria del satellite NON viene registrata."
fi

ros2 bag record --use-sim-time \
    --max-cache-size 104857600 \
    "/${CONTROLLER_NAME}/target_pose_aligned" \
    "/${CONTROLLER_NAME}/actual_pose" \
    /joint_states \
    "${EXTRA_TOPICS[@]}" \
    -o "$BAG_DIR"