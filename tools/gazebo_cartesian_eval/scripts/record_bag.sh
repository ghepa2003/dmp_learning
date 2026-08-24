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

ros2 bag record \
    "/${CONTROLLER_NAME}/target_pose_aligned" \
    "/${CONTROLLER_NAME}/actual_pose" \
    /joint_states \
    -o "$BAG_DIR"