#!/usr/bin/env bash
# Registra i topic rilevanti per la valutazione del tracking cartesiano.
# Uso: ./record_bag.sh <nome_run>
# Se <nome_run> non è specificato, usa un timestamp.
set -e

RUN_NAME="${1:-run_$(date +%Y%m%d_%H%M%S)}"
BAG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/bags/${RUN_NAME}"

if [ -d "$BAG_DIR" ]; then
    echo "Errore: esiste già un bag con nome '${RUN_NAME}' in ${BAG_DIR}"
    exit 1
fi

echo "Registrazione bag: ${RUN_NAME}"
echo "Output: ${BAG_DIR}"
echo "Premi Ctrl+C per fermare la registrazione al termine del rollout."
echo ""

ros2 bag record \
    /velocity_cartesian_controller/target_pose_aligned \
    /velocity_cartesian_controller/actual_pose \
    /joint_states \
    -o "$BAG_DIR"