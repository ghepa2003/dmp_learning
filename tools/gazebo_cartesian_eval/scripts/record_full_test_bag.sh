#!/usr/bin/env bash
# Registra i topic rilevanti per la valutazione COMPLETA di un test di grasp:
# oltre al tracking cartesiano (come record_bag.sh) anche forza stimata, stato
# della macchina a stati del grasp, comando gripper e posa del target.
#
# Non modifica record_bag.sh: quello resta il registratore "solo tracking" usato
# da run_reach_task_comparison.sh e dagli sweep. Questo e' un registratore
# indipendente con lo stesso stile (stessi argomenti CLI, stessa directory di
# output bag), pensato per essere orchestrato da run_full_test_analysis.sh.
#
# Uso: ./record_full_test_bag.sh <nome_run> [nome_controller]
# nome_controller default: cartesian_impedance_controller
set -e

RUN_NAME="${1:-run_$(date +%Y%m%d_%H%M%S)}"
CONTROLLER_NAME="${2:-cartesian_impedance_controller}"
BAG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/bags/${RUN_NAME}"

if [ -d "$BAG_DIR" ]; then
    echo "Errore: esiste già un bag con nome '${RUN_NAME}' in ${BAG_DIR}"
    exit 1
fi

echo "Registrazione bag COMPLETA: ${RUN_NAME} (controller: ${CONTROLLER_NAME})"
echo "Output: ${BAG_DIR}"
echo "Topic: tracking (target_pose_aligned, actual_pose, joint_states) +"
echo "       forza (contact_wrench_estimate), grasp_state, gripper_position_cmd, target odometry"
echo "Premi Ctrl+C per fermare la registrazione al termine del rollout."
echo ""

# Un topic non ancora pubblicato non fa fallire 'ros2 bag record': il bag
# semplicemente non conterra' messaggi per quel topic (l'estrazione gestisce
# con grazia un CSV vuoto).
#
# 'exec': la shell viene rimpiazzata da 'ros2 bag record', cosi' quando
# run_full_test_analysis.sh lancia questo script in background il PID che
# osserva e' direttamente quello del registratore - resta vivo finche' il bag
# non e' finalizzato dopo il Ctrl-C. In uso standalone il comportamento e'
# identico (Ctrl-C ferma e finalizza il bag).
exec ros2 bag record \
    "/${CONTROLLER_NAME}/target_pose_aligned" \
    "/${CONTROLLER_NAME}/actual_pose" \
    /joint_states \
    "/${CONTROLLER_NAME}/contact_wrench_estimate" \
    /grasp_state_machine/grasp_state \
    /gripper_position_cmd \
    /free_target_object/odometry \
    -o "$BAG_DIR"
