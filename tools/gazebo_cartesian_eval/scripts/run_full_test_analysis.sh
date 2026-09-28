#!/usr/bin/env bash
# run_full_test_analysis.sh — UN SOLO comando per registrare ed analizzare un
# test di grasp completo (tracking cartesiano + forza + stato del grasp).
#
# Flusso d'uso:
#   1. In un terminale:  ./run_full_test_analysis.sh <nome_run>
#      (blocca: sta registrando il bag)
#   2. In un altro terminale: avvia il replay / rollout DMP
#   3. A fine test: Ctrl-C su questo script
#   4. Lo script estrae automaticamente TUTTI i CSV e genera TUTTI i plot,
#      poi stampa dove si trovano.
#
# Non modifica nessuno script esistente: orchestra record_full_test_bag.sh
# (nuovo) + extract_bag_to_csv.py / evaluate_cartesian_tracking_headless.py
# (esistenti, invariati) + extract_force_grasp_to_csv.py / plot_force_grasp_state.py
# (nuovi).
#
# Uso: ./run_full_test_analysis.sh <nome_run> [nome_controller] [hard_force_limit_n]
set -euo pipefail

if [ "$#" -lt 1 ]; then
    echo "Uso: $0 <nome_run> [nome_controller] [hard_force_limit_n]"
    exit 1
fi

RUN_NAME="$1"
CONTROLLER_NAME="${2:-cartesian_impedance_controller}"
HARD_FORCE_LIMIT_N="${3:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOOLS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BAG_DIR="$TOOLS_DIR/bags/${RUN_NAME}"
DATA_DIR="$TOOLS_DIR/data"
PLOTS_TRACKING_DIR="$TOOLS_DIR/plots/03_gain_sweep"
PLOTS_GRASP_DIR="$TOOLS_DIR/plots/04_grasp_test"

if [ -d "$BAG_DIR" ]; then
    echo "Errore: esiste già un bag con nome '${RUN_NAME}' in ${BAG_DIR}" >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# 1. Registrazione (foreground-bloccante fino a Ctrl-C dell'utente)
# ---------------------------------------------------------------------------
echo "=================================================================="
echo "=== REGISTRAZIONE: ${RUN_NAME}"
echo "=== Avvia ora il replay in un altro terminale."
echo "=== Ctrl-C QUI a fine test per estrarre CSV e generare i plot."
echo "=================================================================="

RECORDING_DONE=0
on_int () {
    if [ "$RECORDING_DONE" -eq 0 ]; then
        echo ""
        echo "[run_full_test_analysis] Ctrl-C ricevuto: chiudo la registrazione e avvio l'analisi..."
    fi
}
trap on_int INT

"$SCRIPT_DIR/record_full_test_bag.sh" "$RUN_NAME" "$CONTROLLER_NAME" &
REC_PID=$!

# 'wait' ritorna subito alla ricezione di INT (dopo l'esecuzione del trap); il
# ciclo ripete finche' il registratore non e' davvero terminato, cosi'
# 'ros2 bag record' finalizza il bag. record_full_test_bag.sh usa 'exec', quindi
# REC_PID e' proprio il processo di registrazione.
while kill -0 "$REC_PID" 2>/dev/null; do
    wait "$REC_PID" 2>/dev/null || true
done
RECORDING_DONE=1
trap - INT

# Attendi che il bag sia finalizzato (metadata.yaml scritto), max ~10 s.
for _ in $(seq 1 50); do
    [ -f "$BAG_DIR/metadata.yaml" ] && break
    sleep 0.2
done
if [ ! -f "$BAG_DIR/metadata.yaml" ]; then
    echo "[ERRORE] ${BAG_DIR}/metadata.yaml non trovato: registrazione non finalizzata." >&2
    exit 1
fi
echo ""
echo "Registrazione terminata: ${BAG_DIR}"

# ---------------------------------------------------------------------------
# 2. Analisi automatica in sequenza (fail-loud: uno step fallito ferma tutto)
# ---------------------------------------------------------------------------
run_step () {
    local desc="$1"; shift
    echo ""
    echo ">>> ${desc}"
    if ! "$@"; then
        echo "[ERRORE] step fallito: ${desc}" >&2
        echo "         comando: $*" >&2
        exit 1
    fi
}

run_step "a. Estrazione pose (target_pose_aligned / actual_pose) -> CSV" \
    python3 "$SCRIPT_DIR/extract_bag_to_csv.py" "$BAG_DIR" "$RUN_NAME" "$CONTROLLER_NAME"

run_step "b. Valutazione tracking cartesiano (errore + PNG)" \
    python3 "$SCRIPT_DIR/evaluate_cartesian_tracking_headless.py" "$RUN_NAME"

run_step "c. Estrazione forza / grasp-state / gripper / target -> CSV" \
    python3 "$SCRIPT_DIR/extract_force_grasp_to_csv.py" "$BAG_DIR" "$RUN_NAME" "$CONTROLLER_NAME"

if [ -n "$HARD_FORCE_LIMIT_N" ]; then
    run_step "d. Plot forza / grasp-state / gripper (limite ${HARD_FORCE_LIMIT_N} N)" \
        python3 "$SCRIPT_DIR/plot_force_grasp_state.py" "$RUN_NAME" "$HARD_FORCE_LIMIT_N"
else
    run_step "d. Plot forza / grasp-state / gripper" \
        python3 "$SCRIPT_DIR/plot_force_grasp_state.py" "$RUN_NAME"
fi

# ---------------------------------------------------------------------------
# 3. Riepilogo: path di TUTTI i file prodotti
# ---------------------------------------------------------------------------
report_file () {
    local path="$1"
    if [ -f "$path" ]; then
        echo "  [OK]      $path"
    else
        echo "  [MANCANTE] $path"
    fi
}

echo ""
echo "=================================================================="
echo "=== FILE PRODOTTI per run '${RUN_NAME}'"
echo "=================================================================="
echo "Bag:"
echo "  [OK]      $BAG_DIR"
echo "CSV (pose):"
report_file "$DATA_DIR/target_aligned_${RUN_NAME}.csv"
report_file "$DATA_DIR/actual_pose_${RUN_NAME}.csv"
echo "CSV (forza / grasp-state / gripper / target):"
report_file "$DATA_DIR/force_${RUN_NAME}.csv"
report_file "$DATA_DIR/grasp_state_${RUN_NAME}.csv"
report_file "$DATA_DIR/gripper_cmd_${RUN_NAME}.csv"
report_file "$DATA_DIR/target_odom_${RUN_NAME}.csv"
echo "Plot:"
report_file "$PLOTS_TRACKING_DIR/cartesian_tracking_${RUN_NAME}.png"
report_file "$PLOTS_GRASP_DIR/force_grasp_state_${RUN_NAME}.png"
echo "=================================================================="
