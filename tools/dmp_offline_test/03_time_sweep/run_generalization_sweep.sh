#!/usr/bin/env bash
# Impara la DMP UNA SOLA VOLTA sulla demo "base" (default: goalA_main alla
# durata scelta), poi la testa su tutti gli altri goal disponibili in data/
# tramite setGoal(), senza mai reimparare. Scrive un summary CSV separato da
# quello dello sweep n_basis/durata, perche' la semantica delle metriche e' diversa qui.

set -euo pipefail

DURATION="${1:?Uso: $0 <duration_s> [base_goal_name] [n_basis]}"
BASE_GOAL="${2:-goalA_main}"
N_BASIS="${3:-20}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT_DIR"

PLOT_DIR="plots/03_time_sweep"
mkdir -p build data "${PLOT_DIR}" weights

PKG_DIR="../../src/haptic_dmp_learning"
if [ ! -x build/generalize_test_dmp ]; then
    echo "== Compilazione generalize_test_dmp =="
    g++ -std=c++17 -O2 \
        -I "${PKG_DIR}/include" \
        -I 03_time_sweep \
        -I common \
        -I/usr/include/eigen3 \
        03_time_sweep/generalize_test_dmp.cpp \
        common/metrics.cpp \
        "${PKG_DIR}/src/core/dmp.cpp" \
        "${PKG_DIR}/src/core/quaternion_dmp.cpp" \
        "${PKG_DIR}/src/core/dmp_io.cpp" \
        -o build/generalize_test_dmp \
        -lyaml-cpp
fi

BASE_CSV="data/demo_synth_${DURATION}s_${BASE_GOAL}.csv"
if [ ! -f "$BASE_CSV" ]; then
    echo "Non trovo $BASE_CSV -- controlla durata/nome goal o rigenera con: python3 03_time_sweep/generate_sweep_matrix.py --outdir data"
    exit 1
fi

SUMMARY_CSV="${PLOT_DIR}/generalization_results.csv"
rm -f "$SUMMARY_CSV"

shopt -s nullglob
target_files=(data/demo_synth_${DURATION}s_*.csv)
shopt -u nullglob

echo "== Base: ${BASE_CSV} (appresa una sola volta) =="
echo "== Target: ${#target_files[@]} goal alla durata ${DURATION}s =="

for target_csv in "${target_files[@]}"; do
    target_base="$(basename "${target_csv%.csv}")"
    if [[ "$target_base" == replay_* ]]; then
        continue
    fi

    label="gen_from_${BASE_GOAL}_to_${target_base#demo_synth_${DURATION}s_}"
    replay_out="data/replay_${label}.csv"

    echo "---- ${label} ----"
    build/generalize_test_dmp "$BASE_CSV" "$target_csv" "$replay_out" "$SUMMARY_CSV" "$label" "$N_BASIS"

    python3 03_time_sweep/plot_dmp_timesweep.py \
        --demo "$target_csv" \
        --replay "$replay_out" \
        --plot-dir "${PLOT_DIR}" \
        --label "$label"
done

echo ""
echo "== Test di generalizzazione completato. Riepilogo in ${SUMMARY_CSV} =="
echo "   (la riga 'gen_from_${BASE_GOAL}_to_${BASE_GOAL}' e' il caso di controllo:"
echo "    stesso goal della demo base, dovrebbe avere errore quasi nullo)"
