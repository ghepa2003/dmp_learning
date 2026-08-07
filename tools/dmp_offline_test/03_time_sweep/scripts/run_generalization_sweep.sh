#!/usr/bin/env bash
# Learns DMP ONCE on base demo (default: goalA_main at chosen duration),
# then tests it on all other available goals in data/ via setGoal(), without re-learning.

set -euo pipefail

DURATION="${1:?Usage: $0 <duration_s> [base_goal_name] [n_basis]}"
BASE_GOAL="${2:-goalA_main}"
N_BASIS="${3:-20}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$ROOT_DIR"

PLOT_DIR="plots/03_time_sweep"
mkdir -p build data "${PLOT_DIR}" weights

PKG_DIR="../../src/haptic_dmp_learning"
if [ ! -x build/generalize_test_dmp ]; then
    echo "== Building generalize_test_dmp =="
    g++ -std=c++17 -O2 \
        -I "${PKG_DIR}/include" \
        -I common/include \
        -I/usr/include/eigen3 \
        03_time_sweep/scripts/generalize_test_dmp.cpp \
        common/src/metrics.cpp \
        "${PKG_DIR}/src/core/dmp.cpp" \
        "${PKG_DIR}/src/core/quaternion_dmp.cpp" \
        "${PKG_DIR}/src/core/dmp_io.cpp" \
        -o build/generalize_test_dmp \
        -lyaml-cpp
fi

BASE_CSV="data/demo_synth_${DURATION}s_${BASE_GOAL}.csv"
if [ ! -f "$BASE_CSV" ]; then
    echo "Cannot find $BASE_CSV -- check duration/goal name or regenerate with: python3 03_time_sweep/scripts/generate_sweep_matrix.py --outdir data"
    exit 1
fi

SUMMARY_CSV="${PLOT_DIR}/generalization_results.csv"
rm -f "$SUMMARY_CSV"

shopt -s nullglob
target_files=(data/demo_synth_${DURATION}s_*.csv)
shopt -u nullglob

echo "== Base: ${BASE_CSV} (learned once) =="
echo "== Target: ${#target_files[@]} goals at duration ${DURATION}s =="

for target_csv in "${target_files[@]}"; do
    target_base="$(basename "${target_csv%.csv}")"
    if [[ "$target_base" == replay_* ]]; then
        continue
    fi

    label="gen_from_${BASE_GOAL}_to_${target_base#demo_synth_${DURATION}s_}"
    replay_out="data/replay_${label}.csv"

    echo "---- ${label} ----"
    build/generalize_test_dmp "$BASE_CSV" "$target_csv" "$replay_out" "$SUMMARY_CSV" "$label" "$N_BASIS"

    python3 03_time_sweep/scripts/plot_dmp_timesweep.py \
        --demo "$target_csv" \
        --replay "$replay_out" \
        --plot-dir "${PLOT_DIR}" \
        --label "$label"
done

echo ""
echo "== Generalization test completed. Summary in ${SUMMARY_CSV} =="
echo "   ('gen_from_${BASE_GOAL}_to_${BASE_GOAL}' is control case: same goal as base demo)"
