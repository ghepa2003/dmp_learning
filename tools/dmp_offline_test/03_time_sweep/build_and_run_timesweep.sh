#!/usr/bin/env bash
# Builds learn_and_test_dmp and runs full sweep on all synthetic demos
# present in data/. For each trial: learn + replay in-process + metrics
# (via metrics.cpp/hpp) + per-trial plot. Finally generates aggregated study plots.

set -euo pipefail

N_BASIS="${1:-20}"
REPRESENTATIVE_DURATION="${2:-60}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT_DIR"

PLOT_DIR="plots/03_time_sweep"
mkdir -p build data "${PLOT_DIR}" weights

# --- Build -----------------------------------------------------
PKG_DIR="../../src/haptic_dmp_learning"

echo "== Building learn_and_test_dmp =="
g++ -std=c++17 -O2 \
    -I "${PKG_DIR}/include" \
    -I 03_time_sweep \
    -I common \
    -I/usr/include/eigen3 \
    03_time_sweep/learn_and_test_dmp.cpp \
    common/metrics.cpp \
    "${PKG_DIR}/src/core/dmp.cpp" \
    "${PKG_DIR}/src/core/quaternion_dmp.cpp" \
    "${PKG_DIR}/src/core/dmp_io.cpp" \
    -o build/learn_and_test_dmp \
    -lyaml-cpp

# --- Sweep --------------------------------------------------------------
SUMMARY_CSV="${PLOT_DIR}/sweep_results.csv"
rm -f "$SUMMARY_CSV"

shopt -s nullglob
demo_files=(data/demo_synth_*.csv)
shopt -u nullglob

if [ ${#demo_files[@]} -eq 0 ]; then
    echo "No demo_synth_*.csv files found in data/. Generate matrix first with: python3 03_time_sweep/generate_sweep_matrix.py --outdir data"
    exit 1
fi

echo "== Found ${#demo_files[@]} demos, n_basis=${N_BASIS} =="

for demo_csv in "${demo_files[@]}"; do
    base="$(basename "${demo_csv%.csv}")"

    if [[ "$base" == replay_* ]]; then
        continue
    fi

    yaml_out="weights/${base}.yaml"
    replay_out="data/replay_${base}.csv"

    echo "---- ${base} ----"
    build/learn_and_test_dmp "$demo_csv" "$yaml_out" "$replay_out" "$SUMMARY_CSV" "$base" "$N_BASIS"

    python3 03_time_sweep/plot_dmp_timesweep.py \
        --demo "$demo_csv" \
        --replay "$replay_out" \
        --plot-dir "${PLOT_DIR}" \
        --label "$base"
done

echo ""
echo "== Sweep completed. Generating aggregated study plots =="
python3 03_time_sweep/plot_sweep_study.py \
    --summary-csv "$SUMMARY_CSV" \
    --plot-dir "${PLOT_DIR}" \
    --representative-duration "$REPRESENTATIVE_DURATION"

echo ""
echo "== Done. Numerical summary in ${SUMMARY_CSV}, plots in ${PLOT_DIR}/ =="

