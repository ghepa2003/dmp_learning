#!/usr/bin/env bash
# Sweep on the number of basis functions (n_basis) for ProDMP on Trajectory C (demo_raw_trajC.csv).

set -euo pipefail

N_BASIS_LIST=(5 10 15 20 25 30 40 50 60 80 100 150 200 300 500)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$ROOT_DIR"

PLOT_DIR="plots/04_basis_sweep/real/trajC_prodmp"
mkdir -p build data "${PLOT_DIR}" weights 04_basis_sweep/test_configs

PKG_DIR="../../src/haptic_dmp_learning"

if [ ! -x build/learn_and_test_prodmp ] || \
   [ "${PKG_DIR}/src/core/prodmp.cpp" -nt build/learn_and_test_prodmp ] || \
   [ "${PKG_DIR}/src/core/prodmp_io.cpp" -nt build/learn_and_test_prodmp ] || \
   [ "${PKG_DIR}/src/core/quaternion_dmp.cpp" -nt build/learn_and_test_prodmp ] || \
   [ "${PKG_DIR}/src/core/demo_csv_io.cpp" -nt build/learn_and_test_prodmp ] || \
   [ "${PKG_DIR}/src/core/dmp_io.cpp" -nt build/learn_and_test_prodmp ] || \
   [ "${PKG_DIR}/src/core/demo_params.cpp" -nt build/learn_and_test_prodmp ] || \
   [ "${PKG_DIR}/src/core/joint_states_csv_io.cpp" -nt build/learn_and_test_prodmp ] || \
   [ "common/src/learn_and_test_prodmp.cpp" -nt build/learn_and_test_prodmp ]; then
    echo "== Building learn_and_test_prodmp =="
    g++ -std=c++17 -O2 \
        -I "${PKG_DIR}/include" \
        -I common/include \
        -I/usr/include/eigen3 \
        common/src/learn_and_test_prodmp.cpp \
        common/src/metrics.cpp \
        "${PKG_DIR}/src/core/prodmp.cpp" \
        "${PKG_DIR}/src/core/prodmp_io.cpp" \
        "${PKG_DIR}/src/core/quaternion_dmp.cpp" \
        "${PKG_DIR}/src/core/demo_csv_io.cpp" \
        "${PKG_DIR}/src/core/dmp_io.cpp" \
        "${PKG_DIR}/src/core/dmp.cpp" \
        "${PKG_DIR}/src/core/demo_params.cpp" \
        "${PKG_DIR}/src/core/joint_states_csv_io.cpp" \
        -o build/learn_and_test_prodmp \
        -lyaml-cpp
fi

# Target demo: Trajectory C
DEMO_CSV=""
for path in "../../demo_raw_trajC.csv" "${HOME}/thesis_ws/demo_raw_trajC.csv" "data/demo_raw_trajC.csv"; do
    [ -f "$path" ] && DEMO_CSV="$path" && break
done

if [ -z "$DEMO_CSV" ]; then
    echo "[ERROR] demo_raw_trajC.csv not found!" >&2
    exit 1
fi

echo "== Running ProDMP n_basis sweep on Trajectory C (${DEMO_CSV}) =="

# ProDMP baseline configuration: ridge_lambda=1e-6, position_filter window=0.20s
CONFIG_PATH="04_basis_sweep/test_configs/prodmp_base_filter0.20.yaml"
cat > "$CONFIG_PATH" << EOF
num_basis: 200
ridge_lambda: 1.0e-6
position_filter:
  enabled: true
  window_sec: 0.20
EOF

SUMMARY_CSV="${PLOT_DIR}/nbasis_results_trajC_prodmp.csv"
rm -f "$SUMMARY_CSV"

for n_basis in "${N_BASIS_LIST[@]}"; do
    label="nbasis_$(printf '%04d' "$n_basis")_trajC_prodmp"
    yaml_out="weights/prodmp_${label}.yaml"
    replay_out="data/replay_prodmp_${label}.csv"

    echo "---- ProDMP / ${label} ----" >&2
    build/learn_and_test_prodmp "$DEMO_CSV" "$yaml_out" "$replay_out" "$SUMMARY_CSV" "$label" \
        "$n_basis" - - - "$CONFIG_PATH" >&2
done

echo ""
echo "== Generating comparative plots for ProDMP on Trajectory C =="
python3 common/scripts/plot_nbasis_study.py \
    --summary-csv "$SUMMARY_CSV" \
    --series-label "ProDMP" \
    --plot-dir "${PLOT_DIR}" \
    --title-suffix " (Trajectory C)"

echo ""
echo "== Done. Plots in ${PLOT_DIR}/ =="
