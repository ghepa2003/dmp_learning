#!/usr/bin/env bash
# Sweep 1D on meta-parameter ridge_lambda for ProDMP (n_basis=200, window=0.20s), on Trajectory C (demo_raw_trajC.csv).

set -euo pipefail

N_BASIS=200
WINDOW=0.20
LAMBDA_LIST=(1e-9 1e-8 1e-7 1e-6 1e-5 1e-4 1e-3 1e-2 1e-1 1e0)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$ROOT_DIR"

PLOT_DIR="plots/06_ridge_lambda_sweep_prodmp"
CONFIG_DIR="04_basis_sweep/test_configs/generated_lambda_prodmp"
mkdir -p build data "${PLOT_DIR}" weights "${CONFIG_DIR}"

# --------------------------------------------------------------------------
# Build learn_and_test_prodmp
# --------------------------------------------------------------------------
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

# --------------------------------------------------------------------------
# Locate demo_raw_trajC.csv
# --------------------------------------------------------------------------
DEMO_CSV=""
for path in "../../demo_raw_trajC.csv" "${HOME}/thesis_ws/demo_raw_trajC.csv" "data/demo_raw_trajC.csv"; do
    [ -f "$path" ] && DEMO_CSV="$path" && break
done

if [ -z "$DEMO_CSV" ]; then
    echo "[ERROR] demo_raw_trajC.csv not found!" >&2
    exit 1
fi

SUMMARY_CSV="${PLOT_DIR}/ridge_lambda_results.csv"
rm -f "$SUMMARY_CSV"

for lambda in "${LAMBDA_LIST[@]}"; do
    config_path="${CONFIG_DIR}/lambda_${lambda}.yaml"
    cat > "$config_path" << EOF
num_basis: ${N_BASIS}
ridge_lambda: ${lambda}
position_filter:
  enabled: true
  window_sec: ${WINDOW}
EOF

    label="lambda_${lambda}"
    echo "== ProDMP ridge_lambda=${lambda} (n_basis=${N_BASIS}, window=${WINDOW}) on Trajectory C ==" >&2
    build/learn_and_test_prodmp "$DEMO_CSV" "weights/prodmp_${label}.yaml" \
        "data/replay_prodmp_${label}.csv" "$SUMMARY_CSV" "$label" \
        "$N_BASIS" - - - "$config_path" >&2
done

echo ""
python3 04_basis_sweep/scripts/plot_ridge_lambda_sweep.py \
    --summary-csv "$SUMMARY_CSV" --plot-dir "$PLOT_DIR"

echo "== Done. Results in ${PLOT_DIR}/ =="
