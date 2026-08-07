#!/usr/bin/env bash
# Sweep 1D sul meta-parametro ridge_lambda, alla configurazione vincente
# già trovata (n_basis=200, window=0.20s, ridge attivo), su trajA.
set -euo pipefail

N_BASIS=200
WINDOW=0.20
LAMBDA_LIST=(1e-8 1e-7 1e-6 1e-5 1e-4 1e-3 1e-2 1e-1 1e0)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT_DIR"

PLOT_DIR="plots/06_ridge_lambda_sweep"
CONFIG_DIR="04_basis_sweep/test_configs/generated_lambda"
mkdir -p build data "${PLOT_DIR}" weights "${CONFIG_DIR}"

# --------------------------------------------------------------------------
# Build (riusa lo stesso binario, nessuna modifica al C++)
# --------------------------------------------------------------------------
PKG_DIR="../../src/haptic_dmp_learning"
if [ ! -x build/learn_and_test_dmp ]; then
    echo "== Building learn_and_test_dmp =="
    g++ -std=c++17 -O2 \
        -I "${PKG_DIR}/include" -I 03_time_sweep -I common -I/usr/include/eigen3 \
        03_time_sweep/learn_and_test_dmp.cpp common/metrics.cpp \
        "${PKG_DIR}/src/core/dmp.cpp" "${PKG_DIR}/src/core/quaternion_dmp.cpp" \
        "${PKG_DIR}/src/core/dmp_io.cpp" \
        -o build/learn_and_test_dmp -lyaml-cpp
fi

# --------------------------------------------------------------------------
# Trova demo_raw_trajA.csv
# --------------------------------------------------------------------------
DEMO_CSV=""
for path in "../../demo_raw_trajA.csv" "${HOME}/thesis_ws/demo_raw_trajA.csv" "data/demo_raw_trajA.csv"; do
    [ -f "$path" ] && DEMO_CSV="$path" && break
done
if [ -z "$DEMO_CSV" ]; then
    echo "[ERROR] demo_raw_trajA.csv non trovato" >&2
    exit 1
fi

SUMMARY_CSV="${PLOT_DIR}/ridge_lambda_results.csv"
rm -f "$SUMMARY_CSV"

for lambda in "${LAMBDA_LIST[@]}"; do
    config_path="${CONFIG_DIR}/lambda_${lambda}.yaml"
    cat > "$config_path" << EOF
regression:
  method: "ridge"
  ridge_lambda: ${lambda}
velocity_filter:
  enabled: true
  window_sec_1: ${WINDOW}
  window_sec_2: ${WINDOW}
EOF

    label="lambda_${lambda}"
    echo "== ridge_lambda=${lambda} (n_basis=${N_BASIS}, window=${WINDOW}) ==" >&2
    build/learn_and_test_dmp "$DEMO_CSV" "weights/${label}.yaml" \
        "data/replay_${label}.csv" "$SUMMARY_CSV" "$label" \
        "$N_BASIS" - - - "$config_path" >&2
done

echo ""
python3 04_basis_sweep/plot_ridge_lambda_sweep.py \
    --summary-csv "$SUMMARY_CSV" --plot-dir "$PLOT_DIR"

echo "== Fatto. Risultati in ${PLOT_DIR}/ =="