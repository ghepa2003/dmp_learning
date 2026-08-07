#!/usr/bin/env bash
# Sweep on Ridge regularization parameter (ridge_lambda) across different numbers of basis functions (n_basis)
# specifically for Trajectory A (demo_raw_trajA.csv).

set -euo pipefail

N_BASIS_LIST=(5 10 15 20 25 30 40 50 60 80 100 150 200 300 500)
LAMBDA_LIST=(1e-9 1e-7 1e-5 1e-3 1e-1 1.0)
FILTER_WINDOW="${1:-0.05}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$ROOT_DIR"

BASE_PLOT_DIR="04_basis_sweep/plots/ridge_lambda_sweep"
mkdir -p build data "${BASE_PLOT_DIR}" weights test_configs_tmp

PKG_DIR="../../src/haptic_dmp_learning"

if [ ! -x build/learn_and_test_dmp ] || \
   [ "${PKG_DIR}/src/core/dmp.cpp" -nt build/learn_and_test_dmp ] || \
   [ "${PKG_DIR}/src/core/quaternion_dmp.cpp" -nt build/learn_and_test_dmp ] || \
   [ "${PKG_DIR}/src/core/dmp_io.cpp" -nt build/learn_and_test_dmp ] || \
   [ "common/src/learn_and_test_dmp.cpp" -nt build/learn_and_test_dmp ]; then
    echo "== Building learn_and_test_dmp =="
    g++ -std=c++17 -O2 \
        -I "${PKG_DIR}/include" \
        -I common/include \
        -I/usr/include/eigen3 \
        common/src/learn_and_test_dmp.cpp \
        common/src/metrics.cpp \
        "${PKG_DIR}/src/core/dmp.cpp" \
        "${PKG_DIR}/src/core/quaternion_dmp.cpp" \
        "${PKG_DIR}/src/core/dmp_io.cpp" \
        -o build/learn_and_test_dmp \
        -lyaml-cpp
fi

TRAJ_NAME="demo_raw_trajA.csv"
TRAJ_PATH=""
if [ -f "../../${TRAJ_NAME}" ]; then
    TRAJ_PATH="../../${TRAJ_NAME}"
elif [ -f "${HOME}/thesis_ws/${TRAJ_NAME}" ]; then
    TRAJ_PATH="${HOME}/thesis_ws/${TRAJ_NAME}"
elif [ -f "data/${TRAJ_NAME}" ]; then
    TRAJ_PATH="data/${TRAJ_NAME}"
fi

if [ -z "$TRAJ_PATH" ]; then
    echo "[ERROR] ${TRAJ_NAME} not found!" >&2
    exit 1
fi

t_id="trajA"
echo "== Running Ridge Lambda Sweep for Trajectory A (${TRAJ_PATH}) =="

PLOT_ARGS=()

for lambda in "${LAMBDA_LIST[@]}"; do
    cfg_file="test_configs_tmp/cfg_ridge_lambda_${lambda}.yaml"
    cat <<EOF > "$cfg_file"
regression:
  method: "ridge"
  ridge_lambda: ${lambda}
velocity_filter:
  enabled: true
  window_sec_1: ${FILTER_WINDOW}
  window_sec_2: ${FILTER_WINDOW}
EOF

    series_label="lambda=${lambda} (trajA, Ridge)"
    summary_csv="${BASE_PLOT_DIR}/nbasis_results_lambda_${lambda}_${t_id}.csv"

    if [ ! -f "$summary_csv" ] || [ $(wc -l < "$summary_csv") -le 1 ]; then
        rm -f "$summary_csv"
        echo "--> Ridge Lambda: ${lambda} | Window: ${FILTER_WINDOW}s | Trajectory: ${t_id}" >&2

        for n_basis in "${N_BASIS_LIST[@]}"; do
            label="nbasis_$(printf '%04d' "$n_basis")_${t_id}_lambda_${lambda}"
            yaml_out="weights/lambda_${lambda}_${t_id}_${label}.yaml"
            replay_out="data/replay_nbasis_$(printf '%04d' "$n_basis")_${t_id}_ridge_lambda_${lambda}.csv"

            build/learn_and_test_dmp "$TRAJ_PATH" "$yaml_out" "$replay_out" "$summary_csv" "$label" \
                "$n_basis" - - - "$cfg_file" >&2
        done
    else
        echo "--> Reusing existing summary: ${summary_csv}" >&2
    fi

    PLOT_ARGS+=(--summary-csv "$summary_csv" --series-label "$series_label")
done

echo ""
echo "== Generating plots and combined CSV table for Ridge Lambda Sweep =="
python3 common/scripts/plot_nbasis_study.py "${PLOT_ARGS[@]}" --plot-dir "${BASE_PLOT_DIR}" --title-suffix " (Ridge Lambda Sweep)"

echo ""
echo "== Adding rotation cumulative/net ratio metric =="
python3 common/scripts/add_rotation_ratio_metric.py --combined-csv "${BASE_PLOT_DIR}/nbasis_all_metrics.csv" --replay-dir data

rm -rf test_configs_tmp

echo ""
echo "== Ridge Lambda Sweep completed successfully. Results in ${BASE_PLOT_DIR}/ =="
