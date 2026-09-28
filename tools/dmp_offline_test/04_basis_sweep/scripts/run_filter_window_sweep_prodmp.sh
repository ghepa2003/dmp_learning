#!/usr/bin/env bash
# Sweep on position filter window sizes for ProDMP on Trajectory C (demo_raw_trajC.csv).

set -euo pipefail

N_BASIS_LIST=(5 10 15 20 25 30 40 50 60 80 100 150 200 300 500)
WINDOW_LIST=(0.01 0.02 0.05 0.10 0.20)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$ROOT_DIR"

PLOT_DIR="plots/05_filter_window_sweep_prodmp"
mkdir -p build data "${PLOT_DIR}" weights 04_basis_sweep/test_configs/filter_prodmp

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

# Focus on Trajectory C
TRAJ_NAME="demo_raw_trajC.csv"
TRAJ_PATH=""
for path in "../../${TRAJ_NAME}" "${HOME}/thesis_ws/${TRAJ_NAME}" "data/${TRAJ_NAME}"; do
    [ -f "$path" ] && TRAJ_PATH="$path" && break
done

if [ -z "$TRAJ_PATH" ]; then
    echo "[ERROR] ${TRAJ_NAME} not found!" >&2
    exit 1
fi

t_id="trajC"
echo "== Running ProDMP Filter Window Sweep for Trajectory C (${TRAJ_PATH}) =="

PLOT_ARGS=()

for window in "${WINDOW_LIST[@]}"; do
    cfg_file="04_basis_sweep/test_configs/filter_prodmp/cfg_w${window}.yaml"
    cat <<EOF > "$cfg_file"
num_basis: 200
ridge_lambda: 1.0e-6
position_filter:
  enabled: true
  window_sec: ${window}
EOF

    series_label="window=${window}s (trajC, ProDMP)"
    summary_csv="${PLOT_DIR}/nbasis_results_w${window}_${t_id}.csv"

    if [ ! -f "$summary_csv" ] || [ $(wc -l < "$summary_csv") -le 1 ]; then
        rm -f "$summary_csv"
        echo "--> ProDMP | Window: ${window}s | Trajectory: ${t_id}" >&2

        for n_basis in "${N_BASIS_LIST[@]}"; do
            label="nbasis_$(printf '%04d' "$n_basis")_${t_id}_w${window}"
            yaml_out="weights/prodmp_w${window}_${t_id}_${label}.yaml"
            replay_out="data/replay_prodmp_nbasis_$(printf '%04d' "$n_basis")_${t_id}_w${window}.csv"

            build/learn_and_test_prodmp "$TRAJ_PATH" "$yaml_out" "$replay_out" "$summary_csv" "$label" \
                "$n_basis" - - - "$cfg_file" >&2
        done
    else
        echo "--> Reusing existing summary: ${summary_csv}" >&2
    fi

    PLOT_ARGS+=(--summary-csv "$summary_csv" --series-label "$series_label")
done

echo ""
echo "== Generating plots and combined CSV table for ProDMP filter window sweep =="
python3 common/scripts/plot_nbasis_study.py "${PLOT_ARGS[@]}" --plot-dir "${PLOT_DIR}" --title-suffix " (ProDMP, Trajectory C)"

if [ -f common/scripts/add_rotation_ratio_metric.py ]; then
    echo ""
    echo "== Adding rotation cumulative/net ratio metric =="
    python3 common/scripts/add_rotation_ratio_metric.py --combined-csv "${PLOT_DIR}/nbasis_all_metrics.csv" --replay-dir data
fi

echo ""
echo "== ProDMP Filter Window Sweep completed for Trajectory C. Results in ${PLOT_DIR}/ =="
