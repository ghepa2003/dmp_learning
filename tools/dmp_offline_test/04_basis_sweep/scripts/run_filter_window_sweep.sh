#!/usr/bin/env bash
# Sweep on velocity filter window sizes and regression modes (ridge vs noridge/LWR)
# specifically for Trajectory A (demo_raw_trajA.csv).

set -euo pipefail

N_BASIS_LIST=(5 10 15 20 25 30 40 50 60 80 100 150 200 300 500)
WINDOW_LIST=(0.01 0.02 0.05 0.10 0.20)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$ROOT_DIR"

BASE_PLOT_DIR="04_basis_sweep/plots/05_filter_window_sweep"
mkdir -p build data "${BASE_PLOT_DIR}/ridge" "${BASE_PLOT_DIR}/noridge" weights test_configs_tmp

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

# Focus on Trajectory A
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
echo "== Running Filter Window Sweep for Trajectory A (${TRAJ_PATH}) =="

# Run sweep for modes: ridge and noridge
for mode in "ridge" "noridge"; do
    plot_dir="${BASE_PLOT_DIR}/${mode}"
    mkdir -p "${plot_dir}"
    PLOT_ARGS=()

    if [ "$mode" = "ridge" ]; then
        mode_label="Ridge"
        mode_title=" (Ridge Regression)"
    else
        mode_label="LWR"
        mode_title=" (Independent LWR)"
    fi

    echo ""
    echo "=================================================="
    echo "== Filter Window Sweep for Mode: ${mode_label} (Traj A) =="
    echo "=================================================="

    for window in "${WINDOW_LIST[@]}"; do
        cfg_file="test_configs_tmp/cfg_${mode}_w${window}.yaml"
        if [ "$mode" = "ridge" ]; then
            cat <<EOF > "$cfg_file"
regression:
  method: "ridge"
  ridge_lambda: 1e-6
velocity_filter:
  enabled: true
  window_sec_1: ${window}
  window_sec_2: ${window}
EOF
        else
            cat <<EOF > "$cfg_file"
regression:
  method: "independent_lwr"
velocity_filter:
  enabled: true
  window_sec_1: ${window}
  window_sec_2: ${window}
EOF
        fi

        series_label="window=${window}s (trajA, ${mode_label})"
        summary_csv="${plot_dir}/nbasis_results_w${window}_${t_id}.csv"

        if [ ! -f "$summary_csv" ] || [ $(wc -l < "$summary_csv") -le 1 ]; then
            rm -f "$summary_csv"
            echo "--> Mode: ${mode} | Window: ${window}s | Trajectory: ${t_id}" >&2

            for n_basis in "${N_BASIS_LIST[@]}"; do
                label="nbasis_$(printf '%04d' "$n_basis")_${t_id}_w${window}"
                yaml_out="weights/w${window}_${t_id}_${mode}_${label}.yaml"
                replay_out="data/replay_nbasis_$(printf '%04d' "$n_basis")_${t_id}_${mode}_w${window}.csv"

                build/learn_and_test_dmp "$TRAJ_PATH" "$yaml_out" "$replay_out" "$summary_csv" "$label" \
                    "$n_basis" - - - "$cfg_file" >&2
            done
        else
            echo "--> Reusing existing summary: ${summary_csv}" >&2
        fi

        PLOT_ARGS+=(--summary-csv "$summary_csv" --series-label "$series_label")
    done

    echo ""
    echo "== Generating plots and combined CSV table for mode: ${mode_label} =="
    python3 common/scripts/plot_nbasis_study.py "${PLOT_ARGS[@]}" --plot-dir "${plot_dir}" --title-suffix "${mode_title}"

    echo ""
    echo "== Adding rotation cumulative/net ratio metric for mode: ${mode_label} =="
    python3 common/scripts/add_rotation_ratio_metric.py --combined-csv "${plot_dir}/nbasis_all_metrics.csv" --replay-dir data
done

rm -rf test_configs_tmp

echo ""
echo "== Filter Window Sweep completed for Trajectory A. Results in ${BASE_PLOT_DIR}/ridge and ${BASE_PLOT_DIR}/noridge =="
