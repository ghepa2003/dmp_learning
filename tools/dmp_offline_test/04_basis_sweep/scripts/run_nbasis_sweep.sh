#!/usr/bin/env bash
# Sweep on the number of basis functions (n_basis). Two modes:
#   1) Synthetic (default): generates and uses synthetic demos (clean + noisy).
#   2) Real demo: skips synthetic generation, runs sweep only on provided CSV.

set -euo pipefail

REAL_DEMO=""
if [ "${1:-}" = "--real-demo" ]; then
    REAL_DEMO="${2:?Usage: $0 --real-demo <path_to_csv>}"
    shift 2
fi

DURATION="${1:-60}"
GOAL_NAME="${2:-goalA_main}"

N_BASIS_LIST=(5 10 15 20 25 30 40 50 60 80 100 150 200 300 500)

DX=0.15
DY=0.0
DZ=-0.10
ROT_AXIS="0,0,1"
ROT_ANGLE_DEG=15.0

POS_NOISE_STD=0.0005        # 0.5 mm
ORIENT_NOISE_STD_DEG=0.1
NOISE_SEEDS=(42 43 44 45 46)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$ROOT_DIR"

PLOT_DIR="plots/04_basis_sweep"
mkdir -p build data "${PLOT_DIR}" weights

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

REG_VARIANTS=(
    "lwr_nofilter:04_basis_sweep/configs/lwr_nofilter.yaml"
    "lwr_filter:04_basis_sweep/configs/lwr_filter.yaml"
    "ridge_nofilter:04_basis_sweep/configs/ridge_nofilter.yaml"
    "ridge_filter:04_basis_sweep/configs/ridge_filter.yaml"
)

run_sweep_variant () {
    local variant_name="$1"
    local demo_csv="$2"
    local rv_label="$3"
    local rv_flags="$4"

    local summary_csv="${PLOT_DIR}/nbasis_sweep_results_${variant_name}_${rv_label}.csv"
    rm -f "$summary_csv"

    echo "" >&2
    echo "== Variant: ${variant_name} | Config: ${rv_label} (demo: ${demo_csv}) ==" >&2
    if [ -n "$rv_flags" ] && [ ! -f "$rv_flags" ]; then
        echo "[WARNING] '${rv_flags}' does not exist -- using defaults." >&2
    fi

    for n_basis in "${N_BASIS_LIST[@]}"; do
        label="nbasis_$(printf '%04d' "$n_basis")_${rv_label}"
        yaml_out="weights/${variant_name}_${label}.yaml"
        replay_out="data/replay_${variant_name}_${label}.csv"

        echo "---- ${variant_name} / ${label} ----" >&2
        build/learn_and_test_dmp "$demo_csv" "$yaml_out" "$replay_out" "$summary_csv" "$label" \
            "$n_basis" - - - "${rv_flags:-}" >&2
    done
    echo "$summary_csv"
}

CLEAN_CSV="data/demo_nbasis_sweep_${DURATION}s_${GOAL_NAME}_clean.csv"
if [ -z "$REAL_DEMO" ] && [ ! -f "$CLEAN_CSV" ]; then
    echo "== Generating clean demo (duration ${DURATION}s) =="
    python3 common/scripts/generate_picking_trajectory.py \
        --duration "$DURATION" --transition-duration "$DURATION" \
        --dx "$DX" --dy "$DY" --dz "$DZ" \
        --rot-axis "$ROT_AXIS" --rot-angle-deg "$ROT_ANGLE_DEG" \
        --output "$CLEAN_CSV"
fi

NOISY_CSVS=()
if [ -z "$REAL_DEMO" ]; then
    for seed in "${NOISE_SEEDS[@]}"; do
        noisy_csv="data/demo_nbasis_sweep_${DURATION}s_${GOAL_NAME}_noisy_seed${seed}.csv"
        if [ ! -f "$noisy_csv" ]; then
            echo "== Generating synthetic noisy demo, seed=${seed} =="
            python3 common/scripts/generate_picking_trajectory.py \
                --duration "$DURATION" --transition-duration "$DURATION" \
                --dx "$DX" --dy "$DY" --dz "$DZ" \
                --rot-axis "$ROT_AXIS" --rot-angle-deg "$ROT_ANGLE_DEG" \
                --pos-noise-std "$POS_NOISE_STD" --orient-noise-std-deg "$ORIENT_NOISE_STD_DEG" \
                --noise-seed "$seed" \
                --output "$noisy_csv"
        fi
        NOISY_CSVS+=("$noisy_csv")
    done
fi

if [ -n "$REAL_DEMO" ]; then
    if [ ! -f "$REAL_DEMO" ]; then
        echo "Real demo file not found: ${REAL_DEMO}"
        exit 1
    fi
    echo "== Real demo mode: ${REAL_DEMO} =="
    REAL_PLOT_ARGS=()
    for rv in "${REG_VARIANTS[@]}"; do
        rv_label="${rv%%:*}"
        rv_flags="${rv#*:}"
        summary_csv=$(run_sweep_variant "real" "$REAL_DEMO" "$rv_label" "$rv_flags")
        REAL_PLOT_ARGS+=(--summary-csv "$summary_csv" --series-label "$rv_label")
    done

    echo ""
    echo "== Sweep completed. Generating comparative plots (real demo) =="
    python3 common/scripts/plot_nbasis_study.py "${REAL_PLOT_ARGS[@]}" --plot-dir "${PLOT_DIR}"

    echo ""
    echo "== Done. Plots in ${PLOT_DIR}/ =="
    exit 0
fi

CLEAN_PLOT_ARGS=()
for rv in "${REG_VARIANTS[@]}"; do
    rv_label="${rv%%:*}"
    rv_flags="${rv#*:}"
    clean_summary=$(run_sweep_variant "clean" "$CLEAN_CSV" "$rv_label" "$rv_flags")
    CLEAN_PLOT_ARGS+=(--summary-csv "$clean_summary" --series-label "$rv_label")
done

echo ""
echo "== Generating comparative plots for clean demo =="
python3 common/scripts/plot_nbasis_study.py "${CLEAN_PLOT_ARGS[@]}" --plot-dir "${PLOT_DIR}/clean"

NOISY_PLOT_ARGS=()
for rv in "${REG_VARIANTS[@]}"; do
    rv_label="${rv%%:*}"
    rv_flags="${rv#*:}"
    for i in "${!NOISE_SEEDS[@]}"; do
        seed="${NOISE_SEEDS[$i]}"
        noisy_csv="${NOISY_CSVS[$i]}"
        noisy_summary=$(run_sweep_variant "noisy_seed${seed}" "$noisy_csv" "$rv_label" "$rv_flags")
        NOISY_PLOT_ARGS+=(--summary-csv "$noisy_summary" --series-label "${rv_label} (seed ${seed})")
    done
done

echo ""
echo "== Generating raw plots (all seeds for 4 configurations) =="
python3 common/scripts/plot_nbasis_study.py "${NOISY_PLOT_ARGS[@]}" --plot-dir "${PLOT_DIR}/noisy"

echo ""
echo "== Aggregating seeds into mean +/- std and generating comparative plots == "
python3 common/scripts/aggregate_nbasis_seeds.py \
    --combined-csv "${PLOT_DIR}/noisy/nbasis_all_metrics.csv" \
    --plot-dir "${PLOT_DIR}"

echo ""
echo "== Done. Aggregated comparative plots in ${PLOT_DIR}/, clean plots in ${PLOT_DIR}/clean/, raw noisy plots in ${PLOT_DIR}/noisy/ =="
