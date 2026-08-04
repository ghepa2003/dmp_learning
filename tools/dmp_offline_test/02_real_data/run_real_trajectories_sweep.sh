#!/usr/bin/env bash
# Sweep and comparative analysis on the three real trajectories (Traj A, Traj B, Traj C).
#
# Runs learning and replay for each real trajectory on the 2 regression methods
# specified in REGRESSION_VARIANTS (e.g. LWR vs Ridge).
# Generates per-trial plots (3D, position, angular error) and comparative plots on metrics.

set -euo pipefail

N_BASIS="${1:-20}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT_DIR"

PLOT_DIR="plots/02_real_data"
mkdir -p build data "${PLOT_DIR}" weights

# --------------------------------------------------------------------------
# Build learn_and_test_dmp
# --------------------------------------------------------------------------
PKG_DIR="../../src/haptic_dmp_learning"
if [ ! -x build/learn_and_test_dmp ] || \
   [ "${PKG_DIR}/src/core/dmp.cpp" -nt build/learn_and_test_dmp ] || \
   [ "${PKG_DIR}/src/core/quaternion_dmp.cpp" -nt build/learn_and_test_dmp ] || \
   [ "${PKG_DIR}/src/core/dmp_io.cpp" -nt build/learn_and_test_dmp ] || \
   [ "03_time_sweep/learn_and_test_dmp.cpp" -nt build/learn_and_test_dmp ]; then
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
fi

# --------------------------------------------------------------------------
# Locate the 3 real trajectories (trajA, trajB, trajC)
# --------------------------------------------------------------------------
TRAJ_FILES=()
for name in "demo_raw_trajA.csv" "demo_raw_trajB.csv" "demo_raw_trajC.csv"; do
    if [ -f "../../${name}" ]; then
        TRAJ_FILES+=("../../${name}")
    elif [ -f "${HOME}/thesis_ws/${name}" ]; then
        TRAJ_FILES+=("${HOME}/thesis_ws/${name}")
    elif [ -f "data/${name}" ]; then
        TRAJ_FILES+=("data/${name}")
    fi
done

if [ ${#TRAJ_FILES[@]} -eq 0 ]; then
    echo "[ERROR] No demo_raw_traj*.csv files found!" >&2
    exit 1
fi

# --------------------------------------------------------------------------
# Regression methods in REGRESSION_VARIANTS (e.g. LWR and Ridge)
# --------------------------------------------------------------------------
REGRESSION_VARIANTS=(
    "lwr_nofilter:04_basis_sweep/test_configs/lwr_nofilter.yaml"
    "ridge_filter:04_basis_sweep/test_configs/ridge_filter.yaml"
)

SUMMARY_CSV="${PLOT_DIR}/real_trajectories_summary.csv"
rm -f "$SUMMARY_CSV"

echo "== Starting analysis on ${#TRAJ_FILES[@]} real trajectories x ${#REGRESSION_VARIANTS[@]} variants (n_basis=${N_BASIS}) =="

for demo_csv in "${TRAJ_FILES[@]}"; do
    filename="$(basename "$demo_csv")"
    if [[ "$filename" =~ (traj[A-Z0-9]+) ]]; then
        traj_id="${BASH_REMATCH[1]}"
    else
        traj_id="${filename%.csv}"
    fi

    for rv in "${REGRESSION_VARIANTS[@]}"; do
        rv_label="${rv%%:*}"
        rv_flags="${rv#*:}"
        if [ "$rv_flags" = "$rv_label" ] || [ ! -f "$rv_flags" ]; then
            rv_flags=""
        fi

        label="real_${traj_id}_${rv_label}"
        yaml_out="weights/${label}.yaml"
        replay_out="data/replay_${label}.csv"

        echo "---- Real Trajectory ${traj_id} / Variant ${rv_label} ----"
        build/learn_and_test_dmp "$demo_csv" "$yaml_out" "$replay_out" "$SUMMARY_CSV" "$label" \
            "$N_BASIS" - - - "${rv_flags:-}"

        python3 03_time_sweep/plot_dmp_timesweep.py \
            --demo "$demo_csv" \
            --replay "$replay_out" \
            --plot-dir "${PLOT_DIR}" \
            --label "$label"
    done
done

echo ""
echo "== Generating comparative metric plots across the 3 trajectories =="
python3 02_real_data/plot_real_trajectories_study.py \
    --summary-csv "$SUMMARY_CSV" \
    --plot-dir "${PLOT_DIR}"

echo ""
echo "== Analysis completed. Numerical summary in ${SUMMARY_CSV}, plots in ${PLOT_DIR}/ =="

