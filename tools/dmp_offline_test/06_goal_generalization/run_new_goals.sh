#!/usr/bin/env bash
# Runs DMP Goal Generalization sweep on the 3 real trajectories (trajA, trajB, trajC).
#
# Fits DMP weights ONCE per trajectory (n_basis=100) and executes 5 new distinct goals
# WITHOUT re-training (using setGoal). Computes goal-reaching position and angular errors,
# prints metric summary tables, and generates visualization plots.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT_DIR"

N_BASIS="${1:-100}"
OUT_DIR="plots/06_goal_generalization"
mkdir -p "${OUT_DIR}/build" "${OUT_DIR}/data" "${OUT_DIR}/weights"

PKG_DIR="../../src/haptic_dmp_learning"

echo "== Building run_goal_generalization =="
g++ -std=c++17 -O2 \
    -I "${PKG_DIR}/include" \
    -I 03_time_sweep \
    -I common \
    -I/usr/include/eigen3 \
    06_goal_generalization/run_goal_generalization.cpp \
    common/metrics.cpp \
    "${PKG_DIR}/src/core/dmp.cpp" \
    "${PKG_DIR}/src/core/quaternion_dmp.cpp" \
    "${PKG_DIR}/src/core/dmp_io.cpp" \
    -o "${OUT_DIR}/build/run_goal_generalization" \
    -lyaml-cpp

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

FEATURE_CONFIG="04_basis_sweep/test_configs/ridge_filter.yaml"

echo ""
echo "=========================================================================================="
echo "== Starting Goal Generalization Analysis on ${#TRAJ_FILES[@]} trajectories (n_basis=${N_BASIS}) =="
echo "=========================================================================================="

for demo_csv in "${TRAJ_FILES[@]}"; do
    filename="$(basename "$demo_csv")"
    if [[ "$filename" =~ (traj[A-Z0-9]+) ]]; then
        traj_id="${BASH_REMATCH[1]}"
    else
        traj_id="${filename%.csv}"
    fi

    echo ""
    echo ">>>> Processing Real Trajectory: ${traj_id} <<<<"
    "${OUT_DIR}/build/run_goal_generalization" "$demo_csv" "${OUT_DIR}" "${traj_id}" "$N_BASIS" "$FEATURE_CONFIG"

    python3 06_goal_generalization/plot_goal_generalization.py \
        --demo "$demo_csv" \
        --replay-orig "${OUT_DIR}/data/${traj_id}_replay_orig.csv" \
        --out-dir "${OUT_DIR}" \
        --label "${traj_id}"
done

echo ""
echo "=========================================================================================="
echo "  GOAL GENERALIZATION COMPLETED SUCCESSFULLY!"
echo "  Results summary saved to: ${OUT_DIR}/goal_generalization_summary.csv"
echo "  Plots saved to: ${OUT_DIR}/*_goal_generalization.png"
echo "=========================================================================================="
