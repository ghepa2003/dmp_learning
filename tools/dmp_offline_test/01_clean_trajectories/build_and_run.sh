#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT_DIR"

PKG_DIR="../../src/haptic_dmp_learning"

mkdir -p build data weights plots/01_clean_trajectories

echo "Compiling dmp_offline_test with official sources ($PKG_DIR)..."
g++ -std=c++17 -O2 \
    -I "$PKG_DIR/include" \
    -I 01_clean_trajectories \
    -I common \
    -I/usr/include/eigen3 \
    01_clean_trajectories/test_core_offline.cpp \
    common/metrics.cpp \
    "$PKG_DIR/src/core/demonstration_recorder.cpp" \
    "$PKG_DIR/src/core/dmp.cpp" \
    "$PKG_DIR/src/core/quaternion_dmp.cpp" \
    "$PKG_DIR/src/core/dmp_io.cpp" \
    -lyaml-cpp \
    -o build/dmp_offline_test

./build/dmp_offline_test

echo ""
echo "Done."
echo "  - Generated CSV files in:   data/"
echo "  - Weight YAML files in:     weights/"
echo "  - Metrics summary in:       data/metrics_summary.csv"
echo "To plot a trajectory: python3 01_clean_trajectories/plot_dmp_test.py <trajectory_name>"
echo "Available trajectories: reach_semplice, reach_lift_pitch, rotazione_pura, reach_complesso_gradino"

