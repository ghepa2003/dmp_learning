#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PKG_DIR="../../src/haptic_dmp_learning"

DEFAULT_YAML="/home/lorenzo/thesis_ws/dmp_weights.yaml"
YAML_PATH="${1:-$DEFAULT_YAML}"

g++ -std=c++17 -O2 \
    -I "$PKG_DIR/include" \
    -I/usr/include/eigen3 \
    replay_saved_dmp.cpp \
    "$PKG_DIR/src/core/dmp.cpp" \
    "$PKG_DIR/src/core/quaternion_dmp.cpp" \
    "$PKG_DIR/src/core/dmp_io.cpp" \
    -lyaml-cpp \
    -o replay_saved_dmp

if [ -n "$2" ]; then
    ./replay_saved_dmp "$YAML_PATH" "$2"
else
    ./replay_saved_dmp "$YAML_PATH"
fi

echo ""
echo "Fatto. Per il grafico: python3 plot_real_demo.py"
