#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PKG_DIR="../../src/haptic_dmp_learning"

mkdir -p build data weights plots

DEFAULT_YAML="/home/lorenzo/thesis_ws/dmp_weights.yaml"
if [ ! -f "$DEFAULT_YAML" ] && [ -f "weights/dmp_weights_reach_lift_pitch.yaml" ]; then
    DEFAULT_YAML="weights/dmp_weights_reach_lift_pitch.yaml"
fi
YAML_PATH="${1:-$DEFAULT_YAML}"

g++ -std=c++17 -O2 \
    -I "$PKG_DIR/include" \
    -I/usr/include/eigen3 \
    replay_saved_dmp.cpp \
    "$PKG_DIR/src/core/dmp.cpp" \
    "$PKG_DIR/src/core/quaternion_dmp.cpp" \
    "$PKG_DIR/src/core/dmp_io.cpp" \
    -lyaml-cpp \
    -o build/replay_saved_dmp

if [ -n "$2" ]; then
    ./build/replay_saved_dmp "$YAML_PATH" "$2"
else
    ./build/replay_saved_dmp "$YAML_PATH"
fi

echo ""
echo "Fatto. Replay generato in data/replay_from_yaml.csv"
echo "Per il grafico: python3 plot_real_demo.py"
