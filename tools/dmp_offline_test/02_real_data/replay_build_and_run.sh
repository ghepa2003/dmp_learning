#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT_DIR"

PKG_DIR="../../src/haptic_dmp_learning"

mkdir -p build data weights plots/02_real_data

DEFAULT_YAML="$(pwd)/dmp_weights.yaml"
if [ ! -f "$DEFAULT_YAML" ] && [ -f "weights/dmp_weights_reach_lift_pitch.yaml" ]; then
    DEFAULT_YAML="weights/dmp_weights_reach_lift_pitch.yaml"
fi
YAML_PATH="${1:-$DEFAULT_YAML}"
if [ ! -f "$YAML_PATH" ]; then
    echo "[ERRORE] Il file YAML specificato non esiste: '$YAML_PATH'" >&2
    echo "Suggerimento: usa '~/thesis_ws/dmp_weights_trajA.yaml' oppure '../../dmp_weights_trajA.yaml' (non dimenticare il tilde ~ o /home/lorenzo/)." >&2
    exit 1
fi

g++ -std=c++17 -O2 \
    -I "$PKG_DIR/include" \
    -I/usr/include/eigen3 \
    02_real_data/replay_saved_dmp.cpp \
    "$PKG_DIR/src/core/dmp.cpp" \
    "$PKG_DIR/src/core/quaternion_dmp.cpp" \
    "$PKG_DIR/src/core/dmp_io.cpp" \
    -lyaml-cpp \
    -o build/replay_saved_dmp

if [ -n "${2:-}" ]; then
    ./build/replay_saved_dmp "$YAML_PATH" "$2"
else
    ./build/replay_saved_dmp "$YAML_PATH"
fi

echo ""
echo "Done. Replay generated in data/replay_from_yaml.csv"
echo "To plot: python3 02_real_data/plot_real_demo.py"
