#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

YAML_PATH="${1:-/home/lorenzo/thesis_ws/dmp_weights.yaml}"
DEMO_PATH="${2:-/home/lorenzo/thesis_ws/demo_raw.csv}"

if [ ! -f "$YAML_PATH" ] && [ -f "dmp_weights.yaml" ]; then
    YAML_PATH="dmp_weights.yaml"
fi
if [ ! -f "$DEMO_PATH" ] && [ -f "demo_raw.csv" ]; then
    DEMO_PATH="demo_raw.csv"
fi

PKG_DIR="../../src/haptic_dmp_learning"

echo "Compilazione replay_saved_dmp..."
g++ -std=c++17 -O2 \
    -I "$PKG_DIR/include" \
    -I/usr/include/eigen3 \
    replay_saved_dmp.cpp \
    "$PKG_DIR/src/core/dmp.cpp" \
    "$PKG_DIR/src/core/dmp_io.cpp" \
    -lyaml-cpp \
    -o replay_saved_dmp

echo "Generazione replay da pesi YAML ($YAML_PATH)..."
./replay_saved_dmp "$YAML_PATH"

echo ""
echo "Generazione grafico comparativo con $DEMO_PATH..."
python3 plot_real_demo.py "$DEMO_PATH"

