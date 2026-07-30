#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PKG_DIR="../../src/haptic_dmp_learning"

echo "Compilazione dmp_offline_test con sorgenti ufficiali ($PKG_DIR)..."
g++ -std=c++17 -O2 \
    -I "$PKG_DIR/include" \
    -I. \
    -I/usr/include/eigen3 \
    test_core_offline.cpp \
    metrics.cpp \
    "$PKG_DIR/src/core/demonstration_recorder.cpp" \
    "$PKG_DIR/src/core/dmp.cpp" \
    "$PKG_DIR/src/core/quaternion_dmp.cpp" \
    "$PKG_DIR/src/core/dmp_io.cpp" \
    -lyaml-cpp \
    -o dmp_offline_test

./dmp_offline_test

echo ""
echo "Fatto. File CSV/YAML generati in: $SCRIPT_DIR"
echo "Riassunto metriche: metrics_summary.csv"
echo "Per il grafico di una traiettoria: python3 plot_dmp_test.py <nome_traiettoria>"
echo "Traiettorie disponibili: reach_semplice, reach_lift_pitch, rotazione_pura, reach_complesso_gradino"
