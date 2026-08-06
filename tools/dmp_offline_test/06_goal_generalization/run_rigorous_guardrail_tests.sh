#!/usr/bin/env bash
# Compiles and runs the rigorous guardrail benchmark (Test 1, Test 2, Test 3a/3b, Test 4)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT_DIR"

OUT_DIR="plots/06_goal_generalization"
mkdir -p "${OUT_DIR}/build" "${OUT_DIR}/data"

PKG_DIR="../../src/haptic_dmp_learning"

echo "== Building test_guardrails_rigorous =="
g++ -std=c++17 -O2 \
    -I "${PKG_DIR}/include" \
    -I 03_time_sweep \
    -I common \
    -I/usr/include/eigen3 \
    06_goal_generalization/test_guardrails_rigorous.cpp \
    common/metrics.cpp \
    "${PKG_DIR}/src/core/dmp.cpp" \
    "${PKG_DIR}/src/core/quaternion_dmp.cpp" \
    "${PKG_DIR}/src/core/dmp_io.cpp" \
    -o "${OUT_DIR}/build/test_guardrails_rigorous" \
    -lyaml-cpp

echo "== Executing Rigorous Guardrail Benchmark =="
"${OUT_DIR}/build/test_guardrails_rigorous" "../../demo_raw_trajA.csv" "${OUT_DIR}"

echo ""
echo "== Generating Error-over-Time Plot (Synchronized Run) =="
python3 06_goal_generalization/plot_guardrails_rigorous.py \
    --timeseries "${OUT_DIR}/data/guardrail_timeseries.csv" \
    --summary "${OUT_DIR}/data/guardrail_summary_metrics.csv" \
    --out-dir "${OUT_DIR}"

echo ""
echo "=========================================================================================="
echo "  BENCHMARK RIGOROSO DEI GUARDRAIL COMPLETATO CON SUCCESSO!"
echo "  Plot salvato in: ${OUT_DIR}/guardrails_rigorous_comparison.png"
echo "=========================================================================================="
