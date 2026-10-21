#!/bin/bash
# tests/golden/g25/build_and_run_g25.sh
# Eseguito nel container franka (/root/thesis_ws)

set -eo pipefail

WS_ROOT="${THESIS_WS:-/root/thesis_ws}"
cd "${WS_ROOT}"

source /root/env_thesis.sh

mkdir -p _gate_out

echo "=== [G25] Compilazione step3_cells_gate ==="
g++ -O3 -std=gnu++17 \
  -I/root/thesis_ws/src/satellite_grasp_planner/include \
  -I/root/thesis_ws/install/franka_cartesian_control/include \
  -I/root/thesis_ws/install/haptic_dmp_learning/include \
  -I/usr/include/eigen3 \
  -I/opt/ros/humble/include \
  -I/opt/ros/humble/include/pinocchio/deprecated \
  -I/opt/ros/humble/include/urdfdom \
  -I/opt/ros/humble/include/urdfdom_headers \
  /root/thesis_ws/tests/golden/g25/step3_cells.cpp \
  -o /root/thesis_ws/_gate_out/step3_cells_gate \
  -Wl,-rpath,/root/thesis_ws/install/satellite_grasp_planner/lib:/root/thesis_ws/install/franka_cartesian_control/lib:/opt/ros/humble/lib/x86_64-linux-gnu \
  /root/thesis_ws/install/satellite_grasp_planner/lib/libsatellite_grasp_planner_core.a \
  /root/thesis_ws/install/haptic_dmp_learning/lib/libhaptic_dmp_learning_core.a \
  /root/thesis_ws/install/franka_cartesian_control/lib/libfranka_cartesian_control_core.a \
  /root/thesis_ws/install/franka_cartesian_control/lib/libfranka_cartesian_control.so \
  -pthread /opt/ros/humble/lib/x86_64-linux-gnu/libpinocchio_parsers.so.4.0.0 \
  /opt/ros/humble/lib/x86_64-linux-gnu/liburdfdom_sensor.so.3.0 \
  /opt/ros/humble/lib/x86_64-linux-gnu/liburdfdom_model.so.3.0 \
  /opt/ros/humble/lib/x86_64-linux-gnu/liburdfdom_world.so.3.0 \
  /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.1.0 \
  /usr/lib/x86_64-linux-gnu/libtinyxml.so \
  /opt/ros/humble/lib/x86_64-linux-gnu/libpinocchio_collision.so.4.0.0 \
  /opt/ros/humble/lib/x86_64-linux-gnu/libpinocchio_default.so.4.0.0 \
  /opt/ros/humble/lib/x86_64-linux-gnu/libcoal.so.3.0.3 \
  /usr/lib/x86_64-linux-gnu/libboost_filesystem.so \
  /usr/lib/x86_64-linux-gnu/libboost_serialization.so \
  /opt/ros/humble/lib/x86_64-linux-gnu/liboctomap.so.1.9.8 \
  /opt/ros/humble/lib/x86_64-linux-gnu/liboctomath.so.1.9.8 \
  /usr/lib/x86_64-linux-gnu/libyaml-cpp.so.0.7.0

echo "=== [G25] Esecuzione offline con --full-precision ==="
./_gate_out/step3_cells_gate \
  --input tests/golden/g25/g25_cells_input.csv \
  --output _gate_out/g25_cells_output_run.csv \
  --cell C0,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14,C15 \
  --full-precision

echo "=== [G25] Confronto con golden_full_precision.csv ==="
python3 tests/golden/g25/compare_g25.py tests/golden/g25/golden_full_precision.csv _gate_out/g25_cells_output_run.csv

echo "=== [G25] PASS: Output identico al golden full precision ==="
