#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /root/ros_workspaces/ros2/franka_ws/install/setup.bash
source /root/thesis_ws/install/setup.bash

pkill -9 -f 'ros2 bag record' || true
pkill -9 -f 'ign gazebo' || true
pkill -9 -f 'ruby' || true
pkill -9 -f 'parameter_bridge' || true
pkill -9 -f 'ros2' || true
sleep 2

rm -rf /root/thesis_ws/tools/gazebo_cartesian_eval/bags/reach_task_baseline_impedance_kt200_delay1_fix1007 /tmp/launch_fix1007.log /tmp/bag_fix1007.log /tmp/executor_fix1007.log

echo "=== [1] Starting Gazebo Launch ==="
ros2 launch franka_gazebo_bringup gazebo_cartesian_impedance_control.launch.py headless:=true load_gripper:=true > /tmp/launch_fix1007.log 2>&1 &
LAUNCH_PID=$!

echo "Waiting for controller activation..."
for i in $(seq 1 60); do
    if grep -q "Configured and activated cartesian_impedance_controller" /tmp/launch_fix1007.log; then
        echo "Controller activated at step $i"
        break
    fi
    sleep 0.5
done

echo "Controller active. Sleeping 3s..."
sleep 3

echo "=== [2] Starting Bag Recording ==="
ros2 bag record -o /root/thesis_ws/tools/gazebo_cartesian_eval/bags/reach_task_baseline_impedance_kt200_delay1_fix1007 \
    /cartesian_impedance_controller/actual_pose \
    /cartesian_impedance_controller/target_pose \
    /cartesian_impedance_controller/target_pose_aligned \
    /cartesian_impedance_controller/force_torque_ext \
    /franka_gripper/joint_states \
    /joint_states \
    /clock > /tmp/bag_fix1007.log 2>&1 &
BAG_PID=$!

echo "Bag started. Sleeping 2s..."
sleep 2

echo "=== [3] Starting DMP Gazebo Executor Node ==="
timeout 75 ros2 run haptic_dmp_learning dmp_gazebo_executor_node \
    --ros-args \
    -p target_pose_topic:=/target_pose \
    -p target_twist_topic:=/target_twist \
    -p weights_yaml_path:=/root/thesis_ws/reach_task_baseline_nbasis200.yaml \
    -p target_odom_required:=false \
    -p use_sim_time:=true > /tmp/executor_fix1007.log 2>&1

echo "Executor finished. Sleeping 2s..."
sleep 2

echo "=== [4] Terminating Simulation and Bag ==="
pkill -INT -f 'ros2 bag record' || true
sleep 3
pkill -f 'ros2 launch' || true
pkill -f 'ign gazebo' || true
pkill -f 'ruby' || true
pkill -f 'parameter_bridge' || true
pkill -f 'ros2' || true
sleep 2
pkill -9 -f 'ign gazebo' || true
pkill -9 -f 'ruby' || true
pkill -9 -f 'parameter_bridge' || true
echo "Historical replay run _fix1007 completed successfully!"
