#!/bin/bash
set -eo pipefail

RUN_NAME="$1"
TX="$2"
TY="$3"
TZ="$4"

if [ -z "$RUN_NAME" ] || [ -z "$TX" ] || [ -z "$TY" ] || [ -z "$TZ" ]; then
    echo "Usage: $0 <run_name> <tx> <ty> <tz>"
    exit 1
fi

source /opt/ros/humble/setup.bash
source /root/thesis_ws/install/setup.bash 2>/dev/null || true
source /root/ros_workspaces/ros2/franka_ws/install/setup.bash 2>/dev/null || true

WEIGHTS_PATH="/root/thesis_ws/runs/20260915_090358_fit_reach_task_baseline_prodmp/weights.yaml"
ORIENTATION_PATH="/root/thesis_ws/reach_task_baseline_nbasis200.yaml"
EVAL_DIR="/root/thesis_ws/tools/gazebo_cartesian_eval"

clean_all() {
    echo "=== Cleaning processes and ROS2 graph ==="
    ros2 daemon stop 2>/dev/null || true
    pkill -2 -f "ros2 bag record" 2>/dev/null || true
    sleep 2
    pkill -9 -f "ign gazebo" 2>/dev/null || true
    pkill -9 -f "gz_sim" 2>/dev/null || true
    pkill -9 -f "ros2 launch franka_gazebo_bringup" 2>/dev/null || true
    pkill -9 -f "robot_state_publisher" 2>/dev/null || true
    pkill -9 -f "controller_manager" 2>/dev/null || true
    pkill -9 -f "joint_state_broadcaster" 2>/dev/null || true
    pkill -9 -f "ros2 run haptic_dmp_learning prodmp_gazebo_executor_node" 2>/dev/null || true
    pkill -9 -f "prodmp_gazebo_executor_node" 2>/dev/null || true
    pkill -9 -f "ros2 topic pub" 2>/dev/null || true
    pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
    pkill -9 -f "parameter_bridge" 2>/dev/null || true
    rm -rf /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
    sleep 2
    ros2 daemon start 2>/dev/null || true
}

echo "=================================================================="
echo "Running single goal test: $RUN_NAME"
echo "Target position: ($TX, $TY, $TZ)"
echo "=================================================================="

clean_all
rm -rf "$EVAL_DIR/bags/$RUN_NAME"

echo "1. Launching Gazebo bringup..."
ros2 launch franka_gazebo_bringup gazebo_cartesian_impedance_control_headless.launch.py load_gripper:=true > /tmp/gazebo_bringup.log 2>&1 &
sleep 12

echo "2. Starting ROS2 bag recording..."
ros2 bag record -o "$EVAL_DIR/bags/$RUN_NAME" /cartesian_impedance_controller/target_pose_aligned /cartesian_impedance_controller/actual_pose /joint_states > /tmp/bag_rec.log 2>&1 &
sleep 3

echo "3. Publishing target odometry..."
ros2 topic pub -r 10 /free_target_object/odometry nav_msgs/msg/Odometry "{pose: {pose: {position: {x: $TX, y: $TY, z: $TZ}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}}" > /tmp/target_pub.log 2>&1 &
sleep 2

echo "4. Running prodmp_gazebo_executor_node..."
timeout 28 ros2 run haptic_dmp_learning prodmp_gazebo_executor_node \
    --ros-args \
    -p weights_yaml_path:="$WEIGHTS_PATH" \
    -p orientation_weights_yaml_path:="$ORIENTATION_PATH" \
    -p target_odom_required:=true \
    -p use_sim_time:=true || true

echo "5. Rollout finished. Stopping bag recording cleanly..."
pkill -2 -f "ros2 bag record" 2>/dev/null || true
sleep 3

clean_all

echo "6. Extracting bag to CSV..."
python3 "$EVAL_DIR/scripts/extract_bag_to_csv.py" "$EVAL_DIR/bags/$RUN_NAME" "$RUN_NAME"

echo "Finished $RUN_NAME successfully."
