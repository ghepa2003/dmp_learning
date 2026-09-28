#!/bin/bash

# Run Repeatability Test for Goal 2 (rep2, rep3) and Goal 4 (rep2)

source /opt/ros/humble/setup.bash
source /root/thesis_ws/install/setup.bash 2>/dev/null || true
source /root/ros_workspaces/ros2/franka_ws/install/setup.bash 2>/dev/null || true

WEIGHTS_PATH="/root/thesis_ws/runs/20260915_090358_fit_reach_task_baseline_prodmp/weights.yaml"
ORIENTATION_PATH="/root/thesis_ws/reach_task_baseline_nbasis200.yaml"
EVAL_DIR="/root/thesis_ws/tools/gazebo_cartesian_eval"

clean_processes() {
    echo "=== Cleaning processes ==="
    pkill -2 -f "record" 2>/dev/null || true
    sleep 2
    pkill -9 -f "ign gazebo" 2>/dev/null || true
    pkill -9 -f "gz_sim" 2>/dev/null || true
    pkill -9 -f "robot_state_publisher" 2>/dev/null || true
    pkill -9 -f "controller_manager" 2>/dev/null || true
    pkill -9 -f "prodmp_gazebo_executor_node" 2>/dev/null || true
    pkill -9 -f "franka_gazebo_bringup" 2>/dev/null || true
    pkill -9 -f "free_target_object/odometry" 2>/dev/null || true
    pkill -9 -f "record" 2>/dev/null || true
    sleep 3
}

run_test() {
    local run_name=$1
    local tx=$2
    local ty=$3
    local tz=$4

    echo "================================================================"
    echo "Starting test: $run_name"
    echo "Target: ($tx, $ty, $tz)"
    echo "================================================================"

    clean_processes
    rm -rf "$EVAL_DIR/bags/$run_name"

    echo "1. Launching Gazebo..."
    ros2 launch franka_gazebo_bringup gazebo_cartesian_impedance_control_headless.launch.py load_gripper:=true > /tmp/gazebo_${run_name}.log 2>&1 &
    sleep 12

    echo "2. Starting ROS2 bag recording..."
    ros2 bag record -o "$EVAL_DIR/bags/$run_name" /cartesian_impedance_controller/target_pose_aligned /cartesian_impedance_controller/actual_pose /joint_states > /tmp/bag_${run_name}.log 2>&1 &
    sleep 3

    echo "3. Starting target odometry publisher..."
    ros2 topic pub -r 10 /free_target_object/odometry nav_msgs/msg/Odometry "{pose: {pose: {position: {x: $tx, y: $ty, z: $tz}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}}" > /tmp/pub_${run_name}.log 2>&1 &
    sleep 2

    echo "4. Running prodmp_gazebo_executor_node..."
    ros2 run haptic_dmp_learning prodmp_gazebo_executor_node \
        --ros-args \
        -p weights_yaml_path:="$WEIGHTS_PATH" \
        -p orientation_weights_yaml_path:="$ORIENTATION_PATH" \
        -p target_odom_required:=true \
        -p use_sim_time:=true

    echo "5. Execution finished. Waiting 3s for dwell / bag flush..."
    sleep 3

    echo "6. Stopping bag recording cleanly..."
    pkill -2 -f "record" 2>/dev/null || true
    sleep 3

    echo "7. Tearing down background processes..."
    clean_processes

    echo "8. Extracting bag to CSV..."
    python3 "$EVAL_DIR/scripts/extract_bag_to_csv.py" "$EVAL_DIR/bags/$run_name" "$run_name"

    echo "Done with $run_name"
    sleep 2
}

# Run Goal 2 Repetition 2
run_test "reach_task_goal_2_rep2_prodmp" 0.3407 -0.0759 0.2620

# Run Goal 2 Repetition 3
run_test "reach_task_goal_2_rep3_prodmp" 0.3407 -0.0759 0.2620

# Run Goal 4 Repetition 2
run_test "reach_task_goal_4_rep2_prodmp" 0.3507 -0.1459 0.3420

echo "All repetitions finished successfully!"
