#!/bin/bash
# run_velocity_ab_campaign.sh — Automates Phase B A/B Campaign on Gazebo
set -eo pipefail

CONFIG_YAML="/root/ros_workspaces/ros2/franka_ws/src/franka_ros2/franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml"

EVAL_DIR="/root/thesis_ws/tools/gazebo_cartesian_eval"
WEIGHTS_PRODMP="/root/thesis_ws/runs/20260915_090358_fit_reach_task_baseline_prodmp/weights.yaml"
WEIGHTS_DMP="/root/thesis_ws/reach_task_baseline_nbasis200.yaml"
ORIENTATION_PATH="/root/thesis_ws/reach_task_baseline_nbasis200.yaml"

clean_all() {
    ros2 daemon stop 2>/dev/null || true
    pkill -2 -f "ros2 bag record" 2>/dev/null || true
    sleep 1
    pkill -9 -f "ign gazebo" 2>/dev/null || true
    pkill -9 -f "gz_sim" 2>/dev/null || true
    pkill -9 -f "ros2 launch franka_gazebo_bringup" 2>/dev/null || true
    pkill -9 -f "robot_state_publisher" 2>/dev/null || true
    pkill -9 -f "controller_manager" 2>/dev/null || true
    pkill -9 -f "joint_state_broadcaster" 2>/dev/null || true
    pkill -9 -f "velocity_cartesian_controller" 2>/dev/null || true
    pkill -9 -f "prodmp_gazebo_executor_node" 2>/dev/null || true
    pkill -9 -f "dmp_gazebo_executor_node" 2>/dev/null || true
    pkill -9 -f "ros2 topic pub" 2>/dev/null || true
    pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
    pkill -9 -f "parameter_bridge" 2>/dev/null || true
    rm -rf /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
    sleep 1
    ros2 daemon start 2>/dev/null || true
}

set_feedforward() {
    local enable="$1"
    if [ "$enable" = "true" ]; then
        sed -i 's/feedforward_enabled: false/feedforward_enabled: true/' "$CONFIG_YAML"
    else
        sed -i 's/feedforward_enabled: true/feedforward_enabled: false/' "$CONFIG_YAML"
    fi
}

run_test() {
    local run_name="$1"
    local ff_enabled="$2"
    local formulation="$3" # "prodmp" or "dmp"
    local tx="$4"
    local ty="$5"
    local tz="$6"

    echo ""
    echo "=================================================================="
    echo "RUN: ${run_name} | FF=${ff_enabled} | Form=${formulation}"
    echo "=================================================================="

    clean_all
    set_feedforward "$ff_enabled"
    rm -rf "$EVAL_DIR/bags/$run_name"

    # 1. Launch Gazebo Velocity Controller Bringup
    local launch_log="/tmp/launch_${run_name}.log"
    ros2 launch franka_gazebo_bringup gazebo_velocity_cartesian_control.launch.py headless:=true load_gripper:=true > "$launch_log" 2>&1 &
    local launch_pid=$!

    # Wait for controller activation
    local waited=0
    while ! grep -q "Successfully loaded controller velocity_cartesian_controller" "$launch_log" 2>/dev/null; do
        if ! kill -0 "$launch_pid" 2>/dev/null; then
            echo "[ERROR] Bringup process died for $run_name"
            clean_all
            return 1
        fi
        sleep 1
        waited=$((waited + 1))
        if [ "$waited" -ge 45 ]; then
            echo "[ERROR] Timeout waiting for controller in $run_name"
            clean_all
            return 1
        fi
    done
    sleep 3

    # Check activation log message
    if [ "$ff_enabled" = "true" ]; then
        if grep -q "Velocity feedforward: ENABLED" "$launch_log"; then
            echo "[OK] Confirmed log: 'Velocity feedforward: ENABLED'"
        else
            echo "[WARN] Did not find 'Velocity feedforward: ENABLED' in log!"
        fi
    else
        if grep -q "Velocity feedforward: DISABLED" "$launch_log"; then
            echo "[OK] Confirmed log: 'Velocity feedforward: DISABLED (Kp-only)'"
        fi
    fi

    # 2. Record ROS2 Bag
    ros2 bag record -o "$EVAL_DIR/bags/$run_name" \
        /velocity_cartesian_controller/target_pose_aligned \
        /velocity_cartesian_controller/actual_pose \
        /joint_states > /dev/null 2>&1 &
    local bag_pid=$!
    sleep 2

    # 3. Target Odometry if Goal != Baseline
    local pub_pid=""
    if [ -n "$tx" ] && [ -n "$ty" ] && [ -n "$tz" ]; then
        ros2 topic pub -r 10 /free_target_object/odometry nav_msgs/msg/Odometry \
            "{pose: {pose: {position: {x: $tx, y: $ty, z: $tz}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}}" > /dev/null 2>&1 &
        pub_pid=$!
        sleep 2
    fi

    # 4. Run Executor Node
    local exec_log="/tmp/exec_${run_name}.log"
    if [ "$formulation" = "prodmp" ]; then
        local target_odom_opt="false"
        [ -n "$tx" ] && target_odom_opt="true"
        timeout 35 ros2 run haptic_dmp_learning prodmp_gazebo_executor_node \
            --ros-args \
            -p target_pose_topic:="/target_pose" \
            -p target_twist_topic:="/target_twist" \
            -p weights_yaml_path:="$WEIGHTS_PRODMP" \
            -p orientation_weights_yaml_path:="$ORIENTATION_PATH" \
            -p target_odom_required:="$target_odom_opt" \
            -p use_sim_time:=true > "$exec_log" 2>&1 || true
    else
        timeout 35 ros2 run haptic_dmp_learning dmp_gazebo_executor_node \
            --ros-args \
            -p target_pose_topic:="/target_pose" \
            -p target_twist_topic:="/target_twist" \
            -p weights_yaml_path:="$WEIGHTS_DMP" \
            -p use_sim_time:=true > "$exec_log" 2>&1 || true
    fi

    sleep 2
    kill -INT "$bag_pid" 2>/dev/null || true
    wait "$bag_pid" 2>/dev/null || true

    # Check for feedforward warning/error logs
    if [ "$ff_enabled" = "true" ]; then
        if grep -q "feedforward_enabled=true but" "$launch_log"; then
            echo "[WARNING] Throttled feedforward warnings detected in $launch_log:"
            grep "feedforward_enabled=true but" "$launch_log" | head -n 5
        else
            echo "[OK] ZERO feedforward throttled errors/warnings during run."
        fi
    fi

    clean_all

    # 5. Extract CSV
    python3 "$EVAL_DIR/scripts/extract_bag_to_csv.py" "$EVAL_DIR/bags/$run_name" "$run_name" "velocity_cartesian_controller" > /dev/null 2>&1
    python3 "$EVAL_DIR/scripts/manipulability_check/extract_joint_states_to_csv.py" "$EVAL_DIR/bags/$run_name" "$run_name" "$EVAL_DIR/data" > /dev/null 2>&1
    echo "[DONE] $run_name"
}

# ---------------------------------------------------------------------------
# Test Suite Execution: Baseline Pilot
# ---------------------------------------------------------------------------
source /opt/ros/humble/setup.bash
source /root/ros_workspaces/ros2/franka_ws/install/setup.bash 2>/dev/null || true
source /root/thesis_ws/install/setup.bash 2>/dev/null || true

run_test "vel_ab_baseline_noff_prodmp" "false" "prodmp" "" "" ""
run_test "vel_ab_baseline_ff_prodmp" "true" "prodmp" "" "" ""
run_test "vel_ab_baseline_ff_dmp" "true" "dmp" "" "" ""
