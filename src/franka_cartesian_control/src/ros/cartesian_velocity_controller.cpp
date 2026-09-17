#include "franka_cartesian_control/ros/cartesian_velocity_controller.hpp"
#include "franka_cartesian_control/core/cartesian_error.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <future>

namespace franka_cartesian_control {
namespace ros_wrapper {

/**
 * @brief Initializes parameters for the Cartesian velocity controller.
 * Declares ROS 2 parameters with default values and loads IK solver gains.
 */
controller_interface::CallbackReturn CartesianVelocityController::on_init() {
    try {
        auto node = get_node();

        // 1. Declare joint names for the 7-DOF Franka manipulator
        if (!node->has_parameter("joint_names")) {
            node->declare_parameter<std::vector<std::string>>(
                "joint_names",
                std::vector<std::string>{"fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4",
                                          "fer_joint5", "fer_joint6", "fer_joint7"});
        }
        joint_names_ = node->get_parameter("joint_names").as_string_array();

        // 2. Declare end-effector link frame name (must exist in URDF tree)
        if (!node->has_parameter("ee_frame_name")) {
            node->declare_parameter<std::string>("ee_frame_name", "fer_link8");
        }
        ee_frame_name_ = node->get_parameter("ee_frame_name").as_string();

        // 3. Declare subscribed target pose topic
        if (!node->has_parameter("target_pose_topic")) {
            node->declare_parameter<std::string>("target_pose_topic", "/target_pose");
        }
        target_pose_topic_ = node->get_parameter("target_pose_topic").as_string();

        // 4. Declare DLS IK Solver Parameters (gains, damping lambda, velocity saturations)
        core::VelocityIkSolver::Params ik_params;

        if (!node->has_parameter("kp_linear")) {
            node->declare_parameter<double>("kp_linear", ik_params.kp_linear);
        }
        ik_params.kp_linear = node->get_parameter("kp_linear").as_double();

        if (!node->has_parameter("kp_angular")) {
            node->declare_parameter<double>("kp_angular", ik_params.kp_angular);
        }
        ik_params.kp_angular = node->get_parameter("kp_angular").as_double();

        if (!node->has_parameter("damping_lambda")) {
            node->declare_parameter<double>("damping_lambda", ik_params.damping_lambda);
        }
        ik_params.damping_lambda = node->get_parameter("damping_lambda").as_double();

        if (!node->has_parameter("max_linear_speed")) {
            node->declare_parameter<double>("max_linear_speed", ik_params.max_linear_speed);
        }
        ik_params.max_linear_speed = node->get_parameter("max_linear_speed").as_double();

        if (!node->has_parameter("max_angular_speed")) {
            node->declare_parameter<double>("max_angular_speed", ik_params.max_angular_speed);
        }
        ik_params.max_angular_speed = node->get_parameter("max_angular_speed").as_double();

        if (!node->has_parameter("max_joint_speed")) {
            node->declare_parameter<double>("max_joint_speed", ik_params.max_joint_speed);
        }
        ik_params.max_joint_speed = node->get_parameter("max_joint_speed").as_double();

        ik_solver_.setParams(ik_params);

        // 4b. Velocity feedforward (v_cmd = v_ff + Kp * e). Default DISABLED:
        // byte-identical Kp-only behaviour unless explicitly turned on, so
        // existing/recorded runs are unaffected and A/B comparison is possible
        // without re-running the campaign.
        if (!node->has_parameter("feedforward_enabled")) {
            node->declare_parameter<bool>("feedforward_enabled", false);
        }
        feedforward_enabled_ = node->get_parameter("feedforward_enabled").as_bool();

        if (!node->has_parameter("target_twist_topic")) {
            node->declare_parameter<std::string>("target_twist_topic", "/target_twist");
        }
        target_twist_topic_ = node->get_parameter("target_twist_topic").as_string();

        if (!node->has_parameter("feedforward_tolerance_sec")) {
            node->declare_parameter<double>("feedforward_tolerance_sec", 0.05);
        }
        feedforward_tolerance_sec_ = node->get_parameter("feedforward_tolerance_sec").as_double();

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Configures command interfaces requested by this controller from hardware/Gazebo.
 * Velocity controller claims '<joint_name>/velocity' interfaces for all 7 joints.
 */
controller_interface::InterfaceConfiguration
CartesianVelocityController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& jn : joint_names_) {
        config.names.push_back(jn + "/velocity");
    }
    return config;
}

/**
 * @brief Configures feedback state interfaces read by this controller.
 * Reads position and velocity for all 7 joints to update Pinocchio kinematic state.
 */
controller_interface::InterfaceConfiguration
CartesianVelocityController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& jn : joint_names_) {
        config.names.push_back(jn + "/position");
        config.names.push_back(jn + "/velocity");
    }
    return config;
}

/**
 * @brief Lifecycle configure transition: builds RobotModel, initializes subscribers and real-time publishers.
 */
controller_interface::CallbackReturn CartesianVelocityController::on_configure(
    const rclcpp_lifecycle::State&) {
    auto node = get_node();

    // 1. Refresh tunable IK Solver parameters from node parameters
    core::VelocityIkSolver::Params ik_params;
    ik_params.kp_linear = node->get_parameter("kp_linear").as_double();
    ik_params.kp_angular = node->get_parameter("kp_angular").as_double();
    ik_params.damping_lambda = node->get_parameter("damping_lambda").as_double();
    ik_params.max_linear_speed = node->get_parameter("max_linear_speed").as_double();
    ik_params.max_angular_speed = node->get_parameter("max_angular_speed").as_double();
    ik_params.max_joint_speed = node->get_parameter("max_joint_speed").as_double();
    ik_solver_.setParams(ik_params);

    RCLCPP_INFO(
        node->get_logger(),
        "IK Solver configured: kp_linear=%.2f, kp_angular=%.2f, damping_lambda=%.3f, "
        "max_linear_speed=%.2f, max_angular_speed=%.2f, max_joint_speed=%.2f",
        ik_params.kp_linear, ik_params.kp_angular, ik_params.damping_lambda,
        ik_params.max_linear_speed, ik_params.max_angular_speed, ik_params.max_joint_speed);

    feedforward_enabled_ = node->get_parameter("feedforward_enabled").as_bool();
    target_twist_topic_ = node->get_parameter("target_twist_topic").as_string();
    feedforward_tolerance_sec_ = node->get_parameter("feedforward_tolerance_sec").as_double();
    RCLCPP_INFO(node->get_logger(),
                "Velocity feedforward: %s (topic='%s', tolerance=%.3fs, only used when enabled)",
                feedforward_enabled_ ? "ENABLED" : "DISABLED (Kp-only)",
                target_twist_topic_.c_str(), feedforward_tolerance_sec_);

    // 2. Fetch URDF XML model string from /robot_description (transient_local QoS)
    auto urdf_opt = fetchRobotDescription(node->get_logger());
    if (!urdf_opt) {
        return controller_interface::CallbackReturn::ERROR;
    }
    std::string urdf_xml = *urdf_opt;

    // 3. Build Pinocchio core RobotModel
    try {
        robot_model_ = std::make_unique<core::RobotModel>(urdf_xml, joint_names_, ee_frame_name_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to build RobotModel: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    // 4. Subscribe to target pose topic (writes lock-free into RealtimeBuffer)
    target_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        target_pose_topic_, rclcpp::QoS(10),
        std::bind(&CartesianVelocityController::targetPoseCallback, this, std::placeholders::_1));

    // 4a. Velocity feedforward subscription - created only when enabled, so a
    // disabled controller carries no unused subscription and target_twist_received_
    // simply stays false forever (harmless: the flag is never consulted unless
    // feedforward_enabled_ is true).
    if (feedforward_enabled_) {
        target_twist_sub_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
            target_twist_topic_, rclcpp::QoS(10),
            std::bind(&CartesianVelocityController::targetTwistCallback, this, std::placeholders::_1));
    }

    // 5. Pre-allocate real-time safe publishers for telemetry
    aligned_target_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/target_pose_aligned", rclcpp::QoS(10));
    rt_aligned_target_pub_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
        aligned_target_pub_);

    actual_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/actual_pose", rclcpp::QoS(10));
    rt_actual_pose_pub_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
        actual_pose_pub_);

    return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Lifecycle activate transition: captures initial end-effector pose and resets frame alignment.
 */
controller_interface::CallbackReturn CartesianVelocityController::on_activate(
    const rclcpp_lifecycle::State&) {
    core::RobotModel::JointVector q, dq;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        q(static_cast<int>(i)) = state_interfaces_[2 * i].get_value();
        dq(static_cast<int>(i)) = state_interfaces_[2 * i + 1].get_value();
    }
    robot_model_->update(q, dq);

    // Anchor frame alignment to physical activation pose
    frame_aligner_.reset(robot_model_->eePosition(), robot_model_->eeOrientation());
    target_received_.store(false);
    target_twist_received_.store(false);

    return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Lifecycle deactivate transition: defensively commands zero velocity to all joints.
 */
controller_interface::CallbackReturn CartesianVelocityController::on_deactivate(
    const rclcpp_lifecycle::State&) {
    for (auto& ci : command_interfaces_) {
        ci.set_value(0.0);
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Asynchronous subscription callback: receives incoming target pose and writes to lock-free RT buffer.
 */
void CartesianVelocityController::targetPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    target_pose_buffer_.writeFromNonRT(*msg);
    target_received_.store(true);
}

void CartesianVelocityController::targetTwistCallback(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    target_twist_buffer_.writeFromNonRT(*msg);
    target_twist_received_.store(true);
}

/**
 * @brief Real-time deterministic control loop (1 kHz): executes closed-loop resolved-rate velocity IK.
 */
controller_interface::return_type CartesianVelocityController::update(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
    // 1. Safety check: hold stationary position if no target pose has been received yet
    if (!target_received_.load()) {
        for (auto& ci : command_interfaces_) ci.set_value(0.0);
        return controller_interface::return_type::OK;
    }

    // 2. Read current joint states from hardware interfaces and update Pinocchio kinematics
    core::RobotModel::JointVector q, dq;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        q(static_cast<int>(i)) = state_interfaces_[2 * i].get_value();
        dq(static_cast<int>(i)) = state_interfaces_[2 * i + 1].get_value();
    }
    robot_model_->update(q, dq);

    // 3. Lock-free read of raw target pose from real-time buffer
    const auto& raw_target = *target_pose_buffer_.readFromRT();
    Eigen::Vector3d raw_pos(raw_target.pose.position.x, raw_target.pose.position.y,
                             raw_target.pose.position.z);
    Eigen::Quaterniond raw_quat(raw_target.pose.orientation.w, raw_target.pose.orientation.x,
                                 raw_target.pose.orientation.y, raw_target.pose.orientation.z);
    raw_quat.normalize();

    // 4. Align raw trajectory pose to robot base frame using rigid offset
    auto logger = get_node()->get_logger();
    Eigen::Vector3d target_pos;
    Eigen::Quaterniond target_quat;
    frame_aligner_.align(raw_pos, raw_quat, target_pos, target_quat, &logger);

    // 5. Publish telemetry (lock-free non-blocking trylock)
    if (rt_actual_pose_pub_->trylock()) {
        auto& msg = rt_actual_pose_pub_->msg_;
        msg.header.stamp = time;
        msg.header.frame_id = "fer_link0";
        msg.pose.position.x = robot_model_->eePosition().x();
        msg.pose.position.y = robot_model_->eePosition().y();
        msg.pose.position.z = robot_model_->eePosition().z();
        msg.pose.orientation.w = robot_model_->eeOrientation().w();
        msg.pose.orientation.x = robot_model_->eeOrientation().x();
        msg.pose.orientation.y = robot_model_->eeOrientation().y();
        msg.pose.orientation.z = robot_model_->eeOrientation().z();
        rt_actual_pose_pub_->unlockAndPublish();
    }

    if (rt_aligned_target_pub_->trylock()) {
        auto& msg = rt_aligned_target_pub_->msg_;
        msg.header.stamp = time;
        msg.header.frame_id = "fer_link0";
        msg.pose.position.x = target_pos.x();
        msg.pose.position.y = target_pos.y();
        msg.pose.position.z = target_pos.z();
        msg.pose.orientation.w = target_quat.w();
        msg.pose.orientation.x = target_quat.x();
        msg.pose.orientation.y = target_quat.y();
        msg.pose.orientation.z = target_quat.z();
        rt_aligned_target_pub_->unlockAndPublish();
    }

    // 6. Compute 6D Cartesian Error (linear translation + Lie algebra orientation)
    core::CartesianError err = core::computePoseError(
        robot_model_->eePosition(), robot_model_->eeOrientation(), target_pos, target_quat);

    // 6b. Velocity feedforward (v_cmd = v_ff + Kp * e). Disabled by default -
    // feedforward stays exactly zero and the law below is byte-identical to the
    // pre-feedforward Kp-only controller. When enabled, every failure path logs
    // loudly (throttled) and falls back to v_ff = 0 (the proven Kp-only law) -
    // never a silent reduction, so "feedforward off on purpose" (this flag false)
    // stays distinguishable in the logs from "feedforward broken" (flag true but
    // one of the guards below tripped).
    core::VelocityIkSolver::Vector6d feedforward = core::VelocityIkSolver::Vector6d::Zero();
    if (feedforward_enabled_) {
        auto* clock = get_node()->get_clock().get();
        if (!target_twist_received_.load()) {
            RCLCPP_ERROR_THROTTLE(logger, *clock, 1000,
                "feedforward_enabled=true but no message ever received on target_twist "
                "topic '%s' - commanding Kp-only (v_ff=0) until a sample arrives.",
                target_twist_topic_.c_str());
        } else {
            const auto& raw_twist = *target_twist_buffer_.readFromRT();
            const double twist_age = (time - rclcpp::Time(raw_twist.header.stamp)).seconds();
            const double stamp_skew =
                std::abs((rclcpp::Time(raw_target.header.stamp) -
                          rclcpp::Time(raw_twist.header.stamp)).seconds());

            if (twist_age > feedforward_tolerance_sec_) {
                RCLCPP_ERROR_THROTTLE(logger, *clock, 1000,
                    "feedforward_enabled=true but target_twist on '%s' is stale "
                    "(age=%.4fs > tolerance=%.4fs) - commanding Kp-only (v_ff=0).",
                    target_twist_topic_.c_str(), twist_age, feedforward_tolerance_sec_);
            } else if (stamp_skew > feedforward_tolerance_sec_) {
                RCLCPP_ERROR_THROTTLE(logger, *clock, 1000,
                    "feedforward_enabled=true but target_pose/target_twist stamps are "
                    "not synchronized (skew=%.4fs > tolerance=%.4fs) - the two samples "
                    "would combine values from different rollout ticks; commanding "
                    "Kp-only (v_ff=0) instead.",
                    stamp_skew, feedforward_tolerance_sec_);
            } else {
                // Align the raw (demo-local frame) feedforward into the robot base
                // frame using the SAME offset FrameAligner just applied to the pose
                // (step 4 above) - see FrameAligner::alignVelocity(). Mirrors align()
                // exactly: linear passes through unrotated (align() only translates
                // position, never rotates it), angular is rotated by
                // orientation_offset_ (align() rotates orientation).
                Eigen::Vector3d raw_lin(raw_twist.twist.linear.x, raw_twist.twist.linear.y,
                                         raw_twist.twist.linear.z);
                Eigen::Vector3d raw_ang(raw_twist.twist.angular.x, raw_twist.twist.angular.y,
                                         raw_twist.twist.angular.z);
                Eigen::Vector3d aligned_lin, aligned_ang;
                frame_aligner_.alignVelocity(raw_lin, raw_ang, aligned_lin, aligned_ang);
                feedforward.head<3>() = aligned_lin;
                feedforward.tail<3>() = aligned_ang;
            }
        }
    }

    // 7. Solve DLS Inverse Kinematics: compute desired twist V_des and joint velocities dq_cmd
    auto twist = ik_solver_.desiredTwist(err, feedforward);
    auto dq_cmd = ik_solver_.solve(robot_model_->jacobian(), twist);

    // 8. Write commanded joint velocities directly into hardware command interfaces
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        command_interfaces_[i].set_value(dq_cmd(static_cast<int>(i)));
    }

    return controller_interface::return_type::OK;
}

}  // namespace ros_wrapper
}  // namespace franka_cartesian_control

// Register as dynamically loadable controller plugin
PLUGINLIB_EXPORT_CLASS(franka_cartesian_control::ros_wrapper::CartesianVelocityController,
                        controller_interface::ControllerInterface)