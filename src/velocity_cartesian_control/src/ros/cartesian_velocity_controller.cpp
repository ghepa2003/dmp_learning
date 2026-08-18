#include "velocity_cartesian_control/ros/cartesian_velocity_controller.hpp"
#include "velocity_cartesian_control/core/cartesian_error.hpp"

#include <pluginlib/class_list_macros.hpp>


#include <std_msgs/msg/string.hpp>
#include <future>

namespace velocity_cartesian_control {
namespace ros_wrapper {

// ros2_control velocity controller: reads a target Cartesian pose from a
// topic, computes the pose error against the current end-effector pose
// (via core::RobotModel, Pinocchio-based FK/Jacobian), solves the desired
// joint velocities via damped least squares (core::VelocityIkSolver), and
// commands them on the velocity command interfaces.
controller_interface::CallbackReturn CartesianVelocityController::on_init() {
    try {
        auto node = get_node();

        if (!node->has_parameter("joint_names")) {
            node->declare_parameter<std::vector<std::string>>(
                "joint_names",
                std::vector<std::string>{"fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4",
                                          "fer_joint5", "fer_joint6", "fer_joint7"});
        }
        joint_names_ = node->get_parameter("joint_names").as_string_array();

        if (!node->has_parameter("ee_frame_name")) {
            node->declare_parameter<std::string>("ee_frame_name", "fer_link8");
        }
        ee_frame_name_ = node->get_parameter("ee_frame_name").as_string();

        if (!node->has_parameter("target_pose_topic")) {
            node->declare_parameter<std::string>("target_pose_topic", "/target_pose");
        }
        target_pose_topic_ = node->get_parameter("target_pose_topic").as_string();

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

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

// Configure the controller: read the robot_description parameter (URDF XML content), build the RobotModel, and set up the target pose subscription.
controller_interface::InterfaceConfiguration
CartesianVelocityController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& jn : joint_names_) {
        config.names.push_back(jn + "/velocity");
    }
    return config;
}

// Configure the controller: specify the state interfaces (position and velocity for each joint) that the controller will read.
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

// Configure the controller: read the robot_description parameter (URDF XML content), build the RobotModel, and set up the target pose subscription.
controller_interface::CallbackReturn CartesianVelocityController::on_configure(
    const rclcpp_lifecycle::State&) {
    auto node = get_node();

    // Refresh VelocityIkSolver parameters
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

    // robot_description is NOT automatically available as a parameter on a
    // controller node - it must be fetched by subscribing to the
    // /robot_description topic (published with transient_local QoS by
    // robot_state_publisher, so a late subscriber still receives the last
    // message). Using a dedicated temporary node + executor here, separate
    // from the controller_manager's own executor, to wait synchronously
    // without interfering with it.
    std::string urdf_xml;
    {
        // Create a temporary node to subscribe to the /robot_description topic and wait for the URDF XML content.
        auto temp_node = std::make_shared<rclcpp::Node>("velocity_cartesian_control_urdf_waiter");
        std::promise<std::string> urdf_promise;
        auto urdf_future = urdf_promise.get_future();

        // Subscribe to the /robot_description topic with transient_local QoS to receive the last published message.
        auto sub = temp_node->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(1).transient_local(),
            [&urdf_promise](const std_msgs::msg::String::SharedPtr msg) {
                urdf_promise.set_value(msg->data);
            });

        // Use a single-threaded executor to spin the temporary node and wait for the URDF message.
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(temp_node);

        // Wait for the URDF message with a timeout to avoid blocking indefinitely if the message is not published.
        const auto timeout = std::chrono::seconds(5);
        auto status = executor.spin_until_future_complete(urdf_future, timeout);

        if (status != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(),
                         "Timed out waiting for /robot_description (5s) - is "
                         "robot_state_publisher running and publishing?");
            return controller_interface::CallbackReturn::ERROR;
        }
        urdf_xml = urdf_future.get();
    }

    // Build the RobotModel from the URDF XML content, resolving the end-effector frame and joint order, and initialize the target pose subscription.
    try {
        robot_model_ = std::make_unique<core::RobotModel>(urdf_xml, joint_names_, ee_frame_name_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to build RobotModel: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    // Subscribe to the target pose topic, which updates the target pose buffer in a thread-safe manner.
    target_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        target_pose_topic_, rclcpp::QoS(10),
        std::bind(&CartesianVelocityController::targetPoseCallback, this, std::placeholders::_1));

    // Create publishers for aligned target pose and actual end-effector pose, with realtime-safe wrappers for publishing in the control loop.
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

// Activate the controller: read the current joint states, update the RobotModel, and initialize the target pose to the current end-effector pose to avoid a startup jump.
controller_interface::CallbackReturn CartesianVelocityController::on_activate(
    const rclcpp_lifecycle::State&) {
    // Read the current joint states from the state interfaces and update the RobotModel with the current joint positions and velocities.
    core::RobotModel::JointVector q, dq;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        q(static_cast<int>(i)) = state_interfaces_[2 * i].get_value();
        dq(static_cast<int>(i)) = state_interfaces_[2 * i + 1].get_value();
    }
    robot_model_->update(q, dq);

    activation_ee_position_ = robot_model_->eePosition();
    activation_ee_orientation_ = robot_model_->eeOrientation();

    // Force re-capture of the DMP->robot alignment against the first
    // target received in this activation cycle (not carried over from a
    // previous activation).
    alignment_captured_ = false;
    target_received_.store(false);

    return controller_interface::CallbackReturn::SUCCESS;
}

// Deactivate the controller: command zero velocity on all joints to ensure a safe stop.
controller_interface::CallbackReturn CartesianVelocityController::on_deactivate(
    const rclcpp_lifecycle::State&) {
    // Command zero velocity on the way out, defensively.
    for (auto& ci : command_interfaces_) {
        ci.set_value(0.0);
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

// Callback for receiving the target pose from the subscribed topic. Updates the target pose buffer in a thread-safe manner.
void CartesianVelocityController::targetPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    target_pose_buffer_.writeFromNonRT(*msg);
    target_received_.store(true);
}

// Update the controller: read the current joint states, compute the pose error, solve for desired joint velocities, and command them to the actuators.
controller_interface::return_type CartesianVelocityController::update(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
    if (!target_received_.load()) {
        for (auto& ci : command_interfaces_) ci.set_value(0.0);
        return controller_interface::return_type::OK;
    }

    core::RobotModel::JointVector q, dq;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        q(static_cast<int>(i)) = state_interfaces_[2 * i].get_value();
        dq(static_cast<int>(i)) = state_interfaces_[2 * i + 1].get_value();
    }
    robot_model_->update(q, dq);

    const auto& raw_target = *target_pose_buffer_.readFromRT();
    Eigen::Vector3d raw_pos(raw_target.pose.position.x, raw_target.pose.position.y,
                             raw_target.pose.position.z);
    Eigen::Quaterniond raw_quat(raw_target.pose.orientation.w, raw_target.pose.orientation.x,
                                 raw_target.pose.orientation.y, raw_target.pose.orientation.z);
    raw_quat.normalize();

    if (!alignment_captured_) {
        // Rigid offset: robot's actual starting pose minus the DMP's first
        // published pose. Applied to every subsequent target, this
        // re-anchors the demonstrated relative motion to wherever the
        // robot actually starts, instead of the haptic device's frame.
        position_offset_ = activation_ee_position_ - raw_pos;
        orientation_offset_ = activation_ee_orientation_ * raw_quat.conjugate();
        alignment_captured_ = true;
        RCLCPP_INFO(get_node()->get_logger(),
                    "Captured DMP->robot alignment: position offset = [%.3f, %.3f, %.3f] m",
                    position_offset_.x(), position_offset_.y(), position_offset_.z());
    }

    Eigen::Vector3d target_pos = position_offset_ + raw_pos;
    Eigen::Quaterniond target_quat = (orientation_offset_ * raw_quat).normalized();

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
    
    core::CartesianError err = core::computePoseError(
        robot_model_->eePosition(), robot_model_->eeOrientation(), target_pos, target_quat);

    auto twist = ik_solver_.desiredTwist(err);
    auto dq_cmd = ik_solver_.solve(robot_model_->jacobian(), twist);

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        command_interfaces_[i].set_value(dq_cmd(static_cast<int>(i)));
    }

    return controller_interface::return_type::OK;
}

}  // namespace ros_wrapper
}  // namespace velocity_cartesian_control

// Register the controller as a plugin with the ROS 2 pluginlib system, allowing it to be dynamically loaded by the controller manager.
PLUGINLIB_EXPORT_CLASS(velocity_cartesian_control::ros_wrapper::CartesianVelocityController,
                        controller_interface::ControllerInterface)