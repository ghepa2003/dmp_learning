#include "franka_cartesian_control/ros/cartesian_impedance_controller.hpp"
#include "franka_cartesian_control/core/cartesian_error.hpp"

#include <sstream>
#include <pluginlib/class_list_macros.hpp>

namespace franka_cartesian_control {
namespace ros_wrapper {

/**
 * @brief Initializes parameters for Cartesian impedance control.
 * Declares stiffness (K), damping (D), nullspace gains, and safety limits as tunable ROS 2 parameters.
 */
controller_interface::CallbackReturn CartesianImpedanceController::on_init() {
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

        // 2. Declare end-effector link frame name
        if (!node->has_parameter("ee_frame_name")) {
            node->declare_parameter<std::string>("ee_frame_name", "fer_link8");
        }
        ee_frame_name_ = node->get_parameter("ee_frame_name").as_string();

        // 3. Declare target pose topic name
        if (!node->has_parameter("target_pose_topic")) {
            node->declare_parameter<std::string>("target_pose_topic", "/target_pose");
        }
        target_pose_topic_ = node->get_parameter("target_pose_topic").as_string();

        // 4. Declare Compliance and Nullspace Parameters (SERL-sourced defaults)
        core::CartesianImpedanceSolver::Params params;
        auto declare_and_get = [&node](const std::string& name, double def) {
            if (!node->has_parameter(name)) node->declare_parameter<double>(name, def);
            return node->get_parameter(name).as_double();
        };
        params.translational_stiffness = declare_and_get("translational_stiffness", params.translational_stiffness);
        params.rotational_stiffness = declare_and_get("rotational_stiffness", params.rotational_stiffness);
        params.translational_damping = declare_and_get("translational_damping", params.translational_damping);
        params.rotational_damping = declare_and_get("rotational_damping", params.rotational_damping);
        params.nullspace_stiffness = declare_and_get("nullspace_stiffness", params.nullspace_stiffness);
        params.joint1_nullspace_stiffness = declare_and_get("joint1_nullspace_stiffness", params.joint1_nullspace_stiffness);
        params.delta_tau_max = declare_and_get("delta_tau_max", params.delta_tau_max);
        params.nullspace_pinv_damping = declare_and_get("nullspace_pinv_damping", params.nullspace_pinv_damping);

        if (!node->has_parameter("compensate_gravity_internally")) {
            node->declare_parameter<bool>("compensate_gravity_internally", params.compensate_gravity_internally);
        }
        params.compensate_gravity_internally = node->get_parameter("compensate_gravity_internally").as_bool();

        if (!node->has_parameter("enable_nullspace_leak_diagnostics")) {
            node->declare_parameter<bool>("enable_nullspace_leak_diagnostics", false);
        }
        enable_nullspace_leak_diagnostics_ =
            node->get_parameter("enable_nullspace_leak_diagnostics").as_bool();

        if (!node->has_parameter("enable_contact_force_estimation")) {
            node->declare_parameter<bool>("enable_contact_force_estimation", false);
        }
        enable_contact_force_estimation_ =
            node->get_parameter("enable_contact_force_estimation").as_bool();

        impedance_solver_ = core::CartesianImpedanceSolver(params);

        std::ostringstream joint_names_str;
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            if (i > 0) joint_names_str << ", ";
            joint_names_str << joint_names_[i];
        }

        RCLCPP_INFO(node->get_logger(),
            "CartesianImpedanceController parameters loaded:\n"
            "  joint_names                   = [%s]\n"
            "  ee_frame_name                 = %s\n"
            "  target_pose_topic             = %s\n"
            "  translational_stiffness       = %.3f N/m\n"
            "  rotational_stiffness          = %.3f Nm/rad\n"
            "  translational_damping         = %.3f Ns/m\n"
            "  rotational_damping            = %.3f Nms/rad\n"
            "  nullspace_stiffness           = %.3f\n"
            "  joint1_nullspace_stiffness    = %.3f\n"
            "  delta_tau_max                 = %.3f Nm/cycle\n"
            "  nullspace_pinv_damping        = %.3f\n"
            "  compensate_gravity_internally = %s",
            joint_names_str.str().c_str(),
            ee_frame_name_.c_str(),
            target_pose_topic_.c_str(),
            params.translational_stiffness,
            params.rotational_stiffness,
            params.translational_damping,
            params.rotational_damping,
            params.nullspace_stiffness,
            params.joint1_nullspace_stiffness,
            params.delta_tau_max,
            params.nullspace_pinv_damping,
            params.compensate_gravity_internally ? "true" : "false");

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Configures effort command interfaces for torque control mode.
 * Impedance controller claims '<joint_name>/effort' interfaces for all 7 joints.
 */
controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& jn : joint_names_) {
        config.names.push_back(jn + "/effort");
    }
    return config;
}

/**
 * @brief Configures feedback state interfaces read by this controller (positions and velocities).
 */
controller_interface::InterfaceConfiguration
CartesianImpedanceController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& jn : joint_names_) {
        config.names.push_back(jn + "/position");
        config.names.push_back(jn + "/velocity");
    }
    return config;
}

/**
 * @brief Lifecycle configure transition: fetches URDF, builds Pinocchio model, creates subscribers/publishers.
 */
controller_interface::CallbackReturn CartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State&) {
    auto node = get_node();

    // 1. Fetch URDF XML model string from /robot_description (transient_local QoS)
    auto urdf_opt = fetchRobotDescription(node->get_logger());
    if (!urdf_opt) {
        return controller_interface::CallbackReturn::ERROR;
    }
    std::string urdf_xml = *urdf_opt;

    // 2. Build Pinocchio core RobotModel
    try {
        robot_model_ = std::make_unique<core::RobotModel>(urdf_xml, joint_names_, ee_frame_name_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to build RobotModel: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    // 3. Subscribe to target pose topic
    target_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        target_pose_topic_, rclcpp::QoS(10),
        std::bind(&CartesianImpedanceController::targetPoseCallback, this, std::placeholders::_1));

    // 4. Pre-allocate real-time safe publishers for telemetry
    aligned_target_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/target_pose_aligned", rclcpp::QoS(10));
    rt_aligned_target_pub_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
        aligned_target_pub_);

    actual_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/actual_pose", rclcpp::QoS(10));
    rt_actual_pose_pub_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
        actual_pose_pub_);

    if (enable_nullspace_leak_diagnostics_) {
        nullspace_leak_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
            "~/nullspace_leak", rclcpp::QoS(10));
        rt_nullspace_leak_pub_ =
            std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
                nullspace_leak_pub_);
    }

    if (enable_contact_force_estimation_) {
        contact_wrench_pub_ = node->create_publisher<geometry_msgs::msg::WrenchStamped>(
            "~/contact_wrench_estimate", rclcpp::QoS(10));
        rt_contact_wrench_pub_ =
            std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(
                contact_wrench_pub_);
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Lifecycle activate transition: captures initial end-effector pose, sets nullspace target configuration.
 */
controller_interface::CallbackReturn CartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State&) {
    // 1. Read initial joint positions and velocities
    core::RobotModel::JointVector q, dq;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        q(static_cast<int>(i)) = state_interfaces_[2 * i].get_value();
        dq(static_cast<int>(i)) = state_interfaces_[2 * i + 1].get_value();
    }
    robot_model_->update(q, dq);

    // 2. Anchor frame alignment to physical activation pose
    frame_aligner_.reset(robot_model_->eePosition(), robot_model_->eeOrientation());

    // 3. Set nullspace posture target to the current joint configuration q at activation
    impedance_solver_.setNullspaceTarget(q);

    // 4. Initialize torque memory to zero for smooth slew-rate ramping
    tau_prev_ = core::RobotModel::JointVector::Zero();
    for (auto& ci : command_interfaces_) {
        ci.set_value(0.0);
    }

    target_received_.store(false);

    return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Lifecycle deactivate transition: defensively commands zero torque to all joints.
 */
controller_interface::CallbackReturn CartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State&) {
    for (auto& ci : command_interfaces_) {
        ci.set_value(0.0);
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Asynchronous subscription callback: receives incoming target pose and writes to lock-free RT buffer.
 */
void CartesianImpedanceController::targetPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    target_pose_buffer_.writeFromNonRT(*msg);
    target_received_.store(true);
}

/**
 * @brief Real-time deterministic control loop (1 kHz): computes Cartesian impedance + nullspace torque.
 */
controller_interface::return_type CartesianImpedanceController::update(
    const rclcpp::Time& time, const rclcpp::Duration&) {
    // 1. Hold zero torque if no target pose has been received yet
    if (!target_received_.load()) {
        for (auto& ci : command_interfaces_) ci.set_value(0.0);
        return controller_interface::return_type::OK;
    }

    // 2. Read current joint states and update Pinocchio kinematics, Jacobians, and Coriolis torques
    core::RobotModel::JointVector q, dq;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        q(static_cast<int>(i)) = state_interfaces_[2 * i].get_value();
        dq(static_cast<int>(i)) = state_interfaces_[2 * i + 1].get_value();
    }
    robot_model_->update(q, dq);

    // 3. Lock-free read of raw target pose from real-time buffer
    const auto& raw_target = *target_pose_buffer_.readFromRT();
    Eigen::Vector3d raw_pos(raw_target.pose.position.x, raw_target.pose.position.y, raw_target.pose.position.z);
    Eigen::Quaterniond raw_quat(raw_target.pose.orientation.w, raw_target.pose.orientation.x,
                                 raw_target.pose.orientation.y, raw_target.pose.orientation.z);
    raw_quat.normalize();

    // 4. Align raw target to robot base frame using rigid offset
    auto logger = get_node()->get_logger();
    Eigen::Vector3d target_pos;
    Eigen::Quaterniond target_quat;
    frame_aligner_.align(raw_pos, raw_quat, target_pos, target_quat, &logger);

    // 5. Compute 6D Cartesian Error (linear translation + Lie algebra orientation)
    core::CartesianError err = core::computePoseError(
        robot_model_->eePosition(), robot_model_->eeOrientation(), target_pos, target_quat);

    // 6. Compute total commanded torque: Task-Space + Nullspace + Coriolis [+ Gravity]
    core::RobotModel::JointVector tau = impedance_solver_.computeTorque(
        *robot_model_, err, q, dq, tau_prev_);

    // Diagnostic-only: quantify how much the (kinematically-projected) nullspace torque
    // leaks into task-space acceleration through the anisotropic mass matrix M(q).
    // leak = J * M(q)^-1 * tau_nullspace ; does NOT feed back into tau/command.
    if (enable_nullspace_leak_diagnostics_ && rt_nullspace_leak_pub_ && rt_nullspace_leak_pub_->trylock()) {
        const auto& M = robot_model_->massMatrix();
        const auto& tau_ns = impedance_solver_.lastNullspaceTorque();
        core::RobotModel::JointVector qdd_leak = M.ldlt().solve(tau_ns);
        Eigen::Matrix<double, 6, 1> leak = robot_model_->jacobian() * qdd_leak;

        auto& msg = rt_nullspace_leak_pub_->msg_;
        msg.data.resize(6);
        for (int i = 0; i < 6; ++i) msg.data[i] = leak(i);
        rt_nullspace_leak_pub_->unlockAndPublish();
    }

    // Diagnostic-only: sensorless contact wrench estimate from the quasi-static impedance law.
    // Agnostic to reference source (DMP replay or live haptic streaming) and to the sim/real split —
    // this topic is meant to be fed by the native Franka O_F_ext_hat_K estimate on real hardware
    // instead of this computation. See DESIGN_NOTES.md.
    if (enable_contact_force_estimation_ && rt_contact_wrench_pub_ && rt_contact_wrench_pub_->trylock()) {
        const auto& wrench_est = impedance_solver_.lastEstimatedContactWrench();
        auto& msg = rt_contact_wrench_pub_->msg_;
        msg.header.stamp = time;
        msg.header.frame_id = "fer_link0";
        msg.wrench.force.x = wrench_est(0);
        msg.wrench.force.y = wrench_est(1);
        msg.wrench.force.z = wrench_est(2);
        msg.wrench.torque.x = wrench_est(3);
        msg.wrench.torque.y = wrench_est(4);
        msg.wrench.torque.z = wrench_est(5);
        rt_contact_wrench_pub_->unlockAndPublish();
    }

    // 7. Command torques to hardware effort interfaces
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        command_interfaces_[i].set_value(tau(static_cast<int>(i)));
    }
    tau_prev_ = tau;

    // 8. Publish telemetry (lock-free non-blocking trylock)
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

    return controller_interface::return_type::OK;
}

}  // namespace ros_wrapper
}  // namespace franka_cartesian_control

// Register as dynamically loadable controller plugin
PLUGINLIB_EXPORT_CLASS(franka_cartesian_control::ros_wrapper::CartesianImpedanceController,
                        controller_interface::ControllerInterface)