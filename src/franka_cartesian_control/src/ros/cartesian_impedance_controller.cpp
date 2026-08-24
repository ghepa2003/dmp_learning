#include "franka_cartesian_control/ros/cartesian_impedance_controller.hpp"
#include "franka_cartesian_control/core/cartesian_error.hpp"

#include <sstream>

#include <pluginlib/class_list_macros.hpp>

namespace franka_cartesian_control {
namespace ros_wrapper {

controller_interface::CallbackReturn CartesianImpedanceController::on_init() {
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

        // Impedance gains as parameters, so they can be tuned without
        // recompiling - same SERL-sourced defaults as core::CartesianImpedanceSolver::Params.
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
            "  translational_stiffness       = %.3f\n"
            "  rotational_stiffness          = %.3f\n"
            "  translational_damping         = %.3f\n"
            "  rotational_damping            = %.3f\n"
            "  nullspace_stiffness           = %.3f\n"
            "  joint1_nullspace_stiffness    = %.3f\n"
            "  delta_tau_max                 = %.3f\n"
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

controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& jn : joint_names_) {
        config.names.push_back(jn + "/effort");
    }
    return config;
}

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

controller_interface::CallbackReturn CartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State&) {
    auto node = get_node();

    // robot_description via topic (transient_local), same mechanism as
    // CartesianVelocityController - see that class for full rationale.
    std::string urdf_xml;
    {
        auto temp_node = std::make_shared<rclcpp::Node>("franka_cartesian_control_urdf_waiter_impedance");
        std::promise<std::string> urdf_promise;
        auto urdf_future = urdf_promise.get_future();

        auto sub = temp_node->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(1).transient_local(),
            [&urdf_promise](const std_msgs::msg::String::SharedPtr msg) {
                urdf_promise.set_value(msg->data);
            });

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(temp_node);

        const auto timeout = std::chrono::seconds(5);
        auto status = executor.spin_until_future_complete(urdf_future, timeout);

        if (status != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "Timed out waiting for /robot_description (5s)");
            return controller_interface::CallbackReturn::ERROR;
        }
        urdf_xml = urdf_future.get();
    }

    try {
        robot_model_ = std::make_unique<core::RobotModel>(urdf_xml, joint_names_, ee_frame_name_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to build RobotModel: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    target_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        target_pose_topic_, rclcpp::QoS(10),
        std::bind(&CartesianImpedanceController::targetPoseCallback, this, std::placeholders::_1));

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

controller_interface::CallbackReturn CartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State&) {
    core::RobotModel::JointVector q, dq;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        q(static_cast<int>(i)) = state_interfaces_[2 * i].get_value();
        dq(static_cast<int>(i)) = state_interfaces_[2 * i + 1].get_value();
    }
    robot_model_->update(q, dq);

    activation_ee_position_ = robot_model_->eePosition();
    activation_ee_orientation_ = robot_model_->eeOrientation();

    // Nullspace target = configuration at activation (SERL: "set nullspace
    // equilibrium configuration to initial q" in starting()).
    impedance_solver_.setNullspaceTarget(q);

    // Start from zero commanded torque; the first update() cycle's rate
    // saturation will ramp up smoothly from here (delta_tau_max per cycle).
    tau_prev_ = core::RobotModel::JointVector::Zero();
    for (auto& ci : command_interfaces_) {
        ci.set_value(0.0);
    }

    alignment_captured_ = false;
    target_received_.store(false);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State&) {
    for (auto& ci : command_interfaces_) {
        ci.set_value(0.0);
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

void CartesianImpedanceController::targetPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    target_pose_buffer_.writeFromNonRT(*msg);
    target_received_.store(true);
}

controller_interface::return_type CartesianImpedanceController::update(
    const rclcpp::Time& time, const rclcpp::Duration&) {
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
    Eigen::Vector3d raw_pos(raw_target.pose.position.x, raw_target.pose.position.y, raw_target.pose.position.z);
    Eigen::Quaterniond raw_quat(raw_target.pose.orientation.w, raw_target.pose.orientation.x,
                                 raw_target.pose.orientation.y, raw_target.pose.orientation.z);
    raw_quat.normalize();

    if (!alignment_captured_) {
        position_offset_ = activation_ee_position_ - raw_pos;
        orientation_offset_ = activation_ee_orientation_ * raw_quat.conjugate();
        alignment_captured_ = true;
        RCLCPP_INFO(get_node()->get_logger(),
                    "Captured DMP->robot alignment: position offset = [%.3f, %.3f, %.3f] m",
                    position_offset_.x(), position_offset_.y(), position_offset_.z());
    }

    Eigen::Vector3d target_pos = position_offset_ + raw_pos;
    Eigen::Quaterniond target_quat = (orientation_offset_ * raw_quat).normalized();

    core::CartesianError err = core::computePoseError(
        robot_model_->eePosition(), robot_model_->eeOrientation(), target_pos, target_quat);

    core::RobotModel::JointVector tau = impedance_solver_.computeTorque(
        *robot_model_, err, q, dq, tau_prev_);

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        command_interfaces_[i].set_value(tau(static_cast<int>(i)));
    }
    tau_prev_ = tau;

    // debug publish (same pattern as CartesianVelocityController)
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

PLUGINLIB_EXPORT_CLASS(franka_cartesian_control::ros_wrapper::CartesianImpedanceController,
                        controller_interface::ControllerInterface)