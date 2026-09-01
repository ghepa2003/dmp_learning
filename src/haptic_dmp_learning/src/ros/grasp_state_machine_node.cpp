#include "haptic_dmp_learning/ros/grasp_state_machine_node.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace haptic_dmp_learning {
namespace ros_wrapper {

GraspStateMachineNode::GraspStateMachineNode()
    : Node("grasp_state_machine") {

    geometric_confirmed_topic_ = this->declare_parameter<std::string>(
        "geometric_confirmed_topic", "/geometric_grasp_monitor/geometric_grasp_confirmed");
    force_verified_topic_ = this->declare_parameter<std::string>(
        "force_verified_topic", "/grasp_force_calibration_node/grasp_force_verified");
    force_estimate_topic_ = this->declare_parameter<std::string>(
        "force_estimate_topic", "/cartesian_impedance_controller/contact_wrench_estimate");

    core::GraspStateMachine::Params fsm_params;
    fsm_params.persistence_cycles = this->declare_parameter<int>("persistence_cycles", 5);
    if (fsm_params.persistence_cycles < 1) {
        throw std::runtime_error(
            "grasp_state_machine: persistence_cycles must be >= 1.");
    }

    // hard_force_limit_n is an absolute safety maximum, deliberately without a
    // default: a wrong or forgotten value must fail loudly, not silently gate.
    try {
        fsm_params.hard_force_limit_n = this->declare_parameter<double>("hard_force_limit_n");
    } catch (const std::exception& e) {
        throw std::runtime_error(
            "grasp_state_machine: required parameter 'hard_force_limit_n' (absolute "
            "|F| safety limit, N) was not provided. It is intentionally separate "
            "from the calibration tolerance and has no default. Underlying error: " +
            std::string(e.what()));
    }
    if (!(fsm_params.hard_force_limit_n > 0.0)) {
        throw std::runtime_error(
            "grasp_state_machine: hard_force_limit_n must be > 0.");
    }

    fsm_ = std::make_unique<core::GraspStateMachine>(fsm_params);

    state_pub_ = this->create_publisher<std_msgs::msg::String>(
        "~/grasp_state", rclcpp::QoS(10));

    geom_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        geometric_confirmed_topic_, rclcpp::QoS(10),
        std::bind(&GraspStateMachineNode::geometricCallback, this, std::placeholders::_1));
    verified_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        force_verified_topic_, rclcpp::QoS(10),
        std::bind(&GraspStateMachineNode::forceVerifiedCallback, this, std::placeholders::_1));
    force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        force_estimate_topic_, rclcpp::SensorDataQoS(),
        std::bind(&GraspStateMachineNode::forceCallback, this, std::placeholders::_1));
    // Explicit reset for limit_action (topic, not service: this package is
    // entirely topic-based). limit_action never clears on its own.
    reset_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "~/reset_limit_action", rclcpp::QoS(10),
        std::bind(&GraspStateMachineNode::resetLimitActionCallback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "grasp_state_machine ready | geometric=%s | verified=%s | force=%s | "
                "persistence_cycles=%d | hard_force_limit_n=%.3f N",
                geometric_confirmed_topic_.c_str(), force_verified_topic_.c_str(),
                force_estimate_topic_.c_str(), fsm_->params().persistence_cycles,
                fsm_->params().hard_force_limit_n);

    publishState();
}

void GraspStateMachineNode::forceVerifiedCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    force_verified_ = msg->data;
}

void GraspStateMachineNode::forceCallback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
    const double fx = msg->wrench.force.x;
    const double fy = msg->wrench.force.y;
    const double fz = msg->wrench.force.z;
    f_norm_ = std::sqrt(fx * fx + fy * fy + fz * fz);
}

void GraspStateMachineNode::resetLimitActionCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    const bool val = msg->data;

    // First message only seeds prior state - a latched/transient-local `true`
    // arriving as the first sample must not clear a safety state on its own.
    if (!have_prev_reset_) {
        have_prev_reset_ = true;
        prev_reset_ = val;
        return;
    }
    const bool rising = val && !prev_reset_;
    prev_reset_ = val;
    if (!rising) return;

    const auto state_before = fsm_->state();
    if (!fsm_->requestReset()) {
        RCLCPP_INFO(this->get_logger(),
                    "reset_limit_action rising edge but state is '%s' (not limit_action); ignored.",
                    core::GraspStateMachine::stateName(state_before).c_str());
        return;
    }
    RCLCPP_WARN(this->get_logger(),
                "limit_action cleared by explicit reset edge -> free_space (|F|=%.3f N).", f_norm_);
    publishState();
}

void GraspStateMachineNode::geometricCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    const bool geom = msg->data;

    core::GraspStateMachine::Inputs in;
    in.geometric_confirmed = geom;
    in.force_verified = force_verified_;
    in.f_norm = f_norm_;

    const auto prev = fsm_->state();
    const auto now = fsm_->step(in);

    if (now != prev) {
        RCLCPP_INFO(this->get_logger(), "grasp state: %s -> %s "
                    "(geom=%d streak=%d verified=%d |F|=%.3f N)",
                    core::GraspStateMachine::stateName(prev).c_str(),
                    core::GraspStateMachine::stateName(now).c_str(), geom ? 1 : 0,
                    fsm_->geometricStreak(), force_verified_ ? 1 : 0, f_norm_);
    }
    publishState();
}

void GraspStateMachineNode::publishState() const {
    std_msgs::msg::String out;
    out.data = core::GraspStateMachine::stateName(fsm_->state());
    state_pub_->publish(out);
}

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning
