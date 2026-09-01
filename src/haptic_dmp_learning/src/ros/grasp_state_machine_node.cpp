#include "haptic_dmp_learning/ros/grasp_state_machine_node.hpp"

#include <cmath>
#include <stdexcept>

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
    persistence_cycles_ = this->declare_parameter<int>("persistence_cycles", 5);
    if (persistence_cycles_ < 1) {
        throw std::runtime_error(
            "grasp_state_machine: persistence_cycles must be >= 1.");
    }

    // hard_force_limit_n is an absolute safety maximum, deliberately without a
    // default: a wrong or forgotten value must fail loudly, not silently gate.
    try {
        hard_force_limit_n_ = this->declare_parameter<double>("hard_force_limit_n");
    } catch (const std::exception& e) {
        throw std::runtime_error(
            "grasp_state_machine: required parameter 'hard_force_limit_n' (absolute "
            "|F| safety limit, N) was not provided. It is intentionally separate "
            "from the calibration tolerance and has no default. Underlying error: " +
            std::string(e.what()));
    }
    if (!(hard_force_limit_n_ > 0.0)) {
        throw std::runtime_error(
            "grasp_state_machine: hard_force_limit_n must be > 0.");
    }

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
                force_estimate_topic_.c_str(), persistence_cycles_, hard_force_limit_n_);

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

    if (state_ != State::kLimitAction) {
        RCLCPP_INFO(this->get_logger(),
                    "reset_limit_action rising edge but state is '%s' (not limit_action); ignored.",
                    stateName(state_));
        return;
    }
    RCLCPP_WARN(this->get_logger(),
                "limit_action cleared by explicit reset edge -> free_space (|F|=%.3f N).", f_norm_);
    state_ = State::kFreeSpace;
    geom_true_streak_ = 0;
    publishState();
}

void GraspStateMachineNode::geometricCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    const bool geom = msg->data;
    geom_true_streak_ = geom ? (geom_true_streak_ + 1) : 0;

    const State prev = state_;

    switch (state_) {
        case State::kFreeSpace:
            if (geom) {
                state_ = State::kContactPending;  // geometric true for 1 cycle
            }
            break;

        case State::kContactPending:
            if (!geom) {
                state_ = State::kFreeSpace;  // signal lost
            } else if (geom_true_streak_ >= persistence_cycles_ && force_verified_) {
                state_ = State::kContactConfirmed;
            }
            break;

        case State::kContactConfirmed:
            if (!geom) {
                state_ = State::kFreeSpace;  // signal lost
            } else if (f_norm_ > hard_force_limit_n_) {
                state_ = State::kLimitAction;  // absolute safety limit exceeded
            }
            break;

        case State::kLimitAction:
            // Latched. No automatic exit - not on signal loss, not on |F| dropping
            // back below the limit. Only resetLimitActionCallback() releases it,
            // so a downstream consumer can tell "repelled / still unsafe" apart
            // from "safety action handled". State keeps being published below.
            break;
    }

    if (state_ != prev) {
        RCLCPP_INFO(this->get_logger(), "grasp state: %s -> %s "
                    "(geom=%d streak=%d verified=%d |F|=%.3f N)",
                    stateName(prev), stateName(state_), geom ? 1 : 0,
                    geom_true_streak_, force_verified_ ? 1 : 0, f_norm_);
    }
    publishState();
}

const char* GraspStateMachineNode::stateName(State s) {
    switch (s) {
        case State::kFreeSpace:        return "free_space";
        case State::kContactPending:   return "contact_pending";
        case State::kContactConfirmed: return "contact_confirmed";
        case State::kLimitAction:      return "limit_action";
    }
    return "free_space";
}

void GraspStateMachineNode::publishState() const {
    std_msgs::msg::String out;
    out.data = stateName(state_);
    state_pub_->publish(out);
}

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning
