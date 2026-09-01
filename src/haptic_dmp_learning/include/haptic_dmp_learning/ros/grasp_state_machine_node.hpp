#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include "haptic_dmp_learning/core/grasp_state_machine.hpp"

namespace haptic_dmp_learning {
namespace ros_wrapper {

/**
 * @brief ROS 2 wrapper around core::GraspStateMachine.
 *
 * Owns the four subscriptions and the ~/grasp_state publisher, converts the
 * incoming messages into core::GraspStateMachine::Inputs, drives the machine
 * one cycle per geometric-signal message, and performs the
 * ~/reset_limit_action rising-edge detection (first message seeds prior state
 * only, so a latched / transient-local `true` cannot by itself clear a safety
 * state) before calling core::GraspStateMachine::requestReset().
 *
 * All transition logic and the latching behaviour of `limit_action` live in
 * core::GraspStateMachine - see that header for the state/transition table.
 *
 * Pure observer: subscribes only to grasp_monitoring / grasp_force_calibration /
 * franka_cartesian_control topics, publishes only its own ~/grasp_state.
 */
class GraspStateMachineNode : public rclcpp::Node {
public:
    GraspStateMachineNode();

private:
    void geometricCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void forceVerifiedCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void forceCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
    void resetLimitActionCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void publishState() const;

    // ROS interfaces
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr geom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr verified_sub_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

    // Parameters
    std::string geometric_confirmed_topic_;
    std::string force_verified_topic_;
    std::string force_estimate_topic_;

    // Core state machine + the two inputs it reads each cycle, updated
    // asynchronously by their own callbacks.
    std::unique_ptr<core::GraspStateMachine> fsm_;
    bool force_verified_ = false;
    double f_norm_ = 0.0;

    // ~/reset_limit_action is edge-triggered (0->1): the first message only
    // seeds prior state.
    bool have_prev_reset_ = false;
    bool prev_reset_ = false;
};

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning
