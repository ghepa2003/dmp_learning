#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace haptic_dmp_learning {
namespace ros_wrapper {

/**
 * @brief Fuses the geometric grasp signal and the per-demo force verification
 *        into a four-state grasp state machine.
 *
 * States published on ~/grasp_state (std_msgs/String):
 *   "free_space" | "contact_pending" | "contact_confirmed" | "limit_action"
 *
 * Transitions (driven by the geometric-signal callback, ~15 Hz):
 *   free_space        -> contact_pending   : geometric true for 1 cycle
 *   contact_pending   -> contact_confirmed : geometric true for `persistence_cycles`
 *                                            consecutive cycles AND grasp_force_verified true
 *   contact_pending / contact_confirmed -> free_space : geometric signal lost
 *   contact_confirmed -> limit_action      : |F| > hard_force_limit_n
 *   limit_action      -> free_space        : ONLY on an explicit reset
 *                                            (std_msgs/Bool true on ~/reset_limit_action);
 *                                            never automatically on signal loss.
 *
 * `hard_force_limit_n` is an ABSOLUTE safety maximum, separate from the
 * calibration tolerance; it is mandatory (no silent default), consistent with
 * the project's fail-loud convention. `limit_action` is latched (see
 * DESIGN_NOTES) so that "object repelled by excessive contact" (the failure this
 * component exists to catch) is not silently conflated with "safety action
 * handled" - both would otherwise look the same to a ~/grasp_state consumer.
 *
 * Pure observer: subscribes only to grasp_monitoring / grasp_force_calibration /
 * franka_cartesian_control topics, publishes only its own ~/grasp_state.
 */
class GraspStateMachineNode : public rclcpp::Node {
public:
    GraspStateMachineNode();

private:
    enum class State { kFreeSpace, kContactPending, kContactConfirmed, kLimitAction };

    void geometricCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void forceVerifiedCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void forceCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
    void resetLimitActionCallback(const std_msgs::msg::Bool::SharedPtr msg);

    static const char* stateName(State s);
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
    int persistence_cycles_;
    double hard_force_limit_n_;

    // State
    State state_ = State::kFreeSpace;
    int geom_true_streak_ = 0;
    bool force_verified_ = false;
    double f_norm_ = 0.0;

    // ~/reset_limit_action is edge-triggered (0->1), same pattern as the
    // /touch0/buttons handling in haptic_dmp_wrapper_node: the first message only
    // seeds prior state, so a QoS transient-local / re-published `true` cannot by
    // itself clear a safety state.
    bool have_prev_reset_ = false;
    bool prev_reset_ = false;
};

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning
