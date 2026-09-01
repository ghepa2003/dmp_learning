#include <rclcpp/rclcpp.hpp>
#include "haptic_dmp_learning/ros/grasp_state_machine_node.hpp"

/**
 * @brief Entry point for the grasp state machine node.
 *
 * Fail-loud: a missing mandatory parameter (hard_force_limit_n) throws out of the
 * constructor; log via RCLCPP_FATAL and exit non-zero instead of spinning.
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<haptic_dmp_learning::ros_wrapper::GraspStateMachineNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("grasp_state_machine"),
                     "Fatal initialization error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
