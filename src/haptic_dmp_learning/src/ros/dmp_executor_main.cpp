#include <rclcpp/rclcpp.hpp>
#include "haptic_dmp_learning/ros/dmp_gazebo_executor_node.hpp"

/**
 * @brief Entry point for the DMP Gazebo executor node, which replays trained DMPs to Cartesian controllers.
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<haptic_dmp_learning::ros_wrapper::DmpGazeboExecutorNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("dmp_gazebo_executor_node"), "Fatal initialization error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}