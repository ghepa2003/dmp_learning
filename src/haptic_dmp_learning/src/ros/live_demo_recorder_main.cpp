#include <rclcpp/rclcpp.hpp>
#include "haptic_dmp_learning/ros/live_demo_recorder_node.hpp"

/**
 * @brief Entry point for the live demo recorder and DMP training node with Gazebo real-time visual mirroring.
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<haptic_dmp_learning::ros_wrapper::LiveDemoRecorderNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("live_demo_recorder_node"), "Fatal initialization error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
