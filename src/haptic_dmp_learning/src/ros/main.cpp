#include <rclcpp/rclcpp.hpp>
#include "haptic_dmp_learning/ros/haptic_dmp_wrapper_node.hpp"

/**
 * @brief Entry point for the Geomagic Touch haptic demonstration recorder and DMP training node.
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<haptic_dmp_learning::ros_wrapper::HapticDmpWrapperNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("haptic_dmp_wrapper_node"), "Fatal initialization error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
