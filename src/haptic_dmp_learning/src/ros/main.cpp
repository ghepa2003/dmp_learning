#include <rclcpp/rclcpp.hpp>
#include "haptic_dmp_learning/ros/haptic_dmp_wrapper_node.hpp"

int main(int argc, char** argv) {
    // Initialize ROS2 and create the HapticDmpWrapperNode
    rclcpp::init(argc, argv);
    // Create a shared pointer to the HapticDmpWrapperNode and spin it to process callbacks
    auto node = std::make_shared<haptic_dmp_learning::ros_wrapper::HapticDmpWrapperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
