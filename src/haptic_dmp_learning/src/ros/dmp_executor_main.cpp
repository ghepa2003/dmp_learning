#include <rclcpp/rclcpp.hpp>
#include "haptic_dmp_learning/ros/dmp_gazebo_executor_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<haptic_dmp_learning::ros_wrapper::DmpGazeboExecutorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}