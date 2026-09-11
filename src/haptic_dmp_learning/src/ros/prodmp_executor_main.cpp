#include <rclcpp/rclcpp.hpp>
#include "haptic_dmp_learning/ros/prodmp_gazebo_executor_node.hpp"

/**
 * @brief Entry point for the ProDMP Gazebo executor node - twin of
 *        dmp_executor_main.cpp, replaying an integral-form ProDMP (position) plus
 *        a QuaternionDMP (orientation) as Cartesian target poses.
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<haptic_dmp_learning::ros_wrapper::ProDmpGazeboExecutorNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("prodmp_gazebo_executor_node"),
                     "Fatal initialization error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
