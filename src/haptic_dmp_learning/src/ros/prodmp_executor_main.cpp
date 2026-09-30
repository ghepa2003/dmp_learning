#include <rclcpp/rclcpp.hpp>
#include "haptic_dmp_learning/ros/prodmp_gazebo_executor_node.hpp"

/**
 * @brief Entry point for the ProDMP Gazebo executor node - twin of
 *        dmp_executor_main.cpp, replaying an integral-form ProDMP (position) plus
 *        a QuaternionDMP (orientation) as Cartesian target poses.
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<haptic_dmp_learning::ros_wrapper::ProDmpGazeboExecutorNode> node;
    try {
        node = std::make_shared<haptic_dmp_learning::ros_wrapper::ProDmpGazeboExecutorNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        // Same sentence as the node's own FATALs: if the hold target was the controller's last target, it stays
        // the regulation reference after this process exits (no watchdog in the controller).
        const std::string msg = haptic_dmp_learning::ros_wrapper::hold_target::withHoldNote(
            e.what(), node && node->holdRemainsInController());
        RCLCPP_FATAL(rclcpp::get_logger("prodmp_gazebo_executor_node"),
                     "Fatal initialization error: %s", msg.c_str());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
