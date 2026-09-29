#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "satellite_grasp_planner/ros/grasp_planner_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<satellite_grasp_planner::ros_wrapper::GraspPlannerNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("grasp_planner_node"), "startup failed: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
