#include <rclcpp/rclcpp.hpp>
#include "haptic_dmp_learning/ros/csv_master_pose_player_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<haptic_dmp_learning::ros_wrapper::CsvMasterPosePlayerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
