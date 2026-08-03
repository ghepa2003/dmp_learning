#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "haptic_dmp_learning/core/demonstration_recorder.hpp"
#include "haptic_dmp_learning/core/dmp.hpp"
#include "haptic_dmp_learning/core/quaternion_dmp.hpp" 

namespace haptic_dmp_learning {
namespace ros_wrapper {

// Read /touch0/pose and /touch0/buttons, translate them into
// core::Sample objects, and drive a core::DemonstrationRecorder + core::DMP.
class HapticDmpWrapperNode : public rclcpp::Node {
public:
    HapticDmpWrapperNode();

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void buttonsCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    void startRecording();
    void stopRecordingAndLearn();
    void saveDemoToCsv(const std::string& path) const;

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr buttons_sub_;

    // core objects
    core::DemonstrationRecorder recorder_;
    core::DMP dmp_;
    core::QuaternionDMP quat_dmp_;

    // state
    bool recording_;
    rclcpp::Time record_start_time_;
    std::vector<int32_t> prev_buttons_;  // for rising-edge detection, empty until first msg

    // params
    std::string output_yaml_path_;
    std::string output_demo_csv_path_; 
    int n_basis_;
    double alpha_x_, alpha_z_, beta_z_;
    bool second_order_canonical_;
    std::string feature_flags_path_;
};

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning
