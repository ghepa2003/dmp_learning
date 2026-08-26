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

/**
 * @brief ROS 2 Node for Teleoperation Demonstration Recording and DMP Learning.
 *
 * @details
 * Workflow & Operation:
 * 1. Teleoperation Tracking: Subscribes to `/touch0/pose` from the Geomagic Touch haptic stylus driver (SensorData QoS, ~1 kHz).
 * 2. State Machine Triggering: Subscribes to `/touch0/buttons` (Joy message) and listens for rising-edge transitions:
 *    - Button 0 (Front Grey Button): Starts recording a demonstration, resetting the sample buffer.
 *    - Button 1 (Rear White Button): Stops recording, saves raw CSV, fits 3D position and SO(3) quaternion DMPs,
 *      and serializes learned parameters to YAML.
 */
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

    // Core mathematical objects
    core::DemonstrationRecorder recorder_;
    core::DMP dmp_;
    core::QuaternionDMP quat_dmp_;

    // State machine tracking
    bool recording_;
    rclcpp::Time record_start_time_;
    std::vector<int32_t> prev_buttons_;  ///< Stores previous joy button state for rising-edge detection

    // ROS Parameters
    std::string output_yaml_path_;
    std::string output_demo_csv_path_; 
    int n_basis_;
    double alpha_x_, alpha_z_, beta_z_;
    std::string feature_flags_path_;
};

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning
