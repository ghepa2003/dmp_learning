#pragma once

#include <cstddef>
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
 * 1. Teleoperation Tracking: Subscribes to the master pose topic (ROS parameter `master_pose_topic`,
 *    default `/master_pose_raw`) fed by the Geomagic Touch driver or the CSV stand-in (SensorData QoS, ~1 kHz).
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

    // Quaternion double-cover continuity. The Geomagic Touch driver sometimes
    // reports a physically identical rotation as -q, a representation sign flip
    // that becomes a ~pi jump in QuaternionDMP's log-map increments. A negative
    // dot product between consecutive raw orientations is such a transition:
    // toggle a running sign and apply it to every later sample so the recorded
    // track stays continuous. Compared raw-to-raw so the count is the number of
    // transitions, not the number of negated samples.
    bool has_last_orientation_ = false;
    Eigen::Quaterniond last_corrected_orientation_ = Eigen::Quaterniond::Identity();
    bool quat_negate_parity_ = false;
    std::size_t quat_sign_flips_corrected_ = 0;  ///< transitions this recording

    // ROS Parameters
    std::string pose_topic_;  ///< master pose input topic (declared, not remap-dependent)
    std::string output_yaml_path_;
    std::string output_demo_csv_path_;
    int n_basis_;
    double alpha_x_, alpha_z_, beta_z_;
    std::string feature_flags_path_;
};

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning
