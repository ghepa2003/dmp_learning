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
 * @brief ROS 2 Node for Live Interactive Demonstration Recording, Real-Time Gazebo Mirroring, and DMP Fitting.
 *
 * @details
 * Dual-Role Architecture:
 * 1. Teleoperation Mirroring: Listens to incoming master pose stream on `/master_pose_raw` (from hardware driver or CSV player).
 *    While recording is active, immediately forwards each pose onto `/target_pose` so that the robot in Gazebo mirrors the operator's
 *    motion live through the active Cartesian controller.
 * 2. Online DMP Fitting: Monitors Joy button states on `/touch0/buttons`. On the rising edge of button 1 (stop), fits translational
 *    and rotational DMPs and exports the learned weights to YAML.
 */
class LiveDemoRecorderNode : public rclcpp::Node {
public:
    LiveDemoRecorderNode();

private:
    void masterPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void buttonsCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    void startRecording();
    void stopRecordingAndLearn();

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr master_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr buttons_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;

    // Core mathematical objects
    core::DemonstrationRecorder recorder_;
    core::DMP dmp_;
    core::QuaternionDMP quat_dmp_;

    // State machine
    bool recording_;
    rclcpp::Time record_start_time_;
    std::vector<int32_t> prev_buttons_;  ///< Previous button state for rising-edge detection
    bool gripper_trigger_pending_ = false;

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

    // Parameters
    std::string master_pose_topic_;
    std::string target_pose_topic_;
    std::string buttons_topic_;
    std::string output_yaml_path_;
    std::string output_demo_csv_path_;
    int n_basis_;
    double alpha_x_, alpha_z_, beta_z_;
    std::string feature_flags_path_;
};

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning
