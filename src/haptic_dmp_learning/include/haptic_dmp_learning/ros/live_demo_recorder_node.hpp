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

// The single "downstream" node of the live dry-run pipeline: listens to
// /master_pose_raw (today: csv_master_pose_player_node stand-in; tomorrow:
// the Geomagic Touch driver via a launch remap, unmodified), and - using the
// very same /touch0/buttons start/stop detection as haptic_dmp_wrapper_node -
// records the demonstration, republishes every pose onto /target_pose (so
// the demo is visible live in Gazebo through the existing
// CartesianVelocityController) only while recording is active, and fits the
// DMP (ridge regression + velocity filter, same core:: calls as the wrapper
// node) as soon as the demo ends.
class LiveDemoRecorderNode : public rclcpp::Node {
public:
    LiveDemoRecorderNode();

private:
    void masterPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void buttonsCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    void startRecording();
    void stopRecordingAndLearn();
    void saveDemoToCsv(const std::string& path) const;

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr master_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr buttons_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;

    // core objects
    core::DemonstrationRecorder recorder_;
    core::DMP dmp_;
    core::QuaternionDMP quat_dmp_;

    // state
    bool recording_;
    rclcpp::Time record_start_time_;
    std::vector<int32_t> prev_buttons_;  // for rising-edge detection, empty until first msg

    // params
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
