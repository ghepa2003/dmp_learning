#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "haptic_dmp_learning/core/dmp.hpp"
#include "haptic_dmp_learning/core/quaternion_dmp.hpp"

namespace haptic_dmp_learning {
namespace ros_wrapper {

// Loads a previously learned DMP (position + orientation) from a weights YAML
// and replays it by integrating dmp_.step()/qdmp_.step() at a fixed rate,
// publishing each pose as the target for a downstream Cartesian controller.
class DmpGazeboExecutorNode : public rclcpp::Node {
public:
    DmpGazeboExecutorNode();

private:
    void startTimer();
    void stepCallback();

    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr startup_timer_;
    rclcpp::TimerBase::SharedPtr step_timer_;

    // core objects
    core::DMP dmp_;
    core::QuaternionDMP qdmp_;

    // rollout state
    double dt_;
    double elapsed_;
    bool finished_;

    // params
    std::string weights_yaml_path_;
    std::string target_pose_topic_;
    std::string frame_id_;
    double control_rate_hz_;
    double startup_delay_sec_;
};

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning