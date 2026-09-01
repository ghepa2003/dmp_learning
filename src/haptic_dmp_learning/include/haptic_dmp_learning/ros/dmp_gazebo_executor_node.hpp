#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include "haptic_dmp_learning/core/dmp.hpp"
#include "haptic_dmp_learning/core/quaternion_dmp.hpp"

namespace haptic_dmp_learning {
namespace ros_wrapper {

/**
 * @brief ROS 2 Node for Replaying Trained DMPs as Cartesian Target Trajectories.
 *
 * @details
 * Functionality:
 * 1. Loads pre-trained DMP parameters (position + quaternion orientation) from a YAML file.
 * 2. Initializes the canonical phase x = 1.0 and transformation states.
 * 3. Periodically steps the dynamic systems forward at `control_rate_hz` (e.g. 200 Hz, dt = 0.005 s).
 * 4. Publishes timestamped target poses on `/target_pose` for downstream Cartesian controllers
 *    (`CartesianVelocityController` or `CartesianImpedanceController`).
 * 5. Upon reaching duration tau, clamps target pose to the exact goal attractor.
 */
class DmpGazeboExecutorNode : public rclcpp::Node {
public:
    DmpGazeboExecutorNode();

private:
    void startTimer();
    void stepCallback();

    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_pub_;
    rclcpp::TimerBase::SharedPtr startup_timer_;
    rclcpp::TimerBase::SharedPtr step_timer_;

    // Core mathematical DMP models
    core::DMP dmp_;
    core::QuaternionDMP qdmp_;

    // Rollout tracking state
    double dt_;
    double elapsed_;
    bool finished_;
    double gripper_trigger_t_ = -1.0;
    bool gripper_trigger_sent_ = false;

    // Parameters
    std::string weights_yaml_path_;
    std::string demo_csv_path_;
    std::string target_pose_topic_;
    std::string frame_id_;
    double control_rate_hz_;
    double startup_delay_sec_;
};

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning