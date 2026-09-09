#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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
 *
 * One-shot target retarget: the trained weights carry the goal position frozen
 * from the demonstration. Before the rollout starts, the CURRENT position of
 * free_target_object is read once from its odometry and applied via
 * `DMP::setGoal()`, so the replay converges toward where the object actually is
 * now. Translation only - the orientation DMP (`qdmp_`) keeps the demo goal.
 */
class DmpGazeboExecutorNode : public rclcpp::Node {
public:
    DmpGazeboExecutorNode();

private:
    void startTimer();
    void stepCallback();
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_odom_sub_;
    // TF (world -> EE) for the goal-frame correction in startTimer(); created
    // only when target_odom_required_ is true.
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    // One-shot event: the replay gripper close ramp has finished (fingers at
    // gripper_closed_position_). grasp_force_calibration_node captures force on this.
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr gripper_close_complete_pub_;
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

    // Gripper close ramp state. Instead of one step command at gripper_trigger_t_,
    // the position setpoint is walked from open to closed over
    // gripper_close_ramp_duration_sec_ (see core/gripper_ramp.hpp). elapsed_ is
    // reused as the monotonic sim-time base - no second clock, no extra timer
    // (stepCallback already ticks at control_rate_hz_).
    bool gripper_ramp_active_ = false;
    double gripper_ramp_start_elapsed_ = -1.0;

    // One-shot translational retarget toward the live free_target_object position
    // (see class doc). qdmp_ / orientation is intentionally left untouched.
    bool target_odom_required_;        ///< false => no odom wait/retarget, replay the demo's original goal as-is
    std::string target_odom_topic_;
    std::string world_frame_;          ///< TF frame the target odometry is expressed in
    std::string ee_frame_;             ///< TF end-effector (TCP) frame, looked up as world_frame_ -> ee_frame_
    double target_odom_timeout_sec_;   ///< fail-loud if no odometry within this many seconds (only when required)
    bool target_odom_received_ = false;
    bool odom_wait_started_ = false;   ///< true once startTimer() began counting toward the timeout
    rclcpp::Time odom_wait_start_;     ///< node-clock instant the odometry wait started
    Eigen::Vector3d target_position_ = Eigen::Vector3d::Zero();

    // Parameters
    std::string weights_yaml_path_;
    std::string demo_csv_path_;
    std::string target_pose_topic_;
    std::string frame_id_;
    double control_rate_hz_;
    double startup_delay_sec_;
    double gripper_open_position_;
    double gripper_closed_position_;
    double gripper_close_ramp_duration_sec_;
};

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning