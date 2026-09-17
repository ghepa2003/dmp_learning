#pragma once

#include <string>
#include <vector>
#include <memory>
#include <atomic>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "franka_cartesian_control/core/velocity_ik_solver.hpp"

#include <realtime_tools/realtime_publisher.hpp>

#include "franka_cartesian_control/ros/ros_utils.hpp"

namespace franka_cartesian_control {
namespace ros_wrapper {

// ros2_control velocity controller: reads a target Cartesian pose from a
// topic, computes the pose error against the current end-effector pose
// (via core::RobotModel, Pinocchio-based FK/Jacobian), solves the desired
// joint velocities via damped least squares (core::VelocityIkSolver), and
// commands them on the velocity command interfaces. First step of the
// staged plan (velocity now, impedance/force later reusing RobotModel and
// CartesianError unchanged).

class CartesianVelocityController : public controller_interface::ControllerInterface {
public:
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type update(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void targetTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    // params
    std::vector<std::string> joint_names_;
    std::string ee_frame_name_;
    std::string target_pose_topic_;

    // Velocity feedforward (v_cmd = v_ff + Kp * e). Default OFF: with it off the
    // controller is byte-identical to the pre-feedforward Kp-only behaviour, and
    // the twist subscription/guards below simply go unused.
    bool feedforward_enabled_;
    std::string target_twist_topic_;
    // Single tolerance used both as (a) max age of the twist sample relative to
    // the current control-loop time (staleness) and (b) max |stamp| skew between
    // the target_pose and target_twist samples actually combined in the same
    // v_ff + Kp*e sum (cross-buffer sync) - see update(). Two independent
    // RealtimeBuffers can each individually look "fresh" while holding samples
    // from different DMP/ProDMP ticks; this bounds that skew explicitly instead
    // of trusting per-buffer freshness alone.
    double feedforward_tolerance_sec_;

    // core objects
    std::unique_ptr<core::RobotModel> robot_model_;
    core::VelocityIkSolver ik_solver_;

    // target pose
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
    realtime_tools::RealtimeBuffer<geometry_msgs::msg::PoseStamped> target_pose_buffer_;
    std::atomic<bool> target_received_{false};

    // target velocity feedforward (demo-local frame, same frame as the raw pose
    // handed to FrameAligner - aligned into base frame in update() via
    // FrameAligner::alignVelocity() before use: linear unrotated, angular
    // rotated by the captured orientation offset, mirroring align()).
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr target_twist_sub_;
    realtime_tools::RealtimeBuffer<geometry_msgs::msg::TwistStamped> target_twist_buffer_;
    std::atomic<bool> target_twist_received_{false};

    // frame aligner
    FrameAligner frame_aligner_;

    // debug publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr aligned_target_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr actual_pose_pub_;
    std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>> rt_aligned_target_pub_;
    std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>> rt_actual_pose_pub_;
};

}  // namespace ros_wrapper
}  // namespace franka_cartesian_control