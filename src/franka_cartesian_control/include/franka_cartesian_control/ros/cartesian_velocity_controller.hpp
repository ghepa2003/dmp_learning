#pragma once

#include <string>
#include <vector>
#include <memory>
#include <atomic>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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

    // params
    std::vector<std::string> joint_names_;
    std::string ee_frame_name_;
    std::string target_pose_topic_;

    // core objects
    std::unique_ptr<core::RobotModel> robot_model_;
    core::VelocityIkSolver ik_solver_;

    // target pose
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
    realtime_tools::RealtimeBuffer<geometry_msgs::msg::PoseStamped> target_pose_buffer_;
    std::atomic<bool> target_received_{false};

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