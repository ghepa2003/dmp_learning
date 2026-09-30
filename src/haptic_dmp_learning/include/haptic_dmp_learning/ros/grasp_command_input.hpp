#pragma once

/**
 * @file grasp_command_input.hpp
 * @brief Header-only conversion satellite_grasp_msgs/GraspCommand -> core::satellite_intercept::
 *        GraspCommandInputs, for prodmp_gazebo_executor_node's grasp_goal_source="grasp_command".
 *        Message types only (no rclcpp::Node), so it is unit-testable without rclcpp::init().
 */

#include <rclcpp/time.hpp>

#include "haptic_dmp_learning/core/satellite_intercept.hpp"
#include "satellite_grasp_msgs/msg/grasp_command.hpp"

namespace haptic_dmp_learning {
namespace ros_wrapper {

using satellite_grasp_msgs::msg::GraspCommand;
namespace si = core::satellite_intercept;

// The core mirrors the status constants (it has no ROS): fail the build if the message changes them.
static_assert(si::kGraspStatusUnset == GraspCommand::STATUS_UNSET, "GraspCommand STATUS_UNSET changed");
static_assert(si::kGraspStatusSelected == GraspCommand::STATUS_SELECTED, "GraspCommand STATUS_SELECTED changed");
static_assert(si::kGraspStatusNoFeasibleCandidate == GraspCommand::STATUS_NO_FEASIBLE_CANDIDATE,
              "GraspCommand STATUS_NO_FEASIBLE_CANDIDATE changed");
static_assert(si::kGraspStatusBudgetExhaustedNoCandidate == GraspCommand::STATUS_BUDGET_EXHAUSTED_NO_CANDIDATE,
              "GraspCommand STATUS_BUDGET_EXHAUSTED_NO_CANDIDATE changed");

/// Copies the fields the executor uses (position only: goal orientation, psi, k, provisional_delay_s and
/// best_total_cost are not read). header.stamp -> seconds via rclcpp::Time.
inline si::GraspCommandInputs toGraspCommandInputs(const GraspCommand& msg) {
    si::GraspCommandInputs in;
    in.status = msg.status;
    in.frame_id = msg.header.frame_id;
    in.stamp_s = rclcpp::Time(msg.header.stamp).seconds();
    in.goal_position = Eigen::Vector3d(msg.goal_pose.position.x, msg.goal_pose.position.y, msg.goal_pose.position.z);
    in.start_position = Eigen::Vector3d(msg.start_position.x, msg.start_position.y, msg.start_position.z);
    in.theta_star_rad = msg.theta_star_rad;
    in.omega_rad_s = msg.omega_rad_s;
    in.contact_time_s = msg.contact_time_s;
    in.snapshot_theta_rad = msg.snapshot_theta_rad;
    in.snapshot_t_s = msg.snapshot_t_s;
    in.weights_sha256 = msg.weights_sha256;
    in.rows_evaluated = msg.rows_evaluated;
    in.stop_reason = msg.stop_reason;
    in.bound_violated = msg.bound_violated;
    return in;
}

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning
