#pragma once

/**
 * @file grasp_command_conversion.hpp
 * @brief Pure conversion from a planGraspAndLaunch() result to a GraspCommand message. Uses
 *        rclcpp::Time / message types only (no rclcpp::Node), so it is unit-testable without
 *        rclcpp::init().
 */

#include <cstdint>

#include <rclcpp/time.hpp>

#include "satellite_grasp_msgs/msg/grasp_command.hpp"
#include "satellite_grasp_planner/core/grasp_launch.hpp"
#include "satellite_grasp_planner/core/launch_plan.hpp"

namespace satellite_grasp_planner {
namespace ros_wrapper {

/// Converts one planGraspAndLaunch() @p result into a GraspCommand. goal_pose/k/theta_star_rad/
/// psi_rad/provisional_delay_s are left at their zero-initialized defaults when
/// @p result.status != kSelected (see GraspCommand.msg). contact_time_s is result.tau_launch_s
/// VERBATIM (no executor "+dt" correction). provisional_delay_s is diagnostic only - see the
/// message's own field comment. best_total_cost = selection.best.cost.total, only when SELECTED.
satellite_grasp_msgs::msg::GraspCommand toGraspCommandMsg(const core::GraspLaunchResult& result,
                                                          const core::SatelliteSnapshot& snapshot,
                                                          const rclcpp::Time& stamp);

/// Name of the GraspCommand::STATUS_* / STOP_* constant holding @p status / @p stop_reason, for logs.
/// "STATUS_<unknown>" / "STOP_<unknown>" for a value that matches no constant.
const char* statusName(uint8_t status);
const char* stopReasonName(uint8_t stop_reason);

}  // namespace ros_wrapper
}  // namespace satellite_grasp_planner
