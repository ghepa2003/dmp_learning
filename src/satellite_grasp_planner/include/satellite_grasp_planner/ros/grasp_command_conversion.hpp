#pragma once

/**
 * @file grasp_command_conversion.hpp
 * @brief Pure conversion from a planGraspAndLaunchS1() result to a GraspCommand message. Uses
 *        rclcpp::Time / message types only (no rclcpp::Node), so it is unit-testable without
 *        rclcpp::init().
 */

#include <cstdint>
#include <string>

#include <Eigen/Dense>

#include <rclcpp/time.hpp>

#include "satellite_grasp_msgs/msg/grasp_command.hpp"
#include "satellite_grasp_planner/core/grasp_launch_result.hpp"
#include "satellite_grasp_planner/core/launch_plan.hpp"

namespace satellite_grasp_planner {
namespace ros_wrapper {

/// Values the message reports that are not in GraspLaunchResult: they are inputs of the selection.
struct GraspCommandProvenance {
    Eigen::Vector3d start_position = Eigen::Vector3d::Zero();  ///< planner p0 (EE position at q0), world frame
    std::string weights_sha256;  ///< SelectionInputs::weights_sha256
};

/// Converts one planGraspAndLaunchS1() @p result into a GraspCommand. goal_pose/k/theta_star_rad/
/// wrist_flip/provisional_delay_s and the candidate diagnostics (cost, d_pos_m, w_hat, min_joint_margin_rad,
/// posthoc_dv_m_s) are left at their zero-initialized defaults when @p result.status != kSelected (see
/// GraspCommand.msg). contact_time_s is result.tau_launch_s VERBATIM (no executor "+dt" correction).
/// provisional_delay_s is diagnostic only - see the message's own field comment.
/// start_position = @p provenance.start_position only when SELECTED (zero otherwise);
/// evaluations, selection_elapsed_s, snapshot_theta_rad/snapshot_t_s (from @p snapshot) and weights_sha256
/// are always set.
satellite_grasp_msgs::msg::GraspCommand toGraspCommandMsg(const core::GraspLaunchResult& result,
                                                          const core::SatelliteSnapshot& snapshot,
                                                          const rclcpp::Time& stamp,
                                                          const GraspCommandProvenance& provenance);

/// Name of the GraspCommand::STATUS_* constant holding @p status, for logs. "STATUS_<unknown>" for a value
/// that matches no constant.
const char* statusName(uint8_t status);

}  // namespace ros_wrapper
}  // namespace satellite_grasp_planner
