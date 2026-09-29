#include "satellite_grasp_planner/ros/grasp_command_conversion.hpp"

namespace satellite_grasp_planner {
namespace ros_wrapper {

using satellite_grasp_msgs::msg::GraspCommand;

GraspCommand toGraspCommandMsg(const core::GraspLaunchResult& result, const core::SatelliteSnapshot& snapshot,
                               const rclcpp::Time& stamp) {
    GraspCommand msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "world";

    switch (result.status) {
        case core::GraspLaunchResult::Status::kSelected:
            msg.status = GraspCommand::STATUS_SELECTED;
            break;
        case core::GraspLaunchResult::Status::kNoFeasibleCandidate:
            msg.status = GraspCommand::STATUS_NO_FEASIBLE_CANDIDATE;
            break;
        case core::GraspLaunchResult::Status::kBudgetExhaustedNoCandidate:
            msg.status = GraspCommand::STATUS_BUDGET_EXHAUSTED_NO_CANDIDATE;
            break;
    }

    if (result.status == core::GraspLaunchResult::Status::kSelected) {
        msg.goal_pose.position.x = result.goal_position->x();
        msg.goal_pose.position.y = result.goal_position->y();
        msg.goal_pose.position.z = result.goal_position->z();
        msg.goal_pose.orientation.x = result.goal_orientation->x();
        msg.goal_pose.orientation.y = result.goal_orientation->y();
        msg.goal_pose.orientation.z = result.goal_orientation->z();
        msg.goal_pose.orientation.w = result.goal_orientation->w();
        msg.k = static_cast<uint8_t>(result.k.value());
        msg.theta_star_rad = result.theta_star_rad.value();
        msg.psi_rad = result.psi_rad.value();
        msg.provisional_delay_s = result.launch->delay_s;
    }
    // else: goal_pose/k/theta_star_rad/psi_rad/provisional_delay_s stay at their zero-init defaults.

    msg.contact_time_s = result.tau_launch_s;  // 0.0 when status != kSelected (GraspLaunchResult's own contract)
    msg.omega_rad_s = snapshot.omega_rad_s;

    const auto& sel = result.selection;
    msg.rows_evaluated = sel.rows_evaluated;
    msg.total_rollouts = sel.total_rollouts;
    msg.selection_elapsed_s = sel.elapsed_s;
    switch (sel.stop_reason) {
        case core::GraspSelection::StopReason::kCompleted:
            msg.stop_reason = GraspCommand::STOP_COMPLETED;
            break;
        case core::GraspSelection::StopReason::kBoundSatisfied:
            msg.stop_reason = GraspCommand::STOP_BOUND_SATISFIED;
            break;
        case core::GraspSelection::StopReason::kMaxRowsReached:
            msg.stop_reason = GraspCommand::STOP_MAX_ROWS_REACHED;
            break;
        case core::GraspSelection::StopReason::kTimeBudgetExhausted:
            msg.stop_reason = GraspCommand::STOP_TIME_BUDGET_EXHAUSTED;
            break;
    }
    msg.bound_violated = sel.bound_violated;
    msg.max_w_hat_seen = sel.max_w_hat_seen;

    return msg;
}

}  // namespace ros_wrapper
}  // namespace satellite_grasp_planner
