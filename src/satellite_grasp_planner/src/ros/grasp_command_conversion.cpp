#include "satellite_grasp_planner/ros/grasp_command_conversion.hpp"

namespace satellite_grasp_planner {
namespace ros_wrapper {

using satellite_grasp_msgs::msg::GraspCommand;

GraspCommand toGraspCommandMsg(const core::GraspLaunchResult& result, const core::SatelliteSnapshot& snapshot,
                               const rclcpp::Time& stamp, const GraspCommandProvenance& provenance) {
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
        msg.best_total_cost = result.selection.best.cost.total;
        msg.start_position.x = provenance.start_position.x();
        msg.start_position.y = provenance.start_position.y();
        msg.start_position.z = provenance.start_position.z();
    }
    // else: goal_pose/k/theta_star_rad/psi_rad/provisional_delay_s/best_total_cost/start_position stay at their zero-init defaults.

    msg.contact_time_s = result.tau_launch_s;  // 0.0 when status != kSelected (GraspLaunchResult's own contract)
    msg.omega_rad_s = snapshot.omega_rad_s;
    msg.snapshot_theta_rad = snapshot.theta_rad;
    msg.snapshot_t_s = snapshot.t_s;
    msg.weights_sha256 = provenance.weights_sha256;

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

const char* statusName(uint8_t status) {
    switch (status) {
        case GraspCommand::STATUS_UNSET: return "STATUS_UNSET";
        case GraspCommand::STATUS_SELECTED: return "STATUS_SELECTED";
        case GraspCommand::STATUS_NO_FEASIBLE_CANDIDATE: return "STATUS_NO_FEASIBLE_CANDIDATE";
        case GraspCommand::STATUS_BUDGET_EXHAUSTED_NO_CANDIDATE: return "STATUS_BUDGET_EXHAUSTED_NO_CANDIDATE";
        default: return "STATUS_<unknown>";
    }
}

const char* stopReasonName(uint8_t stop_reason) {
    switch (stop_reason) {
        case GraspCommand::STOP_UNSET: return "STOP_UNSET";
        case GraspCommand::STOP_COMPLETED: return "STOP_COMPLETED";
        case GraspCommand::STOP_BOUND_SATISFIED: return "STOP_BOUND_SATISFIED";
        case GraspCommand::STOP_MAX_ROWS_REACHED: return "STOP_MAX_ROWS_REACHED";
        case GraspCommand::STOP_TIME_BUDGET_EXHAUSTED: return "STOP_TIME_BUDGET_EXHAUSTED";
        default: return "STOP_<unknown>";
    }
}

}  // namespace ros_wrapper
}  // namespace satellite_grasp_planner
