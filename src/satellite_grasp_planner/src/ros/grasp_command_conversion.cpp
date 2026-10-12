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
        case core::GraspLaunchResult::Status::kPosthocFailed:
            msg.status = GraspCommand::STATUS_POSTHOC_FAILED;
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
        msg.wrist_flip = result.wrist_flip;
        msg.provisional_delay_s = result.launch->delay_s;
        msg.cost = result.cost;
        msg.d_pos_m = result.d_pos_m;
        msg.w_hat = result.w_hat;
        msg.min_joint_margin_rad = result.min_joint_margin_rad;
        msg.posthoc_dv_m_s = result.posthoc_dv_m_s;
        msg.start_position.x = provenance.start_position.x();
        msg.start_position.y = provenance.start_position.y();
        msg.start_position.z = provenance.start_position.z();
    }
    // else: goal_pose/k/theta_star_rad/wrist_flip/provisional_delay_s/candidate diagnostics/start_position stay at their zero-init defaults.

    msg.contact_time_s = result.tau_launch_s;  // 0.0 when status != kSelected (GraspLaunchResult's own contract)
    msg.omega_rad_s = snapshot.omega_rad_s;
    msg.snapshot_theta_rad = snapshot.theta_rad;
    msg.snapshot_t_s = snapshot.t_s;
    msg.weights_sha256 = provenance.weights_sha256;

    msg.evaluations = static_cast<uint32_t>(result.evaluations);
    msg.selection_elapsed_s = result.elapsed_s;

    return msg;
}

const char* statusName(uint8_t status) {
    switch (status) {
        case GraspCommand::STATUS_UNSET: return "STATUS_UNSET";
        case GraspCommand::STATUS_SELECTED: return "STATUS_SELECTED";
        case GraspCommand::STATUS_NO_FEASIBLE_CANDIDATE: return "STATUS_NO_FEASIBLE_CANDIDATE";
        case GraspCommand::STATUS_POSTHOC_FAILED: return "STATUS_POSTHOC_FAILED";
        default: return "STATUS_<unknown>";
    }
}

}  // namespace ros_wrapper
}  // namespace satellite_grasp_planner
