#include "satellite_grasp_planner/core/grasp_s3_launch.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

#include "haptic_dmp_learning/core/grasp_roll.hpp"
#include "haptic_dmp_learning/core/math_utils.hpp"

namespace satellite_grasp_planner {
namespace core {

GraspLaunchResult planGraspAndLaunchS1(const SelectionInputs& in, const SatelliteSnapshot& snapshot,
                                       double min_delay_s, const S3Params& s3,
                                       std::optional<double> t_now_override_s, S3Result* s3_result_out) {
    // Same preconditions as planGraspAndLaunch().
    if (!std::isfinite(min_delay_s) || min_delay_s < 0.0) {
        throw std::invalid_argument("planGraspAndLaunchS1: min_delay_s must be finite and >= 0.");
    }
    if (snapshot.omega_rad_s == 0.0) {
        throw std::invalid_argument("planGraspAndLaunchS1: snapshot_at_acquisition.omega_rad_s is zero.");
    }
    if (snapshot.omega_rad_s != in.params.scan.omega_rad_s || snapshot.center != in.cube_params.center_world ||
        snapshot.axis != in.cube_params.axis_world) {
        throw std::invalid_argument(
            "planGraspAndLaunchS1: snapshot_at_acquisition does not match the snapshot 'in' was built from "
            "(omega_rad_s/center/axis differ) - pass the ACQUISITION snapshot.");
    }
    const double tau_launch_s = in.contact_assumed_at_end ? in.prodmp_template.tau() : in.params.scan.tau_contact_s;
    if (!(tau_launch_s > 0.0)) {
        throw std::invalid_argument("planGraspAndLaunchS1: tau_launch_s <= 0 - refusing to plan the launch.");
    }

    const auto t_start = std::chrono::steady_clock::now();
    const haptic_dmp_learning::core::CubeSatelliteModel model(in.cube_params);
    RobotModel& robot = *in.robot;

    robot.update(in.q0, RobotModel::JointVector::Zero());
    DemoReference demo;
    demo.position = in.params.scan.p0 + in.params.scan.delta_g_demo;
    demo.orientation = robot.eeOrientation();
    demo.w_trans_demo = in.params.w_trans_demo;

    S3Context ctx;
    ctx.prodmp_template = &in.prodmp_template;
    ctx.contact_to_end_offset = in.params.scan.contact_to_end_offset;
    ctx.tau_contact_s = in.params.scan.tau_contact_s;
    ctx.tau_launch_s = tau_launch_s;
    ctx.snapshot = snapshot;
    ctx.t_now_s = t_now_override_s.value_or(snapshot.t_s);
    ctx.min_delay_s = min_delay_s;

    const S3Result r = selectGraspS3(model, robot, in.q0, demo, in.params.scan.delta_g_demo, ctx, s3);
    const double elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    if (s3_result_out) *s3_result_out = r;

    GraspLaunchResult out;
    out.selection.rows_evaluated = r.evaluations;
    out.selection.total_rollouts = r.posthoc_checks;
    out.selection.max_w_hat_seen = r.max_w_hat_passing;
    out.selection.bound_violated = false;
    out.selection.stop_reason = GraspSelection::StopReason::kCompleted;
    out.selection.elapsed_s = elapsed_s;

    if (r.status != S3Result::kSelected) {
        out.status = GraspLaunchResult::Status::kNoFeasibleCandidate;
        return out;
    }
    out.status = GraspLaunchResult::Status::kSelected;
    out.tau_launch_s = tau_launch_s;

    const S3Candidate& c = r.chosen;
    const Eigen::Quaterniond& q_nominal = c.eval.pose.orientation_nominal_world;
    const Eigen::Quaterniond q_goal =
        c.eval.wrist_flip ? haptic_dmp_learning::core::applyRoll(q_nominal, haptic_dmp_learning::core::kPi)
                          : q_nominal;
    const Eigen::Vector3d goal = c.eval.pose.position_world + ctx.contact_to_end_offset;
    const double psi = c.eval.wrist_flip ? haptic_dmp_learning::core::kPi : 0.0;

    out.selection.found = true;
    out.selection.goal_position = goal;
    out.selection.goal_orientation = q_goal;
    out.selection.best.row.k = c.eval.k;
    out.selection.best.row.theta_rad = c.eval.theta_rad;
    out.selection.best.row.contact_point = c.eval.pose.position_world;
    out.selection.best.row.goal_param = goal;
    out.selection.best.psi_rad = psi;
    out.selection.best.cost.total = c.cost;

    const double t_now_s = t_now_override_s.value_or(snapshot.t_s + elapsed_s);
    out.launch = planLaunch(snapshot, t_now_s, c.eval.theta_rad, tau_launch_s, min_delay_s);
    out.goal_position = goal;
    out.goal_orientation = q_goal;
    out.k = c.eval.k;
    out.theta_star_rad = c.eval.theta_rad;
    out.psi_rad = psi;
    return out;
}

}  // namespace core
}  // namespace satellite_grasp_planner
