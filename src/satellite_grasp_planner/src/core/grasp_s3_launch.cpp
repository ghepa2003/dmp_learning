#include "satellite_grasp_planner/core/grasp_s3_launch.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace satellite_grasp_planner {
namespace core {

GraspLaunchResult planGraspAndLaunchS1(const SelectionInputs& in, const SatelliteSnapshot& snapshot_at_acquisition,
                                       double min_delay_s, const S3Params& s3,
                                       std::optional<double> t_now_override_s, S3Result* s3_result_out,
                                       std::vector<S3EvalRecord>* eval_log,
                                       const ApproachClearanceModel* clearance_model,
                                       const JointDynamics* joint_dynamics) {
  if (!std::isfinite(min_delay_s) || min_delay_s < 0.0) {
    throw std::invalid_argument("planGraspAndLaunchS1: min_delay_s must be finite and >= 0.");
  }
  if (snapshot_at_acquisition.omega_rad_s == 0.0) {
    throw std::invalid_argument("planGraspAndLaunchS1: snapshot_at_acquisition.omega_rad_s is zero.");
  }
  if (snapshot_at_acquisition.omega_rad_s != in.omega_rad_s || snapshot_at_acquisition.center != in.cube_params.center_world ||
      snapshot_at_acquisition.axis != in.cube_params.axis_world) {
    throw std::invalid_argument(
        "planGraspAndLaunchS1: snapshot_at_acquisition does not match the snapshot 'in' was built from "
        "(omega_rad_s/center/axis differ) - pass the ACQUISITION snapshot.");
  }
  const double tau_launch_s = in.contact_assumed_at_end ? in.prodmp_template.tau() : in.tau_contact_s;
  if (!(tau_launch_s > 0.0)) {
    throw std::invalid_argument("planGraspAndLaunchS1: tau_launch_s <= 0 - refusing to plan the launch.");
  }

  const auto t_start = std::chrono::steady_clock::now();
  const haptic_dmp_learning::core::CubeSatelliteModel model(in.cube_params);
  RobotModel& robot = *in.robot;

  robot.update(in.q0, RobotModel::JointVector::Zero());
  DemoReference demo;
  demo.position = in.p0 + in.delta_g_demo;
  demo.orientation = robot.eeOrientation();
  demo.w_trans_demo = in.w_trans_demo;

  S3Context ctx;
  ctx.prodmp_template = &in.prodmp_template;
  ctx.contact_to_end_offset = in.contact_to_end_offset;
  ctx.tau_contact_s = in.tau_contact_s;
  ctx.tau_launch_s = tau_launch_s;
  ctx.snapshot = snapshot_at_acquisition;
  ctx.t_now_s = t_now_override_s.value_or(snapshot_at_acquisition.t_s);
  ctx.min_delay_s = min_delay_s;
  ctx.eval_log = eval_log;
  if (s3.use_clearance) {
    if (!clearance_model) {
      throw std::invalid_argument("planGraspAndLaunchS1: s1.use_clearance=true needs a clearance_model.");
    }
    ctx.clearance_eval = makeClearanceEvaluator(*clearance_model, ClearanceParams{s3.clearance_dt_s, s3.gripper_open_m});
  }
  if (s3.use_dynamic_follow) {
    if (!joint_dynamics) {
      throw std::invalid_argument("planGraspAndLaunchS1: s1.use_dynamic_follow=true needs joint_dynamics.");
    }
    ctx.joint_dynamics = joint_dynamics;
  }

  const S3Result r = selectGraspS3(model, robot, in.q0, demo, in.delta_g_demo, ctx, s3);
  const double elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
  if (s3_result_out) *s3_result_out = r;

  GraspLaunchResult out;
  out.evaluations = r.evaluations;
  out.elapsed_s = elapsed_s;
  out.stage_ms.scan = r.scan_ms;
  out.stage_ms.clearance = r.clearance_stage_ms;
  out.stage_ms.dynamic = r.dynamic_stage_ms;
  out.stage_ms.posthoc = r.posthoc_ms;
  out.stage_ms.finalists = static_cast<int>(r.v2_finalists.size());
  for (const S3FinalistRecord& f : r.v2_finalists) {
    out.stage_ms.rollout += f.rollout_ms;
    out.stage_ms.hold_end += f.hold_end_ms;
  }

  if (r.status != S3Result::kSelected) {
    out.status = r.status == S3Result::kPosthocFailed ? GraspLaunchResult::Status::kPosthocFailed
                                                      : GraspLaunchResult::Status::kNoFeasibleCandidate;
    return out;
  }
  out.status = GraspLaunchResult::Status::kSelected;
  out.tau_launch_s = tau_launch_s;

  const S3Candidate& c = r.chosen;
  // applyRoll(qn, psi + (flip ? pi : 0)): rolls about the same axis (z_tcp) commute.
  const Eigen::Quaterniond q_goal = goalOrientation(c.eval);
  const Eigen::Vector3d goal = c.eval.pose.position_world + ctx.contact_to_end_offset;

  const double t_now_s = t_now_override_s.value_or(snapshot_at_acquisition.t_s + elapsed_s);
  out.launch = planLaunch(snapshot_at_acquisition, t_now_s, c.eval.theta_rad, tau_launch_s, min_delay_s);
  out.goal_position = goal;
  out.goal_orientation = q_goal;
  out.k = c.eval.k;
  out.theta_star_rad = c.eval.theta_rad;
  out.wrist_flip = c.eval.wrist_flip;
  out.psi_rad = c.eval.psi_rad;
  out.cost = c.cost;
  out.d_pos_m = c.d_pos_m;
  out.w_hat = c.eval.w_hat;
  out.min_joint_margin_rad = c.eval.margins.min_margin_rad;
  out.posthoc_dv_m_s = s3.posthoc_velocity_check ? r.posthoc_dv_m_s : std::numeric_limits<double>::quiet_NaN();
  if (r.hold_end_done) {
    out.hold_end_done = true;
    out.end_ok = r.end_ok;
    out.q_hold_end = r.q_hold_end;
    out.hold_position = r.hold_pose.position;
    out.hold_orientation = r.hold_pose.orientation;
    out.w_end = r.w_end;
    out.w_eff = r.w_eff;
  }
  return out;
}

}  // namespace core
}  // namespace satellite_grasp_planner
