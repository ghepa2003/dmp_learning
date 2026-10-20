#pragma once

/**
 * @file grasp_s3_launch.hpp
 * @brief Selection S1: selectGraspS3() followed by planLaunch(), returned as a GraspLaunchResult that the
 *        GraspCommand conversion consumes.
 */

#include <optional>
#include <vector>

#include "satellite_grasp_planner/core/grasp_launch_result.hpp"
#include "satellite_grasp_planner/core/grasp_s3_selection.hpp"
#include "satellite_grasp_planner/core/selection_inputs.hpp"

namespace satellite_grasp_planner {
namespace core {

/**
 * @throws std::invalid_argument if min_delay_s is not finite or < 0, if the snapshot's omega is zero, if the
 * snapshot does not match the one @p in was built from (omega_rad_s / center / axis), or if tau_launch_s <= 0.
 * The inputs come from SelectionInputs: q_start = in.q0, delta_g_demo / contact_to_end_offset /
 * tau_contact_s / w_trans_demo as in the struct, and the demo reference pose =
 * (p0 + delta_g_demo, orientation of FK(q0)) - the orientation only matters if s3.orientation_weight > 0.
 *
 * t_now: the wait of each candidate is evaluated at t_now_override_s if given, else at snapshot.t_s; the
 * final planLaunch() uses t_now_override_s if given, else snapshot.t_s + the wall-clock selection time
 * (which assumes the snapshot's clock runs at wall-clock rate).
 *
 * Field mapping into GraspLaunchResult (and, through toGraspCommandMsg, GraspCommand):
 *   status: kSelected -> kSelected; kNoCandidate -> kNoFeasibleCandidate; kPosthocFailed -> kPosthocFailed
 *     (S3Result::note, available through @p s3_result_out, says why)
 *   goal_position = grasp position + contact_to_end_offset;
 *   goal_orientation = goalOrientation(chosen.eval) = nominal orientation rolled by psi (+ pi about z_tcp if wrist_flip);
 *     psi_rad = chosen psi (0 unless S3Params::force sets it; not part of GraspCommand)
 *   k, theta_star_rad, wrist_flip, cost, d_pos_m, w_hat = chosen candidate; min_joint_margin_rad = margin of its
 *     IK solution; posthoc_dv_m_s = S3Result::posthoc_dv_m_s, or NaN if
 *     s3.posthoc_velocity_check is false (the velocity was not measured)
 *   evaluations = S3Result::evaluations; elapsed_s = wall-clock time of the selection
 *
 * eval_log: optional per-candidate log forwarded to S3Context::eval_log (nullptr = off, no change of result).
 *
 * clearance_model: required (non-null, outliving the call) iff s3.use_clearance; its evaluator is built here from
 * s3.clearance_dt_s and s3.gripper_open_m and passed in S3Context::clearance_eval. Ignored if use_clearance is off.
 * @throws std::invalid_argument if s3.use_clearance and clearance_model is null.
 */
GraspLaunchResult planGraspAndLaunchS1(const SelectionInputs& in, const SatelliteSnapshot& snapshot_at_acquisition,
                                       double min_delay_s, const S3Params& s3,
                                       std::optional<double> t_now_override_s = std::nullopt,
                                       S3Result* s3_result_out = nullptr,
                                       std::vector<S3EvalRecord>* eval_log = nullptr,
                                       const ApproachClearanceModel* clearance_model = nullptr);

}  // namespace core
}  // namespace satellite_grasp_planner
