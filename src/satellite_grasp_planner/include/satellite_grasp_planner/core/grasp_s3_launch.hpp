#pragma once

/**
 * @file grasp_s3_launch.hpp
 * @brief Selection S1: selectGraspS3() followed by planLaunch(), returned as a GraspLaunchResult that the
 *        GraspCommand conversion consumes.
 */

#include <optional>

#include "satellite_grasp_planner/core/grasp_launch_result.hpp"
#include "satellite_grasp_planner/core/grasp_s3_selection.hpp"
#include "satellite_grasp_planner/core/selection_inputs.hpp"

namespace satellite_grasp_planner {
namespace core {

/**
 * @throws std::invalid_argument if min_delay_s is not finite or < 0, if the snapshot's omega is zero, if the
 * snapshot does not match the one @p in was built from (omega_rad_s / center / axis), or if tau_launch_s <= 0.
 * The inputs come from SelectionInputs: q_start = in.q0, delta_g_demo / contact_to_end_offset /
 * tau_contact_s from in.params.scan, w_trans_demo from in.params.w_trans_demo, and the demo reference pose =
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
 *   goal_orientation = nominal orientation, rolled by pi about z_tcp if wrist_flip
 *   k, theta_star_rad, wrist_flip, cost, d_pos_m, w_hat = chosen candidate; min_joint_margin_rad = margin of its
 *     IK solution; posthoc_dv_m_s = S3Result::posthoc_dv_m_s
 *   evaluations = S3Result::evaluations; elapsed_s = wall-clock time of the selection
 */
GraspLaunchResult planGraspAndLaunchS1(const SelectionInputs& in, const SatelliteSnapshot& snapshot_at_acquisition,
                                       double min_delay_s, const S3Params& s3,
                                       std::optional<double> t_now_override_s = std::nullopt,
                                       S3Result* s3_result_out = nullptr);

}  // namespace core
}  // namespace satellite_grasp_planner
