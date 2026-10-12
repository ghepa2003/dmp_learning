#pragma once

/**
 * @file grasp_s3_launch.hpp
 * @brief Counterpart of planGraspAndLaunch() that selects with selectGraspS3() (selection_mode "s1") and
 *        returns the same GraspLaunchResult, so the GraspCommand conversion is shared.
 */

#include <optional>

#include "satellite_grasp_planner/core/grasp_launch.hpp"
#include "satellite_grasp_planner/core/grasp_s3_selection.hpp"

namespace satellite_grasp_planner {
namespace core {

/**
 * Same preconditions and @throws as planGraspAndLaunch() (min_delay_s, omega, snapshot consistency with
 * @p in). The inputs come from SelectionInputs: q_start = in.q0, delta_g_demo / contact_to_end_offset /
 * tau_contact_s from in.params.scan, w_trans_demo from in.params.w_trans_demo, and the demo reference pose =
 * (p0 + delta_g_demo, orientation of FK(q0)) - the orientation only matters if s3.orientation_weight > 0.
 * The legacy search parameters in in.params (scan window/step, max_rows, psi, time budget) are NOT used.
 *
 * t_now: the wait of each candidate is evaluated at t_now_override_s if given, else at snapshot.t_s; the
 * final planLaunch() uses t_now_override_s if given, else snapshot.t_s + the wall-clock selection time
 * (same convention and caveats as planGraspAndLaunch()).
 *
 * Field mapping into GraspLaunchResult (and, through toGraspCommandMsg, GraspCommand):
 *   status: kSelected -> kSelected; kNoCandidate and kPosthocFailed -> kNoFeasibleCandidate (no dedicated
 *     message constant exists; S3Result::note says which one it was, selection.* carries the counts)
 *   goal_position = grasp position + contact_to_end_offset (as the legacy goal_param);
 *   goal_orientation = nominal orientation, rolled by pi about z_tcp if wrist_flip; psi_rad = wrist_flip ? pi : 0
 *   k, theta_star_rad = chosen candidate; selection.best.cost.total = S3 cost (-> best_total_cost)
 *   selection.rows_evaluated = evaluations; selection.total_rollouts = posthoc_checks
 *   selection.max_w_hat_seen = max w_hat over the constraint-passing candidates; bound_violated = false
 *   selection.stop_reason = kCompleted: closest existing constant (the S3 grid is always scanned entirely;
 *     there is no bound, row cap or time budget)
 */
GraspLaunchResult planGraspAndLaunchS1(const SelectionInputs& in, const SatelliteSnapshot& snapshot_at_acquisition,
                                       double min_delay_s, const S3Params& s3,
                                       std::optional<double> t_now_override_s = std::nullopt,
                                       S3Result* s3_result_out = nullptr);

}  // namespace core
}  // namespace satellite_grasp_planner
