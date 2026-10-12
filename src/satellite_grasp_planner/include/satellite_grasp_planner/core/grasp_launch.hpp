#pragma once

/**
 * @file grasp_launch.hpp
 * @brief Single entry point chaining selectGrasp() and planLaunch() from SelectionInputs, with one
 *        output struct that never exposes a goal without an explicit "selected" status (no silent
 *        zero goal - see GraspLaunchResult).
 */

#include <optional>

#include <Eigen/Dense>

#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "satellite_grasp_planner/core/grasp_launch_result.hpp"  // GraspLaunchResult: moved out, re-included for existing users
#include "satellite_grasp_planner/core/grasp_selection.hpp"
#include "satellite_grasp_planner/core/launch_plan.hpp"
#include "satellite_grasp_planner/core/selection_inputs.hpp"

namespace satellite_grasp_planner {
namespace core {

/**
 * @brief Runs selectGrasp() from @p in, then (if a candidate was found) planLaunch() for it.
 *
 * @p snapshot_at_acquisition MUST be the ACQUISITION snapshot: the same satellite measurement
 * @p in was built from (i.e. what was passed as the snapshot argument to buildSelectionInputs()).
 * This is enforced up front (see the first @throws below) - it is NOT the snapshot re-read right
 * before the actual launch; that one belongs to a SEPARATE, later planLaunch() call (see
 * GraspLaunchResult::launch: PROVISIONAL).
 *
 * @p in.params / @p in.prodmp_template / @p in.robot / @p in.q0 are passed to selectGrasp() exactly as
 * built by buildSelectionInputs() - this function does NOT modify scan.tau_contact_s or any other scan
 * result, so the verified selection stays identical to calling selectGrasp() directly.
 *
 * @throws std::invalid_argument, BEFORE selectGrasp() runs (no rollout is attempted), if: @p
 * min_delay_s is not finite or is < 0; @p snapshot_at_acquisition.omega_rad_s == 0; or
 * @p snapshot_at_acquisition is inconsistent with @p in - omega_rad_s != in.params.scan.omega_rad_s,
 * center != in.cube_params.center_world, or axis != in.cube_params.axis_world (exact comparison:
 * buildSelectionInputs() copies these values verbatim from the snapshot it was given, so any
 * difference means @p snapshot_at_acquisition is not that same snapshot).
 *
 * tau_launch_s = in.contact_assumed_at_end ? in.prodmp_template.tau() : in.params.scan.tau_contact_s.
 * Rationale: scanCandidates() itself treats tau_contact_s <= 0 as "contact at the template's own tau"
 * (see candidate_scan.cpp) - the ProDMP is rolled out that far before the velocity/goal terms are
 * computed - but launchDelay()/planLaunch() do NOT know that convention: launchDelay() has no
 * tau_contact_s > 0 check at all, and planLaunch() throws for tau_contact_s <= 0. Forwarding
 * in.params.scan.tau_contact_s verbatim would therefore either silently mis-time the launch or throw;
 * this function always substitutes the correct value instead, so 0 is never passed to launchDelay()
 * or planLaunch(). @throws std::invalid_argument if the resulting tau_launch_s is <= 0 (defensive:
 * prodmp_template.tau() is validated > 0 by learnFromDemonstration() but NOT by setLearnedParameters(),
 * which is what loading a template from a weights file on disk uses - a corrupt weights file could in
 * principle carry a non-positive tau).
 *
 * t_now_s (the instant selection ended, passed to planLaunch()) = @p t_now_override_s if given,
 * else snapshot_at_acquisition.t_s + selection.elapsed_s. CAVEATS: (1) this assumes
 * snapshot_at_acquisition's clock runs at wall-clock rate - false under a Gazebo real-time factor != 1;
 * (2) selection.elapsed_s times ONLY selectGrasp() itself: it excludes time already spent inside
 * buildSelectionInputs(), which runs one full extra rollout (referenceWTrans(), see
 * candidate_eval.hpp's "Cost and threading" timing note for one rollout's duration) whenever the
 * demo's end_joints_rad is absent (in.w_trans_demo_is_proxy == true) - in that case the default
 * underestimates real elapsed time and @p t_now_override_s should be used instead. @p min_delay_s
 * (forwarded to planLaunch()) must be the communication delay alone: it must NOT also include the
 * selection time already folded into t_now_s.
 *
 * The result is PROVISIONAL (see GraspLaunchResult::launch).
 *
 * @param min_delay_s REQUIRED, no default: the communication delay D is not yet decided.
 * @param clock Injectable clock forwarded to selectGrasp(); nullptr uses std::chrono::steady_clock.
 */
GraspLaunchResult planGraspAndLaunch(const SelectionInputs& in,
                                     const SatelliteSnapshot& snapshot_at_acquisition,
                                     double min_delay_s, ClockFn clock = nullptr,
                                     std::optional<double> t_now_override_s = std::nullopt);

}  // namespace core
}  // namespace satellite_grasp_planner
