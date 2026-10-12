#pragma once

/**
 * @file grasp_launch_result.hpp
 * @brief Output of the grasp-selection + launch-planning entry points (planGraspAndLaunch(),
 *        planGraspAndLaunchS1()): one struct that never exposes a goal without an explicit "selected"
 *        status (no silent zero goal).
 */

#include <optional>

#include <Eigen/Dense>

#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "satellite_grasp_planner/core/grasp_selection.hpp"
#include "satellite_grasp_planner/core/launch_plan.hpp"

namespace satellite_grasp_planner {
namespace core {

struct GraspLaunchResult {
    enum class Status {
        kSelected,                  ///< a feasible candidate was found; all optionals below are set.
        kNoFeasibleCandidate,       ///< none of the EVALUATED rows was feasible (search completed, or
                                    ///< stopped by the bound/max_rows - check selection.stop_reason /
                                    ///< selection.truncated before concluding no feasible grasp exists
                                    ///< at all: rows past the stop point were never tried).
        kBudgetExhaustedNoCandidate,  ///< derived from selection.stop_reason == kTimeBudgetExhausted
                                      ///< with selection.found == false: the time budget ran out
                                      ///< before any feasible candidate was found.
    };
    Status status = Status::kNoFeasibleCandidate;

    /// Set only when status == kSelected. Do NOT read selection.goal_position/goal_orientation
    /// directly to decide whether a goal exists: those are the raw selectGrasp() outputs and stay at
    /// their Zero()/Identity() defaults when nothing was found (see GraspSelection).
    std::optional<Eigen::Vector3d> goal_position;
    std::optional<Eigen::Quaterniond> goal_orientation;
    std::optional<haptic_dmp_learning::core::GraspPointId> k;
    std::optional<double> theta_star_rad;
    std::optional<double> psi_rad;

    /// Full selectGrasp() diagnostics, ALWAYS populated regardless of status (bound_violated,
    /// max_w_hat_seen, stop_reason, rows_evaluated, ...). Its goal_position/goal_orientation/best are
    /// diagnostics only: use the top-level optionals above, not these, to obtain "the" goal.
    GraspSelection selection;

    /// Present iff status == kSelected. PROVISIONAL: re-run planLaunch() on a freshly re-read
    /// SatelliteSnapshot right before the actual launch; this one is only a preview computed at
    /// selection time.
    std::optional<LaunchPlan> launch;

    /// The tau actually passed to planLaunch() (see planGraspAndLaunch()'s doc comment); 0.0 when
    /// status != kSelected (no launch was planned).
    double tau_launch_s = 0.0;
};

}  // namespace core
}  // namespace satellite_grasp_planner
