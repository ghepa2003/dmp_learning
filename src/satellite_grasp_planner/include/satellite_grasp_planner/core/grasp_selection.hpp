#pragma once

/**
 * @file grasp_selection.hpp
 * @brief Selection of the grasp (k, theta, psi): candidate scan (cheap, velocity/goal terms) followed
 *        by full evaluations (rollout + collision) in partial_cost order, stopped by a bound that
 *        accounts for the terms the partial cost does not see (-w_m*w_hat and w_psi*P). Everything runs sequentially on ONE shared RobotModel; it could be parallelized
 * with one RobotModel instance per thread (RobotModel is not thread-safe).
 */

#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/grasp_cost.hpp"
#include "haptic_dmp_learning/core/grasp_roll.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "satellite_grasp_planner/core/candidate_eval.hpp"
#include "satellite_grasp_planner/core/candidate_scan.hpp"

namespace satellite_grasp_planner {
namespace core {

/// Monotonic clock for selectGrasp()'s time budget: returns seconds on an arbitrary epoch, only
/// differences are meaningful. Passing nullptr to selectGrasp() uses a std::chrono::steady_clock
/// wrapper; tests inject a fake one for determinism (see grasp_selection.cpp / test_grasp_selection.cpp).
using ClockFn = std::function<double()>;

/// PROVISIONAL proxy of w_trans,demo: translational manipulability at the END of a replica of the
/// demo from the starting posture (orientation kept), i.e. the w_trans_final of
/// checkKinematicFeasibility(template, p_start + delta_g_demo, q_start, robot, q0). To be replaced
/// by the value recorded with the new demo.
double referenceWTrans(const haptic_dmp_learning::core::ProDMP& prodmp_template,
                       const std::shared_ptr<franka_cartesian_control::core::RobotModel>& robot,
                       const franka_cartesian_control::core::RobotModel::JointVector& q0,
                       const Eigen::Vector3d& delta_g_demo);

struct SelectionParams {
    ScanParams scan;
    double w_trans_demo = 1.0;  ///< > 0 (see referenceWTrans)
    double psi_tol_rad = haptic_dmp_learning::core::kDefaultRollToleranceRad;
    double psi_refine_rad = 10.0 * 3.14159265358979323846 / 180.0;
    /// Assumed upper bound on w_hat = w_trans / w_trans_demo of any candidate, used ONLY by the
    /// stopping rule. Derived from the maximum observed, 2.06, with w_trans_demo = 0.0913; if a
    /// candidate exceeds it (see GraspSelection::bound_violated) the guarantee no longer holds.
    double w_hat_upper_bound = 2.5;
    int max_rows = 40;  ///< time cap: at most this many scan rows are evaluated
    double d_safe_m = 0.010;
    double pos_tol_mm = 20.0;
    /// Soft wall-clock budget for the WHOLE selectGrasp() call (scan included). nullopt = no limit,
    /// behavior identical to not having this field. SOFT: the first rollout of the call always starts
    /// (nothing measured yet to compare against), and a rollout that runs longer than every rollout
    /// seen so far in this call can overshoot the budget by up to about one rollout's duration - see
    /// selectGrasp()'s doc comment.
    std::optional<double> time_budget_s = std::nullopt;
};

struct EvaluatedCandidate {
    CandidateRow row;
    double psi_rad = 0.0;
    CandidateEval eval;
    haptic_dmp_learning::core::GraspCostBreakdown cost;
    int rollouts_used = 0;  ///< evaluateCandidate calls spent on this row (2 or 6, fewer if budget-cut)
    /// True if the time budget expired while this row's variants were still being evaluated: only the
    /// variants actually rolled out (rollouts_used of them) are present, the row is otherwise handled
    /// like any other (best feasible kept, or least-violating if none feasible).
    bool row_interrupted_by_budget = false;
};

struct GraspSelection {
    bool found = false;
    EvaluatedCandidate best;
    std::vector<EvaluatedCandidate> evaluated;  ///< one entry per evaluated ROW (its chosen psi)
    Eigen::Vector3d goal_position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond goal_orientation = Eigen::Quaterniond::Identity();
    int total_rollouts = 0;
    double elapsed_s = 0.0;
    bool stopped_by_bound = false;    ///< stopped because no later row can beat best (see selectGrasp)
    bool truncated = false;           ///< stopped by max_rows before the bound could apply
    int rows_evaluated = 0;
    int rows_skipped_by_bound = 0;    ///< rows left unevaluated when the bound stopped the search
    double max_w_hat_seen = 0.0;      ///< max w_hat over the FEASIBLE variants evaluated
    bool bound_violated = false;      ///< max_w_hat_seen > w_hat_upper_bound
    bool budget_exhausted = false;    ///< stopped because time_budget_s was reached (see selectGrasp)
    int rows_skipped_by_budget = 0;   ///< rows not attempted at all because the budget ran out first

    /// Single, unambiguous reason the search stopped (stopped_by_bound / truncated / budget_exhausted
    /// are kept for backward compatibility but are mutually exclusive by construction: each stops the
    /// loop immediately, checked in this fixed priority order: bound, then max_rows, then budget).
    enum class StopReason { kCompleted, kBoundSatisfied, kMaxRowsReached, kTimeBudgetExhausted };
    StopReason stop_reason = StopReason::kCompleted;
};

/**
 * ALL scan rows (every grasp point together) are sorted by increasing partial_cost and evaluated in
 * that order (branch-and-bound style). Before evaluating row r, if a feasible best exists with total
 * best_total and  r.partial_cost - w_m * w_hat_upper_bound >= best_total,  the search stops: since
 * total = partial - w_m*w_hat + w_psi*P with P >= 0, w_hat <= w_hat_upper_bound and rows sorted
 * ascending, no later row can beat best. Otherwise it stops at max_rows (truncated).
 * OPTIMALITY holds only with respect to the scan GRID and to the ASSUMED w_hat bound.
 * For each row: psi in {0, pi}
 * first; only if neither is feasible also {+-psi_refine, pi +- psi_refine}. Among the feasible
 * variants the one with minimum combineCost total is kept; if none is feasible, the variant with
 * the smallest max_joint_violation_rad is kept with eval.feasible = false (diagnostics; its cost is
 * computed but is not a selectable candidate). best = feasible entry of minimum total.
 * goal_position = best.row.goal_param, goal_orientation = applyRoll(nominal, best psi).
 * @throws std::invalid_argument for max_rows <= 0, w_hat_upper_bound <= 0, w_trans_demo <= 0, or a
 *         time_budget_s that is set but not finite or < 0 (0 is valid: it means "no rollout")
 *         (scan parameters are validated by scanCandidates).
 *
 * TIME BUDGET (params.time_budget_s, SOFT): the clock starts here, before scanCandidates(). Before
 * every evaluateCandidate() call, if elapsed-so-far + (the longest single evaluateCandidate duration
 * measured so far in THIS call, 0 before the first) >= *time_budget_s, the call is skipped and the
 * search stops (GraspSelection::budget_exhausted = true). Consequences of "soft": the very first
 * rollout of a call always starts (the estimate is 0 then), and because the estimate is the longest
 * PAST duration, a rollout that turns out slower than all previous ones is not anticipated - the
 * actual wall time can exceed time_budget_s by up to about one rollout's duration. A row cut short
 * mid-way keeps only the variants it managed to evaluate before the budget was hit
 * (EvaluatedCandidate::row_interrupted_by_budget, rollouts_used < the 2 or 6 it would otherwise have)
 * and is otherwise scored normally; a row on which the budget blocked even the first variant is not
 * counted in rows_evaluated at all. @p clock is injectable for tests (see ClockFn); nullptr uses
 * std::chrono::steady_clock.
 */
GraspSelection selectGrasp(
    const haptic_dmp_learning::core::CubeSatelliteModel& model,
    const haptic_dmp_learning::core::ProDMP& prodmp_template,
    const std::shared_ptr<franka_cartesian_control::core::RobotModel>& robot,
    const franka_cartesian_control::core::RobotModel::JointVector& q0,
    const SelectionParams& params, ClockFn clock = nullptr);

}  // namespace core
}  // namespace satellite_grasp_planner
