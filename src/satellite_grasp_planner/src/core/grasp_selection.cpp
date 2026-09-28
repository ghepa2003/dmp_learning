#include "satellite_grasp_planner/core/grasp_selection.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>

#include "satellite_grasp_planner/core/kinematic_feasibility.hpp"

namespace satellite_grasp_planner {
namespace core {

using franka_cartesian_control::core::RobotModel;
using haptic_dmp_learning::core::GraspPointId;
using haptic_dmp_learning::core::kPi;

double referenceWTrans(const haptic_dmp_learning::core::ProDMP& prodmp_template,
                       const std::shared_ptr<RobotModel>& robot, const RobotModel::JointVector& q0,
                       const Eigen::Vector3d& delta_g_demo) {
    if (!robot) throw std::invalid_argument("referenceWTrans: robot is null.");
    robot->update(q0, RobotModel::JointVector::Zero());
    const Eigen::Vector3d p_start = robot->eePosition();
    const Eigen::Quaterniond q_start = robot->eeOrientation();
    const FeasibilityResult kin =
        checkKinematicFeasibility(prodmp_template, p_start + delta_g_demo, q_start, robot, q0);
    return kin.w_trans_final;
}

GraspSelection selectGrasp(const haptic_dmp_learning::core::CubeSatelliteModel& model,
                           const haptic_dmp_learning::core::ProDMP& prodmp_template,
                           const std::shared_ptr<RobotModel>& robot, const RobotModel::JointVector& q0,
                           const SelectionParams& params) {
    if (params.max_rows <= 0) {
        throw std::invalid_argument("selectGrasp: max_rows must be > 0.");
    }
    if (!(params.w_hat_upper_bound > 0.0)) {
        throw std::invalid_argument("selectGrasp: w_hat_upper_bound must be > 0.");
    }
    if (!(params.w_trans_demo > 0.0)) {
        throw std::invalid_argument("selectGrasp: w_trans_demo must be > 0.");
    }
    if (!robot) throw std::invalid_argument("selectGrasp: robot is null.");

    const auto t_start = std::chrono::steady_clock::now();
    const std::vector<CandidateRow> sorted =
        sortedByPartialCost(scanCandidates(model, prodmp_template, params.scan));

    const haptic_dmp_learning::core::GraspCostReferences refs{params.scan.e_v_ref, params.scan.e_g_ref,
                                                              params.w_trans_demo};
    GraspSelection out;

    const int n_rows = static_cast<int>(sorted.size());
    for (int i = 0; i < n_rows; ++i) {
        const CandidateRow& row = sorted[static_cast<std::size_t>(i)];
        const GraspPointId k = row.k;

        // Stopping rule: no remaining row can beat best (see the header).
        if (out.found &&
            row.partial_cost - params.scan.weights.w_m * params.w_hat_upper_bound >=
                out.best.cost.total) {
            out.stopped_by_bound = true;
            out.rows_skipped_by_bound = n_rows - i;
            break;
        }
        if (out.rows_evaluated >= params.max_rows) {
            out.truncated = true;
            break;
        }
        ++out.rows_evaluated;

        std::vector<EvaluatedCandidate> variants;
        auto evalPsi = [&](double psi) {
            EvaluatedCandidate v;
            v.row = row;
            v.psi_rad = psi;
            v.eval = evaluateCandidate(model, k, row.theta_rad, psi, prodmp_template, robot, q0,
                                       params.d_safe_m, params.pos_tol_mm);
            const haptic_dmp_learning::core::GraspCostTerms terms{row.e_v, row.e_g,
                                                                 v.eval.w_trans_final, psi};
            v.cost = haptic_dmp_learning::core::combineCost(terms, refs, params.scan.weights,
                                                            params.psi_tol_rad);
            variants.push_back(v);
        };
        auto anyFeasible = [&]() {
            for (const auto& v : variants) {
                if (v.eval.feasible) return true;
            }
            return false;
        };

        evalPsi(0.0);
        evalPsi(kPi);
        if (!anyFeasible()) {
            evalPsi(params.psi_refine_rad);
            evalPsi(-params.psi_refine_rad);
            evalPsi(kPi + params.psi_refine_rad);
            evalPsi(kPi - params.psi_refine_rad);
        }

        // Feasible variant of minimum total; otherwise the least joint-limit violation
        // (kept, flagged infeasible, for diagnostics).
        const EvaluatedCandidate* chosen = nullptr;
        if (anyFeasible()) {
            for (const auto& v : variants) {
                if (!v.eval.feasible) continue;
                out.max_w_hat_seen = std::max(out.max_w_hat_seen, v.cost.w_hat);
                if (!chosen || v.cost.total < chosen->cost.total) chosen = &v;
            }
        } else {
            for (const auto& v : variants) {
                if (!chosen || v.eval.max_joint_violation_rad < chosen->eval.max_joint_violation_rad) {
                    chosen = &v;
                }
            }
        }
        EvaluatedCandidate keep = *chosen;
        keep.rollouts_used = static_cast<int>(variants.size());
        out.total_rollouts += keep.rollouts_used;
        out.evaluated.push_back(keep);
        if (keep.eval.feasible && (!out.found || keep.cost.total < out.best.cost.total)) {
            out.best = keep;
            out.found = true;
        }
    }
    out.bound_violated = out.max_w_hat_seen > params.w_hat_upper_bound;

    if (out.found) {
        out.goal_position = out.best.row.goal_param;
        out.goal_orientation = haptic_dmp_learning::core::applyRoll(
            model.graspPoseAt(out.best.row.k, out.best.row.theta_rad).orientation_nominal_world,
            out.best.psi_rad);
    }
    out.elapsed_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    return out;
}

}  // namespace core
}  // namespace satellite_grasp_planner
