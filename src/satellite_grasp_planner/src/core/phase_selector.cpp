#include "satellite_grasp_planner/core/phase_selector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

namespace satellite_grasp_planner {
namespace core {

namespace {

/// @brief Linear-interpolation percentile (0-100) of a value set. Used
/// instead of a bare minimum so a single noisy/transient sample along a
/// ~12000-step simulated path cannot dominate the score - see PhaseSelector
/// class docs.
double percentile(std::vector<double> values, double p) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const double rank = (p / 100.0) * static_cast<double>(values.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(rank));
    const size_t hi = static_cast<size_t>(std::ceil(rank));
    if (lo == hi) {
        return values[lo];
    }
    const double frac = rank - static_cast<double>(lo);
    return values[lo] * (1.0 - frac) + values[hi] * frac;
}

/// @brief Constant-rate SLERP with shortest-path hemisphere alignment (same
/// convention as franka_cartesian_control::core::computePoseError).
Eigen::Quaterniond slerpShortestPath(const Eigen::Quaterniond& q0, const Eigen::Quaterniond& q1,
                                      double s) {
    Eigen::Quaterniond target = q1;
    if (q0.coeffs().dot(target.coeffs()) < 0.0) {
        target.coeffs() = -target.coeffs();
    }
    return q0.slerp(s, target).normalized();
}

}  // namespace

Eigen::Vector3d SatelliteRotationModel::transformPointAt(double t,
                                                           const Eigen::Vector3d& point_body) const {
    const double theta = phase0 + omega * t;
    const Eigen::AngleAxisd rot(theta, axis.normalized());
    return center + rot * point_body;
}

Eigen::Quaterniond SatelliteRotationModel::transformOrientationAt(
    double t, const Eigen::Quaterniond& quat_body) const {
    const double theta = phase0 + omega * t;
    const Eigen::AngleAxisd rot(theta, axis.normalized());
    return (Eigen::Quaterniond(rot) * quat_body).normalized();
}

double SatelliteRotationModel::period() const {
    if (std::abs(omega) < 1e-9) {
        return 0.0;
    }
    return 2.0 * M_PI / std::abs(omega);
}

PhaseSelector::PhaseSelector(std::shared_ptr<RobotModel> robot_model,
                              const haptic_dmp_learning::core::ProDMP& prodmp_template)
    : PhaseSelector(std::move(robot_model), prodmp_template, Params()) {}

PhaseSelector::PhaseSelector(std::shared_ptr<RobotModel> robot_model,
                              const haptic_dmp_learning::core::ProDMP& prodmp_template,
                              const Params& params)
    : robot_model_(robot_model),
      prodmp_template_(prodmp_template),
      simulator_(robot_model, params.sim_params),
      params_(params) {}

PhaseSelector::Candidate PhaseSelector::scoreCandidate(
    double t_start, const SatelliteRotationModel& rotation, const Eigen::Vector3d& grasp_pos_body,
    const Eigen::Quaterniond& grasp_quat_body, const RobotModel::JointVector& q0,
    const Eigen::Vector3d& ee_init_position, const Eigen::Quaterniond& ee_init_orientation) const {
    Candidate cand;
    cand.t_start = t_start;

    const double tau = prodmp_template_.tau();
    const double arrival = t_start + tau;
    cand.goal_position = rotation.transformPointAt(arrival, grasp_pos_body);
    cand.goal_orientation = rotation.transformOrientationAt(arrival, grasp_quat_body);

    const double dist_from_base = cand.goal_position.norm();
    if (dist_from_base > params_.max_reach_m) {
        cand.is_feasible = false;
        cand.score = 0.0;
        std::cerr << "[PhaseSelector] Candidate at t_start=" << t_start
                  << "s (goal=[" << cand.goal_position.x() << ", " << cand.goal_position.y()
                  << ", " << cand.goal_position.z() << "], dist=" << dist_from_base
                  << "m) exceeds max_reach_m=" << params_.max_reach_m
                  << "m. Pruned as infeasible without simulation." << std::endl;
        return cand;
    }

    // Copy the fitted template (value type, cheap) and retarget it to this
    // candidate's fixed arrival goal - see class docs for why the goal is
    // fixed per-candidate rather than tracking the moving target mid-approach
    // (that is Level 2).
    haptic_dmp_learning::core::ProDMP prodmp = prodmp_template_;
    prodmp.setRelativeGoal(false);
    prodmp.setInitialConditions(/*init_time=*/0.0, ee_init_position, Eigen::Vector3d::Zero());
    prodmp.setGoal(cand.goal_position);

    const double dt = params_.sim_params.dt;
    const int n_steps = std::max(1, static_cast<int>(std::ceil(tau / dt)));

    std::vector<CartesianSample> trajectory;
    trajectory.reserve(static_cast<size_t>(n_steps));
    for (int i = 0; i < n_steps; ++i) {
        CartesianSample sample;
        sample.position = prodmp.step(dt);
        const double s = std::min(1.0, static_cast<double>(i + 1) * dt / tau);
        sample.orientation = slerpShortestPath(ee_init_orientation, cand.goal_orientation, s);
        // No velocity feedforward - see class docs (matches the
        // feedforward_enabled_=false baseline the score is validated
        // against).
        trajectory.push_back(sample);
    }

    auto steps = simulator_.simulate(trajectory, q0);
    std::vector<double> w_series;
    w_series.reserve(steps.size());
    for (const auto& step : steps) {
        w_series.push_back(step.w_trans);
    }
    cand.score = percentile(w_series, params_.score_percentile);

    return cand;
}

PhaseSelector::Result PhaseSelector::select(const SatelliteRotationModel& rotation,
                                             const Eigen::Vector3d& grasp_pos_body,
                                             const Eigen::Quaterniond& grasp_quat_body,
                                             const RobotModel::JointVector& q0,
                                             const Eigen::Vector3d& ee_init_position,
                                             const Eigen::Quaterniond& ee_init_orientation) const {
    Result result;

    const auto scan_start = std::chrono::steady_clock::now();
    auto budgetExceeded = [&]() {
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - scan_start).count();
        return elapsed > params_.max_wall_time_sec;
    };

    const double period = rotation.period();

    // --- Coarse pass ---
    std::vector<double> coarse_t_starts;
    if (period <= 0.0) {
        // Degenerate omega == 0 case: the phase never changes, so scanning
        // t_start is meaningless - a single evaluation at t_start = 0
        // suffices (see test_phase_selector.cpp for the validation this
        // enables against the 5 known static goals).
        coarse_t_starts.push_back(0.0);
    } else {
        const int n = std::max(1, params_.coarse_candidates);
        coarse_t_starts.reserve(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i) {
            coarse_t_starts.push_back(period * static_cast<double>(i) / static_cast<double>(n));
        }
    }

    std::vector<Candidate> evaluated;
    bool budget_hit = false;

    for (double t_start : coarse_t_starts) {
        if (budgetExceeded()) {
            budget_hit = true;
            break;
        }
        evaluated.push_back(scoreCandidate(t_start, rotation, grasp_pos_body, grasp_quat_body, q0,
                                            ee_init_position, ee_init_orientation));
    }

    // --- Fine pass: refine only around the coarse best, only if not
    // degenerate, budget allows, and at least one feasible candidate was found ---
    if (period > 0.0 && !budget_hit && !evaluated.empty()) {
        const Candidate* coarse_best = nullptr;
        for (const auto& c : evaluated) {
            if (c.is_feasible) {
                if (!coarse_best || c.score > coarse_best->score) {
                    coarse_best = &c;
                }
            }
        }
        if (coarse_best) {
            const double best_t = coarse_best->t_start;
            const double half_window = 0.5 * params_.fine_window_fraction * period;

            const int n_fine = std::max(1, params_.fine_candidates);
            for (int i = 0; i < n_fine; ++i) {
                if (budgetExceeded()) {
                    budget_hit = true;
                    break;
                }
                const double frac =
                    (n_fine == 1) ? 0.5 : static_cast<double>(i) / static_cast<double>(n_fine - 1);
                double t_start = best_t - half_window + frac * (2.0 * half_window);
                // Wrap into [0, period) - the search is over a periodic phase.
                t_start = std::fmod(t_start, period);
                if (t_start < 0.0) {
                    t_start += period;
                }
                evaluated.push_back(scoreCandidate(t_start, rotation, grasp_pos_body, grasp_quat_body,
                                                    q0, ee_init_position, ee_init_orientation));
            }
        }
    }

    if (evaluated.empty()) {
        // Budget was hit before even the first candidate could be scored -
        // fail loud, never return a silent empty/default result.
        result.candidates_evaluated = 0;
        result.budget_exceeded = true;
        result.message = "PhaseSelector: wall-time budget (" +
                          std::to_string(params_.max_wall_time_sec) +
                          "s) exceeded before any candidate could be evaluated.";
        std::cerr << "[PhaseSelector] FAIL: " << result.message << std::endl;
        return result;
    }

    int n_infeasible = 0;
    std::vector<const Candidate*> feasible_ptrs;
    for (const auto& c : evaluated) {
        if (!c.is_feasible) {
            n_infeasible++;
        } else {
            feasible_ptrs.push_back(&c);
        }
    }
    result.candidates_evaluated = static_cast<int>(evaluated.size());
    result.candidates_infeasible = n_infeasible;
    result.budget_exceeded = budget_hit;

    if (feasible_ptrs.empty()) {
        result.below_threshold = true;
        result.message = "PhaseSelector: all " + std::to_string(evaluated.size()) +
                          " candidates were infeasible (exceeded max_reach_m=" +
                          std::to_string(params_.max_reach_m) + "m).";
        std::cerr << "[PhaseSelector] FAIL: " << result.message << std::endl;
        return result;
    }

    const auto best_it = *std::max_element(
        feasible_ptrs.begin(), feasible_ptrs.end(),
        [](const Candidate* a, const Candidate* b) { return a->score < b->score; });

    result.t_start = best_it->t_start;
    result.score = best_it->score;
    result.best_score_lower_bound = result.score * (1.0 - params_.known_branch_uncertainty);
    result.goal_position = best_it->goal_position;
    result.goal_orientation = best_it->goal_orientation;

    // Determine runner-up score among feasible candidates (distinct from best)
    if (feasible_ptrs.size() >= 2) {
        double second_best = 0.0;
        for (const auto* c : feasible_ptrs) {
            if (c == best_it) continue;
            if (c->score > second_best) second_best = c->score;
        }
        result.runner_up_score = second_best;
        if (result.score > 1e-9) {
            result.score_margin = (result.score - result.runner_up_score) / result.score;
            if (result.score_margin < params_.indecision_margin_fraction) {
                result.indecisive_margin = true;
                result.message +=
                    "PhaseSelector: score margin (" + std::to_string(result.score_margin * 100.0) +
                    "%) vs runner-up (" + std::to_string(result.runner_up_score) +
                    ") is below indecision_margin_fraction (" +
                    std::to_string(params_.indecision_margin_fraction * 100.0) +
                    "%) - distinction is within proxy kinematic branch uncertainty. ";
                std::cerr << "[PhaseSelector] INFO: " << result.message << std::endl;
            }
        }
    }

    std::cout << "[PhaseSelector] Best candidate score: " << result.score
              << ", realistic range considering known postural bifurcation: ["
              << result.best_score_lower_bound << ", " << result.score << "]" << std::endl;

    if (budget_hit) {
        result.message += "PhaseSelector: wall-time budget (" +
                           std::to_string(params_.max_wall_time_sec) + "s) exceeded after " +
                           std::to_string(result.candidates_evaluated) +
                           " candidates - returning the best found so far, NOT a fully "
                           "converged scan. ";
        std::cerr << "[PhaseSelector] WARNING: " << result.message << std::endl;
    }

    if (result.score < params_.score_threshold) {
        result.below_threshold = true;
        result.message +=
            "PhaseSelector: best score " + std::to_string(result.score) +
            " is below score_threshold=" + std::to_string(params_.score_threshold) +
            " - Level 1 phase selection alone is likely insufficient for this target/grasp "
            "geometry (every sampled phase passes through poor manipulability); consider "
            "Level 2 trajectory refinement or a different grasp point.";
        std::cerr << "[PhaseSelector] WARNING: " << result.message << std::endl;
    }

    return result;
}

}  // namespace core
}  // namespace satellite_grasp_planner
