#include "satellite_grasp_planner/core/grasp_s3_selection.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>

#include "haptic_dmp_learning/core/grasp_cost.hpp"
#include "haptic_dmp_learning/core/grasp_roll.hpp"
#include "haptic_dmp_learning/core/math_utils.hpp"
#include "satellite_grasp_planner/core/kinematic_feasibility.hpp"

namespace satellite_grasp_planner {
namespace core {

namespace {

using haptic_dmp_learning::core::GraspPointId;

constexpr std::size_t kRankedSize = 20;
constexpr double kDt = 0.005;  // same step as scanCandidates (candidate_scan.cpp)
constexpr GraspPointId kAllK[4] = {GraspPointId::kP0, GraspPointId::kP90, GraspPointId::kP180,
                                   GraspPointId::kP270};

void validateCtx(const S3Params& p, const S3Context& ctx) {
    auto pos = [](double v) { return std::isfinite(v) && v > 0.0; };
    (void)p;
    if (!ctx.prodmp_template) throw std::invalid_argument("selectGraspS3: ctx.prodmp_template is null.");
    if (!pos(ctx.tau_launch_s)) throw std::invalid_argument("selectGraspS3: ctx.tau_launch_s must be > 0.");
    if (ctx.snapshot.omega_rad_s == 0.0) throw std::invalid_argument("selectGraspS3: omega is zero.");
    if (!std::isfinite(ctx.min_delay_s) || ctx.min_delay_s < 0.0) {
        throw std::invalid_argument("selectGraspS3: min_delay_s must be finite and >= 0.");
    }
}

bool lessCost(const S3Candidate& a, const S3Candidate& b) {
    if (a.cost != b.cost) return a.cost < b.cost;
    if (a.eval.k != b.eval.k) return static_cast<int>(a.eval.k) < static_cast<int>(b.eval.k);
    return a.eval.theta_rad < b.eval.theta_rad;
}

}  // namespace

void validateS3Params(const S3Params& p) {
    auto pos = [](double v) { return std::isfinite(v) && v > 0.0; };
    if (!pos(p.length_scale_m) || !pos(p.angle_scale_rad) || !pos(p.theta_step_deg) ||
        !pos(p.theta_refine_step_deg) || !pos(p.r_trust_m) || !pos(p.delta_v_max_m_s)) {
        throw std::invalid_argument("selectGraspS3: scales, steps, r_trust_m and delta_v_max_m_s must be finite and > 0.");
    }
    if (!(p.orientation_weight >= 0.0) || !std::isfinite(p.orientation_weight) ||
        !(p.epsilon_wait_per_s >= 0.0) || !std::isfinite(p.epsilon_wait_per_s)) {
        throw std::invalid_argument("selectGraspS3: orientation_weight and epsilon_wait_per_s must be finite and >= 0.");
    }
    if (!std::isfinite(p.w_hat_min) || !std::isfinite(p.margin_min_rad)) {
        throw std::invalid_argument("selectGraspS3: w_hat_min and margin_min_rad must be finite.");
    }
    if (p.max_posthoc_candidates < 0) throw std::invalid_argument("selectGraspS3: max_posthoc_candidates < 0.");
    if (!(p.ik.max_iter > 0) || !pos(p.ik.pos_tol_m) || !pos(p.ik.ori_tol_rad) || !pos(p.ik.max_step_rad) ||
        !pos(p.ik.damping) || !(std::isfinite(p.ik.nullspace_gain) && p.ik.nullspace_gain >= 0.0)) {
        throw std::invalid_argument("selectGraspS3: invalid IkParams.");
    }
}

double s3Distance(double d_pos_m, double ori_rad, double length_scale_m, double orientation_weight,
                  double angle_scale_rad) {
    if (!(std::isfinite(length_scale_m) && length_scale_m > 0.0 && std::isfinite(angle_scale_rad) &&
          angle_scale_rad > 0.0 && std::isfinite(orientation_weight) && orientation_weight >= 0.0)) {
        throw std::invalid_argument("s3Distance: scales must be finite and > 0, weight finite and >= 0.");
    }
    const double a = d_pos_m / length_scale_m;
    const double b = ori_rad / angle_scale_rad;
    return std::sqrt(a * a + orientation_weight * b * b);
}

S3Result selectGraspS3(const haptic_dmp_learning::core::CubeSatelliteModel& model, RobotModel& robot,
                       const RobotModel::JointVector& q_start, const DemoReference& demo,
                       const Eigen::Vector3d& delta_g_demo, const S3Context& ctx, const S3Params& params) {
    validateS3Params(params);
    validateCtx(params, ctx);
    const double kPi = haptic_dmp_learning::core::kPi;

    robot.update(q_start, RobotModel::JointVector::Zero());
    const Eigen::Vector3d p0 = robot.eePosition();

    S3Result res;
    std::vector<S3Candidate> accepted;
    S3Candidate least_bad[4];
    bool have_least[4] = {false, false, false, false};

    auto evalOne = [&](GraspPointId k, double theta) {
        S3Candidate c;
        c.eval = evaluateGraspPose(model, k, theta, robot, q_start, demo, params.length_scale_m,
                                   params.angle_scale_rad, params.ik);
        ++res.evaluations;
        const Eigen::Vector3d goal = c.eval.pose.position_world + ctx.contact_to_end_offset;
        c.d_pos_m = ((goal - p0) - delta_g_demo).norm();
        c.d = s3Distance(c.d_pos_m, c.eval.dist_to_demo.ori_rad, params.length_scale_m,
                         params.orientation_weight, params.angle_scale_rad);
        if (!c.eval.feasible_basic) {
            c.reject_reason = "ik";
        } else if (!(c.eval.w_hat >= params.w_hat_min)) {
            c.reject_reason = "w_hat";
        } else if (!(c.eval.margins.min_margin_rad >= params.margin_min_rad)) {
            c.reject_reason = "margine";
        } else if (!(c.d_pos_m <= params.r_trust_m)) {
            c.reject_reason = "r_trust";
        } else {
            c.passes_constraints = true;
            res.max_w_hat_passing = std::max(res.max_w_hat_passing, c.eval.w_hat);
            c.wait_s = planLaunch(ctx.snapshot, ctx.t_now_s, theta, ctx.tau_launch_s, ctx.min_delay_s).delay_s;
            c.cost = c.d + params.epsilon_wait_per_s * c.wait_s;
        }
        const int ki = static_cast<int>(k);
        if (!c.passes_constraints && (!have_least[ki] || c.d_pos_m < least_bad[ki].d_pos_m)) {
            least_bad[ki] = c;
            have_least[ki] = true;
        }
        return c;
    };

    const double step = params.theta_step_deg * kPi / 180.0;
    const double refine = params.theta_refine_step_deg * kPi / 180.0;
    const int n_theta = static_cast<int>(std::ceil(2.0 * kPi / step - 1e-9));  // [-pi, pi)
    const int n_refine = static_cast<int>(std::floor(step / refine + 1e-9));

    for (GraspPointId k : kAllK) {
        S3Candidate best;
        bool have_best = false;
        for (int i = 0; i < n_theta; ++i) {
            const double theta = -kPi + i * step;
            S3Candidate c = evalOne(k, theta);
            if (!c.passes_constraints) continue;
            accepted.push_back(c);
            if (!have_best || lessCost(c, best)) {
                best = c;
                have_best = true;
            }
        }
        if (!have_best) continue;
        const double theta_best = best.eval.theta_rad;
        for (int j = -n_refine; j <= n_refine; ++j) {
            if (j == 0) continue;
            S3Candidate c = evalOne(k, theta_best + j * refine);
            if (c.passes_constraints) accepted.push_back(c);
        }
    }

    std::stable_sort(accepted.begin(), accepted.end(), lessCost);

    // ---- post-hoc checks, best first ----
    std::unique_ptr<haptic_dmp_learning::core::ProDMP> prodmp_at_contact;
    if (params.posthoc_velocity_check) {
        // Same preparation as scanCandidates(): support goal p0 + delta_g_demo, stepped to the contact instant.
        prodmp_at_contact = std::make_unique<haptic_dmp_learning::core::ProDMP>(*ctx.prodmp_template);
        prodmp_at_contact->setRelativeGoal(false);
        prodmp_at_contact->setInitialConditions(0.0, p0, Eigen::Vector3d::Zero());
        prodmp_at_contact->setGoal(p0 + delta_g_demo);
        const double tau_contact = ctx.tau_contact_s > 0.0 ? ctx.tau_contact_s : ctx.prodmp_template->tau();
        const int n_steps = std::max(1, static_cast<int>(std::ceil(tau_contact / kDt - 1e-9)));
        for (int i = 0; i < n_steps; ++i) prodmp_at_contact->step(kDt);
    }
    const std::shared_ptr<RobotModel> robot_ptr(&robot, [](RobotModel*) {});  // non-owning

    int chosen_idx = -1;
    const int n_check = std::min(static_cast<int>(accepted.size()), params.max_posthoc_candidates);
    for (int i = 0; i < n_check && chosen_idx < 0; ++i) {
        S3Candidate& c = accepted[static_cast<std::size_t>(i)];
        ++res.posthoc_checks;
        double dv = 0.0;
        if (params.posthoc_velocity_check) {
            const Eigen::Vector3d goal = c.eval.pose.position_world + ctx.contact_to_end_offset;
            const Eigen::Vector3d v_ee = prodmp_at_contact->velocityForCandidateGoal(goal);
            const Eigen::Vector3d v_t = haptic_dmp_learning::core::surfaceVelocity(
                model, c.eval.k, c.eval.theta_rad, ctx.snapshot.omega_rad_s);
            dv = (v_t - v_ee).norm();
            if (!(dv <= params.delta_v_max_m_s)) {
                std::ostringstream m;
                m << "velocita' relativa " << dv << " m/s > " << params.delta_v_max_m_s;
                c.reject_reason = m.str();
                continue;
            }
        }
        double margin = 0.0;
        if (params.posthoc_path_check) {
            const Eigen::Quaterniond& qn = c.eval.pose.orientation_nominal_world;
            const Eigen::Quaterniond orient =
                c.eval.wrist_flip ? haptic_dmp_learning::core::applyRoll(qn, kPi) : qn;
            const FeasibilityResult fr = checkKinematicFeasibility(
                *ctx.prodmp_template, c.eval.pose.position_world, orient, robot_ptr, q_start);
            if (!fr.feasible) {
                std::ostringstream m;
                m << "percorso: violazione limiti " << fr.max_joint_violation_rad << " rad (giunto " << fr.worst_joint
                  << ")";
                c.reject_reason = m.str();
                continue;
            }
            margin = fr.min_joint_margin_rad;
        }
        chosen_idx = i;
        res.posthoc_dv_m_s = dv;
        res.posthoc_min_margin_rad = margin;
    }

    if (n_check == 0 && !accepted.empty()) chosen_idx = 0;  // max_posthoc_candidates == 0: no post-hoc stage

    // ---- outputs ----
    const std::size_t n_ranked = std::min(kRankedSize, accepted.size());
    res.ranked.assign(accepted.begin(), accepted.begin() + static_cast<std::ptrdiff_t>(n_ranked));
    for (int ki = 0; ki < 4; ++ki) {
        bool found = false;
        for (const auto& c : accepted) {
            if (static_cast<int>(c.eval.k) == ki) {
                res.best_per_k[static_cast<std::size_t>(ki)] = c;
                found = true;
                break;
            }
        }
        if (!found && have_least[ki]) res.best_per_k[static_cast<std::size_t>(ki)] = least_bad[ki];
        if (!found && !have_least[ki]) res.best_per_k[static_cast<std::size_t>(ki)].eval.k = kAllK[ki];
    }

    if (accepted.empty()) {
        res.status = S3Result::kNoCandidate;
        res.note = "no candidate satisfies the constraints (ik, w_hat >= w_hat_min, margin >= margin_min, d_pos <= r_trust)";
    } else if (chosen_idx < 0) {
        res.status = S3Result::kPosthocFailed;
        std::ostringstream m;
        m << "all " << res.posthoc_checks << " post-hoc candidates failed (of " << accepted.size()
          << " constraint-passing); first: " << accepted.front().reject_reason;
        res.note = m.str();
    } else {
        res.status = S3Result::kSelected;
        res.chosen = accepted[static_cast<std::size_t>(chosen_idx)];
    }
    return res;
}

}  // namespace core
}  // namespace satellite_grasp_planner
