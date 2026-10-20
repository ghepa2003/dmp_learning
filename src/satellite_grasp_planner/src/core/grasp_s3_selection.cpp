#include "satellite_grasp_planner/core/grasp_s3_selection.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "haptic_dmp_learning/core/surface_velocity.hpp"
#include "haptic_dmp_learning/core/tcp_roll.hpp"
#include "haptic_dmp_learning/core/math_utils.hpp"
#include "satellite_grasp_planner/core/kinematic_feasibility.hpp"
#include "satellite_grasp_planner/core/manipulability.hpp"

namespace satellite_grasp_planner {
namespace core {

namespace {

using haptic_dmp_learning::core::GraspPointId;

constexpr std::size_t kRankedSize = 20;
constexpr double kDt = 0.005;  // same step as kinematic_feasibility.hpp (dt default)
constexpr double kRolloutViolTolRad = 1e-4;  // same as the max_joint_violation_tol_rad default of checkKinematicFeasibility
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

std::string rejectMaskToString(unsigned mask) {
    std::string s;
    auto add = [&](const char* n) { s += (s.empty() ? "" : ",") + std::string(n); };
    if (mask & kRejectIk) add("ik");
    if (mask & kRejectWHat) add("w_hat");
    if (mask & kRejectMargin) add("margine");
    if (mask & kRejectRTrust) add("r_trust");
    if (mask & kRejectStartGoal) add("start_goal");
    return s.empty() ? "none" : s;
}

/// Circular distance of two thetas, in [0, pi].
double thetaDistRad(double a, double b) {
    const double kPi = haptic_dmp_learning::core::kPi;
    double d = std::fmod(std::abs(a - b), 2.0 * kPi);
    if (d > kPi) d = 2.0 * kPi - d;
    return d;
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
    if (!std::isfinite(p.posthoc_cluster_deg) || p.posthoc_cluster_deg < 0.0) {
        throw std::invalid_argument("selectGraspS3: posthoc_cluster_deg must be finite and >= 0.");
    }
    if (p.force_flip < -1 || p.force_flip > 1) {
        throw std::invalid_argument("selectGraspS3: force_flip must be -1, 0 or 1.");
    }
    (void)parseBranchRule(p.branch_rule);  // throws std::invalid_argument for an unknown value, no fallback
    const S3Mode mode = parseS3Mode(p.mode);  // throws for an unknown value, no fallback
    if (!(std::isfinite(p.kappa_w_mm) && p.kappa_w_mm >= 0.0)) {
        throw std::invalid_argument("selectGraspS3: kappa_w must be finite and >= 0.");
    }
    if (!(std::isfinite(p.kappa_t_mm_s) && p.kappa_t_mm_s >= 0.0)) {
        throw std::invalid_argument("selectGraspS3: kappa_t must be finite and >= 0.");
    }
    if (!(std::isfinite(p.tau_j_mm) && p.tau_j_mm >= 0.0)) {
        throw std::invalid_argument("selectGraspS3: tau_j_mm must be finite and >= 0.");
    }
    if (p.finalists_n < 1) throw std::invalid_argument("selectGraspS3: finalists_n must be >= 1.");
    if (!(std::isfinite(p.min_clearance_mm) && p.min_clearance_mm >= 0.0)) {
        throw std::invalid_argument("selectGraspS3: s1.min_clearance_mm must be finite and >= 0.");
    }
    validateClearanceParams(ClearanceParams{p.clearance_dt_s, p.gripper_open_m});  // s1.clearance_dt_s, s1.gripper_open_m
    if (p.use_clearance && mode != S3Mode::kV2) {
        throw std::invalid_argument(
            "selectGraspS3: s1.use_clearance=true is only defined in s1.mode=v2 (the constraint acts on the v2 finalists).");
    }
    if (!(std::isfinite(p.max_start_goal_deg) && p.max_start_goal_deg >= 0.0 && p.max_start_goal_deg <= 180.0)) {
        throw std::invalid_argument("selectGraspS3: max_start_goal_deg must be finite and in [0, 180].");
    }
    if (mode == S3Mode::kV2 && p.rollout_q_ref.empty()) {
        throw std::invalid_argument(
            "selectGraspS3: mode 'v2' requires rollout_q_ref (s1.rollout_q_ref: 7 values, the controller's nullspace "
            "target); there is no default.");
    }
    if (!p.rollout_q_ref.empty() && p.rollout_q_ref.size() != static_cast<std::size_t>(RobotModel::kNumJoints)) {
        throw std::invalid_argument("selectGraspS3: rollout_q_ref must have exactly 7 values (got " +
                                    std::to_string(p.rollout_q_ref.size()) + ").");
    }
    for (double v : p.rollout_q_ref) {
        if (!std::isfinite(v)) throw std::invalid_argument("selectGraspS3: rollout_q_ref must be finite.");
    }
    if (!p.force && p.force_flip != -1) {
        throw std::invalid_argument("selectGraspS3: force_flip requires force = true.");
    }
    if (p.force) {
        if (p.force_k < 0 || p.force_k > 3) {
            throw std::invalid_argument("selectGraspS3: force_k must be in 0..3.");
        }
        if (!std::isfinite(p.force_theta_deg) || p.force_theta_deg < -180.0 || p.force_theta_deg > 180.0) {
            throw std::invalid_argument("selectGraspS3: force_theta_deg must be finite and in [-180, 180].");
        }
        if (!std::isfinite(p.force_psi_deg) || std::abs(p.force_psi_deg) > 8.0) {
            throw std::invalid_argument("selectGraspS3: |force_psi_deg| must be finite and <= 8.");
        }
    }
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

unsigned s3AprioriRejectMask(const PoseEvaluation& eval, double d_pos_m, const S3Params& params) {
    unsigned mask = 0;
    if (!eval.feasible_basic) mask |= kRejectIk;
    if (!(eval.w_hat >= params.w_hat_min)) mask |= kRejectWHat;
    if (!(eval.margins.min_margin_rad >= params.margin_min_rad)) mask |= kRejectMargin;
    if (!(d_pos_m <= params.r_trust_m)) mask |= kRejectRTrust;
    return mask;
}

S3Mode parseS3Mode(const std::string& name) {
    if (name == "legacy") return S3Mode::kLegacy;
    if (name == "v2") return S3Mode::kV2;
    throw std::invalid_argument("s1.mode: unknown value '" + name + "' (expected 'legacy' or 'v2').");
}

unsigned s3AprioriRejectMaskV2(const PoseEvaluation& eval, double d_pos_m, const S3Params& params) {
    unsigned mask = 0;
    if (!eval.feasible_basic) mask |= kRejectIk;
    if (!(d_pos_m <= params.r_trust_m)) mask |= kRejectRTrust;
    if (params.max_start_goal_deg < 180.0 &&
        eval.dist_to_demo.ori_rad * 180.0 / haptic_dmp_learning::core::kPi > params.max_start_goal_deg) {
        mask |= kRejectStartGoal;
    }
    return mask;
}

double s3CostV2(double d_mm, double w_full, double t_wait_s, const S3Params& params) {
    double w_term = 0.0;
    if (params.use_w && params.kappa_w_mm > 0.0) {
        if (!(w_full > 0.0)) return std::numeric_limits<double>::infinity();
        w_term = params.kappa_w_mm * (-std::log(w_full));
    }
    return d_mm + w_term + params.kappa_t_mm_s * t_wait_s;
}

int s3PickWinnerV2(const std::vector<double>& j, const std::vector<double>& t_wait_s, double tau_j_mm) {
    if (j.size() != t_wait_s.size()) throw std::invalid_argument("s3PickWinnerV2: size mismatch.");
    if (!(std::isfinite(tau_j_mm) && tau_j_mm >= 0.0)) throw std::invalid_argument("s3PickWinnerV2: tau_j_mm must be >= 0.");
    double j_min = std::numeric_limits<double>::infinity();
    for (double v : j) {
        if (std::isfinite(v)) j_min = std::min(j_min, v);
    }
    if (!std::isfinite(j_min)) return -1;
    int best = -1;
    for (std::size_t i = 0; i < j.size(); ++i) {
        if (!std::isfinite(j[i]) || j[i] > j_min + tau_j_mm) continue;
        const std::size_t b = static_cast<std::size_t>(best);
        if (best < 0 || t_wait_s[i] < t_wait_s[b] || (t_wait_s[i] == t_wait_s[b] && j[i] < j[b])) {
            best = static_cast<int>(i);
        }
    }
    return best;
}

bool s3ClearanceRound(const std::vector<double>& j, std::vector<char>& alive, std::vector<char>& done,
                      std::vector<double>& clearance_mm, const std::function<double(std::size_t)>& eval,
                      double min_clearance_mm, double tau_j_mm) {
    if (j.size() != alive.size() || j.size() != done.size() || j.size() != clearance_mm.size()) {
        throw std::invalid_argument("s3ClearanceRound: size mismatch.");
    }
    if (!(std::isfinite(min_clearance_mm) && min_clearance_mm >= 0.0) || !(std::isfinite(tau_j_mm) && tau_j_mm >= 0.0)) {
        throw std::invalid_argument("s3ClearanceRound: min_clearance_mm and tau_j_mm must be finite and >= 0.");
    }
    std::vector<std::size_t> order;
    for (std::size_t i = 0; i < j.size(); ++i) {
        if (alive[i] && std::isfinite(j[i])) order.push_back(i);
    }
    std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return j[a] < j[b]; });
    auto ensure = [&](std::size_t i) {
        if (!done[i]) {
            clearance_mm[i] = eval(i);
            done[i] = 1;
        }
        const bool ok = clearance_mm[i] >= min_clearance_mm;  // NaN -> false
        if (!ok) alive[i] = 0;
        return ok;
    };
    double j_star = 0.0;
    bool have = false;
    for (const std::size_t i : order) {
        if (ensure(i)) {
            j_star = j[i];
            have = true;
            break;
        }
    }
    if (!have) return false;
    for (const std::size_t i : order) {
        if (!alive[i] || done[i]) continue;
        if (j[i] <= j_star + tau_j_mm) ensure(i);  // same band as s3PickWinnerV2: J <= J_min + tau_j
    }
    return true;
}

double s3DPos(const Eigen::Vector3d& position_world, const Eigen::Vector3d& contact_to_end_offset,
              const Eigen::Vector3d& p0, const Eigen::Vector3d& delta_g_demo) {
    const Eigen::Vector3d goal = position_world + contact_to_end_offset;
    return ((goal - p0) - delta_g_demo).norm();
}

S3Result selectGraspS3(const haptic_dmp_learning::core::CubeSatelliteModel& model, RobotModel& robot,
                       const RobotModel::JointVector& q_start, const DemoReference& demo,
                       const Eigen::Vector3d& delta_g_demo, const S3Context& ctx, const S3Params& params) {
    validateS3Params(params);
    validateCtx(params, ctx);
    const BranchRule branch_rule = parseBranchRule(params.branch_rule);
    const bool v2 = parseS3Mode(params.mode) == S3Mode::kV2;
    RobotModel::JointVector q_rollout_ref = RobotModel::JointVector::Zero();
    if (v2) {
        for (int i = 0; i < RobotModel::kNumJoints; ++i) q_rollout_ref(i) = params.rollout_q_ref[static_cast<std::size_t>(i)];
        const RobotModel::JointVector lo = robot.jointLowerLimits();
        const RobotModel::JointVector hi = robot.jointUpperLimits();
        if (!((q_rollout_ref.array() >= lo.array()).all() && (q_rollout_ref.array() <= hi.array()).all())) {
            throw std::invalid_argument("selectGraspS3: rollout_q_ref is outside the URDF joint limits.");
        }
    }
    const double kPi = haptic_dmp_learning::core::kPi;

    robot.update(q_start, RobotModel::JointVector::Zero());
    const Eigen::Vector3d p0 = robot.eePosition();
    const bool use_clearance = v2 && params.use_clearance;  // validateS3Params: use_clearance implies v2
    if (use_clearance && !ctx.clearance_eval) {
        throw std::invalid_argument("selectGraspS3: s1.use_clearance=true needs S3Context::clearance_eval.");
    }
    // Start orientation of the approach trajectory = FK(q_start), as in checkKinematicFeasibility (read only if used).
    const Eigen::Quaterniond quat0 = use_clearance ? robot.eeOrientation() : Eigen::Quaterniond::Identity();

    S3Result res;
    std::vector<S3Candidate> accepted;
    S3Candidate least_bad[4];
    bool have_least[4] = {false, false, false, false};

    auto evalOne = [&](GraspPointId k, double theta, bool refine_stage, double psi, int flip) {
        S3Candidate c;
        const bool logging = ctx.eval_log != nullptr;
        const bool need_extra = logging || v2;  // v2 needs w_full of the static solution (compute_extra)
        const PoseEvaluationDetail det =
            evaluateGraspPoseDetailed(model, k, theta, psi, robot, q_start, demo, params.length_scale_m,
                                      params.angle_scale_rad, params.ik, need_extra, flip, branch_rule);
        c.eval = det.eval;
        c.psi_rad = psi;
        if (v2) c.w_full_static = det.branch[det.selected].w_full;
        ++res.evaluations;
        c.d_pos_m = s3DPos(c.eval.pose.position_world, ctx.contact_to_end_offset, p0, delta_g_demo);
        c.d = s3Distance(c.d_pos_m, c.eval.dist_to_demo.ori_rad, params.length_scale_m,
                         params.orientation_weight, params.angle_scale_rad);
        const unsigned mask =
            v2 ? s3AprioriRejectMaskV2(c.eval, c.d_pos_m, params) : s3AprioriRejectMask(c.eval, c.d_pos_m, params);
        c.reject_mask = mask;
        if (mask & kRejectIk) {
            c.reject_reason = "ik";
        } else if (mask & kRejectWHat) {
            c.reject_reason = "w_hat";
        } else if (mask & kRejectMargin) {
            c.reject_reason = "margine";
        } else if (mask & kRejectRTrust) {
            c.reject_reason = "r_trust";
        } else if (mask & kRejectStartGoal) {
            c.reject_reason = "start_goal";
        } else {
            c.passes_constraints = true;
            res.max_w_hat_passing = std::max(res.max_w_hat_passing, c.eval.w_hat);
            c.wait_s = planLaunch(ctx.snapshot, ctx.t_now_s, theta, ctx.tau_launch_s, ctx.min_delay_s).delay_s;
            c.cost = c.d + params.epsilon_wait_per_s * c.wait_s;
        }
        if (logging) {
            S3EvalRecord r;
            r.k = k;
            r.theta_rad = theta;
            r.psi_rad = psi;
            r.refine_stage = refine_stage;
            r.reject_mask = mask;
            r.passes_constraints = c.passes_constraints;
            r.wrist_flip = c.eval.wrist_flip;
            r.d_pos_m = c.d_pos_m;
            r.d = c.d;
            r.wait_s = c.passes_constraints
                           ? c.wait_s
                           : planLaunch(ctx.snapshot, ctx.t_now_s, theta, ctx.tau_launch_s, ctx.min_delay_s).delay_s;
            r.cost_current = c.d + params.epsilon_wait_per_s * r.wait_s;
            r.w_hat = c.eval.w_hat;
            r.w_trans = c.eval.w_trans;
            r.margin_rad = c.eval.margins.min_margin_rad;
            r.branch[0] = det.branch[0];
            r.branch[1] = det.branch[1];
            ctx.eval_log->push_back(r);
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

    res.forced = params.force;
    if (params.force) {
        // Test mode: only this candidate; everything after the scans (ranking, post-hoc, outputs) is unchanged.
        const GraspPointId kf = kAllK[params.force_k];
        const double theta_f = params.force_theta_deg * kPi / 180.0;
        const double psi_f = params.force_psi_deg * kPi / 180.0;
        S3Candidate c = evalOne(kf, theta_f, false, psi_f, params.force_flip);
        if (c.passes_constraints) {
            accepted.push_back(c);
        } else if (params.force_ignore_constraints) {
            // evalOne computes wait/cost only for passing candidates: same formulas, forced theta.
            c.wait_s = planLaunch(ctx.snapshot, ctx.t_now_s, theta_f, ctx.tau_launch_s, ctx.min_delay_s).delay_s;
            c.cost = c.d + params.epsilon_wait_per_s * c.wait_s;
            res.forced_violations = "a-priori: " + rejectMaskToString(c.reject_mask);
            accepted.push_back(c);
        } else {
            res.note = "FORCED candidate rejected a priori (force_ignore_constraints=false, force_flip=" +
                       std::to_string(params.force_flip) + "): " +
                       rejectMaskToString(c.reject_mask);
        }
    }

    for (GraspPointId k : kAllK) {
        if (params.force) break;
        S3Candidate best;
        bool have_best = false;
        for (int i = 0; i < n_theta; ++i) {
            const double theta = -kPi + i * step;
            S3Candidate c = evalOne(k, theta, false, 0.0, -1);
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
            S3Candidate c = evalOne(k, theta_best + j * refine, true, 0.0, -1);
            if (c.passes_constraints) accepted.push_back(c);
        }
    }

    std::stable_sort(accepted.begin(), accepted.end(), lessCost);

    // ---- post-hoc checks, best first ----
    std::unique_ptr<haptic_dmp_learning::core::ProDMP> prodmp_at_contact;
    if (params.posthoc_velocity_check) {
        // Support goal p0 + delta_g_demo, stepped to the contact instant.
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
    const double cluster_rad = params.posthoc_cluster_deg * kPi / 180.0;
    std::vector<std::pair<int, double>> checked;  // (k, theta) of the candidates already examined
    auto inCluster = [&](const S3Candidate& c) {
        for (const auto& kt : checked) {
            if (kt.first != static_cast<int>(c.eval.k)) continue;
            double dth = std::fmod(std::abs(c.eval.theta_rad - kt.second), 2.0 * kPi);
            if (dth > kPi) dth = 2.0 * kPi - dth;
            if (dth < cluster_rad) return true;
        }
        return false;
    };
    for (std::size_t i = 0; !v2 && i < accepted.size() && chosen_idx < 0 && res.posthoc_checks < params.max_posthoc_candidates;
         ++i) {
        S3Candidate& c = accepted[i];
        if (inCluster(c)) continue;
        checked.emplace_back(static_cast<int>(c.eval.k), c.eval.theta_rad);
        ++res.posthoc_checks;
        S3PosthocEntry entry;
        entry.k = c.eval.k;
        entry.theta_rad = c.eval.theta_rad;
        entry.wait_s = c.wait_s;
        double dv = 0.0;
        if (params.posthoc_velocity_check) {
            const Eigen::Vector3d goal = c.eval.pose.position_world + ctx.contact_to_end_offset;
            const Eigen::Vector3d v_ee = prodmp_at_contact->velocityForCandidateGoal(goal);
            const Eigen::Vector3d v_t = haptic_dmp_learning::core::surfaceVelocity(
                model, c.eval.k, c.eval.theta_rad, ctx.snapshot.omega_rad_s);
            dv = (v_t - v_ee).norm();
            entry.v_ee = v_ee;
            entry.v_target = v_t;
            entry.dv_m_s = dv;
            if (!(dv <= params.delta_v_max_m_s)) {
                std::ostringstream m;
                m << "velocita' relativa " << dv << " m/s > " << params.delta_v_max_m_s;
                c.reject_reason = m.str();
                entry.reject_reason = c.reject_reason;
                res.posthoc_log.push_back(entry);
                continue;
            }
        }
        double margin = 0.0;
        if (params.posthoc_path_check) {
            // applyRoll(qn, psi + (flip ? pi : 0)): rolls about the same axis (z_tcp) commute.
            const Eigen::Quaterniond orient = goalOrientation(c.eval);
            const FeasibilityResult fr = checkKinematicFeasibility(
                *ctx.prodmp_template, c.eval.pose.position_world, orient, robot_ptr, q_start);
            entry.path_checked = true;
            entry.path_ok = fr.feasible;
            if (!fr.feasible) {
                std::ostringstream m;
                m << "percorso: violazione limiti " << fr.max_joint_violation_rad << " rad (giunto " << fr.worst_joint
                  << ")";
                c.reject_reason = m.str();
                entry.reject_reason = c.reject_reason;
                res.posthoc_log.push_back(entry);
                continue;
            }
            margin = fr.min_joint_margin_rad;
            entry.min_margin_rad = margin;
        }
        entry.accepted = true;
        res.posthoc_log.push_back(entry);
        chosen_idx = static_cast<int>(i);
        res.posthoc_dv_m_s = dv;
        res.posthoc_min_margin_rad = margin;
    }

    bool clearance_no_candidate = false;
    std::string clearance_note;
    if (v2) {
        const double kRad2Deg = 180.0 / kPi;

        // ---- v2 phase 2: finalists = best (J_static) of each (k, wrist branch, theta cluster), at most finalists_n ----
        struct StaticItem {
            int idx;
            double d_mm;
            double j_static;
        };
        std::vector<StaticItem> order;
        order.reserve(accepted.size());
        for (std::size_t i = 0; i < accepted.size(); ++i) {
            const double d_mm = accepted[i].d * params.length_scale_m * 1000.0;  // = d_pos in mm when orientation_weight = 0
            order.push_back({static_cast<int>(i), d_mm, s3CostV2(d_mm, accepted[i].w_full_static, 0.0, params)});
        }
        std::stable_sort(order.begin(), order.end(), [&](const StaticItem& a, const StaticItem& b) {
            if (a.j_static != b.j_static) return a.j_static < b.j_static;
            return lessCost(accepted[static_cast<std::size_t>(a.idx)], accepted[static_cast<std::size_t>(b.idx)]);
        });
        std::vector<StaticItem> groups;  // representatives, in J_static order
        for (const StaticItem& it : order) {
            if (groups.size() >= static_cast<std::size_t>(params.finalists_n)) break;
            const S3Candidate& c = accepted[static_cast<std::size_t>(it.idx)];
            bool dup = false;
            for (const StaticItem& g : groups) {
                const S3Candidate& r = accepted[static_cast<std::size_t>(g.idx)];
                if (r.eval.k != c.eval.k || r.eval.wrist_flip != c.eval.wrist_flip) continue;
                if (thetaDistRad(c.eval.theta_rad, r.eval.theta_rad) < cluster_rad) {
                    dup = true;
                    break;
                }
            }
            if (!dup) groups.push_back(it);
        }

        // ---- v2 phase 3: rollout of every finalist with nullspace target rollout_q_ref ----
        std::vector<S3FinalistRecord> fin;
        fin.reserve(groups.size());
        const RobotModel::JointVector zero_dq = RobotModel::JointVector::Zero();
        for (const StaticItem& g : groups) {
            const S3Candidate& c = accepted[static_cast<std::size_t>(g.idx)];
            S3FinalistRecord rec;
            rec.cand = c;
            rec.accepted_index = g.idx;
            rec.d_mm = g.d_mm;
            rec.w_static = c.w_full_static;
            rec.j_static = g.j_static;
            rec.t_wait_s = c.wait_s;
            rec.start_goal_deg = c.eval.dist_to_demo.ori_rad * kRad2Deg;
            const Eigen::Quaterniond orient = goalOrientation(c.eval);
            const FeasibilityResult fr = checkKinematicFeasibility(
                *ctx.prodmp_template, c.eval.pose.position_world, orient, robot_ptr, q_start, kDt, kRolloutViolTolRad,
                q_rollout_ref);
            rec.rollout_done = true;
            rec.rollout_feasible = fr.feasible;
            rec.rollout_max_violation_rad = fr.max_joint_violation_rad;
            rec.rollout_min_margin_rad = fr.min_joint_margin_rad;
            rec.rollout_pos_err_mm = fr.pos_err_final_mm;
            robot.update(fr.q_final, zero_dq);  // w_eval = w_full(q_final of the rollout)
            rec.rollout_ori_err_deg =
                se3Distance(robot.eePosition(), robot.eeOrientation(), c.eval.pose.position_world, orient, 1.0, 1.0)
                    .ori_rad * kRad2Deg;
            rec.w_rollout = fullManipulability(robot.jacobian()).w_full;
            if (fr.feasible) {
                rec.j = s3CostV2(rec.d_mm, rec.w_rollout, rec.t_wait_s, params);
                rec.cand.cost = rec.j;
                accepted[static_cast<std::size_t>(g.idx)].cost = rec.j;
            }
            fin.push_back(rec);
        }

        // ---- v2 tie-break (smallest t_wait within J_min + tau_j) + post-hoc velocity check, next best on failure ----
        std::vector<char> alive(fin.size(), 0);
        for (std::size_t i = 0; i < fin.size(); ++i) alive[i] = fin[i].rollout_feasible ? 1 : 0;

        // Clearance constraint (S3Params::use_clearance): evaluated lazily, see s3ClearanceRound. Off: none of this runs.
        std::vector<char> clr_done(fin.size(), 0);
        std::vector<double> clr_mm(fin.size(), 0.0);
        bool clearance_any_admissible = false;
        bool any_rollout_feasible = false;
        for (const auto& f : fin) any_rollout_feasible = any_rollout_feasible || f.rollout_feasible;
        auto clearanceOf = [&](std::size_t i) -> double {
            S3FinalistRecord& f = fin[i];
            const S3Candidate& cand = accepted[static_cast<std::size_t>(f.accepted_index)];
            const auto t0 = std::chrono::steady_clock::now();
            // Executor-faithful trajectory: goal = grasp position + contact_to_end_offset (grasp_s3_launch.cpp), orientation
            // of the chosen branch, start = FK(q_start), times as the executor publishes them (t = 0 first).
            const std::vector<ApproachSample> traj =
                timedApproachTrajectory(*ctx.prodmp_template, p0, quat0,
                                        cand.eval.pose.position_world + ctx.contact_to_end_offset,
                                        goalOrientation(cand.eval), kDt);
            ClearanceQuery q;
            q.trajectory = &traj;
            q.tau_s = ctx.prodmp_template->tau();
            q.theta_star_rad = cand.eval.theta_rad;
            q.omega_rad_s = ctx.snapshot.omega_rad_s;
            q.center_world = ctx.snapshot.center;
            q.axis_world = ctx.snapshot.axis;
            q.k = static_cast<int>(cand.eval.k);
            q.wrist_flip = cand.eval.wrist_flip;
            const ClearanceResult cr = ctx.clearance_eval(q);
            res.clearance_stage_ms +=
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            f.clearance_done = true;
            f.clearance_mm = cr.min_mm_no_tail;
            f.clearance_pair = cr.pair_no_tail.str();
            f.clearance_t_rel_s = cr.t_rel_no_tail_s;
            f.clearance_full_mm = cr.min_mm;
            f.clearance_rejected = !(cr.min_mm_no_tail >= params.min_clearance_mm);
            return cr.min_mm_no_tail;
        };
        while (chosen_idx < 0) {
            if (use_clearance && !params.force) {
                std::vector<double> js_all(fin.size(), std::numeric_limits<double>::infinity());
                for (std::size_t i = 0; i < fin.size(); ++i) {
                    if (fin[i].rollout_feasible) js_all[i] = fin[i].j;
                }
                if (s3ClearanceRound(js_all, alive, clr_done, clr_mm, clearanceOf, params.min_clearance_mm, params.tau_j_mm)) {
                    clearance_any_admissible = true;
                }
            }
            std::vector<std::size_t> pool;
            std::vector<double> js, ts;
            for (std::size_t i = 0; i < fin.size(); ++i) {
                if (!alive[i]) continue;
                pool.push_back(i);
                js.push_back(fin[i].j);
                ts.push_back(fin[i].t_wait_s);
            }
            const int pick = s3PickWinnerV2(js, ts, params.tau_j_mm);
            if (pick < 0) break;
            const std::size_t fi = pool[static_cast<std::size_t>(pick)];
            S3FinalistRecord& rec = fin[fi];
            S3Candidate& c = accepted[static_cast<std::size_t>(rec.accepted_index)];
            if (use_clearance && params.force && !clr_done[fi]) {  // forced: the constraint never discards, it warns
                clr_mm[fi] = clearanceOf(fi);
                clr_done[fi] = 1;
                if (rec.clearance_rejected) {
                    std::ostringstream w;
                    w << "WARN s1.use_clearance: FORCED candidate k=" << static_cast<int>(c.eval.k) << " theta="
                      << c.eval.theta_rad * kRad2Deg << " deg has clearance " << rec.clearance_mm << " mm < "
                      << params.min_clearance_mm << " mm (" << rec.clearance_pair << " at t_rel " << rec.clearance_t_rel_s
                      << " s); kept because force is on";
                    res.warnings.push_back(w.str());
                }
            }
            if (params.max_posthoc_candidates == 0) {  // no post-hoc stage
                chosen_idx = rec.accepted_index;
                rec.winner = true;
                break;
            }
            if (res.posthoc_checks >= params.max_posthoc_candidates) break;
            ++res.posthoc_checks;
            S3PosthocEntry entry;
            entry.k = c.eval.k;
            entry.theta_rad = c.eval.theta_rad;
            entry.wait_s = c.wait_s;
            double dv = 0.0;
            if (params.posthoc_velocity_check) {
                const Eigen::Vector3d goal = c.eval.pose.position_world + ctx.contact_to_end_offset;
                const Eigen::Vector3d v_ee = prodmp_at_contact->velocityForCandidateGoal(goal);
                const Eigen::Vector3d v_t = haptic_dmp_learning::core::surfaceVelocity(
                    model, c.eval.k, c.eval.theta_rad, ctx.snapshot.omega_rad_s);
                dv = (v_t - v_ee).norm();
                entry.v_ee = v_ee;
                entry.v_target = v_t;
                entry.dv_m_s = dv;
                if (!(dv <= params.delta_v_max_m_s)) {
                    std::ostringstream m;
                    m << "velocita' relativa " << dv << " m/s > " << params.delta_v_max_m_s;
                    c.reject_reason = m.str();
                    entry.reject_reason = c.reject_reason;
                    res.posthoc_log.push_back(entry);
                    alive[fi] = 0;
                    continue;
                }
            }
            entry.path_checked = true;  // the rollout above is the path check
            entry.path_ok = true;
            entry.min_margin_rad = rec.rollout_min_margin_rad;
            entry.accepted = true;
            res.posthoc_log.push_back(entry);
            chosen_idx = rec.accepted_index;
            rec.winner = true;
            res.posthoc_dv_m_s = dv;
            res.posthoc_min_margin_rad = rec.rollout_min_margin_rad;
        }
        res.v2_finalists = fin;
        if (use_clearance && !params.force && chosen_idx < 0 && any_rollout_feasible && !clearance_any_admissible) {
            clearance_no_candidate = true;
            const S3FinalistRecord* best = nullptr;
            for (const auto& f : fin) {
                if (f.clearance_done && (!best || f.clearance_mm > best->clearance_mm)) best = &f;
            }
            std::ostringstream m;
            m << "v2 clearance: no finalist has clearance >= " << params.min_clearance_mm << " mm ("
              << res.v2_finalists.size() << " finalists)";
            if (best) {
                m << "; best clearance " << best->clearance_mm << " mm for k=" << static_cast<int>(best->cand.eval.k)
                  << " branch " << (best->cand.eval.wrist_flip ? 'B' : 'A') << " theta="
                  << best->cand.eval.theta_rad * kRad2Deg << " deg (" << best->clearance_pair << " at t_rel "
                  << best->clearance_t_rel_s << " s)";
            }
            clearance_note = m.str();
        }
        if (chosen_idx < 0 && params.force && params.force_ignore_constraints && !accepted.empty()) {
            chosen_idx = 0;  // forced + ignore: the failure is reported, not obeyed (as in legacy)
            res.forced_violations += std::string(res.forced_violations.empty() ? "" : "; ") +
                                     "v2: no finalist passed the rollout / post-hoc checks";
        }
    }

    if (params.force && params.force_ignore_constraints && chosen_idx < 0 && !accepted.empty() &&
        res.posthoc_checks > 0) {
        // Forced + ignore: the post-hoc failure is reported, not obeyed.
        chosen_idx = 0;
        const S3PosthocEntry& e = res.posthoc_log.back();
        res.posthoc_dv_m_s = e.dv_m_s;
        res.posthoc_min_margin_rad = e.min_margin_rad;
        res.forced_violations += std::string(res.forced_violations.empty() ? "" : "; ") + "post-hoc: " + e.reject_reason;
    }
    if (!v2 && params.max_posthoc_candidates == 0 && !accepted.empty()) chosen_idx = 0;  // max_posthoc_candidates == 0: no post-hoc stage

    // ---- outputs ----
    const std::size_t n_ranked = std::min(kRankedSize, accepted.size());
    res.ranked.assign(accepted.begin(), accepted.begin() + static_cast<std::ptrdiff_t>(n_ranked));
    if (v2) {  // finalists that passed the rollout, by J
        std::vector<const S3FinalistRecord*> fs;
        for (const auto& f : res.v2_finalists) {
            if (f.rollout_feasible) fs.push_back(&f);
        }
        std::stable_sort(fs.begin(), fs.end(), [](const S3FinalistRecord* a, const S3FinalistRecord* b) { return a->j < b->j; });
        res.ranked.clear();
        for (const auto* f : fs) {
            if (res.ranked.size() >= kRankedSize) break;
            res.ranked.push_back(accepted[static_cast<std::size_t>(f->accepted_index)]);
        }
    }
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

    if (clearance_no_candidate) {
        res.status = S3Result::kNoCandidate;
        res.note = clearance_note;
    } else if (accepted.empty()) {
        res.status = S3Result::kNoCandidate;
        if (!params.force) {
            res.note = v2 ? "no candidate satisfies the v2 constraints (ik, d_pos <= r_trust, start_goal <= max_start_goal_deg)"
                          : "no candidate satisfies the constraints (ik, w_hat >= w_hat_min, margin >= margin_min, d_pos <= r_trust)";
        }
    } else if (chosen_idx < 0) {
        res.status = S3Result::kPosthocFailed;
        std::ostringstream m;
        m << "all " << res.posthoc_checks << " post-hoc candidates failed (of " << accepted.size()
          << " constraint-passing); first: " << accepted.front().reject_reason;
        res.note = (params.force ? "FORCED candidate: " : "") + m.str();
        if (v2) {
            int n_feasible = 0;
            for (const auto& f : res.v2_finalists) n_feasible += f.rollout_feasible ? 1 : 0;
            std::ostringstream m2;
            m2 << "v2: no finalist passed the rollout and the post-hoc checks (" << res.v2_finalists.size()
               << " finalists, " << n_feasible << " rollout-feasible, " << res.posthoc_checks << " post-hoc checks)";
            res.note = (params.force ? "FORCED candidate: " : "") + m2.str();
        }
    } else {
        res.status = S3Result::kSelected;
        res.chosen = accepted[static_cast<std::size_t>(chosen_idx)];
    }
    return res;
}

}  // namespace core
}  // namespace satellite_grasp_planner
