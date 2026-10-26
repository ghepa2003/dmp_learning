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
#include "haptic_dmp_learning/core/math_utils.hpp"
#include "satellite_grasp_planner/core/follow_margin.hpp"
#include "satellite_grasp_planner/core/kinematic_feasibility.hpp"
#include "satellite_grasp_planner/core/manipulability.hpp"

namespace satellite_grasp_planner {
namespace core {

namespace {

using haptic_dmp_learning::core::GraspPointId;

constexpr std::size_t kRankedSize = 20;
constexpr GraspPointId kAllK[4] = {GraspPointId::kP0, GraspPointId::kP90, GraspPointId::kP180,
                                   GraspPointId::kP270};

/// Finite and strictly positive.
bool isFinitePositive(double v) { return std::isfinite(v) && v > 0.0; }

void validateCtx(const S3Context& ctx) {
  if (!ctx.prodmp_template) throw std::invalid_argument("selectGraspS3: ctx.prodmp_template is null.");
  if (!isFinitePositive(ctx.tau_launch_s)) throw std::invalid_argument("selectGraspS3: ctx.tau_launch_s must be > 0.");
  if (ctx.snapshot.omega_rad_s == 0.0) throw std::invalid_argument("selectGraspS3: omega is zero.");
  if (!std::isfinite(ctx.min_delay_s) || ctx.min_delay_s < 0.0) {
    throw std::invalid_argument("selectGraspS3: min_delay_s must be finite and >= 0.");
  }
}

std::string rejectMaskToString(unsigned mask) {
  std::string s;
  auto add = [&](const char* n) { s += (s.empty() ? "" : ",") + std::string(n); };
  if (mask & kRejectIk) add("ik");
  if (mask & kRejectRTrust) add("r_trust");
  if (mask & kRejectStartGoal) add("start_goal");
  return s.empty() ? "none" : s;
}

/// J_static of a candidate (the criterion of the finalists): d_mm + kappa_w_mm * (-ln w_full_static), t_wait = 0.
/// d_mm = d * length_scale_m * 1000 (= d_pos in mm when orientation_weight = 0).
double staticCostMm(const S3Candidate& c, const S3Params& params) {
  return s3CostV2(c.d * params.length_scale_m * 1000.0, c.w_full_static, 0.0, params);
}

/// The one order of `accepted` (hence of the finalists and of best_per_k): J_static (S3Candidate::cost, set by
/// CandidateEvaluator) ascending; ties broken deterministically by k, theta, wrist_flip ascending.
bool lessStatic(const S3Candidate& a, const S3Candidate& b) {
  if (a.cost != b.cost) return a.cost < b.cost;
  if (a.eval.k != b.eval.k) return static_cast<int>(a.eval.k) < static_cast<int>(b.eval.k);
  if (a.eval.theta_rad != b.eval.theta_rad) return a.eval.theta_rad < b.eval.theta_rad;
  return a.eval.wrist_flip < b.eval.wrist_flip;
}

}  // namespace

double thetaDistRad(double a, double b) {
  const double kPi = haptic_dmp_learning::core::kPi;
  double d = std::fmod(std::abs(a - b), 2.0 * kPi);
  if (d > kPi) d = 2.0 * kPi - d;
  return d;
}

void validateS3Params(const S3Params& params) {
  if (!isFinitePositive(params.length_scale_m) || !isFinitePositive(params.angle_scale_rad) || !isFinitePositive(params.theta_step_deg) ||
      !isFinitePositive(params.r_trust_m) || !isFinitePositive(params.delta_v_max_m_s)) {
    throw std::invalid_argument("selectGraspS3: scales, steps, r_trust_m and delta_v_max_m_s must be finite and > 0.");
  }
  if (!(params.orientation_weight >= 0.0) || !std::isfinite(params.orientation_weight)) {
    throw std::invalid_argument("selectGraspS3: orientation_weight must be finite and >= 0.");
  }
  if (params.max_posthoc_candidates < 0) throw std::invalid_argument("selectGraspS3: max_posthoc_candidates < 0.");
  if (!std::isfinite(params.posthoc_cluster_deg) || params.posthoc_cluster_deg < 0.0) {
    throw std::invalid_argument("selectGraspS3: posthoc_cluster_deg must be finite and >= 0.");
  }
  if (params.force_flip < -1 || params.force_flip > 1) {
    throw std::invalid_argument("selectGraspS3: force_flip must be -1, 0 or 1.");
  }
  (void)parseBranchRule(params.branch_rule);     // throws std::invalid_argument for an unknown value, no fallback
  if (!(std::isfinite(params.kappa_w_mm) && params.kappa_w_mm >= 0.0)) {
    throw std::invalid_argument("selectGraspS3: kappa_w must be finite and >= 0.");
  }
  if (!(std::isfinite(params.kappa_t_mm_s) && params.kappa_t_mm_s >= 0.0)) {
    throw std::invalid_argument("selectGraspS3: kappa_t must be finite and >= 0.");
  }
  if (!(std::isfinite(params.tau_j_mm) && params.tau_j_mm >= 0.0)) {
    throw std::invalid_argument("selectGraspS3: tau_j_mm must be finite and >= 0.");
  }
  if (params.finalists_n < 1) throw std::invalid_argument("selectGraspS3: finalists_n must be >= 1.");
  if (!(std::isfinite(params.min_clearance_mm) && params.min_clearance_mm >= 0.0)) {
    throw std::invalid_argument("selectGraspS3: s1.min_clearance_mm must be finite and >= 0.");
  }
  validateClearanceParams(ClearanceParams{params.clearance_dt_s, params.gripper_open_m});  // s1.clearance_dt_s, s1.gripper_open_m
  if (!(std::isfinite(params.min_margin_deg) && params.min_margin_deg >= 0.0)) {
    throw std::invalid_argument("selectGraspS3: s1.min_margin_deg must be finite and >= 0.");
  }
  if (!(std::isfinite(params.margin_follow_s) && params.margin_follow_s >= 0.0)) {
    throw std::invalid_argument("selectGraspS3: s1.margin_follow_s must be finite and >= 0.");
  }
  if (!isFinitePositive(params.margin_follow_dt_s)) {
    throw std::invalid_argument("selectGraspS3: s1.margin_follow_dt_s must be finite and > 0.");
  }
  if (!(std::isfinite(params.max_start_goal_deg) && params.max_start_goal_deg >= 0.0 && params.max_start_goal_deg <= 180.0)) {
    throw std::invalid_argument("selectGraspS3: max_start_goal_deg must be finite and in [0, 180].");
  }
  if (params.rollout_q_ref.empty()) {
    throw std::invalid_argument(
        "selectGraspS3: rollout_q_ref is required (s1.rollout_q_ref: 7 values [rad], the controller's nullspace "
        "target = its joint configuration at activation); there is no default and no fallback.");
  }
  if (!params.rollout_q_ref.empty() && params.rollout_q_ref.size() != static_cast<std::size_t>(RobotModel::kNumJoints)) {
    throw std::invalid_argument("selectGraspS3: rollout_q_ref must have exactly 7 values (got " +
                                std::to_string(params.rollout_q_ref.size()) + ").");
  }
  for (double v : params.rollout_q_ref) {
    if (!std::isfinite(v)) throw std::invalid_argument("selectGraspS3: rollout_q_ref must be finite.");
  }
  if (!params.force && params.force_flip != -1) {
    throw std::invalid_argument("selectGraspS3: force_flip requires force = true.");
  }
  if (params.force) {
    if (params.force_k < 0 || params.force_k > 3) {
      throw std::invalid_argument("selectGraspS3: force_k must be in 0..3.");
    }
    if (!std::isfinite(params.force_theta_deg) || params.force_theta_deg < -180.0 || params.force_theta_deg > 180.0) {
      throw std::invalid_argument("selectGraspS3: force_theta_deg must be finite and in [-180, 180].");
    }
    if (!std::isfinite(params.force_psi_deg) || std::abs(params.force_psi_deg) > 8.0) {
      throw std::invalid_argument("selectGraspS3: |force_psi_deg| must be finite and <= 8.");
    }
  }
  if (!(params.ik.max_iter > 0) || !isFinitePositive(params.ik.pos_tol_m) || !isFinitePositive(params.ik.ori_tol_rad) || !isFinitePositive(params.ik.max_step_rad) ||
      !isFinitePositive(params.ik.damping) || !(std::isfinite(params.ik.nullspace_gain) && params.ik.nullspace_gain >= 0.0)) {
    throw std::invalid_argument("selectGraspS3: invalid IkParams.");
  }
}

double s3Distance(double d_pos_m, double ori_rad, double length_scale_m, double orientation_weight,
                  double angle_scale_rad) {
  if (!(isFinitePositive(length_scale_m) && isFinitePositive(angle_scale_rad) &&
        std::isfinite(orientation_weight) && orientation_weight >= 0.0)) {
    throw std::invalid_argument("s3Distance: scales must be finite and > 0, weight finite and >= 0.");
  }
  const double a = d_pos_m / length_scale_m;
  const double b = ori_rad / angle_scale_rad;
  return std::sqrt(a * a + orientation_weight * b * b);
}

unsigned s3AprioriRejectMaskV2(const PoseEvaluation& eval, double d_pos_m, const S3Params& params) {
  unsigned mask = 0;
  if (!eval.feasible_basic) mask |= kRejectIk;
  if (!(d_pos_m <= params.r_trust_m)) mask |= kRejectRTrust;
  if (params.max_start_goal_deg < 180.0 &&
      haptic_dmp_learning::core::radToDeg(eval.dist_to_demo.ori_rad) > params.max_start_goal_deg) {
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

namespace {

/// Note of the lazy use_margin flow when no finalist is admissible: counts of the discards by margin / clearance and the
/// finalist with the best margin. Read only.
std::string noAdmissibleMarginNote(const std::vector<S3FinalistRecord>& fin, const S3Result& res, const S3Params& params,
                                   double kRad2Deg) {
  const S3FinalistRecord* best = nullptr;
  int n_margin = 0, n_clearance = 0;
  for (const auto& f : fin) {
    if (f.discard_reason == "margin") ++n_margin;
    if (f.discard_reason == "clearance") ++n_clearance;
    if (f.margin_done && (!best || f.margin_deg > best->margin_deg)) best = &f;
  }
  std::ostringstream m;
  m << "v2 margin/clearance: no finalist is admissible (" << fin.size() << " finalists, " << res.rollouts_done
    << " rollouts, " << n_margin << " discarded by margin < " << params.min_margin_deg << " deg, " << n_clearance
    << " by clearance < " << params.min_clearance_mm << " mm)";
  if (best) {
    m << "; best margin " << best->margin_deg << " deg for k=" << static_cast<int>(best->cand.eval.k)
      << " branch " << (best->cand.eval.wrist_flip ? 'B' : 'A') << " theta="
      << best->cand.eval.theta_rad * kRad2Deg << " deg (fer_joint" << best->margin_joint + 1 << " at t_rel "
      << best->margin_t_rel_s << " s)";
  }
  return m.str();
}

/// res.ranked (the rollout-feasible finalists by J, first kRankedSize) and res.best_per_k (the first of each k in the J_static order of
/// @p accepted; a rejected one with its reason if none passes).
void fillRankedAndBestPerK(S3Result& res, const std::vector<S3Candidate>& accepted, const S3Candidate (&least_bad)[4],
                           const bool (&have_least)[4]) {
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
}

/// res.status, res.note and res.chosen from the outcome of the selection stages.
void finalizeStatus(S3Result& res, const std::vector<S3Candidate>& accepted, const S3Params& params,
                    bool clearance_no_candidate, const std::string& clearance_note, int chosen_idx) {
  if (clearance_no_candidate) {
    res.status = S3Result::kNoCandidate;
    res.note = clearance_note;
  } else if (accepted.empty()) {
    res.status = S3Result::kNoCandidate;
    if (!params.force) {
      res.note = "no candidate satisfies the v2 constraints (ik, d_pos <= r_trust, start_goal <= max_start_goal_deg)";
    }
  } else if (chosen_idx < 0) {
    res.status = S3Result::kPosthocFailed;
    int n_feasible = 0;
    for (const auto& f : res.v2_finalists) n_feasible += f.rollout_feasible ? 1 : 0;
    std::ostringstream m;
    m << "v2: no finalist passed the rollout and the post-hoc checks (" << res.v2_finalists.size() << " finalists, "
      << n_feasible << " rollout-feasible, " << res.posthoc_checks << " post-hoc checks)";
    res.note = (params.force ? "FORCED candidate: " : "") + m.str();
  } else {
    res.status = S3Result::kSelected;
    res.chosen = accepted[static_cast<std::size_t>(chosen_idx)];
  }
}

/// Evaluation of one (k, theta, psi, flip) candidate: IK of both wrist branches, a-priori mask (ik, r_trust, start_goal), and for a
/// passing one wait_s and cost = J_static. Former evalOne lambda of selectGraspS3: the members are the variables it
/// captured by reference (all read only except res.evaluations, least_bad and have_least, written through).
struct CandidateEvaluator {
  const haptic_dmp_learning::core::CubeSatelliteModel& model;
  RobotModel& robot;
  const RobotModel::JointVector& q_start;
  const DemoReference& demo;
  const Eigen::Vector3d& delta_g_demo;
  const S3Context& ctx;
  const S3Params& params;
  BranchRule branch_rule;
  const Eigen::Vector3d& p0;
  S3Result& res;
  S3Candidate (&least_bad)[4];
  bool (&have_least)[4];

  S3Candidate operator()(GraspPointId k, double theta, double psi, int flip) const {
    S3Candidate c;
    const bool logging = ctx.eval_log != nullptr;
    const bool need_extra = true;  // w_full of the static solution (compute_extra) enters J_static
    const PoseEvaluationDetail det =
        evaluateGraspPoseDetailed(model, k, theta, psi, robot, q_start, demo, params.length_scale_m,
                                  params.angle_scale_rad, params.ik, need_extra, flip, branch_rule);
    c.eval = det.eval;
    c.psi_rad = psi;
    c.w_full_static = det.branch[det.selected].w_full;
    ++res.evaluations;
    c.d_pos_m = s3DPos(c.eval.pose.position_world, ctx.contact_to_end_offset, p0, delta_g_demo);
    c.d = s3Distance(c.d_pos_m, c.eval.dist_to_demo.ori_rad, params.length_scale_m,
                     params.orientation_weight, params.angle_scale_rad);
    const unsigned mask = s3AprioriRejectMaskV2(c.eval, c.d_pos_m, params);
    c.reject_mask = mask;
    if (mask & kRejectIk) {
      c.reject_reason = "ik";
    } else if (mask & kRejectRTrust) {
      c.reject_reason = "r_trust";
    } else if (mask & kRejectStartGoal) {
      c.reject_reason = "start_goal";
    } else {
      c.passes_constraints = true;
      c.wait_s = planLaunch(ctx.snapshot, ctx.t_now_s, theta, ctx.tau_launch_s, ctx.min_delay_s).delay_s;
      c.cost = staticCostMm(c, params);
    }
    if (logging) {
      S3EvalRecord r;
      r.k = k;
      r.theta_rad = theta;
      r.psi_rad = psi;
      r.reject_mask = mask;
      r.passes_constraints = c.passes_constraints;
      r.wrist_flip = c.eval.wrist_flip;
      r.d_pos_m = c.d_pos_m;
      r.d = c.d;
      r.wait_s = c.passes_constraints
                     ? c.wait_s
                     : planLaunch(ctx.snapshot, ctx.t_now_s, theta, ctx.tau_launch_s, ctx.min_delay_s).delay_s;
      r.j_static = staticCostMm(c, params);
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
  }
};

/// Candidate scans: the forced candidate, or the theta scan of each k on the single grid -pi + i * theta_step_deg (no
/// refinement). Fills @p accepted (unsorted) and res.forced / res.forced_violations / res.note.
void collectCandidates(const CandidateEvaluator& evalOne, const S3Context& ctx, const S3Params& params, S3Result& res,
                       std::vector<S3Candidate>& accepted) {
  const double kPi = haptic_dmp_learning::core::kPi;

  const double step = haptic_dmp_learning::core::degToRad(params.theta_step_deg);
  const int n_theta = static_cast<int>(std::ceil(2.0 * kPi / step - 1e-9));  // [-pi, pi)

  res.forced = params.force;
  if (params.force) {
    // Test mode: only this candidate; everything after the scans (ranking, post-hoc, outputs) is unchanged.
    const GraspPointId kf = kAllK[params.force_k];
    const double theta_f = haptic_dmp_learning::core::degToRad(params.force_theta_deg);
    const double psi_f = haptic_dmp_learning::core::degToRad(params.force_psi_deg);
    S3Candidate c = evalOne(kf, theta_f, psi_f, params.force_flip);
    if (c.passes_constraints) {
      accepted.push_back(c);
    } else if (params.force_ignore_constraints) {
      // evalOne computes wait/cost only for passing candidates: same formulas, forced theta.
      c.wait_s = planLaunch(ctx.snapshot, ctx.t_now_s, theta_f, ctx.tau_launch_s, ctx.min_delay_s).delay_s;
      c.cost = staticCostMm(c, params);
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
    for (int i = 0; i < n_theta; ++i) {
      const double theta = -kPi + i * step;
      S3Candidate c = evalOne(k, theta, 0.0, -1);
      if (c.passes_constraints) accepted.push_back(c);
    }
  }
}

/// Post-hoc velocity check shared by both flows (lazy, eager). Without the check: true, dv and
/// @p entry untouched. With it: fills entry.v_ee / v_target / dv_m_s and @p dv; false = discarded (reject_reason set
/// on @p c and on the entry, entry appended to the log). What the discard also does (continue, discard_reason,
/// alive) stays at the call site.
bool posthocVelocityCheck(const haptic_dmp_learning::core::CubeSatelliteModel& model, const S3Context& ctx,
                          const S3Params& params, const std::unique_ptr<haptic_dmp_learning::core::ProDMP>& prodmp_at_contact,
                          S3Result& res, S3Candidate& c, S3PosthocEntry& entry, double& dv) {
  if (!params.posthoc_velocity_check) return true;
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
    return false;
  }
  return true;
}

struct StaticItem {
  int idx;
  double d_mm;
  double j_static;
};

/// Phase 2: finalists = best (J_static) of each (k, wrist branch, theta cluster), at most finalists_n, in J_static order.
/// @p accepted must already be sorted by lessStatic (J_static = S3Candidate::cost, ties by k, theta, wrist_flip): the same
/// order serves the finalists and best_per_k.
std::vector<StaticItem> selectStaticFinalists(const std::vector<S3Candidate>& accepted, const S3Params& params, double cluster_rad) {
  std::vector<StaticItem> groups;  // representatives, in J_static order
  for (std::size_t i = 0; i < accepted.size(); ++i) {
    if (groups.size() >= static_cast<std::size_t>(params.finalists_n)) break;
    const S3Candidate& c = accepted[i];
    bool dup = false;
    for (const StaticItem& g : groups) {
      const S3Candidate& r = accepted[static_cast<std::size_t>(g.idx)];
      if (r.eval.k != c.eval.k || r.eval.wrist_flip != c.eval.wrist_flip) continue;
      if (thetaDistRad(c.eval.theta_rad, r.eval.theta_rad) < cluster_rad) {
        dup = true;
        break;
      }
    }
    if (!dup) groups.push_back({static_cast<int>(i), c.d * params.length_scale_m * 1000.0, c.cost});
  }
  return groups;
}

}  // namespace

S3Result selectGraspS3(const haptic_dmp_learning::core::CubeSatelliteModel& model, RobotModel& robot,
                       const RobotModel::JointVector& q_start, const DemoReference& demo,
                       const Eigen::Vector3d& delta_g_demo, const S3Context& ctx, const S3Params& params) {
  validateS3Params(params);
  validateCtx(ctx);
  const BranchRule branch_rule = parseBranchRule(params.branch_rule);
  RobotModel::JointVector q_rollout_ref = RobotModel::JointVector::Zero();
  for (int i = 0; i < RobotModel::kNumJoints; ++i) q_rollout_ref(i) = params.rollout_q_ref[static_cast<std::size_t>(i)];
  {
    const RobotModel::JointVector lo = robot.jointLowerLimits();
    const RobotModel::JointVector hi = robot.jointUpperLimits();
    if (!((q_rollout_ref.array() >= lo.array()).all() && (q_rollout_ref.array() <= hi.array()).all())) {
      throw std::invalid_argument("selectGraspS3: rollout_q_ref is outside the URDF joint limits.");
    }
  }
  const double kPi = haptic_dmp_learning::core::kPi;

  robot.update(q_start, RobotModel::JointVector::Zero());
  const Eigen::Vector3d p0 = robot.eePosition();
  const bool use_clearance = params.use_clearance;
  const bool use_margin = params.use_margin;
  // use_margin without force: lazy evaluation in J_static order (see S3Params::use_margin). With force the constraint only warns.
  const bool lazy = use_margin && !params.force;
  if (use_clearance && !ctx.clearance_eval) {
    throw std::invalid_argument("selectGraspS3: s1.use_clearance=true needs S3Context::clearance_eval.");
  }
  // Start orientation of the approach trajectory = FK(q_start), as in checkKinematicFeasibility (read only if used).
  const Eigen::Quaterniond quat0 = use_clearance ? robot.eeOrientation() : Eigen::Quaterniond::Identity();

  S3Result res;
  std::vector<S3Candidate> accepted;
  S3Candidate least_bad[4];
  bool have_least[4] = {false, false, false, false};

  const CandidateEvaluator evalOne{model, robot, q_start, demo, delta_g_demo, ctx, params, branch_rule, p0, res, least_bad, have_least};

  collectCandidates(evalOne, ctx, params, res, accepted);

  std::stable_sort(accepted.begin(), accepted.end(), lessStatic);

  // ---- finalists, rollout and post-hoc checks, in J_static order ----
  std::unique_ptr<haptic_dmp_learning::core::ProDMP> prodmp_at_contact;
  if (params.posthoc_velocity_check) {
    // Support goal p0 + delta_g_demo, stepped to the contact instant.
    prodmp_at_contact = std::make_unique<haptic_dmp_learning::core::ProDMP>(*ctx.prodmp_template);
    prodmp_at_contact->setRelativeGoal(false);
    prodmp_at_contact->setInitialConditions(0.0, p0, Eigen::Vector3d::Zero());
    prodmp_at_contact->setGoal(p0 + delta_g_demo);
    const double tau_contact = ctx.tau_contact_s > 0.0 ? ctx.tau_contact_s : ctx.prodmp_template->tau();
    const int n_steps = std::max(1, static_cast<int>(std::ceil(tau_contact / kDefaultSimDtS - 1e-9)));
    for (int i = 0; i < n_steps; ++i) prodmp_at_contact->step(kDefaultSimDtS);
  }
  const std::shared_ptr<RobotModel> robot_ptr(&robot, [](RobotModel*) {});  // non-owning

  int chosen_idx = -1;
  const double cluster_rad = haptic_dmp_learning::core::degToRad(params.posthoc_cluster_deg);
  // Post-hoc velocity check shared by both flows (lazy, eager); see posthocVelocityCheck.
  auto posthocVelocityPasses = [&](S3Candidate& c, S3PosthocEntry& entry, double& dv) -> bool {
    return posthocVelocityCheck(model, ctx, params, prodmp_at_contact, res, c, entry, dv);
  };
  bool clearance_no_candidate = false;
  std::string clearance_note;
  const double kRad2Deg = 180.0 / kPi;

  // ---- v2 phase 2: finalists = best (J_static) of each (k, wrist branch, theta cluster), at most finalists_n ----
  const std::vector<StaticItem> groups = selectStaticFinalists(accepted, params, cluster_rad);

  // ---- v2 phase 3: rollout of every finalist with nullspace target rollout_q_ref (use_margin without force: lazy, below) ----
  std::vector<S3FinalistRecord> fin;
  fin.reserve(groups.size());
  const RobotModel::JointVector zero_dq = RobotModel::JointVector::Zero();
  auto makeRecord = [&](const StaticItem& g) {
    const S3Candidate& c = accepted[static_cast<std::size_t>(g.idx)];
    S3FinalistRecord rec;
    rec.cand = c;
    rec.accepted_index = g.idx;
    rec.d_mm = g.d_mm;
    rec.w_static = c.w_full_static;
    rec.t_wait_s = c.wait_s;
    rec.start_goal_deg = c.eval.dist_to_demo.ori_rad * kRad2Deg;
    return rec;
  };
  auto runRollout = [&](S3FinalistRecord& rec) {
    const S3Candidate& c = accepted[static_cast<std::size_t>(rec.accepted_index)];
    const Eigen::Quaterniond orient = goalOrientation(c.eval);
    const FeasibilityResult fr = checkKinematicFeasibility(
        *ctx.prodmp_template, c.eval.pose.position_world, orient, robot_ptr, q_start, kDefaultSimDtS, kDefaultJointViolationTolRad,
        q_rollout_ref);
    ++res.rollouts_done;
    rec.rollout_done = true;
    rec.rollout_feasible = fr.feasible;
    rec.rollout_min_margin_rad = fr.min_joint_margin_rad;
    if (fr.min_margin_joint >= 0 && fr.min_margin_step >= 0) {  // something was simulated
      rec.rollout_end.q_end = fr.q_end_post_step;
      rec.rollout_end.min_margin_rad = fr.min_joint_margin_rad;
      rec.rollout_end.joint = fr.min_margin_joint;
      rec.rollout_end.t_rel_s = static_cast<double>(fr.min_margin_step + 1) * kDefaultSimDtS - ctx.prodmp_template->tau();
      rec.rollout_end_valid = true;
    }
    rec.rollout_pos_err_mm = fr.pos_err_final_mm;
    robot.update(fr.q_final, zero_dq);  // w_eval = w_full(q_final of the rollout)
    rec.rollout_ori_err_deg =
        se3Distance(robot.eePosition(), robot.eeOrientation(), c.eval.pose.position_world, orient, 1.0, 1.0)
            .ori_rad *
        kRad2Deg;
    rec.w_rollout = fullManipulability(robot.jacobian()).w_full;
    if (fr.feasible) {
      rec.j = s3CostV2(rec.d_mm, rec.w_rollout, rec.t_wait_s, params);
      rec.cand.cost = rec.j;
      accepted[static_cast<std::size_t>(rec.accepted_index)].cost = rec.j;
    }
  };
  // Predicted joint margin over [start, AT_END + margin_follow_s] (followMargin, the G16 procedure).
  auto runMargin = [&](S3FinalistRecord& rec) {
    const S3Candidate& c = accepted[static_cast<std::size_t>(rec.accepted_index)];
    FollowMarginParams fp;
    fp.follow_s = params.margin_follow_s;
    fp.dt_approach_s = kDefaultSimDtS;
    fp.dt_follow_s = params.margin_follow_dt_s;
    const FollowMarginResult fm = followMargin(
        *ctx.prodmp_template, c.eval.pose.position_world, goalOrientation(c.eval), ctx.snapshot.center,
        ctx.snapshot.axis, ctx.snapshot.omega_rad_s, robot_ptr, q_start, q_rollout_ref, fp,
        rec.rollout_end_valid ? &rec.rollout_end : nullptr);  // same finalist, seed, q_ref, dt: the approach is not repeated
    ++res.margin_evals_done;
    rec.margin_done = true;
    rec.margin_deg = fm.min_margin_rad * kRad2Deg;
    rec.margin_joint = fm.joint;
    rec.margin_t_rel_s = fm.t_rel_s;
    rec.margin_rejected = !(rec.margin_deg >= params.min_margin_deg);  // NaN -> rejected
  };
  for (const StaticItem& g : groups) {
    S3FinalistRecord rec = makeRecord(g);
    if (!lazy) {
      runRollout(rec);
      if (use_margin) runMargin(rec);  // force: evaluated for the single forced finalist, warns below
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
                                goalOrientation(cand.eval), kDefaultSimDtS);
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
    f.clearance_rejected = !(cr.min_mm_no_tail >= params.min_clearance_mm);
    return cr.min_mm_no_tail;
  };
  // Accept in the rollout-based flows (lazy, eager): the rollout is the path check, @p rec becomes the winner.
  auto acceptRolloutWinner = [&](S3FinalistRecord& rec, S3PosthocEntry& entry, double dv) {
    entry.path_checked = true;  // the rollout is the path check
    entry.path_ok = true;
    entry.min_margin_rad = rec.rollout_min_margin_rad;
    entry.accepted = true;
    res.posthoc_log.push_back(entry);
    chosen_idx = rec.accepted_index;
    rec.winner = true;
    res.posthoc_dv_m_s = dv;
    res.posthoc_min_margin_rad = rec.rollout_min_margin_rad;
  };
  // ---- use_margin without force: lazy flow in J_static order (S3Params::use_margin). Per finalist: rollout (w, margin
  //      on [start, tau]) -> followMargin -> clearance -> post-hoc velocity check; stop at the first that passes all of
  //      them. No rollout is done on the finalists after it. ----
  if (lazy) {
    bool any_admissible = false;  // passed rollout, margin and clearance (the velocity check may still discard it)
    bool any_rollout_ok = false;
    for (std::size_t fi = 0; fi < fin.size() && chosen_idx < 0; ++fi) {
      S3FinalistRecord& rec = fin[fi];
      S3Candidate& c = accepted[static_cast<std::size_t>(rec.accepted_index)];
      runRollout(rec);
      if (!rec.rollout_feasible) {
        rec.discard_reason = "rollout";
        continue;
      }
      any_rollout_ok = true;
      runMargin(rec);
      if (rec.margin_rejected) {
        rec.discard_reason = "margin";
        continue;
      }
      if (use_clearance) {
        clearanceOf(fi);
        if (rec.clearance_rejected) {
          rec.discard_reason = "clearance";
          continue;
        }
      }
      any_admissible = true;
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
      if (!posthocVelocityPasses(c, entry, dv)) {
        rec.discard_reason = "velocity";
        continue;
      }
      acceptRolloutWinner(rec, entry, dv);
    }
    if (chosen_idx < 0 && any_rollout_ok && !any_admissible) {
      // No finalist satisfies rollout + margin (+ clearance): kNoCandidate, no fallback.
      clearance_no_candidate = true;  // (flag shared with the clearance constraint: it selects kNoCandidate + clearance_note)
      clearance_note = noAdmissibleMarginNote(fin, res, params, kRad2Deg);
    }
    any_rollout_feasible = any_rollout_ok;
  }
  while (!lazy && chosen_idx < 0) {
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
    if (use_margin && params.force && rec.margin_done && rec.margin_rejected) {  // forced: the constraint never discards, it warns
      std::ostringstream w;
      w << "WARN s1.use_margin: FORCED candidate k=" << static_cast<int>(c.eval.k) << " theta="
        << c.eval.theta_rad * kRad2Deg << " deg has predicted joint margin " << rec.margin_deg << " deg < "
        << params.min_margin_deg << " deg (fer_joint" << rec.margin_joint + 1 << " at t_rel " << rec.margin_t_rel_s
        << " s); kept because force is on";
      res.warnings.push_back(w.str());
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
    if (!posthocVelocityPasses(c, entry, dv)) {
      alive[fi] = 0;
      continue;
    }
    acceptRolloutWinner(rec, entry, dv);
  }
  res.v2_finalists = fin;
  if (!lazy && use_clearance && !params.force && chosen_idx < 0 && any_rollout_feasible && !clearance_any_admissible) {
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
    chosen_idx = 0;  // forced + ignore: the failure is reported, not obeyed
    res.forced_violations += std::string(res.forced_violations.empty() ? "" : "; ") +
                             "v2: no finalist passed the rollout / post-hoc checks";
  }

  // ---- outputs ----
  fillRankedAndBestPerK(res, accepted, least_bad, have_least);

  finalizeStatus(res, accepted, params, clearance_no_candidate, clearance_note, chosen_idx);
  return res;
}

}  // namespace core
}  // namespace satellite_grasp_planner
