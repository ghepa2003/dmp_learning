#pragma once

/**
 * @file grasp_s3_selection.hpp
 * @brief Reduced-problem grasp selection "S3": pick the (k, theta) whose grasp position is closest to the
 *        deformation of the single demo, subject to manipulability / joint-margin / trust-radius / IK
 *        constraints, with a small penalty on the waiting time. Velocity and path feasibility are only
 *        checked a posteriori. No collision, no e_v term in the cost, no continuous roll search.
 *        Eigen + Pinocchio only, NO ROS.
 *
 *   d_pos = || (goal_cand - p0) - delta_g_demo ||,   goal_cand = grasp position + contact_to_end_offset
 *           (contact_to_end_offset comes from the demo, see SelectionInputs)
 *   d     = sqrt(d_pos^2 / l^2 + orientation_weight * ori^2 / angle_scale^2)
 *   cost  = d + epsilon_wait_per_s * wait_s
 * wait_s is the delay of planLaunch() (launchDelay logic, with min_delay_s).
 */

#include <array>
#include <functional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "satellite_grasp_planner/core/approach_clearance.hpp"
#include "satellite_grasp_planner/core/follow_margin.hpp"
#include "satellite_grasp_planner/core/grasp_pose_eval.hpp"
#include "satellite_grasp_planner/core/launch_plan.hpp"

namespace satellite_grasp_planner {
namespace core {

struct S3Params {
  double length_scale_m = 0.10;
  double orientation_weight = 0.0;
  double angle_scale_rad = 1.0;
  double epsilon_wait_per_s = 1e-4;
  double w_hat_min = 0.5;
  double margin_min_rad = 0.3;
  double r_trust_m = 0.30;
  double delta_v_max_m_s = 3e-3;
  double theta_step_deg = 2.0;
  double theta_refine_step_deg = 0.1;
  IkParams ik;
  bool posthoc_path_check = true;
  bool posthoc_velocity_check = true;
  int max_posthoc_candidates = 10;
  /// The post-hoc stage skips a candidate of the same k within this many degrees of theta of one already
  /// checked (near-duplicates). 0 disables the clustering.
  double posthoc_cluster_deg = 10.0;
  /// Test mode: skip the scans and evaluate ONLY (force_k, force_theta_deg, force_psi_deg); ranking, post-hoc
  /// checks, best_per_k and the chosen candidate follow unchanged. force_k in 0..3 (GraspPointId), force_theta_deg
  /// in [-180, 180], |force_psi_deg| <= 8 (validateS3Params throws otherwise). A candidate violating an a-priori
  /// or post-hoc constraint gives kNoCandidate / kPosthocFailed (reasons in S3Result::note), unless
  /// force_ignore_constraints: then it is selected anyway and the violations are listed in
  /// S3Result::forced_violations. Default: off, behaviour unchanged.
  bool force = false;
  int force_k = 0;
  double force_theta_deg = 0.0;
  double force_psi_deg = 0.0;
  bool force_ignore_constraints = false;
  /// With force: -1 = branch chosen by the current rule (static joint margin, see evaluateGraspPoseDetailed),
  /// 0 / 1 = ONLY that wrist branch (wrist_flip == force_flip), no fallback to the other one: if its IK does not
  /// converge the candidate violates "ik" like any other. Must be -1 when force is off.
  int force_flip = -1;
  /// Wrist-branch rule when force_flip == -1 (see BranchRule): "margin" (default, behaviour unchanged) or "rotation"
  /// (branch with the smaller angle between its orientation and FK(q_start)). Any other value: validateS3Params
  /// throws. force_flip >= 0 takes precedence over it.
  std::string branch_rule = "margin";

  // ---- mode "v2" (s1.mode); with "legacy" (default) none of the fields below is read and nothing changes ----
  /// "legacy" (default, bit-identical to the code before v2) or "v2" (see selectGraspS3). Other value: validateS3Params throws.
  /// v2: psi = 0 in the scans; a-priori constraints are ik (+ joint limits), r_trust and max_start_goal_deg (w_hat_min and
  /// margin_min_rad are NOT applied); finalists are the best of each (k, wrist branch, theta cluster of
  /// posthoc_cluster_deg) by J_static, at most finalists_n; each gets a kinematic rollout with nullspace target
  /// rollout_q_ref; J = d_mm [+ kappa_w_mm * (-ln w_full(q_final))] + kappa_t_mm_s * t_wait; among the finalists with
  /// J <= J_min + tau_j_mm the smallest t_wait wins; the post-hoc velocity check then runs on the winner (next best
  /// if it fails). posthoc_path_check is ignored (the rollout is the path check). force works as in legacy (the single
  /// forced candidate is the only finalist; force_psi_deg is honoured).
  std::string mode = "legacy";
  /// v2 a-priori: reject a candidate whose chosen-branch angle to FK(q_start) (dist_to_demo.ori_rad) exceeds this
  /// [deg]. 180 = disabled (the data cover only 16.5 and 34.5 deg: success, 179.6 deg: failure). In [0, 180].
  double max_start_goal_deg = 180.0;
  int finalists_n = 12;  ///< v2: >= 1
  /// v2: nullspace target of the rollout [rad], 7 values; MUST be given (non-empty) in v2, no fallback. It must equal
  /// the nullspace target of the real controller (its joint configuration at activation).
  std::vector<double> rollout_q_ref;
  bool use_w = true;          ///< v2: false -> J = d_mm [+ kappa_t * t_wait] (no w term)
  double kappa_w_mm = 7.5;    ///< v2: weight of -ln(w_full) [mm], ASSUMED, >= 0
  double kappa_t_mm_s = 0.0;  ///< v2: weight of the wait [mm/s], >= 0
  double tau_j_mm = 2.0;      ///< v2: tie band on J [mm], ASSUMED, >= 0

  // ---- v2 clearance constraint (docs/S3_SPEC.md, "vincolo di clearance"); with use_clearance == false (default)
  //      nothing below is read and the selection is bit-identical ----
  /// v2 only (true with mode "legacy" is an error): a finalist whose clearance (approachClearance(), commanded TCP
  /// trajectory vs rotating satellite, window [0, tau - 2 s]) is below min_clearance_mm is discarded. Lazy: the finalists are
  /// scanned in J order; the first admissible one fixes J*; the others with J <= J* + tau_j_mm are evaluated too;
  /// among the admissible ones the smaller wait wins. No admissible finalist: kNoCandidate (no fallback). With
  /// force the constraint only warns (S3Result::warnings). Needs S3Context::clearance_eval.
  bool use_clearance = false;
  double min_clearance_mm = 2.0;  ///< ASSUMED threshold [mm], finite and >= 0
  double clearance_dt_s = 0.05;   ///< sampling of the window [s], finite and > 0
  /// Finger joint position of the open gripper [m], both fingers, in [0, 0.08]. MUST equal the executor's
  /// gripper_open_position (haptic_dmp_learning/config/params.yaml:49, value 0.06): the planner cannot read it.
  double gripper_open_m = 0.060;

  // ---- v2 joint-margin constraint (docs/S3_SPEC.md §10, criterion 11); with use_margin == false (default) nothing
  //      below is read and the selection is bit-identical ----
  /// v2 only (true with mode "legacy" is an error): a finalist whose predicted joint margin over [start, AT_END + margin_follow_s]
  /// (followMargin(): the G16 / step 0c procedure, nullspace reference rollout_q_ref, seed q_start) is below min_margin_deg is
  /// discarded. LAZY, in J_static order: for each finalist rollout (w, margin), then margin, then clearance (if use_clearance),
  /// then the post-hoc velocity check; the first finalist satisfying all of them is selected and no further rollout is done
  /// (so the wait tie-break within tau_j_mm of the eager v2 selection does NOT apply in this mode). None admissible:
  /// kNoCandidate (no fallback). With force the constraint only warns (S3Result::warnings).
  bool use_margin = false;
  /// ASSUMED threshold [deg], finite and >= 0 (observed G16 interval: failed 13.26-20.84, succeeded 28.63-35.00; 24 lies in the gap).
  double min_margin_deg = 24.0;
  /// Duration of the simulated follow after AT_END [s], finite and >= 0. 3.0 = executor's post_grasp_follow_s (and the G16 runs).
  double margin_follow_s = 3.0;
  /// Integration step of the follow [s], finite and > 0. 0.01 = step0c_margins.cpp (the G16 / criterion-11 values).
  double margin_follow_dt_s = 0.01;
};

enum class S3Mode { kLegacy,
                    kV2 };

/// "legacy" -> kLegacy, "v2" -> kV2 (exact). @throws std::invalid_argument otherwise (no fallback).
S3Mode parseS3Mode(const std::string& name);

/// A-priori rejection bits (selectGraspS3 cascade order: ik, w_hat, margin, r_trust). A candidate passes iff the
/// mask is 0. The mask holds ALL failing constraints; S3Candidate::reject_reason keeps only the first one.
enum S3RejectBit : unsigned { kRejectIk = 1u,
                              kRejectWHat = 2u,
                              kRejectMargin = 4u,
                              kRejectRTrust = 8u,
                              kRejectStartGoal = 128u  ///< v2 only (max_start_goal_deg); never set by s3AprioriRejectMask. 16..64 are the landscape tool diagnostics
};

/// One row of the optional per-candidate log (S3Context::eval_log): one per evaluateGraspPose call of the scan, in
/// call order. Filled with the same values the selection uses; wait_s / cost_current are computed for EVERY row
/// (S3Candidate::wait_s is 0 for rejected candidates, the log is not).
struct S3EvalRecord {
  haptic_dmp_learning::core::GraspPointId k = haptic_dmp_learning::core::GraspPointId::kP0;
  double theta_rad = 0.0;
  double psi_rad = 0.0;
  bool refine_stage = false;  ///< false: coarse grid, true: refinement around the best of k
  unsigned reject_mask = 0;   ///< S3RejectBit
  bool passes_constraints = false;
  bool wrist_flip = false;
  double d_pos_m = 0.0;
  double d = 0.0;
  double wait_s = 0.0;
  double cost_current = 0.0;  ///< d + epsilon_wait_per_s * wait_s
  double w_hat = 0.0;
  double w_trans = 0.0;
  double margin_rad = 0.0;  ///< min joint margin of the IK solution of the chosen branch
  BranchDetail branch[2];   ///< both branches (manipulability fields filled)
};

/// Everything besides the robot/pose evaluation that selectGraspS3 needs (see SelectionInputs).
struct S3Context {
  const haptic_dmp_learning::core::ProDMP* prodmp_template = nullptr;  ///< required, not modified
  Eigen::Vector3d contact_to_end_offset = Eigen::Vector3d::Zero();     ///< SelectionInputs::contact_to_end_offset
  double tau_contact_s = 0.0;                                          ///< <= 0: the template's tau; used for the velocity check
  double tau_launch_s = 0.0;                                           ///< > 0, forwarded to planLaunch() (see planGraspAndLaunchS1)
  SatelliteSnapshot snapshot;                                          ///< omega, center, axis, theta/t of the acquisition
  double t_now_s = 0.0;                                                ///< instant the selection ends (as planGraspAndLaunchS1's t_now_s)
  double min_delay_s = 0.0;
  /// Optional per-candidate log (default nullptr = off, no change of behaviour or result). Not owned.
  std::vector<S3EvalRecord>* eval_log = nullptr;
  /// Clearance of one finalist (see S3Params::use_clearance). Required (non-empty) iff use_clearance; ignored otherwise.
  /// Production: makeClearanceEvaluator(model, {clearance_dt_s, gripper_open_m}).
  ClearanceEvaluator clearance_eval;
};

struct S3Candidate {
  PoseEvaluation eval;
  double d_pos_m = 0.0;
  double d = 0.0;
  double wait_s = 0.0;              ///< 0 if the candidate did not pass the constraints
  double cost = 0.0;                ///< d + epsilon * wait_s (meaningful only if passes_constraints)
  double psi_rad = 0.0;             ///< roll about z_tcp of this candidate (== eval.psi_rad; 0 in the scans)
  unsigned reject_mask = 0;         ///< S3RejectBit mask of the a-priori constraints (all failing ones)
  bool passes_constraints = false;  ///< a-priori constraints; a post-hoc failure sets reject_reason only
  std::string reject_reason;        ///< "ik", "w_hat", "margine", "r_trust" or a post-hoc reason
  double w_full_static = 0.0;       ///< v2: w_full at the static IK solution of the chosen branch (0 in legacy)
};

/// v2: one finalist of phase 2/3 (the rollout columns are meaningful only if rollout_done).
struct S3FinalistRecord {
  S3Candidate cand;         ///< cand.cost = J (final) for a feasible finalist
  int accepted_index = -1;  ///< index in the accepted list at the time of the finalist stage (internal)
  double d_mm = 0.0;
  double w_static = 0.0;
  bool rollout_done = false;
  bool rollout_feasible = false;
  double w_rollout = 0.0;  ///< w_full(q_final of the rollout)
  double j = 0.0;          ///< J (with w_rollout and kappa_t * t_wait); meaningful if rollout_feasible
  double t_wait_s = 0.0;
  double start_goal_deg = 0.0;      ///< chosen branch vs FK(q_start)
  double rollout_pos_err_mm = 0.0;  ///< final pose error of the rollout vs the goal
  double rollout_ori_err_deg = 0.0;
  double rollout_min_margin_rad = 0.0;  ///< min joint margin along the rollout
  /// End state of the rollout (q after the last step, margin/joint/time of its approach minimum), handed to followMargin() so
  /// it does not re-simulate the approach. Valid iff rollout_end_valid.
  FollowApproachState rollout_end;
  bool rollout_end_valid = false;
  bool winner = false;
  // ---- clearance constraint (filled only if S3Params::use_clearance and the finalist was evaluated: lazy) ----
  bool clearance_done = false;
  bool clearance_rejected = false;  ///< evaluated and below min_clearance_mm (discarded; with force: only warned)
  double clearance_mm = 0.0;        ///< ClearanceResult::min_mm_no_tail, the value the constraint uses
  std::string clearance_pair;       ///< ClearancePair::str() of that minimum
  double clearance_t_rel_s = 0.0;   ///< its time relative to contact (t - tau, <= 0)
  // ---- joint-margin constraint (filled only if S3Params::use_margin and the finalist reached that stage: lazy) ----
  bool margin_done = false;
  bool margin_rejected = false;  ///< evaluated and below min_margin_deg
  double margin_deg = 0.0;       ///< followMargin() min margin over [start, AT_END + margin_follow_s] [deg]
  int margin_joint = -1;         ///< limiting joint, 0-based (0 = fer_joint1)
  double margin_t_rel_s = 0.0;   ///< its time relative to AT_END [s]
  /// Why the finalist was discarded in the lazy (use_margin) flow: "rollout" (joint-limit violation), "margin", "clearance",
  /// "velocity"; empty if not discarded / not evaluated. Empty in the eager flow.
  std::string discard_reason;
};

/// One post-hoc check (clustering skips are not logged). Vectors are zero when the velocity check is off.
struct S3PosthocEntry {
  haptic_dmp_learning::core::GraspPointId k = haptic_dmp_learning::core::GraspPointId::kP0;
  double theta_rad = 0.0;
  double wait_s = 0.0;
  double dv_m_s = 0.0;                                 ///< |v_target - v_ee|
  Eigen::Vector3d v_ee = Eigen::Vector3d::Zero();      ///< ProDMP velocity at contact for this goal
  Eigen::Vector3d v_target = Eigen::Vector3d::Zero();  ///< surface velocity of the grasp point
  bool path_checked = false;                           ///< false if the path check is off or the velocity check failed first
  bool path_ok = false;                                ///< meaningful only if path_checked
  double min_margin_rad = 0.0;                         ///< min joint margin along the path (0 if not path_checked or violated)
  bool accepted = false;                               ///< passed every enabled post-hoc check
  std::string reject_reason;                           ///< empty if accepted
};

struct S3Result {
  enum Status { kSelected,
                kNoCandidate,
                kPosthocFailed };
  Status status = kNoCandidate;
  S3Candidate chosen;                       ///< valid only when status == kSelected
  std::vector<S3Candidate> ranked;          ///< first 20 constraint-passing candidates by cost (post-hoc annotated)
  std::array<S3Candidate, 4> best_per_k;    ///< best of each k (a rejected one, with its reason, if none passes)
  int evaluations = 0;                      ///< evaluateGraspPose calls
  int posthoc_checks = 0;                   ///< candidates examined by the post-hoc stage
  double posthoc_dv_m_s = 0.0;              ///< |v_target - v_ee| of the chosen candidate (0 if not checked)
  double posthoc_min_margin_rad = 0.0;      ///< min joint margin along the path of the chosen one (0 if not checked)
  std::vector<S3PosthocEntry> posthoc_log;  ///< one entry per post-hoc check, in order
  std::string note;
  bool forced = false;                         ///< S3Params::force was on
  std::string forced_violations;               ///< force_ignore_constraints: constraints violated and ignored (empty if none)
  std::vector<S3FinalistRecord> v2_finalists;  ///< mode "v2" only: finalists in J_static order (empty in legacy)
  double clearance_stage_ms = 0.0;             ///< wall-clock time spent in the clearance evaluations (0 if use_clearance is off)
  std::vector<std::string> warnings;           ///< use_clearance / use_margin with force: below threshold (not discarded)
  int rollouts_done = 0;                       ///< v2: kinematic rollouts of finalists performed (eager: all finalists; use_margin: up to the first admissible; 0 in legacy)
  int margin_evals_done = 0;                   ///< v2 use_margin: followMargin() evaluations performed
};

/// Parameter checks done by selectGraspS3 (finite, positive scales/steps, non-negative weights; force_*, branch_rule: see S3Params).
/// @throws std::invalid_argument.
void validateS3Params(const S3Params& params);

/// d = sqrt(d_pos^2/l^2 + orientation_weight * ori^2/angle_scale^2). @throws std::invalid_argument for
/// non-positive/non-finite scales or a negative weight.
double s3Distance(double d_pos_m, double ori_rad, double length_scale_m, double orientation_weight,
                  double angle_scale_rad);

/// A-priori constraint mask of one evaluated candidate (see S3RejectBit): ik = !feasible_basic, w_hat = !(w_hat >=
/// w_hat_min), margin = !(min_margin >= margin_min_rad), r_trust = !(d_pos <= r_trust_m). Same predicates, same NaN
/// behaviour as the cascade of selectGraspS3 (which calls this function).
unsigned s3AprioriRejectMask(const PoseEvaluation& eval, double d_pos_m, const S3Params& params);

/// v2 a-priori mask: ik (= !feasible_basic), r_trust, and kRejectStartGoal when max_start_goal_deg < 180 and the chosen
/// branch's dist_to_demo.ori_rad exceeds it. w_hat_min / margin_min_rad are NOT applied. Same NaN behaviour as
/// s3AprioriRejectMask for ik / r_trust.
unsigned s3AprioriRejectMaskV2(const PoseEvaluation& eval, double d_pos_m, const S3Params& params);

/// v2 cost: d_mm + (use_w ? kappa_w_mm * (-ln w_full) : 0) + kappa_t_mm_s * t_wait_s. w_full <= 0 or NaN with use_w:
/// +infinity (a singular candidate never beats a regular one).
double s3CostV2(double d_mm, double w_full, double t_wait_s, const S3Params& params);

/// v2 tie-break: index of the entry with the smallest t_wait among those with J <= J_min + tau_j_mm (J_min over the
/// finite entries; ties on t_wait: smaller J, then smaller index). -1 if @p j is empty or has no finite entry.
/// @throws std::invalid_argument if the vectors differ in size or tau_j_mm < 0.
int s3PickWinnerV2(const std::vector<double>& j, const std::vector<double>& t_wait_s, double tau_j_mm);

/**
 * One round of the lazy v2 clearance filter (pure; selectGraspS3 calls it before every tie-break pick).
 * @param j  J of each finalist (non-finite = never considered, as in s3PickWinnerV2)
 * @param alive  in/out: entries still in play; a finalist evaluated below @p min_clearance_mm is set to 0
 * @param done / clearance_mm  in/out cache: eval(i) runs at most once per i
 * Scans the alive finite-J entries by increasing J (ties: smaller index), evaluating each until the first one with
 * clearance >= min_clearance_mm: its J is J*. Then evaluates every alive entry with J <= J* + tau_j_mm (the same
 * comparison as s3PickWinnerV2). A NaN clearance counts as a violation.
 * @return true iff at least one alive entry is admissible (J* exists); false: none is, and every alive finite-J entry
 *         was evaluated and discarded.
 * @throws std::invalid_argument for size mismatches or a non-finite / negative threshold or band.
 */
bool s3ClearanceRound(const std::vector<double>& j, std::vector<char>& alive, std::vector<char>& done,
                      std::vector<double>& clearance_mm, const std::function<double(std::size_t)>& eval,
                      double min_clearance_mm, double tau_j_mm);

/// d_pos = || (position_world + contact_to_end_offset - p0) - delta_g_demo ||  (the one used by selectGraspS3).
double s3DPos(const Eigen::Vector3d& position_world, const Eigen::Vector3d& contact_to_end_offset,
              const Eigen::Vector3d& p0, const Eigen::Vector3d& delta_g_demo);

/**
 * Scans k in {0,90,180,270} x theta in [-180, 180) deg (theta_step_deg), refines the best accepted of each k
 * with theta_refine_step_deg in +-theta_step_deg, ranks by cost and runs the post-hoc checks (relative
 * velocity |omega x r - v_ProDMP| <= delta_v_max_m_s; kinematic path via checkKinematicFeasibility with the
 * orientation of the chosen branch) on the best candidates, in order, up to max_posthoc_candidates checks. A candidate of the same k within
 * posthoc_cluster_deg of theta of an already checked one is skipped (not counted as a check).
 * @p robot is used as-is (not thread-safe) and left in an unspecified configuration.
 * @throws std::invalid_argument for invalid parameters or a null template.
 */
S3Result selectGraspS3(const haptic_dmp_learning::core::CubeSatelliteModel& model, RobotModel& robot,
                       const RobotModel::JointVector& q_start, const DemoReference& demo,
                       const Eigen::Vector3d& delta_g_demo, const S3Context& ctx, const S3Params& params);

}  // namespace core
}  // namespace satellite_grasp_planner
