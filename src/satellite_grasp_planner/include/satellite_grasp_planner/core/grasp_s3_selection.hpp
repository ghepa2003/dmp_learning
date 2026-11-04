#pragma once

/**
 * @file grasp_s3_selection.hpp
 * @brief Reduced-problem grasp selection "S3": pick the (k, theta) whose grasp position is closest to the
 *        deformation of the single demo, on a single theta grid, with a manipulability term. A-priori mask: ik,
 *        trust radius, start-goal angle. The finalists (best of each k / wrist branch / theta cluster, in J_static order)
 *        get a kinematic rollout, the optional joint-margin and hand-satellite clearance constraints and the post-hoc
 *        velocity check; the first admissible one wins. No continuous roll search.
 *        Eigen + Pinocchio only, NO ROS.
 *
 *   d_pos    = || (goal_cand - p0) - delta_g_demo ||,   goal_cand = grasp position + contact_to_end_offset
 *              (contact_to_end_offset comes from the demo, see SelectionInputs)
 *   d        = sqrt(d_pos^2 / l^2 + orientation_weight * ori^2 / angle_scale^2)
 *   J_static = d * l * 1000 [mm] + kappa_w_mm * (-ln w_full_static)        (t_wait = 0; see s3CostV2)
 * wait_s is the delay of planLaunch() (launchDelay logic, with min_delay_s).
 */

#include <array>
#include <limits>
#include <functional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/pose_rotation.hpp"
#include "satellite_grasp_planner/core/approach_clearance.hpp"
#include "satellite_grasp_planner/core/dynamic_follow.hpp"
#include "satellite_grasp_planner/core/follow_margin.hpp"
#include "satellite_grasp_planner/core/grasp_pose_eval.hpp"
#include "satellite_grasp_planner/core/launch_plan.hpp"

namespace satellite_grasp_planner {
namespace core {

/// DynamicFollowParams as the selector uses them: the struct defaults (set A of the validation) with dt_s = 0.0005 s (out17: stable in 46
/// of 46 simulations at 0.5 and 1 ms; 0.00025 s stays selectable through s1.follow_dt_s).
inline DynamicFollowParams defaultSelectorDynamicFollowParams() {
  DynamicFollowParams p;
  p.dt_s = 0.0005;
  return p;
}

struct S3Params {
  double length_scale_m = 0.10;
  double orientation_weight = 0.0;
  double angle_scale_rad = 1.0;
  double r_trust_m = 0.30;
  double delta_v_max_m_s = 3e-3;
  double theta_step_deg = 2.0;
  IkParams ik;
  bool posthoc_velocity_check = true;
  int max_posthoc_candidates = 10;
  /// A candidate of the same k and wrist branch within this many degrees of theta of a finalist already kept is not a
  /// finalist (near-duplicates). 0 disables the clustering.
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

  // ---- selection (J_static / J) ----
  /// psi = 0 in the scans; a-priori constraints are ik (+ joint limits), r_trust and max_start_goal_deg; finalists are the
  /// best of each (k, wrist branch, theta cluster of posthoc_cluster_deg) by J_static, at most finalists_n; each gets a
  /// kinematic rollout with nullspace target rollout_q_ref (the rollout is the path check);
  /// J = d_mm [+ kappa_w_mm * (-ln w_full(q_final))] + kappa_t_mm_s * t_wait; among the finalists with
  /// J <= J_min + tau_j_mm the smallest t_wait wins; the post-hoc velocity check then runs on the winner (next best
  /// if it fails). With force the single forced candidate is the only finalist; force_psi_deg is honoured.
  /// A-priori: reject a candidate whose chosen-branch angle to FK(q_start) (dist_to_demo.ori_rad) exceeds this
  /// [deg]. 180 = disabled (the data cover only 16.5 and 34.5 deg: success, 179.6 deg: failure). In [0, 180].
  double max_start_goal_deg = 180.0;
  int finalists_n = 12;  ///< >= 1
  /// Nullspace target of the rollout [rad], 7 values; MUST be given (non-empty): validateS3Params throws otherwise, no
  /// fallback. It must equal the nullspace target of the real controller (its joint configuration at activation).
  std::vector<double> rollout_q_ref;
  bool use_w = true;          ///< false -> J = d_mm [+ kappa_t * t_wait] (no w term)
  double kappa_w_mm = 7.5;    ///< weight of -ln(w_full) [mm], ASSUMED, >= 0
  double kappa_t_mm_s = 0.0;  ///< weight of the wait [mm/s], >= 0
  double tau_j_mm = 2.0;      ///< tie band on J [mm], ASSUMED, >= 0

  // ---- v2 clearance constraint (docs/S3_SPEC.md, "vincolo di clearance"); with use_clearance == false (default)
  //      nothing below is read and the selection is bit-identical ----
  /// A finalist whose clearance (approachClearance(), commanded TCP
  /// trajectory vs rotating satellite, window [0, tau - 2 s]) is below min_clearance_mm is discarded. Lazy: the finalists are
  /// scanned in J order; the first admissible one fixes J*; the others with J <= J* + tau_j_mm are evaluated too;
  /// among the admissible ones the smaller wait wins. No admissible finalist: kNoCandidate (no fallback). With
  /// force the constraint only warns (S3Result::warnings). Needs S3Context::clearance_eval.
  bool use_clearance = false;
  double min_clearance_mm = 2.0;  ///< ASSUMED threshold [mm], finite and >= 0
  double clearance_dt_s = 0.05;   ///< sampling of the window [s], finite and > 0
  /// Finger joint position of the open gripper [m], both fingers, in [0, 0.08]. MUST equal the executor's
  /// gripper_open_position (haptic_dmp_learning/config/params.yaml:93, prodmp_gazebo_executor_node, value 0.06): the planner cannot read it.
  double gripper_open_m = 0.060;

  // ---- v2 joint-margin constraint (docs/S3_SPEC.md §10, criterion 11); with use_margin == false (default) nothing
  //      below is read and the selection is bit-identical ----
  /// A finalist whose predicted joint margin over [start, AT_END + margin_follow_s]
  /// (followMargin(): the G16 / step 0c procedure, nullspace reference rollout_q_ref, seed q_start) is below min_margin_deg is
  /// discarded. LAZY, in J_static order: for each finalist rollout (w, margin), then margin, then clearance (if use_clearance),
  /// then the post-hoc velocity check; the first finalist satisfying all of them is selected and no further rollout is done
  /// (so the wait tie-break within tau_j_mm of the eager selection does NOT apply in this mode). None admissible:
  /// kNoCandidate (no fallback). With force the constraint only warns (S3Result::warnings).
  bool use_margin = false;
  /// flusso pigro in ordine di J statico, senza margine ne' L2: per ogni finalista rollout, V1' (se use_hold_end), clearance (se
  /// use_clearance), controllo di velocita'; vince il primo che li supera tutti. Falso (default): nessun effetto. Con force: ignorato.
  bool lazy_static_order = false;
  /// ASSUMED threshold [deg], finite and >= 0 (observed G16 interval: failed 13.26-20.84, succeeded 28.63-35.00; 24 lies in the gap).
  double min_margin_deg = 24.0;
  /// Duration of the simulated follow after AT_END [s], finite and >= 0. 3.0 = executor's post_grasp_follow_s (and the G16 runs).
  double margin_follow_s = 3.0;
  /// Integration step of the follow [s], finite and > 0. 0.01 = step0c_margins.cpp (the G16 / criterion-11 values).
  double margin_follow_dt_s = 0.01;

  // ---- L2 dynamic follow (core/dynamic_follow.hpp). With use_dynamic_follow == false (default) nothing below is read and the
  //      selection is bit-identical. ----
  /// REPLACES use_margin / followMargin (use_margin and use_dynamic_follow together = validation error). Same LAZY flow as
  /// use_margin (J_static order, first admissible wins, no eager branch). Per finalist: kinematic rollout (V1: joint limits, also the
  /// source of the AT_END time), then the clearance (V2, if use_clearance; evaluated BEFORE L2 on the kinematic trajectory, as a
  /// pre-filter), then ONE continuous L2 forward-dynamics run of the arm under the Cartesian impedance law over the whole path:
  /// the approach from the trigger (t = 0) to AT_END and the hold up to AT_END + margin_follow_s, then the post-hoc velocity check
  /// (which is also run as an analytical pre-check before L2, so that a finalist failing it never costs an L2 run).
  /// L2 inputs: q0 = q_start (the robot's current configuration), dq0 = 0 (ASSUMED: the robot is at rest when the plan starts),
  /// nullspace target = q_start (the controller's is its q at activation, cartesian_impedance_controller.cpp:313), fingers at follow_finger_pos_m, target = the commanded TCP trajectory (timedApproachTrajectory:
  /// goal = grasp position + contact_to_end_offset, zero-order hold) followed, from AT_END, by the grasp pose co-rotating with
  /// the satellite. The finalist is admissible iff the minimum over the approach AND the hold of min(q - lo, hi - q) over the 7 joints
  /// of the L2 trajectory is >= margin_delta_deg and L2 did not diverge (divergence_rad guard). With force the criterion only warns
  /// (S3Result::warnings), as use_margin.
  bool use_dynamic_follow = false;
  /// ASSUMED threshold [deg], finite and >= 0 (validateS3Params). Used only if use_dynamic_follow.
  double margin_delta_deg = 5.0;
  /// L2 gains, plant, dt, fingers, dq0 (s1.follow_*). Defaults = set A of the validation, but dt_s = 0.0005 (see defaultSelectorDynamicFollowParams).
  DynamicFollowParams dynamic_follow = defaultSelectorDynamicFollowParams();

  // ---- V1' hold-end IK constraint (decision 28/10). With use_hold_end == false (default) nothing
  //      below is read and the selection is unchanged. ----
  /// When true: both the grasp pose and the hold-end pose (+hold_follow_s co-rotating with the satellite)
  /// must be reachable by IK within joint limits. Finalists with end_ok == false are discarded with
  /// discard_reason = "hold_end_infeasible". For admissible finalists, effective manipulability
  /// w_eff = min(w_grasp, w_end) enters the cost. Can be combined with use_margin and with use_dynamic_follow (with L2 the order is rollout, clearance, V1', velocity pre-check, L2).
  /// NOTE: clearance and r_trust are NOT checked at hold end.
  bool use_hold_end = false;
  /// Duration of the simulated hold after grasp [s], finite and > 0. Must match the executor's post_grasp_follow_s.
  double hold_follow_s = 3.0;
};

/// Result of the hold-end IK evaluation (holdPoseIk).
struct HoldPoseIkResult {
  bool end_ok = false;  ///< IK converged and final q strictly within joint limits
  haptic_dmp_learning::core::RotatablePose hold_pose;  ///< grasp pose rotated by omega * hold_follow_s about satellite axis
  RobotModel::JointVector q_hold_end = RobotModel::JointVector::Zero();  ///< joint solution at hold end
  double w_end = 0.0;  ///< fullManipulability at q_hold_end (0.0 if not end_ok)
  IkResult ik;         ///< raw IK result
};

/**
 * @brief Computes the hold-end pose rotated by omega * hold_follow_s about the satellite axis,
 *        solves IK with seed and nullspace reference = q_seed (the static grasp IK solution),
 *        and evaluates manipulability w_end.
 *        Position uses the static IK convention (position_world, without contact_to_end_offset).
 *        NOTE: Clearance and r_trust are NOT checked at the hold-end pose.
 */
HoldPoseIkResult holdPoseIk(RobotModel& robot,
                            const Eigen::Vector3d& grasp_position,
                            const Eigen::Quaterniond& grasp_orientation,
                            const Eigen::Vector3d& satellite_center,
                            const Eigen::Vector3d& satellite_axis,
                            double omega_rad_s,
                            double hold_follow_s,
                            const RobotModel::JointVector& q_seed,
                            const IkParams& ik_params = IkParams());

/// A-priori rejection bits (selectGraspS3 cascade order: ik, r_trust, start_goal). A candidate passes iff the
/// mask is 0. The mask holds ALL failing constraints; S3Candidate::reject_reason keeps only the first one.
/// Bits 2 and 4 (w_hat, static margin of the removed legacy selector) are retired and never set; 16..64 are the landscape
/// tool diagnostics.
enum S3RejectBit : unsigned { kRejectIk = 1u,
                              kRejectRTrust = 8u,
                              kRejectStartGoal = 128u  ///< max_start_goal_deg
};

/// One row of the optional per-candidate log (S3Context::eval_log): one per evaluateGraspPose call of the scan, in
/// call order. Filled with the same values the selection uses; wait_s / j_static are computed for EVERY row
/// (S3Candidate::wait_s is 0 for rejected candidates, the log is not).
struct S3EvalRecord {
  haptic_dmp_learning::core::GraspPointId k = haptic_dmp_learning::core::GraspPointId::kP0;
  double theta_rad = 0.0;
  double psi_rad = 0.0;
  unsigned reject_mask = 0;   ///< S3RejectBit
  bool passes_constraints = false;
  bool wrist_flip = false;
  double d_pos_m = 0.0;
  double d = 0.0;
  double wait_s = 0.0;
  double j_static = 0.0;      ///< J_static of the row (d_mm + kappa_w * (-ln w_full_static), t_wait = 0; +inf if w_full <= 0 with use_w)
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
  /// Per-joint <dynamics> of the URDF (damping, friction) for the L2 follow. Required (non-null, outliving the call) iff
  /// use_dynamic_follow; ignored otherwise. Not owned.
  const JointDynamics* joint_dynamics = nullptr;
};

struct S3Candidate {
  PoseEvaluation eval;
  double d_pos_m = 0.0;
  double d = 0.0;
  double wait_s = 0.0;              ///< 0 if the candidate did not pass the constraints
  /// J of the candidate (meaningful only if passes_constraints): J_static (t_wait = 0) when it comes out of the scan, which
  /// is the order of the finalists; once a finalist has been rolled out and found feasible, the J of the rollout
  /// (d_mm + kappa_w * (-ln w_rollout) + kappa_t * t_wait, S3FinalistRecord::j). The chosen candidate carries the latter.
  double cost = 0.0;
  double psi_rad = 0.0;             ///< roll about z_tcp of this candidate (== eval.psi_rad; 0 in the scans)
  unsigned reject_mask = 0;         ///< S3RejectBit mask of the a-priori constraints (all failing ones)
  bool passes_constraints = false;  ///< a-priori constraints; a post-hoc failure sets reject_reason only
  std::string reject_reason;        ///< "ik", "r_trust", "start_goal" or a post-hoc reason
  double w_full_static = 0.0;       ///< w_full at the static IK solution of the chosen branch
};

/// One finalist of phase 2/3 (the rollout columns are meaningful only if rollout_done).
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
  /// Why the finalist was discarded in the lazy (use_margin / use_dynamic_follow) flow: "rollout" (joint-limit violation), "margin",
  /// "dynamic_approach" (L2 approach margin < margin_delta_deg), "dynamic_hold" (approach ok, L2 hold margin < margin_delta_deg or no
  /// usable AT_END state), "dynamic_diverged" (L2 non-finite or beyond divergence_rad; takes precedence), "clearance", "velocity";
  /// empty if not discarded / not evaluated. Empty in the eager flow.
  std::string discard_reason;
  // ---- L2 dynamic follow (additive; filled only if S3Params::use_dynamic_follow and the finalist reached that stage). One
  //      continuous L2 run from q_start (dq = 0) at the trigger to AT_END + margin_follow_s: the state at AT_END is the L2 one, not the
  //      kinematic rollout's. Angles in degrees, joints 0-based. ----
  bool dyn_done = false;
  bool dyn_rejected = false;    ///< evaluated and total margin < margin_delta_deg, or diverged (with force: only warned)
  bool dyn_diverged = false;
  double dyn_t_div_s = 0.0;     ///< meaningful iff dyn_diverged: time relative to AT_END (negative = during the approach)
  double dyn_at_end_s = 0.0;    ///< AT_END on the L2 clock (time since the trigger, snapped to the L2 grid)
  double dyn_approach_margin_deg = std::numeric_limits<double>::quiet_NaN();  ///< L2 minimum over the approach [0, AT_END]
  int dyn_approach_joint = -1;                                                ///< its joint
  double dyn_approach_t_rel_s = 0.0;                                          ///< its time relative to AT_END (<= 0)
  double dyn_hold_margin_deg = std::numeric_limits<double>::quiet_NaN();      ///< L2 minimum over the hold [AT_END, AT_END + margin_follow_s]
  int dyn_hold_joint = -1;                                                    ///< its joint
  double dyn_hold_t_rel_s = 0.0;                                              ///< its time after AT_END (>= 0)
  double dyn_total_margin_deg = std::numeric_limits<double>::quiet_NaN();     ///< min of the two; compared with margin_delta_deg
  RobotModel::JointVector dyn_q_end_deg = RobotModel::JointVector::Constant(std::numeric_limits<double>::quiet_NaN());  ///< L2 q at AT_END
  RobotModel::JointVector dyn_q_h1p5_deg = RobotModel::JointVector::Constant(std::numeric_limits<double>::quiet_NaN());  ///< L2 q at AT_END + 1.5 s
  RobotModel::JointVector dyn_q_h3_deg = RobotModel::JointVector::Constant(std::numeric_limits<double>::quiet_NaN());    ///< L2 q at AT_END + 3.0 s
  double dyn_elapsed_ms = 0.0;  ///< wall-clock time of the L2 evaluation (target construction + simulation + margins)
  // Diagnostics of the L2 evaluation (they change no decision). NaN until computed.
  double dyn_pose_err_pos_t0_m = std::numeric_limits<double>::quiet_NaN();    ///< |FK(L2 q at AT_END) - target(AT_END)| position [m]
  double dyn_pose_err_rot_t0_deg = std::numeric_limits<double>::quiet_NaN();  ///< geodesic angle FK(L2 q at AT_END) vs target(AT_END) [deg]
  double dyn_max_abs_dq_from_qend_deg = std::numeric_limits<double>::quiet_NaN();  ///< max over joints and hold rows of |q - L2 q at AT_END|
  double dyn_t_first_exceed_s = std::numeric_limits<double>::quiet_NaN();  ///< first hold row with a joint > 360 deg from L2 q at AT_END; -1 if none
  double dyn_initial_rot_err_deg = std::numeric_limits<double>::quiet_NaN();  ///< geodesic angle FK(q_start) vs target(0) [deg]; a warning is added if >= 3
  double dyn_vs_rollout_qend_deg = std::numeric_limits<double>::quiet_NaN();  ///< max_j |L2 q at AT_END - kinematic rollout q_end|
  // ---- V1' hold-end IK constraint (additive; filled only if S3Params::use_hold_end). ----
  // NOTE: clearance and r_trust are NOT checked at hold end.
  bool hold_end_done = false;
  bool end_ok = false;
  RobotModel::JointVector q_hold_end = RobotModel::JointVector::Zero();
  haptic_dmp_learning::core::RotatablePose hold_pose;
  double w_end = 0.0;
  double w_eff = 0.0;
  // Wall-clock time of the V1 rollout and of the V1' IK [ms] (0 if the stage did not run). Diagnostic only.
  double rollout_ms = 0.0;
  double hold_end_ms = 0.0;
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
  std::vector<S3Candidate> ranked;          ///< the rollout-feasible finalists by J (first 20, post-hoc annotated)
  std::array<S3Candidate, 4> best_per_k;    ///< best J_static of each k (a rejected one, with its reason, if none passes)
  int evaluations = 0;                      ///< evaluateGraspPose calls
  int posthoc_checks = 0;                   ///< candidates examined by the post-hoc stage
  double posthoc_dv_m_s = 0.0;              ///< |v_target - v_ee| of the chosen candidate (0 if not checked)
  double posthoc_min_margin_rad = 0.0;      ///< min joint margin along the path of the chosen one (0 if not checked)
  std::vector<S3PosthocEntry> posthoc_log;  ///< one entry per post-hoc check, in order
  std::string note;
  bool forced = false;                         ///< S3Params::force was on
  std::string forced_violations;               ///< force_ignore_constraints: constraints violated and ignored (empty if none)
  std::vector<S3FinalistRecord> v2_finalists;  ///< finalists in J_static order
  double clearance_stage_ms = 0.0;             ///< wall-clock time spent in the clearance evaluations (0 if use_clearance is off)
  std::vector<std::string> warnings;           ///< use_clearance / use_margin with force: below threshold (not discarded)
  int rollouts_done = 0;                       ///< kinematic rollouts of finalists performed (eager: all finalists; use_margin: up to the first admissible)
  int margin_evals_done = 0;                   ///< use_margin: followMargin() evaluations performed
  int dynamic_evals_done = 0;                  ///< use_dynamic_follow: L2 evaluations performed (finalists that reached L2)
  double dynamic_stage_ms = 0.0;               ///< use_dynamic_follow: wall-clock time of all L2 evaluations
  /// Wall-clock stage times [ms], diagnostic only. In the lazy flow the stages are interleaved per finalist (V1 rollout, V2 clearance,
  /// V1', L2, velocity): each stage total is the SUM of the time of that stage over all the finalists examined.
  /// scan_ms: candidate generation, static evaluation and the J_static sort (collectCandidates + stable_sort). posthoc_ms: post-hoc
  /// velocity stage (the ProDMP-at-contact setup, plus every velocity check, also the L2 pre-check); 0 if posthoc_velocity_check is
  /// off. The V1 and V1' totals are the sums of S3FinalistRecord::rollout_ms / hold_end_ms over v2_finalists.
  double scan_ms = 0.0;
  double posthoc_ms = 0.0;
  /// use_dynamic_follow: finalists discarded per constraint, to decide the order of the constraints. Each discarded finalist is
  /// counted once, in the first constraint that discarded it (V1 rollout, V2 clearance, V1' hold-end IK, velocity, L2 approach, L2 hold, L2 divergence).
  struct DynamicCounters {
    int rolled_out = 0;           ///< finalists that got a kinematic rollout
    int discarded_v1_rollout = 0;
    int discarded_v2_clearance = 0;
    int discarded_v1p_hold_end = 0;  ///< V1' (use_hold_end): hold-end IK infeasible; counted before the velocity check and L2
    int evaluated_l2 = 0;         ///< = dynamic_evals_done
    int discarded_l2_approach = 0;
    int discarded_l2_hold = 0;
    int discarded_l2_diverged = 0;
    int discarded_velocity = 0;
  } dyn_counters;
  /// use_dynamic_follow + use_clearance, winner only: clearance recomputed on the TCP path of the L2 run (FK of the L2 q sampled every
  /// kDefaultSimDtS up to AT_END, same evaluator and window as the constraint). Reported, never used as a constraint. NaN if not computed.
  double winner_l2_clearance_mm = std::numeric_limits<double>::quiet_NaN();
  std::string winner_l2_clearance_pair;
  double winner_l2_clearance_t_rel_s = 0.0;
  // Additive hold-end outputs (populated when s1.use_hold_end is true and a winner is chosen)
  bool hold_end_done = false;
  bool end_ok = false;
  RobotModel::JointVector q_hold_end = RobotModel::JointVector::Zero();
  haptic_dmp_learning::core::RotatablePose hold_pose;
  double w_end = 0.0;
  double w_eff = 0.0;
};

/// Parameter checks done by selectGraspS3 (finite, positive scales/steps, non-negative weights; force_*, branch_rule: see S3Params).
/// @throws std::invalid_argument.
void validateS3Params(const S3Params& params);

/// Circular distance of two thetas [rad], in [0, pi]: fmod(|a - b|, 2*pi), folded to 2*pi - d above pi. Used by the
/// post-hoc cluster rule of selectGraspS3 and by analysis/s3_landscape.cpp (--selftest) to replay it.
double thetaDistRad(double a, double b);

/// d = sqrt(d_pos^2/l^2 + orientation_weight * ori^2/angle_scale^2). @throws std::invalid_argument for
/// non-positive/non-finite scales or a negative weight.
double s3Distance(double d_pos_m, double ori_rad, double length_scale_m, double orientation_weight,
                  double angle_scale_rad);

/// A-priori constraint mask of one evaluated candidate (see S3RejectBit): ik (= !feasible_basic), r_trust
/// (= !(d_pos <= r_trust_m); NaN rejects), and kRejectStartGoal when max_start_goal_deg < 180 and the chosen branch's
/// dist_to_demo.ori_rad exceeds it. The cascade of selectGraspS3 calls this function.
unsigned s3AprioriRejectMaskV2(const PoseEvaluation& eval, double d_pos_m, const S3Params& params);

/// Cost: d_mm + (use_w ? kappa_w_mm * (-ln w_full) : 0) + kappa_t_mm_s * t_wait_s. w_full <= 0 or NaN with use_w:
/// +infinity (a singular candidate never beats a regular one).
double s3CostV2(double d_mm, double w_full, double t_wait_s, const S3Params& params);

/// Tie-break: index of the entry with the smallest t_wait among those with J <= J_min + tau_j_mm (J_min over the
/// finite entries; ties on t_wait: smaller J, then smaller index). -1 if @p j is empty or has no finite entry.
/// @throws std::invalid_argument if the vectors differ in size or tau_j_mm < 0.
int s3PickWinnerV2(const std::vector<double>& j, const std::vector<double>& t_wait_s, double tau_j_mm);

/**
 * One round of the lazy clearance filter (pure; selectGraspS3 calls it before every tie-break pick).
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
 * Scans k in {0,90,180,270} x theta in [-180, 180) deg on a single grid (theta_step_deg, no refinement), orders the
 * constraint-passing candidates by J_static (ties: k, theta, wrist_flip ascending), takes the finalists (see
 * S3Params::finalists_n, posthoc_cluster_deg) and runs on them, in order, the kinematic rollout, the optional joint-margin and
 * clearance constraints and the post-hoc velocity check (relative velocity |omega x r - v_ProDMP| <= delta_v_max_m_s), up to
 * max_posthoc_candidates checks. See S3Params for the exact flow.
 * @p robot is used as-is (not thread-safe) and left in an unspecified configuration.
 * @throws std::invalid_argument for invalid parameters or a null template.
 */
S3Result selectGraspS3(const haptic_dmp_learning::core::CubeSatelliteModel& model, RobotModel& robot,
                       const RobotModel::JointVector& q_start, const DemoReference& demo,
                       const Eigen::Vector3d& delta_g_demo, const S3Context& ctx, const S3Params& params);

}  // namespace core
}  // namespace satellite_grasp_planner
