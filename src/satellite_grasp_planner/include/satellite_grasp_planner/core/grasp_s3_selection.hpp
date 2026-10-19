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
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
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
};

/// A-priori rejection bits (selectGraspS3 cascade order: ik, w_hat, margin, r_trust). A candidate passes iff the
/// mask is 0. The mask holds ALL failing constraints; S3Candidate::reject_reason keeps only the first one.
enum S3RejectBit : unsigned { kRejectIk = 1u, kRejectWHat = 2u, kRejectMargin = 4u, kRejectRTrust = 8u };

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
    double margin_rad = 0.0;    ///< min joint margin of the IK solution of the chosen branch
    BranchDetail branch[2];     ///< both branches (manipulability fields filled)
};

/// Everything besides the robot/pose evaluation that selectGraspS3 needs (see SelectionInputs).
struct S3Context {
    const haptic_dmp_learning::core::ProDMP* prodmp_template = nullptr;  ///< required, not modified
    Eigen::Vector3d contact_to_end_offset = Eigen::Vector3d::Zero();     ///< SelectionInputs::contact_to_end_offset
    double tau_contact_s = 0.0;  ///< <= 0: the template's tau; used for the velocity check
    double tau_launch_s = 0.0;   ///< > 0, forwarded to planLaunch() (see planGraspAndLaunchS1)
    SatelliteSnapshot snapshot;  ///< omega, center, axis, theta/t of the acquisition
    double t_now_s = 0.0;        ///< instant the selection ends (as planGraspAndLaunchS1's t_now_s)
    double min_delay_s = 0.0;
    /// Optional per-candidate log (default nullptr = off, no change of behaviour or result). Not owned.
    std::vector<S3EvalRecord>* eval_log = nullptr;
};

struct S3Candidate {
    PoseEvaluation eval;
    double d_pos_m = 0.0;
    double d = 0.0;
    double wait_s = 0.0;  ///< 0 if the candidate did not pass the constraints
    double cost = 0.0;    ///< d + epsilon * wait_s (meaningful only if passes_constraints)
    double psi_rad = 0.0;             ///< roll about z_tcp of this candidate (== eval.psi_rad; 0 in the scans)
    unsigned reject_mask = 0;         ///< S3RejectBit mask of the a-priori constraints (all failing ones)
    bool passes_constraints = false;  ///< a-priori constraints; a post-hoc failure sets reject_reason only
    std::string reject_reason;        ///< "ik", "w_hat", "margine", "r_trust" or a post-hoc reason
};

/// One post-hoc check (clustering skips are not logged). Vectors are zero when the velocity check is off.
struct S3PosthocEntry {
    haptic_dmp_learning::core::GraspPointId k = haptic_dmp_learning::core::GraspPointId::kP0;
    double theta_rad = 0.0;
    double wait_s = 0.0;
    double dv_m_s = 0.0;                 ///< |v_target - v_ee|
    Eigen::Vector3d v_ee = Eigen::Vector3d::Zero();      ///< ProDMP velocity at contact for this goal
    Eigen::Vector3d v_target = Eigen::Vector3d::Zero();  ///< surface velocity of the grasp point
    bool path_checked = false;           ///< false if the path check is off or the velocity check failed first
    bool path_ok = false;                ///< meaningful only if path_checked
    double min_margin_rad = 0.0;         ///< min joint margin along the path (0 if not path_checked or violated)
    bool accepted = false;               ///< passed every enabled post-hoc check
    std::string reject_reason;           ///< empty if accepted
};

struct S3Result {
    enum Status { kSelected, kNoCandidate, kPosthocFailed };
    Status status = kNoCandidate;
    S3Candidate chosen;  ///< valid only when status == kSelected
    std::vector<S3Candidate> ranked;  ///< first 20 constraint-passing candidates by cost (post-hoc annotated)
    std::array<S3Candidate, 4> best_per_k;  ///< best of each k (a rejected one, with its reason, if none passes)
    int evaluations = 0;       ///< evaluateGraspPose calls
    double max_w_hat_passing = 0.0;  ///< max w_hat over ALL constraint-passing candidates (not just ranked)
    int posthoc_checks = 0;    ///< candidates examined by the post-hoc stage
    double posthoc_dv_m_s = 0.0;          ///< |v_target - v_ee| of the chosen candidate (0 if not checked)
    double posthoc_min_margin_rad = 0.0;  ///< min joint margin along the path of the chosen one (0 if not checked)
    std::vector<S3PosthocEntry> posthoc_log;  ///< one entry per post-hoc check, in order
    std::string note;
    bool forced = false;             ///< S3Params::force was on
    std::string forced_violations;   ///< force_ignore_constraints: constraints violated and ignored (empty if none)
};

/// Parameter checks done by selectGraspS3 (finite, positive scales/steps, non-negative weights; force_*: see S3Params).
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
