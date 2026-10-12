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
 *           (the same goal the legacy scan hands to the ProDMP: ScanParams::contact_to_end_offset)
 *   d     = sqrt(d_pos^2 / l^2 + orientation_weight * ori^2 / angle_scale^2)
 *   cost  = d + epsilon_wait_per_s * wait_s
 * wait_s is the delay of planLaunch() (same launchDelay logic and min_delay_s as the legacy planner).
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
};

/// Everything besides the robot/pose evaluation that selectGraspS3 needs (the data selectGrasp takes from
/// SelectionParams/SelectionInputs).
struct S3Context {
    const haptic_dmp_learning::core::ProDMP* prodmp_template = nullptr;  ///< required, not modified
    Eigen::Vector3d contact_to_end_offset = Eigen::Vector3d::Zero();     ///< ScanParams::contact_to_end_offset
    double tau_contact_s = 0.0;  ///< <= 0: the template's tau (as ScanParams); used for the velocity check
    double tau_launch_s = 0.0;   ///< > 0, forwarded to planLaunch() (see planGraspAndLaunch)
    SatelliteSnapshot snapshot;  ///< omega, center, axis, theta/t of the acquisition
    double t_now_s = 0.0;        ///< instant the selection ends (as planGraspAndLaunch's t_now_s)
    double min_delay_s = 0.0;
};

struct S3Candidate {
    PoseEvaluation eval;
    double d_pos_m = 0.0;
    double d = 0.0;
    double wait_s = 0.0;  ///< 0 if the candidate did not pass the constraints
    double cost = 0.0;    ///< d + epsilon * wait_s (meaningful only if passes_constraints)
    bool passes_constraints = false;  ///< a-priori constraints; a post-hoc failure sets reject_reason only
    std::string reject_reason;        ///< "ik", "w_hat", "margine", "r_trust" or a post-hoc reason
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
    std::string note;
};

/// Parameter checks done by selectGraspS3 (finite, positive scales/steps, non-negative weights).
/// @throws std::invalid_argument.
void validateS3Params(const S3Params& params);

/// d = sqrt(d_pos^2/l^2 + orientation_weight * ori^2/angle_scale^2). @throws std::invalid_argument for
/// non-positive/non-finite scales or a negative weight.
double s3Distance(double d_pos_m, double ori_rad, double length_scale_m, double orientation_weight,
                  double angle_scale_rad);

/**
 * Scans k in {0,90,180,270} x theta in [-180, 180) deg (theta_step_deg), refines the best accepted of each k
 * with theta_refine_step_deg in +-theta_step_deg, ranks by cost and runs the post-hoc checks (relative
 * velocity |omega x r - v_ProDMP| <= delta_v_max_m_s; kinematic path via checkKinematicFeasibility with the
 * orientation of the chosen branch) on the best candidates, in order, up to max_posthoc_candidates.
 * @p robot is used as-is (not thread-safe) and left in an unspecified configuration.
 * @throws std::invalid_argument for invalid parameters or a null template.
 */
S3Result selectGraspS3(const haptic_dmp_learning::core::CubeSatelliteModel& model, RobotModel& robot,
                       const RobotModel::JointVector& q_start, const DemoReference& demo,
                       const Eigen::Vector3d& delta_g_demo, const S3Context& ctx, const S3Params& params);

}  // namespace core
}  // namespace satellite_grasp_planner
