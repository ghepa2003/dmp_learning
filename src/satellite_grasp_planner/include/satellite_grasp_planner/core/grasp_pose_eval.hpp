#pragma once

/**
 * @file grasp_pose_eval.hpp
 * @brief Per-candidate evaluation of a grasp pose (k, theta) for the reduced problem (single demo, no
 *        collision, velocity only as a-posteriori check): static pose IK with gripper-symmetry branch,
 *        joint-limit margins, translational manipulability and SE(3) distance to the demo end pose.
 *        Eigen + Pinocchio (via RobotModel) only, NO ROS.
 *
 * Frames: poses are expressed in the RobotModel base frame (fer_link0), which coincides with the world
 * frame in the planner (robot_base_world = 0). The RobotModel end-effector frame must be fer_hand_tcp.
 */

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"

namespace satellite_grasp_planner {
namespace core {

using franka_cartesian_control::core::RobotModel;

struct IkParams {
    double pos_tol_m = 1e-4;
    double ori_tol_rad = 1e-3;
    int max_iter = 300;
    double damping = 0.05;         ///< DLS lambda of the primary (pose) task
    double nullspace_gain = 1.0;   ///< nullspace pull toward q_ref (per iteration, step = 1)
    double max_step_rad = 0.2;     ///< cap on max_j |dq_j| per iteration
};

struct IkResult {
    bool converged = false;  ///< pose reached within tolerances AND limits_ok
    RobotModel::JointVector q = RobotModel::JointVector::Zero();
    double pos_err_m = 0.0;
    double ori_err_rad = 0.0;
    int iterations = 0;       ///< number of integration steps taken
    bool limits_ok = false;   ///< final q inside [lower, upper] (guaranteed by per-step clamping)
};

/**
 * Static pose IK: iterates NullspaceBiasedResolution (unit time step, kp = 0.5, saturations disabled,
 * per-iteration step capped by max_step_rad) from q_seed, nullspace pull toward q_ref, clamping q to the
 * URDF joint limits at every step. The pose is the one of the RobotModel end-effector frame.
 * `converged` is true only if the pose is reached within the tolerances with q inside the limits.
 */
IkResult solvePoseIk(RobotModel& robot, const Eigen::Vector3d& position,
                     const Eigen::Quaterniond& orientation, const RobotModel::JointVector& q_seed,
                     const RobotModel::JointVector& q_ref, const IkParams& params = IkParams());

struct JointMargins {
    RobotModel::JointVector margin_rad = RobotModel::JointVector::Zero();  ///< min(q - lo, hi - q) per joint
    double min_margin_rad = 0.0;
    int worst_joint = 0;  ///< 0-based index of the smallest margin
};

JointMargins jointMargins(const RobotModel& robot, const RobotModel::JointVector& q);

/// Translational manipulability sqrt(det(Jt Jt^T)) at q (calls robot.update(q, 0)).
double wTransAt(RobotModel& robot, const RobotModel::JointVector& q);

struct Se3Distance {
    double d = 0.0;
    double pos_m = 0.0;
    double ori_rad = 0.0;  ///< geodesic angle in [0, pi], invariant to q / -q
};

/// d = sqrt(pos^2 / length_scale^2 + ori^2 / angle_scale^2). Scales must be finite and > 0.
Se3Distance se3Distance(const Eigen::Vector3d& p1, const Eigen::Quaterniond& q1,
                        const Eigen::Vector3d& p2, const Eigen::Quaterniond& q2,
                        double length_scale_m, double angle_scale_rad);

struct DemoReference {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    double w_trans_demo = 0.0;
};

/// FK of the end-effector frame plus w_trans at the demo's final joints.
DemoReference demoReferenceFromEndJoints(RobotModel& robot, const RobotModel::JointVector& end_joints_rad);

struct PoseEvaluation {
    haptic_dmp_learning::core::GraspPointId k = haptic_dmp_learning::core::GraspPointId::kP0;
    double theta_rad = 0.0;
    double psi_rad = 0.0;     ///< deliberate roll about the local z_tcp (0 for evaluateGraspPose)
    haptic_dmp_learning::core::GraspTargetPose pose;  ///< nominal pose from CubeSatelliteModel::graspPoseAt
    bool wrist_flip = false;  ///< the chosen branch is q_nominal * Rz(pi) (inner/outer finger swapped)
    IkResult ik;
    double w_trans = 0.0;     ///< at ik.q
    double w_hat = 0.0;       ///< w_trans / demo.w_trans_demo
    JointMargins margins;     ///< at ik.q
    Se3Distance dist_to_demo; ///< target pose of the CHOSEN branch vs the demo end pose
    bool feasible_basic = false;  ///< ik.converged && ik.limits_ok (no w_hat / margin thresholds here)
};

/// Orientation of the CHOSEN branch of @p eval: applyRoll(q_nominal, psi + (wrist_flip ? pi : 0)). Rolls about the same
/// axis (local z_tcp) commute, so this equals applyRoll(applyRoll(qn, psi), pi). For psi == 0 and no flip it returns
/// the nominal quaternion unmodified (bit-identical to the pre-psi code).
Eigen::Quaterniond goalOrientation(const PoseEvaluation& eval);

/// Per-branch diagnostics of evaluateGraspPoseDetailed (branch 0 = nominal orientation, 1 = nominal * Rz(pi)).
/// The manipulability fields are filled only when compute_extra is true.
struct BranchDetail {
    bool converged = false;
    bool limits_ok = false;
    RobotModel::JointVector q = RobotModel::JointVector::Zero();
    double pos_err_m = 0.0;
    double ori_err_rad = 0.0;
    int iterations = 0;
    double min_margin_rad = 0.0;
    int worst_joint = 0;
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();  ///< the target orientation of this branch
    double w_trans = 0.0, w_rot = 0.0, w_full = 0.0, smin_trans = 0.0, smin_rot = 0.0;
};

struct PoseEvaluationDetail {
    PoseEvaluation eval;       ///< what evaluateGraspPose returns (chosen branch)
    BranchDetail branch[2];
    int selected = 0;          ///< index of the chosen branch (== eval.wrist_flip)
};

/**
 * Same computation as evaluateGraspPose (which is a wrapper of this function with psi = 0, compute_extra = false),
 * with a deliberate roll offset psi_rad about the LOCAL z of the TCP (the approach axis z_tcp = -R n_body, see
 * CubeSatelliteModel::graspPoseAt): the branch orientations are applyRoll(q_nominal, psi) and
 * applyRoll(q_nominal, pi + psi). The TCP position does not depend on psi. For psi == 0 the nominal orientation is
 * used unmodified, so the result is bit-identical to the previous evaluateGraspPose.
 * compute_extra additionally fills w_trans / w_rot / w_full / smin_* for BOTH branches (one extra Jacobian pass per
 * branch); the robot model is left updated at the chosen branch's q, as evaluateGraspPose leaves it.
 */
PoseEvaluationDetail evaluateGraspPoseDetailed(const haptic_dmp_learning::core::CubeSatelliteModel& model,
                                               haptic_dmp_learning::core::GraspPointId k, double theta_rad,
                                               double psi_rad, RobotModel& robot,
                                               const RobotModel::JointVector& q_start, const DemoReference& demo,
                                               double length_scale_m, double angle_scale_rad,
                                               const IkParams& params, bool compute_extra);

/**
 * Evaluates the grasp (k, theta): runs solvePoseIk for both gripper-equivalent orientations
 * (nominal and nominal * Rz(pi)), each seeded with q_start and q_ref = q_start, and keeps the better:
 * converged && limits_ok first; then larger min joint margin; within 1e-3 rad of equal margin, smaller
 * ||q - q_start||. If neither branch converges, the one with smaller position then orientation error wins.
 * @throws std::invalid_argument if demo.w_trans_demo <= 0 or the scales are not finite and > 0.
 */
PoseEvaluation evaluateGraspPose(const haptic_dmp_learning::core::CubeSatelliteModel& model,
                                 haptic_dmp_learning::core::GraspPointId k, double theta_rad,
                                 RobotModel& robot, const RobotModel::JointVector& q_start,
                                 const DemoReference& demo, double length_scale_m,
                                 double angle_scale_rad, const IkParams& params = IkParams());

}  // namespace core
}  // namespace satellite_grasp_planner
