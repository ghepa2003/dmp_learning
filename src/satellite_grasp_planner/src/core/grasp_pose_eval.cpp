#include "satellite_grasp_planner/core/grasp_pose_eval.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "franka_cartesian_control/core/cartesian_error.hpp"
#include "haptic_dmp_learning/core/math_utils.hpp"
#include "haptic_dmp_learning/core/tcp_roll.hpp"
#include "satellite_grasp_planner/core/joint_path_simulator.hpp"
#include "satellite_grasp_planner/core/manipulability.hpp"

namespace satellite_grasp_planner {
namespace core {

namespace {

constexpr double kMarginTieRad = 1e-3;
constexpr double kRotationTieRad = 1e-3;
/// Damping of the nullspace projector: small, so that N = I - J^# J is (nearly) an exact projector and the
/// nullspace pull does not leak into the task and bias the converged pose error.
constexpr double kProjectorDamping = 1e-3;
constexpr double kUnboundedSpeed = 1e9;

struct Branch {
  IkResult ik;
  JointMargins margins;
  double dq_norm = 0.0;
};

/// rot_a / rot_b: angle to the demo orientation of a / b, read only when rule == kRotation.
bool better(const Branch& a, const Branch& b, BranchRule rule, double rot_a, double rot_b) {
  if (a.ik.converged != b.ik.converged) return a.ik.converged;
  if (!a.ik.converged) {
    if (a.ik.pos_err_m != b.ik.pos_err_m) return a.ik.pos_err_m < b.ik.pos_err_m;
    return a.ik.ori_err_rad < b.ik.ori_err_rad;
  }
  if (rule == BranchRule::kRotation && std::abs(rot_a - rot_b) > kRotationTieRad) return rot_a < rot_b;
  if (std::abs(a.margins.min_margin_rad - b.margins.min_margin_rad) > kMarginTieRad) {
    return a.margins.min_margin_rad > b.margins.min_margin_rad;
  }
  return a.dq_norm < b.dq_norm;
}

}  // namespace

BranchRule parseBranchRule(const std::string& name) {
  if (name == "margin") return BranchRule::kMargin;
  if (name == "rotation") return BranchRule::kRotation;
  throw std::invalid_argument("branch_rule: unknown value '" + name + "' (expected 'margin' or 'rotation').");
}

IkResult solvePoseIk(RobotModel& robot, const Eigen::Vector3d& position,
                     const Eigen::Quaterniond& orientation, const RobotModel::JointVector& q_seed,
                     const RobotModel::JointVector& q_ref, const IkParams& params) {
  NullspaceBiasedResolution::Params ns;
  ns.ik_params.kp_linear = 0.5;  // unit time step: kp * dt = 0.5
  ns.ik_params.kp_angular = 0.5;
  ns.ik_params.damping_lambda = params.damping;
  ns.ik_params.max_linear_speed = kUnboundedSpeed;
  ns.ik_params.max_angular_speed = kUnboundedSpeed;
  ns.ik_params.max_joint_speed = kUnboundedSpeed;
  ns.nullspace_gain = params.nullspace_gain;
  ns.joint1_nullspace_gain_scale = 1.0;
  ns.nullspace_damping = kProjectorDamping;
  const NullspaceBiasedResolution strategy(ns);

  CartesianSample target;
  target.position = position;
  target.orientation = orientation.normalized();

  const RobotModel::JointVector lo = robot.jointLowerLimits();
  const RobotModel::JointVector hi = robot.jointUpperLimits();
  const RobotModel::JointVector zero = RobotModel::JointVector::Zero();

  IkResult res;
  RobotModel::JointVector q = q_seed.cwiseMax(lo).cwiseMin(hi);
  for (int it = 0;; ++it) {
    robot.update(q, zero);
    const auto err = franka_cartesian_control::core::computePoseError(
        robot.eePosition(), robot.eeOrientation(), target.position, target.orientation);
    res.pos_err_m = err.linear.norm();
    res.ori_err_rad = err.angular.norm();
    res.iterations = it;
    if ((res.pos_err_m < params.pos_tol_m && res.ori_err_rad < params.ori_tol_rad) ||
        it >= params.max_iter || !q.allFinite()) {
      break;
    }
    RobotModel::JointVector dq = strategy.computeJointVelocity(robot, target, q, q_ref);
    const double m = dq.cwiseAbs().maxCoeff();
    if (!std::isfinite(m)) break;
    if (m > params.max_step_rad) dq *= params.max_step_rad / m;
    q = (q + dq).cwiseMax(lo).cwiseMin(hi);
  }

  res.q = q;
  res.limits_ok = q.allFinite() && ((q.array() >= lo.array()).all() && (q.array() <= hi.array()).all());
  res.converged = res.limits_ok && std::isfinite(res.pos_err_m) && std::isfinite(res.ori_err_rad) &&
                  res.pos_err_m < params.pos_tol_m && res.ori_err_rad < params.ori_tol_rad;
  return res;
}

JointMargins jointMargins(const RobotModel& robot, const RobotModel::JointVector& q) {
  JointMargins m;
  m.margin_rad = (q - robot.jointLowerLimits()).cwiseMin(robot.jointUpperLimits() - q);
  int worst = 0;
  m.min_margin_rad = m.margin_rad.minCoeff(&worst);
  m.worst_joint = worst;
  return m;
}

double wTransAt(RobotModel& robot, const RobotModel::JointVector& q) {
  robot.update(q, RobotModel::JointVector::Zero());
  return wTransFromJacobian(robot.jacobian().topRows<3>());
}

Se3Distance se3Distance(const Eigen::Vector3d& p1, const Eigen::Quaterniond& q1,
                        const Eigen::Vector3d& p2, const Eigen::Quaterniond& q2,
                        double length_scale_m, double angle_scale_rad) {
  if (!(std::isfinite(length_scale_m) && length_scale_m > 0.0 && std::isfinite(angle_scale_rad) &&
        angle_scale_rad > 0.0)) {
    throw std::invalid_argument("se3Distance: scales must be finite and > 0.");
  }
  Se3Distance out;
  out.pos_m = (p1 - p2).norm();
  const Eigen::Quaterniond rel = q1.normalized().conjugate() * q2.normalized();
  out.ori_rad = 2.0 * std::atan2(rel.vec().norm(), std::abs(rel.w()));
  const double a = out.pos_m / length_scale_m;
  const double b = out.ori_rad / angle_scale_rad;
  out.d = std::sqrt(a * a + b * b);
  return out;
}

DemoReference demoReferenceFromEndJoints(RobotModel& robot, const RobotModel::JointVector& end_joints_rad) {
  DemoReference ref;
  ref.w_trans_demo = wTransAt(robot, end_joints_rad);  // leaves the model updated at end_joints_rad
  ref.position = robot.eePosition();
  ref.orientation = robot.eeOrientation();
  return ref;
}

Eigen::Quaterniond goalOrientation(const PoseEvaluation& eval) {
  const Eigen::Quaterniond& qn = eval.pose.orientation_nominal_world;
  if (eval.psi_rad == 0.0 && !eval.wrist_flip) return qn;
  return haptic_dmp_learning::core::applyRoll(
      qn, eval.psi_rad + (eval.wrist_flip ? haptic_dmp_learning::core::kPi : 0.0));
}

PoseEvaluationDetail evaluateGraspPoseDetailed(const haptic_dmp_learning::core::CubeSatelliteModel& model,
                                               haptic_dmp_learning::core::GraspPointId k, double theta_rad,
                                               double psi_rad, RobotModel& robot,
                                               const RobotModel::JointVector& q_start, const DemoReference& demo,
                                               double length_scale_m, double angle_scale_rad,
                                               const IkParams& params, bool compute_extra, int force_flip,
                                               BranchRule branch_rule) {
  if (force_flip < -1 || force_flip > 1) {
    throw std::invalid_argument("evaluateGraspPose: force_flip must be -1, 0 or 1.");
  }
  if (!(demo.w_trans_demo > 0.0)) {
    throw std::invalid_argument("evaluateGraspPose: demo.w_trans_demo must be > 0.");
  }
  if (!(std::isfinite(length_scale_m) && length_scale_m > 0.0 && std::isfinite(angle_scale_rad) &&
        angle_scale_rad > 0.0)) {
    throw std::invalid_argument("evaluateGraspPose: scales must be finite and > 0.");
  }

  PoseEvaluation ev;
  ev.k = k;
  ev.theta_rad = theta_rad;
  ev.psi_rad = psi_rad;
  ev.pose = model.graspPoseAt(k, theta_rad);

  const Eigen::Quaterniond orient[2] = {
      psi_rad == 0.0 ? ev.pose.orientation_nominal_world
                     : haptic_dmp_learning::core::applyRoll(ev.pose.orientation_nominal_world, psi_rad),
      haptic_dmp_learning::core::applyRoll(ev.pose.orientation_nominal_world,
                                           haptic_dmp_learning::core::kPi + psi_rad)};
  // Orientation-only: filled for both branches whatever the IK or force_flip do (same function as dist_to_demo).
  for (int i = 0; i < 2; ++i) {
    ev.branch_rot_rad[i] =
        se3Distance(ev.pose.position_world, orient[i], demo.position, demo.orientation, length_scale_m, angle_scale_rad)
            .ori_rad;
  }
  Branch br[2];
  for (int i = 0; i < 2; ++i) {
    if (force_flip >= 0 && i != force_flip) continue;  // forced branch only
    br[i].ik = solvePoseIk(robot, ev.pose.position_world, orient[i], q_start, q_start, params);
    br[i].margins = jointMargins(robot, br[i].ik.q);
    br[i].dq_norm = (br[i].ik.q - q_start).norm();
  }
  const int best = force_flip >= 0 ? force_flip : (better(br[1], br[0], branch_rule, ev.branch_rot_rad[1], ev.branch_rot_rad[0]) ? 1 : 0);

  ev.wrist_flip = (best == 1);
  ev.ik = br[best].ik;
  ev.margins = br[best].margins;
  ev.w_trans = wTransAt(robot, ev.ik.q);
  ev.w_hat = ev.w_trans / demo.w_trans_demo;
  ev.dist_to_demo = se3Distance(ev.pose.position_world, orient[best], demo.position, demo.orientation,
                                length_scale_m, angle_scale_rad);
  ev.feasible_basic = ev.ik.converged && ev.ik.limits_ok;

  PoseEvaluationDetail out;
  out.selected = best;
  for (int i = 0; i < 2; ++i) {
    BranchDetail& d = out.branch[i];
    d.orientation = orient[i];
    if (force_flip >= 0 && i != force_flip) continue;  // not evaluated: only the orientation is meaningful
    d.converged = br[i].ik.converged;
    d.limits_ok = br[i].ik.limits_ok;
    d.q = br[i].ik.q;
    d.pos_err_m = br[i].ik.pos_err_m;
    d.ori_err_rad = br[i].ik.ori_err_rad;
    d.iterations = br[i].ik.iterations;
    d.min_margin_rad = br[i].margins.min_margin_rad;
    d.worst_joint = br[i].margins.worst_joint;
    d.orientation = orient[i];
    if (compute_extra) {
      robot.update(d.q, RobotModel::JointVector::Zero());
      const FullManipulability fm = fullManipulability(robot.jacobian());
      d.w_trans = fm.w_trans;
      d.w_rot = fm.w_rot;
      d.w_full = fm.w_full;
      d.smin_trans = fm.smin_trans;
      d.smin_rot = fm.smin_rot;
    }
  }
  if (compute_extra) robot.update(ev.ik.q, RobotModel::JointVector::Zero());
  out.eval = ev;
  return out;
}

PoseEvaluation evaluateGraspPose(const haptic_dmp_learning::core::CubeSatelliteModel& model,
                                 haptic_dmp_learning::core::GraspPointId k, double theta_rad,
                                 RobotModel& robot, const RobotModel::JointVector& q_start,
                                 const DemoReference& demo, double length_scale_m,
                                 double angle_scale_rad, const IkParams& params) {
  return evaluateGraspPoseDetailed(model, k, theta_rad, 0.0, robot, q_start, demo, length_scale_m,
                                   angle_scale_rad, params, false)
      .eval;
}

}  // namespace core
}  // namespace satellite_grasp_planner
