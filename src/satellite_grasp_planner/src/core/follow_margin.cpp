#include "satellite_grasp_planner/core/follow_margin.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "haptic_dmp_learning/core/pose_rotation.hpp"
#include "satellite_grasp_planner/core/joint_path_simulator.hpp"
#include "satellite_grasp_planner/core/kinematic_feasibility.hpp"

namespace satellite_grasp_planner {
namespace core {

CartesianSample followTargetSample(const Eigen::Vector3d& center, const Eigen::Vector3d& axis, double omega_rad_s,
                                   const haptic_dmp_learning::core::RotatablePose& goal_at_end, double t_follow_s) {
  const Eigen::Vector3d axis_unit = axis.normalized();  // exact (0,0,1) for an exact +z axis
  const haptic_dmp_learning::core::RotatablePose cur =
      haptic_dmp_learning::core::rotatePoseAboutAxis(center, axis, goal_at_end, omega_rad_s * t_follow_s);
  CartesianSample target;
  target.position = cur.position;
  target.orientation = cur.orientation;
  const Eigen::Vector3d r = cur.position - center;
  // omega a x r; for a = +z this is (-omega r_y, omega r_x, 0) (the previous literal form) component by component.
  target.linear_velocity = omega_rad_s * axis_unit.cross(r);
  target.angular_velocity = omega_rad_s * axis_unit;
  return target;
}

FollowMarginResult followMargin(const haptic_dmp_learning::core::ProDMP& prodmp_template,
                                const Eigen::Vector3d& target_position, const Eigen::Quaterniond& target_orientation,
                                const Eigen::Vector3d& center, const Eigen::Vector3d& axis, double omega_rad_s,
                                const std::shared_ptr<RobotModel>& robot_model, const RobotModel::JointVector& q_seed,
                                const RobotModel::JointVector& q_ref, const FollowMarginParams& params,
                                const FollowApproachState* approach) {
  if (!robot_model) throw std::invalid_argument("followMargin: robot_model is null.");
  if (!(params.dt_approach_s > 0.0) || !std::isfinite(params.dt_approach_s) || !(params.dt_follow_s > 0.0) ||
      !std::isfinite(params.dt_follow_s)) {
    throw std::invalid_argument("followMargin: dt_approach_s and dt_follow_s must be finite and > 0.");
  }
  if (!(params.follow_s >= 0.0) || !std::isfinite(params.follow_s)) {
    throw std::invalid_argument("followMargin: follow_s must be finite and >= 0.");
  }
  if (!std::isfinite(omega_rad_s)) throw std::invalid_argument("followMargin: omega must be finite.");
  if (!(axis.allFinite() && axis.norm() > 0.0)) {
    throw std::invalid_argument("followMargin: the satellite axis must be finite and non-zero.");
  }

  const NullspaceBiasedResolution strategy{NullspaceBiasedResolution::Params()};  // defaults = impedance-controller proxy
  const RobotModel::JointVector& lo = robot_model->jointLowerLimits();
  const RobotModel::JointVector& hi = robot_model->jointUpperLimits();
  const RobotModel::JointVector dq_zero = RobotModel::JointVector::Zero();

  FollowMarginResult res;
  double min_all = 1e9;  // same sentinel as step0c_margins.cpp
  double min_approach = 1e9;
  auto sample = [&](const RobotModel::JointVector& q, double t_rel, bool approach) {
    for (int j = 0; j < RobotModel::kNumJoints; ++j) {
      const double m = std::min(q(j) - lo(j), hi(j) - q(j));
      if (m < min_all) {  // strict: the first minimum wins
        min_all = m;
        res.joint = j;
        res.t_rel_s = t_rel;
      }
      if (approach && m < min_approach) min_approach = m;
    }
  };

  RobotModel::JointVector q = q_seed;
  if (approach) {
    // Approach already simulated by the caller (same operations, hence the same state): continue from its end.
    q = approach->q_end;
    if (approach->joint >= 0) {
      min_all = approach->min_margin_rad;
      min_approach = approach->min_margin_rad;
      res.joint = approach->joint;
      res.t_rel_s = approach->t_rel_s;
    }
  } else {
    // Start pose = FK(q_seed), as checkKinematicFeasibility.
    robot_model->update(q_seed, RobotModel::JointVector::Zero());
    const Eigen::Vector3d p0 = robot_model->eePosition();
    const Eigen::Quaterniond quat0 = robot_model->eeOrientation();
    const double tau = prodmp_template.tau();
    const std::vector<CartesianSample> trajectory =
        buildApproachTrajectory(prodmp_template, p0, quat0, target_position, target_orientation, params.dt_approach_s);
    // Approach: the margin of the configuration before step i is labelled with the end time of the step, (i + 1) dt - tau.
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
      robot_model->update(q, dq_zero);
      sample(q, static_cast<double>(i + 1) * params.dt_approach_s - tau, true);
      q += strategy.computeJointVelocity(*robot_model, trajectory[i], q, q_ref) * params.dt_approach_s;
    }
  }
  res.q_at_end = q;

  // Follow: the grasp pose at AT_END co-rotates with the satellite.
  const haptic_dmp_learning::core::RotatablePose goal_at_end{target_position, target_orientation};
  const int n_follow = static_cast<int>(std::round(params.follow_s / params.dt_follow_s));
  for (int step = 1; step <= n_follow; ++step) {
    const double t_follow = step * params.dt_follow_s;
    const CartesianSample target = followTargetSample(center, axis, omega_rad_s, goal_at_end, t_follow);

    robot_model->update(q, dq_zero);
    sample(q, t_follow, false);
    q += strategy.computeJointVelocity(*robot_model, target, q, q_ref) * params.dt_follow_s;
  }
  robot_model->update(q, dq_zero);
  res.q_final = q;

  if (min_all < 1e9) {
    res.min_margin_rad = min_all;
    res.approach_min_margin_rad = min_approach;
  } else {
    res.joint = -1;  // nothing sampled (tau <= 0 and follow_s == 0)
    res.min_margin_rad = 0.0;
    res.approach_min_margin_rad = 0.0;
  }
  return res;
}

}  // namespace core
}  // namespace satellite_grasp_planner
