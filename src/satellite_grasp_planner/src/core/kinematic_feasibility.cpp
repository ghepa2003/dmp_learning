#include "satellite_grasp_planner/core/kinematic_feasibility.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "haptic_dmp_learning/core/math_utils.hpp"
#include "satellite_grasp_planner/core/joint_path_simulator.hpp"

namespace satellite_grasp_planner {
namespace core {

std::vector<CartesianSample> buildApproachTrajectory(const haptic_dmp_learning::core::ProDMP& prodmp_template,
                                                     const Eigen::Vector3d& p0, const Eigen::Quaterniond& quat0,
                                                     const Eigen::Vector3d& target_position,
                                                     const Eigen::Quaterniond& target_orientation, double dt) {
  if (!(dt > 0.0)) {
    throw std::invalid_argument("buildApproachTrajectory: dt must be > 0");
  }
  const double tau = prodmp_template.tau();
  const int n_steps = static_cast<int>(std::ceil(tau / dt));

  haptic_dmp_learning::core::ProDMP prodmp = prodmp_template;  // step() mutates: work on a copy
  prodmp.setRelativeGoal(false);
  prodmp.setInitialConditions(0.0, p0, Eigen::Vector3d::Zero());
  prodmp.setGoal(target_position);

  std::vector<CartesianSample> trajectory;
  trajectory.reserve(static_cast<size_t>(n_steps));
  for (int i = 0; i < n_steps; ++i) {
    CartesianSample sample;
    sample.position = prodmp.step(dt);
    const double s = std::min(1.0, static_cast<double>(i + 1) * dt / tau);
    sample.orientation =
        haptic_dmp_learning::core::slerpShortestPath(quat0, target_orientation, s);
    trajectory.push_back(sample);
  }
  return trajectory;
}

std::vector<ApproachSample> timedApproachTrajectory(const haptic_dmp_learning::core::ProDMP& prodmp_template,
                                                    const Eigen::Vector3d& p0, const Eigen::Quaterniond& quat0,
                                                    const Eigen::Vector3d& target_position,
                                                    const Eigen::Quaterniond& target_orientation, double dt) {
  const double tau = prodmp_template.tau();
  const std::vector<CartesianSample> samples =
      buildApproachTrajectory(prodmp_template, p0, quat0, target_position, target_orientation, dt);
  std::vector<ApproachSample> out;
  out.reserve(samples.size() + 1);
  ApproachSample first;
  first.t_s = 0.0;
  first.position = p0;
  first.orientation = quat0;
  out.push_back(first);
  for (std::size_t i = 0; i < samples.size(); ++i) {
    ApproachSample a;
    a.t_s = static_cast<double>(i + 1) * dt;
    a.position = a.t_s >= tau ? target_position : samples[i].position;
    a.orientation = samples[i].orientation;
    out.push_back(a);
  }
  return out;
}

FeasibilityResult checkKinematicFeasibility(
    const haptic_dmp_learning::core::ProDMP& prodmp_template,
    const Eigen::Vector3d& target_position,
    const Eigen::Quaterniond& target_orientation,
    const std::shared_ptr<RobotModel>& robot_model,
    const RobotModel::JointVector& q0,
    double dt,
    double max_joint_violation_tol_rad,
    const std::optional<RobotModel::JointVector>& q_ref) {
  if (!robot_model) {
    throw std::invalid_argument("checkKinematicFeasibility: robot_model is null.");
  }
  if (!(dt > 0.0)) {
    throw std::invalid_argument("checkKinematicFeasibility: dt must be > 0.");
  }

  // Start pose = FK at q0.
  robot_model->update(q0, RobotModel::JointVector::Zero());
  const Eigen::Vector3d p0 = robot_model->eePosition();
  const Eigen::Quaterniond quat0 = robot_model->eeOrientation();

  const std::vector<CartesianSample> trajectory =
      buildApproachTrajectory(prodmp_template, p0, quat0, target_position, target_orientation, dt);

  NullspaceBiasedResolution::Params ns_params;  // defaults = impedance-controller proxy, as in the tools
  JointPathSimulator::Params sim_params;
  sim_params.dt = dt;
  sim_params.strategy = std::make_shared<NullspaceBiasedResolution>(ns_params);
  const JointPathSimulator sim(robot_model, sim_params);
  RobotModel::JointVector q_post = q0;
  const auto steps = sim.simulate(trajectory, q0, q_ref.value_or(q0), &q_post);

  FeasibilityResult res;
  if (steps.empty()) {
    return res;  // tau <= 0: nothing simulated, infeasible
  }

  const RobotModel::JointVector& lo = robot_model->jointLowerLimits();
  const RobotModel::JointVector& hi = robot_model->jointUpperLimits();
  double max_viol = 0.0;
  double min_margin = std::numeric_limits<double>::infinity();
  int min_margin_joint = -1;
  int min_margin_step = -1;
  for (std::size_t i = 0; i < steps.size(); ++i) {
    const auto& step = steps[i];
    Eigen::Index jmin = 0;
    const double step_min = (step.q - lo).cwiseMin(hi - step.q).minCoeff(&jmin);  // same value as the former minCoeff()
    if (step_min < min_margin) {                                                  // strict: the first minimum wins (min_margin is unchanged otherwise)
      min_margin = step_min;
      min_margin_joint = static_cast<int>(jmin);
      min_margin_step = static_cast<int>(i);
    }
    for (int j = 0; j < RobotModel::kNumJoints; ++j) {
      const double below = lo(j) - step.q(j);
      const double above = step.q(j) - hi(j);
      if (std::max(below, above) > max_viol) {  // strictly: only real violations are recorded
        res.worst_joint = j;
      }
      max_viol = std::max({max_viol, below, above});
    }
  }
  res.max_joint_violation_rad = max_viol;
  res.min_joint_margin_rad = min_margin;
  res.min_margin_joint = min_margin_joint;
  res.min_margin_step = min_margin_step;
  res.q_end_post_step = q_post;
  res.q_final = steps.back().q;

  robot_model->update(steps.back().q, RobotModel::JointVector::Zero());
  res.pos_err_final_mm = (robot_model->eePosition() - target_position).norm() * 1000.0;

  // Only the joint-limit criterion decides: pos_err_final_mm is diagnostic (it contains the
  // simulator's Kp tracking-lag artefact, see the header).
  res.feasible = res.max_joint_violation_rad < max_joint_violation_tol_rad;
  return res;
}

}  // namespace core
}  // namespace satellite_grasp_planner
