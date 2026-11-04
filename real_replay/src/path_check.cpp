#include "real_replay/path_check.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "satellite_grasp_planner/core/joint_path_simulator.hpp"

namespace real_replay {

namespace sgp = satellite_grasp_planner::core;
namespace fcc = franka_cartesian_control::core;

PathCheckResult checkJointPath(const Config& cfg, const std::string& urdf_xml, ITargetSource& src,
                               const Array7& q_act) {
  auto model = std::make_shared<fcc::RobotModel>(urdf_xml, cfg.robot.joint_names, cfg.robot.ee_frame);

  fcc::RobotModel::JointVector q0;
  for (int i = 0; i < 7; ++i) q0(i) = q_act[static_cast<std::size_t>(i)];
  model->update(q0, fcc::RobotModel::JointVector::Zero());

  Pose act;
  act.p = model->eePosition();
  act.q = model->eeOrientation();

  // Sample the source at the planner's integration step.
  constexpr double dt = sgp::kDefaultSimDtS;
  const std::size_t max_samples = static_cast<std::size_t>((src.nominalDurationS() + 1.0) / dt) + 10;
  std::vector<sgp::CartesianSample> traj;
  traj.reserve(max_samples);
  auto push = [&](const Pose& p) {
    sgp::CartesianSample s;
    s.position = p.p;
    s.orientation = p.q;
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    Eigen::Vector3d w = Eigen::Vector3d::Zero();
    src.twist(v, w);
    s.linear_velocity = v;
    s.angular_velocity = w;
    traj.push_back(s);
  };
  src.start(act);
  push(src.sample(0.0));
  while (!src.done() && traj.size() < max_samples) push(src.sample(dt));

  sgp::JointPathSimulator sim(model);  // default Params: dt = kDefaultSimDtS, impedance proxy strategy
  fcc::RobotModel::JointVector q_after;
  const auto steps = sim.simulate(traj, q0, q0, &q_after);

  PathCheckResult r;
  r.samples = steps.size();
  r.duration_s = static_cast<double>(steps.size()) * dt;
  r.margin_rad = cfg.safety.joint_limit_margin_rad;
  r.min_w_trans = std::numeric_limits<double>::infinity();
  r.min_margin_rad.fill(std::numeric_limits<double>::infinity());
  const auto& lo = model->jointLowerLimits();
  const auto& hi = model->jointUpperLimits();
  for (const auto& s : steps) {
    r.min_w_trans = std::min(r.min_w_trans, s.w_trans);
    for (int j = 0; j < 7; ++j) {
      const std::size_t k = static_cast<std::size_t>(j);
      const double m = std::min(s.q(j) - lo(j), hi(j) - s.q(j));
      r.min_margin_rad[k] = std::min(r.min_margin_rad[k], m);
    }
  }
  // Per-axis extrema of diag(Lambda) along the simulated path (offline: not real-time code).
  r.lambda_min.fill(std::numeric_limits<double>::infinity());
  r.lambda_max.fill(0.0);
  for (std::size_t i = 0; i < steps.size(); ++i) {
    model->update(steps[i].q, fcc::RobotModel::JointVector::Zero());
    const fcc::RobotModel::Matrix6d L = model->reflectedCartesianInertia();
    for (int a = 0; a < 6; ++a) {
      const std::size_t k = static_cast<std::size_t>(a);
      if (i == 0) r.lambda_act[k] = L(a, a);
      r.lambda_min[k] = std::min(r.lambda_min[k], L(a, a));
      r.lambda_max[k] = std::max(r.lambda_max[k], L(a, a));
    }
  }
  r.ok = !steps.empty();
  double worst = std::numeric_limits<double>::infinity();
  for (int j = 0; j < 7; ++j) {
    const std::size_t k = static_cast<std::size_t>(j);
    if (!(r.min_margin_rad[k] >= r.margin_rad)) r.ok = false;
    if (r.min_margin_rad[k] < worst) {
      worst = r.min_margin_rad[k];
      r.worst_joint = j;
    }
    r.q_end[k] = q_after(j);
  }
  return r;
}

}  // namespace real_replay
