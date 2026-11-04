#include "satellite_grasp_planner/core/dynamic_follow.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>

#include "franka_cartesian_control/core/cartesian_error.hpp"
#include "franka_cartesian_control/core/impedance_solver.hpp"
#include "satellite_grasp_planner/core/follow_margin.hpp"

namespace satellite_grasp_planner {
namespace core {

namespace {

using JV = RobotModel::JointVector;
using franka_cartesian_control::core::CartesianError;
using franka_cartesian_control::core::CartesianImpedanceSolver;

/// Value of the attribute @p name in the text of one XML tag (preceded by a blank, quoted by " or '). False if absent.
bool tagAttribute(const std::string& tag, const std::string& name, double& out) {
  for (const char* quote : {"\"", "'"}) {
    const std::string key = " " + name + "=" + quote;
    std::size_t p = tag.find(key);
    if (p == std::string::npos) continue;
    p += key.size();
    const std::size_t e = tag.find(quote, p);
    if (e == std::string::npos) continue;
    out = std::stod(tag.substr(p, e - p));
    return true;
  }
  return false;
}

bool finitePositive(double x) { return std::isfinite(x) && x > 0.0; }
bool finiteNonNegative(double x) { return std::isfinite(x) && x >= 0.0; }

}  // namespace

// ---------------------------------------------------------------------------------------------------------------------
// URDF <dynamics>
// ---------------------------------------------------------------------------------------------------------------------
JointDynamics parseJointDynamicsFromUrdfXml(const std::string& xml, const std::vector<std::string>& joint_names) {
  if (joint_names.size() != static_cast<std::size_t>(RobotModel::kNumJoints)) {
    throw std::invalid_argument("parseJointDynamicsFromUrdfXml: expected 7 joint names.");
  }
  JointDynamics out;
  for (std::size_t j = 0; j < joint_names.size(); ++j) {
    const std::string& name = joint_names[j];
    bool done = false;
    std::size_t pos = 0;
    while (!done && (pos = xml.find("<joint", pos)) != std::string::npos) {
      const std::size_t after = pos + 6;
      if (after >= xml.size() || !std::isspace(static_cast<unsigned char>(xml[after]))) {  // <joint_state_publisher ...> etc.
        pos = after;
        continue;
      }
      const std::size_t tag_end = xml.find('>', pos);
      if (tag_end == std::string::npos) break;
      const std::string tag = xml.substr(pos, tag_end - pos);
      if (tag.find("name=\"" + name + "\"") == std::string::npos && tag.find("name='" + name + "'") == std::string::npos) {
        pos = tag_end;
        continue;
      }
      if (xml[tag_end - 1] == '/') {
        throw std::runtime_error("parseJointDynamicsFromUrdfXml: joint " + name + " has no <dynamics> tag (self-closing joint).");
      }
      const std::size_t body_end = xml.find("</joint>", tag_end);
      if (body_end == std::string::npos) throw std::runtime_error("parseJointDynamicsFromUrdfXml: unterminated joint " + name);
      const std::string body = xml.substr(tag_end, body_end - tag_end);
      const std::size_t d = body.find("<dynamics");
      if (d == std::string::npos) {
        throw std::runtime_error("parseJointDynamicsFromUrdfXml: joint " + name + " has no <dynamics> tag.");
      }
      const std::size_t de = body.find('>', d);
      const std::string dyn = body.substr(d, de == std::string::npos ? std::string::npos : de - d);
      double damping = 0.0, friction = 0.0;  // URDF default of an absent attribute
      tagAttribute(dyn, "damping", damping);
      tagAttribute(dyn, "friction", friction);
      out.damping(static_cast<Eigen::Index>(j)) = damping;
      out.friction(static_cast<Eigen::Index>(j)) = friction;
      done = true;
    }
    if (!done) throw std::runtime_error("parseJointDynamicsFromUrdfXml: joint not found in the URDF: " + name);
  }
  return out;
}

JointDynamics loadJointDynamicsFromUrdfFile(const std::string& urdf_path, const std::vector<std::string>& joint_names) {
  std::ifstream f(urdf_path);
  if (!f.is_open()) throw std::runtime_error("loadJointDynamicsFromUrdfFile: cannot open " + urdf_path);
  std::stringstream buf;
  buf << f.rdbuf();
  return parseJointDynamicsFromUrdfXml(buf.str(), joint_names);
}

// ---------------------------------------------------------------------------------------------------------------------
// parameters
// ---------------------------------------------------------------------------------------------------------------------
void validateDynamicFollowParams(const DynamicFollowParams& p) {
  if (!finitePositive(p.translational_stiffness) || !finitePositive(p.rotational_stiffness) ||
      !finitePositive(p.translational_damping) || !finitePositive(p.rotational_damping)) {
    throw std::invalid_argument("DynamicFollowParams: Cartesian stiffness and damping must be finite and > 0.");
  }
  if (!finiteNonNegative(p.nullspace_stiffness) || !finiteNonNegative(p.joint1_nullspace_stiffness) ||
      !finiteNonNegative(p.nullspace_pinv_damping)) {
    throw std::invalid_argument("DynamicFollowParams: nullspace gains must be finite and >= 0.");
  }
  if (!finitePositive(p.delta_tau_max)) throw std::invalid_argument("DynamicFollowParams: delta_tau_max must be finite and > 0.");
  if (!std::isfinite(p.plant_gravity_z_m_s2) || !std::isfinite(p.model_gravity_z_m_s2) || p.model_gravity_z_m_s2 == 0.0) {
    throw std::invalid_argument("DynamicFollowParams: gravities must be finite and the model gravity non-zero.");
  }
  if (!finitePositive(p.friction_tanh_eps_rad_s)) {
    throw std::invalid_argument("DynamicFollowParams: friction_tanh_eps_rad_s must be finite and > 0.");
  }
  if (!finitePositive(p.dt_s)) throw std::invalid_argument("DynamicFollowParams: dt_s must be finite and > 0.");
  if (!finitePositive(p.divergence_rad)) throw std::invalid_argument("DynamicFollowParams: divergence_rad must be finite and > 0.");
  if (!(std::isfinite(p.finger_pos_m) && p.finger_pos_m >= 0.0 && p.finger_pos_m <= 0.08)) {
    throw std::invalid_argument("DynamicFollowParams: finger_pos_m must be finite and in [0, 0.08] m.");
  }
}

// ---------------------------------------------------------------------------------------------------------------------
// targets
// ---------------------------------------------------------------------------------------------------------------------
PoseSeries::PoseSeries(const std::vector<double>& t, const std::vector<Eigen::Vector3d>& p,
                       const std::vector<Eigen::Quaterniond>& q) {
  if (t.empty() || t.size() != p.size() || t.size() != q.size()) {
    throw std::invalid_argument("PoseSeries: times, positions and orientations must be non-empty and of the same length.");
  }
  std::vector<std::size_t> idx(t.size());
  std::iota(idx.begin(), idx.end(), std::size_t{0});
  std::stable_sort(idx.begin(), idx.end(), [&t](std::size_t a, std::size_t b) { return t[a] < t[b]; });
  for (std::size_t i : idx) {
    if (!std::isfinite(t[i]) || !p[i].allFinite() || !q[i].coeffs().allFinite() || q[i].norm() == 0.0) {
      throw std::invalid_argument("PoseSeries: non-finite sample or zero quaternion.");
    }
    t_.push_back(t[i]);
    p_.push_back(p[i]);
    q_.push_back(q[i].normalized());
  }
}

CartesianSample PoseSeries::at(double t_s) const {
  const auto it = std::upper_bound(t_.begin(), t_.end(), t_s + 1e-12);
  const std::size_t i = it == t_.begin() ? 0 : static_cast<std::size_t>(it - t_.begin()) - 1;
  CartesianSample s;
  s.position = p_[i];
  s.orientation = q_[i];
  return s;
}

DynamicFollowTarget makePoseSeriesTarget(std::shared_ptr<const PoseSeries> series) {
  if (!series) throw std::invalid_argument("makePoseSeriesTarget: null series.");
  return [series](double t_s) { return series->at(t_s); };
}

DynamicFollowTarget makeRotatingTarget(const Eigen::Vector3d& center, const Eigen::Vector3d& axis, double omega_rad_s,
                                       const haptic_dmp_learning::core::RotatablePose& goal_at_end) {
  if (!(axis.allFinite() && axis.norm() > 0.0) || !center.allFinite() || !std::isfinite(omega_rad_s)) {
    throw std::invalid_argument("makeRotatingTarget: non-finite center / omega or zero axis.");
  }
  return [center, axis, omega_rad_s, goal_at_end](double t_s) {
    return followTargetSample(center, axis, omega_rad_s, goal_at_end, t_s);
  };
}

// ---------------------------------------------------------------------------------------------------------------------
// result
// ---------------------------------------------------------------------------------------------------------------------
const RobotModel::JointVector& DynamicFollowResult::qAt(double t_s) const {
  const long i = std::lround(t_s / dt_s);
  if (i < 0 || static_cast<std::size_t>(i) >= q.size()) throw std::out_of_range("DynamicFollowResult::qAt: no row at that time.");
  return q[static_cast<std::size_t>(i)];
}

// ---------------------------------------------------------------------------------------------------------------------
// simulator
// ---------------------------------------------------------------------------------------------------------------------
DynamicFollowSimulator::DynamicFollowSimulator(std::shared_ptr<RobotModel> robot, const JointDynamics& dynamics,
                                               const DynamicFollowParams& params)
    : robot_(std::move(robot)), dyn_(dynamics), params_(params) {
  if (!robot_) throw std::invalid_argument("DynamicFollowSimulator: robot is null.");
  validateDynamicFollowParams(params_);
  if (!dyn_.damping.allFinite() || !dyn_.friction.allFinite()) {
    throw std::invalid_argument("DynamicFollowSimulator: non-finite joint dynamics.");
  }
}

RobotModel::JointVector DynamicFollowSimulator::frictionTorque(const RobotModel::JointVector& v) const {
  JV f = JV::Zero();
  for (int j = 0; j < RobotModel::kNumJoints; ++j) {
    const double d = params_.apply_urdf_damping ? dyn_.damping(j) : 0.0;
    const double c = params_.apply_urdf_friction ? dyn_.friction(j) : 0.0;
    f(j) = d * v(j) + c * std::tanh(v(j) / params_.friction_tanh_eps_rad_s);
  }
  return f;
}

DynamicFollowResult DynamicFollowSimulator::simulate(const RobotModel::JointVector& q0, const RobotModel::JointVector& dq0,
                                                     const RobotModel::JointVector& q_nullspace,
                                                     const DynamicFollowTarget& target, double horizon_s,
                                                     const DynamicFollowObserver* observer) {
  if (!target) throw std::invalid_argument("DynamicFollowSimulator::simulate: empty target.");
  if (!q0.allFinite() || !dq0.allFinite() || !q_nullspace.allFinite()) {
    throw std::invalid_argument("DynamicFollowSimulator::simulate: non-finite initial condition or nullspace target.");
  }
  if (!finitePositive(horizon_s)) throw std::invalid_argument("DynamicFollowSimulator::simulate: horizon_s must be > 0.");
  const double dt = params_.dt_s;
  const int n = static_cast<int>(std::lround(horizon_s / dt));
  if (n < 1) throw std::invalid_argument("DynamicFollowSimulator::simulate: horizon shorter than one step.");

  CartesianImpedanceSolver::Params sp;
  sp.translational_stiffness = params_.translational_stiffness;
  sp.rotational_stiffness = params_.rotational_stiffness;
  sp.translational_damping = params_.translational_damping;
  sp.rotational_damping = params_.rotational_damping;
  sp.nullspace_stiffness = params_.nullspace_stiffness;
  sp.joint1_nullspace_stiffness = params_.joint1_nullspace_stiffness;
  sp.nullspace_pinv_damping = params_.nullspace_pinv_damping;
  sp.delta_tau_max = params_.delta_tau_max;
  sp.compensate_gravity_internally = params_.compensate_gravity_internally;
  CartesianImpedanceSolver solver(sp);
  solver.setNullspaceTarget(q_nullspace);
  // First cycle: tau_prev is unknown, no slew limit (inertial_pred.py:379-380). An infinite limit returns tau_prev + (tau_d - tau_prev)
  // with tau_prev = 0, i.e. tau_d exactly.
  CartesianImpedanceSolver::Params sp_first = sp;
  sp_first.delta_tau_max = std::numeric_limits<double>::infinity();
  CartesianImpedanceSolver solver_first(sp_first);
  solver_first.setNullspaceTarget(q_nullspace);

  const double g_scale = params_.plant_gravity_z_m_s2 / params_.model_gravity_z_m_s2;
  const JV lo = robot_->jointLowerLimits();
  const JV hi = robot_->jointUpperLimits();

  // a = M^-1 (tau - f(v) - c - g_plant) with M, c, g of the model state last passed to RobotModel::update().
  auto accel = [&](const JV& tau, const JV& v) -> JV {
    const JV rhs = tau - frictionTorque(v) - robot_->coriolis() - g_scale * robot_->gravity();
    const JV a = robot_->massMatrix().llt().solve(rhs);
    return a;
  };
  auto accel_at = [&](const JV& qs, const JV& vs, const JV& tau) -> JV {
    robot_->update(qs, vs);
    return accel(tau, vs);
  };

  DynamicFollowResult res;
  res.dt_s = dt;
  res.q.reserve(static_cast<std::size_t>(n) + 1);
  res.min_margin_rad = std::numeric_limits<double>::quiet_NaN();
  res.min_margin_joint = -1;
  bool have_margin = false;
  auto account = [&](const JV& qq, int row) {
    if (!qq.allFinite()) return;
    const JV m = (qq - lo).cwiseMin(hi - qq);
    Eigen::Index jmin = 0;
    const double mm = m.minCoeff(&jmin);
    if (!have_margin || mm < res.min_margin_rad) {  // strict: the first minimum wins (np.argmin)
      have_margin = true;
      res.min_margin_rad = mm;
      res.min_margin_joint = static_cast<int>(jmin);
      res.min_margin_t_s = row * dt;
    }
  };

  JV q = q0;
  JV v = dq0;
  JV tau_prev = JV::Zero();
  res.q.push_back(q);
  account(q, 0);

  for (int i = 0; i < n; ++i) {
    robot_->update(q, v);
    const CartesianSample tg = target(i * dt);
    const CartesianError err = franka_cartesian_control::core::computePoseError(
        robot_->eePosition(), robot_->eeOrientation(), tg.position, tg.orientation);
    const JV tau = (i == 0 ? solver_first : solver).computeTorque(*robot_, err, q, v, tau_prev);
    tau_prev = tau;

    const JV k1q = v;
    const JV k1v = accel(tau, v);
    if (observer && *observer) {
      DynamicFollowStepInfo info;
      info.step = i;
      info.t_s = i * dt;
      info.q = q;
      info.v = v;
      info.tau = tau;
      info.acc = k1v;
      (*observer)(info);
    }
    const JV k2q = v + 0.5 * dt * k1v;
    const JV k2v = accel_at(q + 0.5 * dt * k1q, v + 0.5 * dt * k1v, tau);
    const JV k3q = v + 0.5 * dt * k2v;
    const JV k3v = accel_at(q + 0.5 * dt * k2q, v + 0.5 * dt * k2v, tau);
    const JV k4q = v + dt * k3v;
    const JV k4v = accel_at(q + dt * k3q, v + dt * k3v, tau);
    q = q + dt / 6.0 * (k1q + 2.0 * k2q + 2.0 * k3q + k4q);
    v = v + dt / 6.0 * (k1v + 2.0 * k2v + 2.0 * k3v + k4v);

    // Divergence: non-finite state, or a joint farther than divergence_rad from q0 (the old test was the first term only).
    const bool blown_up = !(q.allFinite() && v.allFinite()) || (q - q0).cwiseAbs().maxCoeff() > params_.divergence_rad;
    if (blown_up) {
      if (!res.diverged) {
        res.diverged = true;
        res.t_div_s = (i + 1) * dt;
      }
      if (params_.abort_on_divergence) break;
    }
    res.q.push_back(q);
    account(q, i + 1);
    res.steps_done = i + 1;
  }
  return res;
}

}  // namespace core
}  // namespace satellite_grasp_planner
