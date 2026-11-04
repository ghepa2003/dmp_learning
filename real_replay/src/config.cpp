#include "real_replay/config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace real_replay {
namespace {

using YAML::Node;

Node section(const Node& root, const std::string& name) {
  const Node n = root[name];
  if (!n.IsDefined() || !n.IsMap()) {
    throw std::runtime_error("config: missing or non-map section '" + name + "'");
  }
  return n;
}

template <typename T>
T req(const Node& sec, const std::string& ctx, const std::string& key) {
  const Node n = sec[key];
  if (!n.IsDefined()) throw std::runtime_error("config: missing '" + ctx + "." + key + "'");
  try {
    return n.as<T>();
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("config: invalid '" + ctx + "." + key + "': " + e.what());
  }
}

template <typename T>
T opt(const Node& sec, const std::string& ctx, const std::string& key, const T& def) {
  const Node n = sec[key];
  if (!n.IsDefined()) return def;
  try {
    return n.as<T>();
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("config: invalid '" + ctx + "." + key + "': " + e.what());
  }
}

template <std::size_t N>
std::array<double, N> reqArray(const Node& sec, const std::string& ctx, const std::string& key) {
  const auto v = req<std::vector<double>>(sec, ctx, key);
  if (v.size() != N) {
    throw std::runtime_error("config: '" + ctx + "." + key + "' must have " + std::to_string(N) +
                             " elements, got " + std::to_string(v.size()));
  }
  std::array<double, N> a{};
  std::copy(v.begin(), v.end(), a.begin());
  return a;
}

void requireFinitePositive(double v, const std::string& name) {
  if (!(std::isfinite(v) && v > 0.0)) throw std::runtime_error("config: '" + name + "' must be > 0");
}
void requireFiniteNonNegative(double v, const std::string& name) {
  if (!(std::isfinite(v) && v >= 0.0)) throw std::runtime_error("config: '" + name + "' must be >= 0");
}
void requireAtLeastOne(int v, const std::string& name) {
  if (v < 1) throw std::runtime_error("config: '" + name + "' must be >= 1");
}

}  // namespace

Config loadConfig(const std::string& yaml_path) {
  Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("config: cannot load '" + yaml_path + "': " + e.what());
  }
  if (!root.IsMap()) throw std::runtime_error("config: root of '" + yaml_path + "' is not a map");

  Config c{};
  c.config_path = std::filesystem::absolute(yaml_path).string();
  const std::filesystem::path base = std::filesystem::path(c.config_path).parent_path();
  auto resolve = [&base](const std::string& s) {
    std::filesystem::path p(s);
    if (p.is_relative()) p = base / p;
    return p.lexically_normal().string();
  };

  {
    const Node s = section(root, "robot");
    c.robot.ip = req<std::string>(s, "robot", "ip");
    c.robot.ee_frame = req<std::string>(s, "robot", "ee_frame");
    c.robot.joint_names = req<std::vector<std::string>>(s, "robot", "joint_names");
    if (c.robot.joint_names.size() != 7) throw std::runtime_error("config: robot.joint_names needs 7 names");
  }
  {
    const Node s = section(root, "paths");
    c.paths.urdf = resolve(req<std::string>(s, "paths", "urdf"));
    c.paths.log_dir = resolve(req<std::string>(s, "paths", "log_dir"));
  }
  {
    const Node s = section(root, "run");
    c.run.max_duration_s = req<double>(s, "run", "max_duration_s");
    requireFinitePositive(c.run.max_duration_s, "run.max_duration_s");
  }
  {
    const Node s = section(root, "hold");
    c.hold.duration_s = req<double>(s, "hold", "duration_s");
    requireFinitePositive(c.hold.duration_s, "hold.duration_s");
  }
  {
    const Node s = section(root, "trajectory");
    c.trajectory.target_update_divisor = opt<int>(s, "trajectory", "target_update_divisor", 1);
    c.trajectory.stop_at_fraction = opt<double>(s, "trajectory", "stop_at_fraction", 1.0);
    requireAtLeastOne(c.trajectory.target_update_divisor, "trajectory.target_update_divisor");
    if (!(c.trajectory.stop_at_fraction > 0.0 && c.trajectory.stop_at_fraction <= 1.0)) {
      throw std::runtime_error("config: trajectory.stop_at_fraction must be in (0, 1]");
    }
  }
  {
    const Node s = section(root, "step");
    StepConfig& t = c.step;
    t.axis = reqArray<3>(s, "step", "axis");
    t.amplitude_m = req<double>(s, "step", "amplitude_m");
    t.move_duration_s = req<double>(s, "step", "move_duration_s");
    t.dwell_s = req<double>(s, "step", "dwell_s");
    t.return_to_start = req<bool>(s, "step", "return_to_start");
    t.return_duration_s = req<double>(s, "step", "return_duration_s");
    if (!(std::isfinite(t.axis[0]) && std::isfinite(t.axis[1]) && std::isfinite(t.axis[2]) &&
          (t.axis[0] * t.axis[0] + t.axis[1] * t.axis[1] + t.axis[2] * t.axis[2]) > 1e-12)) {
      throw std::runtime_error("config: step.axis must be a non-zero finite vector");
    }
    if (s["rot_axis"].IsDefined()) t.rot_axis = reqArray<3>(s, "step", "rot_axis");
    t.rot_angle_rad = opt<double>(s, "step", "rot_angle_rad", t.rot_angle_rad);
    if (!(std::isfinite(t.rot_axis[0]) && std::isfinite(t.rot_axis[1]) && std::isfinite(t.rot_axis[2]) &&
          (t.rot_axis[0] * t.rot_axis[0] + t.rot_axis[1] * t.rot_axis[1] + t.rot_axis[2] * t.rot_axis[2]) > 1e-12)) {
      throw std::runtime_error("config: step.rot_axis must be a non-zero finite vector");
    }
    requireFinitePositive(t.rot_angle_rad, "step.rot_angle_rad");
    requireFinitePositive(t.amplitude_m, "step.amplitude_m");
    requireFinitePositive(t.move_duration_s, "step.move_duration_s");
    requireFiniteNonNegative(t.dwell_s, "step.dwell_s");
    if (t.return_to_start) requireFinitePositive(t.return_duration_s, "step.return_duration_s");
  }
  {
    const Node s = section(root, "replay");
    c.replay.weights_path = resolve(req<std::string>(s, "replay", "weights_path"));
    c.replay.expected_sha256 = opt<std::string>(s, "replay", "expected_sha256", "");
    c.replay.hold_after_end_s = req<double>(s, "replay", "hold_after_end_s");
    requireFiniteNonNegative(c.replay.hold_after_end_s, "replay.hold_after_end_s");
  }
  {
    const Node s = section(root, "gains");
    GainsConfig& g = c.gains;
    g.translational_stiffness = req<double>(s, "gains", "translational_stiffness");
    g.rotational_stiffness = req<double>(s, "gains", "rotational_stiffness");
    g.translational_damping = req<double>(s, "gains", "translational_damping");
    g.rotational_damping = req<double>(s, "gains", "rotational_damping");
    g.nullspace_stiffness = req<double>(s, "gains", "nullspace_stiffness");
    g.joint1_nullspace_stiffness = req<double>(s, "gains", "joint1_nullspace_stiffness");
    g.nullspace_pinv_damping = req<double>(s, "gains", "nullspace_pinv_damping");
    g.delta_tau_max = req<double>(s, "gains", "delta_tau_max");
    g.gain_ramp_s = req<double>(s, "gains", "gain_ramp_s");
    g.gain_ramp_initial = req<double>(s, "gains", "gain_ramp_initial");
    requireFinitePositive(g.translational_stiffness, "gains.translational_stiffness");
    requireFinitePositive(g.rotational_stiffness, "gains.rotational_stiffness");
    requireFinitePositive(g.translational_damping, "gains.translational_damping");
    requireFinitePositive(g.rotational_damping, "gains.rotational_damping");
    requireFiniteNonNegative(g.nullspace_stiffness, "gains.nullspace_stiffness");
    requireFiniteNonNegative(g.joint1_nullspace_stiffness, "gains.joint1_nullspace_stiffness");
    requireFinitePositive(g.nullspace_pinv_damping, "gains.nullspace_pinv_damping");
    requireFinitePositive(g.delta_tau_max, "gains.delta_tau_max");
    requireFiniteNonNegative(g.gain_ramp_s, "gains.gain_ramp_s");
    if (!(g.gain_ramp_initial > 0.0 && g.gain_ramp_initial <= 1.0)) {
      throw std::runtime_error("config: gains.gain_ramp_initial must be in (0, 1]");
    }
  }
  {
    const Node s = section(root, "damping");
    DampingConfig& d = c.damping;
    d.zeta_trans = req<double>(s, "damping", "zeta_trans");
    d.zeta_rot = req<double>(s, "damping", "zeta_rot");
    d.max_damping_per_sample = req<double>(s, "damping", "max_damping_per_sample");
    d.max_q_drift_rad = req<double>(s, "damping", "max_q_drift_rad");
    d.override_d = opt<std::vector<double>>(s, "damping", "override_d", std::vector<double>());
    requireFinitePositive(d.zeta_trans, "damping.zeta_trans");
    requireFinitePositive(d.zeta_rot, "damping.zeta_rot");
    requireFinitePositive(d.max_damping_per_sample, "damping.max_damping_per_sample");
    requireFinitePositive(d.max_q_drift_rad, "damping.max_q_drift_rad");
    if (!d.override_d.empty()) {
      if (d.override_d.size() != 6) throw std::runtime_error("config: damping.override_d must be empty or have 6 values");
      for (double v : d.override_d) requireFinitePositive(v, "damping.override_d[*]");
    }
  }
  {
    const Node s = section(root, "safety");
    SafetyConfig& f = c.safety;
    f.effort_limits_nm = reqArray<7>(s, "safety", "effort_limits_nm");
    f.torque_limit_fraction = req<double>(s, "safety", "torque_limit_fraction");
    f.max_torque_step_nm = req<double>(s, "safety", "max_torque_step_nm");
    f.joint_limit_margin_rad = req<double>(s, "safety", "joint_limit_margin_rad");
    f.max_joint_speed_rad_s = req<double>(s, "safety", "max_joint_speed_rad_s");
    f.joint_speed_consecutive = req<int>(s, "safety", "joint_speed_consecutive");
    f.max_tracking_error_m = req<double>(s, "safety", "max_tracking_error_m");
    f.max_tracking_error_rad = req<double>(s, "safety", "max_tracking_error_rad");
    f.tracking_consecutive = req<int>(s, "safety", "tracking_consecutive");
    f.period_abort_s = req<double>(s, "safety", "period_abort_s");
    f.callback_budget_us = req<double>(s, "safety", "callback_budget_us");
    f.callback_budget_consecutive = req<int>(s, "safety", "callback_budget_consecutive");
    f.min_success_rate = req<double>(s, "safety", "min_success_rate");
    f.success_rate_consecutive = req<int>(s, "safety", "success_rate_consecutive");
    f.watchdog_grace_cycles = req<int>(s, "safety", "watchdog_grace_cycles");
    f.stop_duration_s = req<double>(s, "safety", "stop_duration_s");
    f.stop_damping_scale = req<double>(s, "safety", "stop_damping_scale");
    for (double v : f.effort_limits_nm) requireFinitePositive(v, "safety.effort_limits_nm[*]");
    if (!(f.torque_limit_fraction > 0.0 && f.torque_limit_fraction <= 1.0)) {
      throw std::runtime_error("config: safety.torque_limit_fraction must be in (0, 1]");
    }
    requireFinitePositive(f.max_torque_step_nm, "safety.max_torque_step_nm");
    requireFiniteNonNegative(f.joint_limit_margin_rad, "safety.joint_limit_margin_rad");
    requireFinitePositive(f.max_joint_speed_rad_s, "safety.max_joint_speed_rad_s");
    requireAtLeastOne(f.joint_speed_consecutive, "safety.joint_speed_consecutive");
    requireFinitePositive(f.max_tracking_error_m, "safety.max_tracking_error_m");
    requireFinitePositive(f.max_tracking_error_rad, "safety.max_tracking_error_rad");
    requireAtLeastOne(f.tracking_consecutive, "safety.tracking_consecutive");
    requireFinitePositive(f.period_abort_s, "safety.period_abort_s");
    requireFinitePositive(f.callback_budget_us, "safety.callback_budget_us");
    requireAtLeastOne(f.callback_budget_consecutive, "safety.callback_budget_consecutive");
    if (!(f.min_success_rate >= 0.0 && f.min_success_rate <= 1.0)) {
      throw std::runtime_error("config: safety.min_success_rate must be in [0, 1]");
    }
    requireAtLeastOne(f.success_rate_consecutive, "safety.success_rate_consecutive");
    if (f.watchdog_grace_cycles < 0) throw std::runtime_error("config: safety.watchdog_grace_cycles must be >= 0");
    requireFinitePositive(f.stop_duration_s, "safety.stop_duration_s");
    requireFinitePositive(f.stop_damping_scale, "safety.stop_damping_scale");
  }
  {
    const Node s = section(root, "collision");
    CollisionConfig& k = c.collision;
    k.lower_torque_acc = reqArray<7>(s, "collision", "lower_torque_acc");
    k.upper_torque_acc = reqArray<7>(s, "collision", "upper_torque_acc");
    k.lower_torque_nom = reqArray<7>(s, "collision", "lower_torque_nom");
    k.upper_torque_nom = reqArray<7>(s, "collision", "upper_torque_nom");
    k.lower_force_acc = reqArray<6>(s, "collision", "lower_force_acc");
    k.upper_force_acc = reqArray<6>(s, "collision", "upper_force_acc");
    k.lower_force_nom = reqArray<6>(s, "collision", "lower_force_nom");
    k.upper_force_nom = reqArray<6>(s, "collision", "upper_force_nom");
    k.joint_impedance = reqArray<7>(s, "collision", "joint_impedance");
    k.cartesian_impedance = reqArray<6>(s, "collision", "cartesian_impedance");
  }
  {
    const Node s = section(root, "end_effector");
    EndEffectorConfig& e = c.end_effector;
    e.set_ee = req<bool>(s, "end_effector", "set_ee");
    e.ne_t_ee = reqArray<16>(s, "end_effector", "ne_t_ee");
    e.set_load = req<bool>(s, "end_effector", "set_load");
    e.load_mass = req<double>(s, "end_effector", "load_mass");
    e.f_x_cload = reqArray<3>(s, "end_effector", "f_x_cload");
    e.load_inertia = reqArray<9>(s, "end_effector", "load_inertia");
    requireFiniteNonNegative(e.load_mass, "end_effector.load_mass");
  }
  {
    const Node s = section(root, "preflight");
    PreflightConfig& p = c.preflight;
    p.abort_on_failure = req<bool>(s, "preflight", "abort_on_failure");
    p.m_ee_max_kg = req<double>(s, "preflight", "m_ee_max_kg");
    p.m_load_max_kg = req<double>(s, "preflight", "m_load_max_kg");
    p.fk_position_tol_m = req<double>(s, "preflight", "fk_position_tol_m");
    p.fk_rotation_tol_deg = req<double>(s, "preflight", "fk_rotation_tol_deg");
    p.check_gravity = req<bool>(s, "preflight", "check_gravity");
    p.gravity_vs_measured_tol_nm = req<double>(s, "preflight", "gravity_vs_measured_tol_nm");
    p.gravity_still_max_dq_rad_s = req<double>(s, "preflight", "gravity_still_max_dq_rad_s");
    p.gravity_urdf_warn_nm = req<double>(s, "preflight", "gravity_urdf_warn_nm");
    p.max_damping_per_sample = req<double>(s, "preflight", "max_damping_per_sample");
    p.fail_damping_per_sample = req<double>(s, "preflight", "fail_damping_per_sample");
    p.warn_gain_margin = req<double>(s, "preflight", "warn_gain_margin");
    p.fail_gain_margin = req<double>(s, "preflight", "fail_gain_margin");
    p.fail_spectral_radius = req<double>(s, "preflight", "fail_spectral_radius");
    requireFinitePositive(p.warn_gain_margin, "preflight.warn_gain_margin");
    requireFinitePositive(p.fail_gain_margin, "preflight.fail_gain_margin");
    requireFinitePositive(p.fail_spectral_radius, "preflight.fail_spectral_radius");
    if (p.fail_gain_margin > p.warn_gain_margin) {
      throw std::runtime_error("config: preflight.fail_gain_margin must be <= preflight.warn_gain_margin");
    }
    requireFinitePositive(p.max_damping_per_sample, "preflight.max_damping_per_sample");
    requireFinitePositive(p.fail_damping_per_sample, "preflight.fail_damping_per_sample");
    if (p.fail_damping_per_sample < p.max_damping_per_sample) {
      throw std::runtime_error("config: preflight.fail_damping_per_sample must be >= preflight.max_damping_per_sample");
    }
    requireFiniteNonNegative(p.m_ee_max_kg, "preflight.m_ee_max_kg");
    requireFiniteNonNegative(p.m_load_max_kg, "preflight.m_load_max_kg");
    requireFinitePositive(p.fk_position_tol_m, "preflight.fk_position_tol_m");
    requireFinitePositive(p.fk_rotation_tol_deg, "preflight.fk_rotation_tol_deg");
    requireFinitePositive(p.gravity_vs_measured_tol_nm, "preflight.gravity_vs_measured_tol_nm");
    requireFinitePositive(p.gravity_still_max_dq_rad_s, "preflight.gravity_still_max_dq_rad_s");
    requireFinitePositive(p.gravity_urdf_warn_nm, "preflight.gravity_urdf_warn_nm");
  }
  {
    const Node s = section(root, "runtime");
    c.runtime.mlockall = req<bool>(s, "runtime", "mlockall");
    c.runtime.sched_fifo = req<bool>(s, "runtime", "sched_fifo");
    c.runtime.expected_libfranka_dir = req<std::string>(s, "runtime", "expected_libfranka_dir");
  }
  {
    const Node s = section(root, "dry_run");
    c.dry_run.q = reqArray<7>(s, "dry_run", "q");
    c.dry_run.paced = req<bool>(s, "dry_run", "paced");
    c.dry_run.ignore_tracking_error = req<bool>(s, "dry_run", "ignore_tracking_error");
  }
  return c;
}

}  // namespace real_replay
