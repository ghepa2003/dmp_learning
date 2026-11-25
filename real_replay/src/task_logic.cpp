#include "real_replay/task_logic.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "real_replay/satellite_phase.hpp"
#include "real_replay/satellite_udp.hpp"

namespace real_replay {
namespace task {

namespace {

constexpr double kDeg = sat::kDeg;

std::string f(const char* fmt, double a = 0.0, double b = 0.0, double c = 0.0, double d = 0.0) {
  char buf[400];
  std::snprintf(buf, sizeof buf, fmt, a, b, c, d);
  return buf;
}

YAML::Node req(const YAML::Node& n, const std::string& key, const std::string& where) {
  if (!n || !n.IsMap()) throw std::runtime_error(where + ": missing or not a map");
  const YAML::Node v = n[key];
  if (!v) throw std::runtime_error("missing key '" + where + "." + key + "'");
  return v;
}

double num(const YAML::Node& n, const std::string& key, const std::string& where) {
  try {
    return req(n, key, where).as<double>();
  } catch (const YAML::Exception&) {
    throw std::runtime_error("'" + where + "." + key + "' is not a number");
  }
}

Eigen::Vector3d vec3(const YAML::Node& n, const std::string& key, const std::string& where) {
  const YAML::Node v = req(n, key, where);
  if (!v.IsSequence() || v.size() != 3) throw std::runtime_error("'" + where + "." + key + "' must be a list of 3 numbers");
  return Eigen::Vector3d(v[0].as<double>(), v[1].as<double>(), v[2].as<double>());
}

Eigen::Quaterniond quatWxyz(const YAML::Node& n, const std::string& key, const std::string& where) {
  const YAML::Node v = req(n, key, where);
  if (!v.IsSequence() || v.size() != 4) throw std::runtime_error("'" + where + "." + key + "' must be [w, x, y, z]");
  return Eigen::Quaterniond(v[0].as<double>(), v[1].as<double>(), v[2].as<double>(), v[3].as<double>());
}

bool finiteQ(const Eigen::Quaterniond& q) { return q.coeffs().allFinite(); }

}  // namespace

// ---- States -----------------------------------------------------------------------------------------------------
const char* toString(TaskState s) {
  switch (s) {
    case TaskState::kInit: return "INIT";
    case TaskState::kWaitCmd: return "WAIT_CMD";
    case TaskState::kValidate: return "VALIDATE";
    case TaskState::kWaitTrigger: return "WAIT_TRIGGER";
    case TaskState::kApproach: return "APPROACH";
    case TaskState::kHoldSafe: return "HOLD_SAFE";
    case TaskState::kCloseFollow: return "CLOSE_FOLLOW";
    case TaskState::kSqueeze: return "SQUEEZE";
    case TaskState::kRelease: return "RELEASE";
    case TaskState::kFaultNoMotion: return "FAULT_NOMOTION";
    case TaskState::kFaultMotion: return "FAULT_MOTION";
    case TaskState::kFaultGrasp: return "FAULT_GRASP";
    case TaskState::kDone: return "DONE";
  }
  return "?";
}

bool isStage3Stub(TaskState s) {
  return s == TaskState::kCloseFollow || s == TaskState::kSqueeze || s == TaskState::kRelease || s == TaskState::kFaultGrasp;
}

bool transitionAllowed(TaskState from, TaskState to) {
  using S = TaskState;
  if (from == S::kDone) return false;
  if (to == S::kFaultNoMotion) return from == S::kInit || from == S::kWaitCmd || from == S::kValidate || from == S::kWaitTrigger;
  if (to == S::kFaultMotion) {
    return from == S::kApproach || from == S::kHoldSafe || from == S::kCloseFollow || from == S::kSqueeze || from == S::kRelease;
  }
  if (to == S::kFaultGrasp) return from == S::kCloseFollow || from == S::kSqueeze || from == S::kRelease;  // stage 3
  switch (from) {
    case S::kInit: return to == S::kWaitCmd;
    case S::kWaitCmd: return to == S::kValidate;
    case S::kValidate: return to == S::kWaitTrigger;
    case S::kWaitTrigger: return to == S::kApproach;
    case S::kApproach: return to == S::kHoldSafe || to == S::kCloseFollow;  // stage 3: APPROACH -> CLOSE_FOLLOW
    case S::kHoldSafe: return to == S::kDone;
    case S::kCloseFollow: return to == S::kSqueeze;   // stage 3
    case S::kSqueeze: return to == S::kRelease;       // stage 3
    case S::kRelease: return to == S::kHoldSafe;      // stage 3
    case S::kFaultNoMotion:
    case S::kFaultMotion:
    case S::kFaultGrasp: return to == S::kDone;       // controlled stop after Ctrl-C / linger
    case S::kDone: return false;
  }
  return false;
}

// ---- GraspCommand -----------------------------------------------------------------------------------------------
GraspCommand loadGraspCommand(const std::string& path) {
  YAML::Node r;
  try {
    r = YAML::LoadFile(path);
  } catch (const std::exception& e) {
    throw std::runtime_error("grasp command '" + path + "': " + e.what());
  }
  const std::string w = "grasp_command";
  GraspCommand c;
  c.path = path;
  try {
    c.format = req(r, "format", w).as<std::string>();
    c.status = req(r, "status", w).as<std::string>();
    c.frame_id = req(r, "frame_id", w).as<std::string>();
    c.stamp_unix_s = num(r, "stamp_unix_s", w);
    // degrees -> rad: the ONLY conversion point of the task layer
    c.theta_star = num(r, "theta_star_deg", w) * kDeg;
    c.omega = num(r, "omega_deg_s", w) * kDeg;
    c.contact_time_s = num(r, "contact_time_s", w);
    c.weights_sha256 = req(r, "weights_sha256", w).as<std::string>();
    const YAML::Node sp = req(r, "start_pose", w);
    c.start_p = vec3(sp, "position_m", w + ".start_pose");
    c.start_q = quatWxyz(sp, "quaternion_wxyz", w + ".start_pose");
    const YAML::Node gp = req(r, "goal_pose", w);
    if (!gp.IsMap()) throw std::runtime_error("'grasp_command.goal_pose' must be {position_m, quaternion_wxyz} (null not allowed in stage 2)");
    c.goal_p = vec3(gp, "position_m", w + ".goal_pose");
    c.goal_q = quatWxyz(gp, "quaternion_wxyz", w + ".goal_pose");
    if (r["k"] && !r["k"].IsNull()) c.k = r["k"].as<int>();
    if (r["wrist_flip"] && !r["wrist_flip"].IsNull()) c.wrist_flip = r["wrist_flip"].as<bool>();
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("grasp command '" + path + "': " + e.what());
  } catch (const std::runtime_error& e) {
    throw std::runtime_error("grasp command '" + path + "': " + e.what());
  }
  return c;
}

// ---- Config -----------------------------------------------------------------------------------------------------
HwTaskConfig loadHwTaskConfig(const std::string& path) {
  YAML::Node r;
  try {
    r = YAML::LoadFile(path);
  } catch (const std::exception& e) {
    throw std::runtime_error("hw_config '" + path + "': " + e.what());
  }
  HwTaskConfig c;
  TaskParams& p = c.p;
  try {
    const YAML::Node sc = req(r, "scene", "");
    p.omega_cfg = num(sc, "omega_deg_s", "scene") * kDeg;
    const YAML::Node pw = req(sc, "psi_window_deg", "scene");
    p.psi_lo = pw[0].as<double>() * kDeg;
    p.psi_hi = pw[1].as<double>() * kDeg;
    const YAML::Node r2 = req(r, "robot2", "");
    p.window_margin = num(r2, "window_margin_deg", "robot2") * kDeg;
    const YAML::Node ct = req(r2, "control", "robot2");
    p.sat_ramp_s = num(ct, "ramp_s", "robot2.control");
    p.sat_omega_window_s = num(ct, "omega_window_s", "robot2.control");
    const YAML::Node tm = req(r, "timing", "");
    p.contact_delay_s = num(tm, "contact_delay_s", "timing");
    p.T_arm_min_s = num(tm, "T_arm_min_s", "timing");
    p.margin_arm_s = num(tm, "margin_arm_s", "timing");
    p.T_close_max_s = num(tm, "T_close_max_s", "timing");
    p.T_follow_s = num(tm, "T_follow_s", "timing");
    p.T_ramp_s = num(tm, "T_ramp_s", "timing");
    p.trigger_timeout_margin_s = num(tm, "trigger_timeout_margin_s", "timing");
    const YAML::Node pl = req(r, "plan", "");
    p.omega_cmd_tol = num(pl, "omega_tol_deg_s", "plan") * kDeg;
    const YAML::Node dom = req(pl, "omega_domain_deg_s", "plan");
    p.omega_domain[0] = dom[0].as<double>() * kDeg;
    p.omega_domain[1] = dom[1].as<double>() * kDeg;
    p.tau_tol_s = num(pl, "tau_tol_s", "plan");
    const YAML::Node u = req(r, "udp", "");
    c.udp_host = req(u, "host", "udp").as<std::string>();
    c.udp_state_port = static_cast<int>(num(u, "publish_port", "udp"));
    c.udp_command_port = static_cast<int>(num(u, "command_port", "udp"));
    p.T_age_s = num(u, "T_age_s", "udp");
    const YAML::Node t = req(r, "task", "");
    p.T_hold_stage2_s = num(t, "T_hold_stage2_s", "task");
    p.cmd_max_age_s = num(t, "cmd_max_age_s", "task");
    p.start_pos_tol_m = num(t, "start_pos_tol_m", "task");
    p.start_rot_tol_rad = num(t, "start_rot_tol_rad", "task");
    p.sat_timeout_s = num(t, "sat_timeout_s", "task");
    p.sat_start_timeout_s = num(t, "sat_start_timeout_s", "task");
    p.trigger_max_age_s = num(t, "trigger_max_age_s", "task");
    p.omega_meas_tol = num(t, "omega_meas_tol_deg_s", "task") * kDeg;
    p.trigger_max_d = num(t, "trigger_max_d_deg", "task") * kDeg;
    p.fault_linger_s = num(t, "fault_linger_s", "task");
    c.hb_send_hz = num(t, "hb_send_hz", "task");
    c.jitter_period_s = num(t, "jitter_period_s", "task");
    c.jitter_consecutive = static_cast<int>(num(t, "jitter_consecutive", "task"));
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("hw_config '" + path + "': " + e.what());
  } catch (const std::runtime_error& e) {
    throw std::runtime_error("hw_config '" + path + "': " + e.what());
  }
  auto bad = [&](bool cond, const char* m) {
    if (cond) throw std::runtime_error(std::string("hw_config '") + path + "': " + m);
  };
  bad(!(p.psi_lo < p.psi_hi), "scene.psi_window_deg must be [lo, hi] with lo < hi");
  bad(!(p.contact_delay_s >= 0.0), "timing.contact_delay_s must be >= 0");
  bad(!(p.T_hold_stage2_s >= 0.0), "task.T_hold_stage2_s must be >= 0");
  bad(!(p.sat_timeout_s > 0.0 && p.trigger_max_age_s > 0.0 && p.T_age_s > 0.0), "timeouts must be > 0");
  bad(!(p.trigger_max_d > 0.0 && p.trigger_max_d <= sat::kPi), "task.trigger_max_d_deg in (0, 180]");
  bad(!(c.hb_send_hz >= 2.0 && c.hb_send_hz <= 200.0), "task.hb_send_hz in [2, 200]");
  bad(!(c.jitter_period_s > 0.001 && c.jitter_consecutive >= 1), "task.jitter_period_s > 1 ms, jitter_consecutive >= 1");
  bad(!(p.fault_linger_s >= 0.0), "task.fault_linger_s >= 0");
  return c;
}

// ---- Feasibility ------------------------------------------------------------------------------------------------
Feasibility feasibility(const GraspCommand& cmd, const TaskParams& p, double tau, double theta0) {
  Feasibility r;
  r.theta0 = theta0;
  const double w = cmd.omega;
  const double m = p.window_margin;
  auto add = [&](const std::string& name, bool ok, const std::string& det, bool blocking = true) {
    r.checks.push_back({name, ok, blocking, det});
  };
  if (!(std::isfinite(w) && w != 0.0 && std::isfinite(theta0) && std::isfinite(tau) && std::isfinite(cmd.theta_star))) {
    add("feasibility inputs finite, omega != 0", false, f("omega %.4f deg/s, theta0 %.3f deg", w / kDeg, theta0 / kDeg));
    return r;
  }
  r.s = w > 0.0 ? 1.0 : -1.0;
  r.T_c = tau + p.contact_delay_s;
  const double theta_star = sat::unwrapPhase(cmd.theta_star);
  r.theta_trig = sat::wrapPi(theta_star - w * r.T_c);
  r.theta_trig_branch = sat::unwrapPhase(r.theta_trig);
  const double gap = r.s * (r.theta_trig_branch - theta0);
  r.angle_before_valid = std::fabs(w) * (p.sat_ramp_s / 2.0 + p.sat_omega_window_s);
  r.need_a = r.angle_before_valid + std::fabs(w) * p.margin_arm_s;
  const double arm = std::fabs(w) * p.T_arm_min_s;
  add("A: s*(theta_trig - theta0) >= |omega|*(ramp_s/2 + omega_window_s + margin_arm_s)", gap >= r.need_a - 1e-12,
      f("%.3f deg, required >= %.3f deg", gap / kDeg, r.need_a / kDeg));
  add("B: s*(theta_trig - theta0) >= |omega|*T_arm_min", gap >= arm - 1e-12, f("%.3f deg, required >= %.3f deg", gap / kDeg, arm / kDeg));
  add("C: theta0 inside the window with margin", sat::insideWindow(theta0, p.psi_lo, p.psi_hi, m),
      f("theta0 %.3f deg, admitted [%.1f, %.1f]", theta0 / kDeg, (p.psi_lo + m) / kDeg, (p.psi_hi - m) / kDeg));
  r.theta_end = theta_star + w * (p.T_close_max_s + p.T_follow_s + p.T_ramp_s);
  add("D: theta_end = theta* + omega*(T_close_max+T_follow+T_ramp) inside the window with margin",
      sat::insideWindow(r.theta_end, p.psi_lo, p.psi_hi, m), f("theta_end %.3f deg", r.theta_end / kDeg));
  const double aw = std::fabs(w);
  const bool in_dom = std::fabs(aw - p.omega_domain[0]) <= p.omega_cmd_tol || std::fabs(aw - p.omega_domain[1]) <= p.omega_cmd_tol;
  add("E: |omega| in the domain", in_dom, f("|omega| %.3f deg/s, domain {%.1f, %.1f}", aw / kDeg, p.omega_domain[0] / kDeg, p.omega_domain[1] / kDeg),
      /*blocking=*/false);
  const double need = std::max(arm, r.need_a);
  if (r.s < 0) {
    r.theta0_lo = std::max(p.psi_lo + m, r.theta_trig_branch + need);
    r.theta0_hi = p.psi_hi - m;
  } else {
    r.theta0_lo = p.psi_lo + m;
    r.theta0_hi = std::min(p.psi_hi - m, r.theta_trig_branch - need);
  }
  r.t_trigger_s = (r.theta_trig_branch - theta0) / w;
  r.trigger_timeout_s = r.t_trigger_s + p.sat_ramp_s / 2.0 + p.trigger_timeout_margin_s;
  r.ok = true;
  for (const Check& c : r.checks) {
    if (c.blocking && !c.ok) r.ok = false;
  }
  return r;
}

std::vector<Check> validateCommand(const GraspCommand& c, const TaskParams& p, const ValidationContext& x) {
  std::vector<Check> out;
  auto add = [&](const std::string& name, bool ok, const std::string& det, bool blocking = true) {
    out.push_back({name, ok, blocking, det});
  };
  add("format grasp_command_v2", c.format == "grasp_command_v2", "format = " + c.format);
  add("status SELECTED", c.status == "SELECTED", "status = " + c.status);
  add("frame_id world", c.frame_id == "world", "frame_id = " + c.frame_id);
  const bool fin = std::isfinite(c.stamp_unix_s) && std::isfinite(c.theta_star) && std::isfinite(c.omega) && std::isfinite(c.contact_time_s) &&
                   c.start_p.allFinite() && c.goal_p.allFinite() && finiteQ(c.start_q) && finiteQ(c.goal_q);
  add("all fields finite", fin, fin ? "ok" : "a numeric field is NaN/inf");
  const bool qn = fin && std::fabs(c.start_q.norm() - 1.0) <= 1e-3 && std::fabs(c.goal_q.norm() - 1.0) <= 1e-3;
  add("quaternions unit (+-1e-3)", qn, fin ? f("|q_start| %.6f, |q_goal| %.6f", c.start_q.norm(), c.goal_q.norm()) : "");
  const double age = x.now_unix_s - c.stamp_unix_s;
  add("stamp not in the future", fin && age >= -1.0, f("age %.3f s (tol -1 s)", age));
  add("stamp age <= cmd_max_age_s", fin && age <= p.cmd_max_age_s, f("age %.1f s, max %.1f s", age, p.cmd_max_age_s));
  add("sha256(weights) == weights_sha256", c.weights_sha256 == x.weights_sha256,
      "command " + c.weights_sha256.substr(0, 16) + "..., file " + x.weights_sha256.substr(0, 16) + "...");
  add("contact_time_s == tau (1e-6)", fin && std::fabs(c.contact_time_s - x.tau) <= p.tau_tol_s, f("contact_time %.6f, tau %.6f", c.contact_time_s, x.tau));
  add("omega != 0", fin && c.omega != 0.0, f("omega %.4f deg/s", c.omega / kDeg));
  add("|omega - scene.omega_deg_s| <= plan.omega_tol_deg_s", fin && std::fabs(c.omega - p.omega_cfg) <= p.omega_cmd_tol,
      f("command %.4f, config %.4f deg/s", c.omega / kDeg, p.omega_cfg / kDeg));
  const bool tcp_ok = x.tcp_p.allFinite() && finiteQ(x.tcp_q);
  const double dp = tcp_ok && fin ? (x.tcp_p - c.start_p).norm() : kNaN;
  const double da = tcp_ok && qn ? x.tcp_q.normalized().angularDistance(c.start_q.normalized()) : kNaN;
  add("|start_position - measured TCP| <= start_pos_tol_m", std::isfinite(dp) && dp <= p.start_pos_tol_m, f("%.3f mm (tol %.3f mm)", dp * 1e3, p.start_pos_tol_m * 1e3));
  add("angle(start orientation, measured TCP) <= start_rot_tol_rad", std::isfinite(da) && da <= p.start_rot_tol_rad,
      f("%.4f rad (tol %.4f)", da, p.start_rot_tol_rad));
  add("trajectory.stop_at_fraction == 1 (approach ends at the goal)", x.stop_at_fraction == 1.0, f("%.3f", x.stop_at_fraction));
  add("joint limits along the goal trajectory (with margin)", x.path_checked && x.path_ok,
      x.path_checked ? x.path_detail : "not checked");
  return out;
}

// ---- Filter -----------------------------------------------------------------------------------------------------
const char* toString(SampleVerdict v) {
  switch (v) {
    case SampleVerdict::kOk: return "ok";
    case SampleVerdict::kSeq: return "seq_not_increasing";
    case SampleVerdict::kClockBack: return "clock_backwards";
    case SampleVerdict::kAge: return "age_out_of_range";
    case SampleVerdict::kNonFinite: return "non_finite";
  }
  return "?";
}

SampleVerdict SatFilter::accept(const SatSample& s, double now_mono_s) {
  SampleVerdict v = SampleVerdict::kOk;
  const double age = now_mono_s - static_cast<double>(s.t_mono_ns) * 1e-9;
  if (!std::isfinite(s.theta) || s.theta < sat::kBranchLoRad || s.theta >= sat::kBranchLoRad + 2.0 * sat::kPi ||
      (s.omega_valid && !std::isfinite(s.omega_meas))) {
    v = SampleVerdict::kNonFinite;
  } else if (have_ && s.seq <= last_seq_) {
    v = SampleVerdict::kSeq;
  } else if (have_ && s.t_mono_ns < last_t_) {
    v = SampleVerdict::kClockBack;
  } else if (!(age >= 0.0 && age < t_age_s_)) {
    v = SampleVerdict::kAge;
  }
  ++n_[static_cast<int>(v)];
  if (v == SampleVerdict::kOk || v == SampleVerdict::kAge) {
    // an old but well-formed sample still advances seq/stamp (it is not used)
    have_ = true;
    last_seq_ = s.seq;
    last_t_ = s.t_mono_ns;
  }
  return v;
}

// ---- Trigger ----------------------------------------------------------------------------------------------------
PhaseTrigger::PhaseTrigger(double theta_trig, double omega, double max_d)
    : theta_trig_(theta_trig), s_(omega > 0.0 ? 1.0 : -1.0), max_d_(max_d) {
  if (!(omega != 0.0) || !std::isfinite(omega)) throw std::invalid_argument("PhaseTrigger: omega must be finite and != 0");
}

PhaseTrigger::Result PhaseTrigger::update(double theta) {
  Result r;
  r.d = sat::wrapPi(theta - theta_trig_);
  if (!std::isfinite(r.d)) {
    have_prev_ = false;
    return r;
  }
  if (have_prev_ && s_ * d_prev_ < 0.0 && s_ * r.d >= 0.0 && std::fabs(r.d) < max_d_) r.fired = true;
  d_prev_ = r.d;
  have_prev_ = true;
  return r;
}

// ---- Machine ----------------------------------------------------------------------------------------------------
TaskMachine::TaskMachine(const TaskParams& p, double tau, std::uint32_t ignore_mask) : p_(p), tau_(tau), ignore_mask_(ignore_mask) {
  fig_.tau = tau;
}

void TaskMachine::event(double t, const std::string& name, const std::string& detail) { events_.push_back({t, name, detail}); }

void TaskMachine::enter(TaskState s, double t, const std::string& why) {
  if (!transitionAllowed(state_, s)) {
    // programming error: record it and go to the matching fault state
    event(t, "illegal_transition", std::string(toString(state_)) + " -> " + toString(s));
  }
  event(t, std::string("state ") + toString(s), why);
  state_ = s;
  t_state_ = t;
}

bool TaskMachine::submitCommand(const GraspCommand& cmd) {
  if (have_cmd_) {
    event(t_state_, "duplicate_command", "ignored: one GraspCommand per execution (" + cmd.path + ")");
    return false;
  }
  cmd_ = cmd;
  have_cmd_ = true;
  fig_.theta_star = cmd.theta_star;
  fig_.omega_cmd = cmd.omega;
  return true;
}

void TaskMachine::setValidation(const std::vector<Check>& checks, const Feasibility& fe) {
  if (state_ != TaskState::kValidate || validated_ || validation_failed_) {
    event(t_state_, "duplicate_validation", "ignored");
    return;
  }
  feas_ = fe;
  std::string why;
  for (const Check& c : checks) {
    event(t_state_, std::string("check ") + (c.ok ? "PASS" : (c.blocking ? "FAIL" : "WARN")), c.name + ": " + c.detail);
    if (c.blocking && !c.ok) why += c.name + "; ";
  }
  for (const Check& c : fe.checks) {
    event(t_state_, std::string("check ") + (c.ok ? "PASS" : (c.blocking ? "FAIL" : "WARN")), c.name + ": " + c.detail);
    if (c.blocking && !c.ok) why += c.name + "; ";
  }
  if (!why.empty()) {
    validation_failed_ = true;
    fault_reason_ = "VALIDATE failed: " + why;
    return;  // the fault is entered at the next update() (with its timestamp), BEFORE any START
  }
  validated_ = true;
  fig_.theta_trig = fe.theta_trig;
  trigger_ = PhaseTrigger(fe.theta_trig, cmd_.omega, p_.trigger_max_d);
}

void TaskMachine::setConfirmed(bool yes) {
  if (!needsConfirmation()) return;
  if (yes) {
    confirmed_ = true;
    event(t_state_, "operator_confirmed");
  } else {
    validation_failed_ = true;
    fault_reason_ = "operator did not confirm";
  }
}

void TaskMachine::requestStartAgain(double now_s) {
  if (start_sent_ || start_pending_) {
    event(now_s, "duplicate_start", "ignored: START is sent at most once per execution");
  } else {
    event(now_s, "start_request_out_of_sequence", "ignored: START is sent by the machine after VALIDATE + confirmation");
  }
}

void TaskMachine::fault(const MachineInputs& in, const std::string& why) {
  const bool motion = state_ == TaskState::kApproach || state_ == TaskState::kHoldSafe;
  fault_reason_ = why;
  enter(motion ? TaskState::kFaultMotion : TaskState::kFaultNoMotion, in.now_s, why);
  t_fault_ = in.now_s;
  stop_pending_ = true;
}

bool TaskMachine::satFault(const MachineInputs& in, std::string& why) const {
  if (!rej_baseline_) return false;
  if (in.rejected_seq > rej_seq0_) {
    why = "satellite: seq not increasing";
    return true;
  }
  if (in.rejected_clock > rej_clock0_) {
    why = "satellite: clock went backwards (t_mono decreasing)";
    return true;
  }
  if (in.rejected_nonfinite > rej_nan0_) {
    why = "satellite: non-finite theta/omega in a datagram";
    return true;
  }
  if (!in.sat_have || in.now_s - in.sat.rx_s > p_.sat_timeout_s) {
    why = f("satellite: no valid datagram for more than %.1f s", p_.sat_timeout_s);
    return true;
  }
  if (start_sent_) {
    const std::uint32_t bad = in.sat.faults & ~ignore_mask_;
    if (bad != 0) {
      char hex[16];
      std::snprintf(hex, sizeof hex, "0x%x", static_cast<unsigned>(bad));
      why = std::string("satellite reports faults ") + hex + " (bit 1 = our heartbeat lost; see satellite_node)";
      return true;
    }
    if (sat_running_seen_ && in.sat.state != static_cast<std::uint32_t>(sat::SatState::kRunning)) {
      why = f("satellite left RUNNING (state %.0f)", static_cast<double>(in.sat.state));
      return true;
    }
  }
  return false;
}

MachineActions TaskMachine::update(const MachineInputs& in) {
  MachineActions a;
  using S = TaskState;
  const auto running = static_cast<std::uint32_t>(sat::SatState::kRunning);
  const auto wait_start = static_cast<std::uint32_t>(sat::SatState::kWaitStart);
  std::string why;
  const double age = in.sat_have ? in.now_s - static_cast<double>(in.sat.t_mono_ns) * 1e-9 : kNaN;

  switch (state_) {
    case S::kInit:
    case S::kWaitCmd:
      if (in.abort) {
        fault(in, "operator abort (Ctrl-C)");
      } else if (in.loop_finished) {
        fault(in, "control loop ended: " + in.loop_reason);
      } else if (have_cmd_ && in.activation_ready) {
        enter(S::kValidate, in.now_s, cmd_.path);
        rej_seq0_ = in.rejected_seq;
        rej_clock0_ = in.rejected_clock;
        rej_nan0_ = in.rejected_nonfinite;
        rej_baseline_ = true;
      }
      break;

    case S::kValidate:
      if (in.abort) {
        fault(in, "operator abort (Ctrl-C)");
      } else if (in.loop_finished) {
        fault(in, "control loop ended: " + in.loop_reason);
      } else if (validation_failed_) {
        fault(in, fault_reason_);
      } else if (in.jitter_fault) {
        fault(in, "control-cycle jitter above the threshold");
      } else if (in.now_s - t_state_ > p_.sat_timeout_s && (!in.sat_have || in.now_s - in.sat.rx_s > p_.sat_timeout_s)) {
        fault(in, f("satellite: no valid datagram for more than %.1f s", p_.sat_timeout_s));
      } else if (satFault(in, why) && in.sat_have) {
        fault(in, why);
      } else if (validated_ && confirmed_ && !start_sent_) {
        if (in.sat_have && in.sat.state == wait_start) {
          a.send_start = true;
          start_sent_ = true;
          fig_.t_start_sent = in.now_s;
          event(in.now_s, "START sent", f("satellite theta %.3f deg", in.sat.theta / kDeg));
          enter(S::kWaitTrigger, in.now_s);
        } else if (in.sat_have && in.sat.state != wait_start) {
          fault(in, f("satellite not in WAIT_START (state %.0f): START not sent", static_cast<double>(in.sat.state)));
        }
      }
      break;

    case S::kWaitTrigger: {
      a.hb_enable = true;
      if (in.abort) {
        fault(in, "operator abort (Ctrl-C)");
        break;
      }
      if (in.loop_finished) {
        fault(in, "control loop ended: " + in.loop_reason);
        break;
      }
      if (in.jitter_fault) {
        fault(in, "control-cycle jitter above the threshold");
        break;
      }
      if (in.sat_have && in.sat.state == running) sat_running_seen_ = true;
      if (satFault(in, why)) {
        fault(in, why);
        break;
      }
      if (!sat_running_seen_ && in.now_s - fig_.t_start_sent > p_.sat_start_timeout_s) {
        fault(in, f("satellite not RUNNING %.1f s after START", p_.sat_start_timeout_s));
        break;
      }
      if (in.now_s - fig_.t_start_sent > feas_.trigger_timeout_s) {
        fault(in, f("trigger timeout (%.1f s after START): no motion", feas_.trigger_timeout_s));
        break;
      }
      if (in.sat.seq == last_seq_fed_) break;  // no new sample
      last_seq_fed_ = in.sat.seq;
      if (in.sat.omega_valid && std::fabs(in.sat.omega_meas - cmd_.omega) > p_.omega_meas_tol) {
        fault(in, f("omega_meas %.4f deg/s out of band (cmd %.4f, tol %.2f)", in.sat.omega_meas / kDeg, cmd_.omega / kDeg, p_.omega_meas_tol / kDeg));
        break;
      }
      if (armed_ && !in.sat.omega_valid) {
        fault(in, "omega_valid dropped after arming (satellite not at constant speed)");
        break;
      }
      if (!armed_) {
        if (in.sat.omega_valid && age <= p_.trigger_max_age_s) {
          armed_ = true;
          fig_.t_omega_valid = in.now_s;
          event(in.now_s, "omega_valid: trigger armed",
                f("omega_meas %.4f deg/s, theta %.3f deg, age %.1f ms", in.sat.omega_meas / kDeg, in.sat.theta / kDeg, age * 1e3));
        } else {
          break;
        }
      }
      if (!(age <= p_.trigger_max_age_s)) {
        fault(in, f("sample age %.1f ms > %.0f ms at the trigger", age * 1e3, p_.trigger_max_age_s * 1e3));
        break;
      }
      const PhaseTrigger::Result r = trigger_.update(in.sat.theta);
      if (r.fired) {
        const double dp = (in.tcp_p - cmd_.start_p).norm();
        const double da = in.tcp_q.normalized().angularDistance(cmd_.start_q.normalized());
        if (!(dp <= p_.start_pos_tol_m && da <= p_.start_rot_tol_rad)) {
          fault(in, f("TCP %.2f mm / %.4f rad from start_pose at the trigger: no motion", dp * 1e3, da));
          break;
        }
        fig_.t_trigger = in.now_s;
        fig_.theta_at_trigger = in.sat.theta;
        fig_.d_at_trigger = r.d;
        fig_.age_at_trigger = age;
        event(in.now_s, "TRIGGER", f("theta %.4f deg, d %.4f deg, age %.1f ms, theta_trig %.4f deg", in.sat.theta / kDeg, r.d / kDeg, age * 1e3,
                                      feas_.theta_trig / kDeg));
        a.source = SourceCmd::kApproach;
        enter(S::kApproach, in.now_s);
      }
      break;
    }

    case S::kApproach:
      a.hb_enable = true;
      a.source = SourceCmd::kApproach;
      if (in.abort) {
        fault(in, "operator abort (Ctrl-C)");
      } else if (in.loop_finished) {
        fault(in, "control loop ended: " + in.loop_reason);
      } else if (in.jitter_fault) {
        fault(in, "control-cycle jitter above the threshold");
      } else if (satFault(in, why)) {
        fault(in, why);
      } else if (!(age <= p_.T_age_s)) {
        fault(in, f("satellite sample age %.1f ms > T_age_s during the approach", age * 1e3));
      } else if (!in.sat.omega_valid) {
        fault(in, "omega_valid false during the approach");
      } else if (std::fabs(in.sat.omega_meas - cmd_.omega) > p_.omega_meas_tol) {
        fault(in, f("omega_meas %.4f deg/s out of band during the approach", in.sat.omega_meas / kDeg));
      } else {
        if (in.approach_started && !std::isfinite(fig_.t_approach_start)) {
          fig_.t_approach_start = in.now_s;
          event(in.now_s, "APPROACH started (first 1 kHz cycle)");
        }
        if (in.approach_done) {
          fig_.t_approach_end = in.now_s;
          fig_.theta_at_approach_end = in.sat.theta;
          fig_.age_at_approach_end = age;
          event(in.now_s, "APPROACH ended (t = tau, goal)", f("theta %.4f deg, age %.1f ms", in.sat.theta / kDeg, age * 1e3));
          enter(S::kHoldSafe, in.now_s);
        }
      }
      if (state_ != S::kApproach) a.source = SourceCmd::kFreeze;
      break;

    case S::kHoldSafe:
      a.hb_enable = true;
      a.source = SourceCmd::kFreeze;
      if (in.abort) {
        fault(in, "operator abort (Ctrl-C)");
      } else if (in.loop_finished) {
        fault(in, "control loop ended: " + in.loop_reason);
      } else if (satFault(in, why)) {
        fault(in, why);
      } else if (in.now_s - t_state_ >= p_.T_hold_stage2_s) {
        stop_pending_ = true;
        end_loop_ = true;
        event(in.now_s, "HOLD_SAFE over: STOP to the satellite, controlled stop of the loop");
        enter(S::kDone, in.now_s);
      }
      break;

    case S::kFaultNoMotion:
    case S::kFaultMotion:
      a.source = SourceCmd::kFreeze;
      if (in.abort && !end_loop_) {
        end_loop_ = true;
        event(in.now_s, "Ctrl-C in the fault state: controlled stop");
      }
      if (p_.fault_linger_s > 0.0 && in.now_s - t_fault_ >= p_.fault_linger_s && !end_loop_) {
        end_loop_ = true;
        event(in.now_s, "fault_linger_s elapsed: controlled stop");
      }
      break;

    case S::kDone:
      a.source = SourceCmd::kFreeze;
      break;

    case S::kCloseFollow:
    case S::kSqueeze:
    case S::kRelease:
    case S::kFaultGrasp:
      // stage 3 stubs: never entered in stage 2
      fault(in, std::string("stage-3 stub state ") + toString(state_) + " entered");
      break;
  }

  // the fault may have been entered above in this same call
  if (state_ == S::kFaultNoMotion || state_ == S::kFaultMotion) {
    a.source = SourceCmd::kFreeze;
    a.hb_enable = false;
  }
  if (stop_pending_ && !stop_sent_) {
    a.send_stop = true;
    stop_sent_ = true;
    event(in.now_s, "STOP sent to the satellite", fault_reason_);
  }
  a.end_loop = end_loop_;
  return a;
}

int TaskMachine::exitCode() const {
  switch (state_) {
    case TaskState::kDone: return 0;
    case TaskState::kFaultNoMotion: return 5;
    case TaskState::kFaultMotion: return 6;
    default: return 4;
  }
}

}  // namespace task
}  // namespace real_replay
