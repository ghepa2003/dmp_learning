#include "real_replay/hw_config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>

namespace real_replay {

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;

YAML::Node req(const YAML::Node& n, const std::string& key, const std::string& where) {
  if (!n || !n.IsMap()) throw std::runtime_error("hw_config: '" + where + "' missing or not a map");
  const YAML::Node v = n[key];
  if (!v) throw std::runtime_error("hw_config: missing key '" + where + "." + key + "'");
  return v;
}

double num(const YAML::Node& n, const std::string& key, const std::string& where) {
  const double v = req(n, key, where).as<double>();
  if (!std::isfinite(v)) throw std::runtime_error("hw_config: '" + where + "." + key + "' is not finite");
  return v;
}

template <std::size_t N>
std::array<double, N> arr(const YAML::Node& n, const std::string& key, const std::string& where, double scale = 1.0) {
  const YAML::Node v = req(n, key, where);
  if (!v.IsSequence() || v.size() != N) {
    throw std::runtime_error("hw_config: '" + where + "." + key + "' must be a list of " + std::to_string(N) + " numbers");
  }
  std::array<double, N> out{};
  for (std::size_t i = 0; i < N; ++i) {
    out[i] = v[i].as<double>() * scale;
    if (!std::isfinite(out[i])) throw std::runtime_error("hw_config: '" + where + "." + key + "' has a non-finite value");
  }
  return out;
}

std::string resolve(const std::string& base_dir, const std::string& p) {
  if (p.empty()) return p;
  const std::filesystem::path q(p);
  return q.is_absolute() ? p : (std::filesystem::path(base_dir) / q).lexically_normal().string();
}

void check(bool ok, const std::string& msg) {
  if (!ok) throw std::runtime_error("hw_config: " + msg);
}

}  // namespace

bool massConfirmationOk(const std::string& typed, double expected_kg, double tol_kg) {
  const std::size_t b = typed.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return false;
  const std::size_t e = typed.find_last_not_of(" \t\r\n");
  const std::string s = typed.substr(b, e - b + 1);
  char* end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (end == s.c_str() || *end != '\0' || !std::isfinite(v)) return false;
  return std::fabs(v - expected_kg) <= tol_kg + 1e-12;
}

HwConfig loadHwConfig(const std::string& path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception& e) {
    throw std::runtime_error("hw_config: cannot read '" + path + "': " + e.what());
  }
  HwConfig c;
  c.path = path;
  const std::string dir = std::filesystem::absolute(path).parent_path().string();

  const YAML::Node sc = req(root, "scene", "");
  c.scene.center_m = arr<3>(sc, "center_m", "scene");
  const auto tilt = arr<2>(sc, "axis_tilt_deg", "scene", kDeg);
  c.scene.tilt_x_rad = tilt[0];
  c.scene.tilt_y_rad = tilt[1];
  c.scene.omega_rad_s = num(sc, "omega_deg_s", "scene") * kDeg;
  c.scene.flange_to_center_m = num(sc, "flange_to_center_m", "scene");
  c.scene.collar_phase_offset_rad = num(sc, "collar_phase_offset_deg", "scene") * kDeg;
  const YAML::Node qp = req(sc, "q7_phase", "scene");
  c.scene.q7_phase_offset_rad = num(qp, "offset_deg", "scene.q7_phase") * kDeg;
  c.scene.q7_phase_sign = static_cast<int>(num(qp, "sign", "scene.q7_phase"));
  check(c.scene.q7_phase_sign == 1 || c.scene.q7_phase_sign == -1, "scene.q7_phase.sign must be +1 or -1");
  const auto pw = arr<2>(sc, "psi_window_deg", "scene", kDeg);
  c.scene.psi_lo_rad = pw[0];
  c.scene.psi_hi_rad = pw[1];
  check(pw[0] < pw[1], "scene.psi_window_deg must be [lo, hi] with lo < hi");
  check(pw[0] >= -270.0 * kDeg && pw[1] < 90.0 * kDeg, "scene.psi_window_deg must lie in the phase branch [-270, 90) deg");
  const YAML::Node b2 = req(sc, "base2_in_base1", "scene");
  c.scene.base2_t_m = arr<3>(b2, "t_m", "scene.base2_in_base1");
  c.scene.base2_yaw_rad = num(b2, "yaw_deg", "scene.base2_in_base1") * kDeg;

  const YAML::Node mk = req(root, "mockup", "");
  c.mockup.mass_kg = num(mk, "mass_kg", "mockup");
  check(c.mockup.mass_kg >= 0.0 && c.mockup.mass_kg <= 3.0, "mockup.mass_kg must be in [0, 3]");
  c.mockup.com_m = arr<3>(mk, "com_m", "mockup");

  const YAML::Node r2 = req(root, "robot2", "");
  c.robot2.ip = req(r2, "ip", "robot2").as<std::string>();
  c.robot2.real_replay_config = resolve(dir, req(r2, "real_replay_config", "robot2").as<std::string>());
  const YAML::Node st = req(r2, "start", "robot2");
  c.robot2.start_mode = req(st, "mode", "robot2.start").as<std::string>();
  check(c.robot2.start_mode == "q" || c.robot2.start_mode == "center_tilt", "robot2.start.mode must be 'q' or 'center_tilt'");
  c.robot2.start_q_rad = arr<7>(st, "q_deg", "robot2.start", kDeg);
  c.robot2.psi0_rad = num(st, "psi0_deg", "robot2.start") * kDeg;
  const auto w7 = arr<2>(r2, "q7_window_deg", "robot2", kDeg);
  c.robot2.q7_lo_rad = w7[0];
  c.robot2.q7_hi_rad = w7[1];
  check(w7[0] < w7[1], "robot2.q7_window_deg must be [lo, hi] with lo < hi");
  c.robot2.window_margin_rad = num(r2, "window_margin_deg", "robot2") * kDeg;
  check(c.robot2.window_margin_rad >= 0.0, "robot2.window_margin_deg must be >= 0");
  c.robot2.m_ee_max_kg = num(r2, "m_ee_max_kg", "robot2");
  c.robot2.m_load_tol_kg = num(r2, "m_load_tol_kg", "robot2");
  const YAML::Node tol = req(r2, "tol", "robot2");
  c.robot2.tol_center_m = num(tol, "center_m", "robot2.tol");
  c.robot2.tol_axis_rad = num(tol, "axis_deg", "robot2.tol") * kDeg;
  c.robot2.tol_fk_m = num(tol, "fk_vs_robot_m", "robot2.tol");
  c.robot2.tol_fk_rad = num(tol, "fk_vs_robot_deg", "robot2.tol") * kDeg;
  const YAML::Node ct = req(r2, "control", "robot2");
  c.robot2.kp = arr<7>(ct, "kp", "robot2.control");
  c.robot2.kd = arr<7>(ct, "kd", "robot2.control");
  for (std::size_t k = 0; k < 7; ++k) {
    check(c.robot2.kp[k] >= 0.0 && c.robot2.kp[k] <= 1000.0 && c.robot2.kd[k] >= 0.0 && c.robot2.kd[k] <= 1000.0,
          "robot2.control.kp/kd must be in [0, 1000]");
  }
  c.robot2.max_track_err_rad = num(ct, "max_track_err_rad", "robot2.control");
  check(c.robot2.max_track_err_rad > 0.0 && c.robot2.max_track_err_rad <= 1.0, "robot2.control.max_track_err_rad must be in (0, 1]");
  c.robot2.ramp_s = num(ct, "ramp_s", "robot2.control");
  c.robot2.stop_ramp_s = num(ct, "stop_ramp_s", "robot2.control");
  c.robot2.hold_s = num(ct, "hold_s", "robot2.control");
  check(c.robot2.ramp_s > 0.0 && c.robot2.ramp_s <= 60.0 && c.robot2.stop_ramp_s > 0.0 && c.robot2.stop_ramp_s <= 60.0 &&
            c.robot2.hold_s >= 0.0 && c.robot2.hold_s <= 60.0,
        "robot2.control ramps in (0, 60] s, hold in [0, 60] s");
  c.robot2.max_travel_rad = num(ct, "max_travel_deg", "robot2.control") * kDeg;
  check(c.robot2.max_travel_rad >= 0.0, "robot2.control.max_travel_deg must be >= 0");
  c.robot2.max_wait_start_s = num(ct, "max_wait_start_s", "robot2.control");
  check(c.robot2.max_wait_start_s > 0.0 && c.robot2.max_wait_start_s <= 3600.0, "robot2.control.max_wait_start_s in (0, 3600]");
  c.robot2.countdown_s = static_cast<int>(num(ct, "countdown_s", "robot2.control"));
  check(c.robot2.countdown_s >= 0 && c.robot2.countdown_s <= 30, "robot2.control.countdown_s in [0, 30]");
  c.robot2.omega_window_s = num(ct, "omega_window_s", "robot2.control");
  c.robot2.omega_sample_hz = num(ct, "omega_sample_hz", "robot2.control");
  check(c.robot2.omega_window_s >= 0.1 && c.robot2.omega_window_s <= 30.0, "robot2.control.omega_window_s in [0.1, 30]");
  check(c.robot2.omega_sample_hz >= 10.0 && c.robot2.omega_sample_hz <= 1000.0, "robot2.control.omega_sample_hz in [10, 1000]");
  c.robot2.phase_mismatch_warn_rad = num(ct, "phase_mismatch_warn_deg", "robot2.control") * kDeg;
  const YAML::Node dr = req(r2, "dry_run", "robot2");
  c.robot2.dry_period_jitter_s = num(dr, "period_jitter_s", "robot2.dry_run");
  check(c.robot2.dry_period_jitter_s >= 0.0 && c.robot2.dry_period_jitter_s < 0.0005, "robot2.dry_run.period_jitter_s in [0, 0.0005)");
  c.robot2.dry_meas_delay_cycles = static_cast<int>(num(dr, "meas_delay_cycles", "robot2.dry_run"));
  check(c.robot2.dry_meas_delay_cycles >= 0 && c.robot2.dry_meas_delay_cycles <= 50, "robot2.dry_run.meas_delay_cycles in [0, 50]");

  const YAML::Node u = req(root, "udp", "");
  c.udp.host = req(u, "host", "udp").as<std::string>();
  c.udp.publish_port = static_cast<int>(num(u, "publish_port", "udp"));
  c.udp.monitor_port = static_cast<int>(num(u, "monitor_port", "udp"));
  c.udp.command_port = static_cast<int>(num(u, "command_port", "udp"));
  c.udp.rate_hz = num(u, "rate_hz", "udp");
  c.udp.T_hb_s = num(u, "T_hb_s", "udp");
  c.udp.T_age_s = num(u, "T_age_s", "udp");
  check(c.udp.publish_port > 0 && c.udp.publish_port < 65536 && c.udp.command_port > 0 && c.udp.command_port < 65536 &&
            c.udp.monitor_port >= 0 && c.udp.monitor_port < 65536,
        "udp ports out of range");
  check(c.udp.rate_hz >= 1.0 && c.udp.rate_hz <= 1000.0, "udp.rate_hz in [1, 1000]");
  check(c.udp.T_hb_s > 0.0, "udp.T_hb_s must be > 0");

  const YAML::Node pa = req(root, "paths", "");
  c.log_dir = resolve(dir, req(pa, "log_dir", "paths").as<std::string>());
  return c;
}

}  // namespace real_replay
