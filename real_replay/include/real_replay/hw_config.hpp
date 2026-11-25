#pragma once

// Loader of hw/hw_config.yaml (the sections satellite_node needs: scene, mockup, robot2, udp, paths).
// Angles are converted to rad here; the YAML keys ending in _deg are degrees.

#include <array>
#include <string>

namespace real_replay {

struct HwScene {
  std::array<double, 3> center_m{};
  double tilt_x_rad = 0.0, tilt_y_rad = 0.0;
  double omega_rad_s = 0.0;
  double flange_to_center_m = 0.1035;
  double collar_phase_offset_rad = 0.0;
  double q7_phase_offset_rad = 0.0;
  int q7_phase_sign = -1;
  double psi_lo_rad = 0.0, psi_hi_rad = 0.0;
  std::array<double, 3> base2_t_m{};
  double base2_yaw_rad = 0.0;
};

struct HwMockup {
  double mass_kg = 0.0;
  std::array<double, 3> com_m{};
};

struct HwRobot2 {
  std::string ip;
  std::string real_replay_config;  ///< resolved path
  std::string start_mode;          ///< "q" | "center_tilt"
  std::array<double, 7> start_q_rad{};
  double psi0_rad = 0.0;
  double q7_lo_rad = 0.0, q7_hi_rad = 0.0;
  double window_margin_rad = 0.0;
  double m_ee_max_kg = 0.05;
  double m_load_tol_kg = 0.05;
  double tol_center_m = 0.005, tol_axis_rad = 0.0, tol_fk_m = 0.001, tol_fk_rad = 0.0;
  std::array<double, 7> kp{}, kd{};
  double max_track_err_rad = 0.05;
  double ramp_s = 2.0, stop_ramp_s = 2.0, hold_s = 1.0;
  double max_travel_rad = 0.0;  ///< 0 = window only
  double max_wait_start_s = 120.0;
  int countdown_s = 3;
  double omega_window_s = 5.0;
  double omega_sample_hz = 100.0;
  double phase_mismatch_warn_rad = 0.0;
  double dry_period_jitter_s = 0.0;
  int dry_meas_delay_cycles = 0;
};

struct HwUdp {
  std::string host = "127.0.0.1";
  int publish_port = 5601;
  int monitor_port = 0;  ///< 0 = off
  int command_port = 5602;
  double rate_hz = 100.0;
  double T_hb_s = 0.5;
  double T_age_s = 0.1;
};

struct HwConfig {
  std::string path;
  HwScene scene;
  HwMockup mockup;
  HwRobot2 robot2;
  HwUdp udp;
  std::string log_dir;  ///< resolved
};

/// Parses and validates. Throws std::runtime_error with the offending key.
HwConfig loadHwConfig(const std::string& path);

/// Operator confirmation of the mounted payload: `typed` (kg, surrounding blanks allowed, nothing else) must equal
/// `expected_kg` within tol_kg (default 1 g).
bool massConfirmationOk(const std::string& typed, double expected_kg, double tol_kg = 0.001);

}  // namespace real_replay
