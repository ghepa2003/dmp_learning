#pragma once

// Configuration of the standalone real-robot executable. Everything tunable lives in the YAML
// file (config/real_replay.yaml); nothing numeric is hardcoded in the control path.

#include <array>
#include <string>
#include <vector>

namespace real_replay {

/// Nominal libfranka control cycle [s].
constexpr double kNominalCycleS = 1e-3;

using Array3 = std::array<double, 3>;
using Array6 = std::array<double, 6>;
using Array7 = std::array<double, 7>;
using Array9 = std::array<double, 9>;
using Array16 = std::array<double, 16>;

struct RobotConfig {
  std::string ip;
  std::string ee_frame;                  ///< control frame of the model (fer_link8 = flange)
  std::vector<std::string> joint_names;  ///< exactly 7
};

struct PathsConfig {
  std::string urdf;     ///< resolved against the directory of the YAML file if relative
  std::string log_dir;  ///< idem
};

struct RunConfig {
  double max_duration_s;  ///< hard cap on trajectory time; also sizes the preallocated log
};

struct HoldConfig {
  double duration_s;
};

struct TrajectoryConfig {
  int target_update_divisor;  ///< target sampled every N-th cycle (zero-order hold in between)
  double stop_at_fraction;    ///< replay only: freeze the target at this fraction of tau, (0, 1]
};

struct StepConfig {
  Array3 axis;               ///< direction in the robot base frame (normalised by the source)
  double amplitude_m;
  double move_duration_s;    ///< smoothstep out
  double dwell_s;            ///< pause at the displaced pose
  bool return_to_start;
  double return_duration_s;  ///< smoothstep back (used only if return_to_start)
  // --mode step_rot only (same profile/dwell/return as the translation; position held):
  Array3 rot_axis = {0.0, 0.0, 1.0};  ///< fixed axis in the robot BASE frame (normalised by the source)
  double rot_angle_rad = 0.087;       ///< peak rotation angle (~5 deg)
};

struct ReplayConfig {
  std::string weights_path;     ///< ProDMP weights with inline quaternion_dmp section
  std::string expected_sha256;  ///< empty = do not check
  double hold_after_end_s;      ///< target frozen for this long after the end, then controlled stop
};

struct GainsConfig {
  double translational_stiffness;
  double rotational_stiffness;
  double translational_damping;
  double rotational_damping;
  double nullspace_stiffness;
  double joint1_nullspace_stiffness;
  double nullspace_pinv_damping;
  double delta_tau_max;       ///< solver-internal slew limit [Nm/cycle]
  double gain_ramp_s;         ///< duration of the K ramp (0 = no ramp)
  double gain_ramp_initial;   ///< K scale at t = 0, in (0, 1]
};

/// Per-axis Cartesian damping, derived by the controller (see controller_core.hpp):
///   D_i = min( 2*zeta_i*sqrt(K_i*Lambda_act_i), max_damping_per_sample*Lambda_min_i/dt ).
/// gains.translational_damping / gains.rotational_damping are NOT used any more.
struct DampingConfig {
  double zeta_trans;              ///< target damping ratio of the 3 translational axes
  double zeta_rot;                ///< target damping ratio of the 3 rotational axes
  double max_damping_per_sample;  ///< cap_a: a_i = D_i*dt/Lambda_min_i never exceeds this
  double max_q_drift_rad;         ///< abort if q at activation differs from the q used for the design
  std::vector<double> override_d; ///< empty, or 6 values [Dx Dy Dz Drx Dry Drz]: bypass the design
};

struct SafetyConfig {
  Array7 effort_limits_nm;       ///< per-joint effort limits (URDF values)
  double torque_limit_fraction;  ///< absolute clamp = fraction * effort limit
  double max_torque_step_nm;     ///< explicit rate limit per cycle on the SENT torque
  double joint_limit_margin_rad;
  double max_joint_speed_rad_s;
  int joint_speed_consecutive;
  double max_tracking_error_m;
  double max_tracking_error_rad;
  int tracking_consecutive;
  double period_abort_s;         ///< a single cycle period above this aborts
  double callback_budget_us;
  int callback_budget_consecutive;
  double min_success_rate;
  int success_rate_consecutive;
  int watchdog_grace_cycles;     ///< success-rate check ignored for the first N cycles
  double stop_duration_s;        ///< controlled-stop phase length
  double stop_damping_scale;     ///< D multiplier during the controlled stop
};

struct CollisionConfig {
  Array7 lower_torque_acc, upper_torque_acc, lower_torque_nom, upper_torque_nom;
  Array6 lower_force_acc, upper_force_acc, lower_force_nom, upper_force_nom;
  Array7 joint_impedance;
  Array6 cartesian_impedance;
};

struct EndEffectorConfig {
  bool set_ee;
  Array16 ne_t_ee;  ///< column-major 4x4
  bool set_load;
  double load_mass;
  Array3 f_x_cload;
  Array9 load_inertia;
};

struct PreflightConfig {
  bool abort_on_failure;
  double m_ee_max_kg;
  double m_load_max_kg;
  double fk_position_tol_m;
  double fk_rotation_tol_deg;
  bool check_gravity;
  double gravity_vs_measured_tol_nm;  ///< franka::Model gravity vs measured tau_J at rest: FAIL above this
  double gravity_still_max_dq_rad_s;  ///< "at rest" = max|dq| below this; otherwise that check is a WARN
  double gravity_urdf_warn_nm;        ///< franka::Model vs Pinocchio(URDF): informative WARN above this
  double max_damping_per_sample;   ///< WARN when a_i = D_i*dt/Lambda_min_i exceeds this
  double fail_damping_per_sample;  ///< FAIL (robot mode) when a_i exceeds this
  double warn_gain_margin;         ///< WARN when the gain margin g* (one cycle of delay) is below this
  double fail_gain_margin;         ///< FAIL (robot mode) when g* is below this
  double fail_spectral_radius;     ///< FAIL (robot mode) when max|z| >= this
};

struct RuntimeConfig {
  bool mlockall;
  bool sched_fifo;
  std::string expected_libfranka_dir;  ///< the loaded libfranka*.so must live under this directory
};

struct DryRunConfig {
  Array7 q;     ///< fixed activation configuration
  bool paced;   ///< sleep to a 1 kHz cadence between cycles
  bool ignore_tracking_error;  ///< the dry-run state is fixed, so the target moves away: disable that guard
};

struct Config {
  std::string config_path;
  RobotConfig robot;
  PathsConfig paths;
  RunConfig run;
  HoldConfig hold;
  TrajectoryConfig trajectory;
  StepConfig step;
  ReplayConfig replay;
  GainsConfig gains;
  DampingConfig damping;
  SafetyConfig safety;
  CollisionConfig collision;
  EndEffectorConfig end_effector;
  PreflightConfig preflight;
  RuntimeConfig runtime;
  DryRunConfig dry_run;
};

/// Parses and validates the YAML file. Throws std::runtime_error with a descriptive message.
Config loadConfig(const std::string& yaml_path);

}  // namespace real_replay
