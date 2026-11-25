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

/// One leg of step.sequence: smoothstep from the previous position to f * amplitude_m (f * rot_angle_rad
/// for step_rot) in move_s, then hold for hold_s.
struct StepWaypoint {
  double f;
  double move_s;
  double hold_s;
};

struct StepConfig {
  Array3 axis;               ///< direction in the robot base frame (normalised by the source)
  double amplitude_m;
  double move_duration_s;    ///< smoothstep out            (NOT allowed together with `sequence`)
  double dwell_s;            ///< pause at the displaced pose (NOT allowed together with `sequence`)
  bool return_to_start;      ///< (NOT allowed together with `sequence`)
  double return_duration_s;  ///< smoothstep back (used only if return_to_start; NOT allowed with `sequence`)
  // Optional multi-leg profile. Non-empty: it replaces move/dwell/return; positions f*amplitude_m from the
  // activation pose, preceded by a hold of initial_hold_s at the start pose.
  std::vector<StepWaypoint> sequence;
  double initial_hold_s = 0.0;  ///< only with `sequence`
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
  double rotational_stiffness;  ///< scalar Kr; if rotational_stiffness_per_axis it is only min(xyz), not used by the controller
  bool rotational_stiffness_per_axis = false;  ///< true if gains.rotational_stiffness_xyz was given
  Array3 rotational_stiffness_xyz = {0.0, 0.0, 0.0};  ///< [Krx Kry Krz], base frame (valid only if per_axis)
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
  std::vector<double> rotational_d_max_xyz;  ///< empty (off), or 3 values [Dx Dy Dz] (base frame): after the design D_i = min(D_i, d_max_i) on rx, ry, rz (also D_stop). Ignored if override_d is set.
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
  /// On entering the controlled stop the nullspace target becomes the MEASURED joint configuration at that instant (F6 of the
  /// 2026-11-25 analysis: without it the posture spring keeps pushing the self-motion towards the activation q_ref during the stop).
  /// Default false = previous behaviour.
  bool stop_nullspace_to_current = false;
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
  bool fk_compare_o_t_ee = false;  ///< true: model FK of ee_frame vs O_T_EE (ee_frame = TCP); false: vs flange
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

/// Joint friction feedforward (default OFF; the section is optional in the YAML):
///   tau_ff,j = alpha * gain_alpha(t) * [ tau_c,j * tanh(vhat_j / eps_v) + b_j * vhat_j ],
///   vhat = J^+ * xdot_ref   (damped pseudo-inverse of the solver's Jacobian, reference twist only).
struct FrictionFfConfig {
  bool enabled = false;
  double alpha = 0.0;        ///< overall scale, >= 0 (additionally ramped with the K gain_alpha)
  double eps_v_rad_s = 0.005;
  double dls_lambda = 0.05;  ///< J^+ = J^T (J J^T + lambda^2 I)^-1
  Array7 tau_c = {0.615, 0.361, 0.361, 0.562, 0.352, 0.341, 0.154};  ///< Coulomb [Nm]
  Array7 b = {2.83, 1.89, 2.71, 1.6, 0.38, 0.33, 0.18};              ///< viscous [Nm s/rad]
};

/// Optional (key `start_pose_check`): in robot mode the pose O_T_EE read from the robot must match this one.
struct StartPoseCheckConfig {
  bool present = false;                         ///< false: no check
  Array3 position_m = {0.0, 0.0, 0.0};          ///< expected O_T_EE position, base frame
  std::array<double, 4> quaternion_wxyz = {1.0, 0.0, 0.0, 0.0};  ///< expected orientation, REQUIRED (normalised by the check)
  double tol_position_m = 0.005;
  double tol_rotation_rad = 0.02;
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
  FrictionFfConfig friction_ff;
  StartPoseCheckConfig start_pose_check;
};

/// Parses and validates the YAML file. Throws std::runtime_error with a descriptive message.
Config loadConfig(const std::string& yaml_path);

/// dry_run.ignore_tracking_error disables a safety guard: allowed only in --dry-run. Returns false and
/// fills *message when the config requests it and the program is not in dry-run mode.
bool trackingIgnoreAllowed(const Config& cfg, bool dry_run, std::string* message);

}  // namespace real_replay
