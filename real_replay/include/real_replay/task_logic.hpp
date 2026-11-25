#pragma once

// Task layer of robot 1, stage 2 (command -> end of the approach). PURE logic: no libfranka, no sockets, no
// threads, no clock reads. The runtime (task_runtime.hpp) feeds it inputs and executes its actions, so every
// state, transition and fault is unit-testable (test/test_task_logic.cpp).
//
// Angles in rad, times in s, lengths in m. The GraspCommand YAML is in DEGREES and is converted ONCE, in
// loadGraspCommand() (src/task_logic.cpp), the only place where degrees are read.
//
// Phase convention: theta on the branch [-270 deg, +90 deg) (satellite_phase.hpp unwrapPhase); wrapPi is used
// ONLY for d = wrapPi(theta - theta_trig) and for theta_trig itself (as the simulation executor).

#include <Eigen/Geometry>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace real_replay {
namespace task {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// ---- States -----------------------------------------------------------------------------------------------------
/// The full set of stages 2 and 3. CLOSE_FOLLOW, SQUEEZE, RELEASE and FAULT_GRASP are STUBS in stage 2: they exist
/// (and their transitions are declared in transitionAllowed()) so that stage 3 does not change the structure, but
/// the stage-2 machine never enters them.
enum class TaskState : int {
  kInit = 0,           ///< preflight (start_pose_check, config sanity) - done before the control loop
  kWaitCmd = 1,        ///< hold; activation orientation read ONCE; waiting for the command and the activation
  kValidate = 2,       ///< checks, operator confirmation, START to the satellite
  kWaitTrigger = 3,    ///< hold; phase crossing of theta_trig
  kApproach = 4,       ///< ProDMP rollout over tau
  kHoldSafe = 5,       ///< stage 2 end: frozen target, STOP to the satellite after T_hold_stage2_s, exit
  kCloseFollow = 6,    ///< stage 3 stub
  kSqueeze = 7,        ///< stage 3 stub
  kRelease = 8,        ///< stage 3 stub
  kFaultNoMotion = 9,  ///< fault before the approach: hold, STOP, process alive
  kFaultMotion = 10,   ///< fault during/after the approach: target frozen (v = 0), STOP, process alive
  kFaultGrasp = 11,    ///< stage 3 stub
  kDone = 12,          ///< stage 2 completed (HOLD_SAFE over), controlled stop of the loop requested
};

const char* toString(TaskState s);
bool isStage3Stub(TaskState s);
/// Transition table of stages 2 + 3 (stage 3 edges declared, unused). Any state -> FAULT_* is allowed.
bool transitionAllowed(TaskState from, TaskState to);

/// Command to the target source running in the 1 kHz loop.
enum class SourceCmd : int { kHold = 0, kApproach = 1, kFreeze = 2 };

// ---- GraspCommand -----------------------------------------------------------------------------------------------
struct GraspCommand {
  std::string path;
  std::string format;     ///< "grasp_command_v2"
  std::string status;     ///< SELECTED | NO_FEASIBLE_CANDIDATE | POSTHOC_FAILED | UNSET
  std::string frame_id;   ///< must be "world" (= base of robot 1)
  double stamp_unix_s = kNaN;
  double theta_star = kNaN;      ///< [rad], branch [-270, 90)
  double contact_time_s = kNaN;
  double omega = kNaN;           ///< [rad/s]
  std::string weights_sha256;
  Eigen::Vector3d start_p = Eigen::Vector3d::Constant(kNaN);
  Eigen::Quaterniond start_q = Eigen::Quaterniond(kNaN, kNaN, kNaN, kNaN);
  Eigen::Vector3d goal_p = Eigen::Vector3d::Constant(kNaN);
  Eigen::Quaterniond goal_q = Eigen::Quaterniond(kNaN, kNaN, kNaN, kNaN);
  int k = -1;
  bool wrist_flip = false;
};

/// Reads the YAML (degrees) and converts to rad: the ONLY place where degrees are read. Throws std::runtime_error
/// on a missing key or a non-numeric value (finiteness is checked by validateCommand, not here).
GraspCommand loadGraspCommand(const std::string& path);

// ---- Parameters (hw_config.yaml: scene, robot2.control, timing, plan, task, udp) --------------------------------
struct TaskParams {
  double omega_cfg = kNaN;             ///< scene.omega_deg_s [rad/s]
  double omega_cmd_tol = 0.1 * 3.14159265358979323846 / 180;  ///< plan.omega_tol_deg_s
  double omega_domain[2] = {kNaN, kNaN};  ///< plan.omega_domain_deg_s [rad/s] (warning only)
  double tau_tol_s = 1e-6;
  double psi_lo = kNaN, psi_hi = kNaN;  ///< scene.psi_window_deg
  double window_margin = kNaN;          ///< robot2.window_margin_deg
  double sat_ramp_s = kNaN;             ///< robot2.control.ramp_s
  double sat_omega_window_s = kNaN;     ///< robot2.control.omega_window_s
  double contact_delay_s = 0.0;
  double T_arm_min_s = kNaN, margin_arm_s = kNaN;
  double T_close_max_s = kNaN, T_follow_s = kNaN, T_ramp_s = kNaN;
  double trigger_timeout_margin_s = kNaN;
  double T_age_s = 0.1;                 ///< udp.T_age_s
  // task section
  double T_hold_stage2_s = 3.0;
  double cmd_max_age_s = 600.0;
  double start_pos_tol_m = 0.005, start_rot_tol_rad = 0.02;
  double sat_timeout_s = 2.0;
  double sat_start_timeout_s = 2.0;
  double trigger_max_age_s = 0.2;
  double omega_meas_tol = 0.1 * 3.14159265358979323846 / 180;
  double trigger_max_d = 3.14159265358979323846 / 2;
  double fault_linger_s = 0.0;          ///< 0 = stay alive in the fault state until Ctrl-C
};

struct HwTaskConfig {
  TaskParams p;
  std::string udp_host = "127.0.0.1";
  int udp_state_port = 5601;   ///< udp.publish_port (state from satellite_node)
  int udp_command_port = 5602;
  double hb_send_hz = 20.0;
  double jitter_period_s = 0.003;
  int jitter_consecutive = 3;
};

/// Reads the sections needed by the task layer. Throws std::runtime_error with the offending key.
HwTaskConfig loadHwTaskConfig(const std::string& path);

// ---- Checks / feasibility ---------------------------------------------------------------------------------------
struct Check {
  std::string name;
  bool ok = false;
  bool blocking = true;
  std::string detail;
};

struct Feasibility {
  double s = 0.0;               ///< sgn(omega)
  double T_c = kNaN;            ///< tau + contact_delay_s
  double theta_trig = kNaN;     ///< wrapPi(theta* - omega*T_c), (-pi, pi]
  double theta_trig_branch = kNaN;  ///< same on the branch [-270, 90): used for comparisons
  double theta0 = kNaN;
  double angle_before_valid = kNaN;  ///< |omega|*(ramp_s/2 + omega_window_s)
  double need_a = kNaN;              ///< condition A threshold
  double theta0_lo = kNaN, theta0_hi = kNaN;  ///< admitted theta0 interval (lo > hi = empty)
  double theta_end = kNaN;
  double t_trigger_s = kNaN;         ///< (theta_trig - theta0)/omega
  double trigger_timeout_s = kNaN;   ///< t_trigger + ramp_s/2 + trigger_timeout_margin_s
  std::vector<Check> checks;         ///< A..E
  bool ok = false;
};

/// Conditions of hw.py plan (same names, same formulas):
///   A  s*(theta_trig - theta0) >= |omega|*(ramp_s/2 + omega_window_s + margin_arm_s)   (omega valid in time)
///   B  s*(theta_trig - theta0) >= |omega|*T_arm_min_s
///   C  theta0 inside the phase window with margin
///   D  theta_end = theta* + omega*(T_close_max + T_follow + T_ramp) inside the window with margin
///   E  |omega| in plan.omega_domain_deg_s (WARNING only)
Feasibility feasibility(const GraspCommand& cmd, const TaskParams& p, double tau, double theta0);

struct ValidationContext {
  double now_unix_s = kNaN;
  double tau = kNaN;
  std::string weights_sha256;
  Eigen::Vector3d tcp_p = Eigen::Vector3d::Constant(kNaN);  ///< measured TCP at activation
  Eigen::Quaterniond tcp_q = Eigen::Quaterniond(kNaN, kNaN, kNaN, kNaN);
  bool path_checked = false;  ///< joint-path check of the goal trajectory was run
  bool path_ok = false;
  std::string path_detail;
  double stop_at_fraction = 1.0;  ///< trajectory.stop_at_fraction of the real_replay config: must be 1
};

/// Everything VALIDATE checks except the feasibility (which needs theta0 from the satellite).
std::vector<Check> validateCommand(const GraspCommand& cmd, const TaskParams& p, const ValidationContext& ctx);

// ---- Satellite samples ------------------------------------------------------------------------------------------
struct SatSample {
  std::uint32_t seq = 0;
  std::uint32_t state = 0;
  std::uint32_t faults = 0;
  std::uint32_t flags = 0;
  std::uint64_t t_mono_ns = 0;
  double theta = kNaN;
  double omega_meas = kNaN;
  bool omega_valid = false;
  double rx_s = kNaN;  ///< receive time, CLOCK_MONOTONIC [s]
};

enum class SampleVerdict : int { kOk = 0, kSeq = 1, kClockBack = 2, kAge = 3, kNonFinite = 4 };
const char* toString(SampleVerdict v);

/// Input filter (stateful: last accepted seq and stamp). age = now - t_mono must be in [0, T_age); seq strictly
/// increasing; t_mono non-decreasing; theta finite and on the branch; omega finite when omega_valid.
class SatFilter {
 public:
  explicit SatFilter(double t_age_s) : t_age_s_(t_age_s) {}
  SampleVerdict accept(const SatSample& s, double now_mono_s);
  std::uint64_t count(SampleVerdict v) const { return n_[static_cast<int>(v)]; }

 private:
  double t_age_s_;
  bool have_ = false;
  std::uint32_t last_seq_ = 0;
  std::uint64_t last_t_ = 0;
  std::uint64_t n_[5] = {0, 0, 0, 0, 0};
};

// ---- Trigger ----------------------------------------------------------------------------------------------------
/// Crossing test on d_k = wrapPi(theta_k - theta_trig): fires when sgn(omega)*d_{k-1} < 0 and sgn(omega)*d_k >= 0
/// and |d_k| < max_d. A phase jump (|d| >= max_d, e.g. through the opposite side) never fires.
class PhaseTrigger {
 public:
  PhaseTrigger(double theta_trig, double omega, double max_d);
  struct Result {
    bool fired = false;
    double d = kNaN;
  };
  Result update(double theta);
  void reset() { have_prev_ = false; }
  double thetaTrig() const { return theta_trig_; }

 private:
  double theta_trig_, s_, max_d_;
  bool have_prev_ = false;
  double d_prev_ = 0.0;
};

// ---- Machine ----------------------------------------------------------------------------------------------------
struct MachineInputs {
  double now_s = 0.0;  ///< CLOCK_MONOTONIC [s]
  bool activation_ready = false;
  Eigen::Vector3d tcp_p = Eigen::Vector3d::Zero();  ///< latest measured TCP
  Eigen::Quaterniond tcp_q = Eigen::Quaterniond::Identity();
  bool sat_have = false;      ///< at least one accepted sample
  SatSample sat;              ///< latest ACCEPTED sample
  std::uint64_t rejected_seq = 0, rejected_clock = 0, rejected_nonfinite = 0, rejected_age = 0;  ///< cumulative
  bool approach_started = false;
  bool approach_done = false;
  bool loop_finished = false;
  std::string loop_reason;
  bool jitter_fault = false;
  bool abort = false;  ///< Ctrl-C / SIGTERM
};

struct MachineActions {
  bool send_start = false;  ///< one shot
  bool send_stop = false;   ///< one shot
  bool hb_enable = false;
  SourceCmd source = SourceCmd::kHold;
  bool end_loop = false;    ///< request the controlled stop of the 1 kHz loop
};

struct Event {
  double t = 0.0;
  std::string name;
  std::string detail;
};

/// Phase / timing figures of the run (the scientific result).
struct RunFigures {
  double t_start_sent = kNaN;
  double t_omega_valid = kNaN;
  double t_trigger = kNaN, theta_at_trigger = kNaN, d_at_trigger = kNaN, age_at_trigger = kNaN;
  double t_approach_start = kNaN, t_approach_end = kNaN;
  double theta_at_approach_end = kNaN, age_at_approach_end = kNaN;
  double theta_star = kNaN, theta_trig = kNaN, omega_cmd = kNaN, tau = kNaN;
};

class TaskMachine {
 public:
  TaskMachine(const TaskParams& p, double tau, std::uint32_t sat_fault_mask_ignore);

  /// One GraspCommand per execution: the second one is ignored (event "duplicate_command").
  bool submitCommand(const GraspCommand& cmd);
  const GraspCommand& command() const { return cmd_; }
  bool hasCommand() const { return have_cmd_; }

  /// VALIDATE, step 1 done by the caller: static checks + feasibility (theta0 from the satellite).
  bool needsValidation() const { return state_ == TaskState::kValidate && !validated_ && !validation_failed_; }
  void setValidation(const std::vector<Check>& checks, const Feasibility& f);
  /// VALIDATE, step 2: operator confirmation (or --yes).
  bool needsConfirmation() const { return state_ == TaskState::kValidate && validated_ && !confirmed_; }
  void setConfirmed(bool yes);

  /// Explicit START request (normally the machine sends START itself once confirmed). A second request is
  /// ignored and logged ("duplicate_start"): START is sent at most once per execution.
  void requestStartAgain(double now_s);

  /// Advances the machine. Never throws on regular input.
  MachineActions update(const MachineInputs& in);

  TaskState state() const { return state_; }
  const std::string& faultReason() const { return fault_reason_; }
  const std::vector<Event>& events() const { return events_; }
  const Feasibility& feasibilityResult() const { return feas_; }
  const RunFigures& figures() const { return fig_; }
  bool startSent() const { return start_sent_; }
  bool stopSent() const { return stop_sent_; }
  int exitCode() const;  ///< 0 done, 5 FAULT_NOMOTION, 6 FAULT_MOTION, 4 still running (should not happen)

 private:
  void enter(TaskState s, double t, const std::string& why = "");
  void fault(const MachineInputs& in, const std::string& why);
  void event(double t, const std::string& name, const std::string& detail = "");
  bool satFault(const MachineInputs& in, std::string& why) const;

  TaskParams p_;
  double tau_;
  std::uint32_t ignore_mask_;
  TaskState state_ = TaskState::kWaitCmd;
  GraspCommand cmd_;
  bool have_cmd_ = false;
  bool validated_ = false, validation_failed_ = false, confirmed_ = false;
  bool start_sent_ = false, stop_sent_ = false, stop_pending_ = false, start_pending_ = false;
  bool armed_ = false, sat_running_seen_ = false;
  bool end_loop_ = false;
  double t_state_ = 0.0, t_fault_ = kNaN;
  Feasibility feas_;
  PhaseTrigger trigger_{0.0, 1.0, 1.0};
  std::uint32_t last_seq_fed_ = 0;
  std::uint64_t rej_seq0_ = 0, rej_clock0_ = 0, rej_nan0_ = 0;
  bool rej_baseline_ = false;
  std::string fault_reason_;
  std::vector<Event> events_;
  RunFigures fig_;
};

}  // namespace task
}  // namespace real_replay
