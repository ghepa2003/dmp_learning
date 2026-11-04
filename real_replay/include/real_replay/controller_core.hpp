#pragma once

// ReplayLoop: everything that happens inside one 1 kHz torque cycle, independent of libfranka
// types (the caller converts franka::RobotState <-> LoopInput/LoopOutput). Uses the existing
// RobotModel / CartesianImpedanceSolver unchanged. step() performs no I/O and no locking.
//
// Known allocations inside step(): RobotModel::update() (4 small Eigen heap buffers). They are
// in existing code that this project does not modify; the dry-run measures their cost.

#include <csignal>
#include <memory>
#include <string>

#include "franka_cartesian_control/core/cartesian_error.hpp"
#include "franka_cartesian_control/core/impedance_solver.hpp"
#include "franka_cartesian_control/core/robot_model.hpp"
#include "real_replay/config.hpp"
#include "real_replay/logger.hpp"
#include "real_replay/safety.hpp"
#include "real_replay/target_source.hpp"

namespace real_replay {

/// The existing, unmodified thesis core.
namespace core = franka_cartesian_control::core;

struct LoopInput {
  Array7 q{};
  Array7 dq{};
  Array7 tau_J{};            ///< measured joint torques (log only)
  double period_s = 0.0;     ///< franka::Duration of this cycle (0 on the very first call)
  double success_rate = 1.0; ///< control_command_success_rate
};

struct LoopOutput {
  Array7 tau{};
  bool finished = false;  ///< set motion_finished on the libfranka side
};

using Vector6 = Eigen::Matrix<double, 6, 1>;

/// Per-axis Cartesian damping design, axes [x y z rx ry rz] in the robot BASE frame (the frame of
/// the Jacobian used by the solver, LOCAL_WORLD_ALIGNED, and of the diagonal of Lambda).
///   D_i = min( 2*zeta_i*sqrt(K_i*Lambda_act_i), cap_a*Lambda_min_i/dt )
/// Lambda_act: diagonal of the reflected Cartesian inertia at the activation configuration;
/// Lambda_min_i: its minimum along the joint path (JointPathSimulator) or, without a path, Lambda_act.
/// a_i = D_i*dt/Lambda_min_i is the damping applied per sample (>= 2 cannot be stable).
struct DampingDesign {
  bool path_known = false;   ///< lambda_min/max come from a joint path
  bool overridden = false;   ///< damping.override_d bypassed the design
  Array6 k{};                ///< nominal stiffness per axis
  Array6 lambda_act{};
  Array6 lambda_min{};       ///< effective minimum used for the cap (<= lambda_act)
  Array6 lambda_max{};
  Array6 d{};                ///< D_i, nominal phase
  Array6 d_stop{};           ///< D_i during the controlled stop (scale applied, cap re-applied)
  Array6 zeta{};             ///< D_i / (2 sqrt(K_i Lambda_act_i))
  Array6 a{};                ///< D_i * dt / Lambda_min_i
  Array6 zeta_lmax{};        ///< D_i / (2 sqrt(K_i Lambda_max_i)): damping ratio with D fixed, at the largest inertia
  Array6 b{};                ///< K_i * dt^2 / Lambda_min_i
  Array6 rho{};              ///< delayedSpectralRadius(a_i, b_i), one cycle of delay
  Array6 g1{};               ///< gainMargin(a_i, b_i, 1): largest g with max|z|<1 for (g a, g b), one cycle of delay
  Array6 g2{};               ///< same with two cycles of delay (informative)
  std::array<bool, 6> capped{};
};

/// Spectral radius max|z| of the screening model of one axis: rigid double integrator, one control
/// cycle of delay between state and applied torque, no rotor inertia, no filters. With
/// a = D*dt/Lambda and b = K*dt^2/Lambda the characteristic polynomial is
///   z^3 - 2 z^2 + (1+a) z + (b - a) = 0
/// (state x, v, applied-torque memory; x+ = x + dt v, v+ = v + dt/Lambda u_applied,
/// u_applied+ = -K x - D v). The roots are the eigenvalues of the companion matrix. < 1 is stable.
/// This is a SCREENING criterion, not a proof of stability of the real loop.
double delayedSpectralRadius(double a, double b);

/// Generalisation to `delay` cycles between state and applied torque (delay >= 1):
///   z^delay (z-1)^2 + a (z-1) + b = 0      (delay = 1 is the polynomial above)
/// max|z| of the roots (eigenvalues of the companion matrix).
double delayedSpectralRadius(double a, double b, int delay);

/// Gain margin g*: the largest g such that max|z| < 1 for (a,b) -> (g a, g b), found numerically
/// (geometric search for the first crossing from g = 1, then bisection). 0 if max|z| >= 1 already at
/// g = 1; +infinity if no crossing below 1e6.
double gainMargin(double a, double b, int delay);

/// Closed form for delay = 1, valid for a > 2b: g* = (a - 2b) / (a - b)^2. At g* a complex pair
/// sits on the unit circle and the third root is real, equal to g*(a-b). NaN if a <= 2b.
double gainMarginClosedForm1(double a, double b);

/// Pure function (no model needed). Throws std::invalid_argument if any Lambda is not finite and > 0.
DampingDesign designDamping(const GainsConfig& gains, const DampingConfig& damping, double stop_scale,
                            double dt, const Array6& lambda_act, const Array6& lambda_min,
                            const Array6& lambda_max, bool path_known);

/// tau = J^T * ( -D .* (J * dq) ): damping of the Cartesian twist with a different D per axis.
/// Fixed-size Eigen only: no heap allocation.
core::RobotModel::JointVector axisDampingTorque(const core::RobotModel::Jacobian6x7& J,
                                                const core::RobotModel::JointVector& dq,
                                                const Vector6& D) noexcept;

enum class LoopPhase : int { kWaitFirst = 0, kRunning = 1, kStopping = 2, kFinished = 3 };

class ReplayLoop {
 public:
  ReplayLoop(const Config& cfg, const std::string& urdf_xml, std::unique_ptr<ITargetSource> source);

  /// Optional flag set from a signal handler; polled once per cycle.
  void setExternalStopFlag(const volatile std::sig_atomic_t* flag) noexcept { ext_stop_ = flag; }

  /// One control cycle. Never throws on regular operation.
  void step(const LoopInput& in, LoopOutput& out);

  /// Dry-run only: see SafetyGuard::setTrackingCheckEnabled.
  void setTrackingGuardEnabled(bool enabled) noexcept { guard_.setTrackingCheckEnabled(enabled); }

  /// Nominal length of the target source [s] (without the controlled stop).
  double sourceNominalDurationS() const { return source_->nominalDurationS(); }

  /// Computes the per-axis damping design at configuration `q`. Call it BEFORE the control loop
  /// starts (it uses non-real-time code: reflectedCartesianInertia()). `lambda_min`/`lambda_max`
  /// are the per-axis extrema of diag(Lambda) along the joint path, or nullptr when no path is
  /// known (then Lambda_act is used for both). If it was not called, the first control cycle
  /// calls it once as a fallback.
  void prepareDamping(const Array7& q, const Array6* lambda_min, const Array6* lambda_max);
  const DampingDesign& dampingDesign() const noexcept { return design_; }
  bool dampingPrepared() const noexcept { return damping_ready_; }

  /// Records the duration of the callback that just ran (called by the wrapper after step()).
  void setLastCallbackUs(double us) noexcept;

  /// Marks the run as terminated by an exception raised around step().
  void markException(const char* what) noexcept;

  core::RobotModel& model() noexcept { return model_; }
  const Logger& logger() const noexcept { return logger_; }
  const SafetyGuard& guard() const noexcept { return guard_; }
  LoopPhase phase() const noexcept { return phase_; }
  StopReason stopReason() const noexcept { return stop_reason_; }
  long cycles() const noexcept { return cycle_; }
  long rateLimitedCycles() const noexcept { return n_rate_limited_; }
  long clampedCycles() const noexcept { return n_clamped_; }
  bool hasFirstTarget() const noexcept { return have_first_target_; }
  const Pose& activationPose() const noexcept { return activation_; }
  const Pose& firstTarget() const noexcept { return first_target_; }
  const std::string& exceptionMessage() const noexcept { return exception_msg_; }

 private:
  using JointVec = core::RobotModel::JointVector;

  double gainAlpha(double t) const noexcept;
  void enterStopping(StopReason r, const Eigen::Vector3d& p, const Eigen::Quaterniond& q) noexcept;

  Config cfg_;
  core::RobotModel model_;
  core::CartesianImpedanceSolver solver_;
  core::CartesianImpedanceSolver::Params base_params_;
  SafetyGuard guard_;
  std::unique_ptr<ITargetSource> source_;
  Logger logger_;

  const volatile std::sig_atomic_t* ext_stop_ = nullptr;

  LoopPhase phase_ = LoopPhase::kWaitFirst;
  StopReason stop_reason_ = StopReason::kNone;
  std::string exception_msg_;

  long cycle_ = 0;
  double t_ = 0.0;
  double stop_elapsed_ = 0.0;
  double pending_dt_ = 0.0;
  int since_update_ = 0;
  double last_cb_us_ = 0.0;
  LogRecord* last_rec_ = nullptr;

  DampingDesign design_;
  Vector6 d_nom_ = Vector6::Zero();
  Vector6 d_stop_ = Vector6::Zero();
  bool damping_ready_ = false;
  Array7 q_design_{};
  JointVec tau_solver_prev_ = JointVec::Zero();  ///< last solver output WITHOUT the damping term

  Pose activation_;
  Pose target_;
  Pose first_target_;
  Pose stop_target_;
  bool have_first_target_ = false;

  long n_rate_limited_ = 0;
  long n_clamped_ = 0;
};

}  // namespace real_replay
