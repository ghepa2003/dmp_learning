#include "real_replay/safety.hpp"

#include <cmath>

namespace real_replay {

const char* toString(StopReason r) noexcept {
  switch (r) {
    case StopReason::kNone: return "none";
    case StopReason::kCompleted: return "completed";
    case StopReason::kMaxDuration: return "max_duration";
    case StopReason::kLogFull: return "log_full";
    case StopReason::kUserSignal: return "user_signal";
    case StopReason::kNonFiniteState: return "non_finite_state";
    case StopReason::kNonFiniteTorque: return "non_finite_torque";
    case StopReason::kActivationDrift: return "activation_drift";
    case StopReason::kJointLimit: return "joint_limit";
    case StopReason::kJointSpeed: return "joint_speed";
    case StopReason::kTrackingError: return "tracking_error";
    case StopReason::kPeriodWatchdog: return "period_watchdog";
    case StopReason::kCallbackBudget: return "callback_budget";
    case StopReason::kSuccessRate: return "success_rate";
    case StopReason::kException: return "exception";
  }
  return "unknown";
}

SafetyGuard::SafetyGuard(const SafetyConfig& cfg, const Array7& joint_lower,
                         const Array7& joint_upper)
    : cfg_(cfg),
      lower_(joint_lower),
      upper_(joint_upper),
      speed_(cfg.joint_speed_consecutive),
      track_pos_(cfg.tracking_consecutive),
      track_rot_(cfg.tracking_consecutive),
      cb_(cfg.callback_budget_consecutive),
      success_(cfg.success_rate_consecutive) {
  for (std::size_t i = 0; i < 7; ++i) tau_limit_[i] = cfg.torque_limit_fraction * cfg.effort_limits_nm[i];
  prev_.fill(0.0);
}

void SafetyGuard::reset() noexcept {
  prev_.fill(0.0);
  rate_limited_ = false;
  clamped_ = false;
  speed_.reset();
  track_pos_.reset();
  track_rot_.reset();
  cb_.reset();
  success_.reset();
}

void SafetyGuard::limitTorque(const Array7& desired, Array7& out) noexcept {
  rate_limited_ = false;
  clamped_ = false;
  const double step = cfg_.max_torque_step_nm;
  for (std::size_t i = 0; i < 7; ++i) {
    double v = desired[i];
    const double hi = prev_[i] + step;
    const double lo = prev_[i] - step;
    if (v > hi) {
      v = hi;
      rate_limited_ = true;
    } else if (v < lo) {
      v = lo;
      rate_limited_ = true;
    }
    if (v > tau_limit_[i]) {
      v = tau_limit_[i];
      clamped_ = true;
    } else if (v < -tau_limit_[i]) {
      v = -tau_limit_[i];
      clamped_ = true;
    }
    out[i] = v;
  }
  prev_ = out;
}

StopReason SafetyGuard::checkState(const Array7& q, const Array7& dq, double e_pos_norm,
                                   double e_rot_norm) noexcept {
  bool fast = false;
  for (std::size_t i = 0; i < 7; ++i) {
    if (!std::isfinite(q[i]) || !std::isfinite(dq[i])) return StopReason::kNonFiniteState;
    if (q[i] < lower_[i] + cfg_.joint_limit_margin_rad ||
        q[i] > upper_[i] - cfg_.joint_limit_margin_rad) {
      return StopReason::kJointLimit;
    }
    if (std::fabs(dq[i]) > cfg_.max_joint_speed_rad_s) fast = true;
  }
  if (speed_.update(fast)) return StopReason::kJointSpeed;

  if (!track_enabled_) return StopReason::kNone;
  if (!std::isfinite(e_pos_norm) || !std::isfinite(e_rot_norm)) return StopReason::kTrackingError;
  const bool tp = track_pos_.update(e_pos_norm > cfg_.max_tracking_error_m);
  const bool tr = track_rot_.update(e_rot_norm > cfg_.max_tracking_error_rad);
  if (tp || tr) return StopReason::kTrackingError;
  return StopReason::kNone;
}

StopReason SafetyGuard::checkCycle(double period_s, double last_cb_us, double success_rate,
                                   long cycle) noexcept {
  // !(x >= 0) also catches NaN.
  if (!(period_s >= 0.0) || period_s > cfg_.period_abort_s) return StopReason::kPeriodWatchdog;
  if (cycle >= 1 && cb_.update(last_cb_us > cfg_.callback_budget_us)) {
    return StopReason::kCallbackBudget;
  }
  if (cycle >= cfg_.watchdog_grace_cycles && success_.update(success_rate < cfg_.min_success_rate)) {
    return StopReason::kSuccessRate;
  }
  return StopReason::kNone;
}

}  // namespace real_replay
