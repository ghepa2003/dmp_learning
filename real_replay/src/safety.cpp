#include "real_replay/safety.hpp"

#include <cmath>
#include <cstdio>

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
  double worst = 0.0;
  int worst_j = 0;
  for (std::size_t i = 0; i < 7; ++i) {
    if (!std::isfinite(q[i]) || !std::isfinite(dq[i])) {
      detail_ = {"non-finite state", static_cast<int>(i), std::isfinite(q[i]) ? dq[i] : q[i], 0.0, 0};
      return StopReason::kNonFiniteState;
    }
    if (q[i] < lower_[i] + cfg_.joint_limit_margin_rad) {
      detail_ = {"joint lower limit", static_cast<int>(i), q[i], lower_[i] + cfg_.joint_limit_margin_rad, 0};
      return StopReason::kJointLimit;
    }
    if (q[i] > upper_[i] - cfg_.joint_limit_margin_rad) {
      detail_ = {"joint upper limit", static_cast<int>(i), q[i], upper_[i] - cfg_.joint_limit_margin_rad, 0};
      return StopReason::kJointLimit;
    }
    const double a = std::fabs(dq[i]);
    if (a > cfg_.max_joint_speed_rad_s) {
      fast = true;
      if (a > worst) {
        worst = a;
        worst_j = static_cast<int>(i);
      }
    }
  }
  if (fast) fast_joint_ = worst_j;
  if (speed_.update(fast)) {
    detail_ = {"joint speed", fast_joint_, std::fabs(dq[static_cast<std::size_t>(fast_joint_)]), cfg_.max_joint_speed_rad_s,
               cfg_.joint_speed_consecutive};
    return StopReason::kJointSpeed;
  }

  if (!track_enabled_) return StopReason::kNone;
  if (!std::isfinite(e_pos_norm) || !std::isfinite(e_rot_norm)) {
    detail_ = {"tracking error non-finite", -1, std::isfinite(e_pos_norm) ? e_rot_norm : e_pos_norm, 0.0, 0};
    return StopReason::kTrackingError;
  }
  const bool tp = track_pos_.update(e_pos_norm > cfg_.max_tracking_error_m);
  const bool tr = track_rot_.update(e_rot_norm > cfg_.max_tracking_error_rad);
  if (tp) {
    detail_ = {"tracking position", -1, e_pos_norm, cfg_.max_tracking_error_m, cfg_.tracking_consecutive};
    return StopReason::kTrackingError;
  }
  if (tr) {
    detail_ = {"tracking rotation", -1, e_rot_norm, cfg_.max_tracking_error_rad, cfg_.tracking_consecutive};
    return StopReason::kTrackingError;
  }
  return StopReason::kNone;
}

StopReason SafetyGuard::checkCycle(double period_s, double last_cb_us, double success_rate,
                                   long cycle) noexcept {
  // !(x >= 0) also catches NaN.
  if (!(period_s >= 0.0) || period_s > cfg_.period_abort_s) {
    detail_ = {"cycle period", -1, period_s, cfg_.period_abort_s, 0};
    return StopReason::kPeriodWatchdog;
  }
  if (cycle >= 1 && cb_.update(last_cb_us > cfg_.callback_budget_us)) {
    detail_ = {"callback duration", -1, last_cb_us, cfg_.callback_budget_us, cfg_.callback_budget_consecutive};
    return StopReason::kCallbackBudget;
  }
  if (cycle >= cfg_.watchdog_grace_cycles && success_.update(success_rate < cfg_.min_success_rate)) {
    detail_ = {"control_command_success_rate", -1, success_rate, cfg_.min_success_rate, cfg_.success_rate_consecutive};
    return StopReason::kSuccessRate;
  }
  return StopReason::kNone;
}

void formatStopReason(char* buf, std::size_t n, StopReason r, const StopDetail& d) noexcept {
  if (n == 0) return;
  const char* name = toString(r);
  switch (r) {
    case StopReason::kJointLimit:
      std::snprintf(buf, n, "%s: %s on joint %d: q = %.2f deg, bound (limit -/+ margin) %.2f deg", name, d.what, d.joint + 1,
                    d.value * 57.29577951308232, d.limit * 57.29577951308232);
      break;
    case StopReason::kJointSpeed:
      std::snprintf(buf, n, "%s: joint %d: |dq| = %.3f rad/s > %.3f rad/s for %d consecutive cycles", name, d.joint + 1, d.value, d.limit,
                    d.consecutive);
      break;
    case StopReason::kTrackingError:
      if (d.limit > 0.0) {
        const bool pos = d.what[9] == 'p';  // "tracking position"
        std::snprintf(buf, n, "%s: %s = %.4g %s > %.4g %s for %d consecutive cycles", name, d.what, pos ? d.value * 1e3 : d.value,
                      pos ? "mm" : "rad", pos ? d.limit * 1e3 : d.limit, pos ? "mm" : "rad", d.consecutive);
      } else {
        std::snprintf(buf, n, "%s: %s (%.4g)", name, d.what, d.value);
      }
      break;
    case StopReason::kPeriodWatchdog:
      std::snprintf(buf, n, "%s: cycle period %.3f ms > %.3f ms", name, d.value * 1e3, d.limit * 1e3);
      break;
    case StopReason::kCallbackBudget:
      std::snprintf(buf, n, "%s: callback %.0f us > %.0f us for %d consecutive cycles", name, d.value, d.limit, d.consecutive);
      break;
    case StopReason::kSuccessRate:
      std::snprintf(buf, n, "%s: control_command_success_rate %.3f < %.3f for %d consecutive cycles", name, d.value, d.limit, d.consecutive);
      break;
    case StopReason::kNonFiniteState:
      if (d.what[0] != '\0') {
        std::snprintf(buf, n, "%s: joint %d value %g", name, d.joint + 1, d.value);
      } else {
        std::snprintf(buf, n, "%s", name);
      }
      break;
    default:
      std::snprintf(buf, n, "%s", name);
      break;
  }
  buf[n - 1] = '\0';
}

}  // namespace real_replay
