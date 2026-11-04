#pragma once

// Safety layer for the 1 kHz torque loop. No dynamic allocation, no I/O, no exceptions.

#include <array>

#include "real_replay/config.hpp"

namespace real_replay {

enum class StopReason : int {
  kNone = 0,
  kCompleted,
  kMaxDuration,
  kLogFull,
  kUserSignal,
  kNonFiniteState,
  kActivationDrift,
  kNonFiniteTorque,
  kJointLimit,
  kJointSpeed,
  kTrackingError,
  kPeriodWatchdog,
  kCallbackBudget,
  kSuccessRate,
  kException
};

const char* toString(StopReason r) noexcept;

/// Counts consecutive violations; fires once the count reaches n.
class Debounce {
 public:
  explicit Debounce(int n = 1) noexcept : n_(n < 1 ? 1 : n) {}
  bool update(bool violated) noexcept {
    count_ = violated ? count_ + 1 : 0;
    return count_ >= n_;
  }
  void reset() noexcept { count_ = 0; }

 private:
  int n_;
  int count_ = 0;
};

class SafetyGuard {
 public:
  SafetyGuard(const SafetyConfig& cfg, const Array7& joint_lower, const Array7& joint_upper);

  /// Forgets the previous sent torque (set to zero) and clears all debouncers.
  void reset() noexcept;

  /// Rate-limits `desired` against the previous SENT torque, then clamps to +-limit. The result is
  /// stored as the new previous torque. Rate limit first, clamp second: clamping a value to a box
  /// that contains the previous torque can only shrink the step, so both bounds hold.
  void limitTorque(const Array7& desired, Array7& out) noexcept;
  bool lastRateLimited() const noexcept { return rate_limited_; }
  bool lastClamped() const noexcept { return clamped_; }

  /// Joint limits (with margin), joint speed, tracking error. Call once per cycle.
  StopReason checkState(const Array7& q, const Array7& dq, double e_pos_norm,
                        double e_rot_norm) noexcept;

  /// Period / callback-budget / success-rate watchdogs. `last_cb_us` is the duration of the
  /// previous callback. Call once per cycle.
  StopReason checkCycle(double period_s, double last_cb_us, double success_rate,
                        long cycle) noexcept;

  /// Dry-run only: the state is fixed there, so the target legitimately moves away from it.
  void setTrackingCheckEnabled(bool enabled) noexcept { track_enabled_ = enabled; }

  const Array7& lastTorque() const noexcept { return prev_; }
  const Array7& torqueLimits() const noexcept { return tau_limit_; }

 private:
  SafetyConfig cfg_;
  Array7 lower_;
  Array7 upper_;
  Array7 tau_limit_{};
  Array7 prev_{};
  bool rate_limited_ = false;
  bool clamped_ = false;
  bool track_enabled_ = true;
  Debounce speed_;
  Debounce track_pos_;
  Debounce track_rot_;
  Debounce cb_;
  Debounce success_;
};

}  // namespace real_replay
