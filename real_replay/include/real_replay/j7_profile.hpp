#pragma once

// Pure (libfranka-free, allocation-free in the evaluation functions) reference generator for the
// "satellite" J7 rotation: constant-speed rotation of ONE joint with smooth start and stop ramps.
//
// Velocity profile of the displacement d(t) >= 0 along the rotation direction (q7_ref = q7_0 + sign * d):
//   [0, T1]            v * s(t/T1)                       s(x) = 3x^2 - 2x^3 (smoothstep on the VELOCITY)
//   [T1, T1+Tc]        v
//   [T1+Tc, T1+Tc+T2]  v * (1 - s((t-T1-Tc)/T2))
//   after              0 (final hold, hold_s long)
// s'(0) = s'(1) = 0, so the velocity is C1 (acceleration continuous, zero at both ends of each ramp).
// Integral of s: x^3 - x^4/2, so each ramp covers v*T/2 and the exact total is
//   travel = v*(T1 + T2)/2 + v*Tc   =>   Tc = travel/v - (T1 + T2)/2   (must be >= 0).
//
// An early stop (Ctrl-C, safety) restarts from the CURRENT displacement d0 and velocity v0 with a smoothstep
// ramp to zero of length T2 (stopFrom()). Position and velocity are continuous at the switch. The stop ramp
// starts with zero acceleration (s'(0) = 0): acceleration is therefore continuous when the switch happens in
// the constant-speed phase, and jumps (to zero) only when it happens inside the start ramp or the nominal
// stop ramp. Distance covered by an early stop is v0*T2/2 <= v*T2/2, so an early stop never ends beyond the
// planned travel.

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace real_replay {

struct J7ProfileParams {
  double speed_rad_s = 0.0;   ///< cruise speed v > 0 [rad/s]
  double travel_rad = 0.0;    ///< total rotation > 0 [rad]
  double ramp_s = 2.0;        ///< T1, start ramp > 0 [s]
  double stop_ramp_s = 2.0;   ///< T2, stop ramp > 0 [s]
  double hold_s = 1.0;        ///< final hold >= 0 [s]
};

struct J7Ref {
  double d = 0.0;   ///< displacement along the rotation direction [rad], >= 0
  double v = 0.0;   ///< d/dt [rad/s], >= 0
};

namespace j7 {

inline double smoothstep(double x) { return x * x * (3.0 - 2.0 * x); }
inline double smoothstepDot(double x) { return 6.0 * x * (1.0 - x); }
/// Integral of smoothstep over [0, x].
inline double smoothstepInt(double x) { return x * x * x - 0.5 * x * x * x * x; }

}  // namespace j7

/// Validated, precomputed schedule of the nominal profile.
class J7Profile {
 public:
  /// Throws std::invalid_argument with a clear message on non-finite or out-of-range parameters, or when the
  /// travel is too short to contain the two ramps.
  explicit J7Profile(const J7ProfileParams& p) : p_(p) {
    auto bad = [](const std::string& m) { throw std::invalid_argument("J7Profile: " + m); };
    if (!(std::isfinite(p.speed_rad_s) && p.speed_rad_s > 0.0)) bad("speed must be finite and > 0");
    if (!(std::isfinite(p.travel_rad) && p.travel_rad > 0.0)) bad("travel must be finite and > 0");
    if (!(std::isfinite(p.ramp_s) && p.ramp_s > 0.0)) bad("ramp_s must be finite and > 0");
    if (!(std::isfinite(p.stop_ramp_s) && p.stop_ramp_s > 0.0)) bad("stop_ramp_s must be finite and > 0");
    if (!(std::isfinite(p.hold_s) && p.hold_s >= 0.0)) bad("hold_s must be finite and >= 0");
    const double ramps = 0.5 * p.speed_rad_s * (p.ramp_s + p.stop_ramp_s);
    if (p.travel_rad < ramps) {
      bad("travel " + std::to_string(p.travel_rad) + " rad is shorter than the two ramps alone (" + std::to_string(ramps) +
          " rad = v*(T1+T2)/2): reduce the ramps or the speed, or increase the travel");
    }
    tc_ = (p.travel_rad - ramps) / p.speed_rad_s;
    d_after_start_ = 0.5 * p.speed_rad_s * p.ramp_s;
    d_after_const_ = d_after_start_ + p.speed_rad_s * tc_;
  }

  const J7ProfileParams& params() const { return p_; }
  double constantS() const { return tc_; }                                    ///< Tc
  double motionDurationS() const { return p_.ramp_s + tc_ + p_.stop_ramp_s; }  ///< T1 + Tc + T2
  double totalDurationS() const { return motionDurationS() + p_.hold_s; }
  double constantStartS() const { return p_.ramp_s; }
  double constantEndS() const { return p_.ramp_s + tc_; }

  /// Nominal displacement and velocity at time t >= 0 from the start of the motion (t < 0 is clamped to 0).
  J7Ref sample(double t) const {
    const double v = p_.speed_rad_s;
    if (!(t > 0.0)) return {0.0, 0.0};
    if (t < p_.ramp_s) {
      const double x = t / p_.ramp_s;
      return {v * p_.ramp_s * j7::smoothstepInt(x), v * j7::smoothstep(x)};
    }
    if (t < p_.ramp_s + tc_) return {d_after_start_ + v * (t - p_.ramp_s), v};
    const double ts = t - p_.ramp_s - tc_;
    if (ts < p_.stop_ramp_s) {
      const double y = ts / p_.stop_ramp_s;
      return {d_after_const_ + v * p_.stop_ramp_s * (y - j7::smoothstepInt(y)), v * (1.0 - j7::smoothstep(y))};
    }
    return {p_.travel_rad, 0.0};  // exact: no accumulated rounding
  }

 private:
  J7ProfileParams p_;
  double tc_ = 0.0;
  double d_after_start_ = 0.0;
  double d_after_const_ = 0.0;
};

/// Early stop: from displacement d0 and velocity v0 at the switch time, smoothstep ramp of the velocity to
/// zero over `stop_s`. tau = time since the switch (>= 0). Exact end displacement d0 + v0*stop_s/2.
inline J7Ref stopFrom(double d0, double v0, double stop_s, double tau) {
  if (!(tau > 0.0)) return {d0, v0};
  if (tau >= stop_s) return {d0 + 0.5 * v0 * stop_s, 0.0};
  const double y = tau / stop_s;
  return {d0 + v0 * stop_s * (y - j7::smoothstepInt(y)), v0 * (1.0 - j7::smoothstep(y))};
}

/// Travel check against the joint limits. All angles in rad.
struct J7TravelCheck {
  bool ok = false;
  double lo = 0.0;       ///< lower limit + margin
  double hi = 0.0;       ///< upper limit - margin
  double max_pos = 0.0;  ///< largest rotation possible in the + direction from q0 (<= 0 if q0 is outside [lo, hi])
  double max_neg = 0.0;  ///< largest rotation possible in the - direction (same convention, positive number)
  double q_end = 0.0;    ///< q0 + sign * travel
};

inline J7TravelCheck checkJ7Travel(double q0, int sign, double travel, double lower, double upper, double margin) {
  J7TravelCheck c;
  c.lo = lower + margin;
  c.hi = upper - margin;
  c.max_pos = c.hi - q0;
  c.max_neg = q0 - c.lo;
  c.q_end = q0 + static_cast<double>(sign) * travel;
  c.ok = std::isfinite(q0) && std::isfinite(c.q_end) && q0 >= c.lo && q0 <= c.hi && c.q_end >= c.lo && c.q_end <= c.hi;
  return c;
}

/// Satellite phase from the J7 reading: theta = theta0 + phase_map * (q7 - q7_0). phase_map (+1/-1) is the physical map
/// J7 -> satellite and is independent of the command direction `sign` (q7_ref = q7_0 + sign * d).
inline double phaseTheta(double theta0, int phase_map, double q7, double q7_0) {
  return theta0 + static_cast<double>(phase_map) * (q7 - q7_0);
}

/// Finite-difference slope over a sliding window: (x_now - x_old)/(t_now - t_old), where x_old is the newest
/// stored sample that is at least `window_s` older than the newest one (the oldest one available while the
/// history is shorter than the window). Fixed storage, no allocation in push().
class SlopeFilter {
 public:
  /// capacity: maximum number of samples kept; must cover window_s / min_period (+ margin).
  SlopeFilter(double window_s, std::size_t capacity)
      : window_(window_s), t_(capacity < 4 ? 4 : capacity, 0.0), x_(capacity < 4 ? 4 : capacity, 0.0) {
    if (!(std::isfinite(window_s) && window_s > 0.0)) throw std::invalid_argument("SlopeFilter: window must be > 0");
  }

  /// Adds a sample (t must be non-decreasing) and returns the slope (0 until two samples with t_new > t_old exist).
  double push(double t, double x) noexcept {
    const std::size_t n = t_.size();
    if (count_ == n) {  // full: drop the oldest
      head_ = (head_ + 1) % n;
      --count_;
    }
    const std::size_t pos = (head_ + count_) % n;
    t_[pos] = t;
    x_[pos] = x;
    ++count_;
    while (count_ >= 2 && (t - t_[(head_ + 1) % n]) >= window_) {
      head_ = (head_ + 1) % n;
      --count_;
    }
    const double dt = t - t_[head_];
    return dt > 0.0 ? (x - x_[head_]) / dt : 0.0;
  }

  double window() const { return window_; }

 private:
  double window_;
  std::vector<double> t_, x_;
  std::size_t head_ = 0;
  std::size_t count_ = 0;
};

}  // namespace real_replay
