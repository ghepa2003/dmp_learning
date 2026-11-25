#pragma once

// Pure geometry / phase helpers of the satellite mock-up held by robot 2 (no libfranka, no I/O, no allocation in
// the per-cycle functions). Used by tools/satellite_node.cpp (robot AND dry-run: same code) and by the unit tests.
//
// Conventions (all configurable in hw/hw_config.yaml, see hw/README.md):
//   - frame of reference for everything published = base of robot 1 ("frame 1");
//   - base of robot 2 in frame 1: translation t, rotation yaw about z (Rz(yaw));
//   - flange pose of robot 2 from the Panda DH model (Craig convention, flange d = 0.107), in frame 2, then mapped
//     to frame 1. This is the ONLY way the phase is computed (robot and dry-run). In robot mode the node checks it
//     once at preflight against O_T_EE * F_T_EE^-1 read from the robot;
//   - satellite centre  c = p_flange + flange_to_center * z_flange;
//   - rotation axis published as axis_up = -z_flange (flange z points down): omega > 0 = counter-clockwise seen from
//     above (right-hand rule about axis_up), omega < 0 = clockwise;
//   - phase from the flange: theta_f = x_angle + collar_phase_offset, x_angle = atan2(x_y, x_x) of the flange x axis
//     in frame 1 (angle counter-clockwise from +x, seen from above);
//   - phase from J7: theta_q7 = q7_offset + q7_sign * q7 (nominal -135 deg, -1: valid with q1 = -90 deg, no yaw);
//   - every phase is brought onto the branch [-270 deg, +90 deg) (NOT wrapPi: the admitted window [-180, 0] deg has
//     an edge right on the +-180 jump of wrapPi).

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace real_replay {
namespace sat {

using Q7 = std::array<double, 7>;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kBranchLoRad = -270.0 * kDeg;  ///< lower end of the phase branch [-270, +90) deg

/// x brought into [lo, lo + 2*pi).
inline double wrapToBranch(double x, double lo) noexcept {
  const double two_pi = 2.0 * kPi;
  if (!std::isfinite(x)) return x;
  double y = x - two_pi * std::floor((x - lo) / two_pi);
  if (y >= lo + two_pi) y -= two_pi;  // rounding at the upper end
  if (y < lo) y = lo;
  return y;
}

/// Phase on the branch [-270 deg, +90 deg).
inline double unwrapPhase(double x) noexcept { return wrapToBranch(x, kBranchLoRad); }

/// (-pi, pi].
inline double wrapPi(double x) noexcept {
  const double y = wrapToBranch(x, -kPi);  // [-pi, pi)
  return y == -kPi ? kPi : y;
}

// ---- Panda forward kinematics (DH, Craig / Franka documentation) -------------------------------------------------
struct PandaDh {
  static constexpr std::array<double, 8> a = {0.0, 0.0, 0.0, 0.0825, -0.0825, 0.0, 0.088, 0.0};
  static constexpr std::array<double, 8> d = {0.333, 0.0, 0.316, 0.0, 0.384, 0.0, 0.0, 0.107};
  static constexpr std::array<double, 8> alpha = {0.0, -kPi / 2, kPi / 2, kPi / 2, -kPi / 2, kPi / 2, kPi / 2, 0.0};
};

/// Joint limits of the Panda/FER [rad] (same constants as satellite_j7 / friction_id).
constexpr Q7 kQMin = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
constexpr Q7 kQMax = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};

/// Flange pose (fer_link8) in the base frame of the same robot.
inline Eigen::Isometry3d pandaFlangeFk(const Q7& q) noexcept {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for (std::size_t i = 0; i < 8; ++i) {
    const double th = i < 7 ? q[i] : 0.0;
    const double ca = std::cos(PandaDh::alpha[i]), sa = std::sin(PandaDh::alpha[i]);
    const double ct = std::cos(th), st = std::sin(th);
    Eigen::Matrix4d A;
    A << ct, -st, 0.0, PandaDh::a[i],                 //
        st * ca, ct * ca, -sa, -sa * PandaDh::d[i],  //
        st * sa, ct * sa, ca, ca * PandaDh::d[i],    //
        0.0, 0.0, 0.0, 1.0;
    T = T * A;
  }
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = T.block<3, 3>(0, 0);
  out.translation() = T.block<3, 1>(0, 3);
  return out;
}

/// Base of robot 2 in frame 1: p1 = R * p2 + t, R = Rz(yaw).
struct BaseTransform {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  static BaseTransform fromYaw(const std::array<double, 3>& t_m, double yaw_rad) {
    BaseTransform b;
    b.R = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    b.t = Eigen::Vector3d(t_m[0], t_m[1], t_m[2]);
    return b;
  }
};

/// Nominal axis (upwards unit vector) from the two tilt angles: axis_up = Ry(tilt_y) * Rx(tilt_x) * e_z (frame 1).
inline Eigen::Vector3d axisFromTilt(double tilt_x_rad, double tilt_y_rad) {
  return (Eigen::AngleAxisd(tilt_y_rad, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(tilt_x_rad, Eigen::Vector3d::UnitX()) *
          Eigen::Vector3d::UnitZ())
      .normalized();
}

struct PhaseParams {
  double collar_offset_rad = -45.0 * kDeg;
  double q7_offset_rad = -135.0 * kDeg;
  int q7_sign = -1;
  double flange_to_center_m = 0.1035;
  BaseTransform base2_in_base1;
};

struct SatGeom {
  double theta_flange = 0.0;  ///< phase from the flange x axis, branch [-270, 90) deg [rad]
  double theta_q7 = 0.0;      ///< phase from q7, same branch [rad]
  Eigen::Vector3d center = Eigen::Vector3d::Zero();   ///< frame 1 [m]
  Eigen::Vector3d axis_up = Eigen::Vector3d::UnitZ();  ///< frame 1, unit
  Eigen::Vector3d flange = Eigen::Vector3d::Zero();   ///< flange position, frame 1 [m]
};

inline double phaseFromQ7(const PhaseParams& p, double q7) noexcept {
  return unwrapPhase(p.q7_offset_rad + static_cast<double>(p.q7_sign) * q7);
}

/// Phase from a flange rotation matrix expressed in frame 1.
inline double phaseFromFlangeRotation(const PhaseParams& p, const Eigen::Matrix3d& R1) noexcept {
  return unwrapPhase(std::atan2(R1(1, 0), R1(0, 0)) + p.collar_offset_rad);
}

inline SatGeom computeSatGeom(const PhaseParams& p, const Q7& q) noexcept {
  const Eigen::Isometry3d F2 = pandaFlangeFk(q);
  const Eigen::Matrix3d R1 = p.base2_in_base1.R * F2.linear();
  const Eigen::Vector3d p1 = p.base2_in_base1.R * F2.translation() + p.base2_in_base1.t;
  SatGeom g;
  g.flange = p1;
  g.axis_up = -R1.col(2);
  g.center = p1 + p.flange_to_center_m * R1.col(2);
  g.theta_flange = phaseFromFlangeRotation(p, R1);
  g.theta_q7 = phaseFromQ7(p, q[6]);
  return g;
}

/// Smallest signed difference a - b on the circle, (-pi, pi].
inline double angleDiff(double a, double b) noexcept { return wrapPi(a - b); }

// ---- Window and travel ------------------------------------------------------------------------------------------
/// Travel of J7 limited by the window. q7_ref = q7_0 + sign * d; the nominal profile ends at the window edge minus
/// `margin` in the direction of motion; the travel is also capped by max_travel (if finite and > 0).
struct TravelPlan {
  bool ok = false;
  double travel = 0.0;     ///< [rad] > 0 when ok
  double q7_end = 0.0;     ///< q7_0 + sign * travel
  double inner_lo = 0.0;   ///< window lower + margin
  double inner_hi = 0.0;   ///< window upper - margin
  bool capped = false;     ///< travel reduced by max_travel
  std::string why;         ///< reason when !ok
};

inline TravelPlan planTravelInWindow(double q7_0, int sign, double win_lo, double win_hi, double margin, double max_travel,
                                     double min_travel) {
  TravelPlan t;
  t.inner_lo = win_lo + margin;
  t.inner_hi = win_hi - margin;
  if (!(std::isfinite(q7_0) && std::isfinite(win_lo) && std::isfinite(win_hi) && margin >= 0.0 && (sign == 1 || sign == -1))) {
    t.why = "non-finite input or invalid sign/margin";
    return t;
  }
  if (!(t.inner_lo < t.inner_hi)) {
    t.why = "window with margin is empty";
    return t;
  }
  if (q7_0 < t.inner_lo || q7_0 > t.inner_hi) {
    t.why = "q7 at start outside the window with margin";
    return t;
  }
  double travel = sign > 0 ? t.inner_hi - q7_0 : q7_0 - t.inner_lo;
  if (std::isfinite(max_travel) && max_travel > 0.0 && max_travel < travel) {
    travel = max_travel;
    t.capped = true;
  }
  t.travel = travel;
  t.q7_end = q7_0 + static_cast<double>(sign) * travel;
  if (!(travel >= min_travel && travel > 0.0)) {
    t.why = "travel available in the window (" + std::to_string(travel / kDeg) + " deg) shorter than the minimum (" +
            std::to_string(min_travel / kDeg) + " deg = both ramps)";
    return t;
  }
  t.ok = true;
  return t;
}

/// x inside [lo + margin, hi - margin].
inline bool insideWindow(double x, double lo, double hi, double margin) noexcept {
  return std::isfinite(x) && x >= lo + margin && x <= hi - margin;
}

// ---- Omega: least-squares slope over a sliding time window --------------------------------------------------------
/// Fixed storage (allocated in the constructor only). push() is O(n) over the window (exact, centred sums, no
/// drifting running sums) and allocation-free. Returns NaN until the history spans the whole window.
class OmegaRegressor {
 public:
  /// sample_period_s: nominal spacing of the pushes. The window counts as full when the stored samples span at
  /// least window_s - 1.5 * sample_period_s (pushes driven by a 1 kHz loop are not exactly on a grid).
  OmegaRegressor(double window_s, double sample_period_s)
      : window_(window_s), tol_(1.5 * sample_period_s) {
    if (!(std::isfinite(window_s) && window_s > 0.0)) throw std::invalid_argument("OmegaRegressor: window must be > 0");
    if (!(std::isfinite(sample_period_s) && sample_period_s > 0.0 && sample_period_s < window_s)) {
      throw std::invalid_argument("OmegaRegressor: sample period must be in (0, window)");
    }
    const std::size_t cap = static_cast<std::size_t>(std::ceil(window_s / sample_period_s * 1.5)) + 8;
    t_.assign(cap, 0.0);
    x_.assign(cap, 0.0);
  }

  void reset() noexcept {
    head_ = 0;
    count_ = 0;
    last_ = std::numeric_limits<double>::quiet_NaN();
  }

  /// t non-decreasing. Returns the slope dx/dt [x units / s], NaN while the window is incomplete or on bad input.
  double push(double t, double x) noexcept {
    const std::size_t n = t_.size();
    if (!(std::isfinite(t) && std::isfinite(x))) return last_ = std::numeric_limits<double>::quiet_NaN();
    if (count_ == n) {
      head_ = (head_ + 1) % n;
      --count_;
    }
    const std::size_t pos = (head_ + count_) % n;
    t_[pos] = t;
    x_[pos] = x;
    ++count_;
    while (count_ >= 2 && (t - t_[head_]) > window_ + 1e-9) {
      head_ = (head_ + 1) % n;
      --count_;
    }
    if (count_ < 3 || (t - t_[head_]) < window_ - tol_) return last_ = std::numeric_limits<double>::quiet_NaN();
    double tm = 0.0, xm = 0.0;
    for (std::size_t i = 0; i < count_; ++i) {
      tm += t_[(head_ + i) % n];
      xm += x_[(head_ + i) % n];
    }
    tm /= static_cast<double>(count_);
    xm /= static_cast<double>(count_);
    double stt = 0.0, stx = 0.0;
    for (std::size_t i = 0; i < count_; ++i) {
      const double dt = t_[(head_ + i) % n] - tm;
      stt += dt * dt;
      stx += dt * (x_[(head_ + i) % n] - xm);
    }
    return last_ = (stt > 0.0 ? stx / stt : std::numeric_limits<double>::quiet_NaN());
  }

  double last() const noexcept { return last_; }
  double window() const noexcept { return window_; }

 private:
  double window_;
  double tol_;
  std::vector<double> t_, x_;
  std::size_t head_ = 0;
  std::size_t count_ = 0;
  double last_ = std::numeric_limits<double>::quiet_NaN();
};

/// omega_meas is usable only when the whole regression window [t_run - window_s, t_run] lies inside the
/// constant-speed phase [ramp_s, const_end_s] of a nominal run and the regression is finite.
/// running = nominal profile in progress (RUNNING), t_run = time since START.
inline bool omegaValid(bool running, double t_run, double ramp_s, double window_s, double const_end_s, double omega) noexcept {
  return running && std::isfinite(omega) && t_run >= ramp_s + window_s && t_run <= const_end_s;
}

// ---- Heartbeat --------------------------------------------------------------------------------------------------
/// The receiving thread increments a counter for every valid heartbeat (START counts as one too); the control loop
/// calls update() with its own clock. Armed by the first heartbeat; timed out when no new one arrives for > timeout_s.
class HeartbeatMonitor {
 public:
  explicit HeartbeatMonitor(double timeout_s) : timeout_(timeout_s) {
    if (!(std::isfinite(timeout_s) && timeout_s > 0.0)) throw std::invalid_argument("HeartbeatMonitor: timeout must be > 0");
  }
  /// Returns true when timed out (stays true afterwards).
  bool update(double t, std::uint32_t count) noexcept {
    if (timed_out_) return true;
    if (count != last_count_) {
      last_count_ = count;
      last_t_ = t;
      armed_ = true;
    }
    if (armed_ && (t - last_t_) > timeout_) timed_out_ = true;
    return timed_out_;
  }
  bool armed() const noexcept { return armed_; }
  bool timedOut() const noexcept { return timed_out_; }
  double age(double t) const noexcept { return armed_ ? t - last_t_ : std::numeric_limits<double>::quiet_NaN(); }

 private:
  double timeout_;
  std::uint32_t last_count_ = 0;
  double last_t_ = 0.0;
  bool armed_ = false;
  bool timed_out_ = false;
};

}  // namespace sat
}  // namespace real_replay
