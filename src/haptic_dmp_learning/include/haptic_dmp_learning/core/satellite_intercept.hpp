#pragma once

/**
 * @file satellite_intercept.hpp
 * @brief Pure logic (Eigen + STL only, NO ROS) for the satellite rotation model of
 *        prodmp_gazebo_executor_node: shared goal math (frozen + continuous), phase
 *        measurement from a quaternion, phase-consistency check, trigger crossing,
 *        rollout time base and start-up validation of the "continuous" mode.
 *
 * Conventions
 * - Angles are radians internally; *_deg_* names are degrees.
 * - theta = 0 is the reference pose of the satellite. Positive theta is a right-handed
 *   rotation about the (unit) axis.
 * - Every failure throws (std::invalid_argument for configuration, std::runtime_error for
 *   runtime data): no silent fallbacks.
 */

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace haptic_dmp_learning {
namespace core {
namespace satellite_intercept {

constexpr double kPi = 3.14159265358979323846;

inline double degToRad(double d) { return d * kPi / 180.0; }
inline double radToDeg(double r) { return r * 180.0 / kPi; }

/// Wraps to (-pi, pi].
inline double wrapPi(double a) {
    double w = std::fmod(a + kPi, 2.0 * kPi);
    if (w <= 0.0) w += 2.0 * kPi;
    return w - kPi;
}

/// Wraps to [0, 2*pi).
inline double wrap2Pi(double a) {
    double w = std::fmod(a, 2.0 * kPi);
    if (w < 0.0) w += 2.0 * kPi;
    return w;
}

// ---------------------------------------------------------------------------------------
// 1. Goal math shared by frozen and continuous
// ---------------------------------------------------------------------------------------

/// center + R(theta, axis) * (p - center). Same operations, same order as the original
/// inline code of the frozen branch (axis.normalized() inside).
inline Eigen::Vector3d rotateAboutCenter(const Eigen::Vector3d& center, const Eigen::Vector3d& axis,
                                         double theta_rad, const Eigen::Vector3d& p) {
    const Eigen::AngleAxisd rot(theta_rad, axis.normalized());
    return center + rot * (p - center);
}

struct AnchoredGoal {
    Eigen::Vector3d p_demo_world;     ///< ee_anchor + demo_grasp_goal (grasp point at theta = 0)
    Eigen::Vector3d p_rotated_world;  ///< the same point rotated by theta about (center, axis)
    Eigen::Vector3d goal_relative;    ///< p_rotated_world - ee_anchor (what ProDMP stores with relative goal)
};

/// Anchoring + rotation used by BOTH the frozen and the continuous branch.
inline AnchoredGoal anchorAndRotate(const Eigen::Vector3d& ee_anchor, const Eigen::Vector3d& demo_grasp_goal,
                                    const Eigen::Vector3d& center, const Eigen::Vector3d& axis,
                                    double theta_rad) {
    AnchoredGoal g;
    g.p_demo_world = ee_anchor + demo_grasp_goal;
    g.p_rotated_world = rotateAboutCenter(center, axis, theta_rad, g.p_demo_world);
    g.goal_relative = g.p_rotated_world - ee_anchor;
    return g;
}

/// Distance of p from the line (center, axis).
inline double distanceFromAxis(const Eigen::Vector3d& p, const Eigen::Vector3d& center,
                               const Eigen::Vector3d& axis) {
    const Eigen::Vector3d a = axis.normalized();
    const Eigen::Vector3d d = p - center;
    return (d - d.dot(a) * a).norm();
}

// ---------------------------------------------------------------------------------------
// 2. Phase from a body quaternion
// ---------------------------------------------------------------------------------------

struct PhaseSample {
    double theta_rad = 0.0;        ///< signed angle about the axis, continuously unwrapped
    double off_axis_deg = 0.0;     ///< rotation-vector component orthogonal to the axis
    Eigen::Matrix3d R_rel = Eigen::Matrix3d::Identity();  ///< R(q_body) * R(q_ref)^T
};

class PhaseTracker {
public:
    /// @param off_axis_tol_deg reject samples whose rotation vector has an orthogonal component
    ///        larger than this (default 0.5 deg).
    PhaseTracker(const Eigen::Vector3d& axis, const Eigen::Quaterniond& q_ref, double off_axis_tol_deg = 0.5)
        : axis_(normalizedAxis(axis)),
          R_ref_(q_ref.normalized().toRotationMatrix()),
          tol_deg_(off_axis_tol_deg) {}

    /// @throws std::runtime_error if the rotation is not about the configured axis.
    PhaseSample update(const Eigen::Quaterniond& q_body) {
        PhaseSample s;
        s.R_rel = q_body.normalized().toRotationMatrix() * R_ref_.transpose();
        const Eigen::AngleAxisd aa(s.R_rel);
        const Eigen::Vector3d v = aa.angle() * aa.axis();  // rotation vector, |v| in [0, pi]
        const double raw = v.dot(axis_);
        s.off_axis_deg = radToDeg((v - raw * axis_).norm());
        if (s.off_axis_deg > tol_deg_) {
            std::ostringstream m;
            m << "satellite phase: rotation is off the configured axis (orthogonal component "
              << s.off_axis_deg << " deg > " << tol_deg_ << " deg)";
            throw std::runtime_error(m.str());
        }
        s.theta_rad = has_prev_ ? prev_ + wrapPi(raw - prev_) : wrapPi(raw);
        prev_ = s.theta_rad;
        has_prev_ = true;
        return s;
    }

    bool hasSample() const { return has_prev_; }
    double theta() const { return prev_; }
    const Eigen::Vector3d& axis() const { return axis_; }

private:
    static Eigen::Vector3d normalizedAxis(const Eigen::Vector3d& a) {
        if (!(a.norm() > 1e-9)) throw std::invalid_argument("PhaseTracker: axis norm <= 1e-9");
        return a.normalized();
    }
    Eigen::Vector3d axis_;
    Eigen::Matrix3d R_ref_;
    double tol_deg_;
    bool has_prev_ = false;
    double prev_ = 0.0;
};

// ---------------------------------------------------------------------------------------
// 3. Phase-vs-time consistency (measured omega against configured omega)
// ---------------------------------------------------------------------------------------

class PhaseConsistencyChecker {
public:
    PhaseConsistencyChecker(double omega_rad_s, double window_s, double tol_deg)
        : omega_(omega_rad_s), window_(window_s), tol_deg_(tol_deg) {
        if (!(window_s > 0.0)) throw std::invalid_argument("phase_consistency_window_s must be > 0");
        if (!(tol_deg > 0.0)) throw std::invalid_argument("phase_consistency_tol_deg must be > 0");
    }

    /// Feed one (header.stamp seconds, unwrapped theta) sample. Evaluates a check whenever the
    /// history spans at least `window_s`.
    /// @throws std::runtime_error on non-monotonic stamps or if |dtheta - omega*dt| > tol.
    void update(double t_s, double theta_rad) {
        if (!hist_.empty() && t_s < hist_.back().first) {
            throw std::runtime_error("phase consistency: non-monotonic header.stamp (time axis wrong)");
        }
        hist_.emplace_back(t_s, theta_rad);
        while (hist_.size() >= 2 && (t_s - hist_[1].first) >= window_) hist_.pop_front();
        const double span = t_s - hist_.front().first;
        if (span >= window_) {
            const double d_theta = theta_rad - hist_.front().second;
            last_error_deg_ = radToDeg(d_theta - omega_ * span);
            ++checks_;
            if (std::fabs(last_error_deg_) > tol_deg_) {
                std::ostringstream m;
                m << "phase consistency: measured dtheta " << radToDeg(d_theta) << " deg over " << span
                  << " s differs from omega*dt " << radToDeg(omega_ * span) << " deg by " << last_error_deg_
                  << " deg (> " << tol_deg_ << " deg): configured omega wrong or time axis wrong";
                throw std::runtime_error(m.str());
            }
        }
    }

    bool validated() const { return checks_ > 0; }
    int checks() const { return checks_; }
    double lastErrorDeg() const { return last_error_deg_; }

private:
    double omega_, window_, tol_deg_;
    std::deque<std::pair<double, double>> hist_;
    int checks_ = 0;
    double last_error_deg_ = 0.0;
};

// ---------------------------------------------------------------------------------------
// 4. Trigger: crossing of theta_trig = theta_int - omega * T_total
// ---------------------------------------------------------------------------------------

class InterceptTrigger {
public:
    struct Result {
        bool fired = false;
        double e_rad = 0.0;             ///< wrap(theta_sat - theta_trig) in (-pi, pi]
        bool first_sample_past = false; ///< true only on the first sample, if already past the threshold
    };

    InterceptTrigger(double theta_int_rad, double omega_rad_s, double T_total_s)
        : omega_(omega_rad_s), theta_trig_(wrap2Pi(theta_int_rad - omega_rad_s * T_total_s)) {
        if (!(std::fabs(omega_rad_s) > 0.0)) throw std::invalid_argument("InterceptTrigger: omega == 0");
    }

    double thetaTrigRad() const { return theta_trig_; }

    /// No angular tolerance: fires on the first sample with e >= 0 (omega > 0) / e <= 0 (omega < 0)
    /// after a sample on the other side has been seen. If the very first sample is already past the
    /// threshold, the next lap is awaited.
    Result update(double theta_sat_rad) {
        Result r;
        r.e_rad = wrapPi(theta_sat_rad - theta_trig_);
        const bool past = omega_ > 0.0 ? (r.e_rad >= 0.0) : (r.e_rad <= 0.0);
        if (first_) {
            first_ = false;
            first_past_ = past;
            r.first_sample_past = past;
            armed_ = !past;
            return r;
        }
        if (!armed_) {
            if (!past) armed_ = true;   // a sample on the "before" side: crossing can now be seen
            return r;
        }
        if (past) r.fired = true;
        return r;
    }

    /// Time [s] until theta_trig is next reached, from the given phase.
    double expectedWaitS(double theta_sat_rad) const {
        double d = omega_ > 0.0 ? wrap2Pi(theta_trig_ - theta_sat_rad) : wrap2Pi(theta_sat_rad - theta_trig_);
        if (first_past_ && d < 1e-12) d = 2.0 * kPi;
        return d / std::fabs(omega_);
    }

    bool armed() const { return armed_; }

    /// 360/|omega| + T_total + margin
    static double timeoutS(double omega_deg_s, double T_total_s, double margin_s) {
        return 360.0 / std::fabs(omega_deg_s) + T_total_s + margin_s;
    }

private:
    double omega_;
    double theta_trig_;
    bool first_ = true;
    bool first_past_ = false;
    bool armed_ = false;
};

// ---------------------------------------------------------------------------------------
// 5. Rollout time bases
// ---------------------------------------------------------------------------------------

/// Frozen branch (unchanged behaviour): elapsed advances by a fixed dt per tick and the rollout ends
/// when (elapsed + dt) >= tau.
inline bool fixedStepAtEnd(double elapsed, double dt, double tau) { return (elapsed + dt) >= tau; }

/// Frozen branch: dt to feed into ProDMP::step()/QuaternionDMP::step() for this tick. Mirrors
/// MeasuredTimeBase::update()'s ticks_==0 case (dt_step=0.0 on the first tick): at s=0 every
/// shape + goal term is exactly zero by construction, so the first published sample must equal
/// init_pos_ exactly instead of already s=dt_nominal/tau - otherwise the FrameAligner on the
/// controller side captures that offset and applies it to the whole trajectory, including the
/// goal. elapsed_ still starts at 0.0 and advances by this same value each tick (0.0 on the
/// first, dt_nominal after), so it stays consistent with what was actually stepped.
inline double frozenStepDt(bool is_first_tick, double dt_nominal) { return is_first_tick ? 0.0 : dt_nominal; }

/// Continuous branch: elapsed = now - t_roll0 (clock injected by the caller).
class MeasuredTimeBase {
public:
    struct Tick {
        double elapsed = 0.0;   ///< now - t_roll0 [s]
        double dt_step = 0.0;   ///< elapsed - previous elapsed (0 on the first tick)
        bool first = false;
        bool at_end = false;    ///< elapsed >= tau
    };

    MeasuredTimeBase(double dt_nominal, double tau) : dt_nominal_(dt_nominal), tau_(tau) {
        if (!(dt_nominal > 0.0) || !(tau > 0.0)) throw std::invalid_argument("MeasuredTimeBase: dt and tau must be > 0");
    }

    /// @throws std::runtime_error if the clock goes backwards.
    Tick update(double now_s) {
        Tick t;
        if (ticks_ == 0) {
            t0_ = now_s;
            t.first = true;
        } else {
            if (now_s < last_now_) throw std::runtime_error("MeasuredTimeBase: clock went backwards");
            const double period = now_s - last_now_;
            t.dt_step = period;
            max_period_ = std::max(max_period_, period);
            if (period > 1.5 * dt_nominal_) ++skipped_;
        }
        last_now_ = now_s;
        ++ticks_;
        t.elapsed = now_s - t0_;
        t.at_end = t.elapsed >= tau_;
        return t;
    }

    std::uint64_t tickCount() const { return ticks_; }
    std::uint64_t skippedTicks() const { return skipped_; }
    double maxPeriod() const { return max_period_; }
    double meanPeriod() const { return ticks_ >= 2 ? (last_now_ - t0_) / static_cast<double>(ticks_ - 1) : 0.0; }
    double t0() const { return t0_; }
    double dtNominal() const { return dt_nominal_; }

private:
    double dt_nominal_, tau_;
    std::uint64_t ticks_ = 0, skipped_ = 0;
    double t0_ = 0.0, last_now_ = 0.0, max_period_ = 0.0;
};

// ---------------------------------------------------------------------------------------
// 6. Start-up validation of the "continuous" mode
// ---------------------------------------------------------------------------------------

/// Everything the node read from its parameters. std::optional = parameter not provided.
struct ContinuousInputs {
    bool rotation_enabled = false;
    bool use_sim_time = false;
    bool target_odom_required = true;
    // Explicit-parameter flags: center/axis/omega have code defaults in the frozen branch; in
    // continuous they must be provided explicitly.
    bool axis_explicit = false, center_explicit = false, omega_explicit = false;
    Eigen::Vector3d axis = Eigen::Vector3d::Zero();
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double omega_deg_s = 0.0;

    std::optional<double> intercept_phase_deg;
    std::optional<double> contact_time_s;
    std::optional<std::string> phase_source;   ///< "measured" | "model"
    std::optional<std::string> odom_topic;
    std::optional<std::vector<double>> q_ref;  ///< x, y, z, w
    std::optional<double> model_phase0_deg;

    double contact_time_tolerance_s = 0.0;
    bool allow_contact_time_mismatch = false;
    double phase_consistency_window_s = 0.0;
    double phase_consistency_tol_deg = 0.0;
    double trigger_timeout_margin_s = 0.0;
    double rollout_time_tol_s = 0.0;

    double tau = 0.0;  ///< prodmp tau [s]
    double dt = 0.0;   ///< nominal tick period [s]
};

struct ContinuousConfig {
    Eigen::Vector3d axis_unit = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double omega_deg_s = 0.0, omega_rad_s = 0.0;
    double theta_int_rad = 0.0;
    double contact_time_s = 0.0;
    bool measured = true;
    std::string odom_topic;
    Eigen::Quaterniond q_ref = Eigen::Quaterniond::Identity();
    double theta0_model_rad = 0.0;
    double consistency_window_s = 0.0, consistency_tol_deg = 0.0;
    double trigger_timeout_margin_s = 0.0, rollout_time_tol_s = 0.0;
    double expected_contact_time_s = 0.0;   ///< tau + dt (see expectedContactTimeS)
    bool contact_time_mismatch_allowed = false;
    bool contact_time_mismatch = false;
    double trigger_timeout_s = 0.0;
    double tau = 0.0, dt = 0.0;
};

/// T_total expected from the code, measured from the trigger detection to at_end (node clock):
///   tau  (rollout duration, measured elapsed >= tau)
/// + dt   (first tick of step_timer_, created right after the trigger, fires one period later)
/// + 0    (TF lookup of ee_start is a single non-blocking lookup: no retry latency in continuous)
/// The start-up delay (startup_delay_sec) elapses BEFORE the trigger machinery is armed, so it is
/// not part of T_total.
inline double expectedContactTimeS(double tau, double dt) { return tau + dt; }

namespace detail {
[[noreturn]] inline void missing(const std::string& name) {
    throw std::invalid_argument("satellite_rotation_mode='continuous': parameter '" + name +
                                "' is mandatory and was not provided");
}
[[noreturn]] inline void invalid(const std::string& name, const std::string& why) {
    throw std::invalid_argument("satellite_rotation_mode='continuous': parameter '" + name + "' invalid: " + why);
}
}  // namespace detail

/// @throws std::invalid_argument with an explicit message naming the offending parameter.
inline ContinuousConfig validateContinuous(const ContinuousInputs& in) {
    using detail::invalid;
    using detail::missing;
    ContinuousConfig c;
    if (!in.rotation_enabled) invalid("satellite_rotation_enabled", "must be true");
    if (!in.use_sim_time) invalid("use_sim_time", "must be true (the time base is the node's sim clock)");
    if (in.target_odom_required) invalid("target_odom_required", "must be false (the goal comes from the satellite model)");
    if (!in.axis_explicit) missing("satellite_rotation_axis");
    if (!in.center_explicit) missing("satellite_rotation_center");
    if (!in.omega_explicit) missing("satellite_rotation_angular_velocity_deg_s");
    if (!(in.axis.norm() > 1e-9)) invalid("satellite_rotation_axis", "norm must be > 1e-9");
    c.axis_unit = in.axis.normalized();
    c.center = in.center;
    if (!(std::fabs(in.omega_deg_s) > 1e-9)) invalid("satellite_rotation_angular_velocity_deg_s", "must be non-zero");
    c.omega_deg_s = in.omega_deg_s;
    c.omega_rad_s = degToRad(in.omega_deg_s);

    if (!in.intercept_phase_deg) missing("satellite_intercept_phase_deg");
    c.theta_int_rad = degToRad(*in.intercept_phase_deg);
    if (!in.contact_time_s) missing("satellite_contact_time_s");
    if (!(*in.contact_time_s > 0.0)) invalid("satellite_contact_time_s", "must be > 0");
    c.contact_time_s = *in.contact_time_s;
    if (!in.phase_source) missing("satellite_phase_source");
    if (*in.phase_source == "measured") {
        c.measured = true;
        if (!in.odom_topic || in.odom_topic->empty()) missing("satellite_odom_topic");
        c.odom_topic = *in.odom_topic;
        if (!in.q_ref) missing("satellite_q_ref");
        if (in.q_ref->size() != 4) invalid("satellite_q_ref", "needs 4 elements (x, y, z, w)");
        const auto& q = *in.q_ref;
        const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (!(std::fabs(n - 1.0) <= 1e-6)) invalid("satellite_q_ref", "norm must be 1 +/- 1e-6");
        c.q_ref = Eigen::Quaterniond(q[3], q[0], q[1], q[2]);
    } else if (*in.phase_source == "model") {
        c.measured = false;
        if (!in.model_phase0_deg) missing("satellite_model_phase0_deg");
        c.theta0_model_rad = degToRad(*in.model_phase0_deg);
    } else {
        invalid("satellite_phase_source", "must be 'measured' or 'model', got '" + *in.phase_source + "'");
    }

    if (!(in.phase_consistency_window_s > 0.0)) invalid("phase_consistency_window_s", "must be > 0");
    if (!(in.phase_consistency_tol_deg > 0.0)) invalid("phase_consistency_tol_deg", "must be > 0");
    if (!(in.trigger_timeout_margin_s >= 0.0)) invalid("trigger_timeout_margin_s", "must be >= 0");
    if (!(in.rollout_time_tol_s > 0.0)) invalid("rollout_time_tol_s", "must be > 0");
    if (!(in.contact_time_tolerance_s > 0.0)) invalid("contact_time_tolerance_s", "must be > 0");
    if (!(in.tau > 0.0) || !(in.dt > 0.0)) invalid("tau/dt", "must be > 0");
    c.consistency_window_s = in.phase_consistency_window_s;
    c.consistency_tol_deg = in.phase_consistency_tol_deg;
    c.trigger_timeout_margin_s = in.trigger_timeout_margin_s;
    c.rollout_time_tol_s = in.rollout_time_tol_s;
    c.tau = in.tau;
    c.dt = in.dt;

    c.expected_contact_time_s = expectedContactTimeS(in.tau, in.dt);
    c.contact_time_mismatch_allowed = in.allow_contact_time_mismatch;
    c.contact_time_mismatch = std::fabs(c.contact_time_s - c.expected_contact_time_s) > in.contact_time_tolerance_s;
    if (c.contact_time_mismatch && !in.allow_contact_time_mismatch) {
        std::ostringstream m;
        m << "satellite_contact_time_s=" << c.contact_time_s << " differs from the value expected from the code ("
          << "tau " << in.tau << " + dt " << in.dt << " = " << c.expected_contact_time_s << " s) by more than "
          << "contact_time_tolerance_s=" << in.contact_time_tolerance_s
          << " (set allow_contact_time_mismatch=true to override explicitly)";
        throw std::invalid_argument(m.str());
    }
    c.trigger_timeout_s = InterceptTrigger::timeoutS(c.omega_deg_s, c.contact_time_s, c.trigger_timeout_margin_s);
    return c;
}

}  // namespace satellite_intercept
}  // namespace core
}  // namespace haptic_dmp_learning
