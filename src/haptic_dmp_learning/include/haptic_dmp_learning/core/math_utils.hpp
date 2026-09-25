#pragma once

/**
 * @file math_utils.hpp
 * @brief Header-only math helpers shared across packages (Eigen + STL only, NO ROS):
 *        angle wrapping, degree/radian conversion and shortest-path quaternion SLERP.
 *
 * Names kept from their previous homes (satellite_intercept.hpp, phase_selector.cpp).
 */

#include <cmath>
#include <Eigen/Geometry>

namespace haptic_dmp_learning {
namespace core {

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

/// @brief Constant-rate SLERP with shortest-path hemisphere alignment (same
/// convention as franka_cartesian_control::core::computePoseError).
inline Eigen::Quaterniond slerpShortestPath(const Eigen::Quaterniond& q0,
                                            const Eigen::Quaterniond& q1, double s) {
    Eigen::Quaterniond target = q1;
    if (q0.coeffs().dot(target.coeffs()) < 0.0) {
        target.coeffs() = -target.coeffs();
    }
    return q0.slerp(s, target).normalized();
}

}  // namespace core
}  // namespace haptic_dmp_learning
