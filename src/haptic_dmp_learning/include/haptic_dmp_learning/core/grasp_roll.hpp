#pragma once

/**
 * @file grasp_roll.hpp
 * @brief Pure helpers for the grasp roll psi about the TCP approach axis (local z), exploiting
 *        the 180-degree symmetry of the parallel gripper (Eigen + STL only, NO ROS).
 */

#include <array>
#include <cmath>
#include <stdexcept>

#include <Eigen/Geometry>

#include "haptic_dmp_learning/core/math_utils.hpp"

namespace haptic_dmp_learning {
namespace core {

/// Default roll tolerance psi_tol = 10 deg. PROVISIONAL and configurable: it stands for how much
/// the gripper can be misaligned about the approach axis before the pad no longer sits flat on the
/// wall (pad width), but the pad width has NOT been confirmed, so the value is a placeholder.
constexpr double kDefaultRollToleranceRad = 10.0 * 3.14159265358979323846 / 180.0;

/// Wraps psi [rad] to (-pi/2, pi/2] using the 180-degree symmetry of the gripper
/// (psi and psi + pi are the same grasp). Convention: -pi/2 -> +pi/2. Values within 1e-12 of the
/// boundary are treated as ON it, so round-off in e.g. 3*pi/2 cannot flip the branch.
inline double wrapRollHalfPi(double psi) {
    constexpr double kHalfPi = 0.5 * kPi;
    constexpr double kBoundaryEps = 1e-12;
    double w = wrapPi(psi);  // (-pi, pi]
    if (w > kHalfPi + kBoundaryEps) {
        w -= kPi;
    } else if (w <= -kHalfPi + kBoundaryEps) {
        w += kPi;
    }
    return w;
}

/// Roll penalty (wrapRollHalfPi(psi) / psi_tol)^2: 0 at psi = 0 (mod pi) and exactly 1 when the
/// wrapped deviation reaches the tolerance. @throws std::invalid_argument if psi_tol <= 0.
inline double rollPenalty(double psi, double psi_tol) {
    if (!(psi_tol > 0.0)) {
        throw std::invalid_argument("rollPenalty: psi_tol must be > 0.");
    }
    const double r = wrapRollHalfPi(psi) / psi_tol;
    return r * r;
}

/// q_nominal * AngleAxis(psi, UnitZ), normalized: rotation about the LOCAL z of the TCP, i.e. the
/// approach axis (which therefore does not change).
inline Eigen::Quaterniond applyRoll(const Eigen::Quaterniond& q_nominal, double psi) {
    return (q_nominal * Eigen::Quaterniond(Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ())))
        .normalized();
}

/// The two equivalent roll basins {wrap(psi), wrap(psi + pi)} (wrapPi, in (-pi, pi]), both to be
/// tried in a search.
inline std::array<double, 2> equivalentRolls(double psi) {
    return {wrapPi(psi), wrapPi(psi + kPi)};
}

}  // namespace core
}  // namespace haptic_dmp_learning
