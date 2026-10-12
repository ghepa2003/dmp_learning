#pragma once

/**
 * @file tcp_roll.hpp
 * @brief Roll of a TCP orientation about its own approach axis (local z). Pure, header-only
 *        (Eigen only, NO ROS).
 */

#include <Eigen/Geometry>

namespace haptic_dmp_learning {
namespace core {

/// q_nominal * AngleAxis(psi, UnitZ), normalized: rotation about the LOCAL z of the TCP, i.e. the
/// approach axis (which therefore does not change).
inline Eigen::Quaterniond applyRoll(const Eigen::Quaterniond& q_nominal, double psi) {
    return (q_nominal * Eigen::Quaterniond(Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ())))
        .normalized();
}

}  // namespace core
}  // namespace haptic_dmp_learning
