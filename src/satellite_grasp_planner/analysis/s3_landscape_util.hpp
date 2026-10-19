#pragma once

// Pure helpers of the s3_landscape tool (header-only, Eigen only; testable without a URDF).
//
// Follow arc: after AT_END the executor co-rotates the grasp target with the satellite
// (haptic_dmp_learning/core/pose_rotation.hpp: rotatePoseAboutZ, position' = c + Rz(a)(p - c),
// orientation' = Rz(a) * orientation). The tool samples that arc at t_j = j * follow_s / (n - 1), j = 0..n-1, with
// angle a_j = omega * t_j (omega signed). No rotation formula is rewritten here: rotatePoseAboutZ is used as is.

#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "haptic_dmp_learning/core/pose_rotation.hpp"

namespace s3_landscape {

/// n >= 2 equally spaced times in [0, follow_s]. @throws std::invalid_argument for n < 2 or follow_s < 0.
inline std::vector<double> followSampleTimes(double follow_s, int n) {
    if (n < 2 || !(follow_s >= 0.0) || !std::isfinite(follow_s)) {
        throw std::invalid_argument("followSampleTimes: n >= 2 and finite follow_s >= 0 required.");
    }
    std::vector<double> t(static_cast<std::size_t>(n));
    for (int j = 0; j < n; ++j) t[static_cast<std::size_t>(j)] = follow_s * static_cast<double>(j) / (n - 1);
    return t;
}

/// Grasp pose rotated by omega * t about the satellite axis through @p center. The axis must be world z (as in the
/// planner: the executor's rotatePoseAboutZ is z-only). @throws std::invalid_argument otherwise.
inline haptic_dmp_learning::core::RotatablePose rotateAboutSatelliteAxis(
    const Eigen::Vector3d& center, const Eigen::Vector3d& axis,
    const haptic_dmp_learning::core::RotatablePose& pose, double omega_rad_s, double t_s) {
    if ((axis.normalized() - Eigen::Vector3d::UnitZ()).norm() > 1e-9) {
        throw std::invalid_argument("rotateAboutSatelliteAxis: only the +z axis is supported (rotatePoseAboutZ).");
    }
    return haptic_dmp_learning::core::rotatePoseAboutZ(center, pose, omega_rad_s * t_s);
}

}  // namespace s3_landscape
