#pragma once

// Pure helpers of the s3_landscape tool (header-only; followSampleTimes needs only the standard library; testable without a URDF).
//
// Follow arc: after AT_END the executor co-rotates the grasp target with the satellite
// (haptic_dmp_learning/core/pose_rotation.hpp: rotatePoseAboutAxis, position' = c + R(axis, a)(p - c),
// orientation' = R(axis, a) * orientation). The tool samples that arc at t_j = j * follow_s / (n - 1), j = 0..n-1, with
// angle a_j = omega * t_j (omega signed). No rotation formula is rewritten here: the caller (s3_landscape.cpp) uses
// rotatePoseAboutAxis directly with the angle omega * t_j.

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

}  // namespace s3_landscape
