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

/// Tilted satellite axis, same convention as tools/s1_run.py (tilt_axis, tilt_rotation, tilt_quat_xyzw, tilt_values):
///   axis        a = (sin(alpha) cos(phi), sin(alpha) sin(phi), cos(alpha))
///   R_tilt      = Rot(n, alpha), n = (-sin(phi), cos(phi), 0)  (Rodrigues; R_tilt * z = a)
///   q_ref       = R_tilt as (x, y, z, w) = (-sin(phi) sin(alpha/2), cos(phi) sin(alpha/2), 0, cos(alpha/2))
///   face_normal = R_tilt * (1, 0, 0)
/// alpha in [0, 20] deg, phi in [0, 360) deg (s1_run.validate_tilt). alpha = 0: axis z, q_ref identity, normal +x.
struct AxisTilt {
  Eigen::Vector3d axis;
  Eigen::Quaterniond q_ref;  ///< satellite_q_ref (x, y, z, w); the selector does not use it (the snapshot carries theta), reported only
  Eigen::Vector3d face_normal;
};

inline AxisTilt axisTilt(double alpha_deg, double phi_deg) {
  constexpr double kMaxAlphaDeg = 20.0;  // tools/s1_run.py AXIS_TILT_MAX_DEG
  if (!(std::isfinite(alpha_deg) && alpha_deg >= 0.0 && alpha_deg <= kMaxAlphaDeg)) {
    throw std::invalid_argument("axisTilt: --axis-tilt-deg must be in [0, 20].");
  }
  if (!(std::isfinite(phi_deg) && phi_deg >= 0.0 && phi_deg < 360.0)) {
    throw std::invalid_argument("axisTilt: --axis-lean-deg must be in [0, 360).");
  }
  const double al = alpha_deg * M_PI / 180.0, ph = phi_deg * M_PI / 180.0;
  const Eigen::Vector3d k(-std::sin(ph), std::cos(ph), 0.0);  // already unit length
  Eigen::Matrix3d K;
  K << 0.0, -k.z(), k.y(), k.z(), 0.0, -k.x(), -k.y(), k.x(), 0.0;
  const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() + std::sin(al) * K + (1.0 - std::cos(al)) * (K * K);
  AxisTilt t;
  t.axis = Eigen::Vector3d(std::sin(al) * std::cos(ph), std::sin(al) * std::sin(ph), std::cos(al));
  const double h = al / 2.0, s = std::sin(h);
  t.q_ref = Eigen::Quaterniond(std::cos(h), -std::sin(ph) * s, std::cos(ph) * s, 0.0);  // (w, x, y, z)
  t.face_normal = R * Eigen::Vector3d::UnitX();
  return t;
}

}  // namespace s3_landscape
