#pragma once

#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace haptic_dmp_learning {
namespace core {

/// Position + orientation of a frame, both expressed in the world frame.
struct RotatablePose {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

/**
 * @brief Rigidly rotates @p pose by @p dtheta_rad about the world-z-parallel axis through @p center.
 *
 *   position'    = center + Rz(dtheta) * (position - center)
 *   orientation' = Rz(dtheta) * orientation          (left multiplication: a rotation of the world frame)
 *
 * Used to make a grasp target co-rotate with the satellite after AT_END.
 */
inline RotatablePose rotatePoseAboutZ(const Eigen::Vector3d& center, const RotatablePose& pose,
                                      double dtheta_rad) {
  const Eigen::Quaterniond rz(Eigen::AngleAxisd(dtheta_rad, Eigen::Vector3d::UnitZ()));
  RotatablePose out;
  out.position = center + rz * (pose.position - center);
  out.orientation = (rz * pose.orientation).normalized();
  return out;
}

/**
 * @brief Rigidly rotates @p pose by @p dtheta_rad about the axis @p axis (any direction, normalized here) through @p center.
 *
 *   position'    = center + R(axis, dtheta) * (position - center)
 *   orientation' = R(axis, dtheta) * orientation     (left multiplication: a rotation of the world frame)
 *
 * Same construction as rotatePoseAboutZ(); for axis == UnitZ the two functions give bit-identical results
 * (axis.normalized() of an exact unit vector is exact).
 * @throws std::invalid_argument if @p axis is not finite or has zero norm.
 */
inline RotatablePose rotatePoseAboutAxis(const Eigen::Vector3d& center, const Eigen::Vector3d& axis,
                                         const RotatablePose& pose, double dtheta_rad) {
  if (!(axis.allFinite() && axis.norm() > 0.0)) {
    throw std::invalid_argument("rotatePoseAboutAxis: axis must be finite and non-zero");
  }
  const Eigen::Quaterniond rz(Eigen::AngleAxisd(dtheta_rad, axis.normalized()));
  RotatablePose out;
  out.position = center + rz * (pose.position - center);
  out.orientation = (rz * pose.orientation).normalized();
  return out;
}

/**
 * @brief Effective co-rotation angle after AT_END: follows the satellite for @p follow_s, then stays frozen.
 *
 *   t_since_end < 0 (before AT_END) or follow_s <= 0 (follow for the whole hold) or t_since_end <= follow_s
 *       -> dtheta_now
 *   t_since_end >  follow_s
 *       -> dtheta_frozen (the caller keeps the last value returned while following)
 */
inline double effectiveFollowDtheta(double t_since_end, double follow_s, double dtheta_now,
                                    double dtheta_frozen) {
  if (t_since_end < 0.0 || !(follow_s > 0.0) || t_since_end <= follow_s) {
    return dtheta_now;
  }
  return dtheta_frozen;
}

}  // namespace core
}  // namespace haptic_dmp_learning
