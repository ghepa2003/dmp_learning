#pragma once

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
