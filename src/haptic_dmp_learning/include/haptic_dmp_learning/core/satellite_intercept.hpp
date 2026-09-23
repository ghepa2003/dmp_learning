#pragma once

/**
 * @file satellite_intercept.hpp
 * @brief Pure logic (Eigen + STL only, NO ROS) for the satellite rotation model of
 *        prodmp_gazebo_executor_node: goal math shared by the frozen retarget branch.
 *
 * Conventions
 * - Angles are radians internally; *_deg_* names are degrees.
 * - theta = 0 is the reference pose of the satellite. Positive theta is a right-handed
 *   rotation about the (unit) axis.
 */

#include <Eigen/Dense>

namespace haptic_dmp_learning {
namespace core {
namespace satellite_intercept {

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

}  // namespace satellite_intercept
}  // namespace core
}  // namespace haptic_dmp_learning
