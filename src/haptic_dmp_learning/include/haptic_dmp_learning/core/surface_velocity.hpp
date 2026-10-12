#pragma once

/**
 * @file surface_velocity.hpp
 * @brief Velocity of the grasp points on the spinning satellite (SI units: m, m/s, rad). Pure, header-only,
 *        no ProDMP, no RobotModel, no ROS.
 */

#include <Eigen/Dense>

#include "haptic_dmp_learning/core/cube_satellite_model.hpp"

namespace haptic_dmp_learning {
namespace core {

/// Velocity of grasp point k on the spinning satellite: (omega * axis_unit) x (p_k(theta) - center).
/// omega is SIGNED: positive = counter-clockwise seen from +axis. The real case is clockwise, so
/// omega is negative there.
inline Eigen::Vector3d surfaceVelocity(const CubeSatelliteModel& satellite, GraspPointId k,
                                       double theta_rad, double omega_rad_s) {
    const Eigen::Vector3d r = satellite.graspPoseAt(k, theta_rad).position_world -
                              satellite.cubeCenterWorld();
    return (omega_rad_s * satellite.axisWorld()).cross(r);
}

}  // namespace core
}  // namespace haptic_dmp_learning
