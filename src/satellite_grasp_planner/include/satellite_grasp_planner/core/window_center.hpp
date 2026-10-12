#pragma once

/**
 * @file window_center.hpp
 * @brief Center of the satellite-phase window facing the robot base. Pure, no ROS.
 */

#include <Eigen/Dense>

#include "haptic_dmp_learning/core/cube_satellite_model.hpp"

namespace satellite_grasp_planner {
namespace core {

/// Center of the theta window: the satellite phase at which the face normal (rotated by theta)
/// points, in the plane orthogonal to the axis, towards the robot base:
/// atan2(a.(n x u_perp), n.u_perp), with u = base - center, u_perp = u - (u.a)a, n = face normal
/// (body, theta = 0), a = spin axis. @throws std::invalid_argument if |u_perp| < 1e-9.
double windowCenterTheta(const haptic_dmp_learning::core::CubeSatelliteModel& model,
                         const Eigen::Vector3d& robot_base_world);

}  // namespace core
}  // namespace satellite_grasp_planner
