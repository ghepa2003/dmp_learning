#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace haptic_dmp_learning {
namespace core {

/**
 * @brief Represents a single time-stamped Cartesian pose sample in a teleoperation demonstration.
 *
 * @details
 * During kinesthetic demonstration or master-slave teleoperation (e.g. via Geomagic Touch),
 * end-effector poses are sampled at discrete time instants:
 * - Time `t`: Elapsed time in seconds since the start of recording (t_0 = 0.0).
 * - Position `position`: 3D Cartesian coordinates [x, y, z] in meters.
 * - Orientation `orientation`: Unit quaternion [w, x, y, z] in SO(3).
 */
struct Sample {
    double t = 0.0;                                                 ///< Elapsed timestamp (seconds, t >= 0)
    Eigen::Vector3d position = Eigen::Vector3d::Zero();            ///< 3D Cartesian position (meters)
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity(); ///< Orientation unit quaternion in SO(3)
};

}  // namespace core
}  // namespace haptic_dmp_learning
