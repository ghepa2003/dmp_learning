#include "satellite_grasp_planner/core/window_center.hpp"

#include <cmath>
#include <stdexcept>

namespace satellite_grasp_planner {
namespace core {

using haptic_dmp_learning::core::CubeSatelliteModel;

double windowCenterTheta(const CubeSatelliteModel& model, const Eigen::Vector3d& robot_base_world) {
    const Eigen::Vector3d u = robot_base_world - model.cubeCenterWorld();
    const Eigen::Vector3d a = model.axisWorld();
    const Eigen::Vector3d u_perp = u - u.dot(a) * a;
    if (!(u_perp.norm() >= 1e-9)) {
        throw std::invalid_argument(
            "windowCenterTheta: robot base lies on the spin axis (|u_perp| < 1e-9).");
    }
    const Eigen::Vector3d n = model.faceNormalBody();
    return std::atan2(a.dot(n.cross(u_perp)), n.dot(u_perp));
}

}  // namespace core
}  // namespace satellite_grasp_planner
