#include "satellite_grasp_planner/ros/grasp_planner_checks.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

namespace satellite_grasp_planner {
namespace ros_wrapper {

void checkRobotBaseWorldIsZero(const Eigen::Vector3d& robot_base_world, double tol_m) {
    const double off = robot_base_world.norm();
    if (off > tol_m) {
        std::ostringstream m;
        m << "robot_base_world [" << robot_base_world.transpose() << "] is " << off
          << " m from the origin (limit " << tol_m
          << " m): the selection pipeline assumes robot base == world with no transform; v1 requires "
             "robot_base_world to be zero.";
        throw std::invalid_argument(m.str());
    }
}

void checkCubeAxisMatchesRotationAxis(const Eigen::Vector3d& axis_world, const Eigen::Vector3d& rotation_axis,
                                      double tol) {
    if (!(axis_world.norm() > 1e-9) || !(rotation_axis.norm() > 1e-9)) {
        throw std::invalid_argument("checkCubeAxisMatchesRotationAxis: axis norm <= 1e-9");
    }
    const double d = (axis_world.normalized() - rotation_axis.normalized()).norm();
    if (d > tol) {
        std::ostringstream m;
        m << "cube_geometry.axis_world [" << axis_world.normalized().transpose()
          << "] != satellite_rotation_axis [" << rotation_axis.normalized().transpose()
          << "] (must be identical: both feed the same R(axis, theta) rotation)";
        throw std::invalid_argument(m.str());
    }
}

void checkCubeCenterMatchesRotationCenter(const Eigen::Vector3d& center_world,
                                          const Eigen::Vector3d& rotation_center, double tol_m) {
    const double d = (center_world - rotation_center).norm();
    if (d > tol_m) {
        std::ostringstream m;
        m << "cube_geometry.center_world [" << center_world.transpose() << "] != satellite_rotation_center ["
          << rotation_center.transpose() << "] (must be identical: buildSelectionInputs() overwrites "
             "cube_params.center_world with the snapshot center anyway)";
        throw std::invalid_argument(m.str());
    }
}

void checkQRefOnAxis(const Eigen::Quaterniond& q_ref, const Eigen::Vector3d& rotation_axis, double tol_deg) {
    if (!(rotation_axis.norm() > 1e-9)) {
        throw std::invalid_argument("checkQRefOnAxis: rotation_axis norm <= 1e-9");
    }
    const Eigen::Vector3d axis = rotation_axis.normalized();
    const Eigen::AngleAxisd aa(q_ref.normalized());
    const Eigen::Vector3d v = aa.angle() * aa.axis();
    const double off_rad = (v - v.dot(axis) * axis).norm();
    const double off_deg = off_rad * 180.0 / M_PI;
    if (off_deg > tol_deg) {
        std::ostringstream m;
        m << "satellite_q_ref is not a rotation about satellite_rotation_axis (orthogonal component "
          << off_deg << " deg > " << tol_deg
          << " deg): PhaseTracker would reject the physical spin at startup with this q_ref/axis pair.";
        throw std::invalid_argument(m.str());
    }
}

void checkFirstOdomSample(const Eigen::Vector3d& position, const std::string& frame_id,
                          const Eigen::Vector3d& center) {
    if (frame_id != "world") {
        throw std::runtime_error("satellite odometry frame_id '" + frame_id + "' != 'world'");
    }
    const double off = (position - center).norm();
    if (off > 0.01) {
        std::ostringstream m;
        m << "satellite odometry origin [" << position.x() << ", " << position.y() << ", " << position.z()
          << "] is " << off * 1000.0 << " mm from satellite_rotation_center (limit 10 mm)";
        throw std::runtime_error(m.str());
    }
}

void checkOmegaConsistency(const Eigen::Vector3d& twist_angular, const Eigen::Vector3d& axis, double omega_rad_s,
                           double tol_rad_s) {
    if (!(axis.norm() > 1e-9)) throw std::invalid_argument("checkOmegaConsistency: axis norm <= 1e-9");
    const double measured = twist_angular.dot(axis.normalized());
    const double err = std::fabs(measured - omega_rad_s);
    if (err > tol_rad_s) {
        std::ostringstream m;
        m << "measured angular velocity along axis (" << measured
          << " rad/s) differs from satellite_rotation_angular_velocity_deg_s-derived omega ("
          << omega_rad_s << " rad/s) by " << err << " rad/s (> " << tol_rad_s << " rad/s)";
        throw std::runtime_error(m.str());
    }
}

}  // namespace ros_wrapper
}  // namespace satellite_grasp_planner
