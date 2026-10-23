#include "satellite_grasp_planner/ros/grasp_planner_checks.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

#include "haptic_dmp_learning/core/math_utils.hpp"

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
  const double off_deg = haptic_dmp_learning::core::radToDeg(off_rad);
  if (off_deg > tol_deg) {
    std::ostringstream m;
    m << "satellite_q_ref is not a rotation about satellite_rotation_axis (orthogonal component "
      << off_deg << " deg > " << tol_deg
      << " deg): PhaseTracker would reject the physical spin at startup with this q_ref/axis pair.";
    throw std::invalid_argument(m.str());
  }
}

void checkQRefOnAxis(const Eigen::Quaterniond& q_ref, const Eigen::Vector3d& rotation_axis, double tol_deg,
                     const Eigen::Quaterniond& base) {
  if (base.coeffs() == Eigen::Quaterniond::Identity().coeffs()) {
    checkQRefOnAxis(q_ref, rotation_axis, tol_deg);  // identity base: exactly the historical check
    return;
  }
  const Eigen::Quaterniond q_rel = (q_ref.normalized() * base.normalized().conjugate()).normalized();
  try {
    checkQRefOnAxis(q_rel, rotation_axis, tol_deg);
  } catch (const std::invalid_argument& e) {
    throw std::invalid_argument(std::string(e.what()) +
                                " [checked on q_ref * R_base^-1, R_base = orientation at theta = 0 derived from "
                                "cube_geometry.face_normal_body and the rotation axis]");
  }
}

Eigen::Quaterniond satelliteBaseOrientation(const Eigen::Vector3d& face_normal_body, const Eigen::Vector3d& rotation_axis) {
  if (!(face_normal_body.allFinite() && face_normal_body.norm() > 1e-9) ||
      !(rotation_axis.allFinite() && rotation_axis.norm() > 1e-9)) {
    throw std::invalid_argument("satelliteBaseOrientation: face_normal_body and rotation_axis must be finite, non-zero");
  }
  const Eigen::Vector3d n = face_normal_body.normalized();
  const Eigen::Vector3d a = rotation_axis.normalized();
  if (std::fabs(n.dot(a)) > 1e-6) {
    std::ostringstream m;
    m << "satelliteBaseOrientation: cube_geometry.face_normal_body is not lateral (|n . axis| = " << std::fabs(n.dot(a))
      << ")";
    throw std::invalid_argument(m.str());
  }
  Eigen::Matrix3d R;
  R.col(0) = n;
  R.col(1) = a.cross(n).normalized();
  R.col(2) = a;
  return Eigen::Quaterniond(R).normalized();
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

std::optional<double> axisRateFromQuaternions(const Eigen::Quaterniond& q_prev, const Eigen::Quaterniond& q_curr,
                                              double dt_s, const Eigen::Vector3d& axis) {
  if (!(axis.norm() > 1e-9)) throw std::invalid_argument("axisRateFromQuaternions: axis norm <= 1e-9");
  if (!(dt_s > 0.0) || !std::isfinite(dt_s)) return std::nullopt;
  const double n_prev = q_prev.norm(), n_curr = q_curr.norm();
  if (!std::isfinite(n_prev) || !std::isfinite(n_curr) || !(n_prev > 1e-9) || !(n_curr > 1e-9)) return std::nullopt;
  Eigen::Quaterniond dq = (q_curr.normalized() * q_prev.normalized().inverse()).normalized();
  if (dq.w() < 0.0) dq.coeffs() = -dq.coeffs();  // double cover: shortest arc, angle in [0, pi]
  const double s = dq.vec().norm();
  if (s < 1e-300) return 0.0;
  const double angle = 2.0 * std::atan2(s, dq.w());
  const Eigen::Vector3d rotvec = (angle / s) * dq.vec();
  return rotvec.dot(axis.normalized()) / dt_s;
}

void checkOmegaConsistency(const Eigen::Quaterniond& q_prev, const Eigen::Quaterniond& q_curr, double dt_s,
                           const Eigen::Vector3d& axis, double omega_rad_s, double tol_rad_s) {
  const auto rate = axisRateFromQuaternions(q_prev, q_curr, dt_s, axis);
  if (!rate) throw std::invalid_argument("checkOmegaConsistency: unusable sample pair (dt <= 0 or invalid quaternion)");
  const double measured = *rate;
  const double err = std::fabs(measured - omega_rad_s);
  if (err > tol_rad_s) {
    std::ostringstream m;
    m << "angular velocity along axis estimated from the odometry quaternions (" << measured
      << " rad/s) differs from satellite_rotation_angular_velocity_deg_s-derived omega ("
      << omega_rad_s << " rad/s) by " << err << " rad/s (> " << tol_rad_s << " rad/s)";
    throw std::runtime_error(m.str());
  }
}

}  // namespace ros_wrapper
}  // namespace satellite_grasp_planner
