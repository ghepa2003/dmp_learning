#pragma once

/**
 * @file grasp_planner_checks.hpp
 * @brief Pure (Eigen + STL only, NO rclcpp) startup/runtime validation for grasp_planner_node.
 *        Kept free of rclcpp::Node so these checks are unit-testable without rclcpp::init().
 */

#include <optional>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace satellite_grasp_planner {
namespace ros_wrapper {

/// @throws std::invalid_argument if @p robot_base_world is farther than @p tol_m from the origin.
/// The core selection pipeline (grasp_s3_selection.cpp) combines the robot-base-frame FK position with
/// world-frame satellite geometry with no transform - it implicitly assumes robot base == world
/// (the same assumption prodmp_gazebo_executor_node.cpp states explicitly). v1 enforces this rather
/// than silently relying on it.
void checkRobotBaseWorldIsZero(const Eigen::Vector3d& robot_base_world, double tol_m);

/// @throws std::invalid_argument if the normalized @p axis_world (cube_geometry) differs from the
/// normalized @p rotation_axis (satellite_rotation_axis) by more than @p tol: both must feed the same
/// R(axis, theta) rotation for the tracked phase and the model's phase to refer to the same rotation.
void checkCubeAxisMatchesRotationAxis(const Eigen::Vector3d& axis_world, const Eigen::Vector3d& rotation_axis,
                                      double tol = 1e-6);

/// @throws std::invalid_argument if @p center_world (cube_geometry) differs from @p rotation_center
/// (satellite_rotation_center) by more than @p tol_m. buildSelectionInputs() overwrites
/// cube_params.center_world with the snapshot's center anyway (selection_inputs.cpp), so a mismatch
/// here is silently ignored downstream - this check catches it at startup instead.
void checkCubeCenterMatchesRotationCenter(const Eigen::Vector3d& center_world,
                                          const Eigen::Vector3d& rotation_center, double tol_m = 1e-6);

/// @throws std::invalid_argument if @p q_ref is not (within @p tol_deg) a pure rotation about
/// @p rotation_axis. Mirrors PhaseTracker's own off-axis decomposition (AngleAxis of R(q_ref)
/// projected onto the axis) exactly, so a q_ref that fails this would also make PhaseTracker itself
/// throw once real odometry arrives - this just moves that failure to startup.
void checkQRefOnAxis(const Eigen::Quaterniond& q_ref, const Eigen::Vector3d& rotation_axis, double tol_deg);

/// Same check for a satellite whose theta = 0 orientation is @p base (R_base, world frame) instead of the identity:
/// PhaseTracker computes R_rel = R(q_body) R(q_ref)^T, and a body that spins about the axis starting from R_base is
/// Rot(axis, theta) R_base, so q_ref must be Rot(axis, phi) R_base, i.e. q_ref * base^-1 must be a rotation about
/// @p rotation_axis. A base exactly equal to the identity calls the 3-argument overload unchanged.
/// @throws std::invalid_argument as the 3-argument overload.
void checkQRefOnAxis(const Eigen::Quaterniond& q_ref, const Eigen::Vector3d& rotation_axis, double tol_deg,
                     const Eigen::Quaterniond& base);

/// Orientation of the satellite at theta = 0 as the xacro builds it: body x = collar normal (@p face_normal_body, the
/// cube_geometry parameter, world frame at theta = 0), body z = spin axis (@p rotation_axis), body y = z x x.
/// Columns [n, a x n, a]; for n = +x, a = +z it is exactly the identity quaternion. Both vectors are normalized here.
/// @throws std::invalid_argument for a zero / non-finite vector or if |n . a| > 1e-6 (the face is not lateral: the
///         same tolerance as CubeSatelliteModel::kLateralTolerance).
Eigen::Quaterniond satelliteBaseOrientation(const Eigen::Vector3d& face_normal_body, const Eigen::Vector3d& rotation_axis);

/// @throws std::runtime_error if @p frame_id != "world" or if @p position is farther than 10 mm from
/// @p center. Mirrors prodmp_gazebo_executor_node::onSatelliteOdom()'s first-sample check exactly.
void checkFirstOdomSample(const Eigen::Vector3d& position, const std::string& frame_id,
                          const Eigen::Vector3d& center);

/// Angular velocity [rad/s] of the body about the unit @p axis, estimated from two orientation samples (world frame,
/// time apart @p dt_s): dq = q_curr * q_prev^-1, rotation vector of dq (shortest arc: the sign of the quaternion double
/// cover is irrelevant), projected on the normalized @p axis, divided by @p dt_s. Both quaternions are normalized here.
/// NOT derived from nav_msgs/Odometry twist.angular: Gazebo's OdometryPublisher fills it with d(roll,pitch,yaw)/dt of the
/// pose quaternion (R = Rz(y) Ry(p) Rx(r)), which equals omega . a only for a vertical axis (G28, alpha = 20 deg).
/// Unambiguous only while |omega| * dt_s < pi (the arc is taken in [0, pi]).
/// @return std::nullopt if @p dt_s <= 0 or non-finite, or a quaternion is non-finite or has (near) zero norm.
/// @throws std::invalid_argument if @p axis has norm <= 1e-9.
std::optional<double> axisRateFromQuaternions(const Eigen::Quaterniond& q_prev, const Eigen::Quaterniond& q_curr,
                                              double dt_s, const Eigen::Vector3d& axis);

/// @throws std::runtime_error if the angular velocity about @p axis estimated from (@p q_prev, @p q_curr, @p dt_s)
/// (axisRateFromQuaternions) disagrees with @p omega_rad_s by more than @p tol_rad_s.
/// @throws std::invalid_argument if @p axis is degenerate or the pair is unusable (dt_s <= 0, non-finite / zero quaternion).
void checkOmegaConsistency(const Eigen::Quaterniond& q_prev, const Eigen::Quaterniond& q_curr, double dt_s,
                           const Eigen::Vector3d& axis, double omega_rad_s, double tol_rad_s);

}  // namespace ros_wrapper
}  // namespace satellite_grasp_planner
