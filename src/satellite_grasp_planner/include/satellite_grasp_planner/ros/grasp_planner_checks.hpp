#pragma once

/**
 * @file grasp_planner_checks.hpp
 * @brief Pure (Eigen + STL only, NO rclcpp) startup/runtime validation for grasp_planner_node.
 *        Kept free of rclcpp::Node so these checks are unit-testable without rclcpp::init().
 */

#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace satellite_grasp_planner {
namespace ros_wrapper {

/// @throws std::invalid_argument if @p robot_base_world is farther than @p tol_m from the origin.
/// The core selection pipeline (candidate_scan.cpp) combines the robot-base-frame FK position with
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

/// @throws std::runtime_error if @p frame_id != "world" or if @p position is farther than 10 mm from
/// @p center. Mirrors prodmp_gazebo_executor_node::onSatelliteOdom()'s first-sample check exactly.
void checkFirstOdomSample(const Eigen::Vector3d& position, const std::string& frame_id,
                          const Eigen::Vector3d& center);

/// @throws std::runtime_error if @p twist_angular projected onto @p axis disagrees with
/// @p omega_rad_s by more than @p tol_rad_s.
void checkOmegaConsistency(const Eigen::Vector3d& twist_angular, const Eigen::Vector3d& axis, double omega_rad_s,
                           double tol_rad_s);

}  // namespace ros_wrapper
}  // namespace satellite_grasp_planner
