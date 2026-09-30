#pragma once

/**
 * @file start_state.hpp
 * @brief Joint configuration the selection starts from when it is NOT the default (readyPose): the
 *        planner's start_joints_override_rad parameter. Validation and the distance-from-readyPose report
 *        live here (RobotModel + Eigen only, no ROS) so they are unit-testable.
 */

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "franka_cartesian_control/core/robot_model.hpp"

namespace satellite_grasp_planner {
namespace core {

/// Name of the planner's ROS parameter (double[7], fer_joint1..7 order; empty = readyPose).
inline constexpr const char* kStartJointsParameterName = "start_joints_override_rad";

struct StartState {
    franka_cartesian_control::core::RobotModel::JointVector q =
        franka_cartesian_control::core::RobotModel::JointVector::Zero();
    std::string source;  ///< where q comes from, for the provenance and the log
};

/// @throws std::invalid_argument naming kStartJointsParameterName if @p values does not have exactly 7
/// elements, if one is not finite, or if one is outside [lower, upper] (the message names fer_jointN
/// and the limits).
void validateStartJoints(const std::vector<double>& values,
                         const franka_cartesian_control::core::RobotModel::JointVector& lower,
                         const franka_cartesian_control::core::RobotModel::JointVector& upper);

/// Validates @p values against the joint limits of the robot in @p urdf_path (fer_joint1..7, same
/// joint names and limits parsing as buildSelectionInputs) and returns the StartState
/// {q = values, source = "start_joints_override_rad parameter"}.
/// @throws std::invalid_argument as validateStartJoints(); std::runtime_error if the URDF cannot be read.
StartState makeStartStateFromOverride(const std::vector<double>& values, const std::string& urdf_path);

struct StartDistance {
    double joint_norm_rad = 0.0;  ///< || q0 - readyPose ||
    double joint_max_rad = 0.0;   ///< max_i |q0_i - readyPose_i|
    int joint_max_index = 1;      ///< the i (1..7, fer_jointN) where it happens
    double task_mm = 0.0;         ///< || FK(q0) - FK(readyPose) || [mm]
};

/// Distance of @p q0 from RobotModel::readyPose() in joint space and in task space (EE position). Leaves
/// @p robot updated at @p q0 with zero velocity, as buildSelectionInputs() does.
StartDistance distanceFromReadyPose(franka_cartesian_control::core::RobotModel& robot,
                                    const franka_cartesian_control::core::RobotModel::JointVector& q0);

}  // namespace core
}  // namespace satellite_grasp_planner
