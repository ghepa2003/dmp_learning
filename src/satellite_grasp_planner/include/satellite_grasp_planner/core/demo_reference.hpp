#pragma once

/**
 * @file demo_reference.hpp
 * @brief Reference quantities derived from the demonstration (currently the proxy of w_trans,demo).
 *        Eigen + Pinocchio only, NO ROS.
 */

#include <memory>

#include <Eigen/Dense>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"

namespace satellite_grasp_planner {
namespace core {

/// PROVISIONAL proxy of w_trans,demo: translational manipulability at the END of a replica of the
/// demo from the starting posture (orientation kept), i.e. the w_trans_final of
/// checkKinematicFeasibility(template, p_start + delta_g_demo, q_start, robot, q0). To be replaced
/// by the value recorded with the new demo.
double referenceWTrans(const haptic_dmp_learning::core::ProDMP& prodmp_template,
                       const std::shared_ptr<franka_cartesian_control::core::RobotModel>& robot,
                       const franka_cartesian_control::core::RobotModel::JointVector& q0,
                       const Eigen::Vector3d& delta_g_demo);

}  // namespace core
}  // namespace satellite_grasp_planner
