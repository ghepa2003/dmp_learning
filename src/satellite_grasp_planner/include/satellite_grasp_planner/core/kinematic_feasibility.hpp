#pragma once

#include <memory>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"

namespace satellite_grasp_planner {
namespace core {

struct FeasibilityResult {
    bool feasible = false;
    double max_joint_violation_rad = 0.0;  ///< largest excursion outside the URDF joint limits over the whole path
    /// FK position error of the last configuration vs target. DIAGNOSTIC ONLY: it does not
    /// influence `feasible` (see checkKinematicFeasibility()).
    double pos_err_final_mm = 0.0;
    /// Joint configuration at the last simulated Step (left default-zero if nothing was simulated).
    franka_cartesian_control::core::RobotModel::JointVector q_final =
        franka_cartesian_control::core::RobotModel::JointVector::Zero();
    /// Step::w_trans (translational manipulability) of the last simulated Step.
    double w_trans_final = 0.0;
    /// Diagnostics of the WORST joint-limit violation (the one giving max_joint_violation_rad).
    /// All stay at their defaults when there is no violation.
    int worst_joint = -1;                ///< joint index 0-6 (fer_joint1..7)
    int worst_step = -1;                 ///< simulated Step index of the worst violation
    double worst_step_fraction = 0.0;    ///< worst_step / (n_steps - 1)
    double worst_joint_value_rad = 0.0;  ///< q of that joint at that Step
    double worst_joint_limit_rad = 0.0;  ///< the limit that was exceeded
};

/**
 * @brief Offline kinematic feasibility of reaching a complete target pose from @p q0.
 *
 * Same mechanism as tools/cpp_sweep_free_roll.cpp: ProDMP position rollout (absolute goal, ceil(tau/dt)
 * steps) + shortest-path SLERP of the orientation from the FK pose at q0 to @p target_orientation,
 * simulated with JointPathSimulator (NullspaceBiasedResolution) and checked against the joint limits
 * of @p robot_model. It knows nothing about grasp points, phase or roll: the caller supplies the pose.
 *
 * @param prodmp_template Learned ProDMP; NOT modified (a local copy is rolled out, since step() mutates).
 * @param robot_model Shared (not const) because JointPathSimulator holds a shared_ptr and update() mutates
 *                    the model's cached FK/Jacobian; its cached state is left at the final configuration.
 *                    Do not call concurrently on the same instance (use one RobotModel per thread).
 * @return feasible == (max_joint_violation_rad < max_joint_violation_tol_rad). The final position
 *         error is reported in pos_err_final_mm but is deliberately NOT part of the criterion:
 *         JointPathSimulator integrates with explicit Euler and a proportional Kp feedback on the
 *         Cartesian error (common to both RedundancyResolutionStrategy implementations), which
 *         adds a tracking-lag artefact (measured 5-7 mm on real cases, not reducible by choosing
 *         the other strategy) that the real torque-level Cartesian Impedance Controller does not
 *         have (validated separately on Gazebo). It is therefore not a reliable proxy for the
 *         real accuracy and must not block feasibility.
 */
FeasibilityResult checkKinematicFeasibility(
    const haptic_dmp_learning::core::ProDMP& prodmp_template,
    const Eigen::Vector3d& target_position,
    const Eigen::Quaterniond& target_orientation,
    const std::shared_ptr<franka_cartesian_control::core::RobotModel>& robot_model,
    const franka_cartesian_control::core::RobotModel::JointVector& q0 =
        franka_cartesian_control::core::RobotModel::readyPose(),
    double dt = 0.005,
    double max_joint_violation_tol_rad = 1e-4);

}  // namespace core
}  // namespace satellite_grasp_planner
