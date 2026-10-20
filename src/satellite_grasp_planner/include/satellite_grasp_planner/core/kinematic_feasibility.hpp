#pragma once

#include <memory>
#include <optional>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "satellite_grasp_planner/core/approach_clearance.hpp"
#include "satellite_grasp_planner/core/joint_path_simulator.hpp"

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
    /// min over all simulated Steps and joints of min(q - lower, upper - q) [rad]; negative if a limit is
    /// exceeded. DIAGNOSTIC ONLY (does not influence `feasible`); 0.0 if nothing was simulated.
    double min_joint_margin_rad = 0.0;
    /// Joint (0-6, fer_joint1..7) that gives min_joint_margin_rad (first one on ties); -1 if nothing was simulated.
    /// Unlike worst_joint it is set also when there is no violation. DIAGNOSTIC ONLY.
    int min_margin_joint = -1;
    /// Simulated Step index of min_joint_margin_rad (first one on ties); -1 if nothing was simulated. The time of that sample
    /// relative to the end of the approach is (min_margin_step + 1) dt - tau. DIAGNOSTIC ONLY.
    int min_margin_step = -1;
    /// Joint configuration AFTER the last integration step of the approach (q_final is the one BEFORE it). It is the exact
    /// state at AT_END from which a follow simulation continues (the resolution strategy has no internal state). Zero if nothing
    /// was simulated. Appended last: field order of the earlier members is unchanged.
    franka_cartesian_control::core::RobotModel::JointVector q_end_post_step =
        franka_cartesian_control::core::RobotModel::JointVector::Zero();
};

/**
 * @brief The Cartesian approach trajectory of checkKinematicFeasibility(), as its own function (the loop that used to
 *        be inline there, operation for operation): ProDMP position (absolute goal, ceil(tau/dt) steps from
 *        @p p0, goal @p target_position) and shortest-path SLERP @p quat0 -> @p target_orientation with
 *        s = min(1, (i + 1) dt / tau). Sample i belongs to the time (i + 1) dt.
 * @param prodmp_template NOT modified (a local copy is rolled out). @throws std::invalid_argument if dt <= 0.
 */
std::vector<CartesianSample> buildApproachTrajectory(const haptic_dmp_learning::core::ProDMP& prodmp_template,
                                                     const Eigen::Vector3d& p0, const Eigen::Quaterniond& quat0,
                                                     const Eigen::Vector3d& target_position,
                                                     const Eigen::Quaterniond& target_orientation, double dt);

/**
 * @brief buildApproachTrajectory() with explicit times, as the executor publishes it on /target_pose: a first sample
 *        (t = 0, @p p0, @p quat0) - the executor's first tick has elapsed = 0 - then sample i at t = (i + 1) dt, with
 *        the position clamped to @p target_position from t >= tau on (the executor publishes the goal once
 *        elapsed >= tau). Input of approachClearance().
 */
std::vector<ApproachSample> timedApproachTrajectory(const haptic_dmp_learning::core::ProDMP& prodmp_template,
                                                    const Eigen::Vector3d& p0, const Eigen::Quaterniond& quat0,
                                                    const Eigen::Vector3d& target_position,
                                                    const Eigen::Quaterniond& target_orientation, double dt);

/**
 * @brief Offline kinematic feasibility of reaching a complete target pose from @p q0.
 *
 * ProDMP position rollout (absolute goal, ceil(tau/dt)
 * steps) + shortest-path SLERP of the orientation from the FK pose at q0 to @p target_orientation,
 * simulated with JointPathSimulator (NullspaceBiasedResolution) and checked against the joint limits
 * of @p robot_model. It knows nothing about grasp points, phase or roll: the caller supplies the pose.
 *
 * @param q_ref Nullspace posture target of the simulated controller. Default (nullopt) = @p q0, i.e. the behaviour
 *              before this argument existed (bit-identical). Set it to decouple the seed from the target (the real
 *              impedance controller holds the nullspace target at its activation pose, not at the planner's q_start).
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
    double max_joint_violation_tol_rad = 1e-4,
    const std::optional<franka_cartesian_control::core::RobotModel::JointVector>& q_ref = std::nullopt);

}  // namespace core
}  // namespace satellite_grasp_planner
