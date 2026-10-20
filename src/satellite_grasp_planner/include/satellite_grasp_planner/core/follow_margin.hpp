#pragma once

/**
 * @file follow_margin.hpp
 * @brief Predicted minimum joint margin over [start, AT_END + follow_s]: the ProDMP approach rollout continued by a
 *        simulated co-rotation of the grasp target with the satellite (the executor's post_grasp_follow).
 *
 * This is the procedure that produced the G16 / step 0c margins (_diag_1019/step0c_margins.cpp), operation for
 * operation. It is NOT the "margin_follow_rad" of analysis/s3_landscape.cpp (min of the rollout margin and the margins of
 * hot-started STATIC IK solutions at 7 samples of the follow arc, nullspace reference q_start): that one does not
 * integrate the controller and gives different numbers. Differences, all deliberate here:
 *   - the follow is SIMULATED (NullspaceBiasedResolution, explicit Euler, feed-forward twist of the rotating target),
 *     continuing from the joint state at the end of the approach, not re-solved by IK;
 *   - the nullspace reference of the whole simulation (approach AND follow) is q_ref (G16: readyPose), the seed is
 *     q_seed (G16: q_start); no hot-start chain.
 * Eigen + Pinocchio only, NO ROS.
 */

#include <memory>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"

namespace satellite_grasp_planner {
namespace core {

struct FollowMarginParams {
    double follow_s = 0.0;         ///< duration of the simulated follow after AT_END [s], finite and >= 0; the caller always sets it (G16: 3.0)
    double dt_approach_s = 0.005;  ///< integration step of the approach (as checkKinematicFeasibility and kDt of the selector)
    double dt_follow_s = 0.01;     ///< integration step of the follow (step0c_margins.cpp: 0.01; run_rollout_follow_all.cpp: 0.1)
};

struct FollowMarginResult {
    /// min over the sampled configurations and joints of min(q - lower, upper - q) [rad]; negative if a limit is exceeded.
    double min_margin_rad = 0.0;
    int joint = -1;  ///< limiting joint, 0-based (0 = fer_joint1); -1 if nothing was sampled
    /// Time of the limiting sample relative to AT_END [s]: (i + 1) dt - tau (<= 0) in the approach, t_follow in (0, follow_s] in
    /// the follow. Convention of the G16 diagnostics: the margin of the configuration BEFORE an integration step is labelled with
    /// the end time of that step. The configuration reached after the LAST follow step is not sampled (as in step0c_margins.cpp).
    double t_rel_s = 0.0;
    double approach_min_margin_rad = 0.0;  ///< same minimum restricted to the approach samples
    franka_cartesian_control::core::RobotModel::JointVector q_at_end =
        franka_cartesian_control::core::RobotModel::JointVector::Zero();  ///< joint state at AT_END (end of the approach)
    franka_cartesian_control::core::RobotModel::JointVector q_final =
        franka_cartesian_control::core::RobotModel::JointVector::Zero();  ///< joint state after the follow
};

/**
 * @brief Result of an approach simulation already done by the caller (checkKinematicFeasibility: q_end_post_step,
 *        min_joint_margin_rad, min_margin_joint, min_margin_step), so followMargin() need not repeat it. Valid only if the
 *        approach was simulated with the SAME seed, nullspace reference, target, dt_approach_s and ProDMP template as the
 *        followMargin() call it is given to (not checked).
 */
struct FollowApproachState {
    franka_cartesian_control::core::RobotModel::JointVector q_end =
        franka_cartesian_control::core::RobotModel::JointVector::Zero();  ///< joint state AFTER the last approach step
    double min_margin_rad = 0.0;  ///< approach minimum, as FollowMarginResult::approach_min_margin_rad
    int joint = -1;               ///< its joint, 0-based; -1 = nothing was sampled in the approach
    double t_rel_s = 0.0;         ///< its time relative to AT_END, (step + 1) dt - tau
};

/**
 * @brief Minimum joint margin of the approach followed by the co-rotating follow (see the file comment).
 *
 * Approach: buildApproachTrajectory(prodmp_template, FK(q_seed), target_position, target_orientation, dt_approach_s)
 * simulated with NullspaceBiasedResolution (default params) and nullspace reference @p q_ref. Follow:
 * round(follow_s / dt_follow_s) steps, step n at t = n dt_follow_s, the target is @p target_position/@p target_orientation
 * rotated by omega t about the axis through @p center (core::rotatePoseAboutZ), feed-forward linear velocity
 * (-omega r_y, omega r_x, 0) and angular velocity (0, 0, omega) with r = rotated position - center.
 *
 * @param omega_rad_s signed satellite angular velocity [rad/s], as SatelliteSnapshot::omega_rad_s
 * @param axis only +z is supported (the executor's rotatePoseAboutZ); @throws std::invalid_argument otherwise
 * @param robot_model shared, mutated (left at the final configuration); one instance per thread
 * @param approach optional: if given, the approach is NOT simulated again; the follow starts from approach->q_end and the
 *        approach minimum is taken from it. The result is then identical (bit for bit, see test_follow_margin) to the one
 *        without it. nullptr (default): the approach is simulated here.
 * @throws std::invalid_argument for a null model, non-positive dt, non-finite or negative follow_s, a non-finite
 *         omega, or a non-+z axis.
 */
FollowMarginResult followMargin(const haptic_dmp_learning::core::ProDMP& prodmp_template,
                                const Eigen::Vector3d& target_position, const Eigen::Quaterniond& target_orientation,
                                const Eigen::Vector3d& center, const Eigen::Vector3d& axis, double omega_rad_s,
                                const std::shared_ptr<franka_cartesian_control::core::RobotModel>& robot_model,
                                const franka_cartesian_control::core::RobotModel::JointVector& q_seed,
                                const franka_cartesian_control::core::RobotModel::JointVector& q_ref,
                                const FollowMarginParams& params, const FollowApproachState* approach = nullptr);

}  // namespace core
}  // namespace satellite_grasp_planner
