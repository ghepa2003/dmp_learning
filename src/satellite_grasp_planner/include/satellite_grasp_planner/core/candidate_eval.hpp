#pragma once

/**
 * @file candidate_eval.hpp
 * @brief Full evaluation of one grasp candidate (k, theta, psi): kinematic feasibility of the
 *        rollout + arm/satellite collision at the final configuration.
 */

#include <memory>
#include <string>

#include <Eigen/Dense>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"

namespace satellite_grasp_planner {
namespace core {

struct CandidateEval {
    bool kin_feasible = false;
    double max_joint_violation_rad = 0.0;
    double w_trans_final = 0.0;
    double pos_err_final_mm = 0.0;
    /// pos_err_final_mm < pos_tol_mm. Only meant to recognize an arm that did NOT reach the grasp
    /// pose (see evaluateCandidate()).
    bool reached = false;
    /// NOTE: if !reached the arm is not at the grasp pose, so the collision fields below (and
    /// collision_feasible) refer to a configuration that is not the grasp: they are kept in the
    /// result but are NOT meaningful.
    bool collision_feasible = false;
    double min_distance_arm_m = 0.0;   ///< to the cylinder (see checkSatelliteCollision)
    double min_distance_cube_m = 0.0;  ///< to the cube body
    std::string closest_capsule;
    std::string closest_capsule_cube;
    bool feasible = false;             ///< kin_feasible && reached && collision_feasible
};

/**
 * @brief Evaluates grasp point @p k at satellite phase @p theta_rad with roll @p psi_rad.
 *
 * Target position = CubeSatelliteModel::graspPoseAt(k, theta).position_world, with NO contact-to-end
 * offset for now. Target orientation = applyRoll(orientation_nominal_world, psi_rad). Then
 * checkKinematicFeasibility(prodmp_template, ...) from @p q0; the robot model is then EXPLICITLY
 * updated to the final configuration (zero velocity), not relying on residual state, and
 * checkSatelliteCollision(*robot_model, model, theta_rad, d_safe_m) is run on it.
 *
 * Position criterion: checkKinematicFeasibility() deliberately excludes the final position error
 * (the simulator's Kp tracking lag, 7-11 mm on reached cases, is not a precision proxy). Here
 * it is used ONLY to recognize an arm that did not reach the pose: reached = pos_err_final_mm <
 * @p pos_tol_mm. In the feasibility-map probe the reached candidates lay between 7.5 and 15.1 mm
 * and the unreached ones between 22 and 96 mm; 20 mm is PROVISIONAL, derived from those ~30 samples.
 * If !reached, the collision distances describe a configuration that is not the grasp pose and
 * are not meaningful (they are still returned).
 *
 * Cost and threading: MEASURED 2026-09-29, dev laptop, Release (-O3): production selection = 66
 * rollouts + scan in 5.2 s (at most ~80 ms per rollout). A build without optimization flags is far
 * slower: ~7.6 s per rollout / ~500 s per selection were reported on 2026-09-28 (build type not
 * recorded at the time). RobotModel is NOT thread-safe (update() mutates its cached FK/Jacobian),
 * so every thread must use its own instance.
 */
CandidateEval evaluateCandidate(
    const haptic_dmp_learning::core::CubeSatelliteModel& model,
    haptic_dmp_learning::core::GraspPointId k, double theta_rad, double psi_rad,
    const haptic_dmp_learning::core::ProDMP& prodmp_template,
    const std::shared_ptr<franka_cartesian_control::core::RobotModel>& robot_model,
    const franka_cartesian_control::core::RobotModel::JointVector& q0, double d_safe_m = 0.010,
    double pos_tol_mm = 20.0);

}  // namespace core
}  // namespace satellite_grasp_planner
