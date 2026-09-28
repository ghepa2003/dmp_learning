#include "satellite_grasp_planner/core/candidate_eval.hpp"

#include <stdexcept>

#include "haptic_dmp_learning/core/grasp_roll.hpp"
#include "satellite_grasp_planner/core/kinematic_feasibility.hpp"
#include "satellite_grasp_planner/core/satellite_collision.hpp"

namespace satellite_grasp_planner {
namespace core {

using franka_cartesian_control::core::RobotModel;

CandidateEval evaluateCandidate(const haptic_dmp_learning::core::CubeSatelliteModel& model,
                                haptic_dmp_learning::core::GraspPointId k, double theta_rad,
                                double psi_rad, const haptic_dmp_learning::core::ProDMP& prodmp_template,
                                const std::shared_ptr<RobotModel>& robot_model,
                                const RobotModel::JointVector& q0, double d_safe_m,
                                double pos_tol_mm) {
    if (!robot_model) throw std::invalid_argument("evaluateCandidate: robot_model is null.");

    const auto target = model.graspPoseAt(k, theta_rad);
    const Eigen::Quaterniond target_orientation =
        haptic_dmp_learning::core::applyRoll(target.orientation_nominal_world, psi_rad);

    const FeasibilityResult kin = checkKinematicFeasibility(
        prodmp_template, target.position_world, target_orientation, robot_model, q0);

    // Explicit update to the final configuration: do not rely on the state left by the rollout.
    robot_model->update(kin.q_final, RobotModel::JointVector::Zero());
    const CollisionCheckResult col = checkSatelliteCollision(*robot_model, model, theta_rad, d_safe_m);

    CandidateEval ev;
    ev.kin_feasible = kin.feasible;
    ev.max_joint_violation_rad = kin.max_joint_violation_rad;
    ev.w_trans_final = kin.w_trans_final;
    ev.pos_err_final_mm = kin.pos_err_final_mm;
    ev.reached = kin.pos_err_final_mm < pos_tol_mm;
    ev.collision_feasible = col.feasible;
    ev.min_distance_arm_m = col.min_distance_arm_m;
    ev.min_distance_cube_m = col.min_distance_cube_m;
    ev.closest_capsule = col.closest_capsule;
    ev.closest_capsule_cube = col.closest_capsule_cube;
    ev.feasible = kin.feasible && ev.reached && col.feasible;
    return ev;
}

}  // namespace core
}  // namespace satellite_grasp_planner
