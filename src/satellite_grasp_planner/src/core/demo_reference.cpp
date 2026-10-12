#include "satellite_grasp_planner/core/demo_reference.hpp"

#include <stdexcept>

#include "satellite_grasp_planner/core/kinematic_feasibility.hpp"

namespace satellite_grasp_planner {
namespace core {

using franka_cartesian_control::core::RobotModel;

double referenceWTrans(const haptic_dmp_learning::core::ProDMP& prodmp_template,
                       const std::shared_ptr<RobotModel>& robot, const RobotModel::JointVector& q0,
                       const Eigen::Vector3d& delta_g_demo) {
    if (!robot) throw std::invalid_argument("referenceWTrans: robot is null.");
    robot->update(q0, RobotModel::JointVector::Zero());
    const Eigen::Vector3d p_start = robot->eePosition();
    const Eigen::Quaterniond q_start = robot->eeOrientation();
    const FeasibilityResult kin =
        checkKinematicFeasibility(prodmp_template, p_start + delta_g_demo, q_start, robot, q0);
    return kin.w_trans_final;
}

}  // namespace core
}  // namespace satellite_grasp_planner
