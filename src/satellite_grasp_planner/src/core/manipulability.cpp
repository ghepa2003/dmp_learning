#include "satellite_grasp_planner/core/manipulability.hpp"

namespace satellite_grasp_planner {
namespace core {

Manipulability::Manipulability(
    std::shared_ptr<franka_cartesian_control::core::RobotModel> robot_model)
    : robot_model_(std::move(robot_model)) {}

double Manipulability::wTrans(
    const franka_cartesian_control::core::RobotModel::JointVector& q) const {
    const franka_cartesian_control::core::RobotModel::JointVector dq_zero =
        franka_cartesian_control::core::RobotModel::JointVector::Zero();
    robot_model_->update(q, dq_zero);
    return wTransFromJacobian(robot_model_->jacobian().topRows<3>());
}

}  // namespace core
}  // namespace satellite_grasp_planner
