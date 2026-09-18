#include "satellite_grasp_planner/core/joint_path_simulator.hpp"

#include <stdexcept>
#include "franka_cartesian_control/core/cartesian_error.hpp"
#include "satellite_grasp_planner/core/manipulability.hpp"

namespace satellite_grasp_planner {
namespace core {

// ---------------------------------------------------------------------
// MinNormDlsResolution (Implementation 1)
// ---------------------------------------------------------------------
MinNormDlsResolution::MinNormDlsResolution()
    : MinNormDlsResolution(VelocityIkSolver::Params()) {}

MinNormDlsResolution::MinNormDlsResolution(const VelocityIkSolver::Params& ik_params)
    : ik_params_(ik_params), ik_solver_(ik_params) {}

RobotModel::JointVector MinNormDlsResolution::computeJointVelocity(
    const RobotModel& model,
    const CartesianSample& target,
    const RobotModel::JointVector& /* q */,
    const RobotModel::JointVector& /* q0 */) const {
    franka_cartesian_control::core::CartesianError err =
        franka_cartesian_control::core::computePoseError(
            model.eePosition(), model.eeOrientation(),
            target.position, target.orientation);

    VelocityIkSolver::Vector6d feedforward;
    feedforward.head<3>() = target.linear_velocity;
    feedforward.tail<3>() = target.angular_velocity;

    auto twist = ik_solver_.desiredTwist(err, feedforward);
    return ik_solver_.solve(model.jacobian(), twist);
}

// ---------------------------------------------------------------------
// NullspaceBiasedResolution (Implementation 2)
// ---------------------------------------------------------------------
NullspaceBiasedResolution::NullspaceBiasedResolution()
    : NullspaceBiasedResolution(Params()) {}

NullspaceBiasedResolution::NullspaceBiasedResolution(const Params& params)
    : params_(params), ik_solver_(params.ik_params) {}

RobotModel::JointVector NullspaceBiasedResolution::computeJointVelocity(
    const RobotModel& model,
    const CartesianSample& target,
    const RobotModel::JointVector& q,
    const RobotModel::JointVector& q0) const {
    // 1. Primary task: pose error -> desired twist -> DLS minimum-norm joint velocity
    franka_cartesian_control::core::CartesianError err =
        franka_cartesian_control::core::computePoseError(
            model.eePosition(), model.eeOrientation(),
            target.position, target.orientation);

    VelocityIkSolver::Vector6d feedforward;
    feedforward.head<3>() = target.linear_velocity;
    feedforward.tail<3>() = target.angular_velocity;

    auto twist = ik_solver_.desiredTwist(err, feedforward);
    auto dq_primary = ik_solver_.solve(model.jacobian(), twist);

    // 2. Damped nullspace projector: N = I - J^# J, where J^# = J^T (J J^T + lambda_ns^2 I)^-1
    const auto& J = model.jacobian();
    Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
    JJt.diagonal().array() += params_.nullspace_damping * params_.nullspace_damping;
    Eigen::Matrix<double, 7, 6> J_pinv = J.transpose() * JJt.inverse();
    Eigen::Matrix<double, 7, 7> N = Eigen::Matrix<double, 7, 7>::Identity() - J_pinv * J;

    // 3. Nullspace posture regulation: pull toward ready pose q0
    // Kinematic conversion: nullspace_gain in s^-1 acts as proportional rate
    // toward q0 in the nullspace of the primary task.
    RobotModel::JointVector qe = q0 - q;
    qe(0) *= params_.joint1_nullspace_gain_scale;

    RobotModel::JointVector dq_ns = N * (params_.nullspace_gain * qe);

    return dq_primary + dq_ns;
}

// ---------------------------------------------------------------------
// Factory Helper
// ---------------------------------------------------------------------
std::shared_ptr<RedundancyResolutionStrategy> createStrategyForController(
    const std::string& controller_name) {
    if (controller_name == "cartesian_impedance_controller" || controller_name == "impedance") {
        return std::make_shared<NullspaceBiasedResolution>();
    }
    if (controller_name == "velocity_cartesian_controller" ||
        controller_name == "cartesian_velocity_controller" ||
        controller_name == "velocity") {
        return std::make_shared<MinNormDlsResolution>();
    }
    throw std::invalid_argument(
        "createStrategyForController: unknown controller_name '" + controller_name +
        "'. Expected 'cartesian_impedance_controller' or 'velocity_cartesian_controller'.");
}

// ---------------------------------------------------------------------
// JointPathSimulator
// ---------------------------------------------------------------------
JointPathSimulator::JointPathSimulator(std::shared_ptr<RobotModel> robot_model)
    : JointPathSimulator(std::move(robot_model), Params()) {}

JointPathSimulator::JointPathSimulator(std::shared_ptr<RobotModel> robot_model,
                                         const Params& params)
    : robot_model_(std::move(robot_model)), params_(params) {
    if (!params_.strategy) {
        params_.strategy = std::make_shared<NullspaceBiasedResolution>();
    }
}

std::vector<JointPathSimulator::Step> JointPathSimulator::simulate(
    const std::vector<CartesianSample>& trajectory, const RobotModel::JointVector& q0) const {
    std::vector<Step> steps;
    steps.reserve(trajectory.size());
    if (trajectory.empty()) {
        return steps;
    }

    RobotModel::JointVector q = q0;
    const RobotModel::JointVector dq_zero = RobotModel::JointVector::Zero();

    for (size_t i = 0; i < trajectory.size(); ++i) {
        // 1. FK + Jacobian at the current (already-integrated) configuration.
        robot_model_->update(q, dq_zero);

        Step step;
        step.q = q;
        step.w_trans = wTransFromJacobian(robot_model_->jacobian().topRows<3>());
        steps.push_back(step);

        // 2. Delegate redundancy resolution to the configured strategy.
        const CartesianSample& target = trajectory[i];
        auto dq_cmd = params_.strategy->computeJointVelocity(*robot_model_, target, q, q0);

        // 3. Explicit-Euler forward integration of the joint path.
        q += dq_cmd * params_.dt;
    }

    return steps;
}

}  // namespace core
}  // namespace satellite_grasp_planner
