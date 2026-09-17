#include "franka_cartesian_control/core/velocity_ik_solver.hpp"
#include <algorithm>

namespace franka_cartesian_control {
namespace core {

VelocityIkSolver::VelocityIkSolver() : params_(Params()) {}
VelocityIkSolver::VelocityIkSolver(const Params& params) : params_(params) {}

VelocityIkSolver::Vector6d VelocityIkSolver::desiredTwist(
    const CartesianError& error, const Vector6d& feedforward) const {
    // 1. Proportional closed-loop law for linear and angular task-space errors,
    //    plus the (optionally zero) velocity feedforward: v_cmd = v_ff + Kp * e.
    Eigen::Vector3d v_lin = feedforward.head<3>() + params_.kp_linear * error.linear;
    Eigen::Vector3d v_ang = feedforward.tail<3>() + params_.kp_angular * error.angular;

    // 2. Magnitude saturation for linear velocity vector: preserves directional unit vector
    double lin_norm = v_lin.norm();
    if (lin_norm > params_.max_linear_speed && lin_norm > 1e-9) {
        v_lin *= (params_.max_linear_speed / lin_norm);
    }

    // 3. Magnitude saturation for angular velocity vector: preserves rotational axis
    double ang_norm = v_ang.norm();
    if (ang_norm > params_.max_angular_speed && ang_norm > 1e-9) {
        v_ang *= (params_.max_angular_speed / ang_norm);
    }

    // 4. Assemble 6D spatial twist vector V = [v_linear (3x1); omega_angular (3x1)]
    Vector6d twist;
    twist.head<3>() = v_lin;
    twist.tail<3>() = v_ang;
    return twist;
}

VelocityIkSolver::JointVector VelocityIkSolver::solve(
    const RobotModel::Jacobian6x7& jacobian, const Vector6d& twist) const {

    // 1. Formulate the regularized Gram matrix (J * J^T + lambda^2 * I_6) in task space
    Eigen::Matrix<double, 6, 6> JJt = jacobian * jacobian.transpose();
    JJt.diagonal().array() += params_.damping_lambda * params_.damping_lambda;

    // 2. Solve the linear system using LDLT decomposition (numerically stable for symmetric positive-definite matrices):
    //    dq = J^T * (J * J^T + lambda^2 * I_6)^-1 * twist
    JointVector dq = jacobian.transpose() * JJt.ldlt().solve(twist);

    // 3. Defense against joint-level actuator limits: clamp each velocity component to [-max_joint_speed, +max_joint_speed]
    for (int i = 0; i < JointVector::RowsAtCompileTime; ++i) {
        dq(i) = std::max(-params_.max_joint_speed, std::min(params_.max_joint_speed, dq(i)));
    }
    return dq;
}

}  // namespace core
}  // namespace franka_cartesian_control