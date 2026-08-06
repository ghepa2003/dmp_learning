#include "velocity_cartesian_control/core/velocity_ik_solver.hpp"
#include <algorithm>

namespace velocity_cartesian_control {
namespace core {

VelocityIkSolver::VelocityIkSolver() : params_(Params()) {}
VelocityIkSolver::VelocityIkSolver(const Params& params) : params_(params) {}

// Computes the desired Cartesian twist from a pose error (proportional
// control law), saturating linear/angular speed independently.
VelocityIkSolver::Vector6d VelocityIkSolver::desiredTwist(const CartesianError& error) const {
    Eigen::Vector3d v_lin = params_.kp_linear * error.linear;
    Eigen::Vector3d v_ang = params_.kp_angular * error.angular;

    double lin_norm = v_lin.norm();
    if (lin_norm > params_.max_linear_speed && lin_norm > 1e-9) {
        v_lin *= (params_.max_linear_speed / lin_norm);
    }
    double ang_norm = v_ang.norm();
    if (ang_norm > params_.max_angular_speed && ang_norm > 1e-9) {
        v_ang *= (params_.max_angular_speed / ang_norm);
    }

    Vector6d twist;
    twist.head<3>() = v_lin;
    twist.tail<3>() = v_ang;
    return twist;
}

// Solves dq = J^T (J J^T + lambda^2 I)^-1 * twist, then clamps each
// joint velocity component to +/- max_joint_speed.
VelocityIkSolver::JointVector VelocityIkSolver::solve(
    const RobotModel::Jacobian6x7& jacobian, const Vector6d& twist) const {

    Eigen::Matrix<double, 6, 6> JJt = jacobian * jacobian.transpose();
    JJt.diagonal().array() += params_.damping_lambda * params_.damping_lambda;

    // dq = J^T (J J^T + lambda^2 I)^-1 * twist
    JointVector dq = jacobian.transpose() * JJt.ldlt().solve(twist);

    // Clamp each joint velocity to the specified maximum joint speed
    for (int i = 0; i < JointVector::RowsAtCompileTime; ++i) {
        dq(i) = std::max(-params_.max_joint_speed, std::min(params_.max_joint_speed, dq(i)));
    }
    return dq;
}

}  // namespace core
}  // namespace velocity_cartesian_control