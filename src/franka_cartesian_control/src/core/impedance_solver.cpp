#include "franka_cartesian_control/core/impedance_solver.hpp"
#include <algorithm>

namespace franka_cartesian_control {
namespace core {

CartesianImpedanceSolver::CartesianImpedanceSolver() : params_(Params()) {}

CartesianImpedanceSolver::CartesianImpedanceSolver(const Params& params) : params_(params) {}

CartesianImpedanceSolver::Matrix6d CartesianImpedanceSolver::stiffnessMatrix() const {
    Matrix6d K = Matrix6d::Zero();
    K.topLeftCorner(3, 3) = params_.translational_stiffness * Eigen::Matrix3d::Identity();
    K.bottomRightCorner(3, 3) = params_.rotational_stiffness * Eigen::Matrix3d::Identity();
    return K;
}

CartesianImpedanceSolver::Matrix6d CartesianImpedanceSolver::dampingMatrix() const {
    Matrix6d D = Matrix6d::Zero();
    D.topLeftCorner(3, 3) = params_.translational_damping * Eigen::Matrix3d::Identity();
    D.bottomRightCorner(3, 3) = params_.rotational_damping * Eigen::Matrix3d::Identity();
    return D;
}

CartesianImpedanceSolver::JointVector CartesianImpedanceSolver::saturateTorqueRate(
    const JointVector& tau_calc, const JointVector& tau_prev) const {
    JointVector tau_sat;
    for (int i = 0; i < JointVector::RowsAtCompileTime; ++i) {
        double diff = tau_calc(i) - tau_prev(i);
        diff = std::max(-params_.delta_tau_max, std::min(params_.delta_tau_max, diff));
        tau_sat(i) = tau_prev(i) + diff;
    }
    return tau_sat;
}

CartesianImpedanceSolver::JointVector CartesianImpedanceSolver::computeTorque(
    const RobotModel& model, const CartesianError& err,
    const JointVector& q, const JointVector& dq, const JointVector& tau_prev) {

    const auto& J = model.jacobian();

    // Cartesian error, our convention: err = target - current (see
    // cartesian_error.hpp). SERL's error_ is the opposite sign
    // (current - target), and its law is tau_task = J^T(-K*error - D*Jdq).
    // With our sign flipped, the K term flips too: tau_task = J^T(K*e - D*Jdq).
    Vector6d e;
    e.head<3>() = err.linear;
    e.tail<3>() = err.angular;

    JointVector tau_task = J.transpose() * (stiffnessMatrix() * e - dampingMatrix() * (J * dq));

    // Nullspace projection (I - J^T (J^T)^+), damped pseudo-inverse of J^T
    // for robustness near singularities (deviation from SERL, see Params).
    Eigen::Matrix<double, 7, 7> JJt_pinv_proj;
    {
        Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
        JJt.diagonal().array() += params_.nullspace_pinv_damping * params_.nullspace_pinv_damping;
        Eigen::Matrix<double, 7, 6> J_transpose_pinv = J.transpose() * JJt.inverse();
        JJt_pinv_proj = Eigen::Matrix<double, 7, 7>::Identity() - J.transpose() * J_transpose_pinv.transpose();
    }
    // NOTE: J_transpose_pinv is the damped pseudo-inverse of J^T, not J.

    JointVector qe = q_d_nullspace_ - q;
    qe(0) *= params_.joint1_nullspace_stiffness;
    JointVector dqe = dq;
    dqe(0) *= 2.0 * std::sqrt(params_.joint1_nullspace_stiffness);

    JointVector tau_nullspace = JJt_pinv_proj *
        (params_.nullspace_stiffness * qe - 2.0 * std::sqrt(params_.nullspace_stiffness) * dqe);

    JointVector tau_d = tau_task + tau_nullspace + model.coriolis();
    if (params_.compensate_gravity_internally) {
        tau_d += model.gravity();
    }

    return saturateTorqueRate(tau_d, tau_prev);
}

}  // namespace core
}  // namespace franka_cartesian_control