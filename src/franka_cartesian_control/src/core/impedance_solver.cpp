#include "franka_cartesian_control/core/impedance_solver.hpp"
#include <algorithm>

namespace franka_cartesian_control {
namespace core {

CartesianImpedanceSolver::CartesianImpedanceSolver() : params_(Params()) {}

CartesianImpedanceSolver::CartesianImpedanceSolver(const Params& params) : params_(params) {}

CartesianImpedanceSolver::Matrix6d CartesianImpedanceSolver::stiffnessMatrix() const {
    // Construct a 6x6 Cartesian stiffness matrix K = diag([K_trans * I_3, K_rot * I_3])
    Matrix6d K = Matrix6d::Zero();
    K.topLeftCorner(3, 3) = params_.translational_stiffness * Eigen::Matrix3d::Identity();
    K.bottomRightCorner(3, 3) = params_.rotational_stiffness * Eigen::Matrix3d::Identity();
    return K;
}

CartesianImpedanceSolver::Matrix6d CartesianImpedanceSolver::dampingMatrix() const {
    // Construct a 6x6 Cartesian damping matrix D = diag([D_trans * I_3, D_rot * I_3])
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
        // Slew-rate clamping: prevent torque command delta from exceeding delta_tau_max per control loop cycle
        diff = std::max(-params_.delta_tau_max, std::min(params_.delta_tau_max, diff));
        tau_sat(i) = tau_prev(i) + diff;
    }
    return tau_sat;
}

CartesianImpedanceSolver::JointVector CartesianImpedanceSolver::computeTorque(
    const RobotModel& model, const CartesianError& err,
    const JointVector& q, const JointVector& dq, const JointVector& tau_prev) {

    const auto& J = model.jacobian();

    // 1. Pack Cartesian error: e = [e_p (meters); e_o (radians)] (target - current)
    Vector6d e;
    e.head<3>() = err.linear;
    e.tail<3>() = err.angular;

    // 2. Task-Space Impedance Torque: tau_task = J^T * (K * e - D * (J * dq))
    // Generates compliant restoring force proportional to Cartesian displacement and dissipative damping
    JointVector tau_task = J.transpose() * (stiffnessMatrix() * e - dampingMatrix() * (J * dq));

    // 3. Damped Nullspace Projection Matrix: N^T = I - J^T * (J^T)^#
    // Uses regularized pseudo-inverse (J^T)^# = (J * J^T + lambda^2 * I)^-1 * J
    Eigen::Matrix<double, 7, 7> JJt_pinv_proj;
    {
        Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
        JJt.diagonal().array() += params_.nullspace_pinv_damping * params_.nullspace_pinv_damping;
        Eigen::Matrix<double, 7, 6> J_transpose_pinv = J.transpose() * JJt.inverse();
        JJt_pinv_proj = Eigen::Matrix<double, 7, 7>::Identity() - J.transpose() * J_transpose_pinv.transpose();
    }

    // 4. Joint-Space Posture PD Control projected into the Nullspace
    JointVector qe = q_d_nullspace_ - q;
    qe(0) *= params_.joint1_nullspace_stiffness;
    JointVector dqe = dq;
    dqe(0) *= 2.0 * std::sqrt(params_.joint1_nullspace_stiffness);

    JointVector tau_nullspace = JJt_pinv_proj *
        (params_.nullspace_stiffness * qe - 2.0 * std::sqrt(params_.nullspace_stiffness) * dqe);
    tau_nullspace_last_ = tau_nullspace;  // cache for diagnostic leak analysis (RobotModel::massMatrix())

    // 5. Total Desired Torque: Task-Space + Nullspace + Coriolis [+ Gravity]
    JointVector tau_d = tau_task + tau_nullspace + model.coriolis();
    if (params_.compensate_gravity_internally) {
        tau_d += model.gravity();
    }

    // 6. Apply Slew-Rate Saturation for Actuator Safety
    return saturateTorqueRate(tau_d, tau_prev);
}

}  // namespace core
}  // namespace franka_cartesian_control