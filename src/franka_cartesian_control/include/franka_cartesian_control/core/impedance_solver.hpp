#pragma once

#include <Eigen/Dense>
#include "franka_cartesian_control/core/robot_model.hpp"
#include "franka_cartesian_control/core/cartesian_error.hpp"

namespace franka_cartesian_control {
namespace core {

/**
 * @brief Cartesian Impedance Controller with Redundant Nullspace Projection.
 *
 * @details
 * Control Law & Mathematical Background:
 * 1. Task-Space Cartesian Impedance Law:
 *    The robot acts as a 6D virtual spring-damper system at the end-effector:
 *      F_task = K_x * e - D_x * dx
 *    where:
 *      e = [e_p; e_o] (6x1 pose error: target - current)
 *      dx = J * dq (6x1 current end-effector spatial twist)
 *      K_x = diag(K_trans * I_3, K_rot * I_3) (6x6 stiffness matrix)
 *      D_x = diag(D_trans * I_3, D_rot * I_3) (6x6 damping matrix)
 *    The equivalent joint torque is projected via the Jacobian transpose:
 *      tau_task = J^T * F_task = J^T * (K_x * e - D_x * (J * dq))
 *
 * 2. Redundant Nullspace Projection:
 *    The Franka arm has 7 actuated joints for 6 task-space DOFs (1 redundant DOF).
 *    To maintain posture stability and prevent joint drift without disturbing the Cartesian task,
 *    a secondary joint-space PD law is projected into the nullspace of J^T:
 *      N^T = I_7 - J^T * (J^T)^#
 *    where (J^T)^# is the regularized pseudo-inverse of J^T:
 *      (J^T)^# = (J * J^T + lambda^2 * I_6)^-1 * J
 *    Nullspace torque:
 *      tau_null = N^T * [ K_null * (q_null - q) - 2 * sqrt(K_null) * dq ]
 *    (Joint 1 gets additional stiffness scaling to resist base rotations as per SERL reference).
 *
 * 3. Dynamic Decoupling & Compensation:
 *    tau_d = tau_task + tau_null + c(q, dq) [+ g(q)]
 *    where c(q, dq) is Coriolis torque from Pinocchio RNEA, and g(q) is optional gravity torque.
 *
 * 4. Torque Slew-Rate Limiting:
 *    Protects robot gearboxes against high torque derivatives:
 *      tau_cmd = tau_prev + clamp(tau_d - tau_prev, -delta_tau_max, +delta_tau_max)
 */
class CartesianImpedanceSolver {
public:
    using JointVector = RobotModel::JointVector;
    using Matrix6d = Eigen::Matrix<double, 6, 6>;
    using Vector6d = Eigen::Matrix<double, 6, 1>;

    struct Params {
        // Task-space compliance gains (N/m and Nm/rad)
        double translational_stiffness = 2000.0;  ///< Cartesian linear stiffness (N/m)
        double rotational_stiffness = 150.0;      ///< Cartesian angular stiffness (Nm/rad)
        double translational_damping = 89.0;      ///< Cartesian linear damping (Ns/m)
        double rotational_damping = 7.0;          ///< Cartesian angular damping (Nms/rad)

        // Nullspace posture compliance gains
        double nullspace_stiffness = 0.2;          ///< Uniform nullspace stiffness (Nm/rad)
        double joint1_nullspace_stiffness = 100.0; ///< Pre-multiplier for joint 1 nullspace stiffness
        double nullspace_pinv_damping = 0.05;      ///< Regularization lambda for (J^T)^# pseudo-inverse

        // Actuator safety
        double delta_tau_max = 1.0;                ///< Maximum torque rate limit per cycle (Nm/cycle)
        bool compensate_gravity_internally = false; ///< True if gravity torque should be added explicitly
    };

    CartesianImpedanceSolver();
    explicit CartesianImpedanceSolver(const Params& params);

    /// @brief Sets desired joint equilibrium configuration for nullspace posture control.
    void setNullspaceTarget(const JointVector& q_d_nullspace) { q_d_nullspace_ = q_d_nullspace; }
    const JointVector& nullspaceTarget() const { return q_d_nullspace_; }

    /// @brief Returns the nullspace joint torque computed during the last computeTorque() call.
    /// Diagnostic accessor only — does not affect the commanded torque output.
    const JointVector& lastNullspaceTorque() const { return tau_nullspace_last_; }

    /**
     * @brief Computes commanded joint torques for the current control cycle.
     * @param model Updated robot kinematic and dynamic model.
     * @param err Cartesian pose error (target - current).
     * @param q Current joint positions (rad).
     * @param dq Current joint velocities (rad/s).
     * @param tau_prev Commanded torque from previous cycle (for rate saturation).
     * @return JointVector Commanded joint torque vector (Nm, 7x1).
     */
    JointVector computeTorque(const RobotModel& model, const CartesianError& err,
                               const JointVector& q, const JointVector& dq,
                               const JointVector& tau_prev);

    const Params& params() const { return params_; }
    void setParams(const Params& p) { params_ = p; }

private:
    Params params_;
    JointVector q_d_nullspace_ = JointVector::Zero();
    JointVector tau_nullspace_last_ = JointVector::Zero();

    Matrix6d stiffnessMatrix() const;
    Matrix6d dampingMatrix() const;
    JointVector saturateTorqueRate(const JointVector& tau_calc, const JointVector& tau_prev) const;
};

}  // namespace core
}  // namespace franka_cartesian_control