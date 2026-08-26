#pragma once

#include <Eigen/Dense>
#include "franka_cartesian_control/core/robot_model.hpp"
#include "franka_cartesian_control/core/cartesian_error.hpp"

namespace franka_cartesian_control {
namespace core {

/**
 * @brief Resolved-Rate Cartesian Motion Controller using Damped Least Squares (DLS) Inverse Kinematics.
 *
 * @details
 * Control Law & Mathematical Background:
 * 1. Desired Spatial Twist (Task-Space Proportional Law):
 *    Given linear error e_p in R^3 and orientation error e_o in R^3:
 *      v_lin = K_p,lin * e_p
 *      v_ang = K_p,ang * e_o
 *    Both linear and angular velocity vectors are independently magnitude-saturated:
 *      v_lin = v_lin * min(1.0, max_linear_speed / ||v_lin||)
 *      v_ang = v_ang * min(1.0, max_angular_speed / ||v_ang||)
 *    Twist vector: V_des = [v_lin; v_ang] (6x1)
 *
 * 2. Damped Least Squares (DLS / Levenberg-Marquardt) Inverse:
 *    Near kinematic singularities, standard pseudo-inverse J^# = J^T (J J^T)^-1 becomes ill-conditioned,
 *    leading to dangerously high joint velocities. DLS regularizes the inversion by solving:
 *      min_{dq} ||J * dq - V_des||^2 + lambda^2 * ||dq||^2
 *    Analytic solution:
 *      dq = J^T * (J * J^T + lambda^2 * I_6)^-1 * V_des
 *
 * 3. Joint Velocity Saturation:
 *    Each component of dq is defensively clamped:
 *      dq_i = clamp(dq_i, -max_joint_speed, +max_joint_speed)
 */
class VelocityIkSolver {
public:
    using JointVector = RobotModel::JointVector;
    using Vector6d = Eigen::Matrix<double, 6, 1>;

    struct Params {
        double kp_linear = 1.0;          ///< Proportional gain for linear position error (1/s)
        double kp_angular = 1.0;         ///< Proportional gain for angular orientation error (1/s)
        double damping_lambda = 0.05;    ///< DLS singularity damping factor lambda
        double max_linear_speed = 0.15;  ///< Maximum task-space linear speed cap (m/s)
        double max_angular_speed = 0.5;  ///< Maximum task-space angular speed cap (rad/s)
        double max_joint_speed = 1.5;    ///< Per-joint actuator velocity limit (rad/s)
    };

    VelocityIkSolver();
    explicit VelocityIkSolver(const Params& params);

    /**
     * @brief Computes proportional desired twist [v; omega] from Cartesian pose error.
     * @param error Struct containing linear error e_p (m) and angular error e_o (rad).
     * @return Vector6d Saturated desired twist in base frame [v_x, v_y, v_z, w_x, w_y, w_z]^T.
     */
    Vector6d desiredTwist(const CartesianError& error) const;

    /**
     * @brief Computes joint velocity command dq using Damped Least Squares inversion.
     * @param jacobian Current 6x7 geometric Jacobian matrix J(q).
     * @param twist Desired 6D spatial twist V_des.
     * @return JointVector Commanded joint velocities dq (7x1, rad/s).
     */
    JointVector solve(const RobotModel::Jacobian6x7& jacobian, const Vector6d& twist) const;

    const Params& params() const { return params_; }
    void setParams(const Params& params) { params_ = params; }

private:
    Params params_;
};

}  // namespace core
}  // namespace franka_cartesian_control