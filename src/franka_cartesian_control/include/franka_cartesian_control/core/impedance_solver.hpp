#pragma once

#include <Eigen/Dense>
#include "franka_cartesian_control/core/robot_model.hpp"
#include "franka_cartesian_control/core/cartesian_error.hpp"

namespace franka_cartesian_control {
namespace core {

// Cartesian impedance control law, structurally following the SERL
// reference (task-space impedance + nullspace projection), with
// RobotModel/Pinocchio replacing franka_hw::FrankaModelInterface.
// Gravity compensation is optional (Gazebo adds it automatically in
// effort mode - verified in ign_system.cpp write()); Coriolis is always
// included, since Gazebo does NOT auto-compensate it.
class CartesianImpedanceSolver {
public:
    using JointVector = RobotModel::JointVector;
    using Matrix6d = Eigen::Matrix<double, 6, 6>;
    using Vector6d = Eigen::Matrix<double, 6, 1>;

    struct Params {
        // Task-space gains (SERL-style: independent stiffness/damping,
        // not auto-derived as critical damping - matches the reference
        // exactly, tuning is manual as in the original).
        double translational_stiffness = 2000.0;  // N/m — SERL default (compliance_param.cfg)
        double rotational_stiffness = 150.0;      // Nm/rad — SERL default
        double translational_damping = 89.0;      // Ns/m — SERL default
        double rotational_damping = 7.0;          // Nms/rad — SERL default

        // Nullspace gains - SERL applies an idiosyncratic extra weight on
        // joint1 specifically (base rotation), replicated here faithfully:
        // joint1's nullspace error is pre-scaled by joint1_nullspace_stiffness
        // BEFORE the uniform nullspace_stiffness multiplication, so joint1
        // effectively gets stiffness = nullspace_stiffness * joint1_nullspace_stiffness.
        double nullspace_stiffness = 0.2;         // SERL default (nota: molto più basso del previsto)
        double joint1_nullspace_stiffness = 100.0; // SERL default

        // Torque rate limiting (SERL: delta_tau_max_ = 1.0, hardcoded const;
        // exposed here as a tunable parameter instead).
        double delta_tau_max = 1.0;  // Nm per control cycle

        // Damping factor for the nullspace-projection pseudo-inverse of J^T.
        // DEVIATION FROM SERL: the reference uses a raw (undamped)
        // pseudo-inverse here, which can blow up near singularities - same
        // risk we found and fixed for VelocityIkSolver. Damped for
        // consistency/robustness; set to 0 to recover SERL's exact behavior.
        double nullspace_pinv_damping = 0.05;

        // Gazebo already adds gravity automatically in effort mode
        // (verified in ign_system.cpp). Coriolis is NEVER auto-compensated
        // there, so it is always included regardless of this flag.
        bool compensate_gravity_internally = false;
    };

    CartesianImpedanceSolver();
    explicit CartesianImpedanceSolver(const Params& params);

    // Desired nullspace joint configuration (SERL: q_d_nullspace_, captured
    // from the initial configuration in starting() - here left to the
    // caller, typically set once in on_activate()).
    void setNullspaceTarget(const JointVector& q_d_nullspace) { q_d_nullspace_ = q_d_nullspace; }
    const JointVector& nullspaceTarget() const { return q_d_nullspace_; }

    // Computes the commanded joint torque for this cycle. tau_prev is the
    // torque commanded in the previous cycle (needed for rate saturation,
    // same as SERL's tau_J_d).
    JointVector computeTorque(const RobotModel& model, const CartesianError& err,
                               const JointVector& q, const JointVector& dq,
                               const JointVector& tau_prev);

    const Params& params() const { return params_; }
    void setParams(const Params& p) { params_ = p; }

private:
    Params params_;
    JointVector q_d_nullspace_ = JointVector::Zero();

    Matrix6d stiffnessMatrix() const;
    Matrix6d dampingMatrix() const;
    JointVector saturateTorqueRate(const JointVector& tau_calc, const JointVector& tau_prev) const;
};

}  // namespace core
}  // namespace franka_cartesian_control