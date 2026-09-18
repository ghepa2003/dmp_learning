#pragma once

#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "franka_cartesian_control/core/velocity_ik_solver.hpp"

namespace satellite_grasp_planner {
namespace core {

using franka_cartesian_control::core::RobotModel;
using franka_cartesian_control::core::VelocityIkSolver;

/// @brief One sample of a desired Cartesian task-space trajectory (pose +
/// feedforward twist), expressed in the SAME frame as RobotModel's base
/// frame (fer_link0) - the frame the real CartesianVelocityController
/// resolves IK in, after FrameAligner. No frame alignment is performed here;
/// callers (PhaseSelector) are responsible for expressing candidates in that
/// frame already.
struct CartesianSample {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

/**
 * @brief Strategy interface for resolving kinematic redundancy (computing dq)
 *        during offline forward joint-path simulation.
 */
class RedundancyResolutionStrategy {
public:
    virtual ~RedundancyResolutionStrategy() = default;

    /**
     * @brief Computes commanded joint velocity dq for a simulation step.
     * @param model Updated robot model at current configuration q (FK + Jacobian).
     * @param target Desired Cartesian sample (target pose + feedforward twist).
     * @param q Current joint configuration.
     * @param q0 Initial / ready joint configuration (used as nullspace posture target).
     * @return Commanded joint velocity dq (7x1).
     */
    virtual RobotModel::JointVector computeJointVelocity(
        const RobotModel& model,
        const CartesianSample& target,
        const RobotModel::JointVector& q,
        const RobotModel::JointVector& q0) const = 0;
};

/**
 * @brief Implementation 1: Minimum-norm DLS resolution (Velocity Control proxy).
 *        dq = J^T (J J^T + lambda^2 I)^-1 * twist
 *        Reuses VelocityIkSolver verbatim; matches CartesianVelocityController.
 */
class MinNormDlsResolution : public RedundancyResolutionStrategy {
public:
    MinNormDlsResolution();
    explicit MinNormDlsResolution(const VelocityIkSolver::Params& ik_params);

    RobotModel::JointVector computeJointVelocity(
        const RobotModel& model,
        const CartesianSample& target,
        const RobotModel::JointVector& q,
        const RobotModel::JointVector& q0) const override;

    const VelocityIkSolver::Params& ikParams() const { return ik_params_; }

private:
    VelocityIkSolver::Params ik_params_;
    VelocityIkSolver ik_solver_;
};

/**
 * @brief Implementation 2: Nullspace-biased resolution (Impedance Control proxy).
 *        dq = dq_primary + N * K_ns * (q0 - q)
 *        where N = I - J^# J is the nullspace projector of the primary task.
 *        Matches CartesianImpedanceController's posture regulation behavior.
 */
class NullspaceBiasedResolution : public RedundancyResolutionStrategy {
public:
    struct Params {
        VelocityIkSolver::Params ik_params;  ///< Primary task DLS params (default lambda = 0.05, kp = 1.0)
        /// @brief Proportional nullspace convergence rate (s^-1).
        /// Physically derived from the impedance controller's critically-damped nullspace PD law:
        ///   tau_null = N^T * [ K_null * (q0 - q) - 2 * sqrt(K_null) * dq ]
        /// In 1st-order velocity kinematics (D * dq + K * (q - q0) = 0):
        ///   k_ns = K_null / D_null = K_null / (2 * sqrt(K_null)) = 0.5 * sqrt(K_null)
        /// For K_null = 0.2 Nm/rad: k_ns = 0.5 * sqrt(0.2) = 0.2236 s^-1.
        double nullspace_gain = 0.2236;

        /// @brief Multiplier for joint 1 yaw posture stiffness.
        /// Derived from joint1_nullspace_stiffness = 100 in torque control:
        ///   scale = sqrt(100) = 10.0.
        double joint1_nullspace_gain_scale = 10.0;

        /// @brief Damping lambda for nullspace projector pseudo-inverse (J^T)^#.
        double nullspace_damping = 0.05;
    };

    NullspaceBiasedResolution();
    explicit NullspaceBiasedResolution(const Params& params);

    RobotModel::JointVector computeJointVelocity(
        const RobotModel& model,
        const CartesianSample& target,
        const RobotModel::JointVector& q,
        const RobotModel::JointVector& q0) const override;

    const Params& params() const { return params_; }

private:
    Params params_;
    VelocityIkSolver ik_solver_;
};

/**
 * @brief Factory helper: creates the appropriate redundancy strategy from a controller name.
 * @param controller_name "cartesian_impedance_controller" / "impedance" -> NullspaceBiasedResolution
 *                        "velocity_cartesian_controller" / "cartesian_velocity_controller" / "velocity" -> MinNormDlsResolution
 * @return Shared pointer to the corresponding strategy.
 */
std::shared_ptr<RedundancyResolutionStrategy> createStrategyForController(
    const std::string& controller_name);

/**
 * @brief Offline forward simulation of the joint path q(t) that a Panda
 *        would follow along a candidate Cartesian trajectory.
 */
class JointPathSimulator {
public:
    struct Params {
        double dt = 0.005;  ///< integration step (s); matches control_rate_hz_=200 default
        std::shared_ptr<RedundancyResolutionStrategy> strategy;  ///< Redundancy resolution strategy
    };

    explicit JointPathSimulator(std::shared_ptr<RobotModel> robot_model);
    JointPathSimulator(std::shared_ptr<RobotModel> robot_model, const Params& params);

    /// @brief One simulated instant: joint configuration plus the
    /// translational manipulability at that configuration.
    struct Step {
        RobotModel::JointVector q;
        double w_trans = 0.0;
    };

    /**
     * @brief Simulates the joint path along a full Cartesian trajectory.
     * @param trajectory Time-ordered desired Cartesian samples (pose +
     *                   feedforward twist), sampled at Params::dt.
     * @param q0 Initial joint configuration seeding the Euler integration.
     * @return One Step per input sample, same length/order as trajectory;
     *         output[0].q == q0 (evaluated, not integrated).
     */
    std::vector<Step> simulate(const std::vector<CartesianSample>& trajectory,
                                const RobotModel::JointVector& q0) const;

    const Params& params() const { return params_; }

private:
    std::shared_ptr<RobotModel> robot_model_;
    Params params_;
};

}  // namespace core
}  // namespace satellite_grasp_planner
