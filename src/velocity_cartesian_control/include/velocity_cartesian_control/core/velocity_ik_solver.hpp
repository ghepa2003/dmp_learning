#pragma once

#include <Eigen/Dense>
#include "velocity_cartesian_control/core/robot_model.hpp"
#include "velocity_cartesian_control/core/cartesian_error.hpp"

namespace velocity_cartesian_control {
namespace core {

// Resolved-rate motion control: converts a desired Cartesian pose error into
// joint velocity commands via damped least squares (DLS) pseudo-inverse of
// the Jacobian. Deliberately stateless/parametrized rather than hard-coded -
// gains and damping are all tunable, none verified experimentally yet.
class VelocityIkSolver {
public:
    using JointVector = RobotModel::JointVector;
    using Vector6d = Eigen::Matrix<double, 6, 1>;

    struct Params {
        double kp_linear = 1.0;     // 1/s, proportional gain on position error
        double kp_angular = 1.0;    // 1/s, proportional gain on orientation error
        double damping_lambda = 0.05;   // DLS damping factor
        double max_linear_speed = 0.15;   // m/s, cap on desired cartesian linear speed
        double max_angular_speed = 0.5;   // rad/s, cap on desired cartesian angular speed
        double max_joint_speed = 1.5;     // rad/s, per-joint cap on the DLS output
    };

    VelocityIkSolver();
    explicit VelocityIkSolver(const Params& params);

    // Computes the desired Cartesian twist from a pose error (proportional
    // control law), saturating linear/angular speed independently.
    Vector6d desiredTwist(const CartesianError& error) const;

    // Solves dq = J^T (J J^T + lambda^2 I)^-1 * twist, then clamps each
    // joint velocity component to +/- max_joint_speed.
    JointVector solve(const RobotModel::Jacobian6x7& jacobian, const Vector6d& twist) const;

    const Params& params() const { return params_; }
    void setParams(const Params& params) { params_ = params; }

private:
    Params params_;
};

}  // namespace core
}  // namespace velocity_cartesian_control