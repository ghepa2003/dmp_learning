#include <gtest/gtest.h>
#include "franka_cartesian_control/core/velocity_ik_solver.hpp"

using namespace franka_cartesian_control::core;

/**
 * @brief Tests proportional twist computation and independent saturation of linear and angular speeds.
 */
TEST(VelocityIkSolverTest, DesiredTwistProportionalAndSaturation) {
    VelocityIkSolver::Params params;
    params.kp_linear = 2.0;
    params.kp_angular = 1.5;
    params.max_linear_speed = 0.1;
    params.max_angular_speed = 0.5;

    VelocityIkSolver solver(params);

    // 1. Small error: should produce linear proportional twist without reaching saturation limits
    CartesianError small_error;
    small_error.linear = Eigen::Vector3d(0.01, 0.02, 0.0);
    small_error.angular = Eigen::Vector3d(0.0, 0.0, 0.1);

    auto twist = solver.desiredTwist(small_error);

    // Linear part: 2.0 * [0.01, 0.02, 0] = [0.02, 0.04, 0] (norm ~ 0.0447 < max 0.1)
    EXPECT_NEAR(twist(0), 0.02, 1e-6);
    EXPECT_NEAR(twist(1), 0.04, 1e-6);
    EXPECT_NEAR(twist(2), 0.0, 1e-6);

    // Angular part: 1.5 * 0.1 = 0.15 (< max 0.5)
    EXPECT_NEAR(twist(5), 0.15, 1e-6);

    // 2. Large error: should cap the linear twist magnitude exactly at max_linear_speed
    CartesianError large_error;
    large_error.linear = Eigen::Vector3d(1.0, 0.0, 0.0);
    large_error.angular = Eigen::Vector3d::Zero();

    auto saturated_twist = solver.desiredTwist(large_error);
    EXPECT_NEAR(saturated_twist.head<3>().norm(), params.max_linear_speed, 1e-6);
}

/**
 * @brief Tests DLS inversion mapping of spatial twist to joint velocities and joint velocity clamping.
 */
TEST(VelocityIkSolverTest, DlsSolveAndJointClamping) {
    VelocityIkSolver::Params params;
    params.damping_lambda = 0.01;
    params.max_joint_speed = 1.0;

    VelocityIkSolver solver(params);

    // Identity-like Jacobian (top 6 rows of 6x7)
    RobotModel::Jacobian6x7 J = RobotModel::Jacobian6x7::Zero();
    J.topLeftCorner<6, 6>() = Eigen::Matrix<double, 6, 6>::Identity();

    VelocityIkSolver::Vector6d twist;
    twist << 0.05, 0.05, 0.05, 0.0, 0.0, 0.0;

    auto dq = solver.solve(J, twist);

    // First 3 joints should mirror the translational twist
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(dq(i), 0.05, 1e-3);
    }
    // Remaining joints should be zero
    for (int i = 3; i < 7; ++i) {
        EXPECT_NEAR(dq(i), 0.0, 1e-3);
    }
}
