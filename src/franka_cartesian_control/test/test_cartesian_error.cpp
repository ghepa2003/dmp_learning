#include <gtest/gtest.h>
#include "franka_cartesian_control/core/cartesian_error.hpp"
#include <cmath>

using namespace franka_cartesian_control::core;

/**
 * @brief Tests that identical position and orientation yield zero linear and angular error.
 */
TEST(CartesianErrorTest, ZeroErrorWhenPosesMatch) {
    Eigen::Vector3d p(0.5, 0.2, 0.3);
    Eigen::Quaterniond q(0.7071, 0.7071, 0.0, 0.0);
    q.normalize();

    CartesianError err = computePoseError(p, q, p, q);

    EXPECT_NEAR(err.linear.norm(), 0.0, 1e-6);
    EXPECT_NEAR(err.angular.norm(), 0.0, 1e-6);
}

/**
 * @brief Tests linear displacement error when orientations are identical.
 */
TEST(CartesianErrorTest, PureTranslationError) {
    Eigen::Vector3d p_actual(0.0, 0.0, 0.0);
    Eigen::Vector3d p_target(0.1, -0.2, 0.3);
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

    CartesianError err = computePoseError(p_actual, q, p_target, q);

    EXPECT_NEAR(err.linear.x(), 0.1, 1e-6);
    EXPECT_NEAR(err.linear.y(), -0.2, 1e-6);
    EXPECT_NEAR(err.linear.z(), 0.3, 1e-6);
    EXPECT_NEAR(err.angular.norm(), 0.0, 1e-6);
}

/**
 * @brief Tests Lie algebra so(3) angular error for a 90 degree rotation around Z-axis.
 */
TEST(CartesianErrorTest, PureRotationErrorAroundZ) {
    Eigen::Vector3d p(0.0, 0.0, 0.0);
    Eigen::Quaterniond q_actual = Eigen::Quaterniond::Identity();
    // 90 degrees around Z axis (pi / 2 radians)
    Eigen::Quaterniond q_target(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));

    CartesianError err = computePoseError(p, q_actual, p, q_target);

    EXPECT_NEAR(err.linear.norm(), 0.0, 1e-6);
    EXPECT_NEAR(err.angular.x(), 0.0, 1e-5);
    EXPECT_NEAR(err.angular.y(), 0.0, 1e-5);
    EXPECT_NEAR(err.angular.z(), M_PI / 2.0, 1e-4);
}
