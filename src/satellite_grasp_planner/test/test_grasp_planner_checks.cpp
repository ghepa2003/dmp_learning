// Tests for the pure (no rclcpp::init needed) startup/runtime validation helpers used by
// grasp_planner_node - see grasp_planner_checks.hpp.

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "satellite_grasp_planner/ros/grasp_planner_checks.hpp"

using namespace satellite_grasp_planner::ros_wrapper;

TEST(GraspPlannerChecksTest, RobotBaseWorldZeroPasses) {
    EXPECT_NO_THROW(checkRobotBaseWorldIsZero(Eigen::Vector3d::Zero(), 1e-6));
}

TEST(GraspPlannerChecksTest, RobotBaseWorldNonZeroThrows) {
    EXPECT_THROW(checkRobotBaseWorldIsZero(Eigen::Vector3d(0.01, 0.0, 0.0), 1e-6), std::invalid_argument);
}

TEST(GraspPlannerChecksTest, CubeAxisMatchesRotationAxisPasses) {
    // Different magnitudes, same direction - only the normalized axis matters.
    EXPECT_NO_THROW(checkCubeAxisMatchesRotationAxis(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 2)));
}

TEST(GraspPlannerChecksTest, CubeAxisMismatchThrows) {
    EXPECT_THROW(checkCubeAxisMatchesRotationAxis(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 1, 0)),
                std::invalid_argument);
}

TEST(GraspPlannerChecksTest, CubeCenterMatchesRotationCenterPasses) {
    EXPECT_NO_THROW(
        checkCubeCenterMatchesRotationCenter(Eigen::Vector3d(0.75, 0.0, 0.35), Eigen::Vector3d(0.75, 0.0, 0.35)));
}

TEST(GraspPlannerChecksTest, CubeCenterMismatchThrows) {
    EXPECT_THROW(
        checkCubeCenterMatchesRotationCenter(Eigen::Vector3d(0.75, 0.0, 0.35), Eigen::Vector3d(0.0, 0.0, 0.0)),
        std::invalid_argument);
}

TEST(GraspPlannerChecksTest, QRefIdentityOnAnyAxisPasses) {
    EXPECT_NO_THROW(checkQRefOnAxis(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0, 0, 1), 0.5));
    EXPECT_NO_THROW(checkQRefOnAxis(Eigen::Quaterniond::Identity(), Eigen::Vector3d(1, 0, 0), 0.5));
}

TEST(GraspPlannerChecksTest, QRefOnAxisPasses) {
    const Eigen::Quaterniond q(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
    EXPECT_NO_THROW(checkQRefOnAxis(q, Eigen::Vector3d(0, 0, 1), 0.5));
}

TEST(GraspPlannerChecksTest, QRefOffAxisThrows) {
    const Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()));
    EXPECT_THROW(checkQRefOnAxis(q, Eigen::Vector3d(0, 0, 1), 0.5), std::invalid_argument);
}

TEST(GraspPlannerChecksTest, FirstOdomSampleValidPasses) {
    EXPECT_NO_THROW(checkFirstOdomSample(Eigen::Vector3d(0, 0, 0), "world", Eigen::Vector3d(0, 0, 0)));
}

TEST(GraspPlannerChecksTest, FirstOdomSampleWrongFrameThrows) {
    EXPECT_THROW(checkFirstOdomSample(Eigen::Vector3d(0, 0, 0), "map", Eigen::Vector3d(0, 0, 0)),
                std::runtime_error);
}

TEST(GraspPlannerChecksTest, FirstOdomSampleTooFarThrows) {
    EXPECT_THROW(checkFirstOdomSample(Eigen::Vector3d(0.1, 0, 0), "world", Eigen::Vector3d(0, 0, 0)),
                std::runtime_error);
}

TEST(GraspPlannerChecksTest, OmegaConsistencyMatchPasses) {
    EXPECT_NO_THROW(
        checkOmegaConsistency(Eigen::Vector3d(0, 0, -0.0349), Eigen::Vector3d(0, 0, 1), -0.0349, 1e-3));
}

TEST(GraspPlannerChecksTest, OmegaConsistencyMismatchThrows) {
    EXPECT_THROW(
        checkOmegaConsistency(Eigen::Vector3d(0, 0, 0.0349), Eigen::Vector3d(0, 0, 1), -0.0349, 1e-3),
        std::runtime_error);
}
