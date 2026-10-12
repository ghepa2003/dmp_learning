// Tests for referenceWTrans() with a synthetic ProDMP (mechanism only). See core/demo_reference.hpp.

#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Dense>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "satellite_grasp_planner/core/demo_reference.hpp"
#include "test_fixtures.hpp"

using satellite_grasp_planner::core::referenceWTrans;
using RobotModel = franka_cartesian_control::core::RobotModel;
using satellite_grasp_planner::test_fixtures::loadPandaRobotModel;
using satellite_grasp_planner::test_fixtures::makeSyntheticProDmpTemplate;

TEST(DemoReferenceTest, ReferenceWTransFinitePositive) {
    auto robot = loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    const double w = referenceWTrans(makeSyntheticProDmpTemplate(), robot, RobotModel::readyPose(),
                                     Eigen::Vector3d(0.1, 0.0, 0.0));
    EXPECT_TRUE(std::isfinite(w));
    EXPECT_GT(w, 0.0);
}
