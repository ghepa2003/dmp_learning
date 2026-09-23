#include <gtest/gtest.h>
#include "haptic_dmp_learning/core/controller_param_sync.hpp"
#include <cmath>
#include <limits>

using namespace haptic_dmp_learning::core::controller_param_sync;

// The node itself (syncControllerAlignmentOverride() in prodmp_gazebo_executor_node.cpp) needs a
// live rclcpp node graph with the controller's parameter service up, so it is not unit-testable
// in isolation - what IS pure and worth covering here is the logic that decides what gets sent
// and whether it's valid before it's sent.

TEST(ControllerParamSync, ParamNameMatchesTheControllerSideName) {
    // Not independently verifiable from this package (the controller lives in
    // franka_cartesian_control), but this pins the literal so a future rename on either side is
    // forced to touch this test.
    EXPECT_STREQ(kInitialAlignmentPositionOverrideParamName, "initial_alignment_position_override");
}

TEST(ControllerParamSync, SkipInitialAlignmentParamNameMatchesTheControllerSideName) {
    EXPECT_STREQ(kSkipInitialAlignmentParamName, "skip_initial_alignment");
}

TEST(ControllerParamSync, ToAlignmentOverrideValuePacksXyzInOrder) {
    const Eigen::Vector3d ee_now(0.512, -0.034, 0.287);
    const std::vector<double> value = toAlignmentOverrideValue(ee_now);
    ASSERT_EQ(value.size(), 3u);
    EXPECT_DOUBLE_EQ(value[0], ee_now.x());
    EXPECT_DOUBLE_EQ(value[1], ee_now.y());
    EXPECT_DOUBLE_EQ(value[2], ee_now.z());
}

TEST(ControllerParamSync, ValidValueIsThreeFiniteComponents) {
    EXPECT_TRUE(isValidAlignmentOverrideValue({0.1, -0.2, 0.3}));
    EXPECT_TRUE(isValidAlignmentOverrideValue(toAlignmentOverrideValue(Eigen::Vector3d::Zero())));
}

TEST(ControllerParamSync, WrongSizeIsInvalid) {
    EXPECT_FALSE(isValidAlignmentOverrideValue({}));
    EXPECT_FALSE(isValidAlignmentOverrideValue({0.1, 0.2}));
    EXPECT_FALSE(isValidAlignmentOverrideValue({0.1, 0.2, 0.3, 0.4}));
}

TEST(ControllerParamSync, AnyNanComponentIsInvalid) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(isValidAlignmentOverrideValue({nan, 0.0, 0.0}));
    EXPECT_FALSE(isValidAlignmentOverrideValue({0.0, nan, 0.0}));
    EXPECT_FALSE(isValidAlignmentOverrideValue({0.0, 0.0, nan}));
    EXPECT_FALSE(isValidAlignmentOverrideValue({nan, nan, nan}));
}
