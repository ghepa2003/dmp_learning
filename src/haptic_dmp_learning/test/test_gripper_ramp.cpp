#include <gtest/gtest.h>

#include "haptic_dmp_learning/core/gripper_ramp.hpp"

using haptic_dmp_learning::core::gripper_ramp::defaultGripperControl;
using haptic_dmp_learning::core::gripper_ramp::gripperCommand;
using haptic_dmp_learning::core::gripper_ramp::interpolateGripperPosition;

namespace {

// Nominal configuration used by both nodes: 0.06 = open, 0.0 = closed, 2 s ramp.
constexpr double kOpen = 0.06;
constexpr double kClosed = 0.0;
constexpr double kDuration = 2.0;

}  // namespace

// elapsed = 0 -> exactly the open position (start of the ramp).
TEST(GripperRamp, StartReturnsOpenPositionExactly) {
    EXPECT_DOUBLE_EQ(interpolateGripperPosition(0.0, kDuration, kOpen, kClosed), kOpen);
}

// elapsed = ramp_duration -> exactly the closed position (end of the ramp).
TEST(GripperRamp, EndReturnsClosedPositionExactly) {
    EXPECT_DOUBLE_EQ(interpolateGripperPosition(kDuration, kDuration, kOpen, kClosed), kClosed);
}

// elapsed = ramp_duration / 2 -> exact midpoint between the two positions.
TEST(GripperRamp, HalfwayReturnsMidpoint) {
    // Powers-of-two-friendly values so the midpoint is bit-exact.
    EXPECT_DOUBLE_EQ(interpolateGripperPosition(1.0, 2.0, 1.0, 0.0), 0.5);
    // Same with the nominal config, within tolerance.
    EXPECT_NEAR(interpolateGripperPosition(kDuration / 2.0, kDuration, kOpen, kClosed),
                (kOpen + kClosed) / 2.0, 1e-12);
}

// elapsed well past the duration (10x) -> still clamped to closed, no overshoot.
TEST(GripperRamp, PastDurationClampsToClosedNoOvershoot) {
    EXPECT_DOUBLE_EQ(interpolateGripperPosition(10.0 * kDuration, kDuration, kOpen, kClosed),
                     kClosed);
}

// Negative elapsed (defensive; should not happen) -> open position, no
// backward extrapolation past the start.
TEST(GripperRamp, NegativeElapsedReturnsOpenNoBackwardExtrapolation) {
    EXPECT_DOUBLE_EQ(interpolateGripperPosition(-5.0, kDuration, kOpen, kClosed), kOpen);
}

// Direction-agnostic: identical behaviour when "closed" is the larger value.
TEST(GripperRamp, WorksWhenClosedIsGreaterThanOpen) {
    const double open = 0.0;
    const double closed = 0.06;
    EXPECT_DOUBLE_EQ(interpolateGripperPosition(0.0, kDuration, open, closed), open);
    EXPECT_DOUBLE_EQ(interpolateGripperPosition(kDuration, kDuration, open, closed), closed);
    EXPECT_DOUBLE_EQ(interpolateGripperPosition(3.0 * kDuration, kDuration, open, closed), closed);
    EXPECT_NEAR(interpolateGripperPosition(kDuration / 2.0, kDuration, open, closed),
                (open + closed) / 2.0, 1e-12);
}

// Monotonic non-increasing open -> closed (and non-decreasing in the reversed
// direction), always within the [closed, open] range, across and past the ramp.
TEST(GripperRamp, MonotoneAndBounded) {
    double prev = interpolateGripperPosition(-1.0, kDuration, kOpen, kClosed);
    for (int i = 0; i <= 300; ++i) {
        const double t = i * 0.01;  // 0 .. 3 s, past the 2 s duration
        const double v = interpolateGripperPosition(t, kDuration, kOpen, kClosed);
        EXPECT_LE(v, prev) << "t=" << t;
        EXPECT_LE(v, kOpen);
        EXPECT_GE(v, kClosed);
        prev = v;
    }
    double prev_r = interpolateGripperPosition(-1.0, kDuration, kClosed, kOpen);
    for (int i = 0; i <= 300; ++i) {
        const double v = interpolateGripperPosition(i * 0.01, kDuration, kClosed, kOpen);
        EXPECT_GE(v, prev_r);
        prev_r = v;
    }
}

// gripper_control default: only continuous + grasp_command.
TEST(GripperRamp, DefaultGripperControlByMode) {
    EXPECT_TRUE(defaultGripperControl("continuous", "grasp_command"));
    EXPECT_FALSE(defaultGripperControl("continuous", "parameters"));
    EXPECT_FALSE(defaultGripperControl("frozen", "grasp_command"));
    EXPECT_FALSE(defaultGripperControl("frozen", "parameters"));
    EXPECT_FALSE(defaultGripperControl("", ""));
}

// gripperCommand: before / during / after the ramp, with and without squeeze.
TEST(GripperCommand, WithoutSqueezeEqualsInterpolation) {
    for (double e : {-1.0, 0.0, 0.5, 1.0, 2.0, 5.0}) {
        EXPECT_DOUBLE_EQ(gripperCommand(e, kDuration, kOpen, kClosed, kClosed),
                         interpolateGripperPosition(e, kDuration, kOpen, kClosed));
    }
}
TEST(GripperCommand, WithSqueezeStepsAfterRamp) {
    constexpr double kSqueeze = -0.05;
    EXPECT_DOUBLE_EQ(gripperCommand(-1.0, kDuration, kOpen, kClosed, kSqueeze), kOpen);
    EXPECT_DOUBLE_EQ(gripperCommand(0.0, kDuration, kOpen, kClosed, kSqueeze), kOpen);
    EXPECT_NEAR(gripperCommand(kDuration / 2, kDuration, kOpen, kClosed, kSqueeze), 0.5 * (kOpen + kClosed), 1e-12);
    EXPECT_DOUBLE_EQ(gripperCommand(kDuration, kDuration, kOpen, kClosed, kSqueeze), kSqueeze);
    EXPECT_DOUBLE_EQ(gripperCommand(10.0, kDuration, kOpen, kClosed, kSqueeze), kSqueeze);
    EXPECT_DOUBLE_EQ(gripperCommand(1.0, 0.0, kOpen, kClosed, kSqueeze), kSqueeze);  // degenerate ramp
}
