#include <gtest/gtest.h>

#include "haptic_dmp_learning/core/gripper_ramp.hpp"

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
