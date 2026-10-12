// Pure tests for launchDelay (no ROS). See core/launch_delay.hpp.

#include <gtest/gtest.h>

#include <stdexcept>

#include "haptic_dmp_learning/core/math_utils.hpp"
#include "satellite_grasp_planner/core/launch_delay.hpp"

using satellite_grasp_planner::core::launchDelay;
using haptic_dmp_learning::core::degToRad;

TEST(LaunchDelay, HandComputedAndRange) {
    const double omega = -degToRad(2.0);  // T = 180 s
    // (-60 deg - 0)/(-2 deg/s) = 30 s, minus tau_contact 30 -> 0.
    EXPECT_NEAR(launchDelay(0.0, -degToRad(60.0), omega, 30.0), 0.0, 1e-9);
    // (-30)/(-2) = 15 s, minus 30 = -15 -> +180 = 165.
    EXPECT_NEAR(launchDelay(0.0, -degToRad(30.0), omega, 30.0), 165.0, 1e-9);
    // min_delay shifts the interval: -15 into [10, 190) -> 165 stays, 5 -> 185.
    EXPECT_NEAR(launchDelay(0.0, -degToRad(30.0), omega, 30.0, 10.0), 165.0, 1e-9);
    EXPECT_NEAR(launchDelay(0.0, -degToRad(70.0), omega, 30.0, 10.0), 185.0, 1e-9);

    for (double now : {0.0, 1.0, -2.5}) {
        for (double star : {-3.0, -0.4, 0.7, 2.9}) {
            for (double md : {0.0, 12.5}) {
                const double d = launchDelay(now, star, omega, 30.0, md);
                EXPECT_GE(d, md);
                EXPECT_LT(d, md + 180.0);
            }
        }
    }
    EXPECT_THROW(launchDelay(0.0, 1.0, 0.0, 30.0), std::invalid_argument);
}
