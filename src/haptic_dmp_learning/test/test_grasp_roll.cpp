// Pure tests for the grasp-roll helpers (no ROS). See core/grasp_roll.hpp.

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>

#include "haptic_dmp_learning/core/grasp_roll.hpp"

using namespace haptic_dmp_learning::core;

namespace {
const std::vector<double> kSamples = {0.3, -0.7, 1.9, -2.6, 4.4};
}

TEST(GraspRollWrap, KnownValues) {
    EXPECT_NEAR(wrapRollHalfPi(0.0), 0.0, 1e-12);
    EXPECT_NEAR(wrapRollHalfPi(kPi / 2.0), kPi / 2.0, 1e-12);
    EXPECT_NEAR(wrapRollHalfPi(-kPi / 2.0), kPi / 2.0, 1e-12);  // convention -pi/2 -> +pi/2
    EXPECT_NEAR(wrapRollHalfPi(kPi), 0.0, 1e-12);
    EXPECT_NEAR(wrapRollHalfPi(3.0 * kPi / 2.0), kPi / 2.0, 1e-12);
}

TEST(GraspRollPenalty, ZeroOneAndPiPeriodic) {
    const double tol = kDefaultRollToleranceRad;
    EXPECT_NEAR(rollPenalty(0.0, tol), 0.0, 1e-15);
    EXPECT_NEAR(rollPenalty(tol, tol), 1.0, 1e-12);
    for (double psi : kSamples) {
        EXPECT_NEAR(rollPenalty(psi, tol), rollPenalty(psi + kPi, tol), 1e-9) << "psi=" << psi;
    }
}

TEST(GraspRollPenalty, ThrowsOnNonPositiveTolerance) {
    EXPECT_THROW(rollPenalty(0.1, 0.0), std::invalid_argument);
    EXPECT_THROW(rollPenalty(0.1, -0.1), std::invalid_argument);
}

TEST(GraspRollEquivalent, TwoBasinsDifferByPi) {
    for (double psi : kSamples) {
        const auto r = equivalentRolls(psi);
        EXPECT_NEAR(std::abs(wrapPi(r[0] - r[1])), kPi, 1e-12) << "psi=" << psi;
        EXPECT_NEAR(wrapRollHalfPi(r[0]), wrapRollHalfPi(r[1]), 1e-9) << "psi=" << psi;
    }
}
