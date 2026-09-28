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

TEST(GraspRollApply, AxesAndComposition) {
    const Eigen::Quaterniond q =
        (Eigen::AngleAxisd(0.4, Eigen::Vector3d(1.0, 2.0, 3.0).normalized()) *
         Eigen::Quaterniond::Identity()).normalized();
    const Eigen::Matrix3d R = q.toRotationMatrix();
    const double psi = 0.9;
    const Eigen::Matrix3d Rp = applyRoll(q, psi).toRotationMatrix();

    // Local z (approach axis) unchanged.
    EXPECT_NEAR((Rp.col(2) - R.col(2)).norm(), 0.0, 1e-12);
    // x, y rotated by psi about z.
    const Eigen::Vector3d x_exp = std::cos(psi) * R.col(0) + std::sin(psi) * R.col(1);
    const Eigen::Vector3d y_exp = -std::sin(psi) * R.col(0) + std::cos(psi) * R.col(1);
    EXPECT_NEAR((Rp.col(0) - x_exp).norm(), 0.0, 1e-12);
    EXPECT_NEAR((Rp.col(1) - y_exp).norm(), 0.0, 1e-12);

    // psi = pi flips x and y.
    const Eigen::Matrix3d Rpi = applyRoll(q, kPi).toRotationMatrix();
    EXPECT_NEAR((Rpi.col(0) + R.col(0)).norm(), 0.0, 1e-12);
    EXPECT_NEAR((Rpi.col(1) + R.col(1)).norm(), 0.0, 1e-12);

    // Composition.
    const double a = 0.35, b = -1.1;
    const Eigen::Matrix3d Rab = applyRoll(applyRoll(q, a), b).toRotationMatrix();
    const Eigen::Matrix3d Rsum = applyRoll(q, a + b).toRotationMatrix();
    EXPECT_NEAR((Rab - Rsum).norm(), 0.0, 1e-12);
}

TEST(GraspRollEquivalent, TwoBasinsDifferByPi) {
    for (double psi : kSamples) {
        const auto r = equivalentRolls(psi);
        EXPECT_NEAR(std::abs(wrapPi(r[0] - r[1])), kPi, 1e-12) << "psi=" << psi;
        EXPECT_NEAR(wrapRollHalfPi(r[0]), wrapRollHalfPi(r[1]), 1e-9) << "psi=" << psi;
    }
}
