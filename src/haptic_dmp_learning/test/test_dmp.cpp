#include <gtest/gtest.h>
#include "haptic_dmp_learning/core/dmp.hpp"
#include "haptic_dmp_learning/core/filter_utils.hpp"
#include <vector>
#include <cmath>

using namespace haptic_dmp_learning::core;

/**
 * @brief Unit test verifying DMP learning and rollout convergence to the demonstrated goal.
 *
 * A minimum-jerk trajectory profile s(t) = s^3 * (10 - 15s + 6s^2) is generated as demonstration.
 * The DMP learns the non-linear forcing weights and executes a rollout from y0.
 * The final position at t = tau must converge to within 0.02 m of the target attractor goal.
 */
TEST(DMPCoreTest, ConvergenceToDemonstratedGoal) {
    const int N = 100;
    const double dt = 0.02;
    const double duration = (N - 1) * dt;

    Eigen::Vector3d p_start(0.0, 0.0, 0.0);
    Eigen::Vector3d p_goal(0.3, -0.2, 0.4);

    std::vector<Sample> demo(N);
    for (int i = 0; i < N; ++i) {
        double s = static_cast<double>(i) / (N - 1);
        // Smooth 5th-order minimum-jerk profile: s^3 * (10 - 15s + 6s^2)
        double poly = s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
        demo[i].t = i * dt;
        demo[i].position = p_start + poly * (p_goal - p_start);
        demo[i].orientation = Eigen::Quaterniond::Identity();
    }

    DMP dmp(25, 1.0, 25.0, 6.25, false);
    dmp.learnFromDemonstration(demo);

    EXPECT_TRUE(dmp.isLearned());
    EXPECT_NEAR(dmp.tau(), duration, 1e-4);

    dmp.reset();
    Eigen::Vector3d pos = dmp.position();
    EXPECT_NEAR(pos.x(), p_start.x(), 1e-3);
    EXPECT_NEAR(pos.y(), p_start.y(), 1e-3);
    EXPECT_NEAR(pos.z(), p_start.z(), 1e-3);

    for (int i = 0; i < N - 1; ++i) {
        pos = dmp.step(dt);
    }

    EXPECT_NEAR(pos.x(), p_goal.x(), 0.02);
    EXPECT_NEAR(pos.y(), p_goal.y(), 0.02);
    EXPECT_NEAR(pos.z(), p_goal.z(), 0.02);
}

/**
 * @brief Unit test verifying spatial generalization to a new target goal.
 *
 * Demonstrates a motion towards p_goal_demo, then overrides setGoal(p_goal_new).
 * The DMP dynamic system must adapt its spatial scaling and converge to the new target goal.
 */
TEST(DMPCoreTest, GoalGeneralization) {
    const int N = 100;
    const double dt = 0.02;

    Eigen::Vector3d p_start(0.0, 0.0, 0.0);
    Eigen::Vector3d p_goal_demo(0.3, 0.0, 0.3);
    Eigen::Vector3d p_goal_new(0.5, 0.2, 0.1);

    std::vector<Sample> demo(N);
    for (int i = 0; i < N; ++i) {
        double s = static_cast<double>(i) / (N - 1);
        demo[i].t = i * dt;
        demo[i].position = p_start + s * (p_goal_demo - p_start);
        demo[i].orientation = Eigen::Quaterniond::Identity();
    }

    DMP dmp(25);
    dmp.learnFromDemonstration(demo);

    dmp.reset();
    dmp.setGoal(p_goal_new);

    Eigen::Vector3d pos = dmp.position();
    for (int i = 0; i < N - 1; ++i) {
        pos = dmp.step(dt);
    }

    EXPECT_NEAR(pos.x(), p_goal_new.x(), 0.03);
    EXPECT_NEAR(pos.y(), p_goal_new.y(), 0.03);
    EXPECT_NEAR(pos.z(), p_goal_new.z(), 0.03);
}

/**
 * @brief Unit test verifying the zero-phase moving average smoothing filter.
 *
 * Adds alternating high-frequency noise to a constant 3D signal and verifies
 * that movingAverageSmooth suppresses the variance without introducing phase delay.
 */
TEST(DMPCoreTest, MovingAverageFilterSmoothing) {
    std::vector<Eigen::Vector3d> signal;
    std::vector<double> t;
    const int N = 50;
    const double dt = 0.01;

    for (int i = 0; i < N; ++i) {
        t.push_back(i * dt);
        double noise = (i % 2 == 0) ? 0.1 : -0.1;
        signal.push_back(Eigen::Vector3d(1.0 + noise, 2.0 + noise, 3.0 + noise));
    }

    auto filtered = filter_utils::movingAverageSmooth(signal, t, 0.05);

    EXPECT_EQ(filtered.size(), signal.size());
    // Middle samples should be very close to nominal (1.0, 2.0, 3.0)
    EXPECT_NEAR(filtered[25].x(), 1.0, 0.05);
    EXPECT_NEAR(filtered[25].y(), 2.0, 0.05);
    EXPECT_NEAR(filtered[25].z(), 3.0, 0.05);
}
