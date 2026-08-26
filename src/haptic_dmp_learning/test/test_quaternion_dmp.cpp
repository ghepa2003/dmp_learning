#include <gtest/gtest.h>
#include "haptic_dmp_learning/core/quaternion_dmp.hpp"
#include <vector>
#include <cmath>

using namespace haptic_dmp_learning::core;

/**
 * @brief Unit test verifying the Lie algebra roundtrip: expMap(logMap(q)) and logMap(expMap(r)).
 *
 * Given a rotation vector r in so(3) ~ R^3, expMap generates a unit quaternion q in S^3.
 * logMap must recover the exact rotation vector r to numerical precision.
 */
TEST(QuaternionDMPCoreTest, LogExpRoundtrip) {
    Eigen::Vector3d r(0.2, -0.3, 0.5);
    Eigen::Quaterniond q = QuaternionDMP::expMap(r);
    Eigen::Vector3d r_recovered = QuaternionDMP::logMap(q);

    EXPECT_NEAR(r_recovered.x(), r.x(), 1e-6);
    EXPECT_NEAR(r_recovered.y(), r.y(), 1e-6);
    EXPECT_NEAR(r_recovered.z(), r.z(), 1e-6);
}

/**
 * @brief Unit test verifying that identity orientation maps to zero rotation vector.
 */
TEST(QuaternionDMPCoreTest, IdentityLogMap) {
    Eigen::Quaterniond q_id = Eigen::Quaterniond::Identity();
    Eigen::Vector3d r = QuaternionDMP::logMap(q_id);

    EXPECT_NEAR(r.norm(), 0.0, 1e-8);
}

/**
 * @brief Unit test verifying Quaternion DMP convergence along a spherical linear interpolation (SLERP) path.
 *
 * Demonstrates a 90-degree pitch rotation around the Y axis generated via SLERP.
 * Verifies that the trained QuaternionDMP smoothly reaches the target quaternion orientation on S^3.
 */
TEST(QuaternionDMPCoreTest, RotationRolloutConvergence) {
    const int N = 100;
    const double dt = 0.02;

    Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
    // 90 deg rotation around Y axis: q_goal
    Eigen::Quaterniond q_goal(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()));

    std::vector<Sample> demo(N);
    for (int i = 0; i < N; ++i) {
        double s = static_cast<double>(i) / (N - 1);
        demo[i].t = i * dt;
        demo[i].position = Eigen::Vector3d::Zero();
        demo[i].orientation = q0.slerp(s, q_goal);
    }

    QuaternionDMP qdmp(25);
    qdmp.learnFromDemonstration(demo);

    EXPECT_TRUE(qdmp.isLearned());

    qdmp.reset();
    Eigen::Quaterniond q = qdmp.orientation();
    EXPECT_NEAR(q.angularDistance(q0), 0.0, 1e-3);

    for (int i = 0; i < N - 1; ++i) {
        q = qdmp.step(dt);
    }

    double final_err = q.angularDistance(q_goal);
    EXPECT_LT(final_err, 0.05);
}
