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

/**
 * @brief QuaternionDMP must also drop non-increasing-timestamp samples before
 * differencing the orientation trajectory. Exact duplicate rows carry no new
 * information, so the fit must match the clean one and report the drop count.
 */
TEST(QuaternionDMPCoreTest, DropsNonIncreasingTimestampSamples) {
    const int N = 80;
    const double dt = 0.02;

    Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond q_goal(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()));

    std::vector<Sample> clean;
    for (int i = 0; i < N; ++i) {
        double s = static_cast<double>(i) / (N - 1);
        Sample smp;
        smp.t = i * dt;
        smp.position = Eigen::Vector3d::Zero();
        smp.orientation = q0.slerp(s, q_goal);
        clean.push_back(smp);
    }

    std::vector<Sample> with_dups;
    const int inject_after[] = {15, 30, 31, 55};
    for (int i = 0; i < N; ++i) {
        with_dups.push_back(clean[i]);
        for (int idx : inject_after) {
            if (i == idx) with_dups.push_back(clean[i]);  // dt = 0 vs previous
        }
    }
    ASSERT_EQ(with_dups.size(), clean.size() + 4);

    QuaternionDMP clean_qdmp(25);
    clean_qdmp.learnFromDemonstration(clean);
    EXPECT_EQ(clean_qdmp.diagnostics().dropped_non_monotonic_samples, 0);

    QuaternionDMP dup_qdmp(25);
    dup_qdmp.learnFromDemonstration(with_dups);

    EXPECT_EQ(dup_qdmp.diagnostics().dropped_non_monotonic_samples, 4);
    EXPECT_NEAR(dup_qdmp.tau(), (N - 1) * dt, 1e-9);

    for (int d = 0; d < 3; ++d) {
        for (int i = 0; i < 25; ++i) {
            EXPECT_NEAR(dup_qdmp.weights()[d](i), clean_qdmp.weights()[d](i), 1e-9)
                << "weight mismatch at dim " << d << " basis " << i;
        }
    }
}
