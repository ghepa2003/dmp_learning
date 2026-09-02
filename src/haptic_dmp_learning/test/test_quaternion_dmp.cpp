#include <gtest/gtest.h>
#include "haptic_dmp_learning/core/quaternion_dmp.hpp"
#include "haptic_dmp_learning/core/filter_utils.hpp"
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

/**
 * @brief filter_utils::enforceQuaternionContinuity must undo a quaternion
 * double-cover sign flip: the raw local increment across a q / -q pair is a
 * near-2*pi jump, while after the correction it is the small real rotation.
 */
TEST(QuaternionDMPCoreTest, EnforceQuaternionContinuityUndoesSignFlip) {
    const Eigen::Vector3d axis = Eigen::Vector3d(0.3, -0.5, 0.8).normalized();
    Eigen::Quaterniond qa(Eigen::AngleAxisd(0.70, axis));       qa.normalize();
    Eigen::Quaterniond qb_phys(Eigen::AngleAxisd(0.706, axis)); qb_phys.normalize();
    // Same physical rotation as qb_phys, reported with the whole quaternion negated.
    Eigen::Quaterniond qb_flipped(-qb_phys.w(), -qb_phys.x(), -qb_phys.y(), -qb_phys.z());

    std::vector<Sample> demo(2);
    demo[0].t = 0.00; demo[0].position.setZero(); demo[0].orientation = qa;
    demo[1].t = 0.01; demo[1].position.setZero(); demo[1].orientation = qb_flipped;

    // Raw increment (what unwrapRotationVector would accumulate): ~2*pi.
    Eigen::Quaterniond dq_raw = qb_flipped * qa.conjugate();
    double raw_incr = (2.0 * QuaternionDMP::logMap(dq_raw)).norm();
    EXPECT_GT(raw_incr, 5.0);

    int corrected = -1;
    auto fixed = filter_utils::enforceQuaternionContinuity(demo, &corrected);
    EXPECT_EQ(corrected, 1);
    EXPECT_LT(fixed[0].orientation.dot(fixed[1].orientation), 1.0);
    EXPECT_GT(fixed[0].orientation.dot(fixed[1].orientation), 0.0);

    Eigen::Quaterniond dq_fixed = fixed[1].orientation * fixed[0].orientation.conjugate();
    double fixed_incr = (2.0 * QuaternionDMP::logMap(dq_fixed)).norm();
    EXPECT_LT(fixed_incr, 0.05);  // ~0.006 rad of real motion, no pi jump
}

/**
 * @brief learnFromDemonstration must sign-correct a double-cover flip internally:
 * a demo with a negated block trains to the same weights as the clean demo and
 * reports the corrected count.
 */
TEST(QuaternionDMPCoreTest, LearnCorrectsQuaternionDoubleCoverFlips) {
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

    // Negate a contiguous block [35, 60]: exactly two sign transitions, one
    // into the block at k=35 and one out of it at k=61.
    std::vector<Sample> flipped = clean;
    for (int i = 35; i <= 60; ++i) {
        auto& c = flipped[i].orientation.coeffs();
        c = -c;
    }

    QuaternionDMP clean_qdmp(25);
    clean_qdmp.learnFromDemonstration(clean);
    EXPECT_EQ(clean_qdmp.diagnostics().quat_sign_flips_corrected, 0);

    QuaternionDMP flip_qdmp(25);
    flip_qdmp.learnFromDemonstration(flipped);
    EXPECT_EQ(flip_qdmp.diagnostics().quat_sign_flips_corrected, 2);
    EXPECT_NEAR(flip_qdmp.tau(), (N - 1) * dt, 1e-9);

    for (int d = 0; d < 3; ++d) {
        for (int i = 0; i < 25; ++i) {
            EXPECT_NEAR(flip_qdmp.weights()[d](i), clean_qdmp.weights()[d](i), 1e-9)
                << "weight mismatch at dim " << d << " basis " << i;
        }
    }
}
