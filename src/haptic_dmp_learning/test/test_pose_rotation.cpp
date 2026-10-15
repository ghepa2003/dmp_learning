#include <gtest/gtest.h>

#include "haptic_dmp_learning/core/pose_rotation.hpp"

using haptic_dmp_learning::core::RotatablePose;
using haptic_dmp_learning::core::rotatePoseAboutZ;

namespace {
RotatablePose samplePose() {
    RotatablePose p;
    p.position = Eigen::Vector3d(0.9, 0.2, 0.8);
    p.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d(1, 2, 3).normalized()));
    return p;
}
const Eigen::Vector3d kCenter(0.7, 0.0, 0.77);
}  // namespace

TEST(PoseRotation, ZeroAngleIsIdentity) {
    const RotatablePose p = samplePose();
    const RotatablePose r = rotatePoseAboutZ(kCenter, p, 0.0);
    EXPECT_LT((r.position - p.position).norm(), 1e-15);
    EXPECT_LT(r.orientation.angularDistance(p.orientation), 1e-12);
}

TEST(PoseRotation, QuarterTurnKnownCase) {
    RotatablePose p;
    p.position = Eigen::Vector3d(1.7, 0.0, 0.5);  // 1 m from the centre along +x
    p.orientation = Eigen::Quaterniond::Identity();
    const RotatablePose r = rotatePoseAboutZ(kCenter, p, M_PI / 2.0);
    EXPECT_NEAR(r.position.x(), 0.7, 1e-12);
    EXPECT_NEAR(r.position.y(), 1.0, 1e-12);
    EXPECT_NEAR(r.position.z(), 0.5, 1e-12);  // z unchanged
    const Eigen::Matrix3d R = r.orientation.toRotationMatrix();
    EXPECT_NEAR((R * Eigen::Vector3d::UnitX() - Eigen::Vector3d::UnitY()).norm(), 0.0, 1e-12);
}

TEST(PoseRotation, CompositionOfTwoRotations) {
    const RotatablePose p = samplePose();
    const RotatablePose two = rotatePoseAboutZ(kCenter, rotatePoseAboutZ(kCenter, p, 0.3), 0.5);
    const RotatablePose one = rotatePoseAboutZ(kCenter, p, 0.8);
    EXPECT_LT((two.position - one.position).norm(), 1e-12);
    EXPECT_LT(two.orientation.angularDistance(one.orientation), 1e-12);
}

TEST(PoseRotation, CenterIsFixed) {
    RotatablePose p;
    p.position = kCenter;
    const RotatablePose r = rotatePoseAboutZ(kCenter, p, 1.234);
    EXPECT_LT((r.position - kCenter).norm(), 1e-15);
}

using haptic_dmp_learning::core::effectiveFollowDtheta;

TEST(EffectiveFollowDtheta, BeforeAtEndPassesCurrent) {
    EXPECT_DOUBLE_EQ(effectiveFollowDtheta(-0.5, 3.0, 0.2, 0.9), 0.2);
}
TEST(EffectiveFollowDtheta, DuringFollowPassesCurrent) {
    EXPECT_DOUBLE_EQ(effectiveFollowDtheta(0.0, 3.0, 0.1, 0.0), 0.1);
    EXPECT_DOUBLE_EQ(effectiveFollowDtheta(3.0, 3.0, 0.3, 0.0), 0.3);
}
TEST(EffectiveFollowDtheta, AfterFollowIsFrozen) {
    EXPECT_DOUBLE_EQ(effectiveFollowDtheta(3.1, 3.0, 0.5, 0.3), 0.3);
    EXPECT_DOUBLE_EQ(effectiveFollowDtheta(100.0, 3.0, 9.0, 0.3), 0.3);
}
TEST(EffectiveFollowDtheta, NonPositiveFollowMeansWholeHold) {
    EXPECT_DOUBLE_EQ(effectiveFollowDtheta(50.0, 0.0, 0.7, 0.1), 0.7);
    EXPECT_DOUBLE_EQ(effectiveFollowDtheta(50.0, -1.0, 0.7, 0.1), 0.7);
}
