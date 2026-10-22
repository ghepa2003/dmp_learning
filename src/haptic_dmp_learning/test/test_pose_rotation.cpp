#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "haptic_dmp_learning/core/pose_rotation.hpp"

using haptic_dmp_learning::core::RotatablePose;
using haptic_dmp_learning::core::rotatePoseAboutAxis;
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

// ---------------------------------------------------------------------------------------
// rotatePoseAboutAxis
// ---------------------------------------------------------------------------------------

namespace {
Eigen::Matrix3d rodrigues(const Eigen::Vector3d& axis, double phi) {
  const Eigen::Vector3d a = axis.normalized();
  Eigen::Matrix3d K;
  K << 0.0, -a.z(), a.y(), a.z(), 0.0, -a.x(), -a.y(), a.x(), 0.0;
  return Eigen::Matrix3d::Identity() + std::sin(phi) * K + (1.0 - std::cos(phi)) * K * K;
}
}  // namespace

// For axis == UnitZ the new function is BIT-identical to rotatePoseAboutZ (EXPECT_EQ on every coefficient, no tolerance).
TEST(PoseRotationAxis, UnitZIsBitIdenticalToRotatePoseAboutZ) {
  const RotatablePose p = samplePose();
  for (const double phi : {0.0, 1e-9, 0.3, -0.5, M_PI / 2.0, 2.0, -3.0, 7.0}) {
    for (const Eigen::Vector3d& c : {kCenter, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(-1.3, 2.2, 0.1)}) {
      const RotatablePose a = rotatePoseAboutZ(c, p, phi);
      const RotatablePose b = rotatePoseAboutAxis(c, Eigen::Vector3d::UnitZ(), p, phi);
      EXPECT_EQ(a.position.x(), b.position.x()) << phi;
      EXPECT_EQ(a.position.y(), b.position.y()) << phi;
      EXPECT_EQ(a.position.z(), b.position.z()) << phi;
      EXPECT_EQ(a.orientation.w(), b.orientation.w()) << phi;
      EXPECT_EQ(a.orientation.x(), b.orientation.x()) << phi;
      EXPECT_EQ(a.orientation.y(), b.orientation.y()) << phi;
      EXPECT_EQ(a.orientation.z(), b.orientation.z()) << phi;
    }
  }
}

// An unnormalized z axis gives the same result as UnitZ (normalization of (0,0,k) is exact).
TEST(PoseRotationAxis, UnnormalizedZAxisIsTheSameAsUnitZ) {
  const RotatablePose p = samplePose();
  const RotatablePose a = rotatePoseAboutAxis(kCenter, Eigen::Vector3d::UnitZ(), p, 0.7);
  const RotatablePose b = rotatePoseAboutAxis(kCenter, Eigen::Vector3d(0.0, 0.0, 4.0), p, 0.7);
  EXPECT_EQ(a.position, b.position);
  EXPECT_EQ(a.orientation.coeffs(), b.orientation.coeffs());
}

// Tilted axis against the analytic Rodrigues formula.
TEST(PoseRotationAxis, TiltedAxisMatchesRodrigues) {
  const RotatablePose p = samplePose();
  const Eigen::Vector3d axis(0.3, -0.5, 0.8);  // not unit on purpose
  for (const double phi : {0.2, -1.1, 2.9}) {
    const RotatablePose r = rotatePoseAboutAxis(kCenter, axis, p, phi);
    const Eigen::Matrix3d R = rodrigues(axis, phi);
    EXPECT_LT((r.position - (kCenter + R * (p.position - kCenter))).norm(), 1e-12) << phi;
    const Eigen::Matrix3d Ro = r.orientation.toRotationMatrix();
    EXPECT_LT((Ro - R * p.orientation.toRotationMatrix()).norm(), 1e-12) << phi;
  }
}

// The centre and every point on the axis are fixed; the distance from the axis is preserved.
TEST(PoseRotationAxis, TiltedAxisFixesTheAxisAndPreservesRadius) {
  const Eigen::Vector3d axis = Eigen::Vector3d(0.3, -0.5, 0.8).normalized();
  RotatablePose on_axis;
  on_axis.position = kCenter + 0.37 * axis;
  const RotatablePose r = rotatePoseAboutAxis(kCenter, axis, on_axis, 1.234);
  EXPECT_LT((r.position - on_axis.position).norm(), 1e-14);
  const RotatablePose p = samplePose();
  const RotatablePose q = rotatePoseAboutAxis(kCenter, axis, p, -2.1);
  auto radius = [&](const Eigen::Vector3d& x) {
    const Eigen::Vector3d d = x - kCenter;
    return (d - d.dot(axis) * axis).norm();
  };
  EXPECT_NEAR(radius(q.position), radius(p.position), 1e-12);
  EXPECT_NEAR((q.position - kCenter).dot(axis), (p.position - kCenter).dot(axis), 1e-12);
}

TEST(PoseRotationAxis, CompositionAboutATiltedAxis) {
  const RotatablePose p = samplePose();
  const Eigen::Vector3d axis(0.3, -0.5, 0.8);
  const RotatablePose two = rotatePoseAboutAxis(kCenter, axis, rotatePoseAboutAxis(kCenter, axis, p, 0.3), 0.5);
  const RotatablePose one = rotatePoseAboutAxis(kCenter, axis, p, 0.8);
  EXPECT_LT((two.position - one.position).norm(), 1e-12);
  EXPECT_LT(two.orientation.angularDistance(one.orientation), 1e-12);
}

// The follow feed-forward used by followTargetSample is omega a x (p - c): it is the time derivative of the rotated
// position (central difference of rotatePoseAboutAxis(c, a, pose, omega t) in t), and omega a is the angular velocity
// of the rotated orientation (R(t+h) R(t-h)^-1 = exp(2 h omega a)).
TEST(PoseRotationAxis, FeedForwardIsTheNumericalDerivativeOfTheTarget) {
  const RotatablePose p = samplePose();
  const Eigen::Vector3d axis = Eigen::Vector3d(-0.2, 0.4, 0.9).normalized();
  const double omega = -5.0 * M_PI / 180.0, t = 1.7, h = 1e-4;
  const RotatablePose m = rotatePoseAboutAxis(kCenter, axis, p, omega * (t - h));
  const RotatablePose n = rotatePoseAboutAxis(kCenter, axis, p, omega * t);
  const RotatablePose f = rotatePoseAboutAxis(kCenter, axis, p, omega * (t + h));
  const Eigen::Vector3d v_num = (f.position - m.position) / (2.0 * h);
  const Eigen::Vector3d v_ff = omega * axis.cross(n.position - kCenter);
  EXPECT_LT((v_num - v_ff).norm(), 1e-9);
  const Eigen::Quaterniond dq = f.orientation * m.orientation.conjugate();
  const Eigen::AngleAxisd aa(dq);
  const Eigen::Vector3d w_num = aa.angle() * aa.axis() / (2.0 * h);
  EXPECT_LT((w_num - omega * axis).norm(), 1e-9);
}

TEST(PoseRotationAxis, ZeroOrNonFiniteAxisThrows) {
  const RotatablePose p = samplePose();
  EXPECT_THROW(rotatePoseAboutAxis(kCenter, Eigen::Vector3d::Zero(), p, 0.1), std::invalid_argument);
  EXPECT_THROW(rotatePoseAboutAxis(kCenter, Eigen::Vector3d(0.0, 0.0, NAN), p, 0.1), std::invalid_argument);
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
