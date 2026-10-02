#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "haptic_dmp_learning/core/anchored_target.hpp"

using namespace haptic_dmp_learning::core::anchored_target;

namespace {

const Eigen::Vector3d kAnchorP(0.4, -0.1, 0.5);

Eigen::Quaterniond anchorQ() {
  return Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized()));
}

Eigen::Quaterniond quat(double angle, const Eigen::Vector3d& axis) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis.normalized()));
}

// Angle between two rotations, sign-insensitive.
double rotAngle(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
  return a.angularDistance(b);
}

}  // namespace

// (1) The first sample returns exactly the anchor.
TEST(AnchoredTarget, FirstSampleReturnsAnchor) {
  State st;
  const Eigen::Quaterniond q_master = quat(1.1, Eigen::Vector3d(0.3, -0.5, 0.8));
  const Pose out = computeAnchoredTarget(st, kAnchorP, anchorQ(), Eigen::Vector3d(0.9, 0.2, -0.3), q_master);
  EXPECT_EQ(out.position, kAnchorP);
  EXPECT_EQ(out.orientation.coeffs(), anchorQ().coeffs());
}

// (2) Master translation = target translation (orientation held constant).
TEST(AnchoredTarget, TranslationIsMirrored) {
  State st;
  const Eigen::Vector3d p0(0.9, 0.2, -0.3);
  const Eigen::Quaterniond q = quat(0.4, Eigen::Vector3d::UnitY());
  computeAnchoredTarget(st, kAnchorP, anchorQ(), p0, q);
  const Eigen::Vector3d d(0.05, -0.02, 0.11);
  const Pose out = computeAnchoredTarget(st, kAnchorP, anchorQ(), p0 + d, q);
  EXPECT_NEAR((out.position - (kAnchorP + d)).norm(), 0.0, 1e-12);
  EXPECT_NEAR(rotAngle(out.orientation, anchorQ()), 0.0, 1e-9);
}

// (3) Relative rotation is applied in the body frame: q_pub = anchor * dq,
// which differs from the world-frame composition dq * anchor.
TEST(AnchoredTarget, RelativeRotationIsInBodyFrame) {
  State st;
  const Eigen::Quaterniond q0 = quat(0.9, Eigen::Vector3d(0, 1, 1));
  const Eigen::Quaterniond dq = quat(0.5, Eigen::Vector3d::UnitX());
  const Eigen::Vector3d p(0, 0, 0);
  computeAnchoredTarget(st, kAnchorP, anchorQ(), p, q0);
  const Pose out = computeAnchoredTarget(st, kAnchorP, anchorQ(), p, q0 * dq);

  const Eigen::Quaterniond expected = anchorQ() * dq;
  EXPECT_NEAR(rotAngle(out.orientation, expected), 0.0, 1e-9);
  // Sanity: the body-frame and world-frame results are genuinely different rotations.
  EXPECT_GT(rotAngle(expected, dq * anchorQ()), 1e-3);
  EXPECT_GT(rotAngle(out.orientation, dq * anchorQ()), 1e-3);
}

// (4) q_corr = Q (x) q_raw gives the same output as q_raw alone: the R_z offset cancels.
TEST(AnchoredTarget, LeftOffsetCancels) {
  const Eigen::Quaterniond Q = quat(M_PI / 2.0, Eigen::Vector3d::UnitZ());
  const std::vector<Eigen::Quaterniond> raw = {
      quat(0.3, Eigen::Vector3d(1, 0, 0)), quat(0.8, Eigen::Vector3d(1, 1, 0)),
      quat(1.4, Eigen::Vector3d(0, 1, 1)), quat(2.0, Eigen::Vector3d(1, 1, 1))};
  State st_raw, st_corr;
  for (const auto& q : raw) {
    const Pose a = computeAnchoredTarget(st_raw, kAnchorP, anchorQ(), Eigen::Vector3d::Zero(), q);
    const Pose b = computeAnchoredTarget(st_corr, kAnchorP, anchorQ(), Eigen::Vector3d::Zero(), Q * q);
    EXPECT_NEAR(rotAngle(a.orientation, b.orientation), 0.0, 1e-9);
    EXPECT_GE(a.orientation.dot(b.orientation), 0.0);
  }
}

// (5) Sign continuity: feeding q and -q for the same rotation must not flip the output.
TEST(AnchoredTarget, SignContinuityWithNegatedInput) {
  State st;
  const Eigen::Quaterniond q0 = quat(0.5, Eigen::Vector3d(0, 0, 1));
  Pose prev = computeAnchoredTarget(st, kAnchorP, anchorQ(), Eigen::Vector3d::Zero(), q0);
  for (int i = 1; i <= 20; ++i) {
    Eigen::Quaterniond q = q0 * quat(0.05 * i, Eigen::Vector3d(1, 0, 0));
    if (i % 2 == 0) q.coeffs() = -q.coeffs();  // driver reports -q on alternate samples
    const Pose out = computeAnchoredTarget(st, kAnchorP, anchorQ(), Eigen::Vector3d::Zero(), q);
    EXPECT_GE(prev.orientation.dot(out.orientation), 0.0) << "sample " << i;
    EXPECT_NEAR(rotAngle(out.orientation, anchorQ() * quat(0.05 * i, Eigen::Vector3d(1, 0, 0))), 0.0, 1e-9);
    prev = out;
  }
}

// (6) Non-unit quaternions are rejected.
TEST(AnchoredTarget, NonUnitQuaternionRejected) {
  State st;
  const Eigen::Quaterniond non_unit(2.0, 0.0, 0.0, 0.0);
  EXPECT_THROW(computeAnchoredTarget(st, kAnchorP, non_unit, Eigen::Vector3d::Zero(),
                                     Eigen::Quaterniond::Identity()),
               std::invalid_argument);
  EXPECT_THROW(computeAnchoredTarget(st, kAnchorP, anchorQ(), Eigen::Vector3d::Zero(), non_unit),
               std::invalid_argument);
  EXPECT_THROW(parseAnchorOrientation({0.0, 0.0, 0.0, 1.001}, "anchor_orientation_xyzw"),
               std::invalid_argument);
  EXPECT_NO_THROW(parseAnchorOrientation({0.0, 0.0, 0.0, 1.0}, "anchor_orientation_xyzw"));
}

// Parameter parsing: wrong length is rejected and the message names the parameter.
TEST(AnchoredTarget, WrongLengthRejectedWithParameterName) {
  try {
    parseAnchorPosition({1.0, 2.0}, "anchor_position_m");
    FAIL() << "expected throw";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("anchor_position_m"), std::string::npos);
  }
  try {
    parseAnchorOrientation({0.0, 0.0, 1.0}, "anchor_orientation_xyzw");
    FAIL() << "expected throw";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("anchor_orientation_xyzw"), std::string::npos);
  }
}
