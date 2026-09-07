#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "haptic_dmp_learning/core/frame_correction.hpp"

using namespace haptic_dmp_learning::core::frame_correction;

/**
 * @brief Isolated tests for the fixed Geomagic (omni_base) -> Franka (fer_link0)
 *        base-frame correction R_z(+90 deg).
 *
 * The x-axis mapping is the one verified on hardware (Geomagic x -> Franka y);
 * the y/z mappings are assumed from "pure +90 deg rotation about z" and are
 * flagged below as awaiting a hardware check.
 */

// --- rotatePosition: axis mappings -----------------------------------------

// VERIFIED ON HARDWARE: moving the Geomagic along its own x axis moves the
// robot along its own x axis, i.e. Geomagic x must map onto Franka y.
TEST(FrameCorrection, RotatePositionMapsGeomagicXToFrankaY) {
    const Eigen::Vector3d out = rotatePosition(Eigen::Vector3d(1.0, 0.0, 0.0));
    EXPECT_NEAR(out.x(), 0.0, 1e-12);
    EXPECT_NEAR(out.y(), 1.0, 1e-12);
    EXPECT_NEAR(out.z(), 0.0, 1e-12);
}

// ASSUMED, IN ATTESA DI VERIFICA HARDWARE: follows only from the offset being a
// pure +90 deg rotation about z. Geomagic y -> Franka -x.
TEST(FrameCorrection, RotatePositionMapsGeomagicYToFrankaNegativeX) {
    const Eigen::Vector3d out = rotatePosition(Eigen::Vector3d(0.0, 1.0, 0.0));
    EXPECT_NEAR(out.x(), -1.0, 1e-12);
    EXPECT_NEAR(out.y(), 0.0, 1e-12);
    EXPECT_NEAR(out.z(), 0.0, 1e-12);
}

// ASSUMED, IN ATTESA DI VERIFICA HARDWARE: z is invariant under R_z, so the
// Geomagic z axis is expected to map onto the Franka z axis unchanged.
TEST(FrameCorrection, RotatePositionLeavesZAxisUnchanged) {
    const Eigen::Vector3d out = rotatePosition(Eigen::Vector3d(0.0, 0.0, 1.0));
    EXPECT_NEAR(out.x(), 0.0, 1e-12);
    EXPECT_NEAR(out.y(), 0.0, 1e-12);
    EXPECT_NEAR(out.z(), 1.0, 1e-12);
}

// --- rotatePosition: isometry --------------------------------------------

// A rotation is an isometry: it must preserve the norm of an arbitrary vector
// that is not aligned with any axis.
TEST(FrameCorrection, RotatePositionPreservesNorm) {
    const Eigen::Vector3d p(0.37, -1.42, 0.85);
    const Eigen::Vector3d out = rotatePosition(p);
    EXPECT_NEAR(out.norm(), p.norm(), 1e-12);
    // Consistency with the closed-form R_z(+90 deg): (x, y, z) -> (-y, x, z).
    EXPECT_NEAR(out.x(), -p.y(), 1e-12);
    EXPECT_NEAR(out.y(), p.x(), 1e-12);
    EXPECT_NEAR(out.z(), p.z(), 1e-12);
}

// --- rotateOrientation --------------------------------------------------

// Re-expressing the identity orientation yields exactly the frame offset
// quaternion q_offset (w = z = sqrt(2)/2, x = y = 0).
TEST(FrameCorrection, RotateOrientationOfIdentityIsFrameOffset) {
    const Eigen::Quaterniond out = rotateOrientation(Eigen::Quaterniond::Identity());
    const double s = std::sqrt(2.0) / 2.0;
    // angularDistance is sign-agnostic (q and -q are the same rotation).
    EXPECT_NEAR(out.angularDistance(frameOffsetQuaternion()), 0.0, 1e-12);
    EXPECT_NEAR(std::abs(out.w()), s, 1e-12);
    EXPECT_NEAR(out.x(), 0.0, 1e-12);
    EXPECT_NEAR(out.y(), 0.0, 1e-12);
    EXPECT_NEAR(std::abs(out.z()), s, 1e-12);
}

// A unit quaternion in must give a unit quaternion out (composition of two
// unit quaternions stays on S^3).
TEST(FrameCorrection, RotateOrientationPreservesUnitNorm) {
    Eigen::Quaterniond q(Eigen::AngleAxisd(0.9, Eigen::Vector3d(0.2, -0.5, 0.84).normalized()));
    q.normalize();
    const Eigen::Quaterniond out = rotateOrientation(q);
    EXPECT_NEAR(out.norm(), 1.0, 1e-12);
}

// The offset quaternion itself is a unit quaternion.
TEST(FrameCorrection, FrameOffsetQuaternionIsUnit) {
    EXPECT_NEAR(frameOffsetQuaternion().norm(), 1.0, 1e-12);
}
