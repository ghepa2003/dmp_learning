// Tests for the pure (no rclcpp::init needed) startup/runtime validation helpers used by
// grasp_planner_node - see grasp_planner_checks.hpp.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "satellite_grasp_planner/ros/grasp_planner_checks.hpp"

using namespace satellite_grasp_planner::ros_wrapper;

TEST(GraspPlannerChecksTest, RobotBaseWorldZeroPasses) {
    EXPECT_NO_THROW(checkRobotBaseWorldIsZero(Eigen::Vector3d::Zero(), 1e-6));
}

TEST(GraspPlannerChecksTest, RobotBaseWorldNonZeroThrows) {
    EXPECT_THROW(checkRobotBaseWorldIsZero(Eigen::Vector3d(0.01, 0.0, 0.0), 1e-6), std::invalid_argument);
}

TEST(GraspPlannerChecksTest, CubeAxisMatchesRotationAxisPasses) {
    // Different magnitudes, same direction - only the normalized axis matters.
    EXPECT_NO_THROW(checkCubeAxisMatchesRotationAxis(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 2)));
}

TEST(GraspPlannerChecksTest, CubeAxisMismatchThrows) {
    EXPECT_THROW(checkCubeAxisMatchesRotationAxis(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 1, 0)),
                std::invalid_argument);
}

TEST(GraspPlannerChecksTest, CubeCenterMatchesRotationCenterPasses) {
    EXPECT_NO_THROW(
        checkCubeCenterMatchesRotationCenter(Eigen::Vector3d(0.75, 0.0, 0.35), Eigen::Vector3d(0.75, 0.0, 0.35)));
}

TEST(GraspPlannerChecksTest, CubeCenterMismatchThrows) {
    EXPECT_THROW(
        checkCubeCenterMatchesRotationCenter(Eigen::Vector3d(0.75, 0.0, 0.35), Eigen::Vector3d(0.0, 0.0, 0.0)),
        std::invalid_argument);
}

TEST(GraspPlannerChecksTest, QRefIdentityOnAnyAxisPasses) {
    EXPECT_NO_THROW(checkQRefOnAxis(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0, 0, 1), 0.5));
    EXPECT_NO_THROW(checkQRefOnAxis(Eigen::Quaterniond::Identity(), Eigen::Vector3d(1, 0, 0), 0.5));
}

TEST(GraspPlannerChecksTest, QRefOnAxisPasses) {
    const Eigen::Quaterniond q(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
    EXPECT_NO_THROW(checkQRefOnAxis(q, Eigen::Vector3d(0, 0, 1), 0.5));
}

TEST(GraspPlannerChecksTest, QRefOffAxisThrows) {
    const Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()));
    EXPECT_THROW(checkQRefOnAxis(q, Eigen::Vector3d(0, 0, 1), 0.5), std::invalid_argument);
}

TEST(GraspPlannerChecksTest, FirstOdomSampleValidPasses) {
    EXPECT_NO_THROW(checkFirstOdomSample(Eigen::Vector3d(0, 0, 0), "world", Eigen::Vector3d(0, 0, 0)));
}

TEST(GraspPlannerChecksTest, FirstOdomSampleWrongFrameThrows) {
    EXPECT_THROW(checkFirstOdomSample(Eigen::Vector3d(0, 0, 0), "map", Eigen::Vector3d(0, 0, 0)),
                std::runtime_error);
}

TEST(GraspPlannerChecksTest, FirstOdomSampleTooFarThrows) {
    EXPECT_THROW(checkFirstOdomSample(Eigen::Vector3d(0.1, 0, 0), "world", Eigen::Vector3d(0, 0, 0)),
                std::runtime_error);
}

namespace {
Eigen::Vector3d tiltAxis(double alpha_deg, double phi_deg) {
    const double a = alpha_deg * M_PI / 180.0, p = phi_deg * M_PI / 180.0;
    return Eigen::Vector3d(std::sin(a) * std::cos(p), std::sin(a) * std::sin(p), std::cos(a));
}
Eigen::Quaterniond rotAbout(const Eigen::Vector3d& axis, double angle) {
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis.normalized()));
}
// Odometry twist.angular as Gazebo publishes it (G28): d(roll, pitch, yaw)/dt of the pose quaternion.
Eigen::Vector3d gazeboTwistAngular() { return Eigen::Vector3d(-7.34e-5, -0.011939, -0.032802); }
constexpr double kOmegaTol = 0.00174533;  // rad/s, 0.1 deg/s: the planner's omega_consistency_tol
}  // namespace

TEST(GraspPlannerChecksTest, OmegaConsistencyMatchPasses) {
    const Eigen::Vector3d z(0, 0, 1);
    EXPECT_NO_THROW(checkOmegaConsistency(Eigen::Quaterniond::Identity(), rotAbout(z, -0.0349 * 0.005), 0.005, z,
                                          -0.0349, 1e-3));
}

TEST(GraspPlannerChecksTest, OmegaConsistencyMismatchThrows) {
    const Eigen::Vector3d z(0, 0, 1);
    EXPECT_THROW(checkOmegaConsistency(Eigen::Quaterniond::Identity(), rotAbout(z, 0.0349 * 0.005), 0.005, z,
                                       -0.0349, 1e-3),
                 std::runtime_error);
}

TEST(GraspPlannerChecksTest, AxisRateTiltedAxisRecoversOmega) {
    const double om = -2.0 * M_PI / 180.0, dt = 0.005;
    for (const auto& t : {std::pair<double, double>{20.0, 0.0}, {20.0, 270.0}, {20.0, 33.0}, {7.0, 359.0}}) {
        const Eigen::Vector3d a = tiltAxis(t.first, t.second);
        const Eigen::Quaterniond q0 = rotAbout(Eigen::Vector3d(0.3, -0.5, 0.2), 0.7);  // arbitrary orientation
        const Eigen::Quaterniond q1 = rotAbout(a, om * dt) * q0;                       // world-frame increment
        const auto r = axisRateFromQuaternions(q0, q1, dt, a);
        ASSERT_TRUE(r.has_value());
        EXPECT_NEAR(*r, om, 1e-9);
        EXPECT_NO_THROW(checkOmegaConsistency(q0, q1, dt, a, om, kOmegaTol));
    }
}

// Regression (G28 config B, alpha = 20, phi = 0): the odometry twist is d(rpy)/dt, its projection on a is wrong
// (-0.030849 vs -0.034907: 0.0041 rad/s > tol) while the quaternions rotate exactly at omega about a.
TEST(GraspPlannerChecksTest, OmegaConsistencyRegressionG28TwistIsDerivativeOfRpy) {
    const Eigen::Vector3d a = tiltAxis(20.0, 0.0);
    const double om = -2.0 * M_PI / 180.0, dt = 0.005;
    EXPECT_NEAR(gazeboTwistAngular().dot(a), -0.030849, 1e-6);       // what the old check measured
    EXPECT_GT(std::fabs(gazeboTwistAngular().dot(a) - om), kOmegaTol);  // ... and would have rejected
    const Eigen::Quaterniond q0 = rotAbout(Eigen::Vector3d(1, 0, 0), -0.349);
    EXPECT_NO_THROW(checkOmegaConsistency(q0, rotAbout(a, om * dt) * q0, dt, a, om, kOmegaTol));
}

TEST(GraspPlannerChecksTest, AxisRateVerticalAxisMatchesWz) {
    const Eigen::Vector3d z(0, 0, 1);
    const double dt = 0.01;
    for (double w : {-0.0349066, 0.0349066, 0.5, -1.2}) {
        const auto r = axisRateFromQuaternions(rotAbout(z, 0.4), rotAbout(z, 0.4 + w * dt), dt, z);
        ASSERT_TRUE(r.has_value());
        EXPECT_NEAR(*r, w, 1e-12);
    }
}

TEST(GraspPlannerChecksTest, AxisRateEdgeCases) {
    const Eigen::Vector3d a = tiltAxis(20.0, 33.0);
    const double om = -0.0349066, dt = 0.005;
    const Eigen::Quaterniond q0 = rotAbout(Eigen::Vector3d(0.1, 0.2, 0.3), 1.0);
    const Eigen::Quaterniond q1 = rotAbout(a, om * dt) * q0;
    // dt zero / negative / NaN: no estimate; the checking wrapper rejects the pair
    EXPECT_FALSE(axisRateFromQuaternions(q0, q1, 0.0, a).has_value());
    EXPECT_FALSE(axisRateFromQuaternions(q0, q1, -dt, a).has_value());
    EXPECT_FALSE(axisRateFromQuaternions(q0, q1, std::numeric_limits<double>::quiet_NaN(), a).has_value());
    EXPECT_THROW(checkOmegaConsistency(q0, q1, 0.0, a, om, kOmegaTol), std::invalid_argument);
    // opposite quaternion sign (same rotation), either sample
    const Eigen::Quaterniond neg1(-q1.coeffs());
    const Eigen::Quaterniond neg0(-q0.coeffs());
    EXPECT_NEAR(*axisRateFromQuaternions(q0, neg1, dt, a), om, 1e-9);
    EXPECT_NEAR(*axisRateFromQuaternions(neg0, q1, dt, a), om, 1e-9);
    // not normalized
    const Eigen::Quaterniond big(q1.coeffs() * 3.0), small(q0.coeffs() * 0.25);
    EXPECT_NEAR(*axisRateFromQuaternions(small, big, dt, a), om, 1e-9);
    EXPECT_FALSE(axisRateFromQuaternions(Eigen::Quaterniond(0, 0, 0, 0), q1, dt, a).has_value());
    // wrap: increment straddling a rotation of pi about the axis (angle of q1 near 180 deg, w crosses zero)
    const Eigen::Quaterniond p0 = rotAbout(a, M_PI - 0.5 * std::fabs(om) * dt);
    const Eigen::Quaterniond p1 = rotAbout(a, M_PI + 0.5 * std::fabs(om) * dt);
    EXPECT_NEAR(*axisRateFromQuaternions(p0, p1, dt, a), std::fabs(om), 1e-9);
    EXPECT_NEAR(*axisRateFromQuaternions(p1, p0, dt, a), -std::fabs(om), 1e-9);
    // identical orientation -> 0; degenerate axis throws
    EXPECT_NEAR(*axisRateFromQuaternions(q0, q0, dt, a), 0.0, 1e-15);
    EXPECT_THROW(axisRateFromQuaternions(q0, q1, dt, Eigen::Vector3d::Zero()), std::invalid_argument);
}

// ---------------------------------------------------------------------------------------
// Tilted satellite: q_ref relative to the theta = 0 orientation R_base
// ---------------------------------------------------------------------------------------
namespace {
Eigen::Quaterniond tiltQuat(double alpha_deg, double beta_deg) {
    const double a = alpha_deg * M_PI / 180.0, b = beta_deg * M_PI / 180.0;
    return Eigen::Quaterniond(Eigen::AngleAxisd(a, Eigen::Vector3d(-std::sin(b), std::cos(b), 0.0)));
}
}  // namespace

TEST(GraspPlannerChecksTest, QRefWithIdentityBaseIsTheHistoricalCheck) {
    const Eigen::Quaterniond I = Eigen::Quaterniond::Identity();
    EXPECT_NO_THROW(checkQRefOnAxis(I, Eigen::Vector3d(0, 0, 1), 0.5, I));
    EXPECT_NO_THROW(checkQRefOnAxis(Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ())),
                                    Eigen::Vector3d(0, 0, 1), 0.5, I));
    EXPECT_THROW(checkQRefOnAxis(Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX())),
                                 Eigen::Vector3d(0, 0, 1), 0.5, I),
                 std::invalid_argument);
}

TEST(GraspPlannerChecksTest, QRefAcceptsRTiltTimesRotationAboutTheTiltedAxis) {
    for (const auto& t : {std::pair<double, double>{5.0, 0.0}, {20.0, 33.0}, {45.0, -120.0}, {80.0, 250.0}}) {
        const Eigen::Quaterniond R_tilt = tiltQuat(t.first, t.second);
        const Eigen::Vector3d axis = R_tilt * Eigen::Vector3d::UnitZ();
        for (const double phi : {0.0, 0.4, -2.0}) {
            const Eigen::Quaterniond q_ref = R_tilt * Eigen::Quaterniond(Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitZ()));
            EXPECT_NO_THROW(checkQRefOnAxis(q_ref, axis, 0.5, R_tilt)) << t.first << " " << t.second << " " << phi;
            // equivalently Rot(a, phi) R_tilt
            const Eigen::Quaterniond q_ref2 = Eigen::Quaterniond(Eigen::AngleAxisd(phi, axis)) * R_tilt;
            EXPECT_NO_THROW(checkQRefOnAxis(q_ref2, axis, 0.5, R_tilt));
        }
        // Without the base the same (correct) q_ref is rejected: it is not a rotation about the tilted axis.
        EXPECT_THROW(checkQRefOnAxis(R_tilt, axis, 0.5), std::invalid_argument);
    }
}

TEST(GraspPlannerChecksTest, QRefOffAxisIsRejectedWithABase) {
    const Eigen::Quaterniond R_tilt = tiltQuat(20.0, 33.0);
    const Eigen::Vector3d axis = R_tilt * Eigen::Vector3d::UnitZ();
    // q_ref = R_tilt composed with a 10 deg rotation about an axis orthogonal to the spin axis
    const Eigen::Vector3d ortho = axis.unitOrthogonal();
    const Eigen::Quaterniond bad = Eigen::Quaterniond(Eigen::AngleAxisd(10.0 * M_PI / 180.0, ortho)) * R_tilt;
    EXPECT_THROW(checkQRefOnAxis(bad, axis, 0.5, R_tilt), std::invalid_argument);
    // identity q_ref with a non-identity base is also off-axis
    EXPECT_THROW(checkQRefOnAxis(Eigen::Quaterniond::Identity(), axis, 0.5, R_tilt), std::invalid_argument);
}

TEST(GraspPlannerChecksTest, SatelliteBaseOrientationIsExactlyIdentityForTheVerticalCase) {
    const Eigen::Quaterniond q = satelliteBaseOrientation(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitZ());
    EXPECT_EQ(q.coeffs(), Eigen::Quaterniond::Identity().coeffs());  // bit-exact: the historical check path is used
    // unnormalized inputs, same result
    EXPECT_EQ(satelliteBaseOrientation(Eigen::Vector3d(3, 0, 0), Eigen::Vector3d(0, 0, 2)).coeffs(),
              Eigen::Quaterniond::Identity().coeffs());
}

TEST(GraspPlannerChecksTest, SatelliteBaseOrientationOfATiltedSatelliteIsRTilt) {
    for (const auto& t : {std::pair<double, double>{5.0, 0.0}, {20.0, 33.0}, {45.0, -120.0}, {80.0, 250.0}}) {
        const Eigen::Quaterniond R_tilt = tiltQuat(t.first, t.second);
        const Eigen::Quaterniond base =
            satelliteBaseOrientation(R_tilt * Eigen::Vector3d::UnitX(), R_tilt * Eigen::Vector3d::UnitZ());
        EXPECT_LT((base.toRotationMatrix() - R_tilt.toRotationMatrix()).norm(), 1e-12);
    }
}

TEST(GraspPlannerChecksTest, SatelliteBaseOrientationRejectsBadInputs) {
    EXPECT_THROW(satelliteBaseOrientation(Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ()), std::invalid_argument);  // not lateral
    EXPECT_THROW(satelliteBaseOrientation(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ()), std::invalid_argument);
    EXPECT_THROW(satelliteBaseOrientation(Eigen::Vector3d::UnitX(), Eigen::Vector3d(0, 0, NAN)), std::invalid_argument);
}

// For a vertical axis ANY lateral face normal gives a base that is a rotation about z, so the 4-argument check agrees
// with the 3-argument one for every q_ref (on-axis stays on-axis, off-axis stays off-axis).
TEST(GraspPlannerChecksTest, VerticalAxisWithAnyLateralNormalAgreesWithTheHistoricalCheck) {
    const Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
    const Eigen::Quaterniond on_axis(Eigen::AngleAxisd(0.3, z));
    const Eigen::Quaterniond off_axis(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()));
    for (const Eigen::Vector3d& n : {Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(-1, 1, 0)}) {
        const Eigen::Quaterniond base = satelliteBaseOrientation(n, z);
        EXPECT_NO_THROW(checkQRefOnAxis(on_axis, z, 0.5, base));
        EXPECT_NO_THROW(checkQRefOnAxis(Eigen::Quaterniond::Identity(), z, 0.5, base));
        EXPECT_THROW(checkQRefOnAxis(off_axis, z, 0.5, base), std::invalid_argument);
    }
}
