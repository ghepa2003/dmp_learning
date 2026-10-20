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

TEST(GraspPlannerChecksTest, OmegaConsistencyMatchPasses) {
    EXPECT_NO_THROW(
        checkOmegaConsistency(Eigen::Vector3d(0, 0, -0.0349), Eigen::Vector3d(0, 0, 1), -0.0349, 1e-3));
}

TEST(GraspPlannerChecksTest, OmegaConsistencyMismatchThrows) {
    EXPECT_THROW(
        checkOmegaConsistency(Eigen::Vector3d(0, 0, 0.0349), Eigen::Vector3d(0, 0, 1), -0.0349, 1e-3),
        std::runtime_error);
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
