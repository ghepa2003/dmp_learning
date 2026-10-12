// Pure tests for the satellite surface-velocity helpers (no ROS). See core/surface_velocity.hpp.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include "haptic_dmp_learning/core/surface_velocity.hpp"

using namespace haptic_dmp_learning::core;

namespace {

const double kOmega = 2.0 * M_PI / 180.0;  // 0.0349066 rad/s
const std::vector<GraspPointId> kAllPoints = {GraspPointId::kP0, GraspPointId::kP90,
                                              GraspPointId::kP180, GraspPointId::kP270};

void expectVecNear(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double tol) {
    for (int i = 0; i < 3; ++i) EXPECT_NEAR(a(i), b(i), tol) << "component " << i;
}

// Real satellite (CAD), explicit so the hand values below do not follow the defaults:
// center 0, axis +Z, n = +X, side 0.150, standoff 0.08, outer radius 0.0725, wall 0.009, grasp
// radius 0.068 (mid-wall). Rim distance from the face centre line: 0.075 + 0.08 = 0.155 m.
CubeSatelliteModel::Params realSatelliteParams() {
    CubeSatelliteModel::Params p;
    p.cube_side_m = 0.150;
    p.standoff_m = 0.08;
    p.collar_radius_m = 0.0725;
    p.wall_thickness_m = 0.009;
    p.grasp_radius_m = 0.068;
    return p;
}

// Distances of the grasp points from the rotation axis (+Z through the origin):
//   kP90/kP270 (on +Z / -Z of the collar): 0.075 + 0.08                = 0.155 m
//   kP0/kP180  (lateral, +-Y):             sqrt(0.155^2 + 0.068^2)     = 0.1693 m
const double kRimDist = 0.075 + 0.08;
const double kRimDistLateral = std::sqrt(0.155 * 0.155 + 0.068 * 0.068);

}  // namespace

// Real-satellite params (above): at theta = 0
// kP0=(0.155,-0.068,0), kP90=(0.155,0,0.068), kP180=(0.155,0.068,0), kP270=(0.155,0,-0.068).
// Speeds |v| = omega * distance from the axis: kOmega * 0.155 = 5.41 mm/s (kP90/kP270),
// kOmega * 0.1693 = 5.91 mm/s (kP0/kP180).
TEST(SurfaceVelocity, MagnitudesDirectionsAndSign) {
    const CubeSatelliteModel m{realSatelliteParams()};
    const double r_axial = kRimDistLateral;

    EXPECT_NEAR(surfaceVelocity(m, GraspPointId::kP90, 0.0, kOmega).norm(), kOmega * kRimDist, 1e-12);
    EXPECT_NEAR(surfaceVelocity(m, GraspPointId::kP270, 0.0, kOmega).norm(), kOmega * kRimDist, 1e-12);
    EXPECT_NEAR(surfaceVelocity(m, GraspPointId::kP0, 0.0, kOmega).norm(), kOmega * r_axial, 1e-12);
    EXPECT_NEAR(surfaceVelocity(m, GraspPointId::kP180, 0.0, kOmega).norm(), kOmega * r_axial, 1e-12);

    // v is perpendicular to the axis and to the radius vector (point - center).
    for (GraspPointId k : kAllPoints) {
        const Eigen::Vector3d v = surfaceVelocity(m, k, 0.0, kOmega);
        const Eigen::Vector3d r = m.graspPoseAt(k, 0.0).position_world - m.cubeCenterWorld();
        EXPECT_NEAR(v.dot(m.axisWorld()), 0.0, 1e-12);
        EXPECT_NEAR(v.dot(r), 0.0, 1e-12);
    }

    // Clockwise (omega < 0): kP90/kP270 at x = +0.155 move along -Y; counter-clockwise: +Y.
    for (GraspPointId k : {GraspPointId::kP90, GraspPointId::kP270}) {
        expectVecNear(surfaceVelocity(m, k, 0.0, -kOmega), Eigen::Vector3d(0.0, -kOmega * kRimDist, 0.0),
                      1e-12);
        expectVecNear(surfaceVelocity(m, k, 0.0, kOmega), Eigen::Vector3d(0.0, kOmega * kRimDist, 0.0),
                      1e-12);
        expectVecNear(surfaceVelocity(m, k, 0.0, kOmega), -surfaceVelocity(m, k, 0.0, -kOmega), 1e-15);
    }
}

TEST(SurfaceVelocity, MagnitudeIndependentOfTheta) {
    const CubeSatelliteModel m{CubeSatelliteModel::Params{}};
    for (GraspPointId k : kAllPoints) {
        const double ref = surfaceVelocity(m, k, 0.0, kOmega).norm();
        for (double theta : {0.4, -1.3, 2.2, 3.0, -2.9}) {
            EXPECT_NEAR(surfaceVelocity(m, k, theta, kOmega).norm(), ref, 1e-12) << theta;
        }
    }
}

TEST(SurfaceVelocity, MeanSquaredMatchesHandComputation) {
    const CubeSatelliteModel m{realSatelliteParams()};
    // Two points at |v|^2 = w^2 * 0.155^2, two at w^2 * (0.155^2 + 0.068^2) (= 0.1693^2 w^2):
    // w^2 * (2 * 0.155^2 + 2 * 0.1693^2) / 4 = 3.209e-5 m^2/s^2.
    const double expected =
        (2.0 * kOmega * kOmega * 0.155 * 0.155 +
         2.0 * kOmega * kOmega * (0.155 * 0.155 + 0.068 * 0.068)) / 4.0;
    EXPECT_NEAR(expected, 3.209e-5, 1e-8);  // guards the hand arithmetic quoted above
    EXPECT_NEAR(meanSurfaceSpeedSquared(m, kOmega), expected, 1e-15);
    EXPECT_NEAR(meanSurfaceSpeedSquared(m, -kOmega), expected, 1e-15);
}
