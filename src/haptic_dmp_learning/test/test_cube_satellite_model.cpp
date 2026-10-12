// Pure-geometry tests for the cube satellite model (no ROS).
// See include/haptic_dmp_learning/core/cube_satellite_model.hpp.

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>

#include "haptic_dmp_learning/core/cube_satellite_model.hpp"

using namespace haptic_dmp_learning::core;

namespace {

double deg(double d) { return d * M_PI / 180.0; }

const std::vector<GraspPointId> kAllPoints = {GraspPointId::kP0, GraspPointId::kP90,
                                              GraspPointId::kP180, GraspPointId::kP270};

void expectVecNear(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double tol) {
    for (int i = 0; i < 3; ++i) EXPECT_NEAR(a(i), b(i), tol) << "component " << i;
}

/// Non-trivial params: off-origin center, tilted axis, face normal perpendicular to it.
CubeSatelliteModel::Params generalParams() {
    CubeSatelliteModel::Params p;
    p.center_world = Eigen::Vector3d(0.45, -0.05, 0.35);
    p.axis_world = Eigen::Vector3d(1.0, 2.0, 3.0);  // not normalized on purpose
    p.face_normal_body = Eigen::Vector3d(2.0, -1.0, 0.0);  // (1,2,3).(2,-1,0) = 0
    return p;
}

}  // namespace

// ---------------------------------------------------------------------------------------
// Analytical positions
// ---------------------------------------------------------------------------------------
TEST(CubeSatelliteModelPositions, ThetaZeroDefaultParamsMatchAnalytical) {
    // n = +X, axis = +Z -> u = n x axis = -Y, v = +Z. Face+standoff offset = 0.075 + 0.08 = 0.155;
    // grasp_radius_m unset -> grasp radius = collar_radius_m = 0.0725.
    const CubeSatelliteModel m{CubeSatelliteModel::Params{}};
    expectVecNear(m.graspPoseAt(GraspPointId::kP0, 0.0).position_world, {0.155, -0.0725, 0.0}, 1e-12);
    expectVecNear(m.graspPoseAt(GraspPointId::kP90, 0.0).position_world, {0.155, 0.0, 0.0725}, 1e-12);
    expectVecNear(m.graspPoseAt(GraspPointId::kP180, 0.0).position_world, {0.155, 0.0725, 0.0}, 1e-12);
    expectVecNear(
        m.graspPoseAt(GraspPointId::kP270, 0.0).position_world, {0.155, 0.0, -0.0725}, 1e-12);
}

// ---------------------------------------------------------------------------------------
// Collar invariants
// ---------------------------------------------------------------------------------------
TEST(CubeSatelliteModelInvariants, PointsEquidistantFromCollarCenterAndAt90Degrees) {
    CubeSatelliteModel::Params par = generalParams();
    par.grasp_radius_m = par.collar_radius_m;  // explicit: this test checks the grasp radius
    const CubeSatelliteModel m(par);
    const Eigen::Vector3d n = par.face_normal_body.normalized();
    const Eigen::Vector3d axis = par.axis_world.normalized();
    for (double theta : {0.0, deg(30.0), deg(145.0), deg(-200.0)}) {
        const Eigen::Vector3d collar_center =
            par.center_world +
            Eigen::AngleAxisd(theta, axis) * ((par.cube_side_m / 2.0 + par.standoff_m) * n);
        std::vector<Eigen::Vector3d> d;
        for (auto k : kAllPoints) {
            d.push_back(m.graspPoseAt(k, theta).position_world - collar_center);
            EXPECT_NEAR(d.back().norm(), *par.grasp_radius_m, 1e-12);
        }
        for (size_t i = 0; i < d.size(); ++i) {
            const Eigen::Vector3d& a = d[i];
            const Eigen::Vector3d& b = d[(i + 1) % d.size()];
            EXPECT_NEAR(a.dot(b), 0.0, 1e-12) << "adjacent points not at 90 deg, theta=" << theta;
            expectVecNear(a, -d[(i + 2) % d.size()], 1e-12);  // opposite points at 180 deg
        }
    }
}

// ---------------------------------------------------------------------------------------
// Orientation
// ---------------------------------------------------------------------------------------
TEST(CubeSatelliteModelOrientation, ProperRotationForEveryPoint) {
    const CubeSatelliteModel m(generalParams());
    for (double theta : {0.0, deg(45.0), deg(180.0), deg(-77.0)}) {
        for (auto k : kAllPoints) {
            const Eigen::Quaterniond q = m.graspPoseAt(k, theta).orientation_nominal_world;
            EXPECT_NEAR(q.norm(), 1.0, 1e-12);
            const Eigen::Matrix3d R = q.toRotationMatrix();
            EXPECT_NEAR(R.determinant(), 1.0, 1e-12);
            const Eigen::Matrix3d RtR = R.transpose() * R;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    EXPECT_NEAR(RtR(i, j), i == j ? 1.0 : 0.0, 1e-12) << i << "," << j;
        }
    }
}

TEST(CubeSatelliteModelOrientation, OrientationColumnsMatchDefinition) {
    // Default params, theta = 0, k = P0: z = -X, y = -r_0 = -u = +Y, x = y x z = +Y x -X = +Z.
    const CubeSatelliteModel m{CubeSatelliteModel::Params{}};
    const Eigen::Matrix3d R =
        m.graspPoseAt(GraspPointId::kP0, 0.0).orientation_nominal_world.toRotationMatrix();
    expectVecNear(R.col(0), {0.0, 0.0, 1.0}, 1e-12);
    expectVecNear(R.col(1), {0.0, 1.0, 0.0}, 1e-12);
    expectVecNear(R.col(2), {-1.0, 0.0, 0.0}, 1e-12);
}

TEST(CubeSatelliteModelOrientation, ApproachAxisIndependentOfPoint) {
    const CubeSatelliteModel m(generalParams());
    for (double theta : {0.0, deg(45.0), deg(180.0)}) {
        const Eigen::Vector3d z_ref = m.graspPoseAt(GraspPointId::kP0, theta).approach_axis_world;
        expectVecNear(z_ref, -m.faceNormalWorld(theta), 1e-12);
        for (auto k : kAllPoints) {
            const GraspTargetPose p = m.graspPoseAt(k, theta);
            expectVecNear(p.approach_axis_world, z_ref, 1e-12);
            // Consistent with the quaternion's z column.
            expectVecNear(p.orientation_nominal_world.toRotationMatrix().col(2), z_ref, 1e-12);
        }
    }
}

// ---------------------------------------------------------------------------------------
// Construction validation
// ---------------------------------------------------------------------------------------
TEST(CubeSatelliteModelConstruction, RejectsFaceNormalParallelToAxis) {
    CubeSatelliteModel::Params p;
    p.face_normal_body = Eigen::Vector3d::UnitZ();  // parallel
    EXPECT_THROW(CubeSatelliteModel{p}, std::invalid_argument);
    p.face_normal_body = -Eigen::Vector3d::UnitZ();  // anti-parallel
    EXPECT_THROW(CubeSatelliteModel{p}, std::invalid_argument);
    p.face_normal_body = Eigen::Vector3d(1e-3, 0.0, 1.0);  // nearly parallel
    EXPECT_THROW(CubeSatelliteModel{p}, std::invalid_argument);
}

TEST(CubeSatelliteModelConstruction, RejectsZeroNormVectors) {
    CubeSatelliteModel::Params p;
    p.axis_world = Eigen::Vector3d::Zero();
    EXPECT_THROW(CubeSatelliteModel{p}, std::invalid_argument);
    p = CubeSatelliteModel::Params{};
    p.face_normal_body = Eigen::Vector3d(1e-12, 0.0, 0.0);
    EXPECT_THROW(CubeSatelliteModel{p}, std::invalid_argument);
}

TEST(CubeSatelliteModelConstruction, AcceptsPerpendicularNonUnitInputs) {
    EXPECT_NO_THROW(CubeSatelliteModel{generalParams()});
}

// ---------------------------------------------------------------------------------------
// Rigid rotation with theta
// ---------------------------------------------------------------------------------------
TEST(CubeSatelliteModelRotation, PositionRotatesRigidlyAboutAxis) {
    const CubeSatelliteModel::Params par = generalParams();
    const CubeSatelliteModel m(par);
    const Eigen::Vector3d axis = par.axis_world.normalized();
    for (double theta : {deg(45.0), deg(180.0), deg(-120.0)}) {
        for (auto k : kAllPoints) {
            const Eigen::Vector3d r0 = m.graspPoseAt(k, 0.0).position_world - par.center_world;
            const Eigen::Vector3d rt = m.graspPoseAt(k, theta).position_world - par.center_world;
            EXPECT_NEAR(rt.norm(), r0.norm(), 1e-12);
            EXPECT_NEAR(rt.dot(axis), r0.dot(axis), 1e-12);  // axial component unchanged
            expectVecNear(rt, Eigen::AngleAxisd(theta, axis) * r0, 1e-12);
            // Signed angle about the axis between the perpendicular components equals theta.
            const Eigen::Vector3d a = r0 - r0.dot(axis) * axis;
            const Eigen::Vector3d b = rt - rt.dot(axis) * axis;
            const double ang = std::atan2(axis.dot(a.cross(b)), a.dot(b));
            EXPECT_NEAR(std::cos(ang), std::cos(theta), 1e-12);
            EXPECT_NEAR(std::sin(ang), std::sin(theta), 1e-12);
        }
    }
}

TEST(CubeSatelliteModelRotation, ThetaPiFlipsDefaultPointsThroughCenter) {
    // Default: axis Z, so theta = 180 deg negates the x/y components and keeps z.
    const CubeSatelliteModel m{CubeSatelliteModel::Params{}};
    expectVecNear(
        m.graspPoseAt(GraspPointId::kP90, M_PI).position_world, {-0.155, 0.0, 0.0725}, 1e-12);
    expectVecNear(m.faceNormalWorld(M_PI), {-1.0, 0.0, 0.0}, 1e-12);
}

// ---------------------------------------------------------------------------------------
// Construction checks on the collar geometry
// ---------------------------------------------------------------------------------------
TEST(CubeSatelliteModelCollar, ConstructorRejectsInvalidWallThickness) {
    CubeSatelliteModel::Params p;
    p.wall_thickness_m = 0.0;
    EXPECT_THROW(CubeSatelliteModel{p}, std::invalid_argument);
    p.wall_thickness_m = -0.01;
    EXPECT_THROW(CubeSatelliteModel{p}, std::invalid_argument);
    p.wall_thickness_m = 0.0725;  // == default collar_radius_m
    EXPECT_THROW(CubeSatelliteModel{p}, std::invalid_argument);
    p.wall_thickness_m = 0.08;  // > collar_radius_m
    EXPECT_THROW(CubeSatelliteModel{p}, std::invalid_argument);
}

// ---------------------------------------------------------------------------------------
// Consistency between graspPoseAt() and the collar (cylinder along the face normal: base on the cube
// face, top at standoff_m beyond it)
// ---------------------------------------------------------------------------------------
TEST(CubeSatelliteModelConsistency, GraspPointsLieOnCollarTopRim) {
    for (const auto& par : {CubeSatelliteModel::Params{}, generalParams()}) {
        const CubeSatelliteModel m(par);
        for (double theta : {0.0, deg(60.0), deg(-100.0)}) {
            const Eigen::Vector3d axis = m.faceNormalWorld(theta);
            const Eigen::Vector3d base = m.cubeCenterWorld() + 0.5 * par.cube_side_m * axis;
            const Eigen::Vector3d top = base + par.standoff_m * axis;
            for (auto k : kAllPoints) {
                const Eigen::Vector3d p = m.graspPoseAt(k, theta).position_world;
                const double axial = (p - base).dot(axis);
                const Eigen::Vector3d radial = (p - base) - axial * axis;
                EXPECT_NEAR(axial, par.standoff_m, 1e-12)
                    << "axial offset error [m], theta=" << theta << " k=" << static_cast<int>(k);
                EXPECT_NEAR(radial.norm(), par.grasp_radius_m.value_or(par.collar_radius_m), 1e-12)
                    << "radial error [m], theta=" << theta << " k=" << static_cast<int>(k);
                // Same axial position as the top center.
                EXPECT_NEAR((p - top).dot(axis), 0.0, 1e-12);
            }
        }
    }
}

TEST(CubeSatelliteModelConsistency, ApproachAxisIsOppositeOfFaceNormal) {
    for (const auto& par : {CubeSatelliteModel::Params{}, generalParams()}) {
        const CubeSatelliteModel m(par);
        for (double theta : {0.0, deg(60.0), deg(-100.0)}) {
            for (auto k : kAllPoints) {
                expectVecNear(m.graspPoseAt(k, theta).approach_axis_world,
                              -m.faceNormalWorld(theta), 1e-12);
            }
        }
    }
}

TEST(CubeSatelliteModelConsistency, ProjectedGraspPointsFormSquareOnTopPlane) {
    for (const auto& par : {CubeSatelliteModel::Params{}, generalParams()}) {
        const CubeSatelliteModel m(par);
        for (double theta : {0.0, deg(60.0), deg(-100.0)}) {
            const Eigen::Vector3d axis = m.faceNormalWorld(theta);
            const Eigen::Vector3d top = m.cubeCenterWorld() + (0.5 * par.cube_side_m + par.standoff_m) * axis;
            std::vector<Eigen::Vector3d> d;  // in-plane offsets from the top center
            for (auto k : kAllPoints) {
                const Eigen::Vector3d q = m.graspPoseAt(k, theta).position_world - top;
                d.push_back(q - q.dot(axis) * axis);  // project onto the plane through top
                EXPECT_NEAR(d.back().norm(), par.grasp_radius_m.value_or(par.collar_radius_m), 1e-12);
            }
            for (size_t i = 0; i < d.size(); ++i) {
                const Eigen::Vector3d& a = d[i];
                const Eigen::Vector3d& b = d[(i + 1) % d.size()];
                // Signed in-plane angle between adjacent points is +90 deg about the axis.
                const double ang = std::atan2(axis.dot(a.cross(b)), a.dot(b));
                EXPECT_NEAR(std::abs(ang), M_PI / 2.0, 1e-12)
                    << "adjacent angle error [rad], theta=" << theta << " i=" << i;
                // Square: side = sqrt(2) * radius.
                EXPECT_NEAR((a - b).norm(),
                            std::sqrt(2.0) * par.grasp_radius_m.value_or(par.collar_radius_m), 1e-12);
            }
        }
    }
}

// ---------------------------------------------------------------------------------------
// Real satellite (CAD) dimensions and grasp_radius_m
// ---------------------------------------------------------------------------------------
// Rim hand-computed from the CAD numbers: cube side 0.150, standoff 0.080, grasp radius 0.068.
// Axis = +Z through the origin, n = +X, u = n x axis = -Y, v = +Z. The rim point k sits at
// (0.155, 0, 0) + 0.068 (cos(phi_k) u + sin(phi_k) v) before the rotation by theta about Z.
// Distance from the rotation axis: 0.155 for the two points on the axis' vertical (k = 90, 270),
// sqrt(0.155^2 + 0.068^2) = 0.1693 for the two lateral ones (k = 0, 180).
TEST(CubeSatelliteModelRealGeometry, RimPointsMatchHandComputedCadDimensions) {
    CubeSatelliteModel::Params par;  // defaults: side 0.150, standoff 0.080
    par.grasp_radius_m = 0.068;
    const CubeSatelliteModel m(par);
    const double edge = 0.075 + 0.080;
    const double rho = 0.068;
    const double lateral = std::sqrt(edge * edge + rho * rho);
    EXPECT_NEAR(lateral, 0.1693, 1e-4);
    for (double theta_deg : {0.0, 90.0}) {
        const double th = deg(theta_deg);
        const double c = std::cos(th), s = std::sin(th);
        for (int i = 0; i < 4; ++i) {
            const GraspPointId k = kAllPoints[static_cast<size_t>(i)];
            const double phi = i * M_PI / 2.0;
            // body-frame rim point: x = edge, y = -rho cos(phi) (u = -Y), z = rho sin(phi) (v = +Z)
            const Eigen::Vector3d p_body(edge, -rho * std::cos(phi), rho * std::sin(phi));
            const Eigen::Vector3d expected(c * p_body.x() - s * p_body.y(),
                                           s * p_body.x() + c * p_body.y(), p_body.z());
            const Eigen::Vector3d got = m.graspPoseAt(k, th).position_world;
            expectVecNear(got, expected, 1e-6);
            const double dist_from_axis = std::hypot(got.x(), got.y());
            const bool vertical = (i == 1 || i == 3);
            EXPECT_NEAR(dist_from_axis, vertical ? edge : lateral, 1e-6)
                << "k=" << i << " theta=" << theta_deg;
        }
    }
}

TEST(CubeSatelliteModelRealGeometry, UnsetGraspRadiusEqualsCollarRadius) {
    CubeSatelliteModel::Params unset = generalParams();
    CubeSatelliteModel::Params set = generalParams();
    set.grasp_radius_m = set.collar_radius_m;
    ASSERT_FALSE(unset.grasp_radius_m.has_value());
    const CubeSatelliteModel a(unset), b(set);
    for (double theta : {0.0, deg(33.0), deg(-120.0)}) {
        for (auto k : kAllPoints) {
            const GraspTargetPose pa = a.graspPoseAt(k, theta);
            const GraspTargetPose pb = b.graspPoseAt(k, theta);
            expectVecNear(pa.position_world, pb.position_world, 1e-15);
            EXPECT_TRUE(pa.orientation_nominal_world.isApprox(pb.orientation_nominal_world, 1e-15));
            expectVecNear(pa.approach_axis_world, pb.approach_axis_world, 1e-15);
        }
    }
}

