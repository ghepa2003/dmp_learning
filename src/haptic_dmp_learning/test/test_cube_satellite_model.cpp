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
    // n = +X, axis = +Z -> u = n x axis = -Y, v = +Z. Face+standoff offset = 0.10 + 0.08 = 0.18.
    const CubeSatelliteModel m{CubeSatelliteModel::Params{}};
    expectVecNear(m.graspPoseAt(GraspPointId::kP0, 0.0).position_world, {0.18, -0.05, 0.0}, 1e-12);
    expectVecNear(m.graspPoseAt(GraspPointId::kP90, 0.0).position_world, {0.18, 0.0, 0.05}, 1e-12);
    expectVecNear(m.graspPoseAt(GraspPointId::kP180, 0.0).position_world, {0.18, 0.05, 0.0}, 1e-12);
    expectVecNear(
        m.graspPoseAt(GraspPointId::kP270, 0.0).position_world, {0.18, 0.0, -0.05}, 1e-12);
}

// ---------------------------------------------------------------------------------------
// Collar invariants
// ---------------------------------------------------------------------------------------
TEST(CubeSatelliteModelInvariants, PointsEquidistantFromCollarCenterAndAt90Degrees) {
    const CubeSatelliteModel::Params par = generalParams();
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
            EXPECT_NEAR(d.back().norm(), par.collar_radius_m, 1e-12);
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
        m.graspPoseAt(GraspPointId::kP90, M_PI).position_world, {-0.18, 0.0, 0.05}, 1e-12);
    expectVecNear(m.faceNormalWorld(M_PI), {-1.0, 0.0, 0.0}, 1e-12);
}

// ---------------------------------------------------------------------------------------
// Hollow-cylinder geometry
// ---------------------------------------------------------------------------------------
TEST(CubeSatelliteModelCylinder, RadiiMatchParamsAndThickness) {
    const CubeSatelliteModel::Params par;
    const CubeSatelliteModel m(par);
    EXPECT_NEAR(m.cylinderOuterRadius(), 0.05, 1e-15);
    EXPECT_NEAR(m.cylinderInnerRadius(), 0.035, 1e-15);
    EXPECT_NEAR(m.cylinderOuterRadius() - m.cylinderInnerRadius(), par.wall_thickness_m, 1e-15);
}

TEST(CubeSatelliteModelCylinder, EndpointsAtThetaZeroDefaultParams) {
    const CubeSatelliteModel m{CubeSatelliteModel::Params{}};
    // Base on the face (side/2 = 0.10), top at side/2 + standoff = 0.18 along +x.
    const Eigen::Vector3d base = m.cylinderBaseCenterWorld(0.0);
    const Eigen::Vector3d top = m.cylinderTopCenterWorld(0.0);
    expectVecNear(base, {0.10, 0.0, 0.0}, 1e-12);
    expectVecNear(top, {0.18, 0.0, 0.0}, 1e-12);
    EXPECT_NEAR((top - base).norm(), 0.08, 1e-12);
}

TEST(CubeSatelliteModelCylinder, AxisEqualsFaceNormalComponentwise) {
    for (const auto& par : {CubeSatelliteModel::Params{}, generalParams()}) {
        const CubeSatelliteModel m(par);
        for (double theta : {deg(30.0), deg(90.0), deg(-200.0)}) {
            expectVecNear(m.cylinderAxisWorld(theta), m.faceNormalWorld(theta), 1e-15);
        }
    }
}

TEST(CubeSatelliteModelCylinder, ConstructorRejectsInvalidWallThickness) {
    CubeSatelliteModel::Params p;
    p.wall_thickness_m = 0.0;
    EXPECT_THROW(CubeSatelliteModel{p}, std::invalid_argument);
    p.wall_thickness_m = -0.01;
    EXPECT_THROW(CubeSatelliteModel{p}, std::invalid_argument);
    p.wall_thickness_m = 0.05;  // == default collar_radius_m
    EXPECT_THROW(CubeSatelliteModel{p}, std::invalid_argument);
    p.wall_thickness_m = 0.06;  // > collar_radius_m
    EXPECT_THROW(CubeSatelliteModel{p}, std::invalid_argument);
}

TEST(CubeSatelliteModelCylinder, EndpointsRotateRigidlyAboutAxis) {
    const CubeSatelliteModel::Params par = generalParams();
    const CubeSatelliteModel m(par);
    const Eigen::Vector3d axis = par.axis_world.normalized();
    for (double theta : {deg(90.0), deg(45.0), deg(-120.0)}) {
        const Eigen::AngleAxisd rot(theta, axis);
        const Eigen::Vector3d b0 = m.cylinderBaseCenterWorld(0.0) - par.center_world;
        const Eigen::Vector3d t0 = m.cylinderTopCenterWorld(0.0) - par.center_world;
        const Eigen::Vector3d bt = m.cylinderBaseCenterWorld(theta) - par.center_world;
        const Eigen::Vector3d tt = m.cylinderTopCenterWorld(theta) - par.center_world;
        expectVecNear(bt, rot * b0, 1e-12);
        expectVecNear(tt, rot * t0, 1e-12);
        EXPECT_NEAR(bt.norm(), b0.norm(), 1e-12);
        EXPECT_NEAR(bt.dot(axis), b0.dot(axis), 1e-12);  // axial component unchanged
        EXPECT_NEAR((tt - bt).norm(), par.standoff_m, 1e-12);
    }
}

// ---------------------------------------------------------------------------------------
// Consistency between graspPoseAt() and the cylinder geometry
// ---------------------------------------------------------------------------------------
TEST(CubeSatelliteModelConsistency, GraspPointsLieOnCylinderTopRim) {
    for (const auto& par : {CubeSatelliteModel::Params{}, generalParams()}) {
        const CubeSatelliteModel m(par);
        for (double theta : {0.0, deg(60.0), deg(-100.0)}) {
            const Eigen::Vector3d axis = m.cylinderAxisWorld(theta);
            const Eigen::Vector3d base = m.cylinderBaseCenterWorld(theta);
            const Eigen::Vector3d top = m.cylinderTopCenterWorld(theta);
            for (auto k : kAllPoints) {
                const Eigen::Vector3d p = m.graspPoseAt(k, theta).position_world;
                const double axial = (p - base).dot(axis);
                const Eigen::Vector3d radial = (p - base) - axial * axis;
                EXPECT_NEAR(axial, par.standoff_m, 1e-12)
                    << "axial offset error [m], theta=" << theta << " k=" << static_cast<int>(k);
                EXPECT_NEAR(radial.norm(), m.cylinderOuterRadius(), 1e-12)
                    << "radial error [m], theta=" << theta << " k=" << static_cast<int>(k);
                // Same axial position as the top center.
                EXPECT_NEAR((p - top).dot(axis), 0.0, 1e-12);
            }
        }
    }
}

TEST(CubeSatelliteModelConsistency, ApproachAxisIsOppositeOfCylinderAxis) {
    for (const auto& par : {CubeSatelliteModel::Params{}, generalParams()}) {
        const CubeSatelliteModel m(par);
        for (double theta : {0.0, deg(60.0), deg(-100.0)}) {
            for (auto k : kAllPoints) {
                expectVecNear(m.graspPoseAt(k, theta).approach_axis_world,
                              -m.cylinderAxisWorld(theta), 1e-12);
            }
        }
    }
}

TEST(CubeSatelliteModelConsistency, ProjectedGraspPointsFormSquareOnTopPlane) {
    for (const auto& par : {CubeSatelliteModel::Params{}, generalParams()}) {
        const CubeSatelliteModel m(par);
        for (double theta : {0.0, deg(60.0), deg(-100.0)}) {
            const Eigen::Vector3d axis = m.cylinderAxisWorld(theta);
            const Eigen::Vector3d top = m.cylinderTopCenterWorld(theta);
            std::vector<Eigen::Vector3d> d;  // in-plane offsets from the top center
            for (auto k : kAllPoints) {
                const Eigen::Vector3d q = m.graspPoseAt(k, theta).position_world - top;
                d.push_back(q - q.dot(axis) * axis);  // project onto the plane through top
                EXPECT_NEAR(d.back().norm(), m.cylinderOuterRadius(), 1e-12);
            }
            for (size_t i = 0; i < d.size(); ++i) {
                const Eigen::Vector3d& a = d[i];
                const Eigen::Vector3d& b = d[(i + 1) % d.size()];
                // Signed in-plane angle between adjacent points is +90 deg about the axis.
                const double ang = std::atan2(axis.dot(a.cross(b)), a.dot(b));
                EXPECT_NEAR(std::abs(ang), M_PI / 2.0, 1e-12)
                    << "adjacent angle error [rad], theta=" << theta << " i=" << i;
                // Square: side = sqrt(2) * radius.
                EXPECT_NEAR((a - b).norm(), std::sqrt(2.0) * m.cylinderOuterRadius(), 1e-12);
            }
        }
    }
}
