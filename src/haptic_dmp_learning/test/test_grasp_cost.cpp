// Pure tests for the grasp-cost helpers (no ROS). See core/grasp_cost.hpp.

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>

#include "haptic_dmp_learning/core/grasp_cost.hpp"

using namespace haptic_dmp_learning::core;

namespace {

const double kOmega = 2.0 * M_PI / 180.0;  // 0.0349066 rad/s
const std::vector<GraspPointId> kAllPoints = {GraspPointId::kP0, GraspPointId::kP90,
                                              GraspPointId::kP180, GraspPointId::kP270};

void expectVecNear(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double tol) {
    for (int i = 0; i < 3; ++i) EXPECT_NEAR(a(i), b(i), tol) << "component " << i;
}

}  // namespace

// Default params: center 0, axis +Z, n = +X, side 0.20, standoff 0.08, collar 0.05, so at theta = 0
// kP0=(0.18,-0.05,0), kP90=(0.18,0,0.05), kP180=(0.18,0.05,0), kP270=(0.18,0,-0.05).
TEST(GraspCostSurfaceVelocity, MagnitudesDirectionsAndSign) {
    const CubeSatelliteModel m{CubeSatelliteModel::Params{}};
    const double r_axial = std::sqrt(0.18 * 0.18 + 0.05 * 0.05);

    EXPECT_NEAR(surfaceVelocity(m, GraspPointId::kP90, 0.0, kOmega).norm(), kOmega * 0.18, 1e-12);
    EXPECT_NEAR(surfaceVelocity(m, GraspPointId::kP270, 0.0, kOmega).norm(), kOmega * 0.18, 1e-12);
    EXPECT_NEAR(surfaceVelocity(m, GraspPointId::kP0, 0.0, kOmega).norm(), kOmega * r_axial, 1e-12);
    EXPECT_NEAR(surfaceVelocity(m, GraspPointId::kP180, 0.0, kOmega).norm(), kOmega * r_axial, 1e-12);

    // v is perpendicular to the axis and to the radius vector (point - center).
    for (GraspPointId k : kAllPoints) {
        const Eigen::Vector3d v = surfaceVelocity(m, k, 0.0, kOmega);
        const Eigen::Vector3d r = m.graspPoseAt(k, 0.0).position_world - m.cubeCenterWorld();
        EXPECT_NEAR(v.dot(m.axisWorld()), 0.0, 1e-12);
        EXPECT_NEAR(v.dot(r), 0.0, 1e-12);
    }

    // Clockwise (omega < 0): kP90/kP270 at x = +0.18 move along -Y; counter-clockwise: +Y.
    for (GraspPointId k : {GraspPointId::kP90, GraspPointId::kP270}) {
        expectVecNear(surfaceVelocity(m, k, 0.0, -kOmega), Eigen::Vector3d(0.0, -kOmega * 0.18, 0.0),
                      1e-12);
        expectVecNear(surfaceVelocity(m, k, 0.0, kOmega), Eigen::Vector3d(0.0, kOmega * 0.18, 0.0),
                      1e-12);
        expectVecNear(surfaceVelocity(m, k, 0.0, kOmega), -surfaceVelocity(m, k, 0.0, -kOmega), 1e-15);
    }
}

TEST(GraspCostSurfaceVelocity, MagnitudeIndependentOfTheta) {
    const CubeSatelliteModel m{CubeSatelliteModel::Params{}};
    for (GraspPointId k : kAllPoints) {
        const double ref = surfaceVelocity(m, k, 0.0, kOmega).norm();
        for (double theta : {0.4, -1.3, 2.2, 3.0, -2.9}) {
            EXPECT_NEAR(surfaceVelocity(m, k, theta, kOmega).norm(), ref, 1e-12) << theta;
        }
    }
}

TEST(GraspCostSurfaceVelocity, MeanSquaredMatchesHandComputation) {
    const CubeSatelliteModel m{CubeSatelliteModel::Params{}};
    // Two points at |v|^2 = w^2 * 0.18^2, two at w^2 * (0.18^2 + 0.05^2).
    const double expected =
        (2.0 * kOmega * kOmega * 0.18 * 0.18 +
         2.0 * kOmega * kOmega * (0.18 * 0.18 + 0.05 * 0.05)) / 4.0;
    EXPECT_NEAR(meanSurfaceSpeedSquared(m, kOmega), expected, 1e-15);
    EXPECT_NEAR(meanSurfaceSpeedSquared(m, -kOmega), expected, 1e-15);
}

TEST(GraspCostTerms, VelocityTerm) {
    const Eigen::Vector3d a(1.0, 2.0, 3.0), b(0.0, 2.0, 5.0);
    EXPECT_DOUBLE_EQ(velocityTerm(a, a), 0.0);
    EXPECT_DOUBLE_EQ(velocityTerm(a, b), velocityTerm(b, a));
    EXPECT_NEAR(velocityTerm(a, b), 5.0, 1e-15);  // diff (1,0,-2)
}

TEST(GraspCostTerms, GoalTerm) {
    const Eigen::Vector3d d(0.1, 0.0, 0.0), dd(0.0, 0.2, -0.1);
    EXPECT_DOUBLE_EQ(goalTerm(d, d), 0.0);
    EXPECT_NEAR(goalTerm(d, dd), 0.06, 1e-15);  // diff (0.1,-0.2,0.1)
}

TEST(GraspCostCombine, UnitTermsAdditivityAndMonotonicity) {
    const GraspCostWeights w;
    GraspCostReferences ref;
    ref.e_v_ref = 4.0;
    ref.e_g_ref = 0.5;
    ref.w_trans_demo = 2.0;
    const double tol = kDefaultRollToleranceRad;

    GraspCostTerms t;
    t.e_v = ref.e_v_ref;
    t.e_g = ref.e_g_ref;
    t.w_trans = ref.w_trans_demo;
    t.psi_rad = tol;
    const GraspCostBreakdown b = combineCost(t, ref, w, tol);
    EXPECT_NEAR(b.e_v_hat, 1.0, 1e-12);
    EXPECT_NEAR(b.e_g_hat, 1.0, 1e-12);
    EXPECT_NEAR(b.w_hat, 1.0, 1e-12);
    EXPECT_NEAR(b.p_psi, 1.0, 1e-12);
    EXPECT_NEAR(b.total, w.w_v + w.w_g - w.w_m + w.w_psi, 1e-12);

    // Higher manipulability lowers the total.
    GraspCostTerms t_more = t;
    t_more.w_trans = 2.0 * ref.w_trans_demo;
    EXPECT_LT(combineCost(t_more, ref, w, tol).total, b.total);

    // psi = 0 -> no roll penalty.
    GraspCostTerms t0 = t;
    t0.psi_rad = 0.0;
    EXPECT_NEAR(combineCost(t0, ref, w, tol).p_psi, 0.0, 1e-15);

    // Additive in the weights: doubling w_g shifts total by w_g * e_g_hat.
    GraspCostWeights w2 = w;
    w2.w_g = 2.0 * w.w_g;
    EXPECT_NEAR(combineCost(t, ref, w2, tol).total - b.total, w.w_g * b.e_g_hat, 1e-12);
}

TEST(GraspCostCombine, ValidateThrows) {
    const GraspCostTerms t;
    const double tol = kDefaultRollToleranceRad;
    for (int i = 0; i < 3; ++i) {
        GraspCostReferences ref;
        (i == 0 ? ref.e_v_ref : i == 1 ? ref.e_g_ref : ref.w_trans_demo) = 0.0;
        EXPECT_THROW(ref.validate(), std::invalid_argument);
        EXPECT_THROW(combineCost(t, ref, GraspCostWeights{}, tol), std::invalid_argument);
        (i == 0 ? ref.e_v_ref : i == 1 ? ref.e_g_ref : ref.w_trans_demo) = -1.0;
        EXPECT_THROW(ref.validate(), std::invalid_argument);
    }
    for (int i = 0; i < 4; ++i) {
        GraspCostWeights w;
        (i == 0 ? w.w_v : i == 1 ? w.w_g : i == 2 ? w.w_m : w.w_psi) = -0.1;
        EXPECT_THROW(w.validate(), std::invalid_argument);
        EXPECT_THROW(combineCost(t, GraspCostReferences{}, w, tol), std::invalid_argument);
    }
    EXPECT_NO_THROW(GraspCostWeights{}.validate());
    EXPECT_NO_THROW(GraspCostReferences{}.validate());
}
