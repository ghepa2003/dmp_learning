// Pure tests for the grasp-cost helpers (no ROS). See core/grasp_cost.hpp.

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>

#include "haptic_dmp_learning/core/grasp_cost.hpp"

using namespace haptic_dmp_learning::core;

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
