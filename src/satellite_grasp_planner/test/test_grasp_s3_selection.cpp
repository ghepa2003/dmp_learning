// Tests for selectGraspS3 (synthetic ProDMP template). Need Pinocchio and $HOME/thesis_ws/fer_flat_effort.urdf
// (tests skip if it is missing, or if the fixture geometry yields no constraint-passing candidate).

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include "test_fixtures.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "satellite_grasp_planner/core/grasp_pose_eval.hpp"
#include "satellite_grasp_planner/core/grasp_s3_selection.hpp"

using namespace satellite_grasp_planner::core;
using haptic_dmp_learning::core::CubeSatelliteModel;
using haptic_dmp_learning::core::GraspPointId;
using haptic_dmp_learning::core::ProDMP;
using RobotModel = franka_cartesian_control::core::RobotModel;
namespace tf = satellite_grasp_planner::test_fixtures;

#define REQUIRE_URDF_OR_SKIP()                                                          \
    do {                                                                                \
        if (!std::ifstream(tf::urdfPath()).good()) {                                    \
            GTEST_SKIP() << "URDF fixture not found: " << tf::urdfPath();               \
        }                                                                               \
    } while (0)

namespace {

constexpr double kOmega = -2.0 * M_PI / 180.0;

struct Setup {
    std::shared_ptr<RobotModel> robot;
    RobotModel::JointVector q_start;
    DemoReference demo;
    ProDMP tmpl;
    Eigen::Vector3d delta_g = Eigen::Vector3d(0.1, 0.0, 0.0);
    CubeSatelliteModel model;
    S3Context ctx;
};

CubeSatelliteModel makeModel(const Eigen::Vector3d& center) {
    CubeSatelliteModel::Params p;
    p.center_world = center;
    p.axis_world = Eigen::Vector3d::UnitZ();
    p.face_normal_body = Eigen::Vector3d::UnitX();
    p.cube_side_m = 0.150;
    p.standoff_m = 0.08;
    p.collar_radius_m = 0.0725;
    p.wall_thickness_m = 0.009;
    p.grasp_radius_m = 0.068;
    return CubeSatelliteModel(p);
}

/// Satellite placed so that the (k = 0, theta = 0) grasp point is ~7 cm from the start TCP (same trick as the
/// grasp_pose_eval end-to-end test).
Setup makeSetup() {
    Setup s{tf::loadPandaRobotModel(), RobotModel::JointVector::Zero(), DemoReference(), tf::makeSyntheticProDmpTemplate(),
            Eigen::Vector3d(0.1, 0.0, 0.0), makeModel(Eigen::Vector3d::Zero()), S3Context()};
    s.q_start << 0.0, -0.7854, 0.0, -2.3562, 0.0, 3.1416, 0.7854;
    s.demo = demoReferenceFromEndJoints(*s.robot, s.q_start);
    const Eigen::Vector3d center = s.demo.position + Eigen::Vector3d(0.05, 0.0, 0.05) - Eigen::Vector3d(0.155, -0.068, 0.0);
    s.model = makeModel(center);
    s.ctx.tau_contact_s = 0.0;
    s.ctx.tau_launch_s = s.tmpl.tau();
    s.ctx.snapshot.center = center;
    s.ctx.snapshot.axis = Eigen::Vector3d::UnitZ();
    s.ctx.snapshot.omega_rad_s = kOmega;
    s.ctx.snapshot.theta_rad = 0.3;
    s.ctx.snapshot.t_s = 0.0;
    s.ctx.t_now_s = 10.0;
    s.ctx.min_delay_s = 0.5;
    return s;
}

/// Constraints wide open (only IK feasibility remains), no post-hoc stage, coarse grid.
S3Params looseParams() {
    S3Params p;
    p.w_hat_min = 0.0;
    p.margin_min_rad = 0.0;
    p.r_trust_m = 100.0;
    p.theta_step_deg = 5.0;
    p.theta_refine_step_deg = 1.0;
    p.posthoc_path_check = false;
    p.posthoc_velocity_check = false;
    return p;
}

}  // namespace

TEST(GraspS3Test, ImpossibleConstraintGivesNoCandidate) {
    REQUIRE_URDF_OR_SKIP();
    Setup s = makeSetup();
    s.ctx.prodmp_template = &s.tmpl;  // set here: Setup may be moved out of makeSetup()
    ASSERT_NE(s.robot, nullptr);
    S3Params p = looseParams();
    p.w_hat_min = 5.0;
    S3Result r;
    ASSERT_NO_THROW(r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p));
    EXPECT_EQ(r.status, S3Result::kNoCandidate);
    EXPECT_TRUE(r.ranked.empty());
    EXPECT_GT(r.evaluations, 0);
    EXPECT_FALSE(r.note.empty());
}

TEST(GraspS3Test, WinnerPassesConstraintsAndIsCheapest) {
    REQUIRE_URDF_OR_SKIP();
    Setup s = makeSetup();
    s.ctx.prodmp_template = &s.tmpl;  // set here: Setup may be moved out of makeSetup()
    ASSERT_NE(s.robot, nullptr);
    const S3Params p = looseParams();
    const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    if (r.status != S3Result::kSelected) GTEST_SKIP() << "no feasible candidate in this fixture: " << r.note;

    EXPECT_TRUE(r.chosen.passes_constraints);
    EXPECT_TRUE(r.chosen.eval.feasible_basic);
    EXPECT_GE(r.chosen.eval.w_hat, p.w_hat_min);
    EXPECT_GE(r.chosen.eval.margins.min_margin_rad, p.margin_min_rad);
    EXPECT_LE(r.chosen.d_pos_m, p.r_trust_m);
    ASSERT_FALSE(r.ranked.empty());
    EXPECT_LE(r.ranked.size(), 20u);
    EXPECT_DOUBLE_EQ(r.chosen.cost, r.ranked.front().cost);  // no post-hoc: the best ranked is the winner
    for (std::size_t i = 0; i < r.ranked.size(); ++i) {
        EXPECT_TRUE(r.ranked[i].passes_constraints);
        EXPECT_GE(r.ranked[i].cost, r.chosen.cost);
        if (i > 0) EXPECT_GE(r.ranked[i].cost, r.ranked[i - 1].cost);
    }
}

TEST(GraspS3Test, DistanceIgnoresOrientationWithZeroWeight) {
    // Two candidates at the same position and different orientation (wrist flip = rotation of pi about z_tcp).
    EXPECT_DOUBLE_EQ(s3Distance(0.05, 0.0, 0.10, 0.0, 1.0), s3Distance(0.05, M_PI, 0.10, 0.0, 1.0));
    EXPECT_NEAR(s3Distance(0.05, 0.0, 0.10, 0.0, 1.0), 0.5, 1e-12);
    EXPECT_GT(s3Distance(0.05, M_PI, 0.10, 1.0, 1.0), s3Distance(0.05, 0.0, 0.10, 1.0, 1.0));
    EXPECT_THROW(s3Distance(0.05, 0.0, 0.0, 0.0, 1.0), std::invalid_argument);
    EXPECT_THROW(s3Distance(0.05, 0.0, 0.1, -1.0, 1.0), std::invalid_argument);

    REQUIRE_URDF_OR_SKIP();
    Setup s = makeSetup();
    s.ctx.prodmp_template = &s.tmpl;  // set here: Setup may be moved out of makeSetup()
    ASSERT_NE(s.robot, nullptr);
    const S3Params p = looseParams();  // orientation_weight = 0
    const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    for (const auto& c : r.ranked) {
        EXPECT_NEAR(c.d, c.d_pos_m / p.length_scale_m, 1e-12);  // wrist_flip or not, d is position-only
    }
}

TEST(GraspS3Test, WaitTimeWithinOnePeriod) {
    REQUIRE_URDF_OR_SKIP();
    Setup s = makeSetup();
    s.ctx.prodmp_template = &s.tmpl;  // set here: Setup may be moved out of makeSetup()
    ASSERT_NE(s.robot, nullptr);
    const S3Params p = looseParams();
    const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    if (r.status != S3Result::kSelected) GTEST_SKIP() << "no feasible candidate in this fixture: " << r.note;
    const double period = 2.0 * M_PI / std::abs(kOmega);  // 180 s
    EXPECT_GE(r.chosen.wait_s, s.ctx.min_delay_s);
    EXPECT_LE(r.chosen.wait_s, s.ctx.min_delay_s + period);
    EXPECT_NEAR(r.chosen.cost, r.chosen.d + p.epsilon_wait_per_s * r.chosen.wait_s, 1e-12);
}

TEST(GraspS3Test, PosthocVelocityRejectsFirstCandidate) {
    REQUIRE_URDF_OR_SKIP();
    Setup s = makeSetup();
    s.ctx.prodmp_template = &s.tmpl;  // set here: Setup may be moved out of makeSetup()
    ASSERT_NE(s.robot, nullptr);
    S3Params p = looseParams();
    p.posthoc_velocity_check = true;
    p.delta_v_max_m_s = 1e-6;
    p.max_posthoc_candidates = 5;
    const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    if (r.status == S3Result::kNoCandidate) GTEST_SKIP() << "no feasible candidate in this fixture: " << r.note;
    ASSERT_FALSE(r.ranked.empty());
    EXPECT_NE(r.ranked.front().reject_reason.find("velocita'"), std::string::npos) << r.ranked.front().reject_reason;
    EXPECT_GE(r.posthoc_checks, 1);
    if (r.status == S3Result::kSelected) {
        EXPECT_GE(r.posthoc_checks, 2);
        EXPECT_LE(r.posthoc_dv_m_s, p.delta_v_max_m_s);
        const bool same = r.chosen.eval.k == r.ranked.front().eval.k &&
                          r.chosen.eval.theta_rad == r.ranked.front().eval.theta_rad;
        EXPECT_FALSE(same);
    } else {
        EXPECT_EQ(r.status, S3Result::kPosthocFailed);
        EXPECT_FALSE(r.note.empty());
    }
}

TEST(GraspS3Test, Deterministic) {
    REQUIRE_URDF_OR_SKIP();
    Setup s = makeSetup();
    s.ctx.prodmp_template = &s.tmpl;  // set here: Setup may be moved out of makeSetup()
    ASSERT_NE(s.robot, nullptr);
    const S3Params p = looseParams();
    const S3Result a = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    const S3Result b = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    EXPECT_EQ(a.status, b.status);
    EXPECT_EQ(a.evaluations, b.evaluations);
    ASSERT_EQ(a.ranked.size(), b.ranked.size());
    for (std::size_t i = 0; i < a.ranked.size(); ++i) {
        EXPECT_EQ(a.ranked[i].eval.k, b.ranked[i].eval.k);
        EXPECT_DOUBLE_EQ(a.ranked[i].eval.theta_rad, b.ranked[i].eval.theta_rad);
        EXPECT_DOUBLE_EQ(a.ranked[i].cost, b.ranked[i].cost);
    }
    if (a.status == S3Result::kSelected) {
        EXPECT_EQ(a.chosen.eval.k, b.chosen.eval.k);
        EXPECT_DOUBLE_EQ(a.chosen.eval.theta_rad, b.chosen.eval.theta_rad);
    }
}

TEST(GraspS3Test, BestPerKHasFourDistinctPoints) {
    REQUIRE_URDF_OR_SKIP();
    Setup s = makeSetup();
    s.ctx.prodmp_template = &s.tmpl;  // set here: Setup may be moved out of makeSetup()
    ASSERT_NE(s.robot, nullptr);
    const S3Params p = looseParams();
    const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    ASSERT_EQ(r.best_per_k.size(), 4u);
    const GraspPointId ks[4] = {GraspPointId::kP0, GraspPointId::kP90, GraspPointId::kP180, GraspPointId::kP270};
    for (std::size_t i = 0; i < 4; ++i) EXPECT_EQ(r.best_per_k[i].eval.k, ks[i]);
}

TEST(GraspS3Test, InvalidInputsThrow) {
    REQUIRE_URDF_OR_SKIP();
    Setup s = makeSetup();
    s.ctx.prodmp_template = &s.tmpl;  // set here: Setup may be moved out of makeSetup()
    ASSERT_NE(s.robot, nullptr);
    S3Params p = looseParams();
    S3Context bad = s.ctx;
    bad.prodmp_template = nullptr;
    EXPECT_THROW(selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, bad, p), std::invalid_argument);
    p.theta_step_deg = 0.0;
    EXPECT_THROW(selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p), std::invalid_argument);
}
