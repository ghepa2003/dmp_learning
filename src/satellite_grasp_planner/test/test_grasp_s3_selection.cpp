// Tests for selectGraspS3 (synthetic ProDMP template). Need Pinocchio and $HOME/thesis_ws/fer_flat_effort.urdf
// (tests skip if it is missing, or if the fixture geometry yields no constraint-passing candidate).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include "test_fixtures.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/tcp_roll.hpp"
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

struct S3TestSetup {
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
S3TestSetup makeS3TestSetup() {
    S3TestSetup s{tf::loadPandaRobotModel(), RobotModel::JointVector::Zero(), DemoReference(), tf::makeSyntheticProDmpTemplate(),
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
    S3TestSetup s = makeS3TestSetup();
    s.ctx.prodmp_template = &s.tmpl;  // set here: S3TestSetup may be moved out of makeS3TestSetup()
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
    S3TestSetup s = makeS3TestSetup();
    s.ctx.prodmp_template = &s.tmpl;  // set here: S3TestSetup may be moved out of makeS3TestSetup()
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
        if (i > 0) {
            EXPECT_GE(r.ranked[i].cost, r.ranked[i - 1].cost);
        }
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
    S3TestSetup s = makeS3TestSetup();
    s.ctx.prodmp_template = &s.tmpl;  // set here: S3TestSetup may be moved out of makeS3TestSetup()
    ASSERT_NE(s.robot, nullptr);
    const S3Params p = looseParams();  // orientation_weight = 0
    const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    for (const auto& c : r.ranked) {
        EXPECT_NEAR(c.d, c.d_pos_m / p.length_scale_m, 1e-12);  // wrist_flip or not, d is position-only
    }
}

TEST(GraspS3Test, WaitTimeWithinOnePeriod) {
    REQUIRE_URDF_OR_SKIP();
    S3TestSetup s = makeS3TestSetup();
    s.ctx.prodmp_template = &s.tmpl;  // set here: S3TestSetup may be moved out of makeS3TestSetup()
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
    S3TestSetup s = makeS3TestSetup();
    s.ctx.prodmp_template = &s.tmpl;  // set here: S3TestSetup may be moved out of makeS3TestSetup()
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

TEST(GraspS3Test, PosthocClusterSkipsNearbyThetaOfSameK) {
    REQUIRE_URDF_OR_SKIP();
    S3TestSetup s = makeS3TestSetup();
    s.ctx.prodmp_template = &s.tmpl;  // set here: S3TestSetup may be moved out of makeS3TestSetup()
    ASSERT_NE(s.robot, nullptr);
    S3Params p = looseParams();
    p.posthoc_velocity_check = true;  // an impossible dv makes every checked candidate fail, so all get examined
    p.delta_v_max_m_s = 1e-6;
    p.max_posthoc_candidates = 100000;
    p.theta_refine_step_deg = 1.0;  // refined candidates sit 1-2 deg from the coarse best of the same k

    p.posthoc_cluster_deg = 0.0;
    const S3Result off = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    if (off.status == S3Result::kNoCandidate) GTEST_SKIP() << "no feasible candidate in this fixture: " << off.note;
    p.posthoc_cluster_deg = 10.0;
    const S3Result on = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);

    ASSERT_EQ(on.posthoc_log.size(), static_cast<std::size_t>(on.posthoc_checks));
    ASSERT_EQ(off.posthoc_log.size(), static_cast<std::size_t>(off.posthoc_checks));
    EXPECT_LT(on.posthoc_checks, off.posthoc_checks);  // near-duplicates (refined thetas) were skipped
    for (std::size_t i = 0; i < on.posthoc_log.size(); ++i) {
        for (std::size_t j = i + 1; j < on.posthoc_log.size(); ++j) {
            if (on.posthoc_log[i].k != on.posthoc_log[j].k) continue;
            double d = std::fmod(std::abs(on.posthoc_log[i].theta_rad - on.posthoc_log[j].theta_rad), 2.0 * M_PI);
            if (d > M_PI) d = 2.0 * M_PI - d;
            EXPECT_GE(d * 180.0 / M_PI, p.posthoc_cluster_deg - 1e-9) << "entries " << i << ", " << j;
        }
    }
}

TEST(GraspS3Test, PosthocChecksLimitedToMaxAndLogged) {
    REQUIRE_URDF_OR_SKIP();
    S3TestSetup s = makeS3TestSetup();
    s.ctx.prodmp_template = &s.tmpl;  // set here: S3TestSetup may be moved out of makeS3TestSetup()
    ASSERT_NE(s.robot, nullptr);
    S3Params p = looseParams();
    p.posthoc_velocity_check = true;
    p.delta_v_max_m_s = 1e-6;
    p.posthoc_cluster_deg = 0.0;
    p.max_posthoc_candidates = 3;
    const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    if (r.status == S3Result::kNoCandidate) GTEST_SKIP() << "no feasible candidate in this fixture: " << r.note;
    EXPECT_EQ(r.status, S3Result::kPosthocFailed);
    const std::size_t expected = std::min<std::size_t>(3, r.ranked.size());  // ranked holds up to 20
    EXPECT_EQ(static_cast<std::size_t>(r.posthoc_checks), expected);
    ASSERT_EQ(r.posthoc_log.size(), expected);
    for (const auto& e : r.posthoc_log) {
        EXPECT_FALSE(e.accepted);
        EXPECT_FALSE(e.path_checked);
        EXPECT_GT(e.dv_m_s, p.delta_v_max_m_s);
        EXPECT_NEAR(e.dv_m_s, (e.v_target - e.v_ee).norm(), 1e-12);
        EXPECT_FALSE(e.reject_reason.empty());
    }
}

TEST(GraspS3Test, PosthocVelocityCheckDisabledSelectsCheapest) {
    REQUIRE_URDF_OR_SKIP();
    S3TestSetup s = makeS3TestSetup();
    s.ctx.prodmp_template = &s.tmpl;  // set here: S3TestSetup may be moved out of makeS3TestSetup()
    ASSERT_NE(s.robot, nullptr);
    S3Params p = looseParams();
    p.posthoc_velocity_check = false;
    p.posthoc_path_check = false;
    p.delta_v_max_m_s = 1e-6;  // would reject everything if the velocity check were on
    const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    if (r.status == S3Result::kNoCandidate) GTEST_SKIP() << "no feasible candidate in this fixture: " << r.note;
    ASSERT_EQ(r.status, S3Result::kSelected);
    ASSERT_FALSE(r.ranked.empty());
    EXPECT_EQ(r.chosen.eval.k, r.ranked.front().eval.k);
    EXPECT_DOUBLE_EQ(r.chosen.eval.theta_rad, r.ranked.front().eval.theta_rad);
    EXPECT_EQ(r.posthoc_checks, 1);
    ASSERT_EQ(r.posthoc_log.size(), 1u);
    EXPECT_TRUE(r.posthoc_log.front().accepted);
}

TEST(GraspS3Test, Deterministic) {
    REQUIRE_URDF_OR_SKIP();
    S3TestSetup s = makeS3TestSetup();
    s.ctx.prodmp_template = &s.tmpl;  // set here: S3TestSetup may be moved out of makeS3TestSetup()
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
    S3TestSetup s = makeS3TestSetup();
    s.ctx.prodmp_template = &s.tmpl;  // set here: S3TestSetup may be moved out of makeS3TestSetup()
    ASSERT_NE(s.robot, nullptr);
    const S3Params p = looseParams();
    const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    ASSERT_EQ(r.best_per_k.size(), 4u);
    const GraspPointId ks[4] = {GraspPointId::kP0, GraspPointId::kP90, GraspPointId::kP180, GraspPointId::kP270};
    for (std::size_t i = 0; i < 4; ++i) EXPECT_EQ(r.best_per_k[i].eval.k, ks[i]);
}

TEST(GraspS3Test, InvalidInputsThrow) {
    REQUIRE_URDF_OR_SKIP();
    S3TestSetup s = makeS3TestSetup();
    s.ctx.prodmp_template = &s.tmpl;  // set here: S3TestSetup may be moved out of makeS3TestSetup()
    ASSERT_NE(s.robot, nullptr);
    S3Params p = looseParams();
    S3Context bad = s.ctx;
    bad.prodmp_template = nullptr;
    EXPECT_THROW(selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, bad, p), std::invalid_argument);
    p.theta_step_deg = 0.0;
    EXPECT_THROW(selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p), std::invalid_argument);
}

// ---- psi propagation and forced candidate (written without being able to build/run: see the hand-over report) ----

TEST(GraspS3PsiTest, GoalOrientationPsiZeroIsNominalBitForBit) {
    PoseEvaluation e;
    e.pose.orientation_nominal_world = Eigen::Quaterniond(0.3, -0.5, 0.2, 0.7).normalized();
    e.psi_rad = 0.0;
    e.wrist_flip = false;
    EXPECT_TRUE(goalOrientation(e).coeffs() == e.pose.orientation_nominal_world.coeffs());
    e.wrist_flip = true;  // psi = 0 with flip == the previous applyRoll(qn, pi)
    EXPECT_TRUE(goalOrientation(e).coeffs() ==
                haptic_dmp_learning::core::applyRoll(e.pose.orientation_nominal_world, M_PI).coeffs());
}

TEST(GraspS3PsiTest, GoalOrientationAddsRollAboutLocalZ) {
    const Eigen::Quaterniond qn = Eigen::Quaterniond(0.3, -0.5, 0.2, 0.7).normalized();
    const double psi = 4.0 * M_PI / 180.0;
    for (bool flip : {false, true}) {
        PoseEvaluation e;
        e.pose.orientation_nominal_world = qn;
        e.psi_rad = psi;
        e.wrist_flip = flip;
        const Eigen::Quaterniond want =
            (qn * Eigen::Quaterniond(Eigen::AngleAxisd(psi + (flip ? M_PI : 0.0), Eigen::Vector3d::UnitZ()))).normalized();
        EXPECT_LT(goalOrientation(e).angularDistance(want), 1e-12) << "flip=" << flip;
        // the approach axis (local z) does not move
        EXPECT_LT((goalOrientation(e) * Eigen::Vector3d::UnitZ() - qn * Eigen::Vector3d::UnitZ()).norm(), 1e-12);
    }
}

TEST(GraspS3PsiTest, EvaluationCarriesPsiAndBranchOrientations) {
    REQUIRE_URDF_OR_SKIP();
    S3TestSetup s = makeS3TestSetup();
    ASSERT_NE(s.robot, nullptr);
    const double psi = 3.0 * M_PI / 180.0;
    const PoseEvaluationDetail d = evaluateGraspPoseDetailed(s.model, GraspPointId::kP0, 0.2, psi, *s.robot, s.q_start,
                                                             s.demo, 0.10, 1.0, IkParams(), false);
    const Eigen::Quaterniond& qn = d.eval.pose.orientation_nominal_world;
    EXPECT_DOUBLE_EQ(d.eval.psi_rad, psi);
    EXPECT_LT(d.branch[0].orientation.angularDistance(haptic_dmp_learning::core::applyRoll(qn, psi)), 1e-12);
    EXPECT_LT(d.branch[1].orientation.angularDistance(haptic_dmp_learning::core::applyRoll(qn, M_PI + psi)), 1e-12);
    EXPECT_LT(goalOrientation(d.eval).angularDistance(d.branch[d.selected].orientation), 1e-12);
    // evaluateGraspPose (psi = 0 wrapper) keeps psi_rad == 0
    EXPECT_EQ(evaluateGraspPose(s.model, GraspPointId::kP0, 0.2, *s.robot, s.q_start, s.demo, 0.10, 1.0).psi_rad, 0.0);
}

TEST(GraspS3ForceTest, DefaultKeepsScanAndPsiZero) {
    REQUIRE_URDF_OR_SKIP();
    S3TestSetup s = makeS3TestSetup();
    s.ctx.prodmp_template = &s.tmpl;
    ASSERT_NE(s.robot, nullptr);
    const S3Params p = looseParams();
    ASSERT_FALSE(p.force);
    const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    if (r.status != S3Result::kSelected) GTEST_SKIP() << "no feasible candidate in this fixture: " << r.note;
    EXPECT_FALSE(r.forced);
    EXPECT_GT(r.evaluations, 1);
    EXPECT_EQ(r.chosen.psi_rad, 0.0);
    EXPECT_EQ(r.chosen.eval.psi_rad, 0.0);
    for (const auto& c : r.ranked) EXPECT_EQ(c.eval.psi_rad, 0.0);
    // same IK solution, bit for bit, as the pre-psi evaluateGraspPose at the chosen (k, theta)
    const PoseEvaluation old = evaluateGraspPose(s.model, r.chosen.eval.k, r.chosen.eval.theta_rad, *s.robot, s.q_start,
                                                 s.demo, p.length_scale_m, p.angle_scale_rad, p.ik);
    EXPECT_TRUE((old.ik.q.array() == r.chosen.eval.ik.q.array()).all());
    EXPECT_EQ(old.wrist_flip, r.chosen.eval.wrist_flip);
}

TEST(GraspS3ForceTest, ForceSelectsExactlyTheGivenCandidate) {
    REQUIRE_URDF_OR_SKIP();
    S3TestSetup s = makeS3TestSetup();
    s.ctx.prodmp_template = &s.tmpl;
    ASSERT_NE(s.robot, nullptr);
    const S3Params scan = looseParams();
    const S3Result r0 = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, scan);
    if (r0.status != S3Result::kSelected) GTEST_SKIP() << "no feasible candidate in this fixture: " << r0.note;
    // force a candidate that is NOT the winner (the second ranked one), psi = 0 first
    ASSERT_GE(r0.ranked.size(), 2u);
    const S3Candidate& other = r0.ranked[1];
    S3Params p = scan;
    p.force = true;
    p.force_k = static_cast<int>(other.eval.k);
    p.force_theta_deg = other.eval.theta_rad * 180.0 / M_PI;
    p.force_psi_deg = 0.0;
    const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    ASSERT_EQ(r.status, S3Result::kSelected) << r.note;
    EXPECT_TRUE(r.forced);
    EXPECT_EQ(r.evaluations, 1);
    EXPECT_EQ(r.chosen.eval.k, other.eval.k);
    EXPECT_NEAR(r.chosen.eval.theta_rad, other.eval.theta_rad, 1e-12);
    EXPECT_NEAR(r.chosen.wait_s, other.wait_s, 1e-6);  // wait computed at the forced theta
    EXPECT_NEAR(r.chosen.cost, other.cost, 1e-9);

    // psi != 0: carried into the candidate (selection may legitimately fail IK for this fixture: then skip)
    p.force_psi_deg = 2.0;
    const S3Result rp = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    if (rp.status != S3Result::kSelected) GTEST_SKIP() << "psi=2 deg not feasible in this fixture: " << rp.note;
    EXPECT_EQ(rp.evaluations, 1);
    EXPECT_DOUBLE_EQ(rp.chosen.psi_rad, 2.0 * M_PI / 180.0);
    EXPECT_DOUBLE_EQ(rp.chosen.eval.psi_rad, 2.0 * M_PI / 180.0);
}

TEST(GraspS3ForceTest, InfeasibleForcedCandidateGivesNoCandidateUnlessIgnored) {
    REQUIRE_URDF_OR_SKIP();
    S3TestSetup s = makeS3TestSetup();
    s.ctx.prodmp_template = &s.tmpl;
    ASSERT_NE(s.robot, nullptr);
    S3Params p = looseParams();
    p.w_hat_min = 5.0;  // nothing passes
    p.force = true;
    p.force_k = 0;
    p.force_theta_deg = 0.0;
    S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    EXPECT_EQ(r.status, S3Result::kNoCandidate);
    EXPECT_EQ(r.evaluations, 1);
    EXPECT_NE(r.note.find("FORCED"), std::string::npos);
    EXPECT_NE(r.note.find("w_hat"), std::string::npos);

    p.force_ignore_constraints = true;
    r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    EXPECT_EQ(r.status, S3Result::kSelected);
    EXPECT_EQ(r.evaluations, 1);
    EXPECT_FALSE(r.chosen.passes_constraints);
    EXPECT_NE(r.forced_violations.find("w_hat"), std::string::npos);
}

TEST(GraspS3ForceTest, InvalidForceInputsThrow) {
    S3Params p;
    p.force = true;
    EXPECT_NO_THROW(validateS3Params(p));
    S3Params q = p;
    q.force_k = 4;
    EXPECT_THROW(validateS3Params(q), std::invalid_argument);
    q = p;
    q.force_k = -1;
    EXPECT_THROW(validateS3Params(q), std::invalid_argument);
    q = p;
    q.force_theta_deg = 180.5;
    EXPECT_THROW(validateS3Params(q), std::invalid_argument);
    q = p;
    q.force_theta_deg = std::nan("");
    EXPECT_THROW(validateS3Params(q), std::invalid_argument);
    q = p;
    q.force_psi_deg = 8.5;
    EXPECT_THROW(validateS3Params(q), std::invalid_argument);
    q = p;
    q.force_psi_deg = -8.5;
    EXPECT_THROW(validateS3Params(q), std::invalid_argument);
    q = p;
    q.force = false;
    q.force_k = 9;  // ignored while force is off
    EXPECT_NO_THROW(validateS3Params(q));
}

// ---- force_flip (written without being able to build/run) ----

TEST(GraspS3FlipTest, EvaluationForcedBranchMatchesUnforcedBranchDetail) {
    REQUIRE_URDF_OR_SKIP();
    S3TestSetup s = makeS3TestSetup();
    ASSERT_NE(s.robot, nullptr);
    const double psi = 3.0 * M_PI / 180.0;
    const PoseEvaluationDetail all = evaluateGraspPoseDetailed(s.model, GraspPointId::kP0, 0.2, psi, *s.robot, s.q_start,
                                                               s.demo, 0.10, 1.0, IkParams(), false);
    const PoseEvaluationDetail dflt = evaluateGraspPoseDetailed(s.model, GraspPointId::kP0, 0.2, psi, *s.robot, s.q_start,
                                                                s.demo, 0.10, 1.0, IkParams(), false, -1);
    EXPECT_EQ(dflt.selected, all.selected);  // default argument == -1
    EXPECT_TRUE((dflt.eval.ik.q.array() == all.eval.ik.q.array()).all());
    for (int flip : {0, 1}) {
        const PoseEvaluationDetail d = evaluateGraspPoseDetailed(s.model, GraspPointId::kP0, 0.2, psi, *s.robot, s.q_start,
                                                                 s.demo, 0.10, 1.0, IkParams(), false, flip);
        EXPECT_EQ(d.selected, flip);
        EXPECT_EQ(d.eval.wrist_flip, flip == 1);
        const Eigen::Quaterniond want = haptic_dmp_learning::core::applyRoll(
            d.eval.pose.orientation_nominal_world, psi + (flip ? M_PI : 0.0));
        EXPECT_LT(goalOrientation(d.eval).angularDistance(want), 1e-12) << "flip=" << flip;
        // the forced branch is the same IK as the branch of the unforced call
        EXPECT_TRUE((d.eval.ik.q.array() == all.branch[flip].q.array()).all()) << "flip=" << flip;
    }
    EXPECT_THROW(evaluateGraspPoseDetailed(s.model, GraspPointId::kP0, 0.2, psi, *s.robot, s.q_start, s.demo, 0.10, 1.0,
                                           IkParams(), false, 2),
                 std::invalid_argument);
}

TEST(GraspS3FlipTest, SelectionForcedBranchAndGoalRoll) {
    REQUIRE_URDF_OR_SKIP();
    S3TestSetup s = makeS3TestSetup();
    s.ctx.prodmp_template = &s.tmpl;
    ASSERT_NE(s.robot, nullptr);
    S3Params p = looseParams();
    p.force = true;
    p.force_k = 0;
    p.force_theta_deg = 10.0;
    p.force_psi_deg = 2.0;
    for (int flip : {0, 1}) {
        p.force_flip = flip;
        const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
        if (r.status != S3Result::kSelected) continue;  // IK of this branch may not converge in the fixture
        EXPECT_EQ(r.evaluations, 1);
        EXPECT_EQ(r.chosen.eval.wrist_flip, flip == 1);
        const double psi = 2.0 * M_PI / 180.0;
        const Eigen::Quaterniond want = haptic_dmp_learning::core::applyRoll(
            r.chosen.eval.pose.orientation_nominal_world, psi + (flip ? M_PI : 0.0));
        EXPECT_LT(goalOrientation(r.chosen.eval).angularDistance(want), 1e-12);
    }
}

TEST(GraspS3FlipTest, NonConvergingForcedBranchGivesNoCandidateAndNoFallback) {
    REQUIRE_URDF_OR_SKIP();
    S3TestSetup s = makeS3TestSetup();
    s.ctx.prodmp_template = &s.tmpl;
    ASSERT_NE(s.robot, nullptr);
    // look for a (k, theta, flip) whose forced branch does not converge but the other one does
    bool found = false;
    int fk = 0, fflip = 0;
    double fth = 0.0;
    for (int k = 0; k < 4 && !found; ++k) {
        for (int i = 0; i < 72 && !found; ++i) {
            const double th = -M_PI + i * (5.0 * M_PI / 180.0);
            const PoseEvaluationDetail d = evaluateGraspPoseDetailed(s.model, static_cast<GraspPointId>(k), th, 0.0,
                                                                     *s.robot, s.q_start, s.demo, 0.10, 1.0, IkParams(), false);
            for (int flip : {0, 1}) {
                if (!d.branch[flip].converged && d.branch[1 - flip].converged) {
                    found = true; fk = k; fth = th; fflip = flip;
                }
            }
        }
    }
    if (!found) GTEST_SKIP() << "no (k, theta) with exactly one converging branch in this fixture";
    S3Params p = looseParams();
    p.force = true;
    p.force_k = fk;
    p.force_theta_deg = fth * 180.0 / M_PI;
    p.force_flip = fflip;
    S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    EXPECT_EQ(r.status, S3Result::kNoCandidate);  // NOT the other (converging) branch
    EXPECT_EQ(r.evaluations, 1);
    EXPECT_NE(r.note.find("ik"), std::string::npos);
    EXPECT_NE(r.note.find("force_flip=" + std::to_string(fflip)), std::string::npos);
    p.force_ignore_constraints = true;
    r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    ASSERT_EQ(r.status, S3Result::kSelected);
    EXPECT_EQ(r.chosen.eval.wrist_flip, fflip == 1);  // still the requested branch
}

TEST(GraspS3FlipTest, DefaultAutoAndValidation) {
    S3Params p;
    EXPECT_EQ(p.force_flip, -1);
    EXPECT_NO_THROW(validateS3Params(p));
    p.force = true;
    p.force_flip = 1;
    EXPECT_NO_THROW(validateS3Params(p));
    p.force_flip = 2;
    EXPECT_THROW(validateS3Params(p), std::invalid_argument);
    p.force_flip = -2;
    EXPECT_THROW(validateS3Params(p), std::invalid_argument);
    p.force = false;
    p.force_flip = 0;  // a forced branch without force is an error, not ignored
    EXPECT_THROW(validateS3Params(p), std::invalid_argument);
}

// ---- branch_rule (written without being able to build/run) ----
//
// Inputs: makeS3TestSetup() (Panda fixture, q_start = {0,-0.7854,0,-2.3562,0,3.1416,0.7854}, cube at the fixture
// center, NOT the Gazebo q0 / cube pose of the measured runs, which are not available to the tests). The measured
// situations are reproduced through demo.orientation (the only quantity the rotation rule reads): for the nominal
// orientation A and B = A * Rz(pi) of a candidate, demo.orientation = applyRoll(A, 16.5 deg) gives angles 16.5 / 163.5
// (k0 case: A closer), demo.orientation = applyRoll(B, 34.5 deg) gives 145.5 / 34.5 (k2 case: B closer).
// (k, theta) = (0, -163.5 deg) and (2, 145.5 deg) are tried first; if a branch does not converge there in this fixture,
// theta is increased in 5 deg steps to the first one where both do (the test prints the one used).

namespace {

struct BothBranches {
    bool found = false;
    double theta_rad = 0.0;
    PoseEvaluationDetail det;  // default rule, demo = s.demo
};

BothBranches findBothConverging(S3TestSetup& s, GraspPointId k, double theta_deg) {
    BothBranches out;
    for (int i = 0; i < 72; ++i) {
        const double th = (theta_deg + 5.0 * i) * M_PI / 180.0;
        const PoseEvaluationDetail d = evaluateGraspPoseDetailed(s.model, k, th, 0.0, *s.robot, s.q_start, s.demo, 0.10,
                                                                 1.0, IkParams(), false);
        if (d.branch[0].converged && d.branch[1].converged) {
            out.found = true;
            out.theta_rad = th;
            out.det = d;
            return out;
        }
    }
    return out;
}

PoseEvaluationDetail evalWith(S3TestSetup& s, GraspPointId k, double theta, const Eigen::Quaterniond& demo_ori, int flip,
                              BranchRule rule) {
    DemoReference demo = s.demo;
    demo.orientation = demo_ori;
    return evaluateGraspPoseDetailed(s.model, k, theta, 0.0, *s.robot, s.q_start, demo, 0.10, 1.0, IkParams(), false, flip,
                                     rule);
}

}  // namespace

TEST(GraspS3BranchRuleTest, RotationPicksSmallerAngle_k0_branchA_and_k2_branchB) {
    REQUIRE_URDF_OR_SKIP();
    S3TestSetup s = makeS3TestSetup();
    ASSERT_NE(s.robot, nullptr);
    using haptic_dmp_learning::core::applyRoll;
    const double deg = M_PI / 180.0;

    const BothBranches b0 = findBothConverging(s, GraspPointId::kP0, -163.5);
    if (!b0.found) GTEST_SKIP() << "k0: no theta with both branches converging in this fixture";
    std::cout << "k0 theta used: " << b0.theta_rad / deg << " deg\n";
    const Eigen::Quaterniond A0 = b0.det.branch[0].orientation;
    const PoseEvaluationDetail r0 = evalWith(s, GraspPointId::kP0, b0.theta_rad, applyRoll(A0, 16.5 * deg), -1, BranchRule::kRotation);
    EXPECT_NEAR(r0.eval.branch_rot_rad[0], 16.5 * deg, 1e-9);
    EXPECT_NEAR(r0.eval.branch_rot_rad[1], 163.5 * deg, 1e-9);
    EXPECT_EQ(r0.selected, 0);
    EXPECT_FALSE(r0.eval.wrist_flip);
    EXPECT_DOUBLE_EQ(r0.eval.dist_to_demo.ori_rad, r0.eval.branch_rot_rad[r0.selected]);

    const BothBranches b2 = findBothConverging(s, GraspPointId::kP180, 145.5);
    if (!b2.found) GTEST_SKIP() << "k2: no theta with both branches converging in this fixture";
    std::cout << "k2 theta used: " << b2.theta_rad / deg << " deg\n";
    const Eigen::Quaterniond B2 = b2.det.branch[1].orientation;
    const PoseEvaluationDetail r2 = evalWith(s, GraspPointId::kP180, b2.theta_rad, applyRoll(B2, 34.5 * deg), -1, BranchRule::kRotation);
    EXPECT_NEAR(r2.eval.branch_rot_rad[0], 145.5 * deg, 1e-9);
    EXPECT_NEAR(r2.eval.branch_rot_rad[1], 34.5 * deg, 1e-9);
    EXPECT_EQ(r2.selected, 1);
    EXPECT_TRUE(r2.eval.wrist_flip);
    EXPECT_DOUBLE_EQ(r2.eval.dist_to_demo.ori_rad, r2.eval.branch_rot_rad[r2.selected]);
}

TEST(GraspS3BranchRuleTest, TieWithin1mradFallsBackToMarginRule) {
    REQUIRE_URDF_OR_SKIP();
    S3TestSetup s = makeS3TestSetup();
    ASSERT_NE(s.robot, nullptr);
    using haptic_dmp_learning::core::applyRoll;
    const BothBranches b = findBothConverging(s, GraspPointId::kP0, -163.5);
    if (!b.found) GTEST_SKIP() << "no theta with both branches converging in this fixture";
    const Eigen::Quaterniond A = b.det.branch[0].orientation;
    // Branch chosen by the margin rule with this demo (the margin rule does not read demo.orientation).
    const int x = evalWith(s, GraspPointId::kP0, b.theta_rad, A, -1, BranchRule::kMargin).selected;
    // Roll of pi/2 + delta from A: angle(A) = pi/2 + delta, angle(B) = pi/2 - delta. delta is signed so that the OTHER
    // branch (1 - x) has the smaller angle: a pure angle rule would pick it, the tie rule must keep x.
    const double sign = (x == 0) ? 1.0 : -1.0;
    const double inside = 0.4e-3, outside = 0.6e-3;  // angle gap 0.8e-3 (< 1e-3) and 1.2e-3 (> 1e-3)

    const PoseEvaluationDetail tie = evalWith(s, GraspPointId::kP0, b.theta_rad,
                                              applyRoll(A, M_PI / 2.0 + sign * inside), -1, BranchRule::kRotation);
    EXPECT_NEAR(std::abs(tie.eval.branch_rot_rad[0] - tie.eval.branch_rot_rad[1]), 2.0 * inside, 1e-9);
    EXPECT_EQ(tie.selected, x);
    EXPECT_TRUE((tie.eval.ik.q.array() == b.det.branch[x].q.array()).all());

    const PoseEvaluationDetail apart = evalWith(s, GraspPointId::kP0, b.theta_rad,
                                                applyRoll(A, M_PI / 2.0 + sign * outside), -1, BranchRule::kRotation);
    EXPECT_EQ(apart.selected, 1 - x);  // gap > 1e-3: the smaller angle wins
}

TEST(GraspS3BranchRuleTest, MarginRuleUnchangedAndIgnoresDemoOrientation) {
    REQUIRE_URDF_OR_SKIP();
    S3TestSetup s = makeS3TestSetup();
    s.ctx.prodmp_template = &s.tmpl;
    ASSERT_NE(s.robot, nullptr);
    using haptic_dmp_learning::core::applyRoll;
    for (GraspPointId k : {GraspPointId::kP0, GraspPointId::kP180}) {
        for (double th : {-163.5, 0.0, 145.5}) {
            const double theta = th * M_PI / 180.0;
            const PoseEvaluationDetail dflt = evaluateGraspPoseDetailed(s.model, k, theta, 0.0, *s.robot, s.q_start, s.demo,
                                                                        0.10, 1.0, IkParams(), false);
            for (const Eigen::Quaterniond& dq : {s.demo.orientation, applyRoll(s.demo.orientation, 1.0), applyRoll(s.demo.orientation, M_PI)}) {
                const PoseEvaluationDetail m = evalWith(s, k, theta, dq, -1, BranchRule::kMargin);
                EXPECT_EQ(m.selected, dflt.selected);
                EXPECT_TRUE((m.eval.ik.q.array() == dflt.eval.ik.q.array()).all());
                EXPECT_EQ(m.eval.margins.min_margin_rad, dflt.eval.margins.min_margin_rad);
            }
        }
    }
    // Full selection: default S3Params == explicit "margin".
    S3Params p = looseParams();
    EXPECT_EQ(p.branch_rule, "margin");
    const S3Result r0 = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    p.branch_rule = "margin";
    const S3Result r1 = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    ASSERT_EQ(r0.status, r1.status);
    EXPECT_EQ(r0.evaluations, r1.evaluations);
    if (r0.status == S3Result::kSelected) {
        EXPECT_EQ(r0.chosen.eval.wrist_flip, r1.chosen.eval.wrist_flip);
        EXPECT_EQ(r0.chosen.eval.theta_rad, r1.chosen.eval.theta_rad);
        EXPECT_EQ(r0.chosen.cost, r1.chosen.cost);
    }
}

TEST(GraspS3BranchRuleTest, UnknownValueIsAnError) {
    EXPECT_EQ(parseBranchRule("margin"), BranchRule::kMargin);
    EXPECT_EQ(parseBranchRule("rotation"), BranchRule::kRotation);
    for (const char* bad : {"", "Margin", "ROTATION", "rotation ", "auto", "margine"}) {
        EXPECT_THROW(parseBranchRule(bad), std::invalid_argument) << "'" << bad << "'";
        S3Params p;
        p.branch_rule = bad;
        EXPECT_THROW(validateS3Params(p), std::invalid_argument) << "'" << bad << "'";
    }
    S3Params ok;
    ok.branch_rule = "rotation";
    EXPECT_NO_THROW(validateS3Params(ok));

    REQUIRE_URDF_OR_SKIP();
    S3TestSetup s = makeS3TestSetup();
    s.ctx.prodmp_template = &s.tmpl;
    ASSERT_NE(s.robot, nullptr);
    S3Params p = looseParams();
    p.branch_rule = "bogus";
    EXPECT_THROW(selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p), std::invalid_argument);
}

TEST(GraspS3BranchRuleTest, ForceFlipTakesPrecedenceOverRotation) {
    REQUIRE_URDF_OR_SKIP();
    S3TestSetup s = makeS3TestSetup();
    s.ctx.prodmp_template = &s.tmpl;
    ASSERT_NE(s.robot, nullptr);
    using haptic_dmp_learning::core::applyRoll;
    const BothBranches b = findBothConverging(s, GraspPointId::kP0, -163.5);
    if (!b.found) GTEST_SKIP() << "no theta with both branches converging in this fixture";
    const Eigen::Quaterniond A = b.det.branch[0].orientation;
    const Eigen::Quaterniond demo_ori = applyRoll(A, 10.0 * M_PI / 180.0);  // rotation rule alone would pick branch 0
    ASSERT_EQ(evalWith(s, GraspPointId::kP0, b.theta_rad, demo_ori, -1, BranchRule::kRotation).selected, 0);
    const PoseEvaluationDetail f = evalWith(s, GraspPointId::kP0, b.theta_rad, demo_ori, 1, BranchRule::kRotation);
    EXPECT_EQ(f.selected, 1);
    EXPECT_TRUE(f.eval.wrist_flip);
    EXPECT_TRUE((f.eval.ik.q.array() == b.det.branch[1].q.array()).all());
    EXPECT_NEAR(f.eval.branch_rot_rad[0], 10.0 * M_PI / 180.0, 1e-9);  // filled even for the branch not solved/chosen

    // Same through selectGraspS3 (forced candidate, rotation rule, force_flip = 1).
    DemoReference demo = s.demo;
    demo.orientation = demo_ori;
    S3Params p = looseParams();
    p.branch_rule = "rotation";
    p.force = true;
    p.force_k = 0;
    p.force_theta_deg = b.theta_rad * 180.0 / M_PI;
    p.force_flip = 1;
    p.force_ignore_constraints = true;
    const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, demo, s.delta_g, s.ctx, p);
    ASSERT_EQ(r.status, S3Result::kSelected);
    EXPECT_TRUE(r.chosen.eval.wrist_flip);
}

// ---- s1.mode "v2" (written without being able to build/run) ----

namespace {

S3Params v2Params() {
    S3Params p = looseParams();
    p.mode = "v2";
    p.branch_rule = "rotation";
    const RobotModel::JointVector q = RobotModel::readyPose();
    p.rollout_q_ref.assign(q.data(), q.data() + 7);
    return p;
}

PoseEvaluation syntheticEval(double ori_deg) {
    PoseEvaluation e;
    e.feasible_basic = true;
    e.w_hat = 0.0;                         // would fail w_hat_min in legacy
    e.margins.min_margin_rad = 0.0;        // would fail margin_min_rad in legacy
    e.dist_to_demo.ori_rad = ori_deg * M_PI / 180.0;
    return e;
}

}  // namespace

TEST(GraspS3V2Test, LegacyIsTheDefaultAndModeIsValidated) {
    S3Params p;
    EXPECT_EQ(p.mode, "legacy");
    EXPECT_NO_THROW(validateS3Params(p));
    EXPECT_EQ(parseS3Mode("legacy"), S3Mode::kLegacy);
    EXPECT_EQ(parseS3Mode("v2"), S3Mode::kV2);
    for (const char* bad : {"", "V2", "Legacy", "v3", "v2 "}) {
        EXPECT_THROW(parseS3Mode(bad), std::invalid_argument) << "'" << bad << "'";
        S3Params q;
        q.mode = bad;
        EXPECT_THROW(validateS3Params(q), std::invalid_argument) << "'" << bad << "'";
    }
}

TEST(GraspS3V2Test, LegacySelectionIgnoresEveryV2Parameter) {
    REQUIRE_URDF_OR_SKIP();
    S3TestSetup s = makeS3TestSetup();
    s.ctx.prodmp_template = &s.tmpl;
    ASSERT_NE(s.robot, nullptr);
    const S3Params base = looseParams();            // mode = "legacy" by default
    S3Params other = looseParams();
    other.mode = "legacy";
    other.max_start_goal_deg = 10.0;
    other.finalists_n = 1;
    other.use_w = false;
    other.kappa_w_mm = 99.0;
    other.kappa_t_mm_s = 5.0;
    other.tau_j_mm = 50.0;
    const RobotModel::JointVector q = RobotModel::readyPose();
    other.rollout_q_ref.assign(q.data(), q.data() + 7);
    const S3Result a = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, base);
    const S3Result b = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, other);
    ASSERT_EQ(a.status, b.status);
    EXPECT_EQ(a.evaluations, b.evaluations);
    EXPECT_EQ(a.ranked.size(), b.ranked.size());
    EXPECT_TRUE(a.v2_finalists.empty());
    EXPECT_TRUE(b.v2_finalists.empty());
    if (a.status == S3Result::kSelected) {
        EXPECT_EQ(a.chosen.eval.theta_rad, b.chosen.eval.theta_rad);
        EXPECT_EQ(a.chosen.eval.wrist_flip, b.chosen.eval.wrist_flip);
        EXPECT_EQ(a.chosen.cost, b.chosen.cost);
        EXPECT_EQ(a.chosen.reject_mask, b.chosen.reject_mask);
    }
}

TEST(GraspS3V2Test, EqualDistanceHigherWWins) {
    const S3Params p;  // use_w = true, kappa_w = 7.5 mm, tau_j = 2 mm
    const double d_mm = 10.0;
    // Candidate 0: w = 0.02, short wait. Candidate 1: w = 0.04, long wait. DeltaJ = 7.5 ln 2 = 5.2 mm > tau_j.
    const double j0 = s3CostV2(d_mm, 0.02, 1.0, p);
    const double j1 = s3CostV2(d_mm, 0.04, 100.0, p);  // kappa_t = 0: the wait does not enter J
    EXPECT_LT(j1, j0 - p.tau_j_mm);
    EXPECT_EQ(s3PickWinnerV2({j0, j1}, {1.0, 100.0}, p.tau_j_mm), 1);
    EXPECT_EQ(s3PickWinnerV2({j1, j0}, {100.0, 1.0}, p.tau_j_mm), 0);
}

TEST(GraspS3V2Test, WithoutWEqualDistanceSmallerWaitWins) {
    S3Params p;
    p.use_w = false;
    const double d_mm = 10.0;
    const double j0 = s3CostV2(d_mm, 0.04, 50.0, p);   // higher w, longer wait
    const double j1 = s3CostV2(d_mm, 0.02, 5.0, p);
    EXPECT_DOUBLE_EQ(j0, d_mm);
    EXPECT_DOUBLE_EQ(j1, d_mm);
    EXPECT_EQ(s3PickWinnerV2({j0, j1}, {50.0, 5.0}, 0.0), 1);
    EXPECT_EQ(s3PickWinnerV2({j0, j1}, {5.0, 50.0}, 0.0), 0);
    // a singular candidate (w = 0) is not penalised when the w term is off
    EXPECT_DOUBLE_EQ(s3CostV2(d_mm, 0.0, 0.0, p), d_mm);
}

TEST(GraspS3V2Test, TieBandPicksSmallerWait) {
    S3Params p;
    const double j_lo_w = s3CostV2(10.0, 0.040, 0.0, p);
    const double j_hi_w = s3CostV2(10.0, 0.044, 0.0, p);  // better by 7.5 ln 1.1 = 0.71 mm
    ASSERT_LT(j_hi_w, j_lo_w);
    ASSERT_LT(j_lo_w - j_hi_w, p.tau_j_mm);
    // inside the band (2 mm): the lower-w candidate with the smaller wait wins
    EXPECT_EQ(s3PickWinnerV2({j_lo_w, j_hi_w}, {1.0, 9.0}, p.tau_j_mm), 0);
    // band narrower than the J gap: the better J wins whatever the wait
    EXPECT_EQ(s3PickWinnerV2({j_lo_w, j_hi_w}, {1.0, 9.0}, 0.5), 1);
    // band edge is inclusive (J <= J_min + tau)
    EXPECT_EQ(s3PickWinnerV2({12.0, 10.0}, {1.0, 9.0}, 2.0), 0);
    EXPECT_EQ(s3PickWinnerV2({12.0001, 10.0}, {1.0, 9.0}, 2.0), 1);
    // kappa_t adds to J
    p.kappa_t_mm_s = 0.5;
    EXPECT_NEAR(s3CostV2(10.0, 0.04, 4.0, p) - s3CostV2(10.0, 0.04, 0.0, p), 2.0, 1e-12);
    EXPECT_EQ(s3PickWinnerV2({}, {}, 1.0), -1);
    EXPECT_THROW(s3PickWinnerV2({1.0}, {1.0, 2.0}, 1.0), std::invalid_argument);
    EXPECT_THROW(s3PickWinnerV2({1.0}, {1.0}, -1.0), std::invalid_argument);
}

TEST(GraspS3V2Test, MaxStartGoalDiscardsAndV2SkipsWHatAndMargin) {
    S3Params p;
    p.mode = "v2";
    const PoseEvaluation far = syntheticEval(100.0);
    p.max_start_goal_deg = 90.0;
    EXPECT_EQ(s3AprioriRejectMaskV2(far, 0.0, p) & kRejectStartGoal, static_cast<unsigned>(kRejectStartGoal));
    p.max_start_goal_deg = 110.0;
    EXPECT_EQ(s3AprioriRejectMaskV2(far, 0.0, p), 0u);   // w_hat = 0 and margin = 0 are NOT rejected in v2
    p.max_start_goal_deg = 180.0;                         // disabled, even at exactly pi
    EXPECT_EQ(s3AprioriRejectMaskV2(syntheticEval(180.0), 0.0, p), 0u);
    // r_trust and ik still apply
    p.r_trust_m = 0.1;
    EXPECT_EQ(s3AprioriRejectMaskV2(far, 0.2, p), static_cast<unsigned>(kRejectRTrust));
    PoseEvaluation bad = far;
    bad.feasible_basic = false;
    EXPECT_EQ(s3AprioriRejectMaskV2(bad, 0.0, p), static_cast<unsigned>(kRejectIk));
    // legacy mask: unchanged bits, never the new one
    const unsigned legacy = s3AprioriRejectMask(far, 0.0, S3Params());
    EXPECT_EQ(legacy, static_cast<unsigned>(kRejectWHat | kRejectMargin));
    EXPECT_EQ(legacy & kRejectStartGoal, 0u);
}

TEST(GraspS3V2Test, ValidationErrors) {
    EXPECT_NO_THROW(validateS3Params(v2Params()));
    auto bad = [](auto mutate) {
        S3Params p = v2Params();
        mutate(p);
        EXPECT_THROW(validateS3Params(p), std::invalid_argument);
    };
    bad([](S3Params& p) { p.kappa_w_mm = -0.1; });
    bad([](S3Params& p) { p.kappa_t_mm_s = -0.1; });
    bad([](S3Params& p) { p.tau_j_mm = -0.1; });
    bad([](S3Params& p) { p.finalists_n = 0; });
    bad([](S3Params& p) { p.max_start_goal_deg = 181.0; });
    bad([](S3Params& p) { p.max_start_goal_deg = -1.0; });
    bad([](S3Params& p) { p.rollout_q_ref.pop_back(); });                 // 6 values
    bad([](S3Params& p) { p.rollout_q_ref.push_back(0.0); });             // 8 values
    bad([](S3Params& p) { p.rollout_q_ref.clear(); });                    // v2: mandatory
    bad([](S3Params& p) { p.rollout_q_ref[2] = std::nan(""); });
    S3Params legacy;                                                      // legacy: empty is fine, wrong length is not
    EXPECT_NO_THROW(validateS3Params(legacy));
    legacy.rollout_q_ref = {0.0, 0.0, 0.0};
    EXPECT_THROW(validateS3Params(legacy), std::invalid_argument);
}

TEST(GraspS3V2Test, MissingRolloutQRefIsAnError) {
    REQUIRE_URDF_OR_SKIP();
    S3TestSetup s = makeS3TestSetup();
    s.ctx.prodmp_template = &s.tmpl;
    ASSERT_NE(s.robot, nullptr);
    S3Params p = looseParams();
    p.mode = "v2";  // rollout_q_ref left empty
    EXPECT_THROW(selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p), std::invalid_argument);
    p = v2Params();
    p.rollout_q_ref[1] = 10.0;  // outside the URDF joint limits
    EXPECT_THROW(selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p), std::invalid_argument);
}

TEST(GraspS3V2Test, EndToEndFinalistsAndWinner) {
    REQUIRE_URDF_OR_SKIP();
    S3TestSetup s = makeS3TestSetup();
    s.ctx.prodmp_template = &s.tmpl;
    ASSERT_NE(s.robot, nullptr);
    S3Params p = v2Params();
    p.w_hat_min = 1e9;      // would reject everything in legacy; ignored in v2
    p.margin_min_rad = 1e9; // idem
    p.finalists_n = 3;
    const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    if (r.status != S3Result::kSelected) GTEST_SKIP() << "no v2 candidate in this fixture: " << r.note;
    ASSERT_FALSE(r.v2_finalists.empty());
    EXPECT_LE(r.v2_finalists.size(), 3u);
    int winners = 0;
    double j_min = 1e300;
    for (const auto& f : r.v2_finalists) {
        EXPECT_TRUE(f.rollout_done);
        if (f.rollout_feasible) j_min = std::min(j_min, f.j);
        winners += f.winner ? 1 : 0;
    }
    EXPECT_EQ(winners, 1);
    for (const auto& f : r.v2_finalists) {
        if (!f.winner) continue;
        EXPECT_TRUE(f.rollout_feasible);
        EXPECT_LE(f.j, j_min + p.tau_j_mm);
        EXPECT_DOUBLE_EQ(r.chosen.cost, f.j);
        for (const auto& g : r.v2_finalists) {
            if (g.rollout_feasible && g.j <= j_min + p.tau_j_mm) EXPECT_GE(g.t_wait_s, f.t_wait_s);
        }
    }
    // one finalist per (k, branch, theta cluster)
    for (std::size_t i = 0; i < r.v2_finalists.size(); ++i) {
        for (std::size_t j = i + 1; j < r.v2_finalists.size(); ++j) {
            const auto& a = r.v2_finalists[i].cand.eval;
            const auto& b = r.v2_finalists[j].cand.eval;
            if (a.k != b.k || a.wrist_flip != b.wrist_flip) continue;
            double d = std::fmod(std::abs(a.theta_rad - b.theta_rad), 2.0 * M_PI);
            if (d > M_PI) d = 2.0 * M_PI - d;
            EXPECT_GE(d * 180.0 / M_PI, p.posthoc_cluster_deg);
        }
    }
}
