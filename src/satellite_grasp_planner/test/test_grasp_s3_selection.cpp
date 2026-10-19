// Tests for selectGraspS3 (synthetic ProDMP template). Need Pinocchio and $HOME/thesis_ws/fer_flat_effort.urdf
// (tests skip if it is missing, or if the fixture geometry yields no constraint-passing candidate).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <fstream>
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
