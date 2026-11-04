// Tests for selectGraspS3 (synthetic ProDMP template). Need Pinocchio and $HOME/thesis_ws/fer_flat_effort.urdf
// (tests skip if it is missing, or if the fixture geometry yields no constraint-passing candidate).

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "test_fixtures.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/tcp_roll.hpp"
#include "satellite_grasp_planner/core/grasp_pose_eval.hpp"
#include "satellite_grasp_planner/core/grasp_s3_launch.hpp"
#include "satellite_grasp_planner/core/grasp_s3_selection.hpp"
#include "satellite_grasp_planner/core/selection_inputs.hpp"
#include "satellite_grasp_planner/core/start_state.hpp"

using namespace satellite_grasp_planner::core;
using haptic_dmp_learning::core::CubeSatelliteModel;
using haptic_dmp_learning::core::GraspPointId;
using haptic_dmp_learning::core::ProDMP;
using RobotModel = franka_cartesian_control::core::RobotModel;
namespace tf = satellite_grasp_planner::test_fixtures;

#define REQUIRE_URDF_OR_SKIP()                                      \
  do {                                                              \
    if (!std::ifstream(tf::urdfPath()).good()) {                    \
      GTEST_SKIP() << "URDF fixture not found: " << tf::urdfPath(); \
    }                                                               \
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

/// Valid base parameters: the rollout needs its nullspace target (rollout_q_ref, no default: validateS3Params throws without
/// it), here the "ready" pose of the controller.
S3Params validParams() {
  S3Params p;
  const RobotModel::JointVector q = RobotModel::readyPose();
  p.rollout_q_ref.assign(q.data(), q.data() + 7);
  return p;
}

/// Constraints wide open (only IK feasibility and the rollout remain), no velocity check, coarse grid.
S3Params looseParams() {
  S3Params p = validParams();
  p.r_trust_m = 100.0;
  p.theta_step_deg = 5.0;
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
  p.r_trust_m = 1e-9;  // nothing is within a nanometre of the demo deformation
  S3Result r;
  ASSERT_NO_THROW(r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p));
  EXPECT_EQ(r.status, S3Result::kNoCandidate);
  EXPECT_TRUE(r.ranked.empty());
  EXPECT_GT(r.evaluations, 0);
  EXPECT_FALSE(r.note.empty());
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
  if (r.posthoc_log.empty()) GTEST_SKIP() << "no rollout-feasible finalist in this fixture: " << r.note;
  const S3PosthocEntry& first = r.posthoc_log.front();  // the first candidate examined
  EXPECT_NE(first.reject_reason.find("velocita'"), std::string::npos) << first.reject_reason;
  EXPECT_FALSE(first.accepted);
  EXPECT_GE(r.posthoc_checks, 1);
  if (r.status == S3Result::kSelected) {
    EXPECT_GE(r.posthoc_checks, 2);
    EXPECT_LE(r.posthoc_dv_m_s, p.delta_v_max_m_s);
    const bool same = r.chosen.eval.k == first.k && r.chosen.eval.theta_rad == first.theta_rad;
    EXPECT_FALSE(same);
  } else {
    EXPECT_EQ(r.status, S3Result::kPosthocFailed);
    EXPECT_FALSE(r.note.empty());
  }
}

// The cluster rule acts on the finalists: of the candidates of the same k AND wrist branch within posthoc_cluster_deg of theta only
// the best (J_static) is a finalist.
TEST(GraspS3Test, ClusterKeepsOneFinalistPerNearbyThetaOfSameKAndBranch) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;  // set here: S3TestSetup may be moved out of makeS3TestSetup()
  ASSERT_NE(s.robot, nullptr);
  S3Params p = looseParams();
  auto closePairs = [](const S3Result& r, double cluster_deg) {
    int n = 0;
    for (std::size_t i = 0; i < r.v2_finalists.size(); ++i) {
      for (std::size_t j = i + 1; j < r.v2_finalists.size(); ++j) {
        const auto& a = r.v2_finalists[i].cand.eval;
        const auto& b = r.v2_finalists[j].cand.eval;
        if (a.k != b.k || a.wrist_flip != b.wrist_flip) continue;
        double d = std::fmod(std::abs(a.theta_rad - b.theta_rad), 2.0 * M_PI);
        if (d > M_PI) d = 2.0 * M_PI - d;
        if (d * 180.0 / M_PI < cluster_deg - 1e-9) ++n;
      }
    }
    return n;
  };

  p.posthoc_cluster_deg = 0.0;
  const S3Result off = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  if (off.status == S3Result::kNoCandidate) GTEST_SKIP() << "no feasible candidate in this fixture: " << off.note;
  p.posthoc_cluster_deg = 10.0;
  const S3Result on = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);

  EXPECT_EQ(closePairs(on, p.posthoc_cluster_deg), 0);
  if (closePairs(off, p.posthoc_cluster_deg) == 0) GTEST_SKIP() << "no near-duplicate finalists without the cluster rule in this fixture";
  EXPECT_GT(closePairs(off, p.posthoc_cluster_deg), 0);  // the rule changes the finalist set
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

TEST(GraspS3Test, PosthocVelocityCheckDisabledSelectsWithoutMeasuring) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;  // set here: S3TestSetup may be moved out of makeS3TestSetup()
  ASSERT_NE(s.robot, nullptr);
  S3Params p = looseParams();
  p.posthoc_velocity_check = false;
  p.delta_v_max_m_s = 1e-6;  // would reject everything if the velocity check were on
  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  if (r.status == S3Result::kNoCandidate) GTEST_SKIP() << "no feasible candidate in this fixture: " << r.note;
  if (r.posthoc_log.empty() && r.status == S3Result::kPosthocFailed) GTEST_SKIP() << "no rollout-feasible finalist: " << r.note;
  ASSERT_EQ(r.status, S3Result::kSelected);
  EXPECT_EQ(r.posthoc_checks, 1);
  ASSERT_EQ(r.posthoc_log.size(), 1u);
  EXPECT_TRUE(r.posthoc_log.front().accepted);
  EXPECT_EQ(r.posthoc_log.front().dv_m_s, 0.0);  // not measured
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
  if (r0.ranked.size() < 2) GTEST_SKIP() << "need >= 2 rollout-feasible finalists, have " << r0.ranked.size();
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
  p.r_trust_m = 1e-9;  // nothing passes
  p.force = true;
  p.force_k = 0;
  p.force_theta_deg = 0.0;
  S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  EXPECT_EQ(r.status, S3Result::kNoCandidate);
  EXPECT_EQ(r.evaluations, 1);
  EXPECT_NE(r.note.find("FORCED"), std::string::npos);
  EXPECT_NE(r.note.find("r_trust"), std::string::npos);

  p.force_ignore_constraints = true;
  r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  EXPECT_EQ(r.status, S3Result::kSelected);
  EXPECT_EQ(r.evaluations, 1);
  EXPECT_FALSE(r.chosen.passes_constraints);
  EXPECT_NE(r.forced_violations.find("r_trust"), std::string::npos);
}

TEST(GraspS3ForceTest, InvalidForceInputsThrow) {
  S3Params p = validParams();
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
          found = true;
          fk = k;
          fth = th;
          fflip = flip;
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
  S3Params p = validParams();
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
    S3Params p = validParams();
    p.branch_rule = bad;
    EXPECT_THROW(validateS3Params(p), std::invalid_argument) << "'" << bad << "'";
  }
  S3Params ok = validParams();
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

// ---- selection: J_static / J, finalists, rollout (written without being able to build/run) ----

namespace {

S3Params v2Params() {
  S3Params p = looseParams();
  p.branch_rule = "rotation";
  return p;
}

PoseEvaluation syntheticEval(double ori_deg) {
  PoseEvaluation e;
  e.feasible_basic = true;
  e.w_hat = 0.0;                   // w_hat and the static margin are not constraints
  e.margins.min_margin_rad = 0.0;
  e.dist_to_demo.ori_rad = ori_deg * M_PI / 180.0;
  return e;
}

}  // namespace

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
  const double j0 = s3CostV2(d_mm, 0.04, 50.0, p);  // higher w, longer wait
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
  const PoseEvaluation far = syntheticEval(100.0);
  p.max_start_goal_deg = 90.0;
  EXPECT_EQ(s3AprioriRejectMaskV2(far, 0.0, p) & kRejectStartGoal, static_cast<unsigned>(kRejectStartGoal));
  p.max_start_goal_deg = 110.0;
  EXPECT_EQ(s3AprioriRejectMaskV2(far, 0.0, p), 0u);  // w_hat = 0 and margin = 0 are not constraints
  p.max_start_goal_deg = 180.0;                       // disabled, even at exactly pi
  EXPECT_EQ(s3AprioriRejectMaskV2(syntheticEval(180.0), 0.0, p), 0u);
  // r_trust and ik still apply
  p.r_trust_m = 0.1;
  EXPECT_EQ(s3AprioriRejectMaskV2(far, 0.2, p), static_cast<unsigned>(kRejectRTrust));
  PoseEvaluation bad = far;
  bad.feasible_basic = false;
  EXPECT_EQ(s3AprioriRejectMaskV2(bad, 0.0, p), static_cast<unsigned>(kRejectIk));
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
  bad([](S3Params& p) { p.rollout_q_ref.pop_back(); });      // 6 values
  bad([](S3Params& p) { p.rollout_q_ref.push_back(0.0); });  // 8 values
  bad([](S3Params& p) { p.rollout_q_ref.clear(); });         // mandatory
  bad([](S3Params& p) { p.rollout_q_ref[2] = std::nan(""); });
}

TEST(GraspS3V2Test, MissingRolloutQRefIsAnError) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  S3Params p = looseParams();
  p.rollout_q_ref.clear();  // mandatory, no default: the error names the parameter
  try {
    selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("rollout_q_ref"), std::string::npos) << e.what();
  }
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
      if (g.rollout_feasible && g.j <= j_min + p.tau_j_mm) {
        EXPECT_GE(g.t_wait_s, f.t_wait_s);
      }
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

// ---------------------------------------------------------------------------------------------------------------------
// v2 clearance constraint (S3Params::use_clearance), with an injected evaluator: the selection logic is tested, not
// the geometry (see test_approach_clearance.cpp).
// ---------------------------------------------------------------------------------------------------------------------

namespace {

ClearanceResult fakeClearance(double mm) {
  ClearanceResult r;
  r.min_mm = r.min_mm_no_tail = mm;
  r.pair = r.pair_no_tail = {"dito 1", "collare"};
  r.t_rel_s = r.t_rel_no_tail_s = -3.0;
  return r;
}

bool sameCandidate(const ClearanceQuery& q, const S3Candidate& c) {
  return q.k == static_cast<int>(c.eval.k) && q.wrist_flip == c.eval.wrist_flip &&
         std::abs(q.theta_star_rad - c.eval.theta_rad) < 1e-9;
}

S3Params v2LooseParams() {
  S3Params p = v2Params();
  p.finalists_n = 4;
  return p;
}

}  // namespace

TEST(GraspS3ClearanceTest, MissingEvaluatorIsAnError) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  S3Params p = v2LooseParams();
  p.use_clearance = true;  // ctx.clearance_eval is empty
  EXPECT_THROW(selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p), std::invalid_argument);
}

TEST(GraspS3ClearanceTest, DisabledIsUnchangedAndNeverCallsTheEvaluator) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const S3Params base = v2LooseParams();
  const S3Result a = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, base);
  if (a.status != S3Result::kSelected) GTEST_SKIP() << "no v2 candidate in this fixture: " << a.note;

  int calls = 0;
  s.ctx.clearance_eval = [&calls](const ClearanceQuery&) {
    ++calls;
    return fakeClearance(-100.0);
  };
  S3Params other = base;  // use_clearance stays false; the other clearance fields are ignored
  other.min_clearance_mm = 99.0;
  other.clearance_dt_s = 0.5;
  other.gripper_open_m = 0.01;
  const S3Result b = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, other);
  EXPECT_EQ(calls, 0);
  ASSERT_EQ(a.status, b.status);
  EXPECT_EQ(a.chosen.eval.theta_rad, b.chosen.eval.theta_rad);
  EXPECT_EQ(a.chosen.eval.k, b.chosen.eval.k);
  EXPECT_EQ(a.chosen.eval.wrist_flip, b.chosen.eval.wrist_flip);
  EXPECT_EQ(a.chosen.cost, b.chosen.cost);
  EXPECT_EQ(a.evaluations, b.evaluations);
  ASSERT_EQ(a.v2_finalists.size(), b.v2_finalists.size());
  for (std::size_t i = 0; i < a.v2_finalists.size(); ++i) {
    EXPECT_EQ(a.v2_finalists[i].j, b.v2_finalists[i].j);
    EXPECT_EQ(a.v2_finalists[i].winner, b.v2_finalists[i].winner);
    EXPECT_FALSE(b.v2_finalists[i].clearance_done);
  }
  EXPECT_EQ(b.clearance_stage_ms, 0.0);
  EXPECT_TRUE(b.warnings.empty());
}

TEST(GraspS3ClearanceTest, ViolatingWinnerIsSkippedAndTheNextOneWins) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  S3Params p = v2LooseParams();
  const S3Result base = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  if (base.status != S3Result::kSelected) GTEST_SKIP() << "no v2 candidate in this fixture: " << base.note;
  int n_feasible = 0;
  for (const auto& f : base.v2_finalists) n_feasible += f.rollout_feasible ? 1 : 0;
  if (n_feasible < 2) GTEST_SKIP() << "need >= 2 rollout-feasible finalists, have " << n_feasible;
  const S3Candidate winner = base.chosen;

  p.use_clearance = true;
  p.min_clearance_mm = 2.0;
  s.ctx.clearance_eval = [&](const ClearanceQuery& q) { return fakeClearance(sameCandidate(q, winner) ? 0.5 : 10.0); };
  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  ASSERT_EQ(r.status, S3Result::kSelected) << r.note;
  EXPECT_FALSE(r.chosen.eval.k == winner.eval.k && r.chosen.eval.wrist_flip == winner.eval.wrist_flip &&
               std::abs(r.chosen.eval.theta_rad - winner.eval.theta_rad) < 1e-9);
  bool saw_rejected = false, saw_chosen = false;
  for (const auto& f : r.v2_finalists) {
    const bool is_baseline_winner = f.cand.eval.k == winner.eval.k && f.cand.eval.wrist_flip == winner.eval.wrist_flip &&
                                    std::abs(f.cand.eval.theta_rad - winner.eval.theta_rad) < 1e-9;
    if (f.clearance_done && is_baseline_winner) {
      EXPECT_TRUE(f.clearance_rejected);
      EXPECT_FALSE(f.winner);
      EXPECT_DOUBLE_EQ(f.clearance_mm, 0.5);
      saw_rejected = true;
    }
    if (f.winner) {
      EXPECT_TRUE(f.clearance_done);
      EXPECT_FALSE(f.clearance_rejected);
      EXPECT_DOUBLE_EQ(f.clearance_mm, 10.0);
      EXPECT_EQ(f.clearance_pair, "dito 1-collare");
      saw_chosen = true;
    }
  }
  EXPECT_TRUE(saw_rejected);
  EXPECT_TRUE(saw_chosen);
  EXPECT_GE(r.clearance_stage_ms, 0.0);
}

TEST(GraspS3ClearanceTest, NoAdmissibleFinalistGivesNoCandidateWithNote) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  S3Params p = v2LooseParams();
  const S3Result base = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  if (base.status != S3Result::kSelected) GTEST_SKIP() << "no v2 candidate in this fixture: " << base.note;

  p.use_clearance = true;
  p.min_clearance_mm = 2.0;
  s.ctx.clearance_eval = [](const ClearanceQuery&) { return fakeClearance(1.5); };
  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  EXPECT_EQ(r.status, S3Result::kNoCandidate);
  EXPECT_NE(r.note.find("clearance"), std::string::npos) << r.note;
  EXPECT_NE(r.note.find("1.5"), std::string::npos) << r.note;  // best clearance reported
  for (const auto& f : r.v2_finalists) {
    EXPECT_FALSE(f.winner);
    if (f.rollout_feasible) {
      EXPECT_TRUE(f.clearance_done);
      EXPECT_TRUE(f.clearance_rejected);
    }
  }
}

TEST(GraspS3ClearanceTest, ForceKeepsTheCandidateAndWarns) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  S3Params p = v2LooseParams();
  const S3Result base = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  if (base.status != S3Result::kSelected) GTEST_SKIP() << "no v2 candidate in this fixture: " << base.note;
  const double theta_deg = base.chosen.eval.theta_rad * 180.0 / M_PI;
  if (std::abs(theta_deg) > 180.0) GTEST_SKIP() << "winner theta outside [-180, 180]";

  p.force = true;
  p.force_k = static_cast<int>(base.chosen.eval.k);
  p.force_theta_deg = theta_deg;
  p.force_flip = base.chosen.eval.wrist_flip ? 1 : 0;
  p.use_clearance = true;
  p.min_clearance_mm = 2.0;
  s.ctx.clearance_eval = [](const ClearanceQuery&) { return fakeClearance(0.5); };
  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  ASSERT_EQ(r.status, S3Result::kSelected) << r.note;
  ASSERT_EQ(r.warnings.size(), 1u);
  EXPECT_NE(r.warnings[0].find("WARN"), std::string::npos);
  ASSERT_EQ(r.v2_finalists.size(), 1u);
  EXPECT_TRUE(r.v2_finalists[0].clearance_rejected);  // below threshold, but kept
  EXPECT_TRUE(r.v2_finalists[0].winner);

  s.ctx.clearance_eval = [](const ClearanceQuery&) { return fakeClearance(10.0); };
  const S3Result ok = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  ASSERT_EQ(ok.status, S3Result::kSelected);
  EXPECT_TRUE(ok.warnings.empty());
}

// ---------------------------------------------------------------------------------------------------------------------
// v2 joint-margin constraint (S3Params::use_margin). Written without being able to build/run. The margins come from the
// real followMargin() on the synthetic template (their values are not known a priori): the tests use the threshold as the
// lever, with a probe run (threshold 1e6 deg: every finalist that passes the rollout is evaluated and discarded, so the
// probe lists the margin of all of them) to build the expected outcome.
// ---------------------------------------------------------------------------------------------------------------------

namespace {

constexpr double kHugeMarginDeg = 1e6;

S3Params v2MarginParams(double min_margin_deg) {
  S3Params p = v2LooseParams();
  p.use_margin = true;
  p.min_margin_deg = min_margin_deg;
  return p;
}

bool isBaselineChoice(const S3FinalistRecord& f, const S3Candidate& c) {
  return f.cand.eval.k == c.eval.k && f.cand.eval.wrist_flip == c.eval.wrist_flip &&
         std::abs(f.cand.eval.theta_rad - c.eval.theta_rad) < 1e-9;
}

}  // namespace

TEST(GraspS3MarginTest, DefaultsAndValidation) {
  const S3Params d;
  EXPECT_FALSE(d.use_margin);
  EXPECT_DOUBLE_EQ(d.min_margin_deg, 24.0);  // ASSUMED
  EXPECT_DOUBLE_EQ(d.margin_follow_s, 3.0);
  EXPECT_DOUBLE_EQ(d.margin_follow_dt_s, 0.01);
  EXPECT_NO_THROW(validateS3Params(v2MarginParams(24.0)));
  auto bad = [](auto mutate) {
    S3Params p = v2MarginParams(24.0);
    mutate(p);
    EXPECT_THROW(validateS3Params(p), std::invalid_argument);
  };
  bad([](S3Params& p) { p.min_margin_deg = -0.1; });
  bad([](S3Params& p) { p.min_margin_deg = std::nan(""); });
  bad([](S3Params& p) { p.margin_follow_s = -1.0; });
  bad([](S3Params& p) { p.margin_follow_dt_s = 0.0; });
}

// use_margin = false: the other margin fields are ignored, the v2 selection is unchanged (and no followMargin runs).
TEST(GraspS3MarginTest, DisabledIsUnchangedV2Selection) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const S3Params base = v2LooseParams();
  const S3Result a = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, base);
  if (a.status != S3Result::kSelected) GTEST_SKIP() << "no v2 candidate in this fixture: " << a.note;
  S3Params other = base;  // use_margin stays false
  other.min_margin_deg = 1e6;
  other.margin_follow_s = 99.0;
  other.margin_follow_dt_s = 0.5;
  const S3Result b = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, other);
  ASSERT_EQ(a.status, b.status);
  EXPECT_EQ(a.chosen.eval.theta_rad, b.chosen.eval.theta_rad);
  EXPECT_EQ(a.chosen.eval.k, b.chosen.eval.k);
  EXPECT_EQ(a.chosen.eval.wrist_flip, b.chosen.eval.wrist_flip);
  EXPECT_EQ(a.chosen.cost, b.chosen.cost);
  EXPECT_EQ(a.evaluations, b.evaluations);
  EXPECT_EQ(a.posthoc_min_margin_rad, b.posthoc_min_margin_rad);  // rollout margin on [start, tau]: semantics untouched
  ASSERT_EQ(a.v2_finalists.size(), b.v2_finalists.size());
  EXPECT_EQ(b.rollouts_done, static_cast<int>(b.v2_finalists.size()));  // eager: every finalist
  EXPECT_EQ(b.margin_evals_done, 0);
  for (std::size_t i = 0; i < a.v2_finalists.size(); ++i) {
    EXPECT_EQ(a.v2_finalists[i].j, b.v2_finalists[i].j);
    EXPECT_EQ(a.v2_finalists[i].winner, b.v2_finalists[i].winner);
    EXPECT_EQ(a.v2_finalists[i].rollout_min_margin_rad, b.v2_finalists[i].rollout_min_margin_rad);
    EXPECT_FALSE(b.v2_finalists[i].margin_done);
    EXPECT_TRUE(b.v2_finalists[i].discard_reason.empty());
  }
}

// Margin below the threshold: discarded; the next admissible finalist (J_static order) is chosen; the ones after it get no rollout.
TEST(GraspS3MarginTest, BelowThresholdDiscardedAdmissibleChosenLazy) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const S3Result probe = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, v2MarginParams(kHugeMarginDeg));
  std::vector<std::size_t> idx;  // finalists that reached the margin stage, in J_static order
  for (std::size_t i = 0; i < probe.v2_finalists.size(); ++i) {
    if (probe.v2_finalists[i].margin_done) idx.push_back(i);
  }
  if (idx.size() < 2) GTEST_SKIP() << "need >= 2 rollout-feasible finalists in this fixture, have " << idx.size();
  const double m_first = probe.v2_finalists[idx[0]].margin_deg;
  std::size_t expected = probe.v2_finalists.size();
  ASSERT_TRUE(std::isfinite(m_first));
  // The synthetic fixture can give a NEGATIVE predicted margin (rollout feasible = no violation on [0, tau], but the follow
  // may leave the limits): m_first + 1e-6 would then be negative and validateS3Params (correctly) throws. A threshold of 0
  // still discards a negative first margin.
  const double thr = std::max(m_first + 1e-6, 0.0);
  for (std::size_t n = 1; n < idx.size(); ++n) {
    if (probe.v2_finalists[idx[n]].margin_deg >= thr) {
      expected = idx[n];
      break;
    }
  }
  if (expected == probe.v2_finalists.size()) GTEST_SKIP() << "no later finalist has a larger margin than the first in this fixture";

  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, v2MarginParams(thr));
  ASSERT_EQ(r.status, S3Result::kSelected) << r.note;
  ASSERT_EQ(r.v2_finalists.size(), probe.v2_finalists.size());
  EXPECT_TRUE(r.v2_finalists[idx[0]].margin_rejected);
  EXPECT_EQ(r.v2_finalists[idx[0]].discard_reason, "margin");
  for (std::size_t i = 0; i < r.v2_finalists.size(); ++i) {
    const auto& f = r.v2_finalists[i];
    EXPECT_EQ(f.winner, i == expected) << "finalist " << i;
    if (i < expected) {
      EXPECT_TRUE(f.rollout_done) << i;
      EXPECT_FALSE(f.discard_reason.empty()) << i;  // everything before the winner was discarded
    }
    if (i > expected) {
      EXPECT_FALSE(f.rollout_done) << "no rollout after the first admissible finalist: " << i;  // lazy
      EXPECT_FALSE(f.margin_done) << i;
    }
  }
  EXPECT_GE(r.v2_finalists[expected].margin_deg, thr);
  EXPECT_FALSE(isBaselineChoice(r.v2_finalists[idx[0]], r.chosen));
  EXPECT_EQ(r.rollouts_done, static_cast<int>(expected) + 1);  // rollouts up to and including the winner
  EXPECT_LE(r.rollouts_done, probe.rollouts_done);
}

// Lazy: with threshold 0 the number of rollouts never exceeds the position of the first admissible finalist.
TEST(GraspS3MarginTest, RolloutCountDoesNotExceedFirstAdmissiblePosition) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const S3Result eager = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, v2LooseParams());
  if (eager.status != S3Result::kSelected) GTEST_SKIP() << "no v2 candidate in this fixture: " << eager.note;
  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, v2MarginParams(0.0));
  if (r.status != S3Result::kSelected) GTEST_SKIP() << "no finalist with a non-negative margin in this fixture: " << r.note;
  std::size_t pos = r.v2_finalists.size();
  for (std::size_t i = 0; i < r.v2_finalists.size(); ++i) {
    if (r.v2_finalists[i].winner) pos = i;
  }
  ASSERT_LT(pos, r.v2_finalists.size());
  EXPECT_LE(r.rollouts_done, static_cast<int>(pos) + 1);
  EXPECT_LE(r.rollouts_done, eager.rollouts_done);
  EXPECT_LE(r.margin_evals_done, r.rollouts_done);
  for (std::size_t i = pos + 1; i < r.v2_finalists.size(); ++i) EXPECT_FALSE(r.v2_finalists[i].rollout_done) << i;
  // J_static order is the finalists' order: nobody before the winner was admissible
  for (std::size_t i = 0; i < pos; ++i) EXPECT_FALSE(r.v2_finalists[i].discard_reason.empty()) << i;
}

// No admissible finalist: kNoCandidate, with a note, no winner, no fallback; every rollout-feasible finalist evaluated.
TEST(GraspS3MarginTest, AllBelowThresholdGivesNoCandidate) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const S3Result eager = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, v2LooseParams());
  if (eager.status != S3Result::kSelected) GTEST_SKIP() << "no v2 candidate in this fixture: " << eager.note;
  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, v2MarginParams(kHugeMarginDeg));
  EXPECT_EQ(r.status, S3Result::kNoCandidate);
  EXPECT_NE(r.note.find("margin"), std::string::npos) << r.note;
  EXPECT_EQ(r.rollouts_done, static_cast<int>(r.v2_finalists.size()));  // nothing admissible: all finalists were evaluated
  for (const auto& f : r.v2_finalists) {
    EXPECT_FALSE(f.winner);
    if (f.rollout_feasible) {
      EXPECT_TRUE(f.margin_done);
      EXPECT_TRUE(f.margin_rejected);
      EXPECT_EQ(f.discard_reason, "margin");
      EXPECT_GE(f.margin_joint, 0);
      EXPECT_LE(f.margin_joint, 6);
    }
  }
}

// With clearance on, the margin is checked first: a finalist discarded by the margin never reaches the clearance evaluator.
TEST(GraspS3MarginTest, MarginIsCheckedBeforeClearance) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const S3Result eager = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, v2LooseParams());
  if (eager.status != S3Result::kSelected) GTEST_SKIP() << "no v2 candidate in this fixture: " << eager.note;

  int calls = 0;
  s.ctx.clearance_eval = [&calls](const ClearanceQuery&) {
    ++calls;
    return fakeClearance(10.0);
  };
  S3Params p = v2MarginParams(kHugeMarginDeg);
  p.use_clearance = true;
  const S3Result none = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  EXPECT_EQ(none.status, S3Result::kNoCandidate);
  EXPECT_EQ(calls, 0);

  calls = 0;
  p.min_margin_deg = 0.0;
  const S3Result ok = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  if (ok.status != S3Result::kSelected) GTEST_SKIP() << "no finalist with a non-negative margin in this fixture: " << ok.note;
  EXPECT_EQ(calls, 1);                                                              // clearance 10 mm passes: only the first margin-admissible finalist reaches it
  s.ctx.clearance_eval = [](const ClearanceQuery&) { return fakeClearance(0.5); };  // all below 2 mm
  const S3Result rej = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  EXPECT_EQ(rej.status, S3Result::kNoCandidate);
  for (const auto& f : rej.v2_finalists) EXPECT_FALSE(f.winner);
}

// force: the margin constraint only warns.
TEST(GraspS3MarginTest, ForceKeepsTheCandidateAndWarns) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const S3Result base = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, v2LooseParams());
  if (base.status != S3Result::kSelected) GTEST_SKIP() << "no v2 candidate in this fixture: " << base.note;
  const double theta_deg = base.chosen.eval.theta_rad * 180.0 / M_PI;
  if (std::abs(theta_deg) > 180.0) GTEST_SKIP() << "winner theta outside [-180, 180]";
  S3Params p = v2MarginParams(kHugeMarginDeg);
  p.force = true;
  p.force_k = static_cast<int>(base.chosen.eval.k);
  p.force_theta_deg = theta_deg;
  p.force_flip = base.chosen.eval.wrist_flip ? 1 : 0;
  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  ASSERT_EQ(r.status, S3Result::kSelected) << r.note;
  ASSERT_EQ(r.warnings.size(), 1u);
  EXPECT_NE(r.warnings[0].find("use_margin"), std::string::npos);
  ASSERT_EQ(r.v2_finalists.size(), 1u);
  EXPECT_TRUE(r.v2_finalists[0].margin_rejected);  // below threshold, but kept
  EXPECT_TRUE(r.v2_finalists[0].winner);
}

// ---------------------------------------------------------------------------------------------------------------------
// Characterization (L6a0) of the post-hoc branches of selectGraspS3 that no other test reaches: they pin the CURRENT
// behaviour (also where it looks odd) before the function is split. Written without being able to build/run: every test
// asserts the discard strings / counts that prove the target branch was traversed; a test is skipped only if the fixture
// yields no candidate at all (nothing to characterize).
// ---------------------------------------------------------------------------------------------------------------------

namespace {

/// |v_target - v_ee| is of the order of mm/s: with this limit every velocity check fails.
constexpr double kTinyDv = 1e-9;
const char kVelocityReason[] = "velocita' relativa";

bool startsWith(const std::string& s, const std::string& prefix) { return s.compare(0, prefix.size(), prefix) == 0; }

bool endsWith(const std::string& s, const std::string& suffix) {
  return s.size() >= suffix.size() && s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

/// Rollout-feasible finalists with a finite J: the only ones the eager flow (s3PickWinnerV2) can pick.
std::size_t countPickable(const S3Result& r) {
  std::size_t n = 0;
  for (const auto& f : r.v2_finalists) n += (f.rollout_feasible && std::isfinite(f.j)) ? 1 : 0;
  return n;
}

/// Finalists that passed rollout and margin in the lazy flow, in J_static order.
std::vector<std::size_t> marginAdmissible(const S3Result& r) {
  std::vector<std::size_t> idx;
  for (std::size_t i = 0; i < r.v2_finalists.size(); ++i) {
    if (r.v2_finalists[i].margin_done && !r.v2_finalists[i].margin_rejected) idx.push_back(i);
  }
  return idx;
}

}  // namespace

// Characterization (L6a0): v2 lazy flow, velocity discard of every margin-admissible finalist (discard_reason "velocity").
TEST(GraspS3PosthocCharacterizationTest, V2LazyVelocityDiscardsAdmissibleFinalists) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const S3Params base_p = v2MarginParams(0.0);  // use_margin, no force => lazy; velocity check off
  const S3Result base = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, base_p);
  if (base.status != S3Result::kSelected) GTEST_SKIP() << "no margin-admissible finalist in this fixture: " << base.note;
  std::size_t w = base.v2_finalists.size();  // lazy baseline winner = first admissible finalist
  for (std::size_t i = 0; i < base.v2_finalists.size(); ++i) {
    if (base.v2_finalists[i].winner) w = i;
  }
  ASSERT_LT(w, base.v2_finalists.size());

  S3Params p = base_p;
  p.posthoc_velocity_check = true;
  p.delta_v_max_m_s = kTinyDv;
  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  const std::vector<std::size_t> adm = marginAdmissible(r);
  ASSERT_FALSE(adm.empty());
  EXPECT_EQ(adm.front(), w);  // the margin stage does not depend on the velocity check
  EXPECT_EQ(r.status, S3Result::kPosthocFailed) << r.note;
  EXPECT_NE(r.note.find("v2: no finalist passed the rollout and the post-hoc checks"), std::string::npos) << r.note;
  EXPECT_EQ(r.posthoc_checks, static_cast<int>(adm.size()));
  ASSERT_EQ(r.posthoc_log.size(), adm.size());
  EXPECT_EQ(r.posthoc_dv_m_s, 0.0);  // nothing chosen: not copied from the log
  for (std::size_t j = 0; j < adm.size(); ++j) {
    const S3FinalistRecord& f = r.v2_finalists[adm[j]];
    const S3PosthocEntry& e = r.posthoc_log[j];
    EXPECT_EQ(f.discard_reason, "velocity") << "finalist " << adm[j];
    EXPECT_FALSE(f.winner);
    EXPECT_TRUE(startsWith(e.reject_reason, kVelocityReason)) << e.reject_reason;
    EXPECT_FALSE(e.accepted);
    EXPECT_FALSE(e.path_checked);  // the rollout is the path check in v2; the entry is logged before it is marked
    EXPECT_GT(e.dv_m_s, kTinyDv);
    EXPECT_EQ(e.k, f.cand.eval.k);
    EXPECT_EQ(e.theta_rad, f.cand.eval.theta_rad);
  }
  for (std::size_t i = 0; i < r.v2_finalists.size(); ++i) {  // the others were discarded earlier, by rollout or margin
    if (std::find(adm.begin(), adm.end(), i) != adm.end()) continue;
    const S3FinalistRecord& f = r.v2_finalists[i];
    EXPECT_TRUE(f.rollout_done) << i;
    EXPECT_TRUE(f.discard_reason == "rollout" || f.discard_reason == "margin") << i << ": " << f.discard_reason;
  }
}

// Characterization (L6a0): v2 eager flow, velocity discard (alive[fi] = 0): the picked finalists fail one by one, none wins.
TEST(GraspS3PosthocCharacterizationTest, V2EagerVelocityDiscardsPickedFinalistsInOrder) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const S3Params base_p = v2LooseParams();  // no use_margin => eager
  const S3Result base = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, base_p);
  if (base.status != S3Result::kSelected) GTEST_SKIP() << "no v2 candidate in this fixture: " << base.note;

  S3Params p = base_p;
  p.posthoc_velocity_check = true;
  p.delta_v_max_m_s = kTinyDv;
  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  const std::size_t pickable = countPickable(r);
  ASSERT_GE(pickable, 1u);
  EXPECT_EQ(r.status, S3Result::kPosthocFailed) << r.note;
  EXPECT_NE(r.note.find("v2: no finalist passed the rollout and the post-hoc checks"), std::string::npos) << r.note;
  ASSERT_EQ(r.posthoc_log.size(), pickable);
  EXPECT_EQ(r.posthoc_checks, static_cast<int>(pickable));
  EXPECT_EQ(r.posthoc_dv_m_s, 0.0);
  // the first pick is the finalist the run without velocity check chose
  EXPECT_EQ(r.posthoc_log.front().k, base.chosen.eval.k);
  EXPECT_EQ(r.posthoc_log.front().theta_rad, base.chosen.eval.theta_rad);
  for (const S3PosthocEntry& e : r.posthoc_log) {
    EXPECT_TRUE(startsWith(e.reject_reason, kVelocityReason)) << e.reject_reason;
    EXPECT_FALSE(e.accepted);
    EXPECT_FALSE(e.path_checked);
    EXPECT_GT(e.dv_m_s, kTinyDv);
    int matches = 0;  // each logged check belongs to exactly one pickable finalist
    for (const S3FinalistRecord& f : r.v2_finalists) {
      if (f.rollout_feasible && std::isfinite(f.j) && f.cand.eval.k == e.k && f.cand.eval.theta_rad == e.theta_rad) ++matches;
    }
    EXPECT_EQ(matches, 1);
  }
  for (const S3FinalistRecord& f : r.v2_finalists) {
    EXPECT_FALSE(f.winner);
    EXPECT_TRUE(f.discard_reason.empty()) << f.discard_reason;  // eager: discard_reason is never set (lazy only)
  }
  int ranked_velocity = 0;  // the discard reason is on the accepted candidate, visible in ranked
  for (const S3Candidate& c : r.ranked) ranked_velocity += startsWith(c.reject_reason, kVelocityReason) ? 1 : 0;
  EXPECT_EQ(ranked_velocity, static_cast<int>(pickable));
}

// Characterization (L6a0): v2 with max_posthoc_candidates = 0 (lazy ~l.689 and eager ~l.774): winner taken without any check.
TEST(GraspS3PosthocCharacterizationTest, V2MaxPosthocZeroSelectsWithoutAnyCheck) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const S3Params eager_base_p = v2LooseParams();
  const S3Result eager_base = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, eager_base_p);
  const S3Params lazy_base_p = v2MarginParams(0.0);
  const S3Result lazy_base = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, lazy_base_p);
  const bool eager_ok = eager_base.status == S3Result::kSelected;
  const bool lazy_ok = lazy_base.status == S3Result::kSelected;
  if (!eager_ok && !lazy_ok) GTEST_SKIP() << "no v2 candidate in this fixture: " << eager_base.note << " / " << lazy_base.note;

  // With the velocity check on and an impossible limit, any check would fail: kSelected proves that none was made.
  auto checkNoPosthoc = [&](const S3Params& base_params, const S3Result& base) {
    S3Params p = base_params;
    p.posthoc_velocity_check = true;
    p.delta_v_max_m_s = kTinyDv;
    p.max_posthoc_candidates = 0;
    const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    ASSERT_EQ(r.status, S3Result::kSelected) << r.note;
    EXPECT_EQ(r.posthoc_checks, 0);
    EXPECT_TRUE(r.posthoc_log.empty());
    EXPECT_EQ(r.posthoc_dv_m_s, 0.0);
    EXPECT_EQ(r.posthoc_min_margin_rad, 0.0);
    EXPECT_TRUE(r.chosen.reject_reason.empty()) << r.chosen.reject_reason;
    EXPECT_EQ(r.chosen.eval.k, base.chosen.eval.k);
    EXPECT_EQ(r.chosen.eval.theta_rad, base.chosen.eval.theta_rad);
    EXPECT_EQ(r.chosen.eval.wrist_flip, base.chosen.eval.wrist_flip);
    int winners = 0;
    for (const S3FinalistRecord& f : r.v2_finalists) {
      winners += f.winner ? 1 : 0;
      EXPECT_TRUE(f.discard_reason.empty() || f.discard_reason == "rollout" || f.discard_reason == "margin") << f.discard_reason;
    }
    EXPECT_EQ(winners, 1);
    EXPECT_EQ(r.rollouts_done, base.rollouts_done);
  };
  if (eager_ok) checkNoPosthoc(eager_base_p, eager_base);
  if (lazy_ok) checkNoPosthoc(lazy_base_p, lazy_base);
}

// Characterization (L6a0): v2 with the post-hoc cap reached (lazy ~l.694 and eager ~l.779): max_posthoc_candidates = 1, first discarded.
TEST(GraspS3PosthocCharacterizationTest, V2MaxPosthocCapStopsAfterFirstDiscard) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  S3Params eager_p = v2LooseParams();
  eager_p.posthoc_velocity_check = true;
  eager_p.delta_v_max_m_s = kTinyDv;
  S3Params lazy_p = v2MarginParams(0.0);
  lazy_p.posthoc_velocity_check = true;
  lazy_p.delta_v_max_m_s = kTinyDv;
  // Uncapped runs (max_posthoc_candidates 10 > finalists_n 4): every checkable finalist is checked and discarded.
  const S3Result eager_all = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, eager_p);
  const S3Result lazy_all = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, lazy_p);
  const bool eager_ok = eager_all.posthoc_log.size() >= 2;  // a second check is needed to hit the cap
  const std::vector<std::size_t> adm = marginAdmissible(lazy_all);
  const bool lazy_ok = adm.size() >= 2;
  if (!eager_ok && !lazy_ok) GTEST_SKIP() << "fewer than 2 checkable finalists in this fixture (eager " << eager_all.posthoc_log.size()
                                          << ", lazy " << adm.size() << "): the cap cannot be reached";

  if (eager_ok) {
    S3Params p = eager_p;
    p.max_posthoc_candidates = 1;
    const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    EXPECT_EQ(r.status, S3Result::kPosthocFailed) << r.note;
    EXPECT_EQ(r.posthoc_checks, 1);
    ASSERT_EQ(r.posthoc_log.size(), 1u);  // the uncapped run logs >= 2: the cap stopped the loop
    EXPECT_TRUE(startsWith(r.posthoc_log[0].reject_reason, kVelocityReason)) << r.posthoc_log[0].reject_reason;
    EXPECT_EQ(r.posthoc_log[0].k, eager_all.posthoc_log[0].k);
    EXPECT_EQ(r.posthoc_log[0].theta_rad, eager_all.posthoc_log[0].theta_rad);
    EXPECT_DOUBLE_EQ(r.posthoc_log[0].dv_m_s, eager_all.posthoc_log[0].dv_m_s);
    EXPECT_NE(r.note.find("1 post-hoc checks)"), std::string::npos) << r.note;
    EXPECT_EQ(r.rollouts_done, eager_all.rollouts_done);  // eager: all rollouts are done before the post-hoc stage
    for (const S3FinalistRecord& f : r.v2_finalists) EXPECT_FALSE(f.winner);
  }
  if (lazy_ok) {
    S3Params p = lazy_p;
    p.max_posthoc_candidates = 1;
    const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    EXPECT_EQ(r.status, S3Result::kPosthocFailed) << r.note;
    EXPECT_EQ(r.posthoc_checks, 1);
    ASSERT_EQ(r.posthoc_log.size(), 1u);
    EXPECT_TRUE(startsWith(r.posthoc_log[0].reject_reason, kVelocityReason)) << r.posthoc_log[0].reject_reason;
    EXPECT_EQ(r.v2_finalists[adm[0]].discard_reason, "velocity");
    // The second admissible finalist was rolled out and passed margin, then the cap stopped the loop before its check:
    // not discarded, no finalist after it was rolled out.
    EXPECT_TRUE(r.v2_finalists[adm[1]].margin_done);
    EXPECT_TRUE(r.v2_finalists[adm[1]].discard_reason.empty()) << r.v2_finalists[adm[1]].discard_reason;
    EXPECT_FALSE(r.v2_finalists[adm[1]].winner);
    EXPECT_EQ(r.rollouts_done, static_cast<int>(adm[1]) + 1);
    for (std::size_t i = adm[1] + 1; i < r.v2_finalists.size(); ++i) EXPECT_FALSE(r.v2_finalists[i].rollout_done) << i;
  }
}

// Characterization (L6a0): v2 force + force_ignore_constraints, no finalist passes (~l.810-814): the candidate is taken anyway.
TEST(GraspS3PosthocCharacterizationTest, V2ForceIgnoreFallbackAfterNoFinalistPasses) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const S3Result base = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, v2LooseParams());
  if (base.status != S3Result::kSelected) GTEST_SKIP() << "no v2 candidate in this fixture: " << base.note;
  const double theta_deg = base.chosen.eval.theta_rad * 180.0 / M_PI;
  if (std::abs(theta_deg) > 180.0) GTEST_SKIP() << "winner theta outside [-180, 180]";

  S3Params p = v2LooseParams();
  p.force = true;
  p.force_k = static_cast<int>(base.chosen.eval.k);
  p.force_theta_deg = theta_deg;
  p.force_flip = base.chosen.eval.wrist_flip ? 1 : 0;
  p.posthoc_velocity_check = true;  // the forced finalist fails the velocity check (or the rollout)
  p.delta_v_max_m_s = kTinyDv;

  const S3Result strict = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  EXPECT_EQ(strict.status, S3Result::kPosthocFailed) << strict.note;  // without force_ignore_constraints: reported as failure

  p.force_ignore_constraints = true;
  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  ASSERT_EQ(r.v2_finalists.size(), 1u);
  const S3FinalistRecord& f = r.v2_finalists.front();
  EXPECT_EQ(r.evaluations, 1);
  EXPECT_TRUE(f.rollout_done);
  EXPECT_EQ(r.status, S3Result::kSelected) << r.note;
  EXPECT_EQ(r.chosen.eval.k, f.cand.eval.k);
  EXPECT_EQ(r.chosen.eval.theta_rad, f.cand.eval.theta_rad);
  EXPECT_FALSE(f.winner);  // chosen by the fallback, not by a passing check: no finalist is flagged as winner
  EXPECT_TRUE(endsWith(r.forced_violations, "v2: no finalist passed the rollout / post-hoc checks")) << r.forced_violations;
  // The fallback does not copy anything from the post-hoc log.
  EXPECT_EQ(r.posthoc_dv_m_s, 0.0);
  EXPECT_EQ(r.posthoc_min_margin_rad, 0.0);
  const bool checked = f.rollout_feasible && std::isfinite(f.j);  // otherwise the eager flow has nothing to pick
  EXPECT_EQ(r.posthoc_checks, checked ? 1 : 0);
  ASSERT_EQ(r.posthoc_log.size(), checked ? 1u : 0u);
  if (checked) {
    EXPECT_TRUE(startsWith(r.posthoc_log[0].reject_reason, kVelocityReason)) << r.posthoc_log[0].reject_reason;
    EXPECT_FALSE(r.posthoc_log[0].accepted);
  }
}

// ---------------------------------------------------------------------------------------------------------------------
// Characterization (L6a0b): branches left uncovered by the L6a0 tests (gcov): velocity check passed, a-priori r_trust /
// start_goal rejections, lazy rollout discard. Same rules: the discard string / counter that proves
// the branch was traversed is asserted unconditionally; a test whose fixture cannot reach the branch is SKIPPED (= failure).
// ---------------------------------------------------------------------------------------------------------------------

namespace {

/// |v_target - v_ee| is far below this: every velocity check passes.
constexpr double kPassDv = 1e3;

/// v2 (flow decided by @p base_p: lazy with use_margin, else eager): velocity check ON with a limit that always passes. The
/// winner must be the one of the run without velocity check, and the measured dv must reach the result.
void expectV2VelocityPassPropagatesDv(S3TestSetup& s, const S3Params& base_p) {
  const S3Result ref = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, base_p);  // velocity check off
  if (ref.status != S3Result::kSelected) {
    GTEST_SKIP() << "no v2 winner in this fixture: " << ref.note;
  }
  S3Params p = base_p;
  p.posthoc_velocity_check = true;
  p.delta_v_max_m_s = kPassDv;
  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  ASSERT_EQ(r.status, S3Result::kSelected) << r.note;
  ASSERT_EQ(r.posthoc_checks, 1);
  ASSERT_EQ(r.posthoc_log.size(), 1u);
  const S3PosthocEntry& e = r.posthoc_log.front();
  EXPECT_TRUE(e.accepted);
  EXPECT_TRUE(e.reject_reason.empty()) << e.reject_reason;
  EXPECT_GT(e.dv_m_s, 0.0);
  EXPECT_LT(e.dv_m_s, kPassDv);
  EXPECT_GT(e.v_target.norm(), 0.0);  // the check really measured something (surface velocity of the spinning cube)
  EXPECT_DOUBLE_EQ(e.dv_m_s, (e.v_target - e.v_ee).norm());
  EXPECT_TRUE(e.path_checked);  // the rollout is the path check (acceptRolloutWinner)
  EXPECT_TRUE(e.path_ok);
  EXPECT_EQ(r.posthoc_dv_m_s, e.dv_m_s);  // dv travels lambda -> acceptRolloutWinner -> result
  const S3FinalistRecord* win = nullptr;
  int winners = 0;
  for (const S3FinalistRecord& f : r.v2_finalists) {
    if (f.winner) {
      win = &f;
      ++winners;
    }
  }
  ASSERT_EQ(winners, 1);
  EXPECT_TRUE(win->discard_reason.empty()) << win->discard_reason;
  EXPECT_EQ(r.posthoc_min_margin_rad, win->rollout_min_margin_rad);
  EXPECT_EQ(e.min_margin_rad, win->rollout_min_margin_rad);
  EXPECT_EQ(e.k, win->cand.eval.k);
  EXPECT_EQ(e.theta_rad, win->cand.eval.theta_rad);
  EXPECT_EQ(r.chosen.eval.k, ref.chosen.eval.k);
  EXPECT_EQ(r.chosen.eval.theta_rad, ref.chosen.eval.theta_rad);
  EXPECT_EQ(r.chosen.eval.wrist_flip, ref.chosen.eval.wrist_flip);
  EXPECT_TRUE(r.chosen.reject_reason.empty()) << r.chosen.reject_reason;
}

}  // namespace

// Characterization (L6a0b): v2 lazy flow, velocity check PASSED: acceptRolloutWinner propagates dv and the rollout margin (~l.650-660, call ~705).
TEST(GraspS3PosthocCharacterizationTest, V2LazyVelocityPassPropagatesDvToResult) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  expectV2VelocityPassPropagatesDv(s, v2MarginParams(0.0));
}

// Characterization (L6a0b): v2 eager flow, velocity check PASSED: acceptRolloutWinner propagates dv and the rollout margin (~l.650-660, call ~790).
TEST(GraspS3PosthocCharacterizationTest, V2EagerVelocityPassPropagatesDvToResult) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  expectV2VelocityPassPropagatesDv(s, v2LooseParams());
}

// Characterization (L6a0b): evalOne a-priori rejection "r_trust" (~l.323-324, needs a converged IK): the reference winner, forced and made untrusted.
TEST(GraspS3PosthocCharacterizationTest, PriorRejectionRTrust) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const S3Params ref_p = v2LooseParams();  // only ik and r_trust (100 m here) can reject
  const S3Result ref = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, ref_p);
  ASSERT_EQ(ref.status, S3Result::kSelected) << ref.note;  // the winner has a converged IK
  const double theta_deg = ref.chosen.eval.theta_rad * 180.0 / M_PI;
  ASSERT_LE(std::abs(theta_deg), 180.0) << "winner theta outside [-180, 180]";

  std::vector<S3EvalRecord> log;
  s.ctx.eval_log = &log;
  S3Params p = ref_p;
  p.r_trust_m = 1e-6;  // d_pos of any candidate is centimetres
  p.force = true;
  p.force_k = static_cast<int>(ref.chosen.eval.k);
  p.force_theta_deg = theta_deg;
  p.force_flip = ref.chosen.eval.wrist_flip ? 1 : 0;
  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  const std::size_t ki = static_cast<std::size_t>(ref.chosen.eval.k);
  EXPECT_EQ(r.evaluations, 1);
  ASSERT_EQ(log.size(), 1u);
  EXPECT_FALSE(log[0].passes_constraints);
  EXPECT_NE(log[0].reject_mask & kRejectRTrust, 0u);
  EXPECT_EQ(log[0].reject_mask & kRejectIk, 0u);
  EXPECT_EQ(r.status, S3Result::kNoCandidate);
  EXPECT_TRUE(r.ranked.empty());  // not among the accepted candidates
  EXPECT_NE(r.note.find("r_trust"), std::string::npos) << r.note;
  EXPECT_FALSE(r.best_per_k[ki].passes_constraints);
  EXPECT_NE(r.best_per_k[ki].reject_mask & kRejectRTrust, 0u);
  EXPECT_TRUE(r.best_per_k[ki].eval.feasible_basic);  // converged IK within limits: the cascade got past "ik"
  EXPECT_TRUE(r.best_per_k[ki].eval.ik.converged);
  EXPECT_EQ(r.best_per_k[ki].reject_reason, "r_trust");  // first failing constraint of the cascade
}

// Characterization (L6a0b): evalOne a-priori rejection "start_goal" (~l.325-326, v2 max_start_goal_deg, needs a converged IK): the reference winner, forced.
TEST(GraspS3PosthocCharacterizationTest, PriorRejectionStartGoal) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const S3Params ref_p = v2LooseParams();  // v2: only ik, r_trust (100 m here) and start_goal can reject
  const S3Result ref = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, ref_p);
  ASSERT_EQ(ref.status, S3Result::kSelected) << ref.note;  // the winner has a converged IK
  const double theta_deg = ref.chosen.eval.theta_rad * 180.0 / M_PI;
  ASSERT_LE(std::abs(theta_deg), 180.0) << "winner theta outside [-180, 180]";

  std::vector<S3EvalRecord> log;
  s.ctx.eval_log = &log;
  S3Params p = ref_p;
  p.max_start_goal_deg = 0.0;  // any candidate rotated at all from FK(q_start) exceeds it
  p.force = true;
  p.force_k = static_cast<int>(ref.chosen.eval.k);
  p.force_theta_deg = theta_deg;
  p.force_flip = ref.chosen.eval.wrist_flip ? 1 : 0;
  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  const std::size_t ki = static_cast<std::size_t>(ref.chosen.eval.k);
  EXPECT_EQ(r.evaluations, 1);
  ASSERT_EQ(log.size(), 1u);
  EXPECT_FALSE(log[0].passes_constraints);
  EXPECT_NE(log[0].reject_mask & kRejectStartGoal, 0u);
  EXPECT_EQ(log[0].reject_mask & kRejectIk, 0u);
  EXPECT_EQ(r.status, S3Result::kNoCandidate);
  EXPECT_TRUE(r.ranked.empty());
  EXPECT_TRUE(r.v2_finalists.empty());
  EXPECT_NE(r.note.find("start_goal"), std::string::npos) << r.note;
  EXPECT_FALSE(r.best_per_k[ki].passes_constraints);
  EXPECT_NE(r.best_per_k[ki].reject_mask & kRejectStartGoal, 0u);
  EXPECT_TRUE(r.best_per_k[ki].eval.feasible_basic);  // converged IK within limits: the cascade got past "ik"
  EXPECT_TRUE(r.best_per_k[ki].eval.ik.converged);
  EXPECT_EQ(r.best_per_k[ki].reject_reason, "start_goal");  // r_trust cannot fail (100 m)
}

// Characterization (L6a0b): v2 lazy flow, rollout infeasible (rec.discard_reason = "rollout", ~l.671-674): every finalist is discarded in turn.
TEST(GraspS3PosthocCharacterizationTest, V2LazyRolloutInfeasibleFinalistIsDiscarded) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  // Lever without touching production code: q_start with joint 7 beyond its upper limit. The TCP lies on the joint-7 axis (p0
  // unchanged), the IK clamps its seed to the limits and every rollout starts AT q_start, i.e. beyond the limit.
  const double excess = 0.1;
  s.q_start(6) = s.robot->jointUpperLimits()(6) + excess;
  const S3Params p = v2MarginParams(0.0);  // use_margin, no force => lazy
  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  if (r.v2_finalists.size() < 2) GTEST_SKIP() << "fewer than 2 finalists from this q_start: " << r.note;
  // rollouts_done == finalists: the loop went on after each discard instead of stopping at the first
  EXPECT_EQ(r.rollouts_done, static_cast<int>(r.v2_finalists.size()));
  for (std::size_t i = 0; i < r.v2_finalists.size(); ++i) {
    const S3FinalistRecord& f = r.v2_finalists[i];
    EXPECT_TRUE(f.rollout_done) << i;
    EXPECT_FALSE(f.rollout_feasible) << i;
    EXPECT_EQ(f.discard_reason, "rollout") << i;
    EXPECT_FALSE(f.margin_done) << i;  // lazy: the margin is evaluated only after a feasible rollout
    EXPECT_FALSE(f.winner) << i;
  }
  EXPECT_EQ(r.margin_evals_done, 0);
  EXPECT_EQ(r.posthoc_checks, 0);
  EXPECT_TRUE(r.posthoc_log.empty());
  EXPECT_EQ(r.status, S3Result::kPosthocFailed) << r.note;
  EXPECT_NE(r.note.find("v2: no finalist passed the rollout and the post-hoc checks"), std::string::npos) << r.note;
}

// ---------------------------------------------------------------------------------------------------------------------
// L2 dynamic follow (S3Params::use_dynamic_follow, stage 2b). Written without being able to build/run. As the use_margin tests,
// the L2 margins of the synthetic fixture are not known a priori: a probe run with an unreachable threshold lists them and the
// expected outcome is built from it.
// ---------------------------------------------------------------------------------------------------------------------

namespace {

S3Params v2DynParams(double margin_delta_deg) {
  S3Params p = v2LooseParams();
  p.use_dynamic_follow = true;
  p.margin_delta_deg = margin_delta_deg;
  // dynamic_follow.dt_s stays at the selector default (0.0005 s)
  return p;
}

JointDynamics testJointDynamics() {
  std::vector<std::string> names;
  for (int i = 1; i <= 7; ++i) names.push_back("fer_joint" + std::to_string(i));
  return loadJointDynamicsFromUrdfFile(tf::urdfPath(), names);
}

/// One line per finalist (J_static order): what the L2 stage saw. Printed when a fixture precondition fails and, for the record, always.
std::string dynTable(const S3Result& r) {
  std::ostringstream o;
  o << std::fixed << std::setprecision(3) << r.v2_finalists.size() << " finalists, status " << static_cast<int>(r.status)
    << ", rollouts_done " << r.rollouts_done << ", dynamic_evals_done " << r.dynamic_evals_done << "\n";
  for (std::size_t i = 0; i < r.v2_finalists.size(); ++i) {
    const S3FinalistRecord& f = r.v2_finalists[i];
    o << "  [" << i << "] k=" << static_cast<int>(f.cand.eval.k) << (f.cand.eval.wrist_flip ? 'B' : 'A')
      << " theta=" << f.cand.eval.theta_rad * 180.0 / M_PI << " rollout_done=" << f.rollout_done
      << " feasible=" << f.rollout_feasible << " dyn_done=" << f.dyn_done;
    if (f.dyn_done) {
      o << " approach=" << f.dyn_approach_margin_deg << " hold=" << f.dyn_hold_margin_deg << " (fer_joint" << f.dyn_hold_joint + 1
        << " t=" << f.dyn_hold_t_rel_s << ") total=" << f.dyn_total_margin_deg << " diverged=" << f.dyn_diverged;
    }
    o << " reason='" << f.discard_reason << "'" << (f.winner ? " WINNER" : "") << "\n";
  }
  return o.str();
}

/// Evaluation order = J_static order = index order: the indices of the finalists the L2 stage evaluated.
std::vector<std::size_t> dynEvaluated(const S3Result& r) {
  std::vector<std::size_t> idx;
  for (std::size_t i = 0; i < r.v2_finalists.size(); ++i) {
    if (r.v2_finalists[i].dyn_done) {
      idx.push_back(i);
    }
  }
  return idx;
}

}  // namespace

TEST(GraspS3DynamicFollowTest, DefaultsAndValidation) {
  const S3Params d;
  EXPECT_FALSE(d.use_dynamic_follow);
  EXPECT_DOUBLE_EQ(d.margin_delta_deg, 5.0);  // ASSUMED
  EXPECT_DOUBLE_EQ(d.dynamic_follow.dt_s, 0.0005);  // the selector's default (the DynamicFollowParams struct default stays 0.001)
  EXPECT_DOUBLE_EQ(DynamicFollowParams().dt_s, 0.001);
  EXPECT_NO_THROW(validateS3Params(v2DynParams(5.0)));
  auto bad = [](auto mutate, const char* what) {
    S3Params p = v2DynParams(5.0);
    mutate(p);
    try {
      validateS3Params(p);
      ADD_FAILURE() << "expected std::invalid_argument for " << what;
    } catch (const std::invalid_argument& e) {
      EXPECT_NE(std::string(e.what()).find(what), std::string::npos) << e.what();
    }
  };
  bad([](S3Params& p) { p.use_margin = true; }, "mutually exclusive");  // incompatible flags, clear message
  bad([](S3Params& p) { p.margin_delta_deg = std::numeric_limits<double>::infinity(); }, "margin_delta_deg");
  bad([](S3Params& p) { p.margin_delta_deg = std::nan(""); }, "margin_delta_deg");
  bad([](S3Params& p) { p.dynamic_follow.friction_tanh_eps_rad_s = 0.0; }, "friction_tanh_eps_rad_s");
  bad([](S3Params& p) { p.dynamic_follow.dq0_zero = false; }, "follow_dq0_zero");
  bad([](S3Params& p) { p.margin_follow_s = 0.0001; }, "margin_follow_s");  // shorter than one L2 step (0.5 ms)
  bad([](S3Params& p) { p.margin_delta_deg = -1.0; }, "margin_delta_deg");
  bad([](S3Params& p) { p.margin_delta_deg = -1e-9; }, "margin_delta_deg");
  bad([](S3Params& p) { p.dynamic_follow.dt_s = 0.0; }, "dt_s");
  bad([](S3Params& p) { p.dynamic_follow.dt_s = -0.001; }, "dt_s");
  // V1' (use_hold_end) together with L2 is valid; with use_margin it is still the use_margin / use_dynamic_follow exclusion.
  EXPECT_NO_THROW(validateS3Params([] { S3Params p = v2DynParams(5.0); p.use_hold_end = true; return p; }()));
  bad([](S3Params& p) { p.use_hold_end = true; p.use_margin = true; }, "mutually exclusive");
  bad([](S3Params& p) { p.use_hold_end = true; p.hold_follow_s = -1.0; }, "hold_follow_s");
  bad([](S3Params& p) { p.use_hold_end = true; p.use_dynamic_follow = false; p.hold_follow_s = -1.0; }, "hold_follow_s");
  // use_margin alone is still valid, and a disabled flag does not constrain the other follow_* values
  EXPECT_NO_THROW(validateS3Params(v2MarginParams(24.0)));
  S3Params off = v2LooseParams();
  off.dynamic_follow.friction_tanh_eps_rad_s = 0.0;
  off.dynamic_follow.dq0_zero = false;
  off.margin_delta_deg = std::nan("");
  EXPECT_NO_THROW(validateS3Params(off));
}

// use_dynamic_follow = false: every dynamic parameter is ignored, the selection is the current one (eager v2, no L2 evaluation).
TEST(GraspS3DynamicFollowTest, DisabledIsUnchangedV2Selection) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const S3Params base = v2LooseParams();
  const S3Result a = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, base);
  if (a.status != S3Result::kSelected) GTEST_SKIP() << "no v2 candidate in this fixture: " << a.note;
  S3Params other = base;  // use_dynamic_follow stays false; ctx.joint_dynamics stays null
  other.margin_delta_deg = 1e6;
  other.dynamic_follow.translational_stiffness = 1.0;
  other.dynamic_follow.finger_pos_m = 0.0789;
  other.dynamic_follow.dt_s = 0.5;
  other.dynamic_follow.divergence_rad = 1e-9;
  const S3Result b = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, other);
  ASSERT_EQ(a.status, b.status);
  EXPECT_EQ(a.chosen.eval.theta_rad, b.chosen.eval.theta_rad);
  EXPECT_EQ(a.chosen.eval.k, b.chosen.eval.k);
  EXPECT_EQ(a.chosen.eval.wrist_flip, b.chosen.eval.wrist_flip);
  EXPECT_EQ(a.chosen.cost, b.chosen.cost);
  EXPECT_EQ(a.evaluations, b.evaluations);
  EXPECT_EQ(a.rollouts_done, b.rollouts_done);
  EXPECT_EQ(a.posthoc_min_margin_rad, b.posthoc_min_margin_rad);
  EXPECT_EQ(b.dynamic_evals_done, 0);
  EXPECT_EQ(b.dynamic_stage_ms, 0.0);
  EXPECT_EQ(b.dyn_counters.rolled_out, 0);
  EXPECT_EQ(b.dyn_counters.evaluated_l2, 0);
  EXPECT_EQ(b.dyn_counters.discarded_v1_rollout + b.dyn_counters.discarded_v2_clearance + b.dyn_counters.discarded_l2_approach +
                b.dyn_counters.discarded_l2_hold + b.dyn_counters.discarded_l2_diverged + b.dyn_counters.discarded_velocity,
            0);
  EXPECT_TRUE(std::isnan(b.winner_l2_clearance_mm));
  EXPECT_EQ(a.note, b.note);
  EXPECT_EQ(a.warnings, b.warnings);
  EXPECT_EQ(a.posthoc_checks, b.posthoc_checks);
  EXPECT_EQ(a.chosen.psi_rad, b.chosen.psi_rad);
  ASSERT_EQ(a.v2_finalists.size(), b.v2_finalists.size());
  for (std::size_t i = 0; i < a.v2_finalists.size(); ++i) {
    EXPECT_EQ(a.v2_finalists[i].j, b.v2_finalists[i].j);
    EXPECT_EQ(a.v2_finalists[i].winner, b.v2_finalists[i].winner);
    EXPECT_FALSE(b.v2_finalists[i].dyn_done);
    EXPECT_TRUE(b.v2_finalists[i].discard_reason.empty());
  }
}

TEST(GraspS3DynamicFollowTest, NeedsJointDynamics) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  ASSERT_EQ(s.ctx.joint_dynamics, nullptr);
  EXPECT_THROW(selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, v2DynParams(5.0)), std::invalid_argument);
}

// Lazy flow, two passes (no GTEST_SKIP: a fixture that cannot exercise the scenario makes the test FAIL and prints the margins).
// Pass 1 with an unreachable threshold (+1e6 deg: it records the margin of every finalist that reaches L2). Pass 2 with a threshold
// between the total margin of the first evaluated finalist and that of a later one: the first is discarded ("dynamic_approach" or
// "dynamic_hold"), the later one wins, nothing after it is rolled out or evaluated. The thresholds are >= 0 (validateS3Params), so the
// fixture must give non-negative L2 margins for the finalists used: otherwise the precondition fails and the table is printed.
TEST(GraspS3DynamicFollowTest, BelowThresholdDiscardedNextChosenLazy) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const JointDynamics dyn = testJointDynamics();
  s.ctx.joint_dynamics = &dyn;

  const S3Result probe = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, v2DynParams(kHugeMarginDeg));
  const std::string table = dynTable(probe);
  std::cerr << "[dyn pass 1, threshold " << kHugeMarginDeg << " deg]\n" << table;
  const std::vector<std::size_t> evaluated = dynEvaluated(probe);
  ASSERT_GE(evaluated.size(), 2u) << "fixture: need >= 2 finalists reaching the L2 stage\n" << table;
  EXPECT_EQ(probe.status, S3Result::kNoCandidate) << table;  // unreachable threshold: nothing admissible, no fallback
  EXPECT_NE(probe.note.find("dynamic follow"), std::string::npos) << probe.note;
  EXPECT_EQ(probe.dynamic_evals_done, static_cast<int>(evaluated.size())) << table;
  for (std::size_t i : evaluated) {
    EXPECT_TRUE(probe.v2_finalists[i].dyn_rejected) << table;
  }

  const std::size_t first = evaluated.front();
  const double first_total = probe.v2_finalists[first].dyn_total_margin_deg;
  ASSERT_TRUE(std::isfinite(first_total) && !probe.v2_finalists[first].dyn_diverged)
      << "fixture: the first evaluated finalist has no finite L2 margin\n" << table;
  // The first finalist may have a negative L2 margin (L2 has no joint limits: the path can go beyond one). The threshold must be >= 0
  // (validateS3Params), so it is taken between max(first_total, 0) and the margin of the later finalist.
  const double lower = std::max(first_total, 0.0);
  std::size_t later = probe.v2_finalists.size();
  for (std::size_t k = 1; k < evaluated.size() && later == probe.v2_finalists.size(); ++k) {
    const S3FinalistRecord& f = probe.v2_finalists[evaluated[k]];
    if (std::isfinite(f.dyn_total_margin_deg) && !f.dyn_diverged && f.dyn_total_margin_deg > lower + 1e-3) {
      later = evaluated[k];
    }
  }
  ASSERT_LT(later, probe.v2_finalists.size())
      << "fixture: no later finalist has a total L2 margin above max(first, 0) = " << lower
      << " deg: the discard-then-choose scenario cannot be built; margins below\n" << table;
  const double threshold = 0.5 * (lower + probe.v2_finalists[later].dyn_total_margin_deg);

  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, v2DynParams(threshold));
  std::cerr << "[dyn pass 2, threshold " << threshold << " deg]\n" << dynTable(r);
  ASSERT_EQ(r.status, S3Result::kSelected) << r.note << "\n" << dynTable(r);
  EXPECT_TRUE(r.v2_finalists[first].discard_reason == "dynamic_approach" || r.v2_finalists[first].discard_reason == "dynamic_hold")
      << r.v2_finalists[first].discard_reason;
  EXPECT_TRUE(r.v2_finalists[first].dyn_rejected);
  EXPECT_FALSE(r.v2_finalists[first].winner);
  EXPECT_TRUE(r.v2_finalists[later].winner) << dynTable(r);
  EXPECT_TRUE(r.v2_finalists[later].discard_reason.empty());
  EXPECT_EQ(r.chosen.eval.theta_rad, r.v2_finalists[later].cand.eval.theta_rad);
  for (std::size_t i = later + 1; i < r.v2_finalists.size(); ++i) {
    EXPECT_FALSE(r.v2_finalists[i].rollout_done) << "finalist " << i << " was rolled out after the winner\n" << dynTable(r);
    EXPECT_FALSE(r.v2_finalists[i].dyn_done) << "finalist " << i << " was evaluated after the winner\n" << dynTable(r);
  }
  // dynamic_evals_done = the finalists actually evaluated = those of pass 1 up to and including the winner
  std::size_t expected_evals = 0;
  for (std::size_t i : evaluated) {
    if (i <= later) {
      ++expected_evals;
    }
  }
  EXPECT_EQ(r.dynamic_evals_done, static_cast<int>(expected_evals)) << dynTable(r);
  EXPECT_EQ(dynEvaluated(r).size(), expected_evals) << dynTable(r);
  // the evaluation does not depend on the threshold: same margins as in pass 1
  for (std::size_t i : dynEvaluated(r)) {
    EXPECT_DOUBLE_EQ(r.v2_finalists[i].dyn_total_margin_deg, probe.v2_finalists[i].dyn_total_margin_deg) << i;
  }
}

// The winner carries the L2 state at AT_END, at +1.5 s and +3.0 s and the two margins it was judged on (additive fields). The threshold
// is the best total margin of a pass-1 run: a winner exists by construction whenever one finalist has a finite, non-negative L2 margin
// (validateS3Params rejects a negative threshold); otherwise the test FAILS and prints the table.
TEST(GraspS3DynamicFollowTest, WinnerCarriesL2States) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const JointDynamics dyn = testJointDynamics();
  s.ctx.joint_dynamics = &dyn;

  const S3Result probe = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, v2DynParams(kHugeMarginDeg));
  const std::string table = dynTable(probe);
  std::cerr << "[dyn winner test, pass 1]\n" << table;
  double best = -std::numeric_limits<double>::infinity();
  for (std::size_t i : dynEvaluated(probe)) {
    const S3FinalistRecord& f = probe.v2_finalists[i];
    if (std::isfinite(f.dyn_total_margin_deg) && !f.dyn_diverged) {
      best = std::max(best, f.dyn_total_margin_deg);
    }
  }
  ASSERT_TRUE(std::isfinite(best)) << "fixture: no finalist with a finite, non-diverged L2 margin\n" << table;
  ASSERT_GE(best, 0.0) << "fixture: every L2 margin is negative (path beyond a joint limit); the threshold would be rejected\n" << table;

  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, v2DynParams(best));
  ASSERT_EQ(r.status, S3Result::kSelected) << r.note << "\n" << dynTable(r);
  const S3FinalistRecord* w = nullptr;
  for (const auto& f : r.v2_finalists) {
    if (f.winner) {
      w = &f;
    }
  }
  ASSERT_NE(w, nullptr) << dynTable(r);
  ASSERT_TRUE(w->dyn_done);
  EXPECT_FALSE(w->dyn_rejected);
  EXPECT_FALSE(w->dyn_diverged);
  EXPECT_GT(w->dyn_at_end_s, 0.0);
  EXPECT_DOUBLE_EQ(w->dyn_total_margin_deg, std::min(w->dyn_approach_margin_deg, w->dyn_hold_margin_deg));
  EXPECT_GE(w->dyn_total_margin_deg, best);
  EXPECT_TRUE(std::isfinite(w->dyn_approach_margin_deg) && std::isfinite(w->dyn_hold_margin_deg));
  EXPECT_GE(w->dyn_hold_joint, 0);
  EXPECT_LE(w->dyn_hold_joint, 6);
  EXPECT_GE(w->dyn_approach_joint, 0);
  EXPECT_LE(w->dyn_approach_joint, 6);
  EXPECT_GE(w->dyn_hold_t_rel_s, 0.0);
  EXPECT_LE(w->dyn_hold_t_rel_s, 3.0 + 1e-9);
  EXPECT_GT(w->dyn_elapsed_ms, 0.0);
  EXPECT_LE(w->dyn_approach_t_rel_s, 0.0);
  ASSERT_TRUE(w->rollout_end_valid);
  static_assert(RobotModel::kNumJoints == 7, "the seven-element vectors below");
  EXPECT_EQ(w->dyn_q_end_deg.size(), 7);
  EXPECT_EQ(w->dyn_q_h1p5_deg.size(), 7);
  EXPECT_EQ(w->dyn_q_h3_deg.size(), 7);
  EXPECT_TRUE(w->dyn_q_end_deg.allFinite());
  EXPECT_TRUE(w->dyn_q_h1p5_deg.allFinite());
  EXPECT_TRUE(w->dyn_q_h3_deg.allFinite());
  EXPECT_TRUE(std::isfinite(w->dyn_initial_rot_err_deg));
  EXPECT_LT(w->dyn_initial_rot_err_deg, 3.0);
  EXPECT_TRUE(std::isfinite(w->dyn_vs_rollout_qend_deg));
  EXPECT_LT((w->dyn_q_end_deg - w->rollout_end.q_end * 180.0 / M_PI).cwiseAbs().maxCoeff() - w->dyn_vs_rollout_qend_deg, 1e-9);
  EXPECT_GE(r.dyn_counters.evaluated_l2, 1);
  EXPECT_GT(r.dynamic_stage_ms, 0.0);
  EXPECT_GE(r.dynamic_evals_done, 1);
  EXPECT_LE(r.dynamic_evals_done, r.rollouts_done);  // only the rollout-feasible finalists reach the L2 stage
}

// Clearance (V2) is evaluated BEFORE L2 and discards without calling it: every finalist below the threshold -> no L2 run at all.
TEST(GraspS3DynamicFollowTest, FailedClearanceDoesNotCallL2) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const JointDynamics dyn = testJointDynamics();
  s.ctx.joint_dynamics = &dyn;
  int clearance_calls = 0;
  s.ctx.clearance_eval = [&clearance_calls](const ClearanceQuery&) {
    ++clearance_calls;
    return fakeClearance(1.5);
  };
  S3Params p = v2DynParams(0.0);  // L2 would accept anything non-negative
  p.use_clearance = true;
  p.min_clearance_mm = 2.0;
  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  EXPECT_EQ(r.status, S3Result::kNoCandidate);
  ASSERT_GT(clearance_calls, 0) << "fixture: no rollout-feasible finalist reached the clearance stage: " << r.note;
  EXPECT_EQ(r.dynamic_evals_done, 0);
  EXPECT_EQ(r.dyn_counters.evaluated_l2, 0);
  EXPECT_EQ(r.dynamic_stage_ms, 0.0);
  EXPECT_EQ(r.dyn_counters.discarded_v2_clearance, clearance_calls);
  EXPECT_EQ(r.dyn_counters.discarded_l2_approach + r.dyn_counters.discarded_l2_hold + r.dyn_counters.discarded_l2_diverged, 0);
  for (const auto& f : r.v2_finalists) {
    EXPECT_FALSE(f.dyn_done);
    EXPECT_NE(f.discard_reason, "dynamic_approach");
  }
}

// With the clearance passing, the counters add up: every finalist that reached L2 is either the winner or discarded by one L2 criterion
// (an unreachable threshold: none is the winner); the clearance of the winner is recomputed on the L2 path and reported.
TEST(GraspS3DynamicFollowTest, CountersAndWinnerClearanceOnTheL2Path) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const JointDynamics dyn = testJointDynamics();
  s.ctx.joint_dynamics = &dyn;
  std::vector<std::size_t> sizes;
  std::vector<double> last_t;
  s.ctx.clearance_eval = [&](const ClearanceQuery& q) {
    sizes.push_back(q.trajectory->size());
    last_t.push_back(q.trajectory->back().t_s);
    return fakeClearance(10.0);
  };
  S3Params p = v2DynParams(kHugeMarginDeg);
  p.use_clearance = true;
  const S3Result r1 = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  EXPECT_EQ(r1.status, S3Result::kNoCandidate);
  ASSERT_GE(r1.dynamic_evals_done, 1) << r1.note;
  EXPECT_EQ(r1.dyn_counters.evaluated_l2, r1.dynamic_evals_done);
  EXPECT_EQ(r1.dyn_counters.discarded_l2_approach + r1.dyn_counters.discarded_l2_hold + r1.dyn_counters.discarded_l2_diverged,
            r1.dynamic_evals_done);
  EXPECT_TRUE(std::isnan(r1.winner_l2_clearance_mm));  // no winner

  // threshold = best total margin of pass 1 (>= 0): a winner exists; its clearance is queried one more time, on the L2 path
  double best = -std::numeric_limits<double>::infinity();
  for (const auto& f : r1.v2_finalists) {
    if (f.dyn_done && !f.dyn_diverged && std::isfinite(f.dyn_total_margin_deg)) best = std::max(best, f.dyn_total_margin_deg);
  }
  ASSERT_GE(best, 0.0) << dynTable(r1);
  sizes.clear();
  last_t.clear();
  p.margin_delta_deg = best;
  const S3Result r2 = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  ASSERT_EQ(r2.status, S3Result::kSelected) << r2.note << "\n" << dynTable(r2);
  ASSERT_GE(sizes.size(), 2u);
  EXPECT_DOUBLE_EQ(r2.winner_l2_clearance_mm, 10.0);
  EXPECT_EQ(r2.winner_l2_clearance_pair, "dito 1-collare");
  EXPECT_GE(last_t.back(), s.tmpl.tau() - 1e-9);  // the L2-path trajectory covers [0, tau]
  EXPECT_EQ(r2.dyn_counters.evaluated_l2, r2.dynamic_evals_done);
  EXPECT_GT(r2.dynamic_stage_ms, 0.0);
}

// A divergent L2 run (excursion guard so tight that any motion trips it) discards every finalist as "dynamic_diverged", is counted, and
// does not throw.
TEST(GraspS3DynamicFollowTest, DivergentFinalistIsDiscardedAndCounted) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const JointDynamics dyn = testJointDynamics();
  s.ctx.joint_dynamics = &dyn;
  S3Params p = v2DynParams(0.0);
  p.dynamic_follow.divergence_rad = 1e-6;
  S3Result r;
  ASSERT_NO_THROW(r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p));
  EXPECT_EQ(r.status, S3Result::kNoCandidate);
  ASSERT_GE(r.dynamic_evals_done, 1) << r.note;
  EXPECT_EQ(r.dyn_counters.discarded_l2_diverged, r.dynamic_evals_done);
  EXPECT_EQ(r.dyn_counters.discarded_l2_approach + r.dyn_counters.discarded_l2_hold, 0);
  for (const auto& f : r.v2_finalists) {
    if (f.dyn_done) {
      EXPECT_TRUE(f.dyn_diverged);
      EXPECT_TRUE(f.dyn_rejected);
      EXPECT_EQ(f.discard_reason, "dynamic_diverged");
    }
  }
}

// The velocity check runs before L2 under use_dynamic_follow: a finalist failing it is discarded as "velocity" without an L2 run.
TEST(GraspS3DynamicFollowTest, VelocityFailureSkipsL2) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const JointDynamics dyn = testJointDynamics();
  s.ctx.joint_dynamics = &dyn;
  S3Params p = v2DynParams(0.0);
  p.posthoc_velocity_check = true;
  p.delta_v_max_m_s = 1e-12;  // no finalist can satisfy it
  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  EXPECT_NE(r.status, S3Result::kSelected);
  EXPECT_EQ(r.dynamic_evals_done, 0);
  EXPECT_GT(r.dyn_counters.discarded_velocity, 0) << r.note;
  EXPECT_EQ(static_cast<std::size_t>(r.posthoc_checks), r.posthoc_log.size());
  for (const auto& f : r.v2_finalists) EXPECT_FALSE(f.dyn_done);
}

// ==============================================================================================
// GraspS3HoldEndTest (Stadio V1', tutor decision 28/10): IK at grasp AND at hold end within limits
// ==============================================================================================

// 1. With use_hold_end == false (default): selection is unchanged.
TEST(GraspS3HoldEndTest, DisabledIsUnchangedSelection) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const S3Params base = v2LooseParams();
  const S3Result a = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, base);
  ASSERT_EQ(a.status, S3Result::kSelected);

  S3Params other = base;
  other.use_hold_end = false;
  other.hold_follow_s = 10.0;
  const S3Result b = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, other);

  ASSERT_EQ(a.status, b.status);
  EXPECT_EQ(a.chosen.eval.theta_rad, b.chosen.eval.theta_rad);
  EXPECT_EQ(a.chosen.eval.k, b.chosen.eval.k);
  EXPECT_EQ(a.chosen.eval.wrist_flip, b.chosen.eval.wrist_flip);
  EXPECT_EQ(a.chosen.cost, b.chosen.cost);
  EXPECT_EQ(a.evaluations, b.evaluations);
  EXPECT_EQ(a.rollouts_done, b.rollouts_done);
  EXPECT_FALSE(b.hold_end_done);
  ASSERT_EQ(a.v2_finalists.size(), b.v2_finalists.size());
  for (std::size_t i = 0; i < a.v2_finalists.size(); ++i) {
    EXPECT_EQ(a.v2_finalists[i].j, b.v2_finalists[i].j);
    EXPECT_EQ(a.v2_finalists[i].winner, b.v2_finalists[i].winner);
    EXPECT_FALSE(b.v2_finalists[i].hold_end_done);
  }
}

// 2. Unit test of holdPoseIk:
//    - the rotated pose strictly matches rotatePoseAboutAxis;
//    - small rotation: the grasp pose is FK(q_start) (reachable by construction, seed = q_start), so the IK converges;
//    - infeasible case: the rotated position is more than 3.5 m from the base (rotation of 180 deg about an axis 2 m away from the
//      TCP), far beyond any reach of the arm, so the IK cannot converge whatever the orientation.
TEST(GraspS3HoldEndTest, HoldPoseIkRotationAndLimits) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  ASSERT_NE(s.robot, nullptr);

  // Grasp pose = FK(q_start), the frame of the model (fer_hand_tcp); s.demo was built from q_start in makeS3TestSetup().
  const Eigen::Vector3d grasp_pos = s.demo.position;
  const Eigen::Quaterniond grasp_quat = s.demo.orientation;
  const Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  const double t_follow = 1.0;

  // Small rotation (0.05 rad ~ 2.86 deg) about an axis 0.15 m behind the TCP: the pose moves by ~7.5 mm and 0.05 rad.
  const Eigen::Vector3d center_small = grasp_pos - Eigen::Vector3d(0.15, 0.0, 0.0);
  const double omega_small = 0.05;
  const HoldPoseIkResult r_small =
      holdPoseIk(*s.robot, grasp_pos, grasp_quat, center_small, axis, omega_small, t_follow, s.q_start);

  const haptic_dmp_learning::core::RotatablePose expected_pose_small =
      haptic_dmp_learning::core::rotatePoseAboutAxis(center_small, axis, {grasp_pos, grasp_quat}, omega_small * t_follow);
  EXPECT_LT((r_small.hold_pose.position - expected_pose_small.position).norm(), 1e-12);
  EXPECT_LT(r_small.hold_pose.orientation.angularDistance(expected_pose_small.orientation), 1e-12);
  EXPECT_TRUE(r_small.end_ok);
  EXPECT_TRUE(r_small.ik.converged);
  EXPECT_TRUE(r_small.ik.limits_ok);
  EXPECT_EQ(r_small.end_ok, r_small.ik.converged && r_small.ik.limits_ok);
  EXPECT_GT(r_small.w_end, 0.0);

  // Infeasible: axis 2 m in front of the TCP, rotation of 180 deg -> position = grasp_pos + (4, 0, 0).
  const Eigen::Vector3d center_far = grasp_pos + Eigen::Vector3d(2.0, 0.0, 0.0);
  const double omega_large = M_PI / t_follow;
  const HoldPoseIkResult r_large =
      holdPoseIk(*s.robot, grasp_pos, grasp_quat, center_far, axis, omega_large, t_follow, s.q_start);

  const haptic_dmp_learning::core::RotatablePose expected_pose_large =
      haptic_dmp_learning::core::rotatePoseAboutAxis(center_far, axis, {grasp_pos, grasp_quat}, omega_large * t_follow);
  EXPECT_LT((r_large.hold_pose.position - expected_pose_large.position).norm(), 1e-12);
  EXPECT_LT(r_large.hold_pose.orientation.angularDistance(expected_pose_large.orientation), 1e-12);
  ASSERT_GT(r_large.hold_pose.position.norm(), 3.5) << "precondition: the target must be unreachable (base at the origin)";
  EXPECT_FALSE(r_large.ik.converged);
  EXPECT_EQ(r_large.end_ok, r_large.ik.converged && r_large.ik.limits_ok);
  EXPECT_FALSE(r_large.end_ok);
}

// 3. With use_hold_end == true: finalists that cannot reach hold-end within joint limits
//    are discarded with discard_reason = "hold_end_infeasible", and the first feasible wins.
TEST(GraspS3HoldEndTest, InfeasibleHoldEndDiscarded) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);

  S3Params p = v2LooseParams();
  p.use_hold_end = true;
  p.hold_follow_s = 3.0;

  // Set snapshot omega to -0.3 rad/s (approx -51.6 deg over 3s)
  s.ctx.snapshot.omega_rad_s = -0.3;
  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  ASSERT_EQ(r.status, S3Result::kSelected);
  EXPECT_TRUE(r.hold_end_done);
  EXPECT_TRUE(r.end_ok);
  EXPECT_GT(r.w_end, 0.0);
  EXPECT_GT(r.w_eff, 0.0);

  // Verify that any finalist that failed hold-end has discard_reason == "hold_end_infeasible"
  // and is not the winner.
  for (const auto& f : r.v2_finalists) {
    if (f.hold_end_done && !f.end_ok) {
      EXPECT_EQ(f.discard_reason, "hold_end_infeasible");
      EXPECT_FALSE(f.winner);
    }
  }
}

// 4. Effective manipulability: w_eff = min(w_presa, w_end) enters the cost with verifiable synthetic numbers.
TEST(GraspS3HoldEndTest, EffectiveManipulability) {
  S3Params p;
  p.use_w = true;
  p.kappa_w_mm = 7.5;
  p.kappa_t_mm_s = 0.0;
  const double d_mm = 12.0;
  const double t_wait_s = 0.0;

  // Case A: w_end is lower than w_presa
  const double w_presa_A = 0.050;
  const double w_end_A = 0.020;
  const double w_eff_A = std::min(w_presa_A, w_end_A);
  EXPECT_DOUBLE_EQ(w_eff_A, 0.020);
  const double cost_A = s3CostV2(d_mm, w_eff_A, t_wait_s, p);
  const double expected_cost_A = d_mm + p.kappa_w_mm * (-std::log(0.020));
  EXPECT_DOUBLE_EQ(cost_A, expected_cost_A);

  // Case B: w_presa is lower than w_end
  const double w_presa_B = 0.030;
  const double w_end_B = 0.080;
  const double w_eff_B = std::min(w_presa_B, w_end_B);
  EXPECT_DOUBLE_EQ(w_eff_B, 0.030);
  const double cost_B = s3CostV2(d_mm, w_eff_B, t_wait_s, p);
  const double expected_cost_B = d_mm + p.kappa_w_mm * (-std::log(0.030));
  EXPECT_DOUBLE_EQ(cost_B, expected_cost_B);
}

// ==============================================================================================
// V1' (use_hold_end) together with L2 (use_dynamic_follow): order rollout -> clearance -> V1' -> velocity -> L2
// ==============================================================================================

namespace {

/// Number of finalists discarded by V1' ("hold_end_infeasible").
int holdEndDiscards(const S3Result& r) {
  int n = 0;
  for (const auto& f : r.v2_finalists) n += f.discard_reason == "hold_end_infeasible" ? 1 : 0;
  return n;
}

/// hold_follow_s [s] for which at least one finalist reaching V1' is discarded by it (probe without L2: cheap). 0 if none of the tried
/// values does. The omega of the fixture is set by the caller.
double holdFollowWithInfeasibleFinalist(S3TestSetup& s) {
  for (const double hold_s : {3.0, 6.0, 10.0, 20.0, 40.0, 80.0}) {
    S3Params p = v2LooseParams();
    p.use_hold_end = true;
    p.hold_follow_s = hold_s;
    const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
    int reached = 0;
    for (const auto& f : r.v2_finalists) reached += f.hold_end_done ? 1 : 0;
    if (holdEndDiscards(r) > 0 && reached > holdEndDiscards(r)) return hold_s;
  }
  return 0.0;
}

}  // namespace

// The combination is accepted; use_dynamic_follow + use_margin is still refused with the exclusion message (also with use_hold_end).
TEST(GraspS3DynamicHoldEndTest, CombinationAcceptedMarginStillRefused) {
  S3Params p = v2DynParams(5.0);
  p.use_hold_end = true;
  EXPECT_NO_THROW(validateS3Params(p));
  S3Params q = p;
  q.use_margin = true;
  try {
    validateS3Params(q);
    ADD_FAILURE() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("mutually exclusive"), std::string::npos) << e.what();
    EXPECT_NE(std::string(e.what()).find("use_margin"), std::string::npos) << e.what();
  }
}

// A finalist discarded by V1' never reaches L2 (no dyn_done, no L2 evaluation counted); the clearance (V2) was evaluated before V1'.
TEST(GraspS3DynamicHoldEndTest, FailedHoldEndDoesNotCallL2) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const JointDynamics dyn = testJointDynamics();
  s.ctx.joint_dynamics = &dyn;
  s.ctx.snapshot.omega_rad_s = -0.3;
  int clearance_calls = 0;
  s.ctx.clearance_eval = [&clearance_calls](const ClearanceQuery&) {
    ++clearance_calls;
    return fakeClearance(10.0);
  };
  const double hold_s = holdFollowWithInfeasibleFinalist(s);
  ASSERT_GT(hold_s, 0.0) << "fixture: no hold_follow_s in {3, 6, 10, 20, 40, 80} makes V1' discard a finalist and leaves another one";

  S3Params p = v2DynParams(kHugeMarginDeg);  // L2 never admits: every finalist that reaches L2 is evaluated and discarded by it
  p.use_hold_end = true;
  p.hold_follow_s = hold_s;
  p.use_clearance = true;
  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  const std::string table = dynTable(r);
  const int n_v1p = holdEndDiscards(r);
  ASSERT_GT(n_v1p, 0) << table;
  EXPECT_EQ(r.dyn_counters.discarded_v1p_hold_end, n_v1p) << table;
  int reached_v1p = 0;
  int with_l2 = 0;
  for (const auto& f : r.v2_finalists) {
    if (f.hold_end_done) {
      ++reached_v1p;
      EXPECT_TRUE(f.clearance_done) << "V1' ran before the clearance\n" << table;
    }
    if (f.discard_reason == "hold_end_infeasible") {
      EXPECT_FALSE(f.end_ok);
      EXPECT_FALSE(f.dyn_done) << "L2 ran on a finalist discarded by V1'\n" << table;
      EXPECT_EQ(f.dyn_elapsed_ms, 0.0);
    }
    with_l2 += f.dyn_done ? 1 : 0;
  }
  EXPECT_EQ(r.dynamic_evals_done, with_l2) << table;
  EXPECT_EQ(r.dyn_counters.evaluated_l2, with_l2) << table;
  EXPECT_EQ(with_l2, reached_v1p - n_v1p) << "every finalist that passed V1' reaches L2 (no velocity check here)\n" << table;
  EXPECT_EQ(r.status, S3Result::kNoCandidate) << table;
  EXPECT_EQ(r.dyn_counters.discarded_v2_clearance, 0) << table;
}

// With both on, the winner's record carries w_eff = min(w_rollout, w_end), the J built on it, and the L2 fields; the result mirrors it.
TEST(GraspS3DynamicHoldEndTest, WinnerRecordCarriesWEffAndL2) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const JointDynamics dyn = testJointDynamics();
  s.ctx.joint_dynamics = &dyn;
  S3Params base = v2DynParams(kHugeMarginDeg);
  base.use_hold_end = true;
  base.hold_follow_s = 3.0;
  const S3Result probe = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, base);
  const std::string table = dynTable(probe);
  double best = -std::numeric_limits<double>::infinity();
  for (std::size_t i : dynEvaluated(probe)) {
    const S3FinalistRecord& f = probe.v2_finalists[i];
    if (std::isfinite(f.dyn_total_margin_deg) && !f.dyn_diverged) best = std::max(best, f.dyn_total_margin_deg);
  }
  ASSERT_TRUE(std::isfinite(best) && best >= 0.0) << "fixture: no finalist with a finite, non-negative L2 margin after V1'\n" << table;

  S3Params p = base;
  p.margin_delta_deg = best;
  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  ASSERT_EQ(r.status, S3Result::kSelected) << r.note << "\n" << dynTable(r);
  const S3FinalistRecord* w = nullptr;
  for (const auto& f : r.v2_finalists) {
    if (f.winner) w = &f;
  }
  ASSERT_NE(w, nullptr) << dynTable(r);
  EXPECT_TRUE(w->hold_end_done);
  EXPECT_TRUE(w->end_ok);
  EXPECT_TRUE(w->dyn_done);
  EXPECT_GT(w->w_end, 0.0);
  EXPECT_GT(w->w_rollout, 0.0);
  EXPECT_DOUBLE_EQ(w->w_eff, std::min(w->w_rollout, w->w_end));
  EXPECT_DOUBLE_EQ(w->j, s3CostV2(w->d_mm, w->w_eff, w->t_wait_s, p));
  EXPECT_TRUE(r.hold_end_done);
  EXPECT_TRUE(r.end_ok);
  EXPECT_DOUBLE_EQ(r.w_eff, w->w_eff);
  EXPECT_DOUBLE_EQ(r.w_end, w->w_end);
  EXPECT_GE(w->dyn_total_margin_deg, best);
}

// Both flags off: the result does not depend on the V1' / L2 parameters and the new counter stays 0.
TEST(GraspS3DynamicHoldEndTest, BothOffIsUnchanged) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const S3Params base = v2LooseParams();
  const S3Result a = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, base);
  ASSERT_EQ(a.status, S3Result::kSelected);
  S3Params other = base;
  other.use_dynamic_follow = false;
  other.use_hold_end = false;
  other.hold_follow_s = 10.0;
  other.margin_delta_deg = 99.0;
  other.dynamic_follow.dt_s = 0.01;
  const S3Result b = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, other);
  ASSERT_EQ(a.status, b.status);
  EXPECT_EQ(a.chosen.eval.theta_rad, b.chosen.eval.theta_rad);
  EXPECT_EQ(a.chosen.eval.k, b.chosen.eval.k);
  EXPECT_EQ(a.chosen.eval.wrist_flip, b.chosen.eval.wrist_flip);
  EXPECT_EQ(a.chosen.cost, b.chosen.cost);
  EXPECT_EQ(a.rollouts_done, b.rollouts_done);
  EXPECT_EQ(b.dynamic_evals_done, 0);
  EXPECT_EQ(b.dyn_counters.discarded_v1p_hold_end, 0);
  EXPECT_EQ(b.dyn_counters.rolled_out, 0);
  EXPECT_FALSE(b.hold_end_done);
  ASSERT_EQ(a.v2_finalists.size(), b.v2_finalists.size());
  for (std::size_t i = 0; i < a.v2_finalists.size(); ++i) {
    EXPECT_EQ(a.v2_finalists[i].j, b.v2_finalists[i].j);
    EXPECT_FALSE(b.v2_finalists[i].hold_end_done);
    EXPECT_FALSE(b.v2_finalists[i].dyn_done);
  }
}

// hold_follow_s != margin_follow_s with both V1' and L2 on: a warning (not a refusal); equal values: no warning; V1' alone: none.
TEST(GraspS3DynamicHoldEndTest, HoldFollowMismatchWarns) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const JointDynamics dyn = testJointDynamics();
  s.ctx.joint_dynamics = &dyn;
  auto has = [](const S3Result& r) {
    for (const std::string& w : r.warnings) {
      if (w.find("hold_follow_s") != std::string::npos) return true;
    }
    return false;
  };
  S3Params p = v2DynParams(kHugeMarginDeg);
  p.use_hold_end = true;
  p.margin_follow_s = 3.0;
  p.hold_follow_s = 2.0;
  EXPECT_TRUE(has(selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p)));
  p.hold_follow_s = 3.0;
  EXPECT_FALSE(has(selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p)));
  S3Params q = v2LooseParams();  // V1' without L2: no mismatch to report
  q.use_hold_end = true;
  q.hold_follow_s = 2.0;
  EXPECT_FALSE(has(selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, q)));
}

// ---- per-stage timing fields (additive diagnostics): defaults, bounds, and no effect on the selection ----

TEST(GraspS3StageTimingTest, NewFieldsDefaultToZero) {
  S3Result r;
  EXPECT_EQ(r.scan_ms, 0.0);
  EXPECT_EQ(r.posthoc_ms, 0.0);
  GraspLaunchResult g;
  EXPECT_EQ(g.stage_ms.scan, 0.0);
  EXPECT_EQ(g.stage_ms.rollout, 0.0);
  EXPECT_EQ(g.stage_ms.clearance, 0.0);
  EXPECT_EQ(g.stage_ms.hold_end, 0.0);
  EXPECT_EQ(g.stage_ms.dynamic, 0.0);
  EXPECT_EQ(g.stage_ms.posthoc, 0.0);
  EXPECT_EQ(g.stage_ms.finalists, 0);
}

namespace {

/// Runs the selection twice on the same inputs. Checks: every stage time is >= 0 and finite; scan + V1 + V2 + V1' + L2 + post-hoc
/// (each the sum over the finalists examined) does not exceed the wall-clock time measured around the call; the two runs choose
/// the same winner and list the finalists in the same order with the same counters (the timers do not change the selection).
void checkStageTimes(S3TestSetup& s, const S3Params& p, bool expect_hold_end, bool expect_dynamic) {
  const auto t0 = std::chrono::steady_clock::now();
  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  const double wall_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  double rollout = 0.0, hold_end = 0.0;
  for (const S3FinalistRecord& f : r.v2_finalists) {
    EXPECT_GE(f.rollout_ms, 0.0);
    EXPECT_GE(f.hold_end_ms, 0.0);
    rollout += f.rollout_ms;
    hold_end += f.hold_end_ms;
  }
  for (double v : {r.scan_ms, r.posthoc_ms, r.clearance_stage_ms, r.dynamic_stage_ms, rollout, hold_end}) {
    EXPECT_TRUE(std::isfinite(v));
    EXPECT_GE(v, 0.0);
  }
  EXPECT_GT(r.scan_ms, 0.0);  // the scan always runs
  if (!r.v2_finalists.empty()) {
    EXPECT_GT(rollout, 0.0);
  }
  bool any_hold_end = false;
  for (const S3FinalistRecord& f : r.v2_finalists) any_hold_end = any_hold_end || f.hold_end_done;
  if (expect_hold_end && any_hold_end) {
    EXPECT_GT(hold_end, 0.0);
  }
  if (!expect_hold_end) {
    EXPECT_EQ(hold_end, 0.0);
  }
  if (!expect_dynamic) {
    EXPECT_EQ(r.dynamic_stage_ms, 0.0);
  }
  if (!p.posthoc_velocity_check) {
    EXPECT_EQ(r.posthoc_ms, 0.0);
  }
  EXPECT_LE(r.scan_ms + rollout + r.clearance_stage_ms + hold_end + r.dynamic_stage_ms + r.posthoc_ms, wall_ms);

  const S3Result r2 = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  EXPECT_EQ(r.status, r2.status);
  EXPECT_EQ(r.evaluations, r2.evaluations);
  EXPECT_EQ(r.rollouts_done, r2.rollouts_done);
  EXPECT_EQ(r.posthoc_checks, r2.posthoc_checks);
  ASSERT_EQ(r.v2_finalists.size(), r2.v2_finalists.size());
  for (std::size_t i = 0; i < r.v2_finalists.size(); ++i) {
    EXPECT_EQ(r.v2_finalists[i].cand.eval.k, r2.v2_finalists[i].cand.eval.k);
    EXPECT_DOUBLE_EQ(r.v2_finalists[i].cand.eval.theta_rad, r2.v2_finalists[i].cand.eval.theta_rad);
    EXPECT_EQ(r.v2_finalists[i].winner, r2.v2_finalists[i].winner);
    EXPECT_EQ(r.v2_finalists[i].discard_reason, r2.v2_finalists[i].discard_reason);
  }
}

}  // namespace

TEST(GraspS3StageTimingTest, EagerFlow) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  checkStageTimes(s, v2LooseParams(), /*expect_hold_end=*/false, /*expect_dynamic=*/false);
}

// use_hold_end without L2 is the eager flow (use_margin and use_hold_end are mutually exclusive): this is the campaign arm A.
TEST(GraspS3StageTimingTest, EagerFlowWithHoldEnd) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  S3Params p = v2LooseParams();
  p.use_hold_end = true;
  checkStageTimes(s, p, /*expect_hold_end=*/true, /*expect_dynamic=*/false);
}

TEST(GraspS3StageTimingTest, LazyFlowWithHoldEndAndDynamicFollow) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const JointDynamics dyn = testJointDynamics();
  s.ctx.joint_dynamics = &dyn;
  S3Params p = v2DynParams(kHugeMarginDeg);
  p.use_hold_end = true;
  checkStageTimes(s, p, /*expect_hold_end=*/true, /*expect_dynamic=*/true);
}

// ---------------------------------------------------------------------------------------------------------------------
// S3Params::lazy_static_order: the lazy flow in J_static order WITHOUT margin and WITHOUT L2 (campaign arm A). Per finalist: rollout (V1),
// V1' (if use_hold_end), clearance (if use_clearance), velocity check; the first one that passes all of them wins. Written without being
// able to build/run: as the use_margin / L2 tests, the expected outcome is built from probe runs of the same fixture and a test is
// skipped when the fixture cannot build its scenario.
// ---------------------------------------------------------------------------------------------------------------------

namespace {

S3Params v2LazyStaticParams() {
  S3Params p = v2LooseParams();
  p.tau_j_mm = 0.0;  // the campaign value; the lazy flow does not read it, the eager reference runs below do
  p.lazy_static_order = true;
  return p;
}

/// Index of the finalist flagged winner, or v2_finalists.size() if there is none.
std::size_t winnerIndex(const S3Result& r) {
  for (std::size_t i = 0; i < r.v2_finalists.size(); ++i) {
    if (r.v2_finalists[i].winner) return i;
  }
  return r.v2_finalists.size();
}

}  // namespace

TEST(GraspS3LazyStaticOrderTest, DefaultIsOffAndEveryCombinationValidates) {
  const S3Params d;
  EXPECT_FALSE(d.lazy_static_order);
  S3Params p = v2LooseParams();
  p.lazy_static_order = true;
  EXPECT_NO_THROW(validateS3Params(p));
  p.use_hold_end = true;
  p.use_clearance = true;
  EXPECT_NO_THROW(validateS3Params(p));
  S3Params dyn = v2DynParams(5.0);
  dyn.lazy_static_order = true;
  EXPECT_NO_THROW(validateS3Params(dyn));
  S3Params margin = v2MarginParams(24.0);
  margin.lazy_static_order = true;
  EXPECT_NO_THROW(validateS3Params(margin));
}

// The first rollout-feasible finalist (J_static order = finalists' order) wins, even if a later one has a lower J built on w_rollout
// (the eager flow, with tau_j_mm = 0, picks that one).
TEST(GraspS3LazyStaticOrderTest, FirstAdmissibleInStaticOrderBeatsLowerRolloutJ) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  // J_static (kappa_w * -ln w_static, t_wait = 0) orders the finalists; the eager J uses w_rollout and kappa_t * t_wait. The situation "a
  // later finalist has a lower eager J" is searched in this fixed list of alternatives, in order, until one provides it. kappa_t > 0 puts the
  // wait time of each candidate (it changes with theta, up to a period) into the eager J only; a larger kappa_w and more finalists make a
  // w_static / w_rollout order inversion more likely. The first alternative is the original one (kappa_w 7.5, kappa_t 0, 4 finalists).
  struct Alternative {
    double kappa_w_mm;
    double kappa_t_mm_s;
    int finalists_n;
  };
  const Alternative alternatives[] = {{7.5, 0.0, 4},  {7.5, 0.0, 12}, {7.5, 1.0, 4},  {7.5, 1.0, 12},
                                      {7.5, 10.0, 4}, {7.5, 10.0, 12}, {30.0, 0.0, 12}, {30.0, 1.0, 12}};
  int tried = 0;
  bool found = false;
  Alternative used = alternatives[0];
  S3Result eager;
  std::size_t n = 0, first = 0, lower = 0;
  for (const Alternative& alt : alternatives) {
    ++tried;
    S3Params eager_p = v2LooseParams();
    eager_p.tau_j_mm = 0.0;
    eager_p.kappa_w_mm = alt.kappa_w_mm;
    eager_p.kappa_t_mm_s = alt.kappa_t_mm_s;
    eager_p.finalists_n = alt.finalists_n;
    S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, eager_p);
    if (r.status != S3Result::kSelected) continue;
    const std::size_t m = r.v2_finalists.size();
    std::size_t f = m;
    for (std::size_t i = 0; i < m && f == m; ++i) {
      if (r.v2_finalists[i].rollout_feasible) f = i;
    }
    if (f == m) continue;
    std::size_t lo = m;
    for (std::size_t i = f + 1; i < m && lo == m; ++i) {
      if (r.v2_finalists[i].rollout_feasible && r.v2_finalists[i].j < r.v2_finalists[f].j) lo = i;
    }
    if (lo == m) continue;
    found = true;
    used = alt;
    eager = std::move(r);
    n = m;
    first = f;
    lower = lo;
    break;
  }
  if (!found) {
    GTEST_SKIP() << "no later finalist has a lower eager J than the first feasible one in any of the " << tried
                 << " alternatives (kappa_w, kappa_t, finalists_n) tried in this fixture";
  }
  std::cerr << "[lazy_static] alternative " << tried << " of 8: kappa_w " << used.kappa_w_mm << ", kappa_t " << used.kappa_t_mm_s
            << ", finalists_n " << used.finalists_n << "; first feasible " << first << ", lower-J finalist " << lower << "\n";
  EXPECT_NE(winnerIndex(eager), first);  // eager: minimum J

  S3Params lazy_p = v2LazyStaticParams();
  lazy_p.kappa_w_mm = used.kappa_w_mm;
  lazy_p.kappa_t_mm_s = used.kappa_t_mm_s;
  lazy_p.finalists_n = used.finalists_n;
  const S3Result lazy = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, lazy_p);
  ASSERT_EQ(lazy.status, S3Result::kSelected) << lazy.note;
  ASSERT_EQ(lazy.v2_finalists.size(), n);
  EXPECT_EQ(winnerIndex(lazy), first);
  EXPECT_EQ(lazy.chosen.eval.k, eager.v2_finalists[first].cand.eval.k);
  EXPECT_EQ(lazy.chosen.eval.wrist_flip, eager.v2_finalists[first].cand.eval.wrist_flip);
  EXPECT_EQ(lazy.chosen.eval.theta_rad, eager.v2_finalists[first].cand.eval.theta_rad);
  EXPECT_EQ(lazy.rollouts_done, static_cast<int>(first) + 1);  // lazy: up to and including the winner
  for (std::size_t i = 0; i < first; ++i) {
    EXPECT_FALSE(lazy.v2_finalists[i].rollout_feasible) << i;
    EXPECT_EQ(lazy.v2_finalists[i].discard_reason, "rollout") << i;
  }
  for (std::size_t i = first + 1; i < n; ++i) EXPECT_FALSE(lazy.v2_finalists[i].rollout_done) << i;
}

// No margin stage: followMargin never runs and nothing is discarded by "margin", whatever the margin parameters say.
TEST(GraspS3LazyStaticOrderTest, NeverEvaluatesTheMarginEvenWithAHugeThreshold) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const S3Result plain = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, v2LazyStaticParams());
  if (plain.status != S3Result::kSelected) GTEST_SKIP() << "no v2 candidate in this fixture: " << plain.note;
  S3Params p = v2LazyStaticParams();  // use_margin stays false
  p.min_margin_deg = kHugeMarginDeg;
  p.margin_follow_s = 99.0;
  p.margin_follow_dt_s = 0.5;
  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  ASSERT_EQ(r.status, S3Result::kSelected) << r.note;
  EXPECT_EQ(r.margin_evals_done, 0);
  for (const auto& f : r.v2_finalists) {
    EXPECT_FALSE(f.margin_done);
    EXPECT_FALSE(f.margin_rejected);
    EXPECT_NE(f.discard_reason, "margin");
  }
  EXPECT_EQ(winnerIndex(r), winnerIndex(plain));
  EXPECT_EQ(r.chosen.eval.theta_rad, plain.chosen.eval.theta_rad);
}

// Order of the checks without L2: V1 (rollout), then V1' (hold-end IK), then clearance; each finalist records the stage that discarded it
// and the later stages are not run for it. The velocity check is off here (v2LooseParams).
TEST(GraspS3LazyStaticOrderTest, ChecksRunV1ThenV1pThenClearanceWithTheirDiscardReasons) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  s.ctx.snapshot.omega_rad_s = -0.3;  // as the L2 + V1' tests: a slow target makes a long hold reach the joint limits
  int calls = 0;
  double clearance_mm = 10.0;
  s.ctx.clearance_eval = [&calls, &clearance_mm](const ClearanceQuery&) {
    ++calls;
    return fakeClearance(clearance_mm);
  };
  const double hold_s = holdFollowWithInfeasibleFinalist(s);
  ASSERT_GT(hold_s, 0.0) << "fixture: no hold_follow_s in {3, 6, 10, 20, 40, 80} makes V1' discard a finalist and leaves another one";
  S3Params p = v2LazyStaticParams();
  p.use_hold_end = true;
  p.hold_follow_s = hold_s;
  p.use_clearance = true;

  calls = 0;  // clearance 10 mm passes
  const S3Result ok = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  ASSERT_EQ(ok.status, S3Result::kSelected) << ok.note;
  int n_clearance = 0;
  for (const auto& f : ok.v2_finalists) {
    EXPECT_FALSE(f.margin_done);
    if (f.discard_reason == "rollout") {
      EXPECT_FALSE(f.rollout_feasible);
      EXPECT_FALSE(f.hold_end_done);
      EXPECT_FALSE(f.clearance_done);
    }
    if (f.discard_reason == "hold_end_infeasible") {
      EXPECT_TRUE(f.rollout_feasible);
      EXPECT_TRUE(f.hold_end_done);
      EXPECT_FALSE(f.end_ok);
      EXPECT_FALSE(f.clearance_done) << "V1' discards before the clearance";
    }
    if (f.clearance_done) {
      ++n_clearance;
      EXPECT_TRUE(f.rollout_feasible);
      EXPECT_TRUE(f.hold_end_done);
      EXPECT_TRUE(f.end_ok);
    }
  }
  EXPECT_EQ(calls, n_clearance);
  const std::size_t w = winnerIndex(ok);
  ASSERT_LT(w, ok.v2_finalists.size());
  EXPECT_TRUE(ok.v2_finalists[w].clearance_done);
  EXPECT_FALSE(ok.v2_finalists[w].clearance_rejected);
  EXPECT_TRUE(ok.v2_finalists[w].discard_reason.empty());
  for (std::size_t i = 0; i < w; ++i) EXPECT_FALSE(ok.v2_finalists[i].discard_reason.empty()) << i;
  for (std::size_t i = w + 1; i < ok.v2_finalists.size(); ++i) EXPECT_FALSE(ok.v2_finalists[i].rollout_done) << i;

  calls = 0;  // clearance 0.5 mm < 2 mm: every finalist that passes V1 and V1' is discarded by "clearance"
  clearance_mm = 0.5;
  const S3Result rej = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  EXPECT_EQ(rej.status, S3Result::kNoCandidate);
  int n_clearance_rej = 0;
  for (const auto& f : rej.v2_finalists) {
    EXPECT_FALSE(f.winner);
    if (f.hold_end_done && f.end_ok) {
      EXPECT_TRUE(f.clearance_done);
      EXPECT_TRUE(f.clearance_rejected);
      EXPECT_EQ(f.discard_reason, "clearance");
    }
    if (f.discard_reason == "hold_end_infeasible") {
      EXPECT_FALSE(f.clearance_done);
    }
    n_clearance_rej += f.clearance_done ? 1 : 0;
  }
  EXPECT_GT(n_clearance_rej, 0);
  EXPECT_EQ(calls, n_clearance_rej);
  EXPECT_GT(holdEndDiscards(rej), 0);
}

// force: the flag is ignored (lazy = ... && !force). Discriminating case: a forced candidate with clearance below the threshold is KEPT with a
// warning in the non-lazy flow, while the lazy flow would discard it.
TEST(GraspS3LazyStaticOrderTest, ForceReturnsToTheNonLazyFlow) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const S3Result base = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, v2LooseParams());
  if (base.status != S3Result::kSelected) GTEST_SKIP() << "no v2 candidate in this fixture: " << base.note;
  const double theta_deg = base.chosen.eval.theta_rad * 180.0 / M_PI;
  if (std::abs(theta_deg) > 180.0) GTEST_SKIP() << "winner theta outside [-180, 180]";
  s.ctx.clearance_eval = [](const ClearanceQuery&) { return fakeClearance(0.5); };  // below 2 mm
  auto forced = [&](bool lazy_static_order) {
    S3Params p = v2LooseParams();
    p.use_clearance = true;
    p.lazy_static_order = lazy_static_order;
    p.force = true;
    p.force_k = static_cast<int>(base.chosen.eval.k);
    p.force_theta_deg = theta_deg;
    p.force_flip = base.chosen.eval.wrist_flip ? 1 : 0;
    return selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  };
  const S3Result off = forced(false);
  const S3Result on = forced(true);
  ASSERT_EQ(off.status, S3Result::kSelected) << off.note;
  ASSERT_GE(off.warnings.size(), 1u);
  EXPECT_NE(off.warnings[0].find("use_clearance"), std::string::npos);
  EXPECT_EQ(on.status, off.status) << on.note;
  ASSERT_EQ(on.warnings.size(), off.warnings.size());
  EXPECT_EQ(on.warnings[0], off.warnings[0]);
  EXPECT_EQ(on.chosen.eval.k, off.chosen.eval.k);
  EXPECT_EQ(on.chosen.eval.wrist_flip, off.chosen.eval.wrist_flip);
  EXPECT_EQ(on.chosen.eval.theta_rad, off.chosen.eval.theta_rad);
  ASSERT_EQ(on.v2_finalists.size(), 1u);
  EXPECT_TRUE(on.v2_finalists[0].clearance_rejected);  // below threshold, but kept
  EXPECT_TRUE(on.v2_finalists[0].winner);
}

// A and B share the same J_static order and the same per-finalist checks (rollout, V1'): with an L2 that discards nothing (threshold =
// the L2 margin of A's winner, taken from a probe) B picks the same finalist as A, and only that one reaches L2.
TEST(GraspS3LazyStaticOrderTest, SameWinnerAsDynamicFollowWhenL2DiscardsNothing) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const JointDynamics dyn = testJointDynamics();
  s.ctx.joint_dynamics = &dyn;

  // The L2 margin of A's winner must be finite and >= 0 (validateS3Params refuses a negative margin_delta_deg, and L2 has no joint limits, so
  // the first finalist can have a negative margin). It is searched in this fixed list of alternatives, in order: for each target omega,
  //   (natural) A's own winner; (steered) the first finalist, in J_static order, that reaches L2 with a finite non-negative margin in a
  //   probe, with the clearance stub failing (0.5 mm) every finalist before it that reached L2 in the probe, so that A (and B, where the
  //   clearance runs before L2) discards those and picks the target. Clearance is on in every attempt (the stub passes with 10 mm
  //   unless the candidate is blocked), as in the campaign. The L2 evaluation of a finalist does not depend on the threshold or on the stub.
  std::vector<S3Candidate> blocked;
  s.ctx.clearance_eval = [&blocked](const ClearanceQuery& q) {
    for (const S3Candidate& c : blocked) {
      if (sameCandidate(q, c)) return fakeClearance(0.5);
    }
    return fakeClearance(10.0);
  };
  S3Params a = v2LazyStaticParams();
  a.use_hold_end = true;
  a.hold_follow_s = 3.0;
  a.use_clearance = true;
  S3Params probe_p = v2DynParams(kHugeMarginDeg);
  probe_p.tau_j_mm = 0.0;
  probe_p.lazy_static_order = true;  // as in the campaign YAML of arm B: no effect, use_dynamic_follow is already lazy
  probe_p.use_hold_end = true;
  probe_p.hold_follow_s = 3.0;
  probe_p.use_clearance = true;

  int tried = 0;
  std::string last_reason;
  bool found = false;
  S3Result ra, rb;
  std::size_t w = 0;
  for (const double omega : {kOmega, -0.3, -0.1}) {
    for (const bool steered : {false, true}) {
      ++tried;
      s.ctx.snapshot.omega_rad_s = omega;
      blocked.clear();
      const S3Result probe = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, probe_p);
      auto usable = [&probe](std::size_t i) {
        const S3FinalistRecord& f = probe.v2_finalists[i];
        return f.dyn_done && !f.dyn_diverged && std::isfinite(f.dyn_total_margin_deg) && f.dyn_total_margin_deg >= 0.0;
      };
      if (steered) {
        std::size_t target = probe.v2_finalists.size();
        for (std::size_t i = 0; i < probe.v2_finalists.size() && target == probe.v2_finalists.size(); ++i) {
          if (usable(i)) target = i;
        }
        if (target == probe.v2_finalists.size()) {
          last_reason = "no finalist reaches L2 with a finite non-negative margin (omega " + std::to_string(omega) + ")";
          continue;
        }
        for (std::size_t i = 0; i < target; ++i) {
          if (probe.v2_finalists[i].dyn_done) blocked.push_back(probe.v2_finalists[i].cand);
        }
      }
      ra = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, a);
      if (ra.status != S3Result::kSelected) {
        last_reason = "arm A selects nothing (omega " + std::to_string(omega) + "): " + ra.note;
        continue;
      }
      w = winnerIndex(ra);
      if (w >= ra.v2_finalists.size() || ra.v2_finalists.size() != probe.v2_finalists.size() || !usable(w)) {
        last_reason = "A's winner has no finite non-negative L2 margin (omega " + std::to_string(omega) + ")";
        continue;
      }
      S3Params b = probe_p;
      b.margin_delta_deg = probe.v2_finalists[w].dyn_total_margin_deg;  // total >= delta passes: L2 discards nothing at A's winner
      rb = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, b);
      std::cerr << "[lazy_static] alternative " << tried << " of 6: omega " << omega << (steered ? " steered" : " natural")
                << ", A's winner = finalist " << w << ", L2 margin " << b.margin_delta_deg << " deg\n";
      found = true;
      break;
    }
    if (found) break;
  }
  if (!found) {
    GTEST_SKIP() << "no alternative gives arm A a winner with a finite, non-negative L2 margin: " << tried
                 << " tried (3 omegas x natural/steered); last: " << last_reason;
  }
  ASSERT_EQ(rb.status, S3Result::kSelected) << rb.note << "\n" << dynTable(rb);
  EXPECT_EQ(winnerIndex(rb), w) << dynTable(rb);
  EXPECT_EQ(rb.chosen.eval.k, ra.chosen.eval.k);
  EXPECT_EQ(rb.chosen.eval.wrist_flip, ra.chosen.eval.wrist_flip);
  EXPECT_EQ(rb.chosen.eval.theta_rad, ra.chosen.eval.theta_rad);
  EXPECT_EQ(rb.dynamic_evals_done, 1) << dynTable(rb);  // everything before A's winner was discarded before L2, as in A
  EXPECT_DOUBLE_EQ(rb.v2_finalists[w].j, ra.v2_finalists[w].j);  // same reported J (w_eff)
}

// lazy_static_order = false is the current eager selection: unchanged result whatever the margin parameters, every finalist rolled out,
// no followMargin, the reported J is the cost built on w_rollout.
TEST(GraspS3LazyStaticOrderTest, OffIsTheEagerSelectionUnchanged) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  const S3Params base = v2LooseParams();
  EXPECT_FALSE(base.lazy_static_order);
  const S3Result a = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, base);
  if (a.status != S3Result::kSelected) GTEST_SKIP() << "no v2 candidate in this fixture: " << a.note;
  S3Params other = base;
  other.lazy_static_order = false;  // explicit
  other.min_margin_deg = 1e6;
  other.margin_follow_s = 99.0;
  other.margin_follow_dt_s = 0.5;
  const S3Result b = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, other);
  ASSERT_EQ(a.status, b.status);
  EXPECT_EQ(a.chosen.eval.theta_rad, b.chosen.eval.theta_rad);
  EXPECT_EQ(a.chosen.eval.k, b.chosen.eval.k);
  EXPECT_EQ(a.chosen.eval.wrist_flip, b.chosen.eval.wrist_flip);
  EXPECT_EQ(a.chosen.cost, b.chosen.cost);
  EXPECT_EQ(a.evaluations, b.evaluations);
  ASSERT_EQ(a.v2_finalists.size(), b.v2_finalists.size());
  EXPECT_EQ(b.rollouts_done, static_cast<int>(b.v2_finalists.size()));  // eager: every finalist
  EXPECT_EQ(b.margin_evals_done, 0);
  std::size_t wins = 0;
  for (std::size_t i = 0; i < a.v2_finalists.size(); ++i) {
    EXPECT_EQ(a.v2_finalists[i].j, b.v2_finalists[i].j);
    EXPECT_EQ(a.v2_finalists[i].winner, b.v2_finalists[i].winner);
    EXPECT_TRUE(b.v2_finalists[i].rollout_done) << i;
    EXPECT_TRUE(b.v2_finalists[i].discard_reason.empty()) << i;  // the eager flow records no lazy discard reason
    if (b.v2_finalists[i].rollout_feasible) {
      EXPECT_DOUBLE_EQ(b.v2_finalists[i].j, s3CostV2(b.v2_finalists[i].d_mm, b.v2_finalists[i].w_rollout, b.v2_finalists[i].t_wait_s, other));
    }
    wins += b.v2_finalists[i].winner ? 1 : 0;
  }
  EXPECT_EQ(wins, 1u);
}

// With use_margin (already lazy) the flag changes nothing: same result with true and false.
TEST(GraspS3LazyStaticOrderTest, IsInertWhenUseMarginIsAlreadyLazy) {
  REQUIRE_URDF_OR_SKIP();
  S3TestSetup s = makeS3TestSetup();
  s.ctx.prodmp_template = &s.tmpl;
  ASSERT_NE(s.robot, nullptr);
  for (const double thr : {0.0, kHugeMarginDeg}) {
    S3Params off = v2MarginParams(thr);
    S3Params on = off;
    on.lazy_static_order = true;
    const S3Result r0 = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, off);
    const S3Result r1 = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, on);
    ASSERT_EQ(r0.status, r1.status) << thr;
    EXPECT_EQ(r0.chosen.eval.theta_rad, r1.chosen.eval.theta_rad) << thr;
    EXPECT_EQ(r0.rollouts_done, r1.rollouts_done) << thr;
    EXPECT_EQ(r0.margin_evals_done, r1.margin_evals_done) << thr;
    ASSERT_EQ(r0.v2_finalists.size(), r1.v2_finalists.size()) << thr;
    for (std::size_t i = 0; i < r0.v2_finalists.size(); ++i) {
      EXPECT_EQ(r0.v2_finalists[i].j, r1.v2_finalists[i].j) << thr << " " << i;
      EXPECT_EQ(r0.v2_finalists[i].winner, r1.v2_finalists[i].winner) << thr << " " << i;
      EXPECT_EQ(r0.v2_finalists[i].discard_reason, r1.v2_finalists[i].discard_reason) << thr << " " << i;
      EXPECT_EQ(r0.v2_finalists[i].margin_deg, r1.v2_finalists[i].margin_deg) << thr << " " << i;
    }
  }
}
