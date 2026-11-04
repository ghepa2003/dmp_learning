// Tests of the per-candidate log (S3Context::eval_log), the psi-rolled evaluation, the full manipulability and the
// follow-arc helpers used by analysis/s3_landscape.cpp. The selector tests need Pinocchio and
// $HOME/thesis_ws/fer_flat_effort.urdf (skipped if missing), like test_grasp_s3_selection.cpp.

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <memory>

#include <Eigen/Dense>

#include "s3_landscape_util.hpp"
#include "test_fixtures.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "satellite_grasp_planner/core/grasp_pose_eval.hpp"
#include "satellite_grasp_planner/core/grasp_s3_selection.hpp"
#include "satellite_grasp_planner/core/manipulability.hpp"

using namespace satellite_grasp_planner::core;
using haptic_dmp_learning::core::CubeSatelliteModel;
using haptic_dmp_learning::core::GraspPointId;
using haptic_dmp_learning::core::ProDMP;
using haptic_dmp_learning::core::RotatablePose;
using RobotModel = franka_cartesian_control::core::RobotModel;
namespace tf = satellite_grasp_planner::test_fixtures;

#define REQUIRE_URDF_OR_SKIP()                                      \
  do {                                                              \
    if (!std::ifstream(tf::urdfPath()).good()) {                    \
      GTEST_SKIP() << "URDF fixture not found: " << tf::urdfPath(); \
    }                                                               \
  } while (0)

namespace {

CubeSatelliteModel makeModel(const Eigen::Vector3d& center) {
  CubeSatelliteModel::Params p;
  p.center_world = center;
  p.axis_world = Eigen::Vector3d::UnitZ();
  p.face_normal_body = Eigen::Vector3d::UnitX();
  p.grasp_radius_m = 0.068;
  return CubeSatelliteModel(p);
}

struct LandscapeFixture {
  std::shared_ptr<RobotModel> robot;
  RobotModel::JointVector q_start;
  DemoReference demo;
  ProDMP tmpl;
  Eigen::Vector3d delta_g = Eigen::Vector3d(0.1, 0.0, 0.0);
  CubeSatelliteModel model;
  S3Context ctx;
};

LandscapeFixture makeLandscapeFixture() {
  LandscapeFixture s{tf::loadPandaRobotModel(), RobotModel::JointVector::Zero(), DemoReference(), tf::makeSyntheticProDmpTemplate(),
                     Eigen::Vector3d(0.1, 0.0, 0.0), makeModel(Eigen::Vector3d::Zero()), S3Context()};
  s.q_start << 0.0, -0.7854, 0.0, -2.3562, 0.0, 3.1416, 0.7854;
  s.demo = demoReferenceFromEndJoints(*s.robot, s.q_start);
  const Eigen::Vector3d center = s.demo.position + Eigen::Vector3d(0.05, 0.0, 0.05) - Eigen::Vector3d(0.155, -0.068, 0.0);
  s.model = makeModel(center);
  s.ctx.tau_launch_s = s.tmpl.tau();
  s.ctx.snapshot.center = center;
  s.ctx.snapshot.omega_rad_s = -2.0 * M_PI / 180.0;
  s.ctx.snapshot.theta_rad = 0.3;
  s.ctx.t_now_s = 10.0;
  s.ctx.min_delay_s = 0.5;
  return s;
}

S3Params looseParams() {
  S3Params p;
  const RobotModel::JointVector q = RobotModel::readyPose();  // mandatory nullspace target of the rollout
  p.rollout_q_ref.assign(q.data(), q.data() + 7);
  p.r_trust_m = 100.0;
  p.theta_step_deg = 5.0;
  p.posthoc_velocity_check = false;
  return p;
}

RobotModel::Jacobian6x7 syntheticJacobian() {
  RobotModel::Jacobian6x7 J = RobotModel::Jacobian6x7::Zero();
  J(0, 0) = 2.0;
  J(1, 1) = 3.0;
  J(2, 2) = 4.0;  // linear block
  J(3, 3) = 0.5;
  J(4, 4) = 1.0;
  J(5, 5) = 2.0;  // angular block
  return J;
}

}  // namespace

TEST(S3LandscapeLog, NullAndNonNullEvalLogGiveTheSameResult) {
  REQUIRE_URDF_OR_SKIP();
  LandscapeFixture s = makeLandscapeFixture();
  s.ctx.prodmp_template = &s.tmpl;
  const S3Params p = looseParams();
  const S3Result a = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  std::vector<S3EvalRecord> log;
  S3Context ctx2 = s.ctx;
  ctx2.eval_log = &log;
  const S3Result b = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, ctx2, p);
  EXPECT_EQ(a.status, b.status);
  EXPECT_EQ(a.evaluations, b.evaluations);
  ASSERT_EQ(a.ranked.size(), b.ranked.size());
  for (std::size_t i = 0; i < a.ranked.size(); ++i) {
    EXPECT_EQ(a.ranked[i].eval.k, b.ranked[i].eval.k);
    EXPECT_EQ(a.ranked[i].eval.theta_rad, b.ranked[i].eval.theta_rad);
    EXPECT_EQ(a.ranked[i].cost, b.ranked[i].cost);
    EXPECT_EQ(a.ranked[i].d_pos_m, b.ranked[i].d_pos_m);
  }
  if (a.status == S3Result::kSelected) {
    EXPECT_EQ(a.chosen.eval.k, b.chosen.eval.k);
    EXPECT_EQ(a.chosen.eval.theta_rad, b.chosen.eval.theta_rad);
    EXPECT_EQ(a.chosen.cost, b.chosen.cost);
  }
}

TEST(S3LandscapeLog, OneRowPerEvaluationAndConsistentWithTheCandidates) {
  REQUIRE_URDF_OR_SKIP();
  LandscapeFixture s = makeLandscapeFixture();
  s.ctx.prodmp_template = &s.tmpl;
  std::vector<S3EvalRecord> log;
  s.ctx.eval_log = &log;
  const S3Params p = looseParams();
  const S3Result r = selectGraspS3(s.model, *s.robot, s.q_start, s.demo, s.delta_g, s.ctx, p);
  EXPECT_EQ(static_cast<int>(log.size()), r.evaluations);
  int passing = 0;
  for (const auto& l : log) {
    EXPECT_EQ(l.passes_constraints, l.reject_mask == 0u);
    if (l.passes_constraints) ++passing;
    EXPECT_EQ(l.j_static, s3CostV2(l.d * p.length_scale_m * 1000.0, l.branch[l.wrist_flip ? 1 : 0].w_full, 0.0, p));
    EXPECT_GE(l.wait_s, s.ctx.min_delay_s);  // wait is logged for every row
  }
  EXPECT_EQ(static_cast<int>(log.size()), 4 * 72);  // 360 / 5 deg per k, a single grid
  if (r.status != S3Result::kNoCandidate) {
    EXPECT_GE(passing, static_cast<int>(r.ranked.size()));
  }
}

TEST(S3LandscapeLog, RollPsiZeroIsBitIdenticalAndKeepsTheTcpPosition) {
  REQUIRE_URDF_OR_SKIP();
  LandscapeFixture s = makeLandscapeFixture();
  const IkParams ik;
  const auto k = GraspPointId::kP0;
  const double theta = 0.2;
  const PoseEvaluation base = evaluateGraspPose(s.model, k, theta, *s.robot, s.q_start, s.demo, 0.1, 1.0, ik);
  const PoseEvaluationDetail d0 = evaluateGraspPoseDetailed(s.model, k, theta, 0.0, *s.robot, s.q_start, s.demo, 0.1, 1.0, ik, true);
  EXPECT_EQ(base.w_hat, d0.eval.w_hat);
  EXPECT_EQ(base.margins.min_margin_rad, d0.eval.margins.min_margin_rad);
  EXPECT_EQ(base.wrist_flip, d0.eval.wrist_flip);
  EXPECT_EQ((base.ik.q - d0.eval.ik.q).norm(), 0.0);
  const PoseEvaluationDetail d5 = evaluateGraspPoseDetailed(s.model, k, theta, 5.0 * M_PI / 180.0, *s.robot, s.q_start,
                                                            s.demo, 0.1, 1.0, ik, true);
  EXPECT_EQ((d5.eval.pose.position_world - base.pose.position_world).norm(), 0.0);
  // the roll is about the approach axis: the approach axis column of the target orientation is unchanged
  const Eigen::Matrix3d R0 = d0.branch[0].orientation.toRotationMatrix();
  const Eigen::Matrix3d R5 = d5.branch[0].orientation.toRotationMatrix();
  EXPECT_NEAR((R0.col(2) - R5.col(2)).norm(), 0.0, 1e-12);
  EXPECT_NEAR((R0.col(2) - base.pose.approach_axis_world).norm(), 0.0, 1e-12);
  EXPECT_NEAR(std::acos(std::min(1.0, R0.col(0).dot(R5.col(0)))), 5.0 * M_PI / 180.0, 1e-9);
}

TEST(S3LandscapeLog, FullManipulabilityOfASyntheticJacobian) {
  const FullManipulability m = fullManipulability(syntheticJacobian());
  EXPECT_NEAR(m.w_trans, 24.0, 1e-12);  // 2*3*4
  EXPECT_NEAR(m.w_rot, 1.0, 1e-12);     // 0.5*1*2
  EXPECT_NEAR(m.w_full, 24.0, 1e-12);   // blocks on disjoint columns: product of all six
  EXPECT_NEAR(m.smin_trans, 2.0, 1e-12);
  EXPECT_NEAR(m.smin_rot, 0.5, 1e-12);
  EXPECT_NEAR(m.w_trans, wTransFromJacobian(syntheticJacobian().topRows<3>()), 1e-12);
}

TEST(S3LandscapeLog, ScalingAngularRowsByCChangesLnWFullBy3LnC) {
  RobotModel::Jacobian6x7 J;
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 7; ++j) J(i, j) = std::sin(1.3 * i + 0.7 * j + 0.2) + 0.1 * (i == j);  // coupled, full rank
  }
  const double c = 2.5;
  RobotModel::Jacobian6x7 Jc = J;
  Jc.bottomRows<3>() *= c;
  const double w = fullManipulability(J).w_full, wc = fullManipulability(Jc).w_full;
  ASSERT_GT(w, 1e-9);
  EXPECT_NEAR(std::log(wc) - std::log(w), 3.0 * std::log(c), 1e-9);
  EXPECT_NEAR(fullManipulability(Jc).w_rot / fullManipulability(J).w_rot, c * c * c, 1e-9);
  EXPECT_NEAR(fullManipulability(Jc).w_trans, fullManipulability(J).w_trans, 1e-12);
}

TEST(S3LandscapeLog, FollowArcMatchesRotatePoseAboutZ) {
  const Eigen::Vector3d center(1.0, 0.0, 0.5), axis = Eigen::Vector3d::UnitZ();
  RotatablePose pose;
  pose.position = Eigen::Vector3d(2.0, 0.0, 0.7);
  pose.orientation = Eigen::Quaterniond::Identity();
  // known case: omega = pi/2 rad/s, t = 1 s -> +90 deg about z through the centre: (2,0) -> (1,1)
  const RotatablePose r = haptic_dmp_learning::core::rotatePoseAboutAxis(center, axis, pose, M_PI / 2.0 * 1.0);
  EXPECT_NEAR((r.position - Eigen::Vector3d(1.0, 1.0, 0.7)).norm(), 0.0, 1e-12);
  EXPECT_NEAR(r.orientation.angularDistance(Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()))), 0.0, 1e-12);
  // identical to the executor's function for a generic case
  const double omega = -2.0 * M_PI / 180.0, t = 2.0;
  const RotatablePose a = haptic_dmp_learning::core::rotatePoseAboutAxis(center, axis, pose, omega * t);
  const RotatablePose b = haptic_dmp_learning::core::rotatePoseAboutZ(center, pose, omega * t);
  EXPECT_EQ((a.position - b.position).norm(), 0.0);
  EXPECT_EQ(a.orientation.angularDistance(b.orientation), 0.0);
  // a non-vertical axis is accepted (rotatePoseAboutAxis); only a zero axis is rejected
  EXPECT_NO_THROW(haptic_dmp_learning::core::rotatePoseAboutAxis(center, Eigen::Vector3d::UnitX(), pose, omega * t));
  EXPECT_THROW(haptic_dmp_learning::core::rotatePoseAboutAxis(center, Eigen::Vector3d::Zero(), pose, omega * t), std::invalid_argument);
  const auto ts = s3_landscape::followSampleTimes(3.0, 7);
  ASSERT_EQ(ts.size(), 7u);
  EXPECT_DOUBLE_EQ(ts.front(), 0.0);
  EXPECT_DOUBLE_EQ(ts[1], 0.5);
  EXPECT_DOUBLE_EQ(ts.back(), 3.0);
  EXPECT_THROW(s3_landscape::followSampleTimes(3.0, 1), std::invalid_argument);
}

// Axis, satellite_q_ref and face normal for a tilted axis: the values are the ones tools/s1_run.py wrote in the cell runs
// (results/val_C5, val_C6, val_C11: run_meta.json "axis"/"q_ref", planner_params_passed cube_geometry.face_normal_body).
// val_C5: alpha 20, phi 0; val_C6: alpha 20, phi 90; val_C11: alpha 15, phi 315.
TEST(S3LandscapeAxisTilt, MatchesTheValuesOfTheCellRuns) {
  struct Case {
    double alpha, phi;
    Eigen::Vector3d axis, normal;
    Eigen::Vector4d q_xyzw;
  };
  const Case cases[] = {
      {20.0, 0.0, {0.3420201433256687, 0.0, 0.9396926207859084}, {0.9396926207859084, 0.0, -0.3420201433256687},
       {-0.0, 0.17364817766693033, 0.0, 0.984807753012208}},
      {20.0, 90.0, {2.094269368838496e-17, 0.3420201433256687, 0.9396926207859084}, {1.0, -3.692761945975143e-18, -2.094269368838496e-17},
       {-0.17364817766693033, 1.0632884247878856e-17, 0.0, 0.984807753012208}},
      {15.0, 315.0, {0.18301270189221927, -0.18301270189221935, 0.9659258262890683}, {0.9829629131445342, 0.017037086855465844, -0.18301270189221927},
       {0.09229595564125727, 0.09229595564125723, 0.0, 0.9914448613738104}},
  };
  for (const Case& c : cases) {
    const s3_landscape::AxisTilt t = s3_landscape::axisTilt(c.alpha, c.phi);
    EXPECT_LT((t.axis - c.axis).norm(), 1e-12) << c.alpha << " " << c.phi;
    EXPECT_LT((t.face_normal - c.normal).norm(), 1e-12) << c.alpha << " " << c.phi;
    EXPECT_LT((Eigen::Vector4d(t.q_ref.x(), t.q_ref.y(), t.q_ref.z(), t.q_ref.w()) - c.q_xyzw).norm(), 1e-12) << c.alpha << " " << c.phi;
    EXPECT_NEAR(t.axis.norm(), 1.0, 1e-14);
    EXPECT_NEAR(t.face_normal.dot(t.axis), 0.0, 1e-12);  // the selector's cube model requires <= 1e-6
  }
}

TEST(S3LandscapeAxisTilt, ZeroTiltIsTheVerticalAxisAndRangeErrorsThrow) {
  const s3_landscape::AxisTilt t = s3_landscape::axisTilt(0.0, 123.0);
  EXPECT_LT((t.axis - Eigen::Vector3d::UnitZ()).norm(), 1e-15);
  EXPECT_LT((t.face_normal - Eigen::Vector3d::UnitX()).norm(), 1e-15);
  EXPECT_NEAR(t.q_ref.angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-15);
  EXPECT_THROW(s3_landscape::axisTilt(20.5, 0.0), std::invalid_argument);
  EXPECT_THROW(s3_landscape::axisTilt(-1.0, 0.0), std::invalid_argument);
  EXPECT_THROW(s3_landscape::axisTilt(10.0, 360.0), std::invalid_argument);
}
