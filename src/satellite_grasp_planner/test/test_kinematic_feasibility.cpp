// Tests for RobotModel joint limits / readyPose() and checkKinematicFeasibility().
// These need Pinocchio and $HOME/thesis_ws/fer_flat_effort.urdf.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/types.hpp"
#include "satellite_grasp_planner/core/kinematic_feasibility.hpp"
#include "test_fixtures.hpp"

using satellite_grasp_planner::core::checkKinematicFeasibility;
using RobotModel = franka_cartesian_control::core::RobotModel;
using satellite_grasp_planner::test_fixtures::loadPandaRobotModel;
using satellite_grasp_planner::test_fixtures::makeSyntheticProDmpTemplate;
using satellite_grasp_planner::test_fixtures::resetRobotToReady;

namespace {

}  // namespace

TEST(RobotModelLimitsTest, JointLimitsMatchKnownFrankaValuesFromUrdf) {
  auto model = loadPandaRobotModel();
  ASSERT_NE(model, nullptr);
  // Known Franka (FR3) joint limits, 4-decimal rounding.
  const double lo[7] = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
  const double hi[7] = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
  for (int j = 0; j < 7; ++j) {
    EXPECT_NEAR(model->jointLowerLimits()(j), lo[j], 1e-4) << "lower, joint " << j + 1;
    EXPECT_NEAR(model->jointUpperLimits()(j), hi[j], 1e-4) << "upper, joint " << j + 1;
  }
}

TEST(RobotModelReadyPoseTest, MatchesHomingValuesUsedInTools) {
  RobotModel::JointVector expected;
  expected << 0.0, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, 1.5707963267948966,
      0.7853981633974483;
  const RobotModel::JointVector q = RobotModel::readyPose();
  for (int j = 0; j < 7; ++j) EXPECT_DOUBLE_EQ(q(j), expected(j)) << "joint " << j + 1;
  // Also the documented closed form.
  EXPECT_NEAR(q(1), -M_PI / 4.0, 1e-15);
  EXPECT_NEAR(q(3), -3.0 * M_PI / 4.0, 1e-15);
}

TEST(KinematicFeasibilityTest, NearbyTargetFromReadyPoseIsFeasible) {
  auto model = loadPandaRobotModel();
  ASSERT_NE(model, nullptr);
  resetRobotToReady(model);
  const Eigen::Vector3d target = model->eePosition() + Eigen::Vector3d(0.1, 0.0, 0.0);
  const Eigen::Quaterniond quat = model->eeOrientation();

  const auto prodmp = makeSyntheticProDmpTemplate();
  const auto res = checkKinematicFeasibility(prodmp, target, quat, model);

  EXPECT_TRUE(res.feasible);
  EXPECT_LT(res.max_joint_violation_rad, 1e-4);
  // Sanity only (simulator tracking lag is a few mm); it does not decide `feasible`.
  EXPECT_LT(res.pos_err_final_mm, 50.0);
}

// Provable violation, independent of IK convergence: JointPathSimulator::simulate() documents
// output[0].q == q0 (evaluated, not integrated), so a q0 whose joint 4 lies beyond its upper
// limit (-0.0698 rad) makes the very first Step violate that limit by (q0(3) - upper) whatever the
// controller then does. The target equals the FK pose at q0 (tiny motion), so the test does not
// depend on the simulator's tracking behaviour.
TEST(KinematicFeasibilityTest, StartBeyondJointLimitIsInfeasibleWithExactViolation) {
  auto model = loadPandaRobotModel();
  ASSERT_NE(model, nullptr);

  RobotModel::JointVector q0 = RobotModel::readyPose();
  q0(3) = -0.02;  // upper limit of joint 4 is -0.0698 -> 0.0498 rad beyond it
  const double expected_min_violation = q0(3) - model->jointUpperLimits()(3);
  ASSERT_GT(expected_min_violation, 1e-4);

  model->update(q0, RobotModel::JointVector::Zero());
  const Eigen::Vector3d target = model->eePosition() + Eigen::Vector3d(0.01, 0.0, 0.0);
  const Eigen::Quaterniond quat = model->eeOrientation();

  const auto prodmp = makeSyntheticProDmpTemplate();
  const auto res = checkKinematicFeasibility(prodmp, target, quat, model, q0);

  EXPECT_FALSE(res.feasible);
  EXPECT_GE(res.max_joint_violation_rad, expected_min_violation - 1e-12);
}

TEST(KinematicFeasibilityTest, DoesNotModifyProDmpTemplate) {
  auto model = loadPandaRobotModel();
  ASSERT_NE(model, nullptr);
  const auto prodmp = makeSyntheticProDmpTemplate();
  const Eigen::Vector3d goal_before = prodmp.goal();
  const Eigen::Vector3d init_before = prodmp.initPos();
  checkKinematicFeasibility(prodmp, Eigen::Vector3d(0.5, 0.0, 0.5), Eigen::Quaterniond::Identity(),
                            model);
  EXPECT_EQ(prodmp.goal(), goal_before);
  EXPECT_EQ(prodmp.initPos(), init_before);
}

// min_margin_joint (new field): the joint that gives min_joint_margin_rad, set also without violation; the value of
// min_joint_margin_rad is unchanged by it.
TEST(KinematicFeasibilityTest, MinMarginJointIdentifiesTheLimitingJoint) {
  auto model = loadPandaRobotModel();
  ASSERT_NE(model, nullptr);
  const auto prodmp = makeSyntheticProDmpTemplate();

  // Same start as StartBeyondJointLimitIsInfeasibleWithExactViolation: joint 4 (index 3) starts 0.0498 rad beyond its limit.
  RobotModel::JointVector q0 = RobotModel::readyPose();
  q0(3) = -0.02;
  model->update(q0, RobotModel::JointVector::Zero());
  const Eigen::Vector3d target = model->eePosition() + Eigen::Vector3d(0.01, 0.0, 0.0);
  const Eigen::Quaterniond quat = model->eeOrientation();
  const auto bad = checkKinematicFeasibility(prodmp, target, quat, model, q0);
  EXPECT_EQ(bad.min_margin_joint, 3);
  const double excess = q0(3) - model->jointUpperLimits()(3);
  ASSERT_GT(excess, 1e-4);
  EXPECT_LE(bad.min_joint_margin_rad, -excess + 1e-12);

  // No violation: worst_joint stays -1, min_margin_joint is still set, in 0..6, and consistent with the margin.
  resetRobotToReady(model);
  const Eigen::Vector3d t2 = model->eePosition() + Eigen::Vector3d(0.1, 0.0, 0.0);
  const auto ok = checkKinematicFeasibility(prodmp, t2, model->eeOrientation(), model);
  ASSERT_TRUE(ok.feasible);
  EXPECT_EQ(ok.worst_joint, -1);
  ASSERT_GE(ok.min_margin_joint, 0);
  ASSERT_LE(ok.min_margin_joint, 6);
  EXPECT_GT(ok.min_joint_margin_rad, 0.0);
}
