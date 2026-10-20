// Tests for followMargin() (core/follow_margin.hpp), synthetic ProDMP template. Need Pinocchio and the URDF fixture.
// The G16 / step 0c reproduction (0.1 deg) is NOT here: it needs the production weights, see analysis/s3_followmargin_check.cpp.

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <memory>

#include <Eigen/Dense>

#include "satellite_grasp_planner/core/follow_margin.hpp"
#include "satellite_grasp_planner/core/kinematic_feasibility.hpp"
#include "test_fixtures.hpp"

using namespace satellite_grasp_planner::core;
using RobotModel = franka_cartesian_control::core::RobotModel;
namespace tf = satellite_grasp_planner::test_fixtures;

#define REQUIRE_URDF_OR_SKIP()                                                          \
    do {                                                                                \
        if (!std::ifstream(tf::urdfPath()).good()) {                                    \
            GTEST_SKIP() << "URDF fixture not found: " << tf::urdfPath();               \
        }                                                                               \
    } while (0)

namespace {

constexpr double kOmega = -5.0 * M_PI / 180.0;

struct Scene {
    std::shared_ptr<RobotModel> robot = tf::loadPandaRobotModel();
    haptic_dmp_learning::core::ProDMP tmpl = tf::makeSyntheticProDmpTemplate();
    RobotModel::JointVector q0 = RobotModel::readyPose();
    Eigen::Vector3d target;
    Eigen::Quaterniond ori;
    Eigen::Vector3d center;
    Scene() {
        robot->update(q0, RobotModel::JointVector::Zero());
        target = robot->eePosition() + Eigen::Vector3d(0.1, 0.0, 0.0);
        ori = robot->eeOrientation();
        center = target + Eigen::Vector3d(-0.15, 0.0, 0.0);  // satellite axis 15 cm from the target
    }
};

FollowMarginParams params(double follow_s, double dt_follow = 0.01) {
    FollowMarginParams p;
    p.follow_s = follow_s;
    p.dt_follow_s = dt_follow;
    return p;
}

}  // namespace

// follow_s = 0: the same approach simulation as checkKinematicFeasibility (seed = q_ref = q0), so the same minimum margin.
TEST(FollowMarginTest, ZeroFollowEqualsTheRolloutMargin) {
    REQUIRE_URDF_OR_SKIP();
    Scene s;
    ASSERT_NE(s.robot, nullptr);
    const FeasibilityResult fr = checkKinematicFeasibility(s.tmpl, s.target, s.ori, s.robot, s.q0, 0.005, 1e-4, s.q0);
    const FollowMarginResult fm =
        followMargin(s.tmpl, s.target, s.ori, s.center, Eigen::Vector3d::UnitZ(), kOmega, s.robot, s.q0, s.q0, params(0.0));
    EXPECT_DOUBLE_EQ(fm.min_margin_rad, fr.min_joint_margin_rad);
    EXPECT_DOUBLE_EQ(fm.approach_min_margin_rad, fr.min_joint_margin_rad);
    EXPECT_EQ(fm.joint, fr.min_margin_joint);
    EXPECT_LE(fm.t_rel_s, 0.0);
}

// The follow adds samples: the minimum over [start, AT_END + follow_s] cannot exceed the one over [start, AT_END].
TEST(FollowMarginTest, FollowCanOnlyLowerTheMargin) {
    REQUIRE_URDF_OR_SKIP();
    Scene s;
    ASSERT_NE(s.robot, nullptr);
    const FollowMarginResult fm =
        followMargin(s.tmpl, s.target, s.ori, s.center, Eigen::Vector3d::UnitZ(), kOmega, s.robot, s.q0, s.q0, params(3.0));
    EXPECT_LE(fm.min_margin_rad, fm.approach_min_margin_rad);
    ASSERT_GE(fm.joint, 0);
    ASSERT_LE(fm.joint, 6);
    EXPECT_LE(fm.t_rel_s, 3.0 + 1e-9);
    EXPECT_GE(fm.t_rel_s, 0.005 - s.tmpl.tau() - 1e-9);
    // The joint state advances: the follow of a rotating target moves the arm (unless omega is 0).
    EXPECT_GT((fm.q_final - fm.q_at_end).norm(), 0.0);
}

TEST(FollowMarginTest, DeterministicAndDoesNotModifyTheTemplate) {
    REQUIRE_URDF_OR_SKIP();
    Scene s;
    ASSERT_NE(s.robot, nullptr);
    const Eigen::Vector3d goal_before = s.tmpl.goal();
    const FollowMarginResult a =
        followMargin(s.tmpl, s.target, s.ori, s.center, Eigen::Vector3d::UnitZ(), kOmega, s.robot, s.q0, s.q0, params(3.0));
    const FollowMarginResult b =
        followMargin(s.tmpl, s.target, s.ori, s.center, Eigen::Vector3d::UnitZ(), kOmega, s.robot, s.q0, s.q0, params(3.0));
    EXPECT_EQ(a.min_margin_rad, b.min_margin_rad);
    EXPECT_EQ(a.joint, b.joint);
    EXPECT_EQ(a.t_rel_s, b.t_rel_s);
    EXPECT_EQ(s.tmpl.goal(), goal_before);
}

// A start beyond a joint limit gives the exact first-sample margin on that joint (as the checkKinematicFeasibility test).
TEST(FollowMarginTest, StartBeyondJointLimitGivesNegativeMarginOnThatJoint) {
    REQUIRE_URDF_OR_SKIP();
    Scene s;
    ASSERT_NE(s.robot, nullptr);
    RobotModel::JointVector q_seed = s.q0;
    q_seed(3) = -0.02;  // upper limit of joint 4 is -0.0698
    const double excess = q_seed(3) - s.robot->jointUpperLimits()(3);
    ASSERT_GT(excess, 1e-4);
    const FollowMarginResult fm = followMargin(s.tmpl, s.target, s.ori, s.center, Eigen::Vector3d::UnitZ(), kOmega, s.robot,
                                               q_seed, s.q0, params(1.0));
    EXPECT_LE(fm.min_margin_rad, -excess + 1e-12);
    EXPECT_EQ(fm.joint, 3);
}

TEST(FollowMarginTest, InvalidInputsThrow) {
    REQUIRE_URDF_OR_SKIP();
    Scene s;
    ASSERT_NE(s.robot, nullptr);
    const Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
    EXPECT_THROW(followMargin(s.tmpl, s.target, s.ori, s.center, z, kOmega, nullptr, s.q0, s.q0, params(3.0)),
                 std::invalid_argument);
    EXPECT_THROW(followMargin(s.tmpl, s.target, s.ori, s.center, z, kOmega, s.robot, s.q0, s.q0, params(-1.0)),
                 std::invalid_argument);
    EXPECT_THROW(followMargin(s.tmpl, s.target, s.ori, s.center, z, kOmega, s.robot, s.q0, s.q0, params(NAN)),
                 std::invalid_argument);
    EXPECT_THROW(followMargin(s.tmpl, s.target, s.ori, s.center, z, kOmega, s.robot, s.q0, s.q0, params(3.0, 0.0)),
                 std::invalid_argument);
    EXPECT_THROW(followMargin(s.tmpl, s.target, s.ori, s.center, Eigen::Vector3d::UnitX(), kOmega, s.robot, s.q0, s.q0,
                              params(3.0)),
                 std::invalid_argument);
    EXPECT_THROW(followMargin(s.tmpl, s.target, s.ori, s.center, z, NAN, s.robot, s.q0, s.q0, params(3.0)),
                 std::invalid_argument);
}

// Reuse of the rollout's end state: followMargin(..., &state from checkKinematicFeasibility) must equal followMargin(...) that
// re-simulates the approach. Expected EXACT (same operations in the same order: the strategy has no internal state, the
// approach margin is the same min(q - lo, hi - q) over the same q); the margin is checked at 1e-12 rad, the rest exactly.
TEST(FollowMarginTest, ReusingTheRolloutEndStateGivesTheSameResult) {
    REQUIRE_URDF_OR_SKIP();
    Scene s;
    ASSERT_NE(s.robot, nullptr);
    for (const double omega : {kOmega, -2.0 * M_PI / 180.0, 0.0}) {
        const FeasibilityResult fr = checkKinematicFeasibility(s.tmpl, s.target, s.ori, s.robot, s.q0, 0.005, 1e-4, s.q0);
        ASSERT_GE(fr.min_margin_joint, 0);
        ASSERT_GE(fr.min_margin_step, 0);
        FollowApproachState st;
        st.q_end = fr.q_end_post_step;
        st.min_margin_rad = fr.min_joint_margin_rad;
        st.joint = fr.min_margin_joint;
        st.t_rel_s = static_cast<double>(fr.min_margin_step + 1) * 0.005 - s.tmpl.tau();

        const FollowMarginResult full = followMargin(s.tmpl, s.target, s.ori, s.center, Eigen::Vector3d::UnitZ(), omega, s.robot,
                                                     s.q0, s.q0, params(3.0));
        const FollowMarginResult reuse = followMargin(s.tmpl, s.target, s.ori, s.center, Eigen::Vector3d::UnitZ(), omega,
                                                      s.robot, s.q0, s.q0, params(3.0), &st);
        EXPECT_NEAR(reuse.min_margin_rad, full.min_margin_rad, 1e-12) << "omega " << omega;
        EXPECT_EQ(reuse.joint, full.joint) << "omega " << omega;
        EXPECT_EQ(reuse.t_rel_s, full.t_rel_s) << "omega " << omega;
        EXPECT_NEAR(reuse.approach_min_margin_rad, full.approach_min_margin_rad, 1e-12);
        EXPECT_EQ(reuse.q_at_end, full.q_at_end);   // exact
        EXPECT_EQ(reuse.q_final, full.q_final);     // exact
    }
}

// q_end_post_step is the state AFTER the last approach step: it equals the state followMargin() reaches at AT_END and
// differs from q_final (the state BEFORE that step); q_final itself is unchanged.
TEST(FollowMarginTest, RolloutEndStateIsThePostStepState) {
    REQUIRE_URDF_OR_SKIP();
    Scene s;
    ASSERT_NE(s.robot, nullptr);
    const FeasibilityResult fr = checkKinematicFeasibility(s.tmpl, s.target, s.ori, s.robot, s.q0, 0.005, 1e-4, s.q0);
    const FollowMarginResult fm = followMargin(s.tmpl, s.target, s.ori, s.center, Eigen::Vector3d::UnitZ(), kOmega, s.robot,
                                               s.q0, s.q0, params(0.0));
    EXPECT_EQ(fr.q_end_post_step, fm.q_at_end);
    EXPECT_NE(fr.q_end_post_step, fr.q_final);
}
