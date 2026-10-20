// Tests for followMargin() (core/follow_margin.hpp), synthetic ProDMP template. Need Pinocchio and the URDF fixture.
// The G16 / step 0c reproduction (0.1 deg) is NOT here: it needs the production weights, see analysis/s3_followmargin_check.cpp.

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <algorithm>
#include <memory>

#include <Eigen/Dense>

#include "haptic_dmp_learning/core/pose_rotation.hpp"
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
    // A non-vertical axis is a valid input now; only a zero / non-finite axis is rejected.
    EXPECT_THROW(followMargin(s.tmpl, s.target, s.ori, s.center, Eigen::Vector3d::Zero(), kOmega, s.robot, s.q0, s.q0,
                              params(3.0)),
                 std::invalid_argument);
    EXPECT_THROW(followMargin(s.tmpl, s.target, s.ori, s.center, Eigen::Vector3d(0.0, NAN, 1.0), kOmega, s.robot, s.q0,
                              s.q0, params(3.0)),
                 std::invalid_argument);
    EXPECT_NO_THROW(followMargin(s.tmpl, s.target, s.ori, s.center, Eigen::Vector3d::UnitX(), kOmega, s.robot, s.q0, s.q0,
                                 params(1.0)));
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

// ---------------------------------------------------------------------------------------
// Tilted satellite axis
// ---------------------------------------------------------------------------------------
namespace {

using haptic_dmp_learning::core::RotatablePose;

RotatablePose goalPose() {
    RotatablePose g;
    g.position = Eigen::Vector3d(0.62, -0.11, 0.83);
    g.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d(1.0, -2.0, 0.5).normalized()));
    return g;
}

Eigen::Matrix3d rodriguesMatrix(const Eigen::Vector3d& axis, double phi) {
    const Eigen::Vector3d a = axis.normalized();
    Eigen::Matrix3d K;
    K << 0.0, -a.z(), a.y(), a.z(), 0.0, -a.x(), -a.y(), a.x(), 0.0;
    return Eigen::Matrix3d::Identity() + std::sin(phi) * K + (1.0 - std::cos(phi)) * K * K;
}

const Eigen::Vector3d kC(0.70, 0.0, 0.77);

}  // namespace

// The target of the follow for a = +z is, coefficient by coefficient, what the z-only implementation produced:
// rotatePoseAboutZ and the literal feed-forward (-omega r_y, omega r_x, 0), (0, 0, omega).
TEST(FollowTargetSampleTest, ZAxisReproducesTheZOnlyImplementation) {
    const RotatablePose g = goalPose();
    for (const double omega : {kOmega, -2.0 * M_PI / 180.0, 0.3, 0.0}) {
        for (const double t : {0.01, 0.37, 1.0, 3.0}) {
            const RotatablePose cur = haptic_dmp_learning::core::rotatePoseAboutZ(kC, g, omega * t);
            const Eigen::Vector3d r = cur.position - kC;
            const CartesianSample ts = followTargetSample(kC, Eigen::Vector3d::UnitZ(), omega, g, t);
            EXPECT_EQ(ts.position.x(), cur.position.x());
            EXPECT_EQ(ts.position.y(), cur.position.y());
            EXPECT_EQ(ts.position.z(), cur.position.z());
            EXPECT_EQ(ts.orientation.coeffs(), cur.orientation.coeffs());
            EXPECT_EQ(ts.linear_velocity.x(), -omega * r.y());
            EXPECT_EQ(ts.linear_velocity.y(), omega * r.x());
            EXPECT_EQ(ts.linear_velocity.z(), 0.0);
            EXPECT_EQ(ts.angular_velocity.x(), 0.0);
            EXPECT_EQ(ts.angular_velocity.y(), 0.0);
            EXPECT_EQ(ts.angular_velocity.z(), omega);
        }
    }
}

// followMargin with the explicit +z axis equals a reference loop that reproduces the PRE-CHANGE implementation verbatim
// (rotatePoseAboutZ + literal feed-forward + the same strategy / step), starting from the same end-of-approach state.
// q_final, margin, joint and time compared with EXPECT_EQ (exact).
TEST(FollowMarginTest, ExplicitZAxisIsIdenticalToTheLegacyZOnlyLoop) {
    REQUIRE_URDF_OR_SKIP();
    Scene s;
    ASSERT_NE(s.robot, nullptr);
    const FeasibilityResult fr = checkKinematicFeasibility(s.tmpl, s.target, s.ori, s.robot, s.q0, 0.005, 1e-4, s.q0);
    ASSERT_GE(fr.min_margin_joint, 0);
    FollowApproachState st;
    st.q_end = fr.q_end_post_step;
    st.min_margin_rad = fr.min_joint_margin_rad;
    st.joint = fr.min_margin_joint;
    st.t_rel_s = static_cast<double>(fr.min_margin_step + 1) * 0.005 - s.tmpl.tau();

    for (const double omega : {kOmega, -2.0 * M_PI / 180.0, 0.4}) {
        // ---- legacy loop (copy of the removed code) ----
        const NullspaceBiasedResolution strategy{NullspaceBiasedResolution::Params()};
        const RobotModel::JointVector lo = s.robot->jointLowerLimits(), hi = s.robot->jointUpperLimits();
        const RobotModel::JointVector dq_zero = RobotModel::JointVector::Zero();
        double min_all = st.min_margin_rad;
        int joint = st.joint;
        double t_rel = st.t_rel_s;
        RobotModel::JointVector q = st.q_end;
        const RotatablePose goal_at_end{s.target, s.ori};
        const int n_follow = static_cast<int>(std::round(3.0 / 0.01));
        for (int step = 1; step <= n_follow; ++step) {
            const double t_follow = step * 0.01;
            const RotatablePose cur = haptic_dmp_learning::core::rotatePoseAboutZ(s.center, goal_at_end, omega * t_follow);
            CartesianSample target;
            target.position = cur.position;
            target.orientation = cur.orientation;
            const Eigen::Vector3d r = cur.position - s.center;
            target.linear_velocity = Eigen::Vector3d(-omega * r.y(), omega * r.x(), 0.0);
            target.angular_velocity = Eigen::Vector3d(0.0, 0.0, omega);
            s.robot->update(q, dq_zero);
            for (int j = 0; j < RobotModel::kNumJoints; ++j) {
                const double m = std::min(q(j) - lo(j), hi(j) - q(j));
                if (m < min_all) {
                    min_all = m;
                    joint = j;
                    t_rel = t_follow;
                }
            }
            q += strategy.computeJointVelocity(*s.robot, target, q, s.q0) * 0.01;
        }
        // ---- new implementation, explicit +z ----
        const FollowMarginResult fm = followMargin(s.tmpl, s.target, s.ori, s.center, Eigen::Vector3d::UnitZ(), omega, s.robot,
                                                   s.q0, s.q0, params(3.0), &st);
        EXPECT_EQ(fm.q_final, q) << "omega " << omega;
        EXPECT_EQ(fm.min_margin_rad, min_all) << "omega " << omega;
        EXPECT_EQ(fm.joint, joint) << "omega " << omega;
        EXPECT_EQ(fm.t_rel_s, t_rel) << "omega " << omega;
        // an unnormalized +z axis (0,0,3) is the same exact unit vector
        const FollowMarginResult fm3 = followMargin(s.tmpl, s.target, s.ori, s.center, Eigen::Vector3d(0.0, 0.0, 3.0), omega,
                                                    s.robot, s.q0, s.q0, params(3.0), &st);
        EXPECT_EQ(fm3.q_final, fm.q_final);
    }
}

// Tilted axis against Rodrigues and against the numerical derivative of the target.
TEST(FollowTargetSampleTest, TiltedAxisMatchesRodriguesAndTheNumericalDerivative) {
    const RotatablePose g = goalPose();
    const Eigen::Vector3d axis = Eigen::Vector3d(0.35, -0.2, 0.9).normalized();
    const double omega = -5.0 * M_PI / 180.0, h = 1e-4;
    for (const double t : {0.5, 1.5, 3.0}) {
        const CartesianSample ts = followTargetSample(kC, axis, omega, g, t);
        const Eigen::Matrix3d R = rodriguesMatrix(axis, omega * t);
        EXPECT_LT((ts.position - (kC + R * (g.position - kC))).norm(), 1e-12);
        EXPECT_LT((ts.orientation.toRotationMatrix() - R * g.orientation.toRotationMatrix()).norm(), 1e-12);
        const CartesianSample a = followTargetSample(kC, axis, omega, g, t - h);
        const CartesianSample b = followTargetSample(kC, axis, omega, g, t + h);
        EXPECT_LT(((b.position - a.position) / (2.0 * h) - ts.linear_velocity).norm(), 1e-9);
        const Eigen::AngleAxisd aa(b.orientation * a.orientation.conjugate());
        EXPECT_LT((aa.angle() * aa.axis() / (2.0 * h) - ts.angular_velocity).norm(), 1e-9);
        EXPECT_LT((ts.angular_velocity - omega * axis).norm(), 1e-15);
    }
}

// Rigid invariance of the follow target: rotating centre, axis and goal pose by R gives R * (poses and velocities).
TEST(FollowTargetSampleTest, RotatingTheWholeSceneRotatesTheTargetSequence) {
    const RotatablePose g = goalPose();
    const Eigen::Vector3d axis = Eigen::Vector3d(0.35, -0.2, 0.9).normalized();
    const double omega = -5.0 * M_PI / 180.0;
    const Eigen::Matrix3d R = Eigen::AngleAxisd(0.9, Eigen::Vector3d(-0.3, 0.8, 0.4).normalized()).toRotationMatrix();
    const Eigen::Quaterniond qR(R);
    const Eigen::Vector3d shift(0.2, -0.4, 0.1);  // the rigid motion is x -> R x + shift
    RotatablePose g2;
    g2.position = R * g.position + shift;
    g2.orientation = (qR * g.orientation).normalized();
    for (int step = 1; step <= 300; step += 37) {
        const double t = step * 0.01;
        const CartesianSample a = followTargetSample(kC, axis, omega, g, t);
        const CartesianSample b = followTargetSample(R * kC + shift, R * axis, omega, g2, t);
        EXPECT_LT((b.position - (R * a.position + shift)).norm(), 1e-12) << t;
        EXPECT_LT((b.orientation.toRotationMatrix() - R * a.orientation.toRotationMatrix()).norm(), 1e-12) << t;
        EXPECT_LT((b.linear_velocity - R * a.linear_velocity).norm(), 1e-12) << t;
        EXPECT_LT((b.angular_velocity - R * a.angular_velocity).norm(), 1e-12) << t;
    }
}

// The approach does not depend on the axis: with follow_s = 0 a tilted axis gives exactly the +z result; with a follow the
// arm is driven by a different target, so the final joint state differs, and the result is finite and well-formed.
TEST(FollowMarginTest, TiltedAxisRunsAndOnlyTheFollowDependsOnIt) {
    REQUIRE_URDF_OR_SKIP();
    Scene s;
    ASSERT_NE(s.robot, nullptr);
    const Eigen::Vector3d tilted = Eigen::Vector3d(0.3, 0.2, 0.9).normalized();
    const FollowMarginResult z0 =
        followMargin(s.tmpl, s.target, s.ori, s.center, Eigen::Vector3d::UnitZ(), kOmega, s.robot, s.q0, s.q0, params(0.0));
    const FollowMarginResult t0 = followMargin(s.tmpl, s.target, s.ori, s.center, tilted, kOmega, s.robot, s.q0, s.q0, params(0.0));
    EXPECT_EQ(t0.min_margin_rad, z0.min_margin_rad);
    EXPECT_EQ(t0.q_final, z0.q_final);
    const FollowMarginResult z3 =
        followMargin(s.tmpl, s.target, s.ori, s.center, Eigen::Vector3d::UnitZ(), kOmega, s.robot, s.q0, s.q0, params(3.0));
    const FollowMarginResult t3 = followMargin(s.tmpl, s.target, s.ori, s.center, tilted, kOmega, s.robot, s.q0, s.q0, params(3.0));
    EXPECT_TRUE(std::isfinite(t3.min_margin_rad));
    ASSERT_GE(t3.joint, 0);
    ASSERT_LE(t3.joint, 6);
    EXPECT_LE(t3.min_margin_rad, t3.approach_min_margin_rad);
    EXPECT_EQ(t3.q_at_end, z3.q_at_end);
    EXPECT_NE(t3.q_final, z3.q_final);
}
