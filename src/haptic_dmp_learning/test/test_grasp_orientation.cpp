// Pure logic of grasp_orientation_mode="planner_goal" (core, no ROS): mode parsing, goal quaternion checks and the
// SLERP profile of the planner's feasibility rollout.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include "haptic_dmp_learning/core/grasp_orientation.hpp"
#include "haptic_dmp_learning/core/math_utils.hpp"

namespace go = haptic_dmp_learning::core::grasp_orientation;
using haptic_dmp_learning::core::radToDeg;
using haptic_dmp_learning::core::slerpShortestPath;

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();
const Eigen::Vector3d kAxis = Eigen::Vector3d::UnitZ();  // satellite rotation axis of the campaign

// Eigen::Quaterniond(w, x, y, z); the values below are written (x, y, z, w) as in ROS / the task text.
Eigen::Quaterniond fromXyzw(double x, double y, double z, double w) { return Eigen::Quaterniond(w, x, y, z); }

/// Same rotation AND same sign (component-wise).
double coeffDiff(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) { return (a.coeffs() - b.coeffs()).norm(); }

/// Nominal grasp of the campaign: k = 3, theta = +115 deg, psi = 0 (goal_pose.orientation, x y z w).
Eigen::Quaterniond nominalGoal() { return fromXyzw(0.690346, 0.153046, 0.153046, 0.690346).normalized(); }

std::string messageOfCheck(const Eigen::Quaterniond& q) {
    try {
        go::checkGoalOrientation(q, kAxis);
    } catch (const std::exception& e) {
        return e.what();
    }
    return "";
}

}  // namespace

// ---- parameter ----------------------------------------------------------------------------------------------

TEST(GraspOrientationMode, ParsesTheTwoValues) {
    EXPECT_EQ(go::parseMode(std::string("planner_goal")), go::Mode::kPlannerGoal);
    EXPECT_EQ(go::parseMode(std::string("demo_native")), go::Mode::kDemoNative);
    EXPECT_STREQ(go::modeName(go::Mode::kPlannerGoal), "planner_goal");
    EXPECT_STREQ(go::modeName(go::Mode::kDemoNative), "demo_native");
}

TEST(GraspOrientationMode, MissingParameterThrowsNamingIt) {
    try {
        go::parseMode(std::nullopt);
        FAIL() << "expected std::invalid_argument";
    } catch (const std::invalid_argument& e) {
        EXPECT_NE(std::string(e.what()).find("grasp_orientation_mode"), std::string::npos);
        EXPECT_NE(std::string(e.what()).find("mandatory"), std::string::npos);
    }
}

TEST(GraspOrientationMode, UnknownValueThrowsNamingParameterAndValue) {
    for (const char* bad : {"", "Planner_Goal", "demo", "slerp", "planner_goal "}) {
        try {
            go::parseMode(std::string(bad));
            FAIL() << "expected std::invalid_argument for '" << bad << "'";
        } catch (const std::invalid_argument& e) {
            EXPECT_NE(std::string(e.what()).find("grasp_orientation_mode"), std::string::npos) << bad;
            EXPECT_NE(std::string(e.what()).find(std::string("'") + bad + "'"), std::string::npos) << bad;
        }
    }
}

// ---- angle --------------------------------------------------------------------------------------------------

TEST(GraspOrientationAngle, InvariantToSignAndKnownValues) {
    const Eigen::Quaterniond id = Eigen::Quaterniond::Identity();
    const Eigen::Quaterniond r90(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
    EXPECT_NEAR(radToDeg(go::quatAngleRad(id, r90)), 90.0, 1e-9);
    EXPECT_NEAR(radToDeg(go::quatAngleRad(id, Eigen::Quaterniond(-r90.coeffs()))), 90.0, 1e-9);
    EXPECT_NEAR(go::quatAngleRad(Eigen::Quaterniond(-id.coeffs()), id), 0.0, 1e-6);
    EXPECT_THROW(go::quatAngleRad(Eigen::Quaterniond(0, 0, 0, 0), id), std::invalid_argument);
}

// ---- goal quaternion of the message -------------------------------------------------------------------------

TEST(GraspOrientationCheck, NominalGraspIsAccepted) {
    EXPECT_EQ(messageOfCheck(nominalGoal()), "");
    // Its approach axis is horizontal: z_tcp . z = 1 - 2 (x^2 + y^2) ~ 0.
    const Eigen::Vector3d z_tcp = nominalGoal().toRotationMatrix().col(2);
    EXPECT_NEAR(z_tcp.dot(kAxis), 0.0, 1e-5);
}

TEST(GraspOrientationCheck, NonFiniteIsRejected) {
    for (int i = 0; i < 4; ++i) {
        Eigen::Quaterniond q = nominalGoal();
        q.coeffs()(i) = kNaN;
        const std::string m = messageOfCheck(q);
        EXPECT_NE(m.find("GraspCommand rejected"), std::string::npos) << i;
        EXPECT_NE(m.find("non-finite"), std::string::npos) << i;
    }
    Eigen::Quaterniond inf = nominalGoal();
    inf.coeffs()(0) = std::numeric_limits<double>::infinity();
    EXPECT_NE(messageOfCheck(inf).find("non-finite"), std::string::npos);
}

TEST(GraspOrientationCheck, NormMustBeOneWithinOneMillionth) {
    const Eigen::Quaterniond q = nominalGoal();
    EXPECT_EQ(messageOfCheck(Eigen::Quaterniond(q.coeffs() * (1.0 + 5e-7))), "");
    EXPECT_EQ(messageOfCheck(Eigen::Quaterniond(q.coeffs() * (1.0 - 5e-7))), "");
    for (double f : {1.0 + 2e-6, 1.0 - 2e-6, 1.001, 0.0}) {
        const std::string m = messageOfCheck(Eigen::Quaterniond(q.coeffs() * f));
        EXPECT_NE(m.find("GraspCommand rejected"), std::string::npos) << f;
        EXPECT_NE(m.find("norm"), std::string::npos) << f;
    }
}

TEST(GraspOrientationCheck, ApproachAxisMustBeOrthogonalToTheRotationAxis) {
    // Rotation by angle a about x: z_tcp = (0, -sin a, cos a), so z_tcp . axis(z) = cos a.
    auto rotX = [](double dot) { return Eigen::Quaterniond(Eigen::AngleAxisd(std::acos(dot), Eigen::Vector3d::UnitX())); };
    EXPECT_EQ(messageOfCheck(rotX(5e-4)), "");
    EXPECT_EQ(messageOfCheck(rotX(-5e-4)), "");
    for (double d : {2e-3, -2e-3, 0.5, 1.0}) {
        const std::string m = messageOfCheck(rotX(d));
        EXPECT_NE(m.find("GraspCommand rejected"), std::string::npos) << d;
        EXPECT_NE(m.find("orthogonal"), std::string::npos) << d;
    }
    // Identity: z_tcp = z = the rotation axis itself.
    EXPECT_NE(messageOfCheck(Eigen::Quaterniond::Identity()).find("orthogonal"), std::string::npos);
    // The axis is normalized before the dot product.
    EXPECT_NO_THROW(go::checkGoalOrientation(nominalGoal(), Eigen::Vector3d(0.0, 0.0, 7.0)));
    EXPECT_THROW(go::checkGoalOrientation(nominalGoal(), Eigen::Vector3d::Zero()), std::invalid_argument);
}

// ---- s(t) = the planner's -------------------------------------------------------------------------------------

TEST(GraspOrientationSlerp, FractionIsMinOneOfTimeOverTauFlooredAtZero) {
    EXPECT_DOUBLE_EQ(go::slerpFraction(0.0, 4.6), 0.0);
    EXPECT_DOUBLE_EQ(go::slerpFraction(2.3, 4.6), 0.5);
    EXPECT_DOUBLE_EQ(go::slerpFraction(4.6, 4.6), 1.0);
    EXPECT_DOUBLE_EQ(go::slerpFraction(9.0, 4.6), 1.0);
    EXPECT_DOUBLE_EQ(go::slerpFraction(-0.1, 4.6), 0.0);
    EXPECT_THROW(go::slerpFraction(1.0, 0.0), std::invalid_argument);
    EXPECT_THROW(go::slerpFraction(1.0, -1.0), std::invalid_argument);
    EXPECT_THROW(go::slerpFraction(kNaN, 4.6), std::invalid_argument);
}

// Numerical equivalence with kinematic_feasibility.cpp (checkKinematicFeasibility): the planner evaluates, for
// sample i, s = std::min(1.0, static_cast<double>(i + 1) * dt / tau) and
// haptic_dmp_learning::core::slerpShortestPath(quat0, target_orientation, s). The executor's function must give the
// identical result for t = (i + 1) * dt. slerpShortestPath is the very same function (math_utils.hpp): there is no
// second implementation to drift from.
TEST(GraspOrientationSlerp, IdenticalToThePlannersFeasibilityRolloutExpression) {
    const double dt = 0.005;
    const double tau = 4.6;
    const int n_steps = static_cast<int>(std::ceil(tau / dt));
    const Eigen::Quaterniond quat0 = fromXyzw(0.999956, -0.000072, -0.009383, -0.000149).normalized();  // Z18-like hold
    const Eigen::Quaterniond target = nominalGoal();
    for (int i = 0; i < n_steps; ++i) {
        const double s_planner = std::min(1.0, static_cast<double>(i + 1) * dt / tau);
        const Eigen::Quaterniond q_planner = slerpShortestPath(quat0, target, s_planner);
        const Eigen::Quaterniond q_exec = go::slerpAt(quat0, target, static_cast<double>(i + 1) * dt, tau);
        ASSERT_EQ(go::slerpFraction(static_cast<double>(i + 1) * dt, tau), s_planner) << i;
        ASSERT_EQ(q_exec.coeffs(), q_planner.coeffs()) << i;
    }
}

// ---- the profile ---------------------------------------------------------------------------------------------

TEST(GraspOrientationSlerp, StartsAtTheHoldEndsAtTheGoalAndHalfwayIsHalfTheAngle) {
    const Eigen::Quaterniond hold = Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0);  // (x, y, z, w) = (1, 0, 0, 0)
    const Eigen::Quaterniond goal = nominalGoal();
    const double tau = 4.6;
    const Eigen::Quaterniond goal_al = go::alignShortestPath(hold, goal);

    EXPECT_LT(coeffDiff(go::slerpAt(hold, goal_al, 0.0, tau), hold), 1e-12);            // s = 0: the hold
    EXPECT_LT(coeffDiff(go::slerpAt(hold, goal_al, tau, tau), goal_al), 1e-12);         // s = 1: the goal
    EXPECT_LT(coeffDiff(go::slerpAt(hold, goal_al, 3.0 * tau, tau), goal_al), 1e-12);   // beyond tau: stays at the goal

    const double total = go::quatAngleRad(hold, goal);
    const Eigen::Quaterniond mid = go::slerpAt(hold, goal_al, 0.5 * tau, tau);
    EXPECT_NEAR(go::quatAngleRad(hold, mid), 0.5 * total, 1e-9);
    EXPECT_NEAR(go::quatAngleRad(mid, goal), 0.5 * total, 1e-9);
    EXPECT_NEAR(mid.norm(), 1.0, 1e-12);
    // 2 acos(0.690346) ~ 92.7 deg for the nominal grasp seen from a hold pointing down (x = 1).
    EXPECT_NEAR(radToDeg(total), 92.7, 0.1);
}

TEST(GraspOrientationSlerp, NominalGraspKnownQuaternionFromAHoldOfOneZeroZeroZero) {
    // k = 3, theta = 115 deg, psi = 0: goal (x, y, z, w) = (0.690346, 0.153046, 0.153046, 0.690346), hold (1, 0, 0, 0).
    const Eigen::Quaterniond hold = fromXyzw(1.0, 0.0, 0.0, 0.0);
    const Eigen::Quaterniond goal = fromXyzw(0.690346, 0.153046, 0.153046, 0.690346);
    EXPECT_EQ(hold.coeffs().dot(goal.coeffs()) > 0.0, true);  // already the same hemisphere: no flip
    const Eigen::Quaterniond end = go::slerpAt(hold, go::alignShortestPath(hold, goal), 4.6, 4.6);
    EXPECT_NEAR(end.x(), 0.690346, 1e-5);
    EXPECT_NEAR(end.y(), 0.153046, 1e-5);
    EXPECT_NEAR(end.z(), 0.153046, 1e-5);
    EXPECT_NEAR(end.w(), 0.690346, 1e-5);
}

TEST(GraspOrientationSlerp, QAndMinusQAreEquivalentAndTakeTheShortestPath) {
    const Eigen::Quaterniond hold = fromXyzw(0.999956, -0.000072, -0.009383, -0.000149).normalized();
    const Eigen::Quaterniond goal = nominalGoal();
    const Eigen::Quaterniond neg(-goal.coeffs());
    const Eigen::Quaterniond a = go::alignShortestPath(hold, goal);
    const Eigen::Quaterniond b = go::alignShortestPath(hold, neg);
    EXPECT_LT(coeffDiff(a, b), 1e-15);  // both land in the hold's hemisphere
    EXPECT_GE(hold.coeffs().dot(a.coeffs()), 0.0);
    for (double t : {0.0, 1.0, 2.3, 4.0, 4.6}) {
        EXPECT_LT(coeffDiff(go::slerpAt(hold, a, t, 4.6), go::slerpAt(hold, b, t, 4.6)), 1e-14) << t;
    }
    // Hold given with the opposite sign: same rotations along the path.
    const Eigen::Quaterniond hold_neg(-hold.coeffs());
    const Eigen::Quaterniond c = go::alignShortestPath(hold_neg, goal);
    for (double t : {1.0, 2.3, 4.6}) {
        EXPECT_NEAR(go::quatAngleRad(go::slerpAt(hold_neg, c, t, 4.6), go::slerpAt(hold, a, t, 4.6)), 0.0, 1e-6) << t;
    }
    // The path never exceeds the shortest angle.
    for (double t = 0.0; t <= 4.6; t += 0.23) {
        EXPECT_LE(go::quatAngleRad(hold, go::slerpAt(hold, a, t, 4.6)), M_PI + 1e-12);
    }
    EXPECT_LE(go::quatAngleRad(hold, goal), M_PI);
}

TEST(GraspOrientationSlerp, DegenerateZeroAngle) {
    const Eigen::Quaterniond q = nominalGoal();
    for (const Eigen::Quaterniond& goal : {q, Eigen::Quaterniond(-q.coeffs())}) {
        const Eigen::Quaterniond al = go::alignShortestPath(q, goal);
        for (double t : {0.0, 2.3, 4.6}) {
            const Eigen::Quaterniond r = go::slerpAt(q, al, t, 4.6);
            EXPECT_LT(coeffDiff(r, q), 1e-12) << t;
            EXPECT_TRUE(r.coeffs().allFinite());
        }
        EXPECT_LT(go::slerpAngularVelocityWorld(q, al, 4.6).norm(), 1e-9);
    }
}

TEST(GraspOrientationSlerp, DegenerateHalfTurn) {
    const Eigen::Quaterniond id = Eigen::Quaterniond::Identity();
    const Eigen::Quaterniond flip = fromXyzw(1.0, 0.0, 0.0, 0.0);  // 180 deg about x
    const Eigen::Quaterniond al = go::alignShortestPath(id, flip);
    EXPECT_NEAR(go::quatAngleRad(id, flip), M_PI, 1e-12);
    EXPECT_LT(coeffDiff(go::slerpAt(id, al, 0.0, 4.6), id), 1e-12);
    EXPECT_LT(coeffDiff(go::slerpAt(id, al, 4.6, 4.6), al), 1e-12);
    const Eigen::Quaterniond mid = go::slerpAt(id, al, 2.3, 4.6);
    EXPECT_TRUE(mid.coeffs().allFinite());
    EXPECT_NEAR(mid.norm(), 1.0, 1e-12);
    EXPECT_NEAR(go::quatAngleRad(id, mid), M_PI / 2.0, 1e-9);
    // 180 deg about x over 4.6 s: 0.6830 rad/s about +x (either sign is a valid shortest path; only |omega| is fixed).
    EXPECT_NEAR(go::slerpAngularVelocityWorld(id, al, 4.6).norm(), M_PI / 4.6, 1e-9);
}

// ---- angular velocity of the profile (twist feed-forward) --------------------------------------------------------

TEST(GraspOrientationSlerp, AngularVelocityIsConstantAndMatchesTheFiniteDifference) {
    const Eigen::Quaterniond hold = fromXyzw(0.999956, -0.000072, -0.009383, -0.000149).normalized();
    const Eigen::Quaterniond al = go::alignShortestPath(hold, nominalGoal());
    const double tau = 4.6;
    const Eigen::Vector3d omega = go::slerpAngularVelocityWorld(hold, al, tau);
    EXPECT_NEAR(omega.norm(), go::quatAngleRad(hold, al) / tau, 1e-9);
    const double h = 1e-6;
    for (double t : {0.5, 1.7, 3.1, 4.4}) {
        const Eigen::Quaterniond q0 = go::slerpAt(hold, al, t, tau);
        const Eigen::Quaterniond q1 = go::slerpAt(hold, al, t + h, tau);
        // World-frame increment: q1 = dq * q0, rotation vector of dq / h.
        Eigen::Quaterniond dq = (q1 * q0.conjugate()).normalized();
        if (dq.w() < 0.0) dq.coeffs() = -dq.coeffs();
        const Eigen::AngleAxisd aa(dq);
        const Eigen::Vector3d fd = aa.angle() * aa.axis() / h;
        EXPECT_LT((fd - omega).norm(), 1e-5) << t;
    }
}

TEST(GraspOrientationSlerp, AngularVelocityRejectsInvalidTau) {
    EXPECT_THROW(go::slerpAngularVelocityWorld(Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity(), 0.0),
                 std::invalid_argument);
}
