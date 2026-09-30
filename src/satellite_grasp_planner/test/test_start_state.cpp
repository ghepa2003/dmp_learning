// Tests of the planner's start configuration (start_joints_override_rad): validation and distance report.

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "satellite_grasp_planner/core/start_state.hpp"
#include "test_fixtures.hpp"

using namespace satellite_grasp_planner::core;
using RobotModel = franka_cartesian_control::core::RobotModel;
using satellite_grasp_planner::test_fixtures::loadPandaRobotModel;
using satellite_grasp_planner::test_fixtures::urdfPath;

namespace {

RobotModel::JointVector lower() {
    RobotModel::JointVector l;
    l << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
    return l;
}
RobotModel::JointVector upper() {
    RobotModel::JointVector u;
    u << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
    return u;
}
std::vector<double> ready() {
    const RobotModel::JointVector q = RobotModel::readyPose();
    return std::vector<double>(q.data(), q.data() + 7);
}

std::string messageOf(const std::vector<double>& v) {
    try {
        validateStartJoints(v, lower(), upper());
    } catch (const std::invalid_argument& e) {
        return e.what();
    }
    return "";
}

}  // namespace

TEST(StartState, ReadyPoseIsValid) {
    EXPECT_NO_THROW(validateStartJoints(ready(), lower(), upper()));
}

TEST(StartState, LimitsAreInclusive) {
    std::vector<double> v = ready();
    v[0] = 2.8973;
    v[3] = -0.0698;
    EXPECT_NO_THROW(validateStartJoints(v, lower(), upper()));
}

TEST(StartState, WrongSizeThrowsNamingTheParameter) {
    for (std::size_t n : {std::size_t(0), std::size_t(6), std::size_t(8)}) {
        std::vector<double> v(n, 0.0);
        const std::string m = messageOf(v);
        EXPECT_NE(m.find("start_joints_override_rad"), std::string::npos) << n;
        EXPECT_NE(m.find("exactly 7"), std::string::npos) << n;
    }
}

TEST(StartState, NonFiniteThrowsNamingTheJoint) {
    std::vector<double> v = ready();
    v[4] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_NE(messageOf(v).find("fer_joint5"), std::string::npos);
    v = ready();
    v[1] = std::numeric_limits<double>::infinity();
    EXPECT_NE(messageOf(v).find("fer_joint2"), std::string::npos);
}

TEST(StartState, OutsideLimitsThrowsNamingJointAndLimits) {
    std::vector<double> v = ready();
    v[3] = 0.5;  // fer_joint4 must be in [-3.0718, -0.0698]
    const std::string m = messageOf(v);
    EXPECT_NE(m.find("fer_joint4"), std::string::npos) << m;
    EXPECT_NE(m.find("outside the joint limits"), std::string::npos) << m;
    v = ready();
    v[6] = -2.9;
    EXPECT_NE(messageOf(v).find("fer_joint7"), std::string::npos);
}

TEST(StartState, FromOverrideKeepsTheValuesAndNamesTheSource) {
    if (!std::ifstream(urdfPath()).good()) GTEST_SKIP() << "URDF fixture not found: " << urdfPath();
    std::vector<double> v = ready();
    v[0] = 0.05;
    v[5] = 1.6;
    const StartState s = makeStartStateFromOverride(v, urdfPath());
    for (int i = 0; i < 7; ++i) EXPECT_EQ(s.q(i), v[static_cast<std::size_t>(i)]);
    EXPECT_EQ(s.source, "start_joints_override_rad parameter");
}

TEST(StartState, FromOverrideUsesTheUrdfLimits) {
    if (!std::ifstream(urdfPath()).good()) GTEST_SKIP() << "URDF fixture not found: " << urdfPath();
    std::vector<double> v = ready();
    v[3] = 0.5;
    EXPECT_THROW(makeStartStateFromOverride(v, urdfPath()), std::invalid_argument);
    EXPECT_THROW(makeStartStateFromOverride(std::vector<double>(6, 0.0), urdfPath()), std::invalid_argument);
    EXPECT_THROW(makeStartStateFromOverride(ready(), "/nonexistent/robot.urdf"), std::runtime_error);
}

TEST(StartState, DistanceFromReadyPoseIsZeroAtReadyPose) {
    auto robot = loadPandaRobotModel();
    if (!robot) GTEST_SKIP() << "URDF fixture not available";
    const StartDistance d = distanceFromReadyPose(*robot, RobotModel::readyPose());
    EXPECT_NEAR(d.joint_norm_rad, 0.0, 1e-15);
    EXPECT_NEAR(d.joint_max_rad, 0.0, 1e-15);
    EXPECT_NEAR(d.task_mm, 0.0, 1e-9);
}

TEST(StartState, DistanceFromReadyPoseReportsJointsAndTaskSpaceAndLeavesTheRobotAtQ0) {
    auto robot = loadPandaRobotModel();
    if (!robot) GTEST_SKIP() << "URDF fixture not available";
    RobotModel::JointVector q0 = RobotModel::readyPose();
    q0(2) += 0.04;   // the largest deviation: fer_joint3
    q0(3) += 0.01;
    const StartDistance d = distanceFromReadyPose(*robot, q0);
    EXPECT_NEAR(d.joint_norm_rad, std::sqrt(0.04 * 0.04 + 0.01 * 0.01), 1e-12);
    EXPECT_NEAR(d.joint_max_rad, 0.04, 1e-12);
    EXPECT_EQ(d.joint_max_index, 3);
    EXPECT_GT(d.task_mm, 1.0);   // tens of mm of joint offset move the TCP by millimetres
    EXPECT_LT(d.task_mm, 100.0);

    // Independent FK of both configurations.
    auto ref = loadPandaRobotModel();
    ref->update(RobotModel::readyPose(), RobotModel::JointVector::Zero());
    const Eigen::Vector3d p_ready = ref->eePosition();
    ref->update(q0, RobotModel::JointVector::Zero());
    EXPECT_NEAR(d.task_mm, (ref->eePosition() - p_ready).norm() * 1000.0, 1e-9);

    // The robot is left updated at q0.
    EXPECT_NEAR((robot->eePosition() - ref->eePosition()).norm(), 0.0, 1e-12);
}
