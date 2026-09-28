// Tests for RobotModel::framePose() and the frame-name lists.
// Needs Pinocchio and $HOME/thesis_ws/fer_flat_effort.urdf (same fixture as the planner tests).

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "franka_cartesian_control/core/robot_model.hpp"

using franka_cartesian_control::core::RobotModel;

namespace {

std::unique_ptr<RobotModel> loadPandaRobotModel() {
    const char* home = std::getenv("HOME");
    const std::string urdf_path =
        std::string(home ? home : "/root") + "/thesis_ws/fer_flat_effort.urdf";
    std::ifstream f(urdf_path);
    if (!f.is_open()) {
        ADD_FAILURE() << "Could not open URDF (required test fixture): " << urdf_path;
        return nullptr;
    }
    std::stringstream buffer;
    buffer << f.rdbuf();
    std::vector<std::string> joint_names;
    for (int i = 1; i <= 7; ++i) joint_names.push_back("fer_joint" + std::to_string(i));
    return std::make_unique<RobotModel>(buffer.str(), joint_names, "fer_hand_tcp");
}

}  // namespace

TEST(RobotModelFramePoseTest, EeFrameMatchesEePositionAndOrientation) {
    auto model = loadPandaRobotModel();
    ASSERT_NE(model, nullptr);
    model->update(RobotModel::readyPose(), RobotModel::JointVector::Zero());
    const auto pose = model->framePose("fer_hand_tcp");
    EXPECT_LT((pose.position - model->eePosition()).norm(), 1e-12);
    EXPECT_LT(pose.orientation.angularDistance(model->eeOrientation()), 1e-12);
}

TEST(RobotModelFramePoseTest, IntermediateFrameDiffersFromEeAndTracksConfiguration) {
    auto model = loadPandaRobotModel();
    ASSERT_NE(model, nullptr);
    const RobotModel::JointVector q_a = RobotModel::readyPose();
    RobotModel::JointVector q_b = q_a;
    q_b(0) += 0.5;  // rotate the base joint
    q_b(1) += 0.3;

    model->update(q_a, RobotModel::JointVector::Zero());
    const auto link4_a = model->framePose("fer_link4");
    EXPECT_GT((link4_a.position - model->eePosition()).norm(), 1e-3);

    model->update(q_b, RobotModel::JointVector::Zero());
    const auto link4_b = model->framePose("fer_link4");
    EXPECT_GT((link4_b.position - link4_a.position).norm(), 1e-3);
}

TEST(RobotModelFramePoseTest, UnknownFrameThrows) {
    auto model = loadPandaRobotModel();
    ASSERT_NE(model, nullptr);
    model->update(RobotModel::readyPose(), RobotModel::JointVector::Zero());
    EXPECT_THROW(model->framePose("no_such_frame"), std::invalid_argument);
}

TEST(RobotModelFrameNamesTest, ListsContainExactlyTheExpectedNames) {
    const std::vector<std::string> arm_expected = {
        "fer_link0", "fer_link1", "fer_link2", "fer_link3", "fer_link4",
        "fer_link5", "fer_link6", "fer_link7", "fer_link8", "fer_hand"};
    const std::vector<std::string> gripper_expected = {"fer_leftfinger", "fer_rightfinger"};
    EXPECT_EQ(RobotModel::armLinkFrameNames(), arm_expected);
    EXPECT_EQ(RobotModel::gripperFrameNames(), gripper_expected);
    for (const auto* v : {&RobotModel::armLinkFrameNames(), &RobotModel::gripperFrameNames()}) {
        EXPECT_EQ(std::set<std::string>(v->begin(), v->end()).size(), v->size());
    }
}

TEST(RobotModelFrameNamesTest, AllListedFramesExistInUrdf) {
    auto model = loadPandaRobotModel();
    ASSERT_NE(model, nullptr);
    model->update(RobotModel::readyPose(), RobotModel::JointVector::Zero());
    for (const auto* v : {&RobotModel::armLinkFrameNames(), &RobotModel::gripperFrameNames()}) {
        for (const auto& n : *v) EXPECT_NO_THROW(model->framePose(n)) << n;
    }
}
