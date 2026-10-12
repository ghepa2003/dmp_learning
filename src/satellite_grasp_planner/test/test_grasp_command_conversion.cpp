// Tests for toGraspCommandMsg() (grasp_command_conversion.hpp): a pure conversion, no rclcpp::init()
// needed. Covers all three GraspLaunchResult::Status values and the candidate diagnostics.

#include <gtest/gtest.h>

#include <cstdint>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "satellite_grasp_planner/core/grasp_launch_result.hpp"
#include "satellite_grasp_planner/ros/grasp_command_conversion.hpp"

using namespace satellite_grasp_planner;
using satellite_grasp_msgs::msg::GraspCommand;

namespace {

core::SatelliteSnapshot makeSnapshot() {
    core::SatelliteSnapshot s;
    s.center = Eigen::Vector3d(1.0, 2.0, 3.0);
    s.axis = Eigen::Vector3d(0, 0, 1);
    s.omega_rad_s = -0.0349;
    s.theta_rad = 0.4;
    s.t_s = 12.5;
    return s;
}

const char* kHash = "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";

ros_wrapper::GraspCommandProvenance makeProvenance() {
    ros_wrapper::GraspCommandProvenance p;
    p.start_position = Eigen::Vector3d(0.3, -0.2, 0.6);
    p.weights_sha256 = kHash;
    return p;
}

}  // namespace

TEST(GraspCommandConversionTest, NoFeasibleCandidateLeavesGoalFieldsDefault) {
    core::GraspLaunchResult result;
    result.status = core::GraspLaunchResult::Status::kNoFeasibleCandidate;
    result.evaluations = 5;
    result.elapsed_s = 1.5;

    const auto snapshot = makeSnapshot();
    const rclcpp::Time stamp(12, 500000000, RCL_ROS_TIME);
    const GraspCommand msg = ros_wrapper::toGraspCommandMsg(result, snapshot, stamp, makeProvenance());

    EXPECT_EQ(msg.status, GraspCommand::STATUS_NO_FEASIBLE_CANDIDATE);
    EXPECT_EQ(msg.goal_pose.position.x, 0.0);
    EXPECT_EQ(msg.goal_pose.position.y, 0.0);
    EXPECT_EQ(msg.goal_pose.position.z, 0.0);
    EXPECT_EQ(msg.k, 0);
    EXPECT_EQ(msg.theta_star_rad, 0.0);
    EXPECT_FALSE(msg.wrist_flip);
    EXPECT_EQ(msg.provisional_delay_s, 0.0);
    EXPECT_EQ(msg.contact_time_s, 0.0);  // tau_launch_s defaults to 0.0 when not SELECTED
    EXPECT_DOUBLE_EQ(msg.omega_rad_s, snapshot.omega_rad_s);
    EXPECT_EQ(msg.evaluations, 5u);
    EXPECT_DOUBLE_EQ(msg.selection_elapsed_s, 1.5);
}

TEST(GraspCommandConversionTest, PosthocFailedHasItsOwnStatus) {
    core::GraspLaunchResult result;
    result.status = core::GraspLaunchResult::Status::kPosthocFailed;
    result.evaluations = 880;

    const auto snapshot = makeSnapshot();
    const rclcpp::Time stamp(0, 0, RCL_ROS_TIME);
    const GraspCommand msg = ros_wrapper::toGraspCommandMsg(result, snapshot, stamp, makeProvenance());

    EXPECT_EQ(msg.status, GraspCommand::STATUS_POSTHOC_FAILED);
    EXPECT_NE(msg.status, GraspCommand::STATUS_NO_FEASIBLE_CANDIDATE);
    EXPECT_EQ(msg.evaluations, 880u);
    EXPECT_EQ(msg.contact_time_s, 0.0);
}

TEST(GraspCommandConversionTest, SelectedPopulatesGoalFieldsAndContactTimeVerbatim) {
    core::GraspLaunchResult result;
    result.status = core::GraspLaunchResult::Status::kSelected;
    result.goal_position = Eigen::Vector3d(0.5, 0.1, 0.2);
    result.goal_orientation = Eigen::Quaterniond(0.9, 0.1, 0.2, 0.3).normalized();
    result.k = haptic_dmp_learning::core::GraspPointId::kP90;
    result.theta_star_rad = 0.77;
    result.wrist_flip = true;
    result.tau_launch_s = 0.42;
    result.launch = core::LaunchPlan{};
    result.launch->delay_s = 3.3;

    const auto snapshot = makeSnapshot();
    const rclcpp::Time stamp(1, 0, RCL_ROS_TIME);
    const GraspCommand msg = ros_wrapper::toGraspCommandMsg(result, snapshot, stamp, makeProvenance());

    EXPECT_EQ(msg.status, GraspCommand::STATUS_SELECTED);
    EXPECT_DOUBLE_EQ(msg.goal_pose.position.x, 0.5);
    EXPECT_DOUBLE_EQ(msg.goal_pose.position.y, 0.1);
    EXPECT_DOUBLE_EQ(msg.goal_pose.position.z, 0.2);
    EXPECT_EQ(msg.k, 1);  // kP90 == 1
    EXPECT_DOUBLE_EQ(msg.theta_star_rad, 0.77);
    EXPECT_TRUE(msg.wrist_flip);
    EXPECT_DOUBLE_EQ(msg.contact_time_s, 0.42);  // tau_launch_s verbatim, no +dt (D6)
    EXPECT_DOUBLE_EQ(msg.provisional_delay_s, 3.3);
}

TEST(GraspCommandConversionTest, CandidateDiagnosticsOnlySetWhenSelected) {
    const auto snapshot = makeSnapshot();
    const rclcpp::Time stamp(0, 0, RCL_ROS_TIME);

    core::GraspLaunchResult selected;
    selected.status = core::GraspLaunchResult::Status::kSelected;
    selected.goal_position = Eigen::Vector3d::Zero();
    selected.goal_orientation = Eigen::Quaterniond::Identity();
    selected.k = haptic_dmp_learning::core::GraspPointId::kP0;
    selected.theta_star_rad = 0.0;
    selected.launch = core::LaunchPlan{};
    selected.cost = 0.1427;
    selected.d_pos_m = 0.0134;
    selected.w_hat = 0.7536;
    selected.min_joint_margin_rad = 0.7664;
    selected.posthoc_dv_m_s = 2.5e-3;
    const GraspCommand sel = ros_wrapper::toGraspCommandMsg(selected, snapshot, stamp, makeProvenance());
    EXPECT_DOUBLE_EQ(sel.cost, 0.1427);
    EXPECT_DOUBLE_EQ(sel.d_pos_m, 0.0134);
    EXPECT_DOUBLE_EQ(sel.w_hat, 0.7536);
    EXPECT_DOUBLE_EQ(sel.min_joint_margin_rad, 0.7664);
    EXPECT_DOUBLE_EQ(sel.posthoc_dv_m_s, 2.5e-3);

    // Not SELECTED: stale candidate values must NOT leak into the message.
    for (const auto status : {core::GraspLaunchResult::Status::kNoFeasibleCandidate,
                              core::GraspLaunchResult::Status::kPosthocFailed}) {
        core::GraspLaunchResult r;
        r.status = status;
        r.cost = 9.0;
        r.d_pos_m = 9.0;
        r.w_hat = 9.0;
        r.min_joint_margin_rad = 9.0;
        r.posthoc_dv_m_s = 9.0;
        const GraspCommand msg = ros_wrapper::toGraspCommandMsg(r, snapshot, stamp, makeProvenance());
        EXPECT_EQ(msg.cost, 0.0);
        EXPECT_EQ(msg.d_pos_m, 0.0);
        EXPECT_EQ(msg.w_hat, 0.0);
        EXPECT_EQ(msg.min_joint_margin_rad, 0.0);
        EXPECT_EQ(msg.posthoc_dv_m_s, 0.0);
    }
}

TEST(GraspCommandConversionTest, NamesMatchConstants) {
    EXPECT_STREQ(ros_wrapper::statusName(GraspCommand::STATUS_SELECTED), "STATUS_SELECTED");
    EXPECT_STREQ(ros_wrapper::statusName(GraspCommand::STATUS_POSTHOC_FAILED), "STATUS_POSTHOC_FAILED");
    EXPECT_STREQ(ros_wrapper::statusName(GraspCommand::STATUS_BUDGET_EXHAUSTED_NO_CANDIDATE),
                 "STATUS_BUDGET_EXHAUSTED_NO_CANDIDATE");
    EXPECT_STREQ(ros_wrapper::statusName(200), "STATUS_<unknown>");
}

TEST(GraspCommandConversionTest, ProvenanceFieldsInAllThreeStatuses) {
    const auto snapshot = makeSnapshot();
    const rclcpp::Time stamp(0, 0, RCL_ROS_TIME);
    using S = core::GraspLaunchResult::Status;
    for (const S status : {S::kSelected, S::kNoFeasibleCandidate, S::kPosthocFailed}) {
        core::GraspLaunchResult r;
        r.status = status;
        if (status == S::kSelected) {
            r.goal_position = Eigen::Vector3d::Zero();
            r.goal_orientation = Eigen::Quaterniond::Identity();
            r.k = haptic_dmp_learning::core::GraspPointId::kP0;
            r.theta_star_rad = 0.0;
            r.launch = core::LaunchPlan{};
        }
        const GraspCommand msg = ros_wrapper::toGraspCommandMsg(r, snapshot, stamp, makeProvenance());
        SCOPED_TRACE(ros_wrapper::statusName(msg.status));
        EXPECT_DOUBLE_EQ(msg.snapshot_theta_rad, 0.4);
        EXPECT_DOUBLE_EQ(msg.snapshot_t_s, 12.5);
        EXPECT_EQ(msg.weights_sha256, kHash);
        if (status == S::kSelected) {
            EXPECT_DOUBLE_EQ(msg.start_position.x, 0.3);
            EXPECT_DOUBLE_EQ(msg.start_position.y, -0.2);
            EXPECT_DOUBLE_EQ(msg.start_position.z, 0.6);
        } else {
            EXPECT_EQ(msg.start_position.x, 0.0);
            EXPECT_EQ(msg.start_position.y, 0.0);
            EXPECT_EQ(msg.start_position.z, 0.0);
        }
    }
}
