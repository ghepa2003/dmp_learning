// Tests for toGraspCommandMsg() (grasp_command_conversion.hpp): a pure conversion, no rclcpp::init()
// needed. Covers all three GraspLaunchResult::Status values, all four StopReason values and
// best_total_cost.

#include <gtest/gtest.h>

#include <cstdint>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "satellite_grasp_planner/core/grasp_launch.hpp"
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

}  // namespace

TEST(GraspCommandConversionTest, NoFeasibleCandidateLeavesGoalFieldsDefault) {
    core::GraspLaunchResult result;
    result.status = core::GraspLaunchResult::Status::kNoFeasibleCandidate;
    result.selection.rows_evaluated = 5;
    result.selection.total_rollouts = 10;
    result.selection.elapsed_s = 1.5;
    result.selection.stop_reason = core::GraspSelection::StopReason::kCompleted;
    result.selection.bound_violated = false;
    result.selection.max_w_hat_seen = 0.9;

    const auto snapshot = makeSnapshot();
    const rclcpp::Time stamp(12, 500000000, RCL_ROS_TIME);
    const GraspCommand msg = ros_wrapper::toGraspCommandMsg(result, snapshot, stamp);

    EXPECT_EQ(msg.status, GraspCommand::STATUS_NO_FEASIBLE_CANDIDATE);
    EXPECT_EQ(msg.goal_pose.position.x, 0.0);
    EXPECT_EQ(msg.goal_pose.position.y, 0.0);
    EXPECT_EQ(msg.goal_pose.position.z, 0.0);
    EXPECT_EQ(msg.k, 0);
    EXPECT_EQ(msg.theta_star_rad, 0.0);
    EXPECT_EQ(msg.psi_rad, 0.0);
    EXPECT_EQ(msg.provisional_delay_s, 0.0);
    EXPECT_EQ(msg.contact_time_s, 0.0);  // tau_launch_s defaults to 0.0 when not SELECTED
    EXPECT_DOUBLE_EQ(msg.omega_rad_s, snapshot.omega_rad_s);
    EXPECT_EQ(msg.rows_evaluated, 5);
    EXPECT_EQ(msg.total_rollouts, 10);
    EXPECT_DOUBLE_EQ(msg.selection_elapsed_s, 1.5);
    EXPECT_EQ(msg.stop_reason, GraspCommand::STOP_COMPLETED);
    EXPECT_FALSE(msg.bound_violated);
    EXPECT_DOUBLE_EQ(msg.max_w_hat_seen, 0.9);
}

TEST(GraspCommandConversionTest, BudgetExhaustedNoCandidate) {
    core::GraspLaunchResult result;
    result.status = core::GraspLaunchResult::Status::kBudgetExhaustedNoCandidate;
    result.selection.stop_reason = core::GraspSelection::StopReason::kTimeBudgetExhausted;

    const auto snapshot = makeSnapshot();
    const rclcpp::Time stamp(0, 0, RCL_ROS_TIME);
    const GraspCommand msg = ros_wrapper::toGraspCommandMsg(result, snapshot, stamp);

    EXPECT_EQ(msg.status, GraspCommand::STATUS_BUDGET_EXHAUSTED_NO_CANDIDATE);
    EXPECT_EQ(msg.stop_reason, GraspCommand::STOP_TIME_BUDGET_EXHAUSTED);
    EXPECT_EQ(msg.contact_time_s, 0.0);
}

TEST(GraspCommandConversionTest, SelectedPopulatesGoalFieldsAndContactTimeVerbatim) {
    core::GraspLaunchResult result;
    result.status = core::GraspLaunchResult::Status::kSelected;
    result.goal_position = Eigen::Vector3d(0.5, 0.1, 0.2);
    result.goal_orientation = Eigen::Quaterniond(0.9, 0.1, 0.2, 0.3).normalized();
    result.k = haptic_dmp_learning::core::GraspPointId::kP90;
    result.theta_star_rad = 0.77;
    result.psi_rad = 1.57;
    result.tau_launch_s = 0.42;
    result.launch = core::LaunchPlan{};
    result.launch->delay_s = 3.3;
    result.selection.stop_reason = core::GraspSelection::StopReason::kBoundSatisfied;

    const auto snapshot = makeSnapshot();
    const rclcpp::Time stamp(1, 0, RCL_ROS_TIME);
    const GraspCommand msg = ros_wrapper::toGraspCommandMsg(result, snapshot, stamp);

    EXPECT_EQ(msg.status, GraspCommand::STATUS_SELECTED);
    EXPECT_DOUBLE_EQ(msg.goal_pose.position.x, 0.5);
    EXPECT_DOUBLE_EQ(msg.goal_pose.position.y, 0.1);
    EXPECT_DOUBLE_EQ(msg.goal_pose.position.z, 0.2);
    EXPECT_EQ(msg.k, 1);  // kP90 == 1
    EXPECT_DOUBLE_EQ(msg.theta_star_rad, 0.77);
    EXPECT_DOUBLE_EQ(msg.psi_rad, 1.57);
    EXPECT_DOUBLE_EQ(msg.contact_time_s, 0.42);  // tau_launch_s verbatim, no +dt (D6)
    EXPECT_DOUBLE_EQ(msg.provisional_delay_s, 3.3);
    EXPECT_EQ(msg.stop_reason, GraspCommand::STOP_BOUND_SATISFIED);
}

TEST(GraspCommandConversionTest, AllStopReasonsMapToTheirConstants) {
    using SR = core::GraspSelection::StopReason;
    const std::vector<std::pair<SR, uint8_t>> expected = {
        {SR::kCompleted, GraspCommand::STOP_COMPLETED},
        {SR::kBoundSatisfied, GraspCommand::STOP_BOUND_SATISFIED},
        {SR::kMaxRowsReached, GraspCommand::STOP_MAX_ROWS_REACHED},
        {SR::kTimeBudgetExhausted, GraspCommand::STOP_TIME_BUDGET_EXHAUSTED},
    };
    const auto snapshot = makeSnapshot();
    const rclcpp::Time stamp(0, 0, RCL_ROS_TIME);
    for (const auto& [reason, constant] : expected) {
        core::GraspLaunchResult result;  // status kNoFeasibleCandidate: only the stop_reason mapping matters
        result.selection.stop_reason = reason;
        const GraspCommand msg = ros_wrapper::toGraspCommandMsg(result, snapshot, stamp);
        EXPECT_EQ(msg.stop_reason, constant) << "StopReason=" << static_cast<int>(reason);
        EXPECT_NE(msg.stop_reason, GraspCommand::STOP_UNSET);
    }
}

TEST(GraspCommandConversionTest, BestTotalCostOnlySetWhenSelected) {
    const auto snapshot = makeSnapshot();
    const rclcpp::Time stamp(0, 0, RCL_ROS_TIME);

    core::GraspLaunchResult selected;
    selected.status = core::GraspLaunchResult::Status::kSelected;
    selected.goal_position = Eigen::Vector3d::Zero();
    selected.goal_orientation = Eigen::Quaterniond::Identity();
    selected.k = haptic_dmp_learning::core::GraspPointId::kP0;
    selected.theta_star_rad = 0.0;
    selected.psi_rad = 0.0;
    selected.launch = core::LaunchPlan{};
    selected.selection.best.cost.total = -1.75;  // negative is legal: manipulability term lowers the cost
    EXPECT_DOUBLE_EQ(ros_wrapper::toGraspCommandMsg(selected, snapshot, stamp).best_total_cost, -1.75);

    // Not SELECTED: a stale best.cost.total in the diagnostics must NOT leak into the message.
    for (const auto status : {core::GraspLaunchResult::Status::kNoFeasibleCandidate,
                              core::GraspLaunchResult::Status::kBudgetExhaustedNoCandidate}) {
        core::GraspLaunchResult r;
        r.status = status;
        r.selection.best.cost.total = 9.0;
        EXPECT_EQ(ros_wrapper::toGraspCommandMsg(r, snapshot, stamp).best_total_cost, 0.0);
    }
}

TEST(GraspCommandConversionTest, NamesMatchConstants) {
    EXPECT_STREQ(ros_wrapper::statusName(GraspCommand::STATUS_SELECTED), "STATUS_SELECTED");
    EXPECT_STREQ(ros_wrapper::statusName(GraspCommand::STATUS_BUDGET_EXHAUSTED_NO_CANDIDATE),
                 "STATUS_BUDGET_EXHAUSTED_NO_CANDIDATE");
    EXPECT_STREQ(ros_wrapper::statusName(200), "STATUS_<unknown>");
    EXPECT_STREQ(ros_wrapper::stopReasonName(GraspCommand::STOP_TIME_BUDGET_EXHAUSTED),
                 "STOP_TIME_BUDGET_EXHAUSTED");
    EXPECT_STREQ(ros_wrapper::stopReasonName(200), "STOP_<unknown>");
}
