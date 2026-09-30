// GraspCommand -> GraspCommandInputs conversion (message types only, no rclcpp::init()).

#include <gtest/gtest.h>

#include "haptic_dmp_learning/ros/grasp_command_input.hpp"

using haptic_dmp_learning::ros_wrapper::toGraspCommandInputs;
using satellite_grasp_msgs::msg::GraspCommand;
namespace si = haptic_dmp_learning::core::satellite_intercept;

TEST(GraspCommandInputTest, CopiesTheFieldsTheExecutorUses) {
    GraspCommand msg;
    msg.header.stamp = rclcpp::Time(12, 500000000, RCL_ROS_TIME);
    msg.header.frame_id = "world";
    msg.status = GraspCommand::STATUS_SELECTED;
    msg.goal_pose.position.x = 0.5;
    msg.goal_pose.position.y = 0.1;
    msg.goal_pose.position.z = 0.4;
    msg.start_position.x = 0.3;
    msg.start_position.y = -0.2;
    msg.start_position.z = 0.5;
    msg.theta_star_rad = 0.7;
    msg.omega_rad_s = -0.0349;
    msg.contact_time_s = 4.0;
    msg.snapshot_theta_rad = 0.2;
    msg.snapshot_t_s = 9.5;
    msg.weights_sha256 = std::string(64, 'c');
    msg.rows_evaluated = 7;
    msg.stop_reason = GraspCommand::STOP_MAX_ROWS_REACHED;
    msg.bound_violated = true;

    const si::GraspCommandInputs in = toGraspCommandInputs(msg);
    EXPECT_EQ(in.status, si::kGraspStatusSelected);
    EXPECT_EQ(in.frame_id, "world");
    EXPECT_DOUBLE_EQ(in.stamp_s, 12.5);
    EXPECT_TRUE(in.goal_position.isApprox(Eigen::Vector3d(0.5, 0.1, 0.4)));
    EXPECT_TRUE(in.start_position.isApprox(Eigen::Vector3d(0.3, -0.2, 0.5)));
    EXPECT_DOUBLE_EQ(in.theta_star_rad, 0.7);
    EXPECT_DOUBLE_EQ(in.omega_rad_s, -0.0349);
    EXPECT_DOUBLE_EQ(in.contact_time_s, 4.0);
    EXPECT_DOUBLE_EQ(in.snapshot_theta_rad, 0.2);
    EXPECT_DOUBLE_EQ(in.snapshot_t_s, 9.5);
    EXPECT_EQ(in.weights_sha256, std::string(64, 'c'));
    EXPECT_EQ(in.rows_evaluated, 7);
    EXPECT_EQ(in.stop_reason, GraspCommand::STOP_MAX_ROWS_REACHED);
    EXPECT_TRUE(in.bound_violated);
}

TEST(GraspCommandInputTest, StatusNamesMatchTheMessageConstants) {
    EXPECT_STREQ(si::graspStatusName(GraspCommand::STATUS_SELECTED), "STATUS_SELECTED");
    EXPECT_STREQ(si::graspStatusName(GraspCommand::STATUS_UNSET), "STATUS_UNSET");
    EXPECT_STREQ(si::graspStatusName(GraspCommand::STATUS_NO_FEASIBLE_CANDIDATE), "STATUS_NO_FEASIBLE_CANDIDATE");
    EXPECT_STREQ(si::graspStatusName(GraspCommand::STATUS_BUDGET_EXHAUSTED_NO_CANDIDATE),
                 "STATUS_BUDGET_EXHAUSTED_NO_CANDIDATE");
}
