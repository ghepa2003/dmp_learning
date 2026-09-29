#pragma once

/**
 * @file grasp_planner_node.hpp
 * @brief One-shot grasp-command planner (v1, Task 1 of 2 - see DESIGN_NOTES.md's module map). The
 *        executor that consumes GraspCommand is a separate, later task.
 *
 * On the FIRST valid sample of @c satellite_odom_topic, builds a SatelliteSnapshot, runs the
 * (expensive: ~5.2 s Release / ~500 s unoptimized, per DESIGN_NOTES.md) selection off the ROS
 * callback thread, then publishes exactly one satellite_grasp_msgs/GraspCommand (QoS reliable +
 * transient_local + depth 1) and goes idle: no resubscription, no reselection. Every mandatory
 * parameter has NO default and fails loud (throws out of the constructor) if unset.
 *
 * ASSUMPTION (enforced at startup via checkRobotBaseWorldIsZero): the core selection pipeline
 * (candidate_scan.cpp) combines robot-base-frame FK with world-frame satellite geometry with no
 * transform - it implicitly assumes robot base == world. prodmp_gazebo_executor_node.cpp makes the
 * same assumption explicitly, for the same reason. robot_base_world must be (near-)zero in v1.
 *
 * cube_geometry.center_world/axis_world are IGNORED by the selection pipeline: buildSelectionInputs()
 * overwrites both with the snapshot's satellite_rotation_center/satellite_rotation_axis
 * (selection_inputs.cpp). They must equal those parameters (enforced at startup via
 * checkCubeCenterMatchesRotationCenter/checkCubeAxisMatchesRotationAxis) purely so the config file
 * cannot silently disagree with itself.
 *
 * CLOCK: runs with use_sim_time=true (asserted at startup: use_sim_time param + ros_time_is_active()).
 * planGraspAndLaunch() is called with clock=nullptr and t_now_override_s=std::nullopt, i.e. its
 * library default t_now_s = snapshot.t_s + WALL-CLOCK elapsed_s is used as-is; this only feeds
 * provisional_delay_s, which GraspCommand.msg documents as diagnostic-only (the consuming executor
 * triggers on phase, not on this delay). If the node clock reads earlier than snapshot.t_s at the end
 * of selection (sim clock inconsistency), the node fails loud (FATAL + rclcpp::shutdown()), same as
 * any other worker-thread failure.
 */

#include <atomic>
#include <optional>
#include <string>
#include <thread>

#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "satellite_grasp_msgs/msg/grasp_command.hpp"
#include "satellite_grasp_planner/core/launch_plan.hpp"
#include "satellite_grasp_planner/ros/grasp_planner_checks.hpp"

namespace satellite_grasp_planner {
namespace ros_wrapper {

class GraspPlannerNode : public rclcpp::Node {
public:
    GraspPlannerNode();

    /// Joins the worker thread: if a selection is in flight (no cancellation mechanism exists in
    /// selectGrasp()), destruction blocks until it finishes - up to ~5.2 s (Release) or ~500 s
    /// (unoptimized build).
    ~GraspPlannerNode() override;

private:
    void onOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void runSelection(const core::SatelliteSnapshot& snapshot);

    std::string demo_params_path_;
    std::string urdf_path_;
    double min_delay_s_ = 0.0;
    haptic_dmp_learning::core::CubeSatelliteModel::Params cube_geometry_;
    Eigen::Vector3d robot_base_world_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d satellite_rotation_center_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d satellite_rotation_axis_ = Eigen::Vector3d::UnitZ();
    double satellite_omega_rad_s_ = 0.0;
    Eigen::Quaterniond satellite_q_ref_ = Eigen::Quaterniond::Identity();
    double omega_consistency_tol_rad_s_ = 0.0;
    std::optional<double> time_budget_s_;
    SelectionOverrides selection_overrides_;  ///< optional SelectionParams overrides, see grasp_planner_checks.hpp
    std::string grasp_command_topic_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<satellite_grasp_msgs::msg::GraspCommand>::SharedPtr pub_;

    std::atomic<bool> dispatched_{false};
    std::thread worker_;
};

}  // namespace ros_wrapper
}  // namespace satellite_grasp_planner
