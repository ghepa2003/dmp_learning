#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/parameter_client.hpp>

#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/quaternion_dmp.hpp"
#include "haptic_dmp_learning/core/controller_param_sync.hpp"

namespace haptic_dmp_learning {
namespace ros_wrapper {

/**
 * @brief ROS 2 twin of DmpGazeboExecutorNode that replays an integral-form
 *        core::ProDMP for the POSITION channel (orientation stays on
 *        core::QuaternionDMP).
 *
 * @details
 * Structurally identical to dmp_gazebo_executor_node - same parameter set, same
 * TF pattern (tf2_ros::Buffer/TransformListener, world_frame_ -> ee_frame_
 * lookup with the same timeout/retry), same gripper close ramp, same end-of-
 * rollout clamp. Two things differ:
 *
 * 1. Position model. core::ProDMP replaces core::DMP. stepCallback() calls
 *    prodmp_.step(dt) and the rollout duration / final clamp use
 *    prodmp_.tau() / prodmp_.goal().
 *
 * 2. Retarget. The classic node has to bridge frames by hand:
 *      dmp_.setGoal(dmp_.y0() + (target_position_world - ee_now))
 *    because DMP::setGoal() lives in the demo-local frame of dmp_.y0(). ProDMP
 *    is linearly parameterised, so "goal relative to the initial position" is a
 *    first-class mode: enable prodmp_.setRelativeGoal(true) once, anchor
 *    prodmp_.setInitialConditions(..., ee_now_world, ...) to the REAL EE pose
 *    read from TF, then prodmp_.setGoal(target_position_world) directly - init
 *    position and goal are already in the same (world) frame, no manual delta.
 *
 * Orientation is NOT covered by ProDMP in this project. This node still loads
 * and runs a core::QuaternionDMP exactly like dmp_gazebo_executor_node. Because
 * a ProDMP weights file has no quaternion_dmp section, the orientation weights
 * must be supplied separately via orientation_weights_yaml_path (the classic
 * combined dmp_weights_<run_id>.yaml); the node fails loud if it cannot obtain
 * them.
 */
class ProDmpGazeboExecutorNode : public rclcpp::Node {
public:
    ProDmpGazeboExecutorNode();

private:
    void startTimer();
    void stepCallback();
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /// @brief Looks up world_frame_ -> ee_frame_ via TF, retrying every 0.1 s
    /// against odom_wait_start_/target_odom_timeout_sec_ until it succeeds or
    /// the shared timeout fires (fail-loud, throws). Factored out of
    /// startTimer() so BOTH the live-target retarget branch and the satellite
    /// rotation branch (which also needs a real EE anchor for
    /// prodmp_.setInitialConditions(), regardless of where the goal itself
    /// comes from) share one implementation instead of two copies.
    /// @return true if ee_now was resolved; false if the caller should return
    ///         and wait for the next 0.1 s re-poll (startTimer() reschedules
    ///         itself before returning false - this function never re-arms
    ///         the timer itself, to keep timer ownership in one place).
    bool lookupEeNowWithRetry(Eigen::Vector3d& ee_now);

    /// @brief Sets core::controller_param_sync::kInitialAlignmentPositionOverrideParamName on the
    /// cartesian_impedance_controller node to `ee_now` (the SAME value used to anchor
    /// prodmp_.setInitialConditions() at the call site), BEFORE step_timer_ starts publishing
    /// /target_pose. Eliminates the FrameAligner race described in controller_param_sync.hpp.
    /// Fail-loud (throws) if the controller cannot be reached or rejects the parameter within
    /// controller_param_sync_timeout_sec_ - no silent fallback to the old racy behavior.
    void syncControllerAlignmentOverride(const Eigen::Vector3d& ee_now);

    /// @brief Sets core::controller_param_sync::kSkipInitialAlignmentParamName on the
    /// cartesian_impedance_controller node, BEFORE step_timer_ starts publishing /target_pose.
    /// Called TOGETHER with syncControllerAlignmentOverride() (same call sites, same client, same
    /// fail-loud convention) from the satellite_rotation and target_odom_required branches only -
    /// see those call sites in startTimer() for why: both already anchor init_pos_/goal in
    /// absolute world coordinates via anchorAndRotate(), and world/fer_link0 are confirmed
    /// coincident, so any FrameAligner offset there - INCLUDING the
    /// initial_alignment_position_override-based one from syncControllerAlignmentOverride() - is
    /// pure noise. The baseline/pure-replay branch (target_odom_required_=false) is deliberately
    /// left untouched: alignment is still needed there.
    void syncControllerSkipInitialAlignment(bool skip);

    /// @brief Lazily creates param_sync_helper_node_/controller_param_client_ if not already
    /// present, shared by syncControllerAlignmentOverride() and
    /// syncControllerSkipInitialAlignment() so both parameters go through the SAME client/helper
    /// node instead of each reconnecting separately.
    ///
    /// @details Why a whole extra rclcpp::Node, not just rclcpp::SyncParametersClient(this, ...):
    /// rclcpp::Node tracks, PER NODE (not per executor instance), whether it is currently
    /// associated with an executor. By the time startTimer() runs, `this` is already added to the
    /// process's main executor (rclcpp::spin(node) in main()). SyncParametersClient's blocking
    /// calls (set_parameters_atomically(), wait_for_service() does NOT need this, but the actual
    /// RPC does) internally do their OWN executor_->add_node(node_base_interface) /
    /// spin_until_future_complete() / remove_node() around the wait - and that add_node() throws
    /// "Node has already been added to an executor." the moment it's tried on `this`, REGARDLESS
    /// of whether executor_ is a brand-new, otherwise-unused rclcpp::Executor instance (confirmed
    /// against the actual rclcpp Humble build in
    /// test/test_param_client_executor_regression.cpp - passing an explicit fresh executor to
    /// SyncParametersClient does NOT sidestep this; the conflict is the NODE's state, not which
    /// executor object is used). The only way to make a blocking parameter-client call from
    /// inside a node that is already spinning itself is to route it through a genuinely different
    /// node that was never added to any executor - hence param_sync_helper_node_, a small
    /// dedicated rclcpp::Node used ONLY as the parameter client's node identity (never spun
    /// directly; SyncParametersClient manages its own private executor around it). This is the
    /// standard ROS 2 Humble workaround for "synchronous service/parameter call from within a
    /// node's own callback" (the same reason e.g. MoveGroupInterface keeps a private internal
    /// node for its own synchronous calls).
    ///
    /// Lifetime: created once, lazily, on first use, and intentionally never destroyed - it lives
    /// for the rest of the process (a plain member, torn down implicitly when `this` is). That
    /// means "prodmp_param_sync_helper" is visible in `ros2 node list` for the process's whole
    /// lifetime after the first call, not just around the 2 blocking calls it's actually used for -
    /// graph noise, not a leak (RAII via shared_ptr; no accumulation - reused across ALL calls
    /// in the process, never recreated). Acceptable given this node's actual deployment: a fresh
    /// subprocess per rollout (see run_satellite_rotation_experiment.py), so the extra node dies
    /// with the process and never accumulates across runs. Revisit if this node is ever
    /// refactored into a long-lived, multi-rollout process.
    void ensureControllerParamClient();

    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    // Velocity feedforward companion to pose_pub_: prodmp_.velocity() / qdmp_.omega()
    // are native analytic state (see core::ProDMP::velocity(), core::QuaternionDMP::
    // omega()), not a finite-difference reconstruction. Published in the SAME
    // demo-local frame as pose_pub_ (no TF/offset applied here - the downstream
    // CartesianVelocityController's FrameAligner rotates it into the robot base
    // frame using the same offset it captures for the pose). Always published,
    // regardless of whether any consumer has feedforward enabled.
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    std::string target_twist_topic_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_odom_sub_;
    // TF (world -> EE) for the retarget in startTimer(); created only when
    // target_odom_required_ is true.
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    // One-shot event: the replay gripper close ramp has finished (fingers at
    // gripper_closed_position_). grasp_force_calibration_node captures force on this.
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr gripper_close_complete_pub_;
    rclcpp::TimerBase::SharedPtr startup_timer_;
    rclcpp::TimerBase::SharedPtr step_timer_;

    // Core mathematical models. ProDMP for position, QuaternionDMP for orientation.
    core::ProDMP prodmp_;
    core::QuaternionDMP qdmp_;

    // Demonstrated initial conditions restored from the ProDMP weights YAML.
    // Used to rewind the integrator on the no-retarget path (ProDMP has no
    // reset(); re-seating the initial conditions is the rewind).
    Eigen::Vector3d demo_init_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d demo_init_vel_ = Eigen::Vector3d::Zero();

    // Rollout tracking state
    double dt_;
    double elapsed_;
    bool finished_;
    double gripper_trigger_t_ = -1.0;
    bool gripper_trigger_sent_ = false;

    // Gripper close ramp state (see core/gripper_ramp.hpp). elapsed_ is reused as
    // the monotonic sim-time base - no second clock, no extra timer.
    bool gripper_ramp_active_ = false;
    double gripper_ramp_start_elapsed_ = -1.0;

    // One-shot translational retarget toward the live free_target_object position.
    // qdmp_ / orientation is intentionally left untouched.
    bool target_odom_required_;        ///< false => no odom wait/retarget, replay the demo's original goal as-is
    std::string target_odom_topic_;
    std::string world_frame_;          ///< TF frame the target odometry is expressed in
    std::string ee_frame_;             ///< TF end-effector (TCP) frame, looked up as world_frame_ -> ee_frame_
    double target_odom_timeout_sec_;   ///< fail-loud if no odometry within this many seconds (only when required)
    bool target_odom_received_ = false;
    bool odom_wait_started_ = false;   ///< true once startTimer() began counting toward the timeout
    rclcpp::Time odom_wait_start_;     ///< node-clock instant the odometry wait started
    Eigen::Vector3d target_position_ = Eigen::Vector3d::Zero();

    // syncControllerAlignmentOverride()/syncControllerSkipInitialAlignment() target: the
    // cartesian_impedance_controller ROS 2 node name/timeout, and a lazily-created parameters
    // client reused across calls (only 2 calls per rollout today, but a future caller doing
    // several retargets in the same process should not reconnect each time). See
    // controller_param_sync.hpp for the parameters themselves.
    std::string cartesian_controller_node_name_;
    double controller_param_sync_timeout_sec_;
    // Dedicated node used ONLY as controller_param_client_'s identity - never `this`, and never
    // spun directly. See ensureControllerParamClient()'s doc comment for exactly why `this` can't
    // be used here (a node already added to an executor breaks SyncParametersClient's blocking
    // calls) and why never explicitly destroying this is fine given how this node is deployed.
    std::shared_ptr<rclcpp::Node> param_sync_helper_node_;
    std::shared_ptr<rclcpp::SyncParametersClient> controller_param_client_;

    // Satellite rotation model (position-only reach test; see class docs and
    // startTimer()). p_grasp_demo, captured once right after the ProDMP load
    // and never touched afterward (prodmp_.goal() would already be
    // overwritten by the time the target_odom_required_ branch runs its own
    // retarget).
    Eigen::Vector3d demo_grasp_goal_ = Eigen::Vector3d::Zero();
    bool satellite_rotation_enabled_;
    std::string satellite_rotation_mode_;  ///< "frozen" (impl.) | "continuous" (not yet)
    Eigen::Vector3d satellite_rotation_axis_;
    Eigen::Vector3d satellite_rotation_center_;
    double satellite_rotation_angular_velocity_deg_s_;
    double satellite_rotation_frozen_phase_deg_;

    // Parameters
    std::string weights_yaml_path_;             ///< ProDMP position weights (prodmp_weights_<run_id>.yaml)
    std::string orientation_weights_yaml_path_; ///< classic combined file for the quaternion_dmp section
    std::string demo_csv_path_;
    std::string target_pose_topic_;
    std::string frame_id_;
    double control_rate_hz_;
    double startup_delay_sec_;
    double gripper_open_position_;
    double gripper_closed_position_;
    double gripper_close_ramp_duration_sec_;
};

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning
