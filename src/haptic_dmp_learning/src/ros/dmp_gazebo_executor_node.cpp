#include "haptic_dmp_learning/ros/dmp_gazebo_executor_node.hpp"
#include "haptic_dmp_learning/core/dmp_io.hpp"
#include "haptic_dmp_learning/core/demo_csv_io.hpp"
#include "haptic_dmp_learning/core/gripper_ramp.hpp"

#include <cstdlib>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <stdexcept>

#include <rclcpp/create_timer.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;

namespace haptic_dmp_learning {
namespace ros_wrapper {

DmpGazeboExecutorNode::DmpGazeboExecutorNode()
    : Node("dmp_gazebo_executor_node"),
      dmp_(20, 4.6, 25.0, 6.25, false),   // Placeholders overwritten by loadFromYaml
      qdmp_(20, 4.6, 25.0, 6.25),
      dt_(0.005),
      elapsed_(0.0),
      finished_(false) {

    // 1. Declare parameters (absolute default so behavior does not depend on
    // the process's current working directory at launch)
    const char* home = std::getenv("HOME");
    const std::string default_weights_path = std::string(home ? home : "/root") + "/thesis_ws/dmp_weights.yaml";
    weights_yaml_path_ = this->declare_parameter<std::string>("weights_yaml_path", default_weights_path);
    target_pose_topic_ = this->declare_parameter<std::string>("target_pose_topic", "/target_pose");
    target_twist_topic_ = this->declare_parameter<std::string>("target_twist_topic", "/target_twist");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "panda_link0");
    control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 200.0);
    startup_delay_sec_ = this->declare_parameter<double>("startup_delay_sec", 1.0);

    // One-shot target retarget (translation only). Same odometry topic the
    // orchestrator waits on for its sim-time anchor; default matches the
    // free_target_object launch bridge (/model/<name>/odometry -> /<name>/odometry).
    target_odom_topic_ = this->declare_parameter<std::string>(
        "target_odom_topic", "/free_target_object/odometry");
    // Fail-loud: refuse to replay toward the demo's frozen goal if the live
    // target position never arrives.
    target_odom_timeout_sec_ = this->declare_parameter<double>("target_odom_timeout_sec", 5.0);
    // When false, skip the target-odometry subscription, wait and retarget
    // entirely: replay the demonstration's ORIGINAL goal (frozen in the loaded
    // weights) exactly as-is. Default true preserves the existing behaviour for
    // every current pipeline use; set false only for a run with no target at all
    // (e.g. a real-hardware open-loop replay), where there is deliberately
    // nothing to retarget toward.
    target_odom_required_ = this->declare_parameter<bool>("target_odom_required", true);
    // TF frames for the goal-frame correction applied before dmp_.setGoal():
    // target_position_ arrives in world_frame_, but dmp_.setGoal() expects a goal
    // in the demo-local frame of dmp_.y0(); the live EE pose (world_frame_ ->
    // ee_frame_) bridges the two. Same naming/defaults as
    // grasp_monitoring/geometric_grasp_monitor.
    world_frame_ = this->declare_parameter<std::string>("world_frame", "world");
    ee_frame_ = this->declare_parameter<std::string>("ee_frame", "fer_hand_tcp");

    // Gripper close ramp (see core/gripper_ramp.hpp). Defaults reproduce the
    // previous single-step behaviour's endpoints: 0.06 = open, 0.0 = closed.
    gripper_open_position_ = this->declare_parameter<double>("gripper_open_position", 0.06);
    gripper_closed_position_ = this->declare_parameter<double>("gripper_closed_position", 0.0);
    gripper_close_ramp_duration_sec_ =
        this->declare_parameter<double>("gripper_close_ramp_duration_sec", 2.0);

    dt_ = 1.0 / control_rate_hz_;

    // 2. Load trained DMP parameters from YAML
    try {
        core::dmp_io::loadFromYaml(weights_yaml_path_, dmp_, qdmp_);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "Failed to load DMP weights from %s: %s",
                     weights_yaml_path_.c_str(), e.what());
        throw;
    }

    if (std::abs(dmp_.tau() - qdmp_.tau()) > 1e-6) {
        RCLCPP_WARN(this->get_logger(),
                    "position tau (%.4f) and orientation tau (%.4f) differ - "
                    "using position tau as rollout duration.",
                    dmp_.tau(), qdmp_.tau());
    }

    // 2b. Parse demo CSV for gripper trigger timestamp if provided.
    // Keep "cannot open the file" (likely a wrong path) loud and separate from
    // "opened it, but this demo has no gripper trigger" (a normal case).
    demo_csv_path_ = this->declare_parameter<std::string>("demo_csv_path", "");
    if (!demo_csv_path_.empty()) {
        std::ifstream probe(demo_csv_path_);
        if (!probe.is_open()) {
            RCLCPP_WARN(this->get_logger(),
                        "Could not open demo CSV at %s; replay without gripper trigger.",
                        demo_csv_path_.c_str());
        } else {
            probe.close();
            gripper_trigger_t_ = core::demo_csv_io::readGripperTriggerTime(demo_csv_path_);
            if (gripper_trigger_t_ >= 0.0) {
                RCLCPP_INFO(this->get_logger(),
                            "Found gripper trigger in CSV at t = %.4f s", gripper_trigger_t_);
            } else {
                RCLCPP_INFO(this->get_logger(),
                            "No 'gripper_trigger' column or no triggering row in %s; "
                            "replay without gripper trigger.",
                            demo_csv_path_.c_str());
            }
        }
    }

    // 3. Reset internal rollout states (x=1, y=y0, q=q0)
    dmp_.reset();
    qdmp_.reset();

    // 4. Create publishers
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        target_pose_topic_, rclcpp::QoS(10));
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        target_twist_topic_, rclcpp::QoS(10));
    gripper_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/gripper_position_cmd", rclcpp::QoS(10));
    // One-shot announce that the gripper close ramp has finished (see header).
    gripper_close_complete_pub_ = this->create_publisher<std_msgs::msg::Empty>(
        "/gripper_close_complete", rclcpp::QoS(10));

    // 4b. One-shot target retarget: when required, subscribe to free_target_object
    // odometry. The first message gives the CURRENT target position; odomCallback
    // stores it, unsubscribes, and startTimer() applies it via dmp_.setGoal()
    // before the rollout starts. The odometry pose is in world_frame_, while
    // dmp_.setGoal() expects the demo-local frame of dmp_.y0(); startTimer()
    // bridges the two with a world_frame_ -> ee_frame_ TF lookup (tf_buffer_).
    // Orientation (qdmp_) is left at the demo goal. When target_odom_required_ is
    // false the subscription and the TF buffer/listener are never created and
    // startTimer() replays the demo's original goal as-is (no wait, no setGoal(),
    // no timeout).
    if (target_odom_required_) {
        target_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            target_odom_topic_, rclcpp::QoS(10),
            std::bind(&DmpGazeboExecutorNode::odomCallback, this, std::placeholders::_1));
        // Needed only for the world -> demo-local goal-frame correction in
        // startTimer(); pointless when there is no retarget.
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    }

    RCLCPP_INFO(this->get_logger(),
                "dmp_gazebo_executor_node ready. Weights: %s | tau: %.3f s | rate: %.1f Hz | "
                "publishing on %s in %.1f s | target_odom_required=%s (topic %s, timeout %.1f s)",
                weights_yaml_path_.c_str(), dmp_.tau(), control_rate_hz_,
                target_pose_topic_.c_str(), startup_delay_sec_,
                target_odom_required_ ? "true" : "false",
                target_odom_topic_.c_str(), target_odom_timeout_sec_);

    // Preventive warning for a common misconfiguration: use_sim_time:=true with
    // no /clock publisher (e.g. a real-hardware run with no Gazebo) leaves every
    // timer of this node stalled forever with no error. use_sim_time is the
    // standard parameter rclcpp auto-declares for every node.
    if (this->get_parameter("use_sim_time").as_bool()) {
        RCLCPP_WARN(this->get_logger(),
                    "use_sim_time=true: this node requires /clock to be actively "
                    "publishing (typically from Gazebo). If this is a REAL HARDWARE "
                    "run, launch with use_sim_time:=false or the node will hang "
                    "waiting for a clock that never arrives.");
    }

    // 5. One-shot startup delay timer to give Gazebo and controller time to stabilize.
    //    Sim-time timer: rclcpp::create_timer bound to the node clock (get_clock())
    //    honours use_sim_time, whereas create_wall_timer is contractually a
    //    steady_clock timer regardless of use_sim_time. Launch this node with
    //    use_sim_time:=true so the startup delay counts sim seconds off /clock.
    //    When it fires, startTimer() then waits for the one-shot target retarget
    //    (re-arming this handle as a short poll timer) before starting the rollout.
    startup_timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        rclcpp::Duration::from_seconds(startup_delay_sec_),
        std::bind(&DmpGazeboExecutorNode::startTimer, this));
}

void DmpGazeboExecutorNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (target_odom_received_) return;  // one-shot: ignore every message after the first

    target_position_ = Eigen::Vector3d(msg->pose.pose.position.x,
                                       msg->pose.pose.position.y,
                                       msg->pose.pose.position.z);
    target_odom_received_ = true;
    // One-shot retarget: no continuous tracking. Drop the subscription so no
    // further odometry is processed for the rest of the rollout.
    target_odom_sub_.reset();

    RCLCPP_INFO(this->get_logger(),
                "Target odometry received (frame '%s'): position [%.4f, %.4f, %.4f]; "
                "unsubscribed (one-shot retarget).",
                msg->header.frame_id.c_str(),
                target_position_.x(), target_position_.y(), target_position_.z());
}

void DmpGazeboExecutorNode::startTimer() {
    if (startup_timer_) startup_timer_->cancel();

    if (!target_odom_required_) {
        // No target to retarget toward: replay the demonstration's ORIGINAL goal
        // (already present in the weights loaded from YAML - dmp_.goal() is left
        // untouched, setGoal() is never called). No odometry subscription, no
        // wait/poll, and the fail-loud timeout does not apply - there is nothing
        // to wait for.
        RCLCPP_INFO(this->get_logger(),
                    "target_odom_required=false: replaying with the DEMO'S ORIGINAL "
                    "GOAL, no retarget applied.");
        RCLCPP_INFO(this->get_logger(), "Starting DMP rollout.");
        // Same sim-time step timer as the retarget path below (see note there).
        step_timer_ = rclcpp::create_timer(
            this, this->get_clock(),
            rclcpp::Duration::from_seconds(dt_),
            std::bind(&DmpGazeboExecutorNode::stepCallback, this));
        return;
    }

    // startup_delay_sec_ has elapsed. Do NOT start stepping until the one-shot
    // target retarget has happened: wait for the first free_target_object
    // odometry, re-polling on the node clock, subject to a fail-loud timeout.
    if (!odom_wait_started_) {
        odom_wait_start_ = this->now();
        odom_wait_started_ = true;
    }

    if (!target_odom_received_) {
        const double waited = (this->now() - odom_wait_start_).seconds();
        if (waited >= target_odom_timeout_sec_) {
            RCLCPP_FATAL(this->get_logger(),
                         "No message on target odometry topic '%s' within %.2f s. "
                         "Refusing to replay toward the demonstration's frozen goal - "
                         "the one-shot target retarget is mandatory. Is "
                         "free_target_object running and bridged?",
                         target_odom_topic_.c_str(), target_odom_timeout_sec_);
            throw std::runtime_error(
                "dmp_gazebo_executor_node: target odometry retarget timed out on '" +
                target_odom_topic_ + "'");
        }
        // Re-poll shortly (node clock, honours use_sim_time like startup_timer_).
        startup_timer_ = rclcpp::create_timer(
            this, this->get_clock(),
            rclcpp::Duration::from_seconds(0.1),
            std::bind(&DmpGazeboExecutorNode::startTimer, this));
        return;
    }

    // Goal-frame correction. target_position_ is in world_frame_, but
    // dmp_.setGoal() expects a goal in the demo-local frame of dmp_.y0(). Bridge
    // the two with the live EE pose (world_frame_ -> ee_frame_): the retarget
    // displacement measured in world is re-applied from y0_ in the demo-local
    // frame. If the TF is not available yet, re-poll on the SAME overall
    // odometry timeout (odom_wait_start_ / target_odom_timeout_sec_), exactly
    // like the "odometry not received" branch above - no separate TF timeout.
    geometry_msgs::msg::TransformStamped ee_tf;
    try {
        ee_tf = tf_buffer_->lookupTransform(world_frame_, ee_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        const double waited = (this->now() - odom_wait_start_).seconds();
        if (waited >= target_odom_timeout_sec_) {
            RCLCPP_FATAL(this->get_logger(),
                         "TF lookup '%s' -> '%s' still unavailable %.2f s after startup "
                         "(%s). Cannot compute the goal-frame correction - refusing to "
                         "replay the retarget. Is the robot state / TF tree being published?",
                         world_frame_.c_str(), ee_frame_.c_str(),
                         target_odom_timeout_sec_, ex.what());
            throw std::runtime_error(
                "dmp_gazebo_executor_node: TF lookup '" + world_frame_ + "' -> '" +
                ee_frame_ + "' timed out for the goal-frame correction");
        }
        // Re-poll shortly (node clock, honours use_sim_time like startup_timer_).
        startup_timer_ = rclcpp::create_timer(
            this, this->get_clock(),
            rclcpp::Duration::from_seconds(0.1),
            std::bind(&DmpGazeboExecutorNode::startTimer, this));
        return;
    }

    const Eigen::Vector3d ee_now(ee_tf.transform.translation.x,
                                 ee_tf.transform.translation.y,
                                 ee_tf.transform.translation.z);
    // Raw datum for future comparison against the pose the controller's
    // FrameAligner captures at its own activation (no comparison done here).
    RCLCPP_INFO(this->get_logger(),
                "Goal-frame correction: EE pose now (%s -> %s) = [%.4f, %.4f, %.4f] (world).",
                world_frame_.c_str(), ee_frame_.c_str(),
                ee_now.x(), ee_now.y(), ee_now.z());

    const Eigen::Vector3d new_dG_world = target_position_ - ee_now;
    const Eigen::Vector3d goal_corretto = dmp_.y0() + new_dG_world;

    // One-shot translational retarget, BEFORE any stepping. DMP::setGoal() is
    // self-contained: it recomputes scale_/scale_reliable_ (kMinDG and
    // amplitude-ratio guards, core/dmp.cpp) and stores goal_; it does NOT touch
    // the integration state. DMP::reset() does the opposite - it restores
    // x_/y_/z_/v_ and leaves goal_/scale_ alone. The two are independent, so the
    // order is not load-bearing; reset() is called last purely to keep an
    // explicit "rollout starts clean from y0" invariant (it is redundant with
    // the constructor's reset() since nothing has stepped yet).
    const Eigen::Vector3d original_goal = dmp_.goal();
    dmp_.setGoal(goal_corretto);
    dmp_.reset();

    RCLCPP_INFO(this->get_logger(),
                "One-shot target retarget: demo goal [%.4f, %.4f, %.4f] -> "
                "new goal [%.4f, %.4f, %.4f] (demo-local, passed to setGoal) | "
                "target_position [%.4f, %.4f, %.4f] (world, from '%s') | "
                "ee_now [%.4f, %.4f, %.4f] (world) | scale reliable x=%d y=%d z=%d "
                "| orientation goal left unchanged.",
                original_goal.x(), original_goal.y(), original_goal.z(),
                goal_corretto.x(), goal_corretto.y(), goal_corretto.z(),
                target_position_.x(), target_position_.y(), target_position_.z(),
                target_odom_topic_.c_str(),
                ee_now.x(), ee_now.y(), ee_now.z(),
                static_cast<int>(dmp_.isScaleReliable(0)),
                static_cast<int>(dmp_.isScaleReliable(1)),
                static_cast<int>(dmp_.isScaleReliable(2)));

    RCLCPP_INFO(this->get_logger(), "Starting DMP rollout.");
    // Sim-time timer (see note on startup_timer_): the rollout tick advances on
    // the node clock, so the integration step dt_ is a sim-time step and the
    // rollout is reproducible independently of the real-time factor.
    //
    // NOTE: tau (rollout duration) is historically computed from wall-clock demo
    // timestamps (core/dmp.cpp:79-84, core/quaternion_dmp.cpp:86-88) and is read
    // back verbatim from the weights YAML here (never recomputed). Any
    // dmp_weights.yaml recorded BEFORE this sim-time conversion carries a tau that
    // is no longer consistent with sim-time playback and must be re-recorded.
    step_timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        rclcpp::Duration::from_seconds(dt_),
        std::bind(&DmpGazeboExecutorNode::stepCallback, this));
}

void DmpGazeboExecutorNode::stepCallback() {
    if (finished_) return;

    // Gripper close ramp. A single step command slams the fingers shut and
    // ejects the object; instead walk core::gripper_ramp::interpolateGripperPosition
    // from open to closed over gripper_close_ramp_duration_sec_. No dedicated
    // timer: stepCallback already ticks at control_rate_hz_ and elapsed_ is a
    // monotonic sim-time base. Three states, gated by the same
    // gripper_trigger_t_ >= 0 && !gripper_trigger_sent_ guard as before.
    if (gripper_trigger_t_ >= 0.0 && !gripper_trigger_sent_) {
        if (!gripper_ramp_active_ && elapsed_ >= gripper_trigger_t_) {
            gripper_ramp_active_ = true;
            gripper_ramp_start_elapsed_ = elapsed_;
            RCLCPP_INFO(this->get_logger(),
                        "Replay: starting %.1f s gripper close ramp at elapsed=%.4f s "
                        "(target t=%.4f s)",
                        gripper_close_ramp_duration_sec_, elapsed_, gripper_trigger_t_);
        }
        if (gripper_ramp_active_) {
            const double ramp_elapsed = elapsed_ - gripper_ramp_start_elapsed_;
            std_msgs::msg::Float64 cmd;
            if (ramp_elapsed >= gripper_close_ramp_duration_sec_) {
                // Exact closed value regardless of tick granularity, then stop
                // ramping (matches the old one-shot guard: block never re-runs).
                cmd.data = gripper_closed_position_;
                gripper_pub_->publish(cmd);
                gripper_ramp_active_ = false;
                gripper_trigger_sent_ = true;
                // Physical command first (above), then announce the ramp is
                // complete so grasp_force_calibration_node captures force on
                // real contact.
                gripper_close_complete_pub_->publish(std_msgs::msg::Empty());
                RCLCPP_INFO(this->get_logger(),
                            "Replay: gripper close ramp complete (%.3f) at elapsed=%.4f s",
                            gripper_closed_position_, elapsed_);
            } else {
                cmd.data = core::gripper_ramp::interpolateGripperPosition(
                    ramp_elapsed, gripper_close_ramp_duration_sec_,
                    gripper_open_position_, gripper_closed_position_);
                gripper_pub_->publish(cmd);
            }
        }
    }

    geometry_msgs::msg::PoseStamped msg;
    const rclcpp::Time stamp = this->now();
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;

    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = stamp;  // SAME stamp as msg - lets the consumer detect
                                      // cross-topic skew between the two samples.
    twist_msg.header.frame_id = frame_id_;

    bool at_end = (elapsed_ + dt_) >= dmp_.tau();

    if (!at_end) {
        // Step translational and rotational DMPs forward by dt
        Eigen::Vector3d ct = Eigen::Vector3d::Zero();
        double cc = 0.0;
        Eigen::Vector3d pos = dmp_.step(dt_, ct, cc);
        Eigen::Quaterniond quat = qdmp_.step(dt_);
        // Analytic velocity state, already integrated inside step() above (see
        // core::DMP::velocity() / core::QuaternionDMP::omega()) - not a
        // finite-difference reconstruction.
        Eigen::Vector3d vel = dmp_.velocity();
        Eigen::Vector3d omega = qdmp_.omega();
        elapsed_ += dt_;

        msg.pose.position.x = pos.x();
        msg.pose.position.y = pos.y();
        msg.pose.position.z = pos.z();
        msg.pose.orientation.w = quat.w();
        msg.pose.orientation.x = quat.x();
        msg.pose.orientation.y = quat.y();
        msg.pose.orientation.z = quat.z();

        twist_msg.twist.linear.x = vel.x();
        twist_msg.twist.linear.y = vel.y();
        twist_msg.twist.linear.z = vel.z();
        twist_msg.twist.angular.x = omega.x();
        twist_msg.twist.angular.y = omega.y();
        twist_msg.twist.angular.z = omega.z();
    } else {
        // Clamp explicitly to goal attractor once tau is reached
        Eigen::Vector3d goal = dmp_.goal();
        msg.pose.position.x = goal.x();
        msg.pose.position.y = goal.y();
        msg.pose.position.z = goal.z();
        Eigen::Quaterniond qgoal = qdmp_.goal();
        msg.pose.orientation.w = qgoal.w();
        msg.pose.orientation.x = qgoal.x();
        msg.pose.orientation.y = qgoal.y();
        msg.pose.orientation.z = qgoal.z();

        // Target is at rest at the goal: feedforward is explicitly zero here,
        // NOT the last pre-clamp rollout velocity - otherwise a feedforward-fed
        // controller would keep pushing past the goal for one tick.
        // twist_msg.twist is already zero-initialized by TwistStamped's default
        // construction; left explicit (all six fields) so the "target is at
        // rest" invariant is visible at the call site, not implicit in a
        // default constructor a future reader might not think to check.
        twist_msg.twist.linear.x = 0.0;
        twist_msg.twist.linear.y = 0.0;
        twist_msg.twist.linear.z = 0.0;
        twist_msg.twist.angular.x = 0.0;
        twist_msg.twist.angular.y = 0.0;
        twist_msg.twist.angular.z = 0.0;

        finished_ = true;
        RCLCPP_INFO(this->get_logger(), "DMP rollout completed at goal.");
    }

    pose_pub_->publish(msg);
    twist_pub_->publish(twist_msg);

    if (finished_ && step_timer_) {
        step_timer_->cancel();
    }
}

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning