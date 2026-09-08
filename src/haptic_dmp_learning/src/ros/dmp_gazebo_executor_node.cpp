#include "haptic_dmp_learning/ros/dmp_gazebo_executor_node.hpp"
#include "haptic_dmp_learning/core/dmp_io.hpp"
#include "haptic_dmp_learning/core/demo_csv_io.hpp"
#include "haptic_dmp_learning/core/gripper_ramp.hpp"

#include <cstdlib>
#include <chrono>
#include <cmath>
#include <fstream>

#include <rclcpp/create_timer.hpp>

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
    frame_id_ = this->declare_parameter<std::string>("frame_id", "panda_link0");
    control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 200.0);
    startup_delay_sec_ = this->declare_parameter<double>("startup_delay_sec", 1.0);

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
    gripper_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/gripper_position_cmd", rclcpp::QoS(10));
    // One-shot announce that the gripper close ramp has finished (see header).
    gripper_close_complete_pub_ = this->create_publisher<std_msgs::msg::Empty>(
        "/gripper_close_complete", rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(),
                "dmp_gazebo_executor_node ready. Weights: %s | tau: %.3f s | rate: %.1f Hz | "
                "publishing on %s in %.1f s",
                weights_yaml_path_.c_str(), dmp_.tau(), control_rate_hz_,
                target_pose_topic_.c_str(), startup_delay_sec_);

    // 5. One-shot startup delay timer to give Gazebo and controller time to stabilize.
    //    Sim-time timer: rclcpp::create_timer bound to the node clock (get_clock())
    //    honours use_sim_time, whereas create_wall_timer is contractually a
    //    steady_clock timer regardless of use_sim_time. Launch this node with
    //    use_sim_time:=true so the startup delay counts sim seconds off /clock.
    startup_timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        rclcpp::Duration::from_seconds(startup_delay_sec_),
        std::bind(&DmpGazeboExecutorNode::startTimer, this));
}

void DmpGazeboExecutorNode::startTimer() {
    startup_timer_->cancel();
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
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;

    bool at_end = (elapsed_ + dt_) >= dmp_.tau();

    if (!at_end) {
        // Step translational and rotational DMPs forward by dt
        Eigen::Vector3d ct = Eigen::Vector3d::Zero();
        double cc = 0.0;
        Eigen::Vector3d pos = dmp_.step(dt_, ct, cc);
        Eigen::Quaterniond quat = qdmp_.step(dt_);
        elapsed_ += dt_;

        msg.pose.position.x = pos.x();
        msg.pose.position.y = pos.y();
        msg.pose.position.z = pos.z();
        msg.pose.orientation.w = quat.w();
        msg.pose.orientation.x = quat.x();
        msg.pose.orientation.y = quat.y();
        msg.pose.orientation.z = quat.z();
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

        finished_ = true;
        RCLCPP_INFO(this->get_logger(), "DMP rollout completed at goal.");
    }

    pose_pub_->publish(msg);

    if (finished_ && step_timer_) {
        step_timer_->cancel();
    }
}

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning