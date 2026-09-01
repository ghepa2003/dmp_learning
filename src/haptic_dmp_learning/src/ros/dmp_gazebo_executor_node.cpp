#include "haptic_dmp_learning/ros/dmp_gazebo_executor_node.hpp"
#include "haptic_dmp_learning/core/dmp_io.hpp"

#include <cstdlib>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>

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

    // 2b. Parse demo CSV for gripper trigger timestamp if provided
    demo_csv_path_ = this->declare_parameter<std::string>("demo_csv_path", "");
    if (!demo_csv_path_.empty()) {
        std::ifstream f(demo_csv_path_);
        if (f.is_open()) {
            std::string line;
            if (std::getline(f, line)) {
                std::stringstream ss(line);
                std::string col;
                int trigger_col_idx = -1;
                int curr_idx = 0;
                while (std::getline(ss, col, ',')) {
                    while (!col.empty() && (col.back() == '\r' || col.back() == ' ')) col.pop_back();
                    if (col == "gripper_trigger") {
                        trigger_col_idx = curr_idx;
                        break;
                    }
                    curr_idx++;
                }

                if (trigger_col_idx >= 0) {
                    while (std::getline(f, line)) {
                        if (line.empty()) continue;
                        std::stringstream lss(line);
                        std::string field;
                        std::vector<std::string> fields;
                        while (std::getline(lss, field, ',')) {
                            while (!field.empty() && (field.back() == '\r' || field.back() == ' ')) field.pop_back();
                            fields.push_back(field);
                        }
                        if (static_cast<int>(fields.size()) > trigger_col_idx) {
                            try {
                                if (std::stoi(fields[trigger_col_idx]) == 1) {
                                    gripper_trigger_t_ = std::stod(fields[0]);
                                    RCLCPP_INFO(this->get_logger(),
                                                "Found gripper trigger in CSV at t = %.4f s",
                                                gripper_trigger_t_);
                                    break;
                                }
                            } catch (...) {}
                        }
                    }
                } else {
                    RCLCPP_INFO(this->get_logger(),
                                "No 'gripper_trigger' column in %s; replay without gripper trigger.",
                                demo_csv_path_.c_str());
                }
            }
        } else {
            RCLCPP_WARN(this->get_logger(),
                        "Could not open demo CSV at %s; replay without gripper trigger.",
                        demo_csv_path_.c_str());
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

    if (gripper_trigger_t_ >= 0.0 && !gripper_trigger_sent_ && elapsed_ >= gripper_trigger_t_) {
        std_msgs::msg::Float64 cmd;
        cmd.data = 0.0;
        gripper_pub_->publish(cmd);
        gripper_trigger_sent_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "Replay: gripper trigger published at elapsed=%.4f s (target t=%.4f s)",
                    elapsed_, gripper_trigger_t_);
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