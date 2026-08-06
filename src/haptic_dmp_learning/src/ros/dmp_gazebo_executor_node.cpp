#include "haptic_dmp_learning/ros/dmp_gazebo_executor_node.hpp"
#include "haptic_dmp_learning/core/dmp_io.hpp"

#include <cstdlib>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace haptic_dmp_learning {
namespace ros_wrapper {

DmpGazeboExecutorNode::DmpGazeboExecutorNode()
    : Node("dmp_gazebo_executor_node"),
      dmp_(20, 4.6, 25.0, 6.25, false),   // placeholders; overwritten by loadFromYaml below
      qdmp_(20, 4.6, 25.0, 6.25),
      dt_(0.005),
      elapsed_(0.0),
      finished_(false) {

    const char* home = std::getenv("HOME");
    std::string default_weights_path = std::string(home ? home : "/root") + "/thesis_ws/dmp_weights.yaml";

    weights_yaml_path_ = this->declare_parameter<std::string>("weights_yaml_path", default_weights_path);
    target_pose_topic_ = this->declare_parameter<std::string>("target_pose_topic", "/target_pose");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "panda_link0");
    control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 200.0);
    startup_delay_sec_ = this->declare_parameter<double>("startup_delay_sec", 1.0);

    dt_ = 1.0 / control_rate_hz_;

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

    dmp_.reset();
    qdmp_.reset();

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        target_pose_topic_, rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(),
                "dmp_gazebo_executor_node ready. Weights: %s | tau: %.3f s | rate: %.1f Hz | "
                "publishing on %s in %.1f s",
                weights_yaml_path_.c_str(), dmp_.tau(), control_rate_hz_,
                target_pose_topic_.c_str(), startup_delay_sec_);

    startup_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(startup_delay_sec_),
        std::bind(&DmpGazeboExecutorNode::startTimer, this));
}

void DmpGazeboExecutorNode::startTimer() {
    startup_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Starting DMP rollout.");
    step_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(dt_),
        std::bind(&DmpGazeboExecutorNode::stepCallback, this));
}

void DmpGazeboExecutorNode::stepCallback() {
    if (finished_) return;

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;

    bool at_end = (elapsed_ + dt_) >= dmp_.tau();

    if (!at_end) {
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
        // Snap exactly to the learned goal instead of integrating past tau,
        // where the forcing term is no longer trustworthy (see open
        // post-tau divergence investigation).
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