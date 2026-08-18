#include "haptic_dmp_learning/ros/live_demo_recorder_node.hpp"
#include "haptic_dmp_learning/core/dmp_io.hpp"
#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <fstream>

// This node is intended for live demonstration recording and DMP learning. 
// It subscribes to a high-rate master pose topic and a button topic, allowing the user to start and stop 
// recording demonstrations while visualizing the demonstrated trajectory on Gazebo. 
// The recorded data can be saved to a CSV file, and the learned DMP parameters are saved to 
// a YAML file for later use.

namespace haptic_dmp_learning {
namespace ros_wrapper {

// Constructor: declare parameters, set up subscriptions and publishers
LiveDemoRecorderNode::LiveDemoRecorderNode()
    : Node("live_demo_recorder_node"),
      dmp_(20, 4.6, 25.0, 6.25, false),  // default DMP parameters; will be overridden by ROS2 params
      recording_(false) {

    n_basis_ = this->declare_parameter<int>("n_basis", 20);
    alpha_x_ = this->declare_parameter<double>("alpha_x", 4.6);
    alpha_z_ = this->declare_parameter<double>("alpha_z", 25.0);
    beta_z_ = this->declare_parameter<double>("beta_z", 6.25);

    // Declare ROS2 parameters for topics and output paths
    master_pose_topic_ = this->declare_parameter<std::string>("master_pose_topic", "/master_pose_raw");
    target_pose_topic_ = this->declare_parameter<std::string>("target_pose_topic", "/target_pose");
    buttons_topic_ = this->declare_parameter<std::string>("buttons_topic", "/touch0/buttons");

    // Default output paths deliberately differ from haptic_dmp_wrapper_node's
    // defaults so a live-demo dry run doesn't clobber a real device session.
    const char* home = std::getenv("HOME");
    std::string default_yaml_path = std::string(home ? home : "/root") + "/thesis_ws/live_demo_dmp_weights.yaml";
    std::string default_csv_path = std::string(home ? home : "/root") + "/thesis_ws/live_demo_raw.csv";
    output_yaml_path_ = this->declare_parameter<std::string>("output_yaml_path", default_yaml_path);
    output_demo_csv_path_ = this->declare_parameter<std::string>("output_demo_csv_path", default_csv_path);

    // Load feature flags from a YAML file, defaulting to the package's config directory
    std::string default_features_path = std::string(home ? home : "/root") + "/thesis_ws/src/haptic_dmp_learning/config/dmp_features.yaml";
    if (!std::ifstream(default_features_path).good()) {
        default_features_path = std::string(home ? home : "/root") + "/thesis_ws/dmp_features.yaml";
    }
    feature_flags_path_ = this->declare_parameter<std::string>("feature_flags_path", default_features_path);

    // If n_basis_ is default 20, check if params.yaml exists and load n_basis from it
    if (n_basis_ == 20) {
        std::vector<std::string> params_candidates = {
            std::string(home ? home : "/root") + "/thesis_ws/src/haptic_dmp_learning/config/params.yaml",
            std::string(home ? home : "/root") + "/thesis_ws/params.yaml"
        };
        for (const auto& ppath : params_candidates) {
            std::ifstream check_f(ppath);
            if (check_f.good()) {
                try {
                    YAML::Node pnode = YAML::LoadFile(ppath);
                    if (pnode["live_demo_recorder_node"] && pnode["live_demo_recorder_node"]["ros__parameters"]) {
                        auto ros_p = pnode["live_demo_recorder_node"]["ros__parameters"];
                        if (ros_p["n_basis"]) n_basis_ = ros_p["n_basis"].as<int>();
                        if (ros_p["alpha_x"]) alpha_x_ = ros_p["alpha_x"].as<double>();
                        if (ros_p["alpha_z"]) alpha_z_ = ros_p["alpha_z"].as<double>();
                        if (ros_p["beta_z"]) beta_z_ = ros_p["beta_z"].as<double>();
                        break;
                    }
                } catch (...) {}
            }
        }
    }

    // Initialize DMP and QuaternionDMP with the loaded parameters
    dmp_ = core::DMP(n_basis_, alpha_x_, alpha_z_, beta_z_);
    quat_dmp_ = core::QuaternionDMP(n_basis_, alpha_x_, alpha_z_, beta_z_);

    // Same regression/filter feature flags (ridge + velocity filter) as
    // haptic_dmp_wrapper_node - see config/dmp_features.yaml.
    core::dmp_io::applyFeatureConfig(feature_flags_path_, dmp_, quat_dmp_);

    // NOTE: /master_pose_raw plays the same role /touch0/pose plays for
    // haptic_dmp_wrapper_node (high-rate raw master stream) -> same QoS choice.
    master_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        master_pose_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LiveDemoRecorderNode::masterPoseCallback, this, std::placeholders::_1));

    // NOTE: /touch0/buttons plays the same role /touch0/buttons plays for
    // haptic_dmp_wrapper_node (button stream) -> same QoS choice.
    buttons_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        buttons_topic_, 10,
        std::bind(&LiveDemoRecorderNode::buttonsCallback, this, std::placeholders::_1));

    // /target_pose is published at the same rate as /master_pose_raw, so a
    // QoS of 10 is sufficient for visualization purposes.
    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        target_pose_topic_, rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(),
                "live_demo_recorder_node ready. %s -> %s (visualization) | %s drives "
                "start/stop | DMP output: %s | Demo CSV: %s",
                master_pose_topic_.c_str(), target_pose_topic_.c_str(), buttons_topic_.c_str(),
                output_yaml_path_.c_str(), output_demo_csv_path_.c_str());
}

void LiveDemoRecorderNode::masterPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!recording_) return;

    // Republish for live visualization via the existing
    // CartesianVelocityController, only while the demo is being recorded.
    // The controller captures its one-shot device/robot alignment on the
    // first /target_pose message it ever receives after activation (see
    // CartesianVelocityController::update()).
    // NOTE: this is a one-way republish, not a feedback loop. The user is
    // expected to move the master device, not the robot, during live demo
    // recording. The robot will follow the master device's motion with a
    // fixed offset, as determined by the controller's one-shot alignment.
    target_pose_pub_->publish(*msg);

    rclcpp::Time now = msg->header.stamp;
    if (now.nanoseconds() == 0) {
        now = this->now();
    }

    core::Sample s;
    s.t = (now - record_start_time_).seconds();
    s.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Eigen::Quaterniond orient(msg->pose.orientation.w, msg->pose.orientation.x,
                           msg->pose.orientation.y, msg->pose.orientation.z);
    s.orientation = orient.normalized();
    recorder_.addSample(s);
}

void LiveDemoRecorderNode::buttonsCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->buttons.size() < 2) {
        RCLCPP_WARN_ONCE(this->get_logger(),
                          "Expected at least 2 entries in %s, got %zu",
                          buttons_topic_.c_str(), msg->buttons.size());
        return;
    }

    if (prev_buttons_.empty()) {
        prev_buttons_.assign(msg->buttons.begin(), msg->buttons.end());
        return;  // first message: just initialize state, nothing to trigger yet
    }

    bool rising0 = (msg->buttons[0] != 0) && (prev_buttons_[0] == 0);
    bool rising1 = (msg->buttons[1] != 0) && (prev_buttons_[1] == 0);

    prev_buttons_.assign(msg->buttons.begin(), msg->buttons.end());

    if (rising0 && !recording_) {
        startRecording();
    } else if (rising1 && recording_) {
        stopRecordingAndLearn();
    }
}

void LiveDemoRecorderNode::startRecording() {
    recorder_.clear();
    recording_ = true;
    record_start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Recording started.");
}

void LiveDemoRecorderNode::stopRecordingAndLearn() {
    recording_ = false;
    RCLCPP_INFO(this->get_logger(), "Recording stopped. %zu samples collected.", recorder_.size());

    if (recorder_.size() < 5) {
        RCLCPP_WARN(this->get_logger(), "Too few samples, discarding this demonstration.");
        return;
    }

    if (!output_demo_csv_path_.empty()) {
        try {
            saveDemoToCsv(output_demo_csv_path_);
            RCLCPP_INFO(this->get_logger(), "Raw demo saved to %s", output_demo_csv_path_.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Saving raw demo failed: %s", e.what());
        }
    }

    try {
        dmp_.learnFromDemonstration(recorder_.samples());
        quat_dmp_.learnFromDemonstration(recorder_.samples());

        if (std::abs(dmp_.tau() - quat_dmp_.tau()) > 1e-6) {
            RCLCPP_WARN(this->get_logger(),
                        "position tau (%.4f) and orientation tau (%.4f) do not match - "
                        "check demo timestamps.",
                        dmp_.tau(), quat_dmp_.tau());
        }
        core::dmp_io::saveToYaml(dmp_, quat_dmp_, output_yaml_path_);
        RCLCPP_INFO(this->get_logger(), "DMP + Quaternion DMP learned and saved to %s", output_yaml_path_.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Learning/saving failed: %s", e.what());
    }
}

void LiveDemoRecorderNode::saveDemoToCsv(const std::string& path) const {
    std::ofstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("saveDemoToCsv: cannot open file for writing: " + path);
    }
    f << "t,x,y,z,qw,qx,qy,qz\n";
    for (const auto& s : recorder_.samples()) {
        f << s.t << "," << s.position.x() << "," << s.position.y() << "," << s.position.z() << ","
        << s.orientation.w() << "," << s.orientation.x() << "," << s.orientation.y() << "," << s.orientation.z() << "\n";
    }
}

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning
