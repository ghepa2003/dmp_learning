#include "haptic_dmp_learning/ros/live_demo_recorder_node.hpp"
#include "haptic_dmp_learning/core/dmp_io.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>
#include <fstream>

namespace haptic_dmp_learning {
namespace ros_wrapper {

LiveDemoRecorderNode::LiveDemoRecorderNode()
    : Node("live_demo_recorder_node"),
      dmp_(20, 4.6, 25.0, 6.25, false),
      recording_(false) {

    // 1. Declare DMP hyper-parameters
    n_basis_ = this->declare_parameter<int>("n_basis", 20);
    alpha_x_ = this->declare_parameter<double>("alpha_x", 4.6);
    alpha_z_ = this->declare_parameter<double>("alpha_z", 25.0);
    beta_z_ = this->declare_parameter<double>("beta_z", 6.25);

    // 2. Declare topic names and output file paths
    master_pose_topic_ = this->declare_parameter<std::string>("master_pose_topic", "/master_pose_raw");
    target_pose_topic_ = this->declare_parameter<std::string>("target_pose_topic", "/target_pose");
    buttons_topic_ = this->declare_parameter<std::string>("buttons_topic", "/touch0/buttons");

    // Absolute defaults so behavior does not depend on the process's current
    // working directory at launch.
    const char* home = std::getenv("HOME");
    const std::string ws_root = std::string(home ? home : "/root") + "/thesis_ws";
    output_yaml_path_ = this->declare_parameter<std::string>("output_yaml_path", ws_root + "/live_demo_dmp_weights.yaml");
    output_demo_csv_path_ = this->declare_parameter<std::string>("output_demo_csv_path", ws_root + "/live_demo_raw.csv");

    // 3. Locate feature flags configuration YAML (absolute default;
    // dmp_io::applyFeatureConfig has its own fallback chain and will refuse
    // to continue rather than silently reverting to unfiltered LWR)
    std::string default_features_path;
    try {
        default_features_path = ament_index_cpp::get_package_share_directory("haptic_dmp_learning") + "/config/dmp_features.yaml";
    } catch (const std::exception&) {
        default_features_path = ws_root + "/src/haptic_dmp_learning/config/dmp_features.yaml";
    }
    feature_flags_path_ = this->declare_parameter<std::string>("feature_flags_path", default_features_path);

    // 4. Initialize core DMP objects and apply algorithm feature flags
    dmp_ = core::DMP(n_basis_, alpha_x_, alpha_z_, beta_z_);
    quat_dmp_ = core::QuaternionDMP(n_basis_, alpha_x_, alpha_z_, beta_z_);
    core::dmp_io::applyFeatureConfig(feature_flags_path_, dmp_, quat_dmp_);

    // 5. Subscribe to raw master pose (~1 kHz, sensor QoS)
    master_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        master_pose_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LiveDemoRecorderNode::masterPoseCallback, this, std::placeholders::_1));

    // 6. Subscribe to button events
    buttons_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        buttons_topic_, 10,
        std::bind(&LiveDemoRecorderNode::buttonsCallback, this, std::placeholders::_1));

    // 7. Publisher for mirroring master poses to Gazebo controller
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

    // Immediately forward pose onto /target_pose for live visualization in Gazebo
    target_pose_pub_->publish(*msg);

    rclcpp::Time now = msg->header.stamp;
    if (now.nanoseconds() == 0) {
        now = this->now();
    }

    // Record sample
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
        return;
    }

    // Rising-edge detection
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

    // Save demonstration trajectory to CSV
    if (!output_demo_csv_path_.empty()) {
        try {
            saveDemoToCsv(output_demo_csv_path_);
            RCLCPP_INFO(this->get_logger(), "Raw demo saved to %s", output_demo_csv_path_.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Saving raw demo failed: %s", e.what());
        }
    }

    // Fit DMP models
    try {
        dmp_.learnFromDemonstration(recorder_.samples());
        quat_dmp_.learnFromDemonstration(recorder_.samples());

        if (std::abs(dmp_.tau() - quat_dmp_.tau()) > 1e-6) {
            RCLCPP_WARN(this->get_logger(),
                        "position tau (%.4f) and orientation tau (%.4f) do not match - "
                        "check demo timestamps.",
                        dmp_.tau(), quat_dmp_.tau());
        }

        const auto& dmp_diag = dmp_.diagnostics();
        const auto& qdmp_diag = quat_dmp_.diagnostics();
        RCLCPP_INFO(this->get_logger(),
                    "DMP learned: |vel(0)|=%.3f m/s, |vel(end)|=%.3f m/s, |z(0)|=%.3f | "
                    "QuaternionDMP: |eta(0)|=%.3f, |eta(end)|=%.3f",
                    dmp_diag.initial_vel_norm, dmp_diag.final_vel_norm, dmp_diag.initial_z_norm,
                    qdmp_diag.initial_eta_norm, qdmp_diag.final_eta_norm);

        // Serialize learned parameters to YAML
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
