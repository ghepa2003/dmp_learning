#include "haptic_dmp_learning/ros/haptic_dmp_wrapper_node.hpp"
#include "haptic_dmp_learning/core/dmp_io.hpp"

#include <cstdlib>
#include <fstream>

namespace haptic_dmp_learning {
namespace ros_wrapper {

HapticDmpWrapperNode::HapticDmpWrapperNode()
    : 
      // Initialize the ROS node with the name "haptic_dmp_wrapper_node"
      Node("haptic_dmp_wrapper_node"),
      dmp_(20, 4.6, 25.0, 6.25, false),  // default DMP parameters; will be overridden by ROS2 params
      recording_(false) {

    // Parameters - override via a params.yaml or -p on the command line.
    // First value is from the params.yaml, second is the default if not specified there.
    n_basis_ = this->declare_parameter<int>("n_basis", 20);
    alpha_x_ = this->declare_parameter<double>("alpha_x", 4.6);
    alpha_z_ = this->declare_parameter<double>("alpha_z", 25.0);
    beta_z_ = this->declare_parameter<double>("beta_z", 6.25);
    second_order_canonical_ = this->declare_parameter<bool>("second_order_canonical", false);

    // Default output paths: allow override via ROS2 parameter.
    const char* home = std::getenv("HOME");
    std::string default_yaml_path = std::string(home ? home : "/root") + "/thesis_ws/dmp_weights.yaml";
    std::string default_csv_path = std::string(home ? home : "/root") + "/thesis_ws/demo_raw.csv";
    output_yaml_path_ = this->declare_parameter<std::string>("output_yaml_path", default_yaml_path);
    output_demo_csv_path_ = this->declare_parameter<std::string>("output_demo_csv_path", default_csv_path);

    std::string default_features_path = std::string(home ? home : "/root") + "/thesis_ws/dmp_features.yaml";
    feature_flags_path_ = this->declare_parameter<std::string>("feature_flags_path", default_features_path);

    // Initialize the DMP with the specified parameters
    dmp_ = core::DMP(n_basis_, alpha_x_, alpha_z_, beta_z_, second_order_canonical_);

    // Applica i feature flag (es. ridge regression) da YAML separato dai
    // pesi. File assente = nessuna modifica, restano i default (LWR
    // indipendente) - attivazione opt-in, non richiede questo file per
    // funzionare come prima.
    core::dmp_io::applyFeatureConfig(feature_flags_path_, dmp_, quat_dmp_);

    // Initialize the subscriptions to the Geomagic Touch topics
    // NOTE: /touch0/pose is published at ~1000 Hz -> best-effort sensor QoS,
    // not the default reliable one (avoids unnecessary buffering/backlog).
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/touch0/pose", rclcpp::SensorDataQoS(),
        std::bind(&HapticDmpWrapperNode::poseCallback, this, std::placeholders::_1));

    // Initialize the subscription to the buttons topic with a queue size of 10
    buttons_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/touch0/buttons", 10,
        std::bind(&HapticDmpWrapperNode::buttonsCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
    
                "haptic_dmp_wrapper_node ready. Press button 0 to start recording a demo, "
                "button 1 to stop and learn the DMP. DMP output: %s | Demo CSV: %s",
                output_yaml_path_.c_str(), output_demo_csv_path_.c_str());
}

void HapticDmpWrapperNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // If not currently recording, ignore the pose messages
    if (!recording_) return;

    // Prefer the message's own timestamp; fall back to reception time if the
    // publisher never populated header.stamp (common with some drivers).
    rclcpp::Time now = msg->header.stamp;
    if (now.nanoseconds() == 0) {
        now = this->now();
    }

    // Create a Sample object with the current time and position, and add it to the recorder
    // The orientation is also captured and normalized.
    core::Sample s;
    s.t = (now - record_start_time_).seconds();
    s.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Eigen::Quaterniond orient(msg->pose.orientation.w, msg->pose.orientation.x,
                           msg->pose.orientation.y, msg->pose.orientation.z);
    s.orientation = orient.normalized();
    recorder_.addSample(s);
}

void HapticDmpWrapperNode::buttonsCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Check that the buttons message has at least 2 entries; if not, log a warning and return
    if (msg->buttons.size() < 2) {
        RCLCPP_WARN_ONCE(this->get_logger(),
                          "Expected at least 2 entries in /touch0/buttons, got %zu",
                          msg->buttons.size());
        return;
    }

    // If this is the first buttons message received, initialize prev_buttons_ and return
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

void HapticDmpWrapperNode::startRecording() {
    recorder_.clear();
    recording_ = true;
    record_start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Recording started.");
}

void HapticDmpWrapperNode::stopRecordingAndLearn() {
    // Stop recording and learn the DMP from the recorded demonstration
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
        std::cerr << "[SAVE diag] dmp.z0() norm before saveToYaml = " << dmp_.z0().norm() << "\n";
        core::dmp_io::saveToYaml(dmp_, quat_dmp_, output_yaml_path_);
        RCLCPP_INFO(this->get_logger(), "DMP + Quaternion DMP learned and saved to %s", output_yaml_path_.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Learning/saving failed: %s", e.what());
    }
}

void HapticDmpWrapperNode::saveDemoToCsv(const std::string& path) const {
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
