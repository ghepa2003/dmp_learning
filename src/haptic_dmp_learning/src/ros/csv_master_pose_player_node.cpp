#include "haptic_dmp_learning/ros/csv_master_pose_player_node.hpp"

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

using namespace std::chrono_literals;

namespace haptic_dmp_learning {
namespace ros_wrapper {

namespace {
// Idle gap between the baseline joy message and the start button rising edge
constexpr double kStartEdgeDelaySec = 0.2;
}  // namespace

CsvMasterPosePlayerNode::CsvMasterPosePlayerNode()
    : Node("csv_master_pose_player_node"),
      next_row_idx_(0),
      finished_(false) {

    // 1. Declare ROS parameters
    demo_csv_path_ = this->declare_parameter<std::string>("demo_csv_path", "reach_task_baseline.csv");
    master_pose_topic_ = this->declare_parameter<std::string>("master_pose_topic", "/master_pose_raw");
    buttons_topic_ = this->declare_parameter<std::string>("buttons_topic", "/touch0/buttons");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "panda_link0");
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 200.0);
    startup_delay_sec_ = this->declare_parameter<double>("startup_delay_sec", 1.0);

    dt_ = 1.0 / publish_rate_hz_;

    // 2. Load demo trajectory from CSV
    loadCsv(demo_csv_path_);

    // 3. Create publishers for pose and button events
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        master_pose_topic_, rclcpp::QoS(10));
    buttons_pub_ = this->create_publisher<sensor_msgs::msg::Joy>(
        buttons_topic_, rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(),
                "csv_master_pose_player_node ready. Demo CSV: %s (%zu rows) | publishing on %s "
                "in %.1f s, buttons on %s",
                demo_csv_path_.c_str(), rows_.size(), master_pose_topic_.c_str(),
                startup_delay_sec_, buttons_topic_.c_str());

    // 4. Initial delay timer: emits baseline idle joy message before triggering start edge
    startup_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(startup_delay_sec_), [this]() {
            startup_timer_->cancel();
            // Publish idle baseline [0, 0] so downstream rising-edge detector initializes
            publishButtons(0, 0);
            start_edge_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(kStartEdgeDelaySec),
                [this]() {
                    start_edge_timer_->cancel();
                    beginPlayback();
                });
        });
}

void CsvMasterPosePlayerNode::loadCsv(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("CsvMasterPosePlayerNode: cannot open demo CSV: " + path);
    }

    std::string line;
    std::getline(f, line);  // Discard header line: t,x,y,z,qw,qx,qy,qz

    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string field;
        std::vector<double> values;
        while (std::getline(ss, field, ',')) {
            values.push_back(std::stod(field));
        }
        if (values.size() != 8) {
            throw std::runtime_error("CsvMasterPosePlayerNode: malformed row in " + path);
        }

        CsvRow row;
        row.t = values[0];
        row.position = Eigen::Vector3d(values[1], values[2], values[3]);
        row.orientation = Eigen::Quaterniond(values[4], values[5], values[6], values[7]).normalized();
        rows_.push_back(row);
    }

    if (rows_.empty()) {
        throw std::runtime_error("CsvMasterPosePlayerNode: no rows loaded from " + path);
    }
}

void CsvMasterPosePlayerNode::publishButtons(int32_t button0, int32_t button1) {
    sensor_msgs::msg::Joy msg;
    msg.header.stamp = this->now();
    msg.buttons = {button0, button1};
    buttons_pub_->publish(msg);
}

void CsvMasterPosePlayerNode::publishRow(const CsvRow& row) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    msg.pose.position.x = row.position.x();
    msg.pose.position.y = row.position.y();
    msg.pose.position.z = row.position.z();
    msg.pose.orientation.w = row.orientation.w();
    msg.pose.orientation.x = row.orientation.x();
    msg.pose.orientation.y = row.orientation.y();
    msg.pose.orientation.z = row.orientation.z();
    pose_pub_->publish(msg);
}

void CsvMasterPosePlayerNode::beginPlayback() {
    RCLCPP_INFO(this->get_logger(), "Playback started.");
    publishButtons(1, 0);  // Emulate button 0 rising edge (Start Recording)

    playback_start_time_ = this->now();
    next_row_idx_ = 0;
    finished_ = false;

    playback_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(dt_),
        std::bind(&CsvMasterPosePlayerNode::playbackCallback, this));
}

void CsvMasterPosePlayerNode::playbackCallback() {
    if (finished_) return;

    // Advance to latest CSV row matching elapsed wall-clock time
    double elapsed = (this->now() - playback_start_time_).seconds();
    while (next_row_idx_ + 1 < rows_.size() && rows_[next_row_idx_ + 1].t <= elapsed) {
        ++next_row_idx_;
    }
    publishRow(rows_[next_row_idx_]);

    // Check for completion
    if (elapsed >= rows_.back().t) {
        finished_ = true;
        playback_timer_->cancel();
        publishButtons(0, 1);  // Emulate button 1 rising edge (Stop & Train DMP)
        RCLCPP_INFO(this->get_logger(), "Playback finished (end of file).");
    }
}

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning
