#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "haptic_dmp_learning/core/types.hpp"

namespace haptic_dmp_learning {
namespace ros_wrapper {

// Stand-in for the Geomagic Touch driver while no physical device is
// attached: loads a previously recorded demo CSV (t,x,y,z,qw,qx,qy,qz,
// non-uniform timestamps) and replays it with a zero-order hold onto
// /master_pose_raw, at the same wall-clock pace it was recorded.
//
// Also emits the button events a real device's start/stop presses would
// produce on /touch0/buttons (sensor_msgs/Joy), so that any downstream node
// listening for that mechanism (see live_demo_recorder_node) behaves
// identically whether the source is this CSV player or the real driver.
class CsvMasterPosePlayerNode : public rclcpp::Node {
public:
    CsvMasterPosePlayerNode();

private:
    void loadCsv(const std::string& path);
    void beginPlayback();
    void playbackCallback();

    struct CsvRow {
        double t;
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
    };

    void publishButtons(int32_t button0, int32_t button1);
    void publishRow(const CsvRow& row);

    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr buttons_pub_;
    rclcpp::TimerBase::SharedPtr startup_timer_;
    rclcpp::TimerBase::SharedPtr start_edge_timer_;
    rclcpp::TimerBase::SharedPtr playback_timer_;

    // demo data
    std::vector<CsvRow> rows_;

    // playback state
    rclcpp::Time playback_start_time_;
    size_t next_row_idx_;
    bool finished_;

    // params
    std::string demo_csv_path_;
    std::string master_pose_topic_;
    std::string buttons_topic_;
    std::string frame_id_;
    double publish_rate_hz_;
    double dt_;
    double startup_delay_sec_;
};

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning
