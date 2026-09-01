#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "haptic_dmp_learning/core/types.hpp"

namespace haptic_dmp_learning {
namespace ros_wrapper {

/**
 * @brief Synthetic Teleoperation Source: Streams recorded CSV trajectories and emulates haptic button events.
 *
 * @details
 * Purpose & Architecture:
 * - Allows full testing of the teleoperation learning pipeline without requiring a physical Geomagic Touch device connected.
 * - Loads a previously recorded CSV demonstration containing timestamps, positions [x, y, z], and quaternions [qw, qx, qy, qz].
 * - Publishes Cartesian poses to `/master_pose_raw` using zero-order hold at `publish_rate_hz` matching the original elapsed time.
 * - Emulates Joy button messages on `/touch0/buttons` (button 0 press at start, button 1 press at finish), seamlessly
 *   triggering downstream recorder and DMP learning nodes (`LiveDemoRecorderNode`).
 */
class CsvMasterPosePlayerNode : public rclcpp::Node {
public:
    CsvMasterPosePlayerNode();

private:
    void beginPlayback();
    void playbackCallback();

    void publishButtons(int32_t button0, int32_t button1);
    void publishRow(const core::Sample& row);

    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr buttons_pub_;
    rclcpp::TimerBase::SharedPtr startup_timer_;
    rclcpp::TimerBase::SharedPtr start_edge_timer_;
    rclcpp::TimerBase::SharedPtr playback_timer_;

    // In-memory trajectory buffer
    std::vector<core::Sample> rows_;

    // Playback state
    rclcpp::Time playback_start_time_;
    size_t next_row_idx_;
    bool finished_;

    // Parameters
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
