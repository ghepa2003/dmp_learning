#pragma once

#include <sys/types.h>  // pid_t

#include <chrono>
#include <string>
#include <utility>
#include <vector>

#include <termios.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/float64.hpp>

namespace haptic_dmp_learning {
namespace ros_wrapper {

/**
 * @brief Time-synchronises the demo and replay flows against a moving target
 *        object, anchoring both to the target's first odometry sim-time stamp.
 *
 * Preliminary fail-loud check (both modes, before anything else): subscribe to
 * /clock and wait for the first message within `clock_wait_timeout_sec`. /clock is
 * NOT bridged by any file in this workspace (it depends on external components
 * such as gz_ros2_control); if it never arrives the node logs FATAL and exits.
 *
 * `mode` = "demo":
 *   - launches the synchronised recording stack (demo_replay_sync.launch.py) with
 *     the generated run_id;
 *   - watches /touch0/buttons for the same rising-edge logic as
 *     haptic_dmp_wrapper_node (buttons[0] start, buttons[1] stop);
 *   - on start: spawn free_target_object, wait for its first odometry, wait until
 *     sim time reaches t0 + sync_delay_sec, then republish an equivalent start
 *     Joy on /touch0/buttons_synced;
 *   - on stop: republish immediately on /touch0/buttons_synced, no delay.
 *
 * `mode` = "replay":
 *   - no button wait; spawn target, wait odometry, wait t0 + sync_delay_sec;
 *   - then launch dmp_gazebo_executor_node (weights for this run_id,
 *     startup_delay_sec:=0.0, use_sim_time:=true) plus geometric_grasp_monitor,
 *     grasp_force_calibration_node (mode:=verify) and grasp_state_machine.
 *   - `run_id` is a MANDATORY input here (identifies the demo whose weights /
 *     calibration are replayed); it is not generated.
 */
class DemoReplaySyncOrchestratorNode : public rclcpp::Node {
public:
    DemoReplaySyncOrchestratorNode();
    ~DemoReplaySyncOrchestratorNode() override;

    bool failed() const { return failed_; }

private:
    enum class Mode { kDemo, kReplay };
    enum class Phase {
        kCheckClock,
        kLaunchRecording,   // demo only
        kWaitButtonStart,   // demo only
        kSpawnTarget,
        kWaitOdom,
        kWaitSync,
        kEmitStartSeed,     // demo only
        kEmitStartEdge,     // demo only
        kLaunchReplay,      // replay only
        kRunning,
        kDone,
        kError
    };

    // Orchestration --------------------------------------------------------
    void tick();
    void fatal(const std::string& why);

    // Subscriptions ------------------------------------------------------
    void clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void buttonsCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    // Helpers ------------------------------------------------------------
    std::string generateRunId() const;
    void publishSyncedButtons(int b0, int b1);
    void publishGripperTrigger();
    void checkKeyboard();
    pid_t launchChild(const std::vector<std::string>& argv, const std::string& tag);
    void stopChildren();
    std::string weightsPathForRunId() const;
    std::string demoCsvPathForRunId() const;

    // Parameters
    Mode mode_;
    std::string run_id_;
    double clock_wait_timeout_sec_;
    double odom_wait_timeout_sec_;
    double sync_delay_sec_;
    std::string target_name_;
    double tx_, ty_, tz_, tvx_, tvy_, tvz_, twx_, twy_, twz_;
    double tsx_, tsy_, tsz_, tmass_;  // free_target_object box size + mass
    std::string buttons_topic_;
    std::string buttons_synced_topic_;
    bool use_csv_playback_;
    double hard_force_limit_n_;  // forwarded to grasp_state_machine (replay)

    // ROS interfaces
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr buttons_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr buttons_synced_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_pub_;
    rclcpp::TimerBase::SharedPtr tick_timer_;
    rclcpp::TimerBase::SharedPtr gripper_init_timer_;
    rclcpp::TimerBase::SharedPtr keyboard_timer_;

    // State
    Phase phase_ = Phase::kCheckClock;
    bool failed_ = false;
    std::chrono::steady_clock::time_point node_start_;
    std::chrono::steady_clock::time_point phase_entered_;

    bool clock_seen_ = false;
    double last_clock_sec_ = 0.0;

    bool odom_seen_ = false;
    double t0_sim_ = 0.0;

    std::vector<int32_t> prev_buttons_;
    bool recording_ = false;

    bool gripper_trigger_sent_ = false;
    bool termios_modified_ = false;
    struct termios orig_termios_{};

    std::vector<std::pair<pid_t, std::string>> children_;
};

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning
