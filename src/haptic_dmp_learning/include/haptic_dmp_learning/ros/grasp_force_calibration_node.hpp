#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace haptic_dmp_learning {
namespace ros_wrapper {

/**
 * @brief Per-demonstration contact-force signature: calibrate or verify.
 *
 * @details
 * Two modes selected by the mandatory `mode` parameter:
 *
 * - `calibrate`: on the first rising edge of the geometric-grasp signal, collect
 *   every contact-wrench force-norm sample that arrives within a
 *   `capture_window_sec` time window (node clock; sim time under
 *   use_sim_time:=true), take their MEDIAN, and write it to a companion YAML
 *   `<calibration_dir>/force_calibration_<run_id>.yaml`. Zero messages in the
 *   window is an explicit error, never a median over an empty buffer.
 * - `verify`: load that companion file for the same `run_id`; on each rising edge
 *   recompute the median force norm over the capture window and publish
 *   `std_msgs/Bool` on `~/grasp_force_verified` — True iff
 *   |median - f_calib| <= max(tolerance_ratio * f_calib, tolerance_floor_n).
 *   If the companion file is missing the node stays up, warns, and publishes
 *   False (it never blocks).
 *
 * Fail-loud (project convention, see haptic_dmp_learning/DESIGN_NOTES.md):
 * `mode` and `run_id` are mandatory; a missing/invalid value throws out of the
 * constructor and main() exits non-zero. Nothing else silently defaults on the
 * critical path.
 *
 * This node is a pure observer of grasp_monitoring and franka_cartesian_control
 * topics — it never publishes onto a control topic.
 */
class GraspForceCalibrationNode : public rclcpp::Node {
public:
    GraspForceCalibrationNode();

private:
    enum class Mode { kCalibrate, kVerify };

    void geometricCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void forceCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
    void finishCaptureWindow();  ///< closes the time window, fired by capture_timer_
    void onWindowComplete(double median_force_norm);

    void writeCalibrationYaml(double median_force_norm) const;
    bool loadCalibrationYaml();  ///< returns true if f_calib_ was populated
    static double median(std::vector<double> v);

    // ROS interfaces
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr geom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr verified_pub_;  ///< verify mode only
    rclcpp::TimerBase::SharedPtr capture_timer_;  ///< one-shot, closes the capture window

    // Parameters
    Mode mode_;
    std::string run_id_;
    std::string geometric_confirmed_topic_;
    std::string force_estimate_topic_;
    double capture_window_sec_;
    std::string calibration_dir_;
    double tolerance_ratio_;
    double tolerance_floor_n_;
    double min_valid_force_n_;  ///< reject a capture whose median is below this
    std::string calibration_file_path_;

    // Rising-edge detection + one-shot-per-edge debounce
    bool have_prev_geom_ = false;
    bool prev_geom_ = false;
    bool armed_ = true;  ///< false after a trigger, re-armed when the signal drops

    // Capture window
    bool capturing_ = false;
    rclcpp::Time capture_start_time_;
    std::vector<double> buffer_;

    // verify mode state
    bool calib_loaded_ = false;
    double f_calib_ = 0.0;
    bool warned_missing_file_ = false;
    bool last_verified_ = false;
};

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning
