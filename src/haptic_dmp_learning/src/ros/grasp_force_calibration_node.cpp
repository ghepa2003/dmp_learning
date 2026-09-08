#include "haptic_dmp_learning/ros/grasp_force_calibration_node.hpp"

#include <rclcpp/create_timer.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <system_error>

namespace haptic_dmp_learning {
namespace ros_wrapper {

namespace {

// Fixed explanatory string stored in every calibration file. Documents the
// standing assumption behind comparing a demo force signature against a replay
// one (see the DESIGN_NOTES section added with this component).
std::string calibrationNote(const std::string& run_id) {
    return
        "Valid only while the joint-space path taken during replay stays close to "
        "the one of the demonstration recorded as run_id '" + run_id + "'. The "
        "contact-wrench estimate is sensorless and configuration-dependent (see "
        "franka_cartesian_control/DESIGN_NOTES.md). Re-record and re-calibrate when "
        "Floating Reference Point / Residual SAC change the replay trajectory.";
}

}  // namespace

GraspForceCalibrationNode::GraspForceCalibrationNode()
    : Node("grasp_force_calibration_node") {

    // 1. mode (mandatory, fail-loud) --------------------------------------------
    std::string mode_str;
    try {
        mode_str = this->declare_parameter<std::string>("mode");
    } catch (const std::exception& e) {
        throw std::runtime_error(
            "grasp_force_calibration_node: required parameter 'mode' "
            "('calibrate' | 'verify') was not provided. Underlying error: " +
            std::string(e.what()));
    }
    if (mode_str == "calibrate") {
        mode_ = Mode::kCalibrate;
    } else if (mode_str == "verify") {
        mode_ = Mode::kVerify;
    } else {
        throw std::runtime_error(
            "grasp_force_calibration_node: parameter 'mode' must be 'calibrate' or "
            "'verify', got '" + mode_str + "'.");
    }

    // 2. run_id (mandatory in both modes, fail-loud) ---------------------------
    try {
        run_id_ = this->declare_parameter<std::string>("run_id");
    } catch (const std::exception& e) {
        throw std::runtime_error(
            "grasp_force_calibration_node: required parameter 'run_id' was not "
            "provided (no silent default on the calibration identity). Underlying "
            "error: " + std::string(e.what()));
    }
    if (run_id_.empty()) {
        throw std::runtime_error(
            "grasp_force_calibration_node: parameter 'run_id' is empty.");
    }

    // 3. Remaining parameters (documented defaults) ---------------------------
    geometric_confirmed_topic_ = this->declare_parameter<std::string>(
        "geometric_confirmed_topic", "/geometric_grasp_monitor/geometric_grasp_confirmed");
    // One-shot event published by whichever node ran the gripper close ramp
    // (demo_replay_sync_orchestrator in demo, dmp_gazebo_executor_node in replay)
    // once the fingers reached the closed position. This is the capture trigger;
    // geometric_confirmed_topic_ above is only the gate.
    gripper_close_complete_topic_ = this->declare_parameter<std::string>(
        "gripper_close_complete_topic", "/gripper_close_complete");
    // NOTE: the sensorless wrench estimate is published by the *impedance*
    // controller node as "~/contact_wrench_estimate", i.e. the fully-qualified
    // name below (the controller_manager is not namespaced in the Franka Gazebo
    // bringup). Subscribing to a bare "~/contact_wrench_estimate" here would
    // resolve to this node's private namespace and receive nothing.
    force_estimate_topic_ = this->declare_parameter<std::string>(
        "force_estimate_topic", "/cartesian_impedance_controller/contact_wrench_estimate");
    // Time window over which force-norm samples are accumulated after the
    // geometric rising edge. A fixed message count would be ~8 ms at the 1 kHz
    // control-loop rate - far shorter than the tens-to-hundreds of ms scale of
    // human teleoperation jitter, so the samples would be strongly correlated
    // and the median would give no more robustness than a single sample.
    capture_window_sec_ = this->declare_parameter<double>("capture_window_sec", 0.25);
    if (!(capture_window_sec_ > 0.0)) {
        throw std::runtime_error(
            "grasp_force_calibration_node: capture_window_sec must be > 0.");
    }

    const char* home = std::getenv("HOME");
    const std::string ws_root = std::string(home ? home : "/root") + "/thesis_ws";
    calibration_dir_ = this->declare_parameter<std::string>(
        "calibration_dir", ws_root + "/calibrations");

    tolerance_ratio_ = this->declare_parameter<double>("tolerance_ratio", 0.3);
    // Floor chosen with margin above a maximum observed bias of ~2 N; provisional,
    // to be re-tuned with more data.
    tolerance_floor_n_ = this->declare_parameter<double>("tolerance_floor_n", 3.0);

    // Minimum median force norm for a capture to count as real contact rather than
    // configuration/tracking bias. Same value and rationale as tolerance_floor_n_
    // above (margin above the ~2 N configuration-dependent tracking bias documented
    // in franka_cartesian_control/DESIGN_NOTES.md); kept as a separate parameter so
    // the calibrate-side gate and the verify-side tolerance floor can diverge later.
    min_valid_force_n_ = this->declare_parameter<double>("min_valid_force_n", 3.0);

    calibration_file_path_ =
        calibration_dir_ + "/force_calibration_" + run_id_ + ".yaml";

    // 4. verify mode: load the companion file now (missing => warn, do not throw)
    if (mode_ == Mode::kVerify) {
        if (loadCalibrationYaml()) {
            RCLCPP_INFO(this->get_logger(),
                        "verify mode: loaded f_calib_norm=%.4f N from %s",
                        f_calib_, calibration_file_path_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(),
                        "verify mode: calibration file %s not found or unreadable; "
                        "the node stays up and publishes False on ~/grasp_force_verified "
                        "until a valid file for run_id '%s' exists.",
                        calibration_file_path_.c_str(), run_id_.c_str());
        }
        verified_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "~/grasp_force_verified", rclcpp::QoS(10));
    }

    // 5. Subscriptions ------------------------------------------------------
    geom_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        geometric_confirmed_topic_, rclcpp::QoS(10),
        std::bind(&GraspForceCalibrationNode::geometricCallback, this, std::placeholders::_1));
    gripper_close_complete_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        gripper_close_complete_topic_, rclcpp::QoS(10),
        std::bind(&GraspForceCalibrationNode::gripperCloseCompleteCallback, this,
                  std::placeholders::_1));
    force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        force_estimate_topic_, rclcpp::SensorDataQoS(),
        std::bind(&GraspForceCalibrationNode::forceCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "grasp_force_calibration_node ready | mode=%s | run_id=%s | "
                "geometric=%s | gripper_close_complete=%s | force=%s | window=%.3f s | "
                "file=%s",
                (mode_ == Mode::kCalibrate ? "calibrate" : "verify"),
                run_id_.c_str(), geometric_confirmed_topic_.c_str(),
                gripper_close_complete_topic_.c_str(),
                force_estimate_topic_.c_str(), capture_window_sec_,
                calibration_file_path_.c_str());
}

void GraspForceCalibrationNode::geometricCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    const bool geom = msg->data;

    // Gate only: remember the latest value. The force capture is triggered by
    // gripperCloseCompleteCallback, not here.
    geometric_confirmed_ = geom;

    if (!geom) {
        // Signal dropped: clear the verify latch (verify-latch logic, unrelated
        // to the capture trigger).
        if (mode_ == Mode::kVerify) last_verified_ = false;
    }

    // verify mode: steady heartbeat so downstream (grasp_state_machine) always has
    // a fresh value between grasps.
    if (mode_ == Mode::kVerify && verified_pub_) {
        std_msgs::msg::Bool out;
        out.data = last_verified_;
        verified_pub_->publish(out);
    }
}

void GraspForceCalibrationNode::gripperCloseCompleteCallback(
    const std_msgs::msg::Empty::SharedPtr /*msg*/) {
    if (capturing_) return;  // already in progress, ignore closely-spaced events

    if (!geometric_confirmed_) {
        RCLCPP_WARN(this->get_logger(),
                    "gripper close complete event received but geometric grasp is "
                    "NOT confirmed (EE not in position) - skipping force capture. "
                    "No calibration/verification will run for this event.");
        return;
    }

    capturing_ = true;
    buffer_.clear();
    capture_start_time_ = this->now();
    // One-shot window timer on the node clock (sim time under
    // use_sim_time:=true). It also guarantees the window closes when the
    // force topic goes completely silent - forceCallback would otherwise
    // never fire to end it.
    capture_timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        rclcpp::Duration::from_seconds(capture_window_sec_),
        std::bind(&GraspForceCalibrationNode::finishCaptureWindow, this));
    RCLCPP_INFO(this->get_logger(),
                "gripper close complete (geometric confirmed) -> capturing "
                "force for %.3f s", capture_window_sec_);
}

void GraspForceCalibrationNode::forceCallback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
    if (!capturing_) return;

    // Append every force sample that lands inside the time window; the one-shot
    // capture_timer_ closes it (finishCaptureWindow).
    const double fx = msg->wrench.force.x;
    const double fy = msg->wrench.force.y;
    const double fz = msg->wrench.force.z;
    buffer_.push_back(std::sqrt(fx * fx + fy * fy + fz * fz));
}

void GraspForceCalibrationNode::finishCaptureWindow() {
    if (capture_timer_) capture_timer_->cancel();
    if (!capturing_) return;
    capturing_ = false;

    const double dt = (this->now() - capture_start_time_).seconds();

    if (buffer_.empty()) {
        // Explicit failure, BOTH modes: not one force message arrived in the whole
        // window (topic silent). Never median an empty buffer, never write a
        // calibration file, never fall back to a default or mark success.
        RCLCPP_ERROR(this->get_logger(),
                     "capture window of %.3f s elapsed with ZERO messages on %s - %s. "
                     "Is the impedance controller active with "
                     "enable_contact_force_estimation:=true?",
                     capture_window_sec_, force_estimate_topic_.c_str(),
                     (mode_ == Mode::kCalibrate
                          ? "no calibration file written for this run_id"
                          : "cannot verify; publishing False"));
        if (mode_ == Mode::kVerify) {
            last_verified_ = false;
            if (verified_pub_) {
                std_msgs::msg::Bool out;
                out.data = false;
                verified_pub_->publish(out);
            }
        }
        return;
    }

    const double med = median(buffer_);

    RCLCPP_INFO(this->get_logger(),
                "capture window closed: %zu samples over %.3f s",
                buffer_.size(), dt);

    // Second reject case (the empty-buffer check above stays the first, separate
    // check): a median within the known configuration/tracking bias band is not a
    // real contact. Same numeric floor as tolerance_floor_n_ in verify mode.
    if (med < min_valid_force_n_) {
        if (mode_ == Mode::kCalibrate) {
            RCLCPP_ERROR(this->get_logger(),
                         "median force %.3f N is below min_valid_force_n=%.3f N - this "
                         "looks like tracking bias, not real contact; no calibration "
                         "written for run_id '%s'. If contact was genuine, lower "
                         "min_valid_force_n explicitly.",
                         med, min_valid_force_n_, run_id_.c_str());
        } else {  // verify: runtime data, not a config error -> WARN, same outcome
            RCLCPP_WARN(this->get_logger(),
                        "median force %.3f N is below min_valid_force_n=%.3f N - this "
                        "looks like tracking bias, not real contact; publishing False "
                        "for run_id '%s'. If contact was genuine, lower "
                        "min_valid_force_n explicitly.",
                        med, min_valid_force_n_, run_id_.c_str());
            last_verified_ = false;
            if (verified_pub_) {
                std_msgs::msg::Bool out;
                out.data = false;
                verified_pub_->publish(out);
            }
        }
        return;
    }

    onWindowComplete(med);
}

void GraspForceCalibrationNode::onWindowComplete(double median_force_norm) {
    if (mode_ == Mode::kCalibrate) {
        try {
            writeCalibrationYaml(median_force_norm);
            RCLCPP_INFO(this->get_logger(),
                        "calibrate: median force norm = %.4f N over %zu samples -> %s",
                        median_force_norm, buffer_.size(), calibration_file_path_.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                         "calibrate: failed to write %s: %s",
                         calibration_file_path_.c_str(), e.what());
        }
        return;
    }

    // verify
    if (!calib_loaded_) {
        if (!warned_missing_file_) {
            RCLCPP_WARN(this->get_logger(),
                        "verify: no calibration loaded for run_id '%s' (%s); "
                        "publishing False.",
                        run_id_.c_str(), calibration_file_path_.c_str());
            warned_missing_file_ = true;
        }
        last_verified_ = false;
    } else {
        const double tol = std::max(tolerance_ratio_ * f_calib_, tolerance_floor_n_);
        last_verified_ = std::abs(median_force_norm - f_calib_) <= tol;
        RCLCPP_INFO(this->get_logger(),
                    "verify: |%.4f - %.4f| = %.4f N vs tol %.4f N -> %s",
                    median_force_norm, f_calib_,
                    std::abs(median_force_norm - f_calib_), tol,
                    last_verified_ ? "MATCH" : "MISMATCH");
    }

    if (verified_pub_) {
        std_msgs::msg::Bool out;
        out.data = last_verified_;
        verified_pub_->publish(out);
    }
}

void GraspForceCalibrationNode::writeCalibrationYaml(double median_force_norm) const {
    std::error_code ec;
    std::filesystem::create_directories(calibration_dir_, ec);
    if (ec) {
        throw std::runtime_error("cannot create calibration_dir '" + calibration_dir_ +
                                 "': " + ec.message());
    }

    YAML::Node root;
    root["f_calib_norm"] = median_force_norm;
    root["run_id"] = run_id_;
    root["note"] = calibrationNote(run_id_);
    root["capture_window_sec"] = capture_window_sec_;
    root["n_samples"] = static_cast<int>(buffer_.size());
    root["force_estimate_topic"] = force_estimate_topic_;

    std::ofstream fout(calibration_file_path_);
    if (!fout.is_open()) {
        throw std::runtime_error("cannot open '" + calibration_file_path_ +
                                 "' for writing");
    }
    fout << root;
}

bool GraspForceCalibrationNode::loadCalibrationYaml() {
    try {
        YAML::Node root = YAML::LoadFile(calibration_file_path_);
        if (!root["f_calib_norm"]) {
            RCLCPP_WARN(this->get_logger(),
                        "calibration file %s has no 'f_calib_norm' field",
                        calibration_file_path_.c_str());
            return false;
        }
        f_calib_ = root["f_calib_norm"].as<double>();
        if (root["run_id"] && root["run_id"].as<std::string>() != run_id_) {
            RCLCPP_WARN(this->get_logger(),
                        "calibration file %s carries run_id '%s' but this node was "
                        "started with run_id '%s'",
                        calibration_file_path_.c_str(),
                        root["run_id"].as<std::string>().c_str(), run_id_.c_str());
        }
        calib_loaded_ = true;
        return true;
    } catch (const std::exception&) {
        calib_loaded_ = false;
        return false;
    }
}

double GraspForceCalibrationNode::median(std::vector<double> v) {
    if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(v.begin(), v.end());
    const size_t n = v.size();
    if (n % 2 == 1) return v[n / 2];
    return 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning
