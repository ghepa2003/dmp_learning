#include "haptic_dmp_learning/ros/haptic_dmp_wrapper_node.hpp"
#include "haptic_dmp_learning/core/dmp_io.hpp"
#include "haptic_dmp_learning/core/frame_correction.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <cstdlib>
#include <fstream>

namespace haptic_dmp_learning {
namespace ros_wrapper {

namespace {
// A single demonstration is never this long; a gap this large between a pose
// stamp and record_start_time_ means the two are on different clocks (typically
// a wall-clock hardware driver while recording under use_sim_time).
constexpr double kMaxPlausibleDemoSeconds = 3600.0;
}  // namespace

HapticDmpWrapperNode::HapticDmpWrapperNode()
    : Node("haptic_dmp_wrapper_node"),
      dmp_(20, 4.6, 25.0, 6.25, false),
      recording_(false) {

    // 1. Declare DMP hyper-parameters as MANDATORY (no default). A wrong
    // n_basis silently degrades the learned trajectory without any error
    // (verified: n_basis=20 instead of the validated 200 raises real-data
    // RMSE from 0.23mm to 1.4mm) - so this node refuses to start unless these
    // come from an explicit --ros-args --params-file <config/params.yaml>.
    try {
        n_basis_ = this->declare_parameter<int>("n_basis");
        alpha_x_ = this->declare_parameter<double>("alpha_x");
        alpha_z_ = this->declare_parameter<double>("alpha_z");
        beta_z_ = this->declare_parameter<double>("beta_z");
    } catch (const std::exception& e) {
        throw std::runtime_error(
            "haptic_dmp_wrapper_node: required parameters (n_basis, alpha_x, alpha_z, beta_z) "
            "were not provided. This node must be launched with "
            "--ros-args --params-file <path/to/haptic_dmp_learning/config/params.yaml> "
            "so the validated DMP hyper-parameters are used instead of silently falling back "
            "to hardcoded ROS defaults. Underlying error: " + std::string(e.what()));
    }

    // 2. Output file paths (absolute defaults so behavior does not depend on
    // the process's current working directory at launch)
    const char* home = std::getenv("HOME");
    const std::string ws_root = std::string(home ? home : "/root") + "/thesis_ws";
    output_yaml_path_ = this->declare_parameter<std::string>("output_yaml_path", ws_root + "/dmp_weights.yaml");
    output_demo_csv_path_ = this->declare_parameter<std::string>("output_demo_csv_path", ws_root + "/demo_raw.csv");

    // 3. Locate optional feature configuration YAML file (absolute default;
    // dmp_io::applyFeatureConfig has its own fallback chain and will refuse
    // to continue rather than silently reverting to unfiltered LWR)
    std::string default_features_path;
    try {
        default_features_path = ament_index_cpp::get_package_share_directory("haptic_dmp_learning") + "/config/dmp_features.yaml";
    } catch (const std::exception&) {
        default_features_path = ws_root + "/src/haptic_dmp_learning/config/dmp_features.yaml";
    }
    feature_flags_path_ = this->declare_parameter<std::string>("feature_flags_path", default_features_path);

    // 3b. Master pose input topic. Declared as a ROS parameter (same pattern as
    // live_demo_recorder_node) instead of a hardcoded "/touch0/pose" that had
    // to be rewired with an external CLI remap: a forgotten remap made the node
    // subscribe to a dead topic and fail silently. Default matches the pipeline
    // convention ("/master_pose_raw").
    pose_topic_ = this->declare_parameter<std::string>("master_pose_topic", "/master_pose_raw");

    // 4. Instantiate core translational and rotational DMP solvers
    dmp_ = core::DMP(n_basis_, alpha_x_, alpha_z_, beta_z_);
    quat_dmp_ = core::QuaternionDMP(n_basis_, alpha_x_, alpha_z_, beta_z_);

    // 5. Apply advanced algorithmic features (Ridge regression, velocity filters)
    core::dmp_io::applyFeatureConfig(feature_flags_path_, dmp_, quat_dmp_);

    // 6. Subscribe to Geomagic Touch pose (~1 kHz, best-effort sensor QoS)
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, rclcpp::SensorDataQoS(),
        std::bind(&HapticDmpWrapperNode::poseCallback, this, std::placeholders::_1));

    // 7. Subscribe to stylus physical buttons (joy message)
    buttons_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/touch0/buttons", 10,
        std::bind(&HapticDmpWrapperNode::buttonsCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "haptic_dmp_wrapper_node ready. Master pose input: %s | Press button 0 to "
                "start recording a demo, button 1 to stop and learn the DMP. "
                "DMP output: %s | Demo CSV: %s",
                pose_topic_.c_str(), output_yaml_path_.c_str(), output_demo_csv_path_.c_str());
}

void HapticDmpWrapperNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!recording_) return;

    // Fixed Geomagic -> Franka base-frame correction. The master pose arrives
    // expressed in the Geomagic base frame (omni_base), which is mounted +90
    // deg about z relative to the Franka base frame (fer_link0). Re-express it
    // NOW, before anything else looks at it - in particular before the
    // quaternion sign-continuity fix below and before the sample is buffered
    // for DMP fitting. The single definition of this rotation (and its
    // hardware-verification status) lives in core/frame_correction.hpp.
    const Eigen::Vector3d position = core::frame_correction::rotatePosition(
        Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
    Eigen::Quaterniond orient = core::frame_correction::rotateOrientation(
        Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                           msg->pose.orientation.y, msg->pose.orientation.z));
    orient.normalize();

    // Sample time base. Prefer msg->header.stamp: it carries the publisher's
    // full timing resolution and is immune to sim-time /clock quantization,
    // which - when we fall back to this->now() under a coarse /clock - collapses
    // consecutive samples onto the same tick, i.e. dt=0 rows that blow up the
    // learned DMP weights. Use the stamp only while it shares
    // record_start_time_'s time base: a stamp from a real-hardware driver
    // (wall-clock) recorded under use_sim_time lands ~1.7e9 s away from the
    // sim-time start, so guard on a plausible delta and fall back to this->now()
    // (loudly, once) otherwise. A zero stamp means "unset" -> also fall back.
    rclcpp::Time now;
    const rclcpp::Time stamp(msg->header.stamp, record_start_time_.get_clock_type());
    if (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) {
        const double delta_from_start = std::abs((stamp - record_start_time_).seconds());
        if (delta_from_start < kMaxPlausibleDemoSeconds) {
            now = stamp;
        } else {
            RCLCPP_WARN_ONCE(this->get_logger(),
                "pose header.stamp is in a different time base than the recording "
                "clock (delta=%.1f s) - falling back to this->now(); sample timing "
                "resolution may be reduced.", delta_from_start);
            now = this->now();
        }
    } else {
        now = this->now();
    }

    // Assemble Sample object and buffer it into recorder
    core::Sample s;
    s.t = (now - record_start_time_).seconds();
    s.position = position;

    // Quaternion double-cover fix, now operating on the frame-corrected
    // orientation computed above. The Geomagic Touch driver occasionally
    // reports the same physical rotation with the whole quaternion negated
    // (q vs -q). That is a representation discontinuity, not motion, and it
    // becomes a ~pi jump in the log-map increments QuaternionDMP accumulates,
    // producing an absurd forcing term at that point and an unstable replay.
    // A negative dot product between consecutive orientations is such a
    // transition: toggle a running sign (q and -q are the same rotation, so it
    // is loss-free) and apply it to this and every later sample.
    if (has_last_orientation_ && last_corrected_orientation_.dot(orient) < 0.0) {
        quat_negate_parity_ = !quat_negate_parity_;
        ++quat_sign_flips_corrected_;
    }
    last_corrected_orientation_ = orient;
    has_last_orientation_ = true;
    if (quat_negate_parity_) {
        orient.coeffs() = -orient.coeffs();
    }
    s.orientation = orient;

    recorder_.addSample(s);
}

void HapticDmpWrapperNode::buttonsCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->buttons.size() < 2) {
        RCLCPP_WARN_ONCE(this->get_logger(),
                          "Expected at least 2 entries in /touch0/buttons, got %zu",
                          msg->buttons.size());
        return;
    }

    if (prev_buttons_.empty()) {
        prev_buttons_.assign(msg->buttons.begin(), msg->buttons.end());
        return;
    }

    // Rising-edge detection (0 -> 1 transition)
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
    has_last_orientation_ = false;
    quat_negate_parity_ = false;
    quat_sign_flips_corrected_ = 0;
    record_start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Recording started.");
}

void HapticDmpWrapperNode::stopRecordingAndLearn() {
    recording_ = false;
    RCLCPP_INFO(this->get_logger(), "Recording stopped. %zu samples collected.", recorder_.size());

    if (quat_sign_flips_corrected_ > 0) {
        RCLCPP_WARN(this->get_logger(),
                    "%zu quaternion double-cover sign transition(s) (q vs -q) were corrected during "
                    "recording - a Geomagic Touch driver artifact.",
                    quat_sign_flips_corrected_);
    }

    if (recorder_.size() < 5) {
        RCLCPP_WARN(this->get_logger(), "Too few samples, discarding this demonstration.");
        return;
    }

    // Save raw demonstrated trajectory to CSV
    if (!output_demo_csv_path_.empty()) {
        try {
            saveDemoToCsv(output_demo_csv_path_);
            RCLCPP_INFO(this->get_logger(), "Raw demo saved to %s", output_demo_csv_path_.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Saving raw demo failed: %s", e.what());
        }
    }

    // Fit weights for both translational and rotational DMPs
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

        if (dmp_diag.dropped_non_monotonic_samples > 0) {
            RCLCPP_WARN(this->get_logger(),
                        "%d demo sample(s) had non-increasing timestamps and were dropped "
                        "before DMP fitting - check the recording clock (a coarse sim-time "
                        "/clock can stamp consecutive samples identically).",
                        dmp_diag.dropped_non_monotonic_samples);
        }

        if (qdmp_diag.quat_sign_flips_corrected > 0) {
            RCLCPP_WARN(this->get_logger(),
                        "%d quaternion double-cover sign transition(s) (q vs -q) were corrected "
                        "before QuaternionDMP fitting.",
                        qdmp_diag.quat_sign_flips_corrected);
        }

        // Serialize learned parameters to destination YAML
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
