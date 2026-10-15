#include "haptic_dmp_learning/ros/live_demo_recorder_node.hpp"
#include "haptic_dmp_learning/core/dmp_io.hpp"
#include "haptic_dmp_learning/core/demo_csv_io.hpp"
#include "haptic_dmp_learning/core/frame_correction.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace haptic_dmp_learning {
namespace ros_wrapper {

namespace {
// A single demonstration is never this long; a gap this large between a pose
// stamp and record_start_time_ means the two are on different clocks (typically
// a wall-clock hardware driver while recording under use_sim_time).
constexpr double kMaxPlausibleDemoSeconds = 3600.0;

// demo_raw_<TS>.csv -> demo_raw_<TS>_target_odom.csv (same rule as deriveJointStatesCsvPath:
// a trailing "_raw.csv" or ".csv" is replaced by the suffix). An empty path stays empty.
std::string deriveTargetOdomCsvPath(const std::string& demo_csv_path) {
    if (demo_csv_path.empty()) return demo_csv_path;
    auto ends_with = [&](const std::string& suffix) {
        return demo_csv_path.size() >= suffix.size() &&
               demo_csv_path.compare(demo_csv_path.size() - suffix.size(), suffix.size(), suffix) == 0;
    };
    if (ends_with("_raw.csv")) return demo_csv_path.substr(0, demo_csv_path.size() - 8) + "_target_odom.csv";
    if (ends_with(".csv")) return demo_csv_path.substr(0, demo_csv_path.size() - 4) + "_target_odom.csv";
    return demo_csv_path + "_target_odom.csv";
}
}  // namespace

LiveDemoRecorderNode::LiveDemoRecorderNode()
    : Node("live_demo_recorder_node"),
      dmp_(20, 4.6, 25.0, 6.25, false),
      recording_(false) {

    // 1. Declare DMP hyper-parameters as MANDATORY (no default), fail-loud.
    // Launched without --params-file, this node would otherwise start silently
    // with n_basis=20 instead of the validated 200 (config/params.yaml): on real
    // teleoperated demo data that degrades the learned trajectory from ~0.23mm
    // RMSE to ~72mm RMSE / ~60deg mean orientation error, with no error message
    // (see DESIGN_NOTES.md). Same pattern as haptic_dmp_wrapper_node and
    // demo_replay_sync_orchestrator_node.
    try {
        n_basis_ = this->declare_parameter<int>("n_basis");
        alpha_x_ = this->declare_parameter<double>("alpha_x");
        alpha_z_ = this->declare_parameter<double>("alpha_z");
        beta_z_ = this->declare_parameter<double>("beta_z");
    } catch (const std::exception& e) {
        throw std::runtime_error(
            "live_demo_recorder_node: required parameters (n_basis, alpha_x, alpha_z, beta_z) "
            "were not provided. This node must be launched with "
            "--ros-args --params-file <path/to/haptic_dmp_learning/config/params.yaml> "
            "so the validated DMP hyper-parameters are used instead of silently falling back "
            "to hardcoded ROS defaults. Underlying error: " + std::string(e.what()));
    }

    // 2. Declare topic names and output file paths
    master_pose_topic_ = this->declare_parameter<std::string>("master_pose_topic", "/master_pose_raw");
    target_pose_topic_ = this->declare_parameter<std::string>("target_pose_topic", "/target_pose");
    buttons_topic_ = this->declare_parameter<std::string>("buttons_topic", "/touch0/buttons");
    joint_states_topic_ = this->declare_parameter<std::string>("joint_states_topic", "/joint_states");

    // Absolute defaults so behavior does not depend on the process's current
    // working directory at launch.
    const char* home = std::getenv("HOME");
    const std::string ws_root = std::string(home ? home : "/root") + "/thesis_ws";
    output_yaml_path_ = this->declare_parameter<std::string>("output_yaml_path", ws_root + "/live_demo_dmp_weights.yaml");
    output_demo_csv_path_ = this->declare_parameter<std::string>("output_demo_csv_path", ws_root + "/live_demo_raw.csv");
    // Same session path as the raw demo, suffix _joint_states (live_demo_raw.csv ->
    // live_demo_joint_states.csv). An empty value disables the joint-states CSV.
    output_joint_states_csv_path_ = this->declare_parameter<std::string>(
        "output_joint_states_csv_path",
        core::joint_states_csv_io::deriveJointStatesCsvPath(output_demo_csv_path_));

    // Target odometry CSV: same session path as the raw demo, suffix _target_odom. Topic default =
    // the one demo_replay_sync_orchestrator_node waits its "first odometry" on ("/" + target_name + "/odometry").
    record_target_odom_ = this->declare_parameter<bool>("record_target_odom", true);
    target_odom_topic_ = this->declare_parameter<std::string>("target_odom_topic", "/free_target_object/odometry");
    output_target_odom_csv_path_ = deriveTargetOdomCsvPath(output_demo_csv_path_);

    // 2b. Optional anchored /target_pose. An empty array means "not provided". Wrong length or a
    // non-unit quaternion is a start-up error naming the parameter; so is enabling the anchor
    // with either array missing.
    anchor_enabled_ = this->declare_parameter<bool>("anchor_enabled", false);
    const auto anchor_p_param = this->declare_parameter<std::vector<double>>(
        "anchor_position_m", std::vector<double>{});
    const auto anchor_q_param = this->declare_parameter<std::vector<double>>(
        "anchor_orientation_xyzw", std::vector<double>{});
    if (!anchor_p_param.empty()) {
        anchor_position_ = core::anchored_target::parseAnchorPosition(anchor_p_param, "anchor_position_m");
    }
    if (!anchor_q_param.empty()) {
        anchor_orientation_ = core::anchored_target::parseAnchorOrientation(
            anchor_q_param, "anchor_orientation_xyzw");
    }
    if (anchor_enabled_) {
        if (anchor_p_param.empty()) {
            throw std::runtime_error("live_demo_recorder_node: anchor_enabled=true but "
                                     "anchor_position_m was not provided");
        }
        if (anchor_q_param.empty()) {
            throw std::runtime_error("live_demo_recorder_node: anchor_enabled=true but "
                                     "anchor_orientation_xyzw was not provided");
        }
    }

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

    // 6b. Subscribe to joint states (same always-on pattern as the master pose; the callback
    // drops messages while not recording)
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LiveDemoRecorderNode::jointStateCallback, this, std::placeholders::_1));

    // 6c. Target odometry (nav_msgs/Odometry, frame 'world', ~200 Hz from ros_gz_bridge). QoS(10) =
    // reliable + volatile, depth 10: the same the orchestrator uses on this topic, compatible with the
    // bridge's default publisher QoS (reliable, volatile, depth 10; free_target_object.launch.py sets no override).
    if (record_target_odom_) {
        target_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            target_odom_topic_, rclcpp::QoS(10),
            std::bind(&LiveDemoRecorderNode::targetOdomCallback, this, std::placeholders::_1));
    }

    // 7. Publisher for mirroring master poses to Gazebo controller
    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        target_pose_topic_, rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(),
                "live_demo_recorder_node ready. %s -> %s (visualization) | %s drives "
                "start/stop | DMP output: %s | Demo CSV: %s",
                master_pose_topic_.c_str(), target_pose_topic_.c_str(), buttons_topic_.c_str(),
                output_yaml_path_.c_str(), output_demo_csv_path_.c_str());
    if (record_target_odom_) {
        RCLCPP_INFO(this->get_logger(), "Target odometry recording ENABLED: %s -> %s (QoS: reliable, volatile, depth 10)",
                    target_odom_topic_.c_str(), output_target_odom_csv_path_.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Target odometry recording DISABLED (record_target_odom=false).");
    }
}

void LiveDemoRecorderNode::masterPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!recording_) return;

    // Fixed Geomagic -> Franka base-frame correction. The master pose arrives
    // expressed in the Geomagic base frame (omni_base), which is mounted +90
    // deg about z relative to the Franka base frame (fer_link0). Re-express it
    // NOW, before anything else looks at it - in particular before the
    // quaternion sign-continuity fix below, before the sample is buffered for
    // DMP fitting, and before the pose is mirrored to /target_pose. The single
    // definition of this rotation (and its hardware-verification status) lives
    // in core/frame_correction.hpp.
    const Eigen::Vector3d position = core::frame_correction::rotatePosition(
        Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
    Eigen::Quaterniond orient = core::frame_correction::rotateOrientation(
        Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                           msg->pose.orientation.y, msg->pose.orientation.z));
    orient.normalize();

    // Mirror the frame-corrected pose (not the raw message) onto /target_pose
    // for live visualization in Gazebo. The payload is now expressed in the
    // Franka base frame, so the header frame_id must say so too (same
    // metadata-vs-content fix as the CSV player's frame_id); "fer_link0" is the
    // base frame name used across the project (FrameAligner / franka URDF).
    geometry_msgs::msg::PoseStamped corrected = *msg;
    corrected.header.frame_id = "fer_link0";
    // With anchor_enabled the published pose is the master motion relative to its first sample,
    // applied on the anchor; the CSV buffer below still gets the corrected master pose.
    Eigen::Vector3d pub_position = position;
    Eigen::Quaterniond pub_orient = orient;
    if (anchor_enabled_) {
        const auto anchored = core::anchored_target::computeAnchoredTarget(
            anchor_state_, anchor_position_, anchor_orientation_, position, orient);
        pub_position = anchored.position;
        pub_orient = anchored.orientation;
    }
    corrected.pose.position.x = pub_position.x();
    corrected.pose.position.y = pub_position.y();
    corrected.pose.position.z = pub_position.z();
    corrected.pose.orientation.w = pub_orient.w();
    corrected.pose.orientation.x = pub_orient.x();
    corrected.pose.orientation.y = pub_orient.y();
    corrected.pose.orientation.z = pub_orient.z();
    target_pose_pub_->publish(corrected);

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
                "master pose header.stamp is in a different time base than the "
                "recording clock (delta=%.1f s) - falling back to this->now(); "
                "sample timing resolution may be reduced.", delta_from_start);
            now = this->now();
        }
    } else {
        now = this->now();
    }

    // Record sample
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

    if (gripper_trigger_pending_) {
        s.gripper_trigger = true;
        gripper_trigger_pending_ = false;
        RCLCPP_INFO(this->get_logger(), "Captured gripper trigger in sample at t=%.4f s", s.t);
    }

    recorder_.addSample(s);
}

void LiveDemoRecorderNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!recording_) return;

    // Joints are mapped by NAME (msg->name order is not fixed).
    core::joint_states_csv_io::JointStateSample sample;
    if (!core::joint_states_csv_io::extractJointPositions(msg->name, msg->position, sample.q)) {
        ++joint_states_skipped_;
        RCLCPP_WARN_ONCE(this->get_logger(),
                         "%s message lacks one of fer_joint1..fer_joint7 - skipping such "
                         "messages (count reported at the end of the recording).",
                         joint_states_topic_.c_str());
        return;
    }

    // Gripper fingers: optional, by name (absent with FRANKA_FINGERS=fixed).
    sample.has_fingers = core::joint_states_csv_io::extractFingerPositions(
        msg->name, msg->position, sample.fingers, &finger2_name_used_);

    // Same time base rule as masterPoseCallback: header.stamp when it shares the recording
    // clock, this->now() otherwise (unset or different clock).
    rclcpp::Time now;
    const rclcpp::Time stamp(msg->header.stamp, record_start_time_.get_clock_type());
    if ((msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) &&
        std::abs((stamp - record_start_time_).seconds()) < kMaxPlausibleDemoSeconds) {
        now = stamp;
    } else {
        now = this->now();
    }
    sample.t = (now - record_start_time_).seconds();
    joint_state_samples_.push_back(sample);
}

void LiveDemoRecorderNode::targetOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!recording_) return;

    // Same time base rule as jointStateCallback: header.stamp when it shares the recording clock
    // (the bridged odometry is stamped in Gazebo sim-time and this node runs with use_sim_time, so
    // t is on the same axis as the joint states and the demo), this->now() otherwise.
    rclcpp::Time now;
    const rclcpp::Time stamp(msg->header.stamp, record_start_time_.get_clock_type());
    if ((msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) &&
        std::abs((stamp - record_start_time_).seconds()) < kMaxPlausibleDemoSeconds) {
        now = stamp;
    } else {
        now = this->now();
        ++target_odom_time_fallbacks_;
    }

    TargetOdomSample s;
    s.t = (now - record_start_time_).seconds();
    const auto& p = msg->pose.pose.position;
    const auto& q = msg->pose.pose.orientation;
    const auto& v = msg->twist.twist.linear;
    const auto& w = msg->twist.twist.angular;
    const double row[13] = {p.x, p.y, p.z, q.w, q.x, q.y, q.z, v.x, v.y, v.z, w.x, w.y, w.z};
    for (int i = 0; i < 13; ++i) s.v[i] = row[i];
    if (target_odom_samples_.empty()) {
        RCLCPP_INFO(this->get_logger(),
                    "Target odometry: first sample received on %s at t=%.4f s (frame_id='%s'), "
                    "pose x=%.4f y=%.4f z=%.4f m (compare with the spawn pose to tell whether it is world-absolute).",
                    target_odom_topic_.c_str(), s.t, msg->header.frame_id.c_str(), p.x, p.y, p.z);
    }
    target_odom_samples_.push_back(s);  // no filtering: equal t are kept, analysis is downstream
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

    if (msg->buttons.size() >= 3 && msg->buttons[2] != 0) {
        gripper_trigger_pending_ = true;
    }
}

void LiveDemoRecorderNode::startRecording() {
    recorder_.clear();
    joint_state_samples_.clear();
    joint_states_skipped_ = 0;
    finger2_name_used_.clear();
    target_odom_samples_.clear();
    target_odom_time_fallbacks_ = 0;
    recording_ = true;
    gripper_trigger_pending_ = false;
    has_last_orientation_ = false;
    quat_negate_parity_ = false;
    quat_sign_flips_corrected_ = 0;
    anchor_state_ = core::anchored_target::State();
    record_start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Recording started.");
    if (record_target_odom_) {
        RCLCPP_INFO(this->get_logger(), "Target odometry recording ACTIVE on %s (QoS: reliable, volatile, depth 10).",
                    target_odom_topic_.c_str());
    }
}

void LiveDemoRecorderNode::saveJointStates() {
    if (output_joint_states_csv_path_.empty()) return;
    if (joint_states_skipped_ > 0) {
        RCLCPP_WARN(this->get_logger(),
                    "%zu %s message(s) lacked one of fer_joint1..fer_joint7 and were skipped.",
                    joint_states_skipped_, joint_states_topic_.c_str());
    }
    if (joint_state_samples_.empty()) {
        RCLCPP_ERROR(this->get_logger(),
                     "%s was not published during the recording (0 valid samples) - verify that "
                     "Gazebo / the robot state publisher is running. Joint-states CSV NOT written.",
                     joint_states_topic_.c_str());
        return;
    }
    try {
        core::joint_states_csv_io::writeJointStatesCsv(output_joint_states_csv_path_,
                                                       joint_state_samples_);
        std::size_t n_fingers = 0;
        for (const auto& s : joint_state_samples_) n_fingers += s.has_fingers ? 1 : 0;
        RCLCPP_INFO(this->get_logger(), "Joint states (%zu samples, %zu with finger joints) saved to %s",
                    joint_state_samples_.size(), n_fingers, output_joint_states_csv_path_.c_str());
        if (n_fingers == 0) {
            RCLCPP_WARN(this->get_logger(),
                        "Finger summary: 0 of %zu rows have both finger joints - width not recorded.",
                        joint_state_samples_.size());
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "Finger summary: %zu rows total, %zu with both fingers, finger 2 name: %s",
                        joint_state_samples_.size(), n_fingers, finger2_name_used_.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Saving joint states failed: %s", e.what());
    }
}

void LiveDemoRecorderNode::saveTargetOdom() {
    if (!record_target_odom_ || output_target_odom_csv_path_.empty()) return;
    if (target_odom_samples_.empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "Target odometry: NO message received on %s during the recording (0 samples) - "
                    "verify that free_target_object is spawned and its odometry bridge is running. "
                    "Target-odometry CSV NOT written.",
                    target_odom_topic_.c_str());
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Target odometry: %zu of %zu samples timed with now() (header.stamp unset or on another clock).",
                target_odom_time_fallbacks_, target_odom_samples_.size());
    if (target_odom_time_fallbacks_ > 0) {
        RCLCPP_WARN(this->get_logger(),
                    "Target odometry: %zu of %zu samples used the now() fallback instead of header.stamp - their t is "
                    "the callback arrival time, NOT the simulation stamp, so the series may not align with the joint states.",
                    target_odom_time_fallbacks_, target_odom_samples_.size());
    }
    try {
        std::ofstream f(output_target_odom_csv_path_);
        if (!f.is_open()) {
            throw std::runtime_error("cannot open file for writing: " + output_target_odom_csv_path_);
        }
        f << std::setprecision(9);
        f << "t,x,y,z,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz\n";
        for (const auto& s : target_odom_samples_) {
            f << s.t;
            for (double x : s.v) f << "," << x;
            f << "\n";
        }
        RCLCPP_INFO(this->get_logger(), "Target odometry (%zu samples) saved to %s",
                    target_odom_samples_.size(), output_target_odom_csv_path_.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Saving target odometry failed: %s", e.what());
    }
}

void LiveDemoRecorderNode::stopRecordingAndLearn() {
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

    // Save demonstration trajectory to CSV
    if (!output_demo_csv_path_.empty()) {
        try {
            core::demo_csv_io::writeDemoCsv(output_demo_csv_path_, recorder_.samples());
            RCLCPP_INFO(this->get_logger(), "Raw demo saved to %s", output_demo_csv_path_.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Saving raw demo failed: %s", e.what());
        }
    }

    saveJointStates();
    saveTargetOdom();

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

        // Serialize learned parameters to YAML
        core::dmp_io::saveToYaml(dmp_, quat_dmp_, output_yaml_path_);
        RCLCPP_INFO(this->get_logger(), "DMP + Quaternion DMP learned and saved to %s", output_yaml_path_.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Learning/saving failed: %s", e.what());
    }
}

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning
