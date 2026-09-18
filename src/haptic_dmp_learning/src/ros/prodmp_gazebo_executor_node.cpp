#include "haptic_dmp_learning/ros/prodmp_gazebo_executor_node.hpp"
#include "haptic_dmp_learning/core/dmp.hpp"
#include "haptic_dmp_learning/core/dmp_io.hpp"
#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "haptic_dmp_learning/core/demo_csv_io.hpp"
#include "haptic_dmp_learning/core/gripper_ramp.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <stdexcept>

#include <rclcpp/create_timer.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;

namespace haptic_dmp_learning {
namespace ros_wrapper {

ProDmpGazeboExecutorNode::ProDmpGazeboExecutorNode()
    : Node("prodmp_gazebo_executor_node"),
      prodmp_(20),                        // Placeholder num_basis, overwritten by loadProDmpFromYaml
      qdmp_(20, 4.6, 25.0, 6.25),
      dt_(0.005),
      elapsed_(0.0),
      finished_(false) {

    // 1. Declare parameters (absolute default so behavior does not depend on
    // the process's current working directory at launch). Same set as
    // dmp_gazebo_executor_node; weights_yaml_path_ now points at a ProDMP file.
    const char* home = std::getenv("HOME");
    const std::string default_weights_path =
        std::string(home ? home : "/root") + "/thesis_ws/prodmp_weights.yaml";
    weights_yaml_path_ = this->declare_parameter<std::string>("weights_yaml_path", default_weights_path);
    // ProDMP covers POSITION only in this project. The orientation channel keeps
    // running on core::QuaternionDMP, whose weights live in the classic combined
    // file (dmp_weights_<run_id>.yaml, quaternion_dmp section). Supplied here.
    orientation_weights_yaml_path_ =
        this->declare_parameter<std::string>("orientation_weights_yaml_path", "");
    target_pose_topic_ = this->declare_parameter<std::string>("target_pose_topic", "/target_pose");
    target_twist_topic_ = this->declare_parameter<std::string>("target_twist_topic", "/target_twist");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "panda_link0");
    control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 200.0);
    startup_delay_sec_ = this->declare_parameter<double>("startup_delay_sec", 1.0);

    // One-shot target retarget (translation only). Same odometry topic the
    // orchestrator waits on for its sim-time anchor; default matches the
    // free_target_object launch bridge (/model/<name>/odometry -> /<name>/odometry).
    target_odom_topic_ = this->declare_parameter<std::string>(
        "target_odom_topic", "/free_target_object/odometry");
    // Fail-loud: refuse to replay toward the demo's frozen goal if the live
    // target position never arrives.
    target_odom_timeout_sec_ = this->declare_parameter<double>("target_odom_timeout_sec", 5.0);
    // When false, skip the target-odometry subscription, wait and retarget
    // entirely: replay the demonstration's ORIGINAL goal (frozen in the loaded
    // weights) exactly as-is. Default true preserves the existing behaviour for
    // every current pipeline use; set false only for a run with no target at all.
    target_odom_required_ = this->declare_parameter<bool>("target_odom_required", true);
    // TF frames for the retarget. target_position_ arrives in world_frame_; the
    // live EE pose (world_frame_ -> ee_frame_) is looked up so init_pos can be
    // anchored to the real EE position. Same naming/defaults as
    // grasp_monitoring/geometric_grasp_monitor.
    world_frame_ = this->declare_parameter<std::string>("world_frame", "world");
    ee_frame_ = this->declare_parameter<std::string>("ee_frame", "fer_hand_tcp");

    // Satellite rotation model (position-only reach test - see startTimer()).
    // Default disabled: with satellite_rotation_enabled=false every branch
    // below is identical to today's behaviour byte-for-byte.
    satellite_rotation_enabled_ =
        this->declare_parameter<bool>("satellite_rotation_enabled", false);
    satellite_rotation_mode_ =
        this->declare_parameter<std::string>("satellite_rotation_mode", "frozen");
    {
        std::vector<double> axis_param = this->declare_parameter<std::vector<double>>(
            "satellite_rotation_axis", std::vector<double>{0.0, 0.0, 1.0});
        std::vector<double> center_param = this->declare_parameter<std::vector<double>>(
            "satellite_rotation_center", std::vector<double>{0.0, 0.0, 0.0});
        if (axis_param.size() != 3 || center_param.size() != 3) {
            RCLCPP_FATAL(this->get_logger(),
                         "satellite_rotation_axis and satellite_rotation_center must each have "
                         "exactly 3 elements (got %zu and %zu).",
                         axis_param.size(), center_param.size());
            throw std::runtime_error(
                "prodmp_gazebo_executor_node: malformed satellite_rotation_axis/center parameter");
        }
        satellite_rotation_axis_ = Eigen::Vector3d(axis_param[0], axis_param[1], axis_param[2]);
        satellite_rotation_center_ =
            Eigen::Vector3d(center_param[0], center_param[1], center_param[2]);
    }
    satellite_rotation_angular_velocity_deg_s_ =
        this->declare_parameter<double>("satellite_rotation_angular_velocity_deg_s", 2.0);
    satellite_rotation_frozen_phase_deg_ =
        this->declare_parameter<double>("satellite_rotation_frozen_phase_deg", 0.0);

    // Gripper close ramp (see core/gripper_ramp.hpp). Defaults reproduce the
    // previous single-step behaviour's endpoints: 0.06 = open, 0.0 = closed.
    gripper_open_position_ = this->declare_parameter<double>("gripper_open_position", 0.06);
    gripper_closed_position_ = this->declare_parameter<double>("gripper_closed_position", 0.0);
    gripper_close_ramp_duration_sec_ =
        this->declare_parameter<double>("gripper_close_ramp_duration_sec", 2.0);

    dt_ = 1.0 / control_rate_hz_;

    // 2. Load trained ProDMP position parameters from YAML. loadProDmpFromYaml
    // returns a fully-initialised model (setLearnedParameters has already seated
    // the demonstrated initial conditions), so prodmp_ is roll-ready right away
    // - there is no separate reset() to call, unlike core::DMP.
    try {
        prodmp_ = core::prodmp_io::loadProDmpFromYaml(weights_yaml_path_);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "Failed to load ProDMP weights from %s: %s",
                     weights_yaml_path_.c_str(), e.what());
        throw;
    }
    demo_init_pos_ = prodmp_.initPos();
    demo_init_vel_ = prodmp_.initVel();
    // p_grasp_demo for the satellite rotation model below - captured now,
    // before anything else can call setGoal() and overwrite it.
    demo_grasp_goal_ = prodmp_.goal();

    // 2b. Load the orientation (quaternion) model. ProDMP does not model
    // orientation in this project by default, so a plain ProDMP weights file
    // has no quaternion_dmp section. Resolution order (unchanged precedence,
    // now also supporting the unified single-file format):
    //   1. orientation_weights_yaml_path explicitly set -> ALWAYS wins,
    //      unconditionally, even if weights_yaml_path_ also carries an inline
    //      quaternion_dmp section (explicit beats implicit).
    //   2. Else, attempt to read an inline `quaternion_dmp:` section from
    //      weights_yaml_path_ itself (unified ProDMP+orientation file) via
    //      prodmp_io::loadProDmpFromYaml's orientation-reporting overload.
    //   3. Else -> fail loud, exactly as before this feature existed (no
    //      silent identity/zero orientation fabrication).
    std::string orientation_path = orientation_weights_yaml_path_;
    bool use_inline_orientation = false;
    if (orientation_path.empty()) {
        core::QuaternionDMP inline_qdmp(20, 4.6, 25.0, 6.25);
        bool has_inline_orientation = false;
        try {
            core::prodmp_io::loadProDmpFromYaml(weights_yaml_path_, inline_qdmp,
                                                has_inline_orientation);
        } catch (const std::exception&) {
            // A genuinely unreadable file already made the ProDMP position load
            // above throw; reaching here just means the file parsed but simply
            // lacks a quaternion_dmp section (has_inline_orientation stays false).
        }
        if (has_inline_orientation) {
            qdmp_ = inline_qdmp;
            use_inline_orientation = true;
            orientation_path = weights_yaml_path_;
            RCLCPP_INFO(this->get_logger(),
                        "Loaded orientation (quaternion_dmp) inline from unified ProDMP weights "
                        "file '%s' (no separate orientation_weights_yaml_path given).",
                        weights_yaml_path_.c_str());
        }
    }
    if (orientation_path.empty()) {
        RCLCPP_FATAL(this->get_logger(),
                     "ProDMP weights '%s' carry no orientation (quaternion_dmp) section - "
                     "ProDMP models POSITION only in this project unless the unified file "
                     "format is used. Provide 'orientation_weights_yaml_path' pointing at the "
                     "classic combined dmp_weights_<run_id>.yaml, or save the ProDMP weights "
                     "with an embedded quaternion_dmp section (unified format).",
                     weights_yaml_path_.c_str());
        throw std::runtime_error(
            "prodmp_gazebo_executor_node: 'orientation_weights_yaml_path' is required "
            "(ProDMP weights do not contain orientation, inline or separate)");
    }
    if (!use_inline_orientation) {
        try {
            // dmp_io::loadFromYaml(path, DMP&, QuaternionDMP&) reads the combined
            // classic format. We only need the quaternion_dmp half; the position DMP
            // it also fills is a throwaway (position comes from prodmp_).
            core::DMP orientation_position_unused(20, 4.6, 25.0, 6.25, false);
            core::dmp_io::loadFromYaml(orientation_path, orientation_position_unused, qdmp_);
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(),
                         "Failed to load orientation (quaternion) weights from %s: %s",
                         orientation_path.c_str(), e.what());
            throw;
        }
    }

    if (std::abs(prodmp_.tau() - qdmp_.tau()) > 1e-6) {
        RCLCPP_WARN(this->get_logger(),
                    "position tau (%.4f) and orientation tau (%.4f) differ - "
                    "using position tau as rollout duration.",
                    prodmp_.tau(), qdmp_.tau());
    }

    // 2c. Parse demo CSV for gripper trigger timestamp if provided. Unchanged
    // logic: "cannot open the file" (likely a wrong path) is kept loud and
    // separate from "opened it, but this demo has no gripper trigger".
    demo_csv_path_ = this->declare_parameter<std::string>("demo_csv_path", "");
    if (!demo_csv_path_.empty()) {
        std::ifstream probe(demo_csv_path_);
        if (!probe.is_open()) {
            RCLCPP_WARN(this->get_logger(),
                        "Could not open demo CSV at %s; replay without gripper trigger.",
                        demo_csv_path_.c_str());
        } else {
            probe.close();
            gripper_trigger_t_ = core::demo_csv_io::readGripperTriggerTime(demo_csv_path_);
            if (gripper_trigger_t_ >= 0.0) {
                RCLCPP_INFO(this->get_logger(),
                            "Found gripper trigger in CSV at t = %.4f s", gripper_trigger_t_);
            } else {
                RCLCPP_INFO(this->get_logger(),
                            "No 'gripper_trigger' column or no triggering row in %s; "
                            "replay without gripper trigger.",
                            demo_csv_path_.c_str());
            }
        }
    }

    // 3. Rollout state.
    //  - Position: prodmp_ is already seated at the demonstrated initial
    //    conditions by loadProDmpFromYaml. When target_odom_required_ is true we
    //    additionally switch it to the native relative-goal mode ONCE here, so
    //    the later setInitialConditions(ee_now) + setGoal(target_world) in
    //    startTimer() need no manual frame bridging. This is what makes the
    //    ProDMP retarget cleaner than the classic DMP one: DMP::setGoal() lives
    //    in the demo-local frame of dmp_.y0(), forcing the caller to compute
    //    "y0() + (target - ee_now)"; ProDMP is linearly parameterised, so
    //    "goal relative to the initial position" is a first-class setting and
    //    the goal can be handed over directly in world coordinates.
    if (target_odom_required_) {
        prodmp_.setRelativeGoal(true);
    }
    //  - Orientation: same as dmp_gazebo_executor_node.
    qdmp_.reset();

    // 4. Create publishers
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        target_pose_topic_, rclcpp::QoS(10));
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        target_twist_topic_, rclcpp::QoS(10));
    gripper_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/gripper_position_cmd", rclcpp::QoS(10));
    gripper_close_complete_pub_ = this->create_publisher<std_msgs::msg::Empty>(
        "/gripper_close_complete", rclcpp::QoS(10));

    // 4b. One-shot target retarget subscription, created only when required
    // (identical to dmp_gazebo_executor_node). The TF listener is created
    // whenever EITHER feature needs to anchor the ProDMP init position to the
    // real EE pose - target_odom_required_ (live retarget) or
    // satellite_rotation_enabled_ (rotation retarget) - so the satellite
    // rotation branch in startTimer() can rely on tf_buffer_ existing without
    // also requiring target_odom_required_=true.
    if (target_odom_required_) {
        target_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            target_odom_topic_, rclcpp::QoS(10),
            std::bind(&ProDmpGazeboExecutorNode::odomCallback, this, std::placeholders::_1));
    }
    if (target_odom_required_ || satellite_rotation_enabled_) {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    }

    RCLCPP_INFO(this->get_logger(),
                "prodmp_gazebo_executor_node ready. ProDMP weights: %s | orientation weights: %s | "
                "tau: %.3f s | rate: %.1f Hz | publishing on %s in %.1f s | "
                "target_odom_required=%s (topic %s, timeout %.1f s)",
                weights_yaml_path_.c_str(), orientation_path.c_str(), prodmp_.tau(),
                control_rate_hz_, target_pose_topic_.c_str(), startup_delay_sec_,
                target_odom_required_ ? "true" : "false",
                target_odom_topic_.c_str(), target_odom_timeout_sec_);

    if (satellite_rotation_enabled_) {
        RCLCPP_INFO(this->get_logger(),
                    "satellite_rotation_enabled=true, mode='%s': axis=[%.4f, %.4f, %.4f], "
                    "center=[%.4f, %.4f, %.4f] (world/base frame), "
                    "angular_velocity_deg_s=%.3f, frozen_phase_deg=%.3f "
                    "(demo grasp point p_grasp_demo=[%.4f, %.4f, %.4f]).",
                    satellite_rotation_mode_.c_str(),
                    satellite_rotation_axis_.x(), satellite_rotation_axis_.y(),
                    satellite_rotation_axis_.z(), satellite_rotation_center_.x(),
                    satellite_rotation_center_.y(), satellite_rotation_center_.z(),
                    satellite_rotation_angular_velocity_deg_s_,
                    satellite_rotation_frozen_phase_deg_, demo_grasp_goal_.x(),
                    demo_grasp_goal_.y(), demo_grasp_goal_.z());
    } else {
        RCLCPP_INFO(this->get_logger(),
                    "satellite_rotation_enabled=false: no rotation model applied.");
    }

    // Preventive warning for use_sim_time:=true with no /clock publisher
    // (identical to dmp_gazebo_executor_node).
    if (this->get_parameter("use_sim_time").as_bool()) {
        RCLCPP_WARN(this->get_logger(),
                    "use_sim_time=true: this node requires /clock to be actively "
                    "publishing (typically from Gazebo). If this is a REAL HARDWARE "
                    "run, launch with use_sim_time:=false or the node will hang "
                    "waiting for a clock that never arrives.");
    }

    // 5. One-shot startup delay timer (sim-time; honours use_sim_time), same as
    // dmp_gazebo_executor_node.
    startup_timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        rclcpp::Duration::from_seconds(startup_delay_sec_),
        std::bind(&ProDmpGazeboExecutorNode::startTimer, this));
}

void ProDmpGazeboExecutorNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (target_odom_received_) return;  // one-shot: ignore every message after the first

    target_position_ = Eigen::Vector3d(msg->pose.pose.position.x,
                                       msg->pose.pose.position.y,
                                       msg->pose.pose.position.z);
    target_odom_received_ = true;
    target_odom_sub_.reset();

    RCLCPP_INFO(this->get_logger(),
                "Target odometry received (frame '%s'): position [%.4f, %.4f, %.4f]; "
                "unsubscribed (one-shot retarget).",
                msg->header.frame_id.c_str(),
                target_position_.x(), target_position_.y(), target_position_.z());
}

bool ProDmpGazeboExecutorNode::lookupEeNowWithRetry(Eigen::Vector3d& ee_now) {
    if (!odom_wait_started_) {
        odom_wait_start_ = this->now();
        odom_wait_started_ = true;
    }

    geometry_msgs::msg::TransformStamped ee_tf;
    try {
        ee_tf = tf_buffer_->lookupTransform(world_frame_, ee_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        const double waited = (this->now() - odom_wait_start_).seconds();
        if (waited >= target_odom_timeout_sec_) {
            RCLCPP_FATAL(this->get_logger(),
                         "TF lookup '%s' -> '%s' still unavailable %.2f s after startup "
                         "(%s). Cannot anchor the ProDMP initial position - refusing to "
                         "replay the retarget. Is the robot state / TF tree being published?",
                         world_frame_.c_str(), ee_frame_.c_str(),
                         target_odom_timeout_sec_, ex.what());
            throw std::runtime_error(
                "prodmp_gazebo_executor_node: TF lookup '" + world_frame_ + "' -> '" +
                ee_frame_ + "' timed out for the retarget");
        }
        startup_timer_ = rclcpp::create_timer(
            this, this->get_clock(),
            rclcpp::Duration::from_seconds(0.1),
            std::bind(&ProDmpGazeboExecutorNode::startTimer, this));
        return false;
    }

    ee_now = Eigen::Vector3d(ee_tf.transform.translation.x, ee_tf.transform.translation.y,
                              ee_tf.transform.translation.z);
    return true;
}

void ProDmpGazeboExecutorNode::startTimer() {
    if (startup_timer_) startup_timer_->cancel();

    if (satellite_rotation_enabled_) {
        // tf_buffer_/tf_listener_ are constructed in 4b. above whenever
        // satellite_rotation_enabled_ is true, INDEPENDENTLY of
        // target_odom_required_ - this branch does not need
        // target_odom_required_=true for anything (it calls
        // prodmp_.setRelativeGoal(true) itself below, and never falls through
        // to the target_odom_required_ branches further down since it always
        // returns). Run with target_odom_required:=false for a pure static
        // reach test - it avoids subscribing to the unused
        // /free_target_object/odometry topic.
        Eigen::Vector3d ee_now;
        if (!lookupEeNowWithRetry(ee_now)) {
            return;  // re-polling scheduled by lookupEeNowWithRetry(); try again in 0.1 s
        }

        if (satellite_rotation_mode_ != "frozen") {
            // Fail loud WITHOUT throwing: startTimer() runs off an rclcpp
            // timer callback, where an uncaught exception is not guaranteed
            // to surface as clearly as a plain log line. The node stays
            // alive (matches the empty-capture-buffer pattern in
            // GraspForceCalibrationNode::finishCaptureWindow()) but never
            // arms step_timer_, so the rollout simply never starts.
            RCLCPP_ERROR(this->get_logger(),
                         "satellite_rotation_mode='%s' not implemented yet - requires Level 1 "
                         "PhaseSelector integration. Node will not start the rollout.",
                         satellite_rotation_mode_.c_str());
            return;
        }

        // 1. Convert demo grasp goal from local demo frame to world coordinates
        // using the real end-effector initial anchor (same principle as target_odom_required_):
        const Eigen::Vector3d p_grasp_demo_world = ee_now + demo_grasp_goal_;

        // 2. Apply satellite rotation model (axis, center, theta) in world frame:
        const double theta_rad = satellite_rotation_frozen_phase_deg_ * M_PI / 180.0;
        const Eigen::AngleAxisd rot(theta_rad, satellite_rotation_axis_.normalized());
        const Eigen::Vector3d p_grasp_rotated_world =
            satellite_rotation_center_ + rot * (p_grasp_demo_world - satellite_rotation_center_);

        // 3. Hand over to ProDMP in world coordinates with relativeGoal=true.
        // ProDMP::setGoal(p_grasp_rotated_world) stores internally:
        //   goal_param_ = p_grasp_rotated_world - ee_now
        // At theta = 0 (rot = Identity):
        //   p_grasp_rotated_world == p_grasp_demo_world == ee_now + demo_grasp_goal_
        //   goal_param_ == demo_grasp_goal_ (ee_now cancels algebraically).
        prodmp_.setRelativeGoal(true);
        prodmp_.setInitialConditions(/*init_time=*/0.0, ee_now, Eigen::Vector3d::Zero());
        prodmp_.setGoal(p_grasp_rotated_world);
        // Solo la posizione del goal viene ruotata secondo il modello di
        // rotazione del satellite; l'orientamento resta quello demo-nativo,
        // coerente con un test di solo reach - la geometria di presa sarà
        // introdotta con il grasping. qdmp_.reset() qui NON applica nessuna
        // rotazione: si limita a riportare lo stato dell'integratore
        // (fase x_, quaternione corrente q_, velocità angolare eta_) alle
        // condizioni della demo (q0_), esattamente come nel ramo
        // target_odom_required_ sopra - goal_ dell'orientamento non viene mai
        // toccato (nessun qdmp_.setGoal() qui, come là), quindi resta quello
        // caricato dal file dei pesi.
        qdmp_.reset();

        RCLCPP_INFO(this->get_logger(),
                    "Satellite rotation retarget (mode=frozen, phase=%.2f deg):\n"
                    "  1. Local demo goal [%.4f, %.4f, %.4f] + EE anchor [%.4f, %.4f, %.4f] -> demo world grasp [%.4f, %.4f, %.4f]\n"
                    "  2. Rotated in world (center=[%.4f, %.4f, %.4f], axis=[%.4f, %.4f, %.4f]) -> target world grasp [%.4f, %.4f, %.4f]\n"
                    "  3. Relative goal parameter passed to ProDMP: [%.4f, %.4f, %.4f] (world, %s -> %s) | orientation demo-native.",
                    satellite_rotation_frozen_phase_deg_,
                    demo_grasp_goal_.x(), demo_grasp_goal_.y(), demo_grasp_goal_.z(),
                    ee_now.x(), ee_now.y(), ee_now.z(),
                    p_grasp_demo_world.x(), p_grasp_demo_world.y(), p_grasp_demo_world.z(),
                    satellite_rotation_center_.x(), satellite_rotation_center_.y(), satellite_rotation_center_.z(),
                    satellite_rotation_axis_.x(), satellite_rotation_axis_.y(), satellite_rotation_axis_.z(),
                    p_grasp_rotated_world.x(), p_grasp_rotated_world.y(), p_grasp_rotated_world.z(),
                    (p_grasp_rotated_world - ee_now).x(), (p_grasp_rotated_world - ee_now).y(), (p_grasp_rotated_world - ee_now).z(),
                    world_frame_.c_str(), ee_frame_.c_str());

        RCLCPP_INFO(this->get_logger(), "Starting ProDMP rollout.");
        step_timer_ = rclcpp::create_timer(
            this, this->get_clock(),
            rclcpp::Duration::from_seconds(dt_),
            std::bind(&ProDmpGazeboExecutorNode::stepCallback, this));
        return;
    }

    if (!target_odom_required_) {
        // No target to retarget toward: replay the demonstration's ORIGINAL goal
        // (already present in the ProDMP weights - prodmp_.goal() is used as-is,
        // setGoal() is never called and relative_goal stays false as loaded).
        // ProDMP has no reset(); re-seating the demonstrated initial conditions
        // is the equivalent rewind of the integrator accumulators.
        prodmp_.setInitialConditions(0.0, demo_init_pos_, demo_init_vel_);
        qdmp_.reset();
        RCLCPP_INFO(this->get_logger(),
                    "target_odom_required=false: replaying with the DEMO'S ORIGINAL "
                    "GOAL, no retarget applied.");
        RCLCPP_INFO(this->get_logger(), "Starting ProDMP rollout.");
        step_timer_ = rclcpp::create_timer(
            this, this->get_clock(),
            rclcpp::Duration::from_seconds(dt_),
            std::bind(&ProDmpGazeboExecutorNode::stepCallback, this));
        return;
    }

    // startup_delay_sec_ has elapsed. Do NOT start stepping until the one-shot
    // target retarget has happened: wait for the first free_target_object
    // odometry, re-polling on the node clock, subject to a fail-loud timeout.
    if (!odom_wait_started_) {
        odom_wait_start_ = this->now();
        odom_wait_started_ = true;
    }

    if (!target_odom_received_) {
        const double waited = (this->now() - odom_wait_start_).seconds();
        if (waited >= target_odom_timeout_sec_) {
            RCLCPP_FATAL(this->get_logger(),
                         "No message on target odometry topic '%s' within %.2f s. "
                         "Refusing to replay toward the demonstration's frozen goal - "
                         "the one-shot target retarget is mandatory. Is "
                         "free_target_object running and bridged?",
                         target_odom_topic_.c_str(), target_odom_timeout_sec_);
            throw std::runtime_error(
                "prodmp_gazebo_executor_node: target odometry retarget timed out on '" +
                target_odom_topic_ + "'");
        }
        startup_timer_ = rclcpp::create_timer(
            this, this->get_clock(),
            rclcpp::Duration::from_seconds(0.1),
            std::bind(&ProDmpGazeboExecutorNode::startTimer, this));
        return;
    }

    // Retarget. target_position_ is in world_frame_. Look up the live EE pose
    // (world_frame_ -> ee_frame_) so the ProDMP initial position can be anchored
    // to where the end-effector actually is now. If the TF is not available yet,
    // re-poll on the SAME overall odometry timeout, exactly like the "odometry
    // not received" branch above - no separate TF timeout.
    Eigen::Vector3d ee_now;
    if (!lookupEeNowWithRetry(ee_now)) {
        return;  // re-polling scheduled by lookupEeNowWithRetry(); try again in 0.1 s
    }

    // Native ProDMP retarget - no manual frame bridging.
    //
    // The classic node must do:
    //     dmp_.setGoal(dmp_.y0() + (target_position_ - ee_now));
    // because DMP::setGoal() interprets its argument in the demo-local frame of
    // dmp_.y0(): the retarget displacement has to be measured in world and then
    // re-applied from y0_ by hand.
    //
    // ProDMP is linearly parameterised - the goal is one more linear parameter
    // with its own closed-form basis - so relative_goal (enabled once in the
    // constructor) makes "goal relative to the initial position" a first-class
    // mode. We simply:
    //   a) anchor the initial position to the REAL EE pose read from TF
    //      (init_vel = 0: no reliable live EE velocity, and the demo replay
    //       starts from rest anyway),
    //   b) hand the goal over directly in the SAME world frame.
    // init position and goal are now both in world, so the relative goal locks
    // on with no extra correction term.
    const Eigen::Vector3d original_goal = prodmp_.goal();
    prodmp_.setInitialConditions(/*init_time=*/0.0, ee_now, Eigen::Vector3d::Zero());
    prodmp_.setGoal(target_position_);
    qdmp_.reset();

    RCLCPP_INFO(this->get_logger(),
                "One-shot target retarget (native ProDMP relative goal): "
                "demo goal [%.4f, %.4f, %.4f] -> new goal [%.4f, %.4f, %.4f] (world) | "
                "init_pos anchored to EE pose [%.4f, %.4f, %.4f] (world, %s -> %s) | "
                "target from '%s' | orientation goal left unchanged.",
                original_goal.x(), original_goal.y(), original_goal.z(),
                target_position_.x(), target_position_.y(), target_position_.z(),
                ee_now.x(), ee_now.y(), ee_now.z(),
                world_frame_.c_str(), ee_frame_.c_str(),
                target_odom_topic_.c_str());

    RCLCPP_INFO(this->get_logger(), "Starting ProDMP rollout.");
    step_timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        rclcpp::Duration::from_seconds(dt_),
        std::bind(&ProDmpGazeboExecutorNode::stepCallback, this));
}

void ProDmpGazeboExecutorNode::stepCallback() {
    if (finished_) return;

    // Gripper close ramp - identical to dmp_gazebo_executor_node: walk
    // core::gripper_ramp::interpolateGripperPosition from open to closed over
    // gripper_close_ramp_duration_sec_, gated by the same
    // gripper_trigger_t_ >= 0 && !gripper_trigger_sent_ guard, on the same
    // elapsed_ sim-time base (no dedicated timer).
    if (gripper_trigger_t_ >= 0.0 && !gripper_trigger_sent_) {
        if (!gripper_ramp_active_ && elapsed_ >= gripper_trigger_t_) {
            gripper_ramp_active_ = true;
            gripper_ramp_start_elapsed_ = elapsed_;
            RCLCPP_INFO(this->get_logger(),
                        "Replay: starting %.1f s gripper close ramp at elapsed=%.4f s "
                        "(target t=%.4f s)",
                        gripper_close_ramp_duration_sec_, elapsed_, gripper_trigger_t_);
        }
        if (gripper_ramp_active_) {
            const double ramp_elapsed = elapsed_ - gripper_ramp_start_elapsed_;
            std_msgs::msg::Float64 cmd;
            if (ramp_elapsed >= gripper_close_ramp_duration_sec_) {
                cmd.data = gripper_closed_position_;
                gripper_pub_->publish(cmd);
                gripper_ramp_active_ = false;
                gripper_trigger_sent_ = true;
                gripper_close_complete_pub_->publish(std_msgs::msg::Empty());
                RCLCPP_INFO(this->get_logger(),
                            "Replay: gripper close ramp complete (%.3f) at elapsed=%.4f s",
                            gripper_closed_position_, elapsed_);
            } else {
                cmd.data = core::gripper_ramp::interpolateGripperPosition(
                    ramp_elapsed, gripper_close_ramp_duration_sec_,
                    gripper_open_position_, gripper_closed_position_);
                gripper_pub_->publish(cmd);
            }
        }
    }

    geometry_msgs::msg::PoseStamped msg;
    const rclcpp::Time stamp = this->now();
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;

    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = stamp;  // SAME stamp as msg - lets the consumer detect
                                      // cross-topic skew between the two samples.
    twist_msg.header.frame_id = frame_id_;

    bool at_end = (elapsed_ + dt_) >= prodmp_.tau();

    if (!at_end) {
        // Step the ProDMP (position) and QuaternionDMP (orientation) forward by dt.
        Eigen::Vector3d pos = prodmp_.step(dt_);
        Eigen::Quaterniond quat = qdmp_.step(dt_);
        // Analytic velocity state, already integrated inside step() above (see
        // core::ProDMP::velocity() / core::QuaternionDMP::omega()) - not a
        // finite-difference reconstruction.
        Eigen::Vector3d vel = prodmp_.velocity();
        Eigen::Vector3d omega = qdmp_.omega();
        elapsed_ += dt_;

        msg.pose.position.x = pos.x();
        msg.pose.position.y = pos.y();
        msg.pose.position.z = pos.z();
        msg.pose.orientation.w = quat.w();
        msg.pose.orientation.x = quat.x();
        msg.pose.orientation.y = quat.y();
        msg.pose.orientation.z = quat.z();

        twist_msg.twist.linear.x = vel.x();
        twist_msg.twist.linear.y = vel.y();
        twist_msg.twist.linear.z = vel.z();
        twist_msg.twist.angular.x = omega.x();
        twist_msg.twist.angular.y = omega.y();
        twist_msg.twist.angular.z = omega.z();
    } else {
        // Clamp explicitly to the goal attractor once tau is reached.
        Eigen::Vector3d goal = prodmp_.goal();
        msg.pose.position.x = goal.x();
        msg.pose.position.y = goal.y();
        msg.pose.position.z = goal.z();
        Eigen::Quaterniond qgoal = qdmp_.goal();
        msg.pose.orientation.w = qgoal.w();
        msg.pose.orientation.x = qgoal.x();
        msg.pose.orientation.y = qgoal.y();
        msg.pose.orientation.z = qgoal.z();

        // Target is at rest at the goal: feedforward is explicitly zero here,
        // NOT the last pre-clamp rollout velocity - otherwise a feedforward-fed
        // controller would keep pushing past the goal for one tick.
        // twist_msg.twist is already zero-initialized by TwistStamped's default
        // construction; left explicit (all six fields) so the "target is at
        // rest" invariant is visible at the call site, not implicit in a
        // default constructor a future reader might not think to check.
        twist_msg.twist.linear.x = 0.0;
        twist_msg.twist.linear.y = 0.0;
        twist_msg.twist.linear.z = 0.0;
        twist_msg.twist.angular.x = 0.0;
        twist_msg.twist.angular.y = 0.0;
        twist_msg.twist.angular.z = 0.0;

        finished_ = true;
        RCLCPP_INFO(this->get_logger(), "ProDMP rollout completed at goal.");
    }

    pose_pub_->publish(msg);
    twist_pub_->publish(twist_msg);

    if (finished_ && step_timer_) {
        step_timer_->cancel();
    }
}

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning
