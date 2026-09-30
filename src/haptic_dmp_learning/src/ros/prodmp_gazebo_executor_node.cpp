#include "haptic_dmp_learning/ros/prodmp_gazebo_executor_node.hpp"
#include "haptic_dmp_learning/core/dmp.hpp"
#include "haptic_dmp_learning/core/dmp_io.hpp"
#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "haptic_dmp_learning/core/demo_csv_io.hpp"
#include "haptic_dmp_learning/core/gripper_ramp.hpp"
#include "haptic_dmp_learning/core/demo_params.hpp"
#include "haptic_dmp_learning/ros/grasp_command_input.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <chrono>
#include <cmath>
#include <fstream>
#include <cstdio>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <vector>

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

    // syncControllerAlignmentOverride(): name of the downstream cartesian controller's ROS 2 node
    // (in ros2_control, the controller runs as its own node named after the controller entry in
    // the controllers YAML - see franka_gazebo_overrides/franka_gazebo_controllers.yaml and the
    // "cartesian_impedance_controller" spawner argument in
    // gazebo_cartesian_impedance_control.launch.py) and how long to wait for its parameter
    // service before failing loud.
    cartesian_controller_node_name_ = this->declare_parameter<std::string>(
        "cartesian_controller_node_name", "cartesian_impedance_controller");
    controller_param_sync_timeout_sec_ =
        this->declare_parameter<double>("controller_param_sync_timeout_sec", 5.0);

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
    // Where the grasp goal comes from. "parameters" (default) = every existing branch, unchanged.
    // "grasp_command" = satellite_grasp_msgs/GraspCommand from grasp_planner_node (continuous mode only).
    grasp_goal_source_ = this->declare_parameter<std::string>("grasp_goal_source", "parameters");
    if (grasp_goal_source_ != "parameters" && grasp_goal_source_ != "grasp_command") {
        throw std::invalid_argument("parameter 'grasp_goal_source' must be 'parameters' or 'grasp_command', got '" +
                                    grasp_goal_source_ + "'");
    }
    grasp_command_mode_ = (grasp_goal_source_ == "grasp_command");

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
    // Displacement (not the raw absolute goal) that the demo covered, for the satellite
    // rotation model below: anchorAndRotate() adds this to a runtime ee anchor to get the
    // grasp point at theta = 0, so it must already be relative to demo_init_pos_ - delegated
    // to ProDMP::demoDisplacement() (goal() - initPos()) instead of recomputed here, so there
    // is exactly one place that does this subtraction. Captured now, before anything else can
    // call setGoal()/setInitialConditions() and move goal_/init_pos_ out from under it.
    demo_grasp_goal_ = prodmp_.demoDisplacement();

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

    // satellite_rotation_mode="continuous": read + validate the extra parameters (throws on any
    // error: main() logs it as fatal and exits), create the odometry subscription / status topic.
    // Untouched for satellite_rotation_enabled=false and for mode="frozen".
    continuous_ = satellite_rotation_enabled_ && satellite_rotation_mode_ == "continuous";
    if (grasp_command_mode_ && !continuous_) {
        throw std::invalid_argument(
            "grasp_goal_source='grasp_command' requires satellite_rotation_enabled=true and "
            "satellite_rotation_mode='continuous'");
    }
    if (continuous_) {
        setupContinuous();
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

void ProDmpGazeboExecutorNode::ensureControllerParamClient() {
    // See this method's full doc comment in the header for WHY a dedicated helper node is
    // required (not just "an explicit executor") and why never destroying it is fine here.
    // Blocking client: we are still in startTimer(), before step_timer_ exists, not in the 1 kHz
    // RT loop - blocking here is explicitly fine, unlike in stepCallback().
    if (controller_param_client_) return;

    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    bool use_sim_time = false;
    if (this->has_parameter("use_sim_time")) {
        use_sim_time = this->get_parameter("use_sim_time").as_bool();
    }
    options.parameter_overrides({{"use_sim_time", use_sim_time}});
    param_sync_helper_node_ = std::make_shared<rclcpp::Node>(
        "prodmp_param_sync_helper", this->get_namespace(), options);
    controller_param_client_ = std::make_shared<rclcpp::SyncParametersClient>(
        param_sync_helper_node_, cartesian_controller_node_name_);
}

void ProDmpGazeboExecutorNode::syncControllerAlignmentOverride(const Eigen::Vector3d& ee_now) {
    namespace cps = core::controller_param_sync;

    const std::vector<double> override_value = cps::toAlignmentOverrideValue(ee_now);
    if (!cps::isValidAlignmentOverrideValue(override_value)) {
        // Defensive only: lookupEeNowWithRetry() never hands back NaN today (a failed TF lookup
        // re-polls or throws, it never returns a NaN ee_now). If it ever did, forwarding that
        // silently would make the controller treat it as "not set" and silently fall back to the
        // very race this fix exists to remove - fail loud instead.
        const std::string m =
            "syncControllerAlignmentOverride: ee_now = [" + std::to_string(ee_now.x()) + ", " +
            std::to_string(ee_now.y()) + ", " + std::to_string(ee_now.z()) +
            "] is not a valid override value (NaN component) - refusing to hand a bogus '" +
            std::string(cps::kInitialAlignmentPositionOverrideParamName) + "' to the controller.";
        RCLCPP_FATAL(this->get_logger(), "%s", m.c_str());
        throw std::runtime_error("prodmp_gazebo_executor_node: " + m);
    }

    ensureControllerParamClient();

    // NOTE: pass a std::chrono::duration<double> (not nanoseconds) here - SyncParametersClient's
    // public wait_for_service()/set_parameters_atomically() overloads are templated on
    // duration<RepT, RatioT> and forward to a PROTECTED nanoseconds overload internally; calling
    // that protected overload directly does not compile from outside the class.
    const std::chrono::duration<double> timeout(controller_param_sync_timeout_sec_);

    if (!controller_param_client_->wait_for_service(timeout)) {
        const std::string m = "syncControllerAlignmentOverride: parameter service of controller "
                               "node '" + cartesian_controller_node_name_ + "' not available "
                               "after " + std::to_string(controller_param_sync_timeout_sec_) +
                               " s. Refusing to start the rollout without the alignment override "
                               "in place - is the controller loaded and activated before this "
                               "node starts, and is cartesian_controller_node_name_ correct?";
        RCLCPP_FATAL(this->get_logger(), "%s", m.c_str());
        throw std::runtime_error("prodmp_gazebo_executor_node: " + m);
    }

    RCLCPP_INFO(this->get_logger(),
                "Setting '%s' on controller node '%s' to ee_now = [%.4f, %.4f, %.4f] m (same "
                "value anchoring the ProDMP's initial conditions) BEFORE starting step_timer_ - "
                "this eliminates the FrameAligner's dependency on which /target_pose sample the "
                "controller's RealtimeBuffer happens to read first.",
                cps::kInitialAlignmentPositionOverrideParamName,
                cartesian_controller_node_name_.c_str(), ee_now.x(), ee_now.y(), ee_now.z());

    const rcl_interfaces::msg::SetParametersResult result = controller_param_client_->set_parameters_atomically(
        {rclcpp::Parameter(cps::kInitialAlignmentPositionOverrideParamName, override_value)}, timeout);

    if (!result.successful) {
        const std::string m = "syncControllerAlignmentOverride: controller node '" +
                               cartesian_controller_node_name_ + "' rejected '" +
                               std::string(cps::kInitialAlignmentPositionOverrideParamName) +
                               "': " + result.reason;
        RCLCPP_FATAL(this->get_logger(), "%s", m.c_str());
        throw std::runtime_error("prodmp_gazebo_executor_node: " + m);
    }

    RCLCPP_INFO(this->get_logger(), "Confirmed: controller node '%s' accepted '%s'.",
                cartesian_controller_node_name_.c_str(), cps::kInitialAlignmentPositionOverrideParamName);
}

void ProDmpGazeboExecutorNode::syncControllerSkipInitialAlignment(bool skip) {
    namespace cps = core::controller_param_sync;

    ensureControllerParamClient();  // same client as syncControllerAlignmentOverride() (reused)

    const std::chrono::duration<double> timeout(controller_param_sync_timeout_sec_);

    if (!controller_param_client_->wait_for_service(timeout)) {
        const std::string m = "syncControllerSkipInitialAlignment: parameter service of "
                               "controller node '" + cartesian_controller_node_name_ +
                               "' not available after " +
                               std::to_string(controller_param_sync_timeout_sec_) +
                               " s. Refusing to start the rollout without '" +
                               std::string(cps::kSkipInitialAlignmentParamName) + "' in place.";
        RCLCPP_FATAL(this->get_logger(), "%s", m.c_str());
        throw std::runtime_error("prodmp_gazebo_executor_node: " + m);
    }

    RCLCPP_INFO(this->get_logger(),
                "Setting '%s' on controller node '%s' to %s BEFORE starting step_timer_ - world "
                "and the robot base frame coincide exactly for this branch (anchorAndRotate() "
                "already publishes in absolute world coordinates), so FrameAligner should apply "
                "identity, not even the initial_alignment_position_override-based offset.",
                cps::kSkipInitialAlignmentParamName, cartesian_controller_node_name_.c_str(),
                skip ? "true" : "false");

    const rcl_interfaces::msg::SetParametersResult result = controller_param_client_->set_parameters_atomically(
        {rclcpp::Parameter(cps::kSkipInitialAlignmentParamName, skip)}, timeout);

    if (!result.successful) {
        const std::string m = "syncControllerSkipInitialAlignment: controller node '" +
                               cartesian_controller_node_name_ + "' rejected '" +
                               std::string(cps::kSkipInitialAlignmentParamName) +
                               "': " + result.reason;
        RCLCPP_FATAL(this->get_logger(), "%s", m.c_str());
        throw std::runtime_error("prodmp_gazebo_executor_node: " + m);
    }

    RCLCPP_INFO(this->get_logger(), "Confirmed: controller node '%s' accepted '%s'.",
                cartesian_controller_node_name_.c_str(), cps::kSkipInitialAlignmentParamName);
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

        if (continuous_) {
            // ee_now here is ee_plan: the anchor of the plan (p0 = ee_plan + demo goal). The rollout
            // itself starts later, at the trigger crossing (onPhaseSample -> startContinuousRollout).
            beginContinuous(ee_now);
            return;
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
        // 2. Apply satellite rotation model (axis, center, theta) in world frame. The anchoring +
        // rotation math lives in core::satellite_intercept::anchorAndRotate (shared with the
        // continuous branch); it performs exactly the operations that used to be inlined here:
        //   p_demo = ee_now + demo_grasp_goal_;  p_rot = center + AngleAxisd(theta, axis.normalized()) * (p_demo - center)
        const double theta_rad = satellite_rotation_frozen_phase_deg_ * M_PI / 180.0;
        const core::satellite_intercept::AnchoredGoal anchored = core::satellite_intercept::anchorAndRotate(
            ee_now, demo_grasp_goal_, satellite_rotation_center_, satellite_rotation_axis_, theta_rad);
        const Eigen::Vector3d& p_grasp_demo_world = anchored.p_demo_world;
        const Eigen::Vector3d& p_grasp_rotated_world = anchored.p_rotated_world;

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
        // ee_now is the ProDMP's actual init position for this branch (setInitialConditions()
        // above), so it's also what the first published /target_pose sample will equal (thanks to
        // the s=0-exact first tick) - the correct value to hand the controller as its expected
        // initial position.
        syncControllerAlignmentOverride(ee_now);
        // world and fer_link0 coincide exactly (confirmed), and anchorAndRotate() above already
        // anchored init_pos_/goal in absolute world coordinates - so the alignment offset itself
        // should be identity, not even the override-based one just set above (that override still
        // has a ~1.89 mm residual from sampling activation_ee_position_ and ee_now at two
        // different times). Deliberately kept TOGETHER with the override call, not instead of it -
        // see syncControllerSkipInitialAlignment()'s doc comment for the defensive fallback this
        // gives if a future branch forgets to call it.
        syncControllerSkipInitialAlignment(true);
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
        // No live ee_now in this branch (no TF lookup, no anchorAndRotate): the ProDMP's actual
        // init position is demo_init_pos_ (setInitialConditions() above), so THAT is what the
        // first published sample will equal - not ee_now, which isn't even in scope here.
        syncControllerAlignmentOverride(demo_init_pos_);
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
    // ee_now is the ProDMP's actual init position for this branch (setInitialConditions() above).
    syncControllerAlignmentOverride(ee_now);
    // Same reasoning as the satellite_rotation branch above: target_position_/ee_now are already
    // in absolute world coordinates and world == fer_link0, so identity is correct here too. See
    // that call site and syncControllerSkipInitialAlignment()'s doc comment.
    syncControllerSkipInitialAlignment(true);
    step_timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        rclcpp::Duration::from_seconds(dt_),
        std::bind(&ProDmpGazeboExecutorNode::stepCallback, this));
}

void ProDmpGazeboExecutorNode::stepCallback() {
    if (finished_) return;

    // Continuous mode ONLY (rollout_clock_ is null in frozen / disabled): the time base is the node
    // clock, elapsed_ = now - t_roll0, and the models are advanced by the MEASURED step. In frozen
    // mode step_dt is dt_ except on the very first tick (see frozen_first_tick_pending_ below).
    double step_dt = dt_;
    bool continuous_at_end = false;
    if (rollout_clock_) {
        const auto tick = rollout_clock_->update(this->now().seconds());
        elapsed_ = tick.elapsed;
        step_dt = tick.dt_step;
        continuous_at_end = tick.at_end;
    } else if (frozen_first_tick_pending_) {
        // core::satellite_intercept::frozenStepDt() - see its doc comment for why the
        // first tick must be free (step_dt=0.0). Only this one tick is free; every
        // following tick uses step_dt=dt_ as before.
        step_dt = core::satellite_intercept::frozenStepDt(/*is_first_tick=*/true, dt_);
        frozen_first_tick_pending_ = false;
    }

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

    bool at_end = rollout_clock_ ? continuous_at_end
                                 : core::satellite_intercept::fixedStepAtEnd(elapsed_, dt_, prodmp_.tau());

    if (!at_end) {
        // Step the ProDMP (position) and QuaternionDMP (orientation) forward by dt.
        Eigen::Vector3d pos = prodmp_.step(step_dt);
        Eigen::Quaterniond quat = qdmp_.step(step_dt);
        // Analytic velocity state, already integrated inside step() above (see
        // core::ProDMP::velocity() / core::QuaternionDMP::omega()) - not a
        // finite-difference reconstruction.
        Eigen::Vector3d vel = prodmp_.velocity();
        Eigen::Vector3d omega = qdmp_.omega();
        if (!rollout_clock_) {
            // elapsed_ still starts at 0.0 and advances by step_dt each tick - only the
            // increment itself changes: 0.0 on the freebie first tick, dt_ every tick after.
            elapsed_ += step_dt;
        } else {
            last_cmd_pos_ = pos;
            have_last_cmd_ = true;
        }

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
        if (rollout_clock_) {
            finishContinuousRollout(goal);
        }
    }

    pose_pub_->publish(msg);
    twist_pub_->publish(twist_msg);

    if (finished_ && step_timer_) {
        step_timer_->cancel();
    }
}


// ======================================================================================
// satellite_rotation_mode = "continuous"
// ======================================================================================
namespace {
std::string fmtG(double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.9g", v);
    return std::string(buf);
}
}  // namespace

void ProDmpGazeboExecutorNode::setupContinuous() {
    namespace si = core::satellite_intercept;

    // Mandatory parameters are declared by TYPE ONLY (no default): an unset one comes back NOT_SET
    // and validateContinuous() names it in the exception.
    auto opt_double = [this](const char* name) -> std::optional<double> {
        const auto v = this->declare_parameter(name, rclcpp::ParameterType::PARAMETER_DOUBLE);
        if (v.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) return std::nullopt;
        return v.get<double>();
    };
    auto opt_string = [this](const char* name) -> std::optional<std::string> {
        const auto v = this->declare_parameter(name, rclcpp::ParameterType::PARAMETER_STRING);
        if (v.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) return std::nullopt;
        return v.get<std::string>();
    };
    auto opt_vec = [this](const char* name) -> std::optional<std::vector<double>> {
        const auto v = this->declare_parameter(name, rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
        if (v.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) return std::nullopt;
        return v.get<std::vector<double>>();
    };

    si::ContinuousInputs in;
    in.rotation_enabled = satellite_rotation_enabled_;
    in.use_sim_time = this->get_parameter("use_sim_time").as_bool();
    in.target_odom_required = target_odom_required_;
    // axis / center / omega have code defaults in the frozen branch: in continuous they must be
    // passed explicitly (an explicit override present on the node).
    const auto& overrides = this->get_node_parameters_interface()->get_parameter_overrides();
    in.axis_explicit = overrides.count("satellite_rotation_axis") > 0;
    in.center_explicit = overrides.count("satellite_rotation_center") > 0;
    in.omega_explicit = overrides.count("satellite_rotation_angular_velocity_deg_s") > 0;
    in.axis = satellite_rotation_axis_;
    in.center = satellite_rotation_center_;
    in.omega_deg_s = satellite_rotation_angular_velocity_deg_s_;

    in.intercept_phase_deg = opt_double("satellite_intercept_phase_deg");
    in.contact_time_s = opt_double("satellite_contact_time_s");
    in.phase_source = opt_string("satellite_phase_source");
    in.odom_topic = opt_string("satellite_odom_topic");
    in.q_ref = opt_vec("satellite_q_ref");
    in.model_phase0_deg = opt_double("satellite_model_phase0_deg");

    in.contact_time_tolerance_s = this->declare_parameter<double>("contact_time_tolerance_s", 0.1);
    in.allow_contact_time_mismatch = this->declare_parameter<bool>("allow_contact_time_mismatch", false);
    in.phase_consistency_window_s = this->declare_parameter<double>("phase_consistency_window_s", 5.0);
    in.phase_consistency_tol_deg = this->declare_parameter<double>("phase_consistency_tol_deg", 1.0);
    in.trigger_timeout_margin_s = this->declare_parameter<double>("trigger_timeout_margin_s", 30.0);
    in.rollout_time_tol_s = this->declare_parameter<double>("rollout_time_tol_s", 0.05);
    in.tau = prodmp_.tau();
    in.dt = dt_;
    in.grasp_command_mode = grasp_command_mode_;

    if (grasp_command_mode_) {
        // Parameters that exist ONLY in grasp_command mode (nothing new is declared in "parameters" mode
        // except grasp_goal_source itself).
        grasp_command_topic_ = this->declare_parameter<std::string>("grasp_command_topic", "/grasp_command");
        if (grasp_command_topic_.empty()) throw std::invalid_argument("parameter 'grasp_command_topic' must not be empty");
        grasp_command_timeout_s_ = this->declare_parameter<double>("grasp_command_timeout_s", 300.0);
        if (!(std::isfinite(grasp_command_timeout_s_) && grasp_command_timeout_s_ > 0.0)) {
            throw std::invalid_argument("parameter 'grasp_command_timeout_s' must be finite and > 0");
        }
        // <= 0 disables the age check (stamps in the future are always rejected).
        grasp_command_max_age_s_ = this->declare_parameter<double>("grasp_command_max_age_s", 0.0);
        if (!std::isfinite(grasp_command_max_age_s_)) {
            throw std::invalid_argument("parameter 'grasp_command_max_age_s' must be finite (<= 0 disables the check)");
        }
        const double omega_tol_deg_s = this->declare_parameter<double>("omega_consistency_tol_deg_per_s", 0.1);
        if (!(std::isfinite(omega_tol_deg_s) && omega_tol_deg_s > 0.0)) {
            throw std::invalid_argument("parameter 'omega_consistency_tol_deg_per_s' must be finite and > 0");
        }
        omega_consistency_tol_rad_s_ = core::degToRad(omega_tol_deg_s);
        const double phase_tol_deg = this->declare_parameter<double>("grasp_command_phase_tol_deg", 1.0);
        if (!(std::isfinite(phase_tol_deg) && phase_tol_deg > 0.0)) {
            throw std::invalid_argument("parameter 'grasp_command_phase_tol_deg' must be finite and > 0");
        }
        grasp_command_phase_tol_rad_ = core::degToRad(phase_tol_deg);
        // Mandatory (no default), like the planner's: world == robot base is only true for robot_base_world = 0.
        const double robot_base_tol_m = this->declare_parameter<double>("robot_base_world_tol_m", 1e-6);
        if (!(std::isfinite(robot_base_tol_m) && robot_base_tol_m >= 0.0)) {
            throw std::invalid_argument("parameter 'robot_base_world_tol_m' must be finite and >= 0");
        }
        const auto rbw = opt_vec("robot_base_world");
        if (!rbw) {
            throw std::invalid_argument(
                "grasp_goal_source='grasp_command': mandatory parameter 'robot_base_world' was not provided");
        }
        if (rbw->size() != 3) throw std::invalid_argument("parameter 'robot_base_world' must have exactly 3 elements");
        si::checkRobotBaseWorldIsZero(Eigen::Vector3d((*rbw)[0], (*rbw)[1], (*rbw)[2]), robot_base_tol_m);
        // Hash of the file the ProDMP was loaded from, compared with the message's weights_sha256.
        weights_sha256_ = core::demo_params::sha256FileHex(weights_yaml_path_);
    }

    cc_ = si::validateContinuous(in);  // throws std::invalid_argument naming the parameter
    const auto& c = *cc_;

    // grasp_command mode: theta_int and T_total are not known yet, the trigger is built on acceptance.
    if (!grasp_command_mode_) {
        trigger_ = std::make_unique<si::InterceptTrigger>(c.theta_int_rad, c.omega_rad_s, c.contact_time_s);
    }
    if (c.measured) {
        phase_tracker_ = std::make_unique<si::PhaseTracker>(c.axis_unit, c.q_ref, 0.5);
        phase_checker_ = std::make_unique<si::PhaseConsistencyChecker>(
            c.omega_rad_s, c.consistency_window_s, c.consistency_tol_deg);
        satellite_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            c.odom_topic, rclcpp::QoS(100),
            std::bind(&ProDmpGazeboExecutorNode::onSatelliteOdom, this, std::placeholders::_1));
    }
    if (grasp_command_mode_) {
        // Same QoS as the planner's publisher: reliable + transient_local + depth 1 (one-shot, retained).
        // Created here so a message published before beginContinuous() is buffered, not lost.
        grasp_command_sub_ = this->create_subscription<satellite_grasp_msgs::msg::GraspCommand>(
            grasp_command_topic_, rclcpp::QoS(1).reliable().transient_local(),
            std::bind(&ProDmpGazeboExecutorNode::onGraspCommand, this, std::placeholders::_1));
    }
    continuous_status_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "~/continuous_status", rclcpp::QoS(10));

    if (grasp_command_mode_) {
        const bool dt_differs = std::fabs(dt_ - si::kPlannerRolloutDtS) > 1e-12;
        RCLCPP_INFO(this->get_logger(),
                    "grasp_goal_source=grasp_command: topic '%s' (reliable, transient_local, depth 1), timeout %.1f s, "
                    "max_age %s, omega tol %.4f rad/s, phase tol %.3f deg, weights sha256 %s.\n"
                    "  center=[%.4f, %.4f, %.4f] axis=[%.6f, %.6f, %.6f] omega(param)=%.6f rad/s  tau=%.5f s  "
                    "dt_=%.6f s (planner rollout dt %.6f s)",
                    grasp_command_topic_.c_str(), grasp_command_timeout_s_,
                    grasp_command_max_age_s_ > 0.0 ? (std::to_string(grasp_command_max_age_s_) + " s").c_str() : "off",
                    omega_consistency_tol_rad_s_, core::radToDeg(grasp_command_phase_tol_rad_), weights_sha256_.c_str(),
                    c.center.x(), c.center.y(), c.center.z(), c.axis_unit.x(), c.axis_unit.y(), c.axis_unit.z(),
                    c.omega_rad_s, c.tau, dt_, si::kPlannerRolloutDtS);
        if (dt_differs) {
            RCLCPP_WARN(this->get_logger(),
                        "executor dt_=%.6f s differs from the planner's rollout dt %.6f s: the trajectory the planner "
                        "validated was integrated with a different step (measured-time stepping makes the executor "
                        "path near, not identical to it).", dt_, si::kPlannerRolloutDtS);
        }
        return;  // no theta_trig / T_total yet: they come from the message
    }
    const std::string source_desc = c.measured ? ("measured, odom=" + c.odom_topic) : std::string("model");
    RCLCPP_INFO(this->get_logger(),
                "satellite_rotation_mode=continuous (v0, open-loop predictive intercept):\n"
                "  center=[%.4f, %.4f, %.4f] axis(normalized)=[%.6f, %.6f, %.6f] omega=%.6f deg/s (%.9f rad/s)\n"
                "  intercept_phase=%.4f deg  contact_time(T_total)=%.4f s (expected from code: tau %.4f + dt %.4f = %.4f s, "
                "tol %.3f s%s)\n"
                "  phase_source=%s  consistency window=%.2f s tol=%.3f deg  trigger_timeout=%.2f s  rollout_time_tol=%.3f s\n"
                "  theta_trig=%.4f deg (mod 360)",
                c.center.x(), c.center.y(), c.center.z(), c.axis_unit.x(), c.axis_unit.y(), c.axis_unit.z(),
                c.omega_deg_s, c.omega_rad_s, core::radToDeg(c.theta_int_rad), c.contact_time_s, c.tau, c.dt,
                c.expected_contact_time_s, in.contact_time_tolerance_s,
                c.contact_time_mismatch ? " - MISMATCH ALLOWED by allow_contact_time_mismatch=true" : "",
                source_desc.c_str(), c.consistency_window_s, c.consistency_tol_deg, c.trigger_timeout_s,
                c.rollout_time_tol_s, core::radToDeg(trigger_->thetaTrigRad()));
    if (c.contact_time_mismatch) {
        RCLCPP_WARN(this->get_logger(),
                    "satellite_contact_time_s=%.4f differs from the expected %.4f s by more than %.3f s "
                    "(allowed explicitly).", c.contact_time_s, c.expected_contact_time_s, in.contact_time_tolerance_s);
    }
}

bool ProDmpGazeboExecutorNode::lookupEeNonBlocking(Eigen::Vector3d& ee) {
    try {
        const auto tf = tf_buffer_->lookupTransform(world_frame_, ee_frame_, tf2::TimePointZero);
        ee = Eigen::Vector3d(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
        return true;
    } catch (const tf2::TransformException&) {
        return false;
    }
}

void ProDmpGazeboExecutorNode::beginContinuous(const Eigen::Vector3d& ee_plan) {
    namespace si = core::satellite_intercept;
    const auto& c = *cc_;
    // The time base is the node clock: it must really be the simulation clock.
    if (!this->get_clock()->ros_time_is_active()) {
        RCLCPP_FATAL(this->get_logger(),
                     "continuous mode: the node clock is not in ROS(sim) time (ros_time_is_active()==false) "
                     "although use_sim_time=true was requested. Refusing to start.");
        throw std::runtime_error("prodmp_gazebo_executor_node: node clock is not the simulation clock");
    }

    ee_plan_ = ee_plan;
    if (grasp_command_mode_) {
        beginWaitingForCommand(ee_plan);
        return;
    }
    // Same anchoring + rotation function as the frozen branch, evaluated at theta_int.
    const si::AnchoredGoal plan = si::anchorAndRotate(ee_plan, demo_grasp_goal_, c.center, c.axis_unit, c.theta_int_rad);
    p0_ = plan.p_demo_world;          // grasp point at theta = 0
    p_goal_ = plan.p_rotated_world;   // c + R(theta_int)(p0 - c)
    const Eigen::Vector3d shoulder(0.0, 0.0, 0.333);
    RCLCPP_INFO(this->get_logger(),
                "continuous plan: ee_plan=[%.4f, %.4f, %.4f] demo_grasp_goal=[%.4f, %.4f, %.4f]\n"
                "  p0(theta=0)=[%.4f, %.4f, %.4f]  center=[%.4f, %.4f, %.4f]  axis=[%.6f, %.6f, %.6f]\n"
                "  r_eff=%.4f m (p0 to axis)  |p0|(base)=%.4f m  |p0-shoulder(0,0,0.333)|=%.4f m\n"
                "  theta_int=%.4f deg -> p_goal=[%.4f, %.4f, %.4f]  (world %s -> %s)",
                ee_plan.x(), ee_plan.y(), ee_plan.z(), demo_grasp_goal_.x(), demo_grasp_goal_.y(), demo_grasp_goal_.z(),
                p0_.x(), p0_.y(), p0_.z(), c.center.x(), c.center.y(), c.center.z(),
                c.axis_unit.x(), c.axis_unit.y(), c.axis_unit.z(),
                si::distanceFromAxis(p0_, c.center, c.axis_unit), p0_.norm(), (p0_ - shoulder).norm(),
                core::radToDeg(c.theta_int_rad), p_goal_.x(), p_goal_.y(), p_goal_.z(),
                world_frame_.c_str(), ee_frame_.c_str());

    const double now_s = this->now().seconds();
    t_wait_start_ = now_s;
    last_wait_log_ = now_s;
    cstate_ = ContinuousState::kWaiting;
    RCLCPP_INFO(this->get_logger(),
                "continuous: WAITING for the trigger at theta_trig=%.4f deg (theta_int %.4f - omega*T_total %.4f), "
                "timeout %.1f s.%s",
                core::radToDeg(trigger_->thetaTrigRad()), core::radToDeg(c.theta_int_rad),
                c.omega_deg_s * c.contact_time_s, c.trigger_timeout_s,
                c.measured ? " Phase from odometry (consumption gated on the phase-consistency check)."
                           : " Phase from the model theta0 + omega*t_sim.");

    continuous_status_timer_ = rclcpp::create_timer(
        this, this->get_clock(), rclcpp::Duration::from_seconds(0.05),
        std::bind(&ProDmpGazeboExecutorNode::publishContinuousStatus, this));
    if (!c.measured) {
        model_phase_timer_ = rclcpp::create_timer(
            this, this->get_clock(), rclcpp::Duration::from_seconds(dt_),
            std::bind(&ProDmpGazeboExecutorNode::onModelPhaseTick, this));
    }
}

void ProDmpGazeboExecutorNode::beginWaitingForCommand(const Eigen::Vector3d& ee_plan) {
    // Controller state BEFORE any wait (as the frozen branch does right before its rollout): the goal is an
    // absolute world position, so the FrameAligner must be the identity. skip_initial_alignment is a plain
    // controller parameter: it does not expire and survives FrameAligner::reset() (ros_utils.hpp,
    // cartesian_impedance_controller.cpp: reset() only in on_activate()), so a long wait cannot undo it.
    syncControllerAlignmentOverride(ee_plan);
    syncControllerSkipInitialAlignment(true);

    const double now_s = this->now().seconds();
    t_cmd_wait_start_ = now_s;
    cstate_ = ContinuousState::kWaitingCommand;
    RCLCPP_INFO(this->get_logger(),
                "continuous: WAITING for a GraspCommand on '%s' (ee_plan=[%.4f, %.4f, %.4f], timeout %.1f s).",
                grasp_command_topic_.c_str(), ee_plan.x(), ee_plan.y(), ee_plan.z(), grasp_command_timeout_s_);
    continuous_status_timer_ = rclcpp::create_timer(
        this, this->get_clock(), rclcpp::Duration::from_seconds(0.05),
        std::bind(&ProDmpGazeboExecutorNode::publishContinuousStatus, this));
    if (pending_command_) {
        const auto msg = pending_command_;
        pending_command_.reset();
        acceptGraspCommand(*msg);
    }
}

void ProDmpGazeboExecutorNode::onGraspCommand(const satellite_grasp_msgs::msg::GraspCommand::SharedPtr msg) {
    if (command_seen_) {
        RCLCPP_WARN(this->get_logger(),
                    "GraspCommand (stamp %.4f s, status %u) ignored: v1 is one-shot, the first message was already taken.",
                    rclcpp::Time(msg->header.stamp).seconds(), static_cast<unsigned>(msg->status));
        return;
    }
    command_seen_ = true;
    if (cstate_ == ContinuousState::kWaitingCommand) {
        acceptGraspCommand(*msg);
    } else {
        pending_command_ = msg;  // consumed by beginWaitingForCommand()
        RCLCPP_INFO(this->get_logger(), "GraspCommand received before the executor is ready: kept for later.");
    }
}

void ProDmpGazeboExecutorNode::acceptGraspCommand(const satellite_grasp_msgs::msg::GraspCommand& msg) {
    namespace si = core::satellite_intercept;
    auto& c = *cc_;
    si::GraspCommandPlan plan;
    try {
        si::GraspCommandCheckParams p;
        p.now_s = this->now().seconds();
        p.world_frame = world_frame_;
        p.omega_param_rad_s = c.omega_rad_s;
        p.omega_tol_rad_s = omega_consistency_tol_rad_s_;
        p.tau_s = prodmp_.tau();
        p.local_weights_sha256 = weights_sha256_;
        p.max_age_s = grasp_command_max_age_s_;
        plan = si::validateGraspCommand(toGraspCommandInputs(msg), p);
        si::checkStartPositionMatches(ee_plan_, plan.start_position);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "%s", e.what());
        throw;
    }

    // From here the config carries the message's theta_int / T_total, so the shared code (status,
    // at_end report, timeout) reads them exactly as it reads the parameters in the other mode.
    c.theta_int_rad = plan.theta_star_rad;
    c.contact_time_s = plan.contact_time_s;
    c.trigger_timeout_s = si::InterceptTrigger::timeoutS(core::radToDeg(plan.omega_rad_s), plan.contact_time_s,
                                                          c.trigger_timeout_margin_s);
    trigger_ = std::make_unique<si::InterceptTrigger>(plan.theta_star_rad, plan.omega_rad_s, plan.contact_time_s);
    p_goal_ = plan.goal_position;
    // Grasp point at theta = 0, so that p_handle (diagnostics) = center + R(theta)(p0 - center) equals the goal
    // at theta_star.
    p0_ = si::rotateAboutCenter(c.center, c.axis_unit, -plan.theta_star_rad, p_goal_);
    p_start_cmd_ = plan.start_position;
    snapshot_theta_rad_ = plan.snapshot_theta_rad;
    snapshot_t_s_ = plan.snapshot_t_s;
    command_omega_rad_s_ = plan.omega_rad_s;
    phase_coherence_pending_ = true;

    const double now_s = this->now().seconds();
    t_wait_start_ = now_s;
    last_wait_log_ = now_s;
    cstate_ = ContinuousState::kWaiting;
    RCLCPP_INFO(this->get_logger(),
                "GraspCommand ACCEPTED (stamp %.4f s, age %.3f s): goal=[%.4f, %.4f, %.4f] start_position=[%.4f, %.4f, %.4f]\n"
                "  theta_star=%.4f deg omega=%.6f rad/s contact_time=%.6f s (tau %.6f s)  weights sha256 %s\n"
                "  theta_trig=wrapPi(theta_star - omega*contact_time)=%.4f deg (mod 360)  timeout %.1f s\n"
                "  omega*dt residual (first tick comes dt_=%.4f s after the trigger, not modelled)=%.5f deg",
                rclcpp::Time(msg.header.stamp).seconds(), plan.age_s, p_goal_.x(), p_goal_.y(), p_goal_.z(),
                plan.start_position.x(), plan.start_position.y(), plan.start_position.z(),
                core::radToDeg(plan.theta_star_rad), plan.omega_rad_s, plan.contact_time_s, prodmp_.tau(),
                weights_sha256_.c_str(), core::radToDeg(plan.theta_trig_rad), c.trigger_timeout_s, dt_,
                core::radToDeg(plan.omega_rad_s * dt_));
}

void ProDmpGazeboExecutorNode::onSatelliteOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    namespace si = core::satellite_intercept;
    const auto& c = *cc_;
    const double stamp_s = rclcpp::Time(msg->header.stamp).seconds();  // header.stamp (sim), never the receive time
    if (!satellite_first_sample_seen_) {
        if (msg->header.frame_id != world_frame_) {
            throw std::runtime_error("satellite odometry frame_id '" + msg->header.frame_id +
                                     "' != world_frame '" + world_frame_ + "'");
        }
        const Eigen::Vector3d pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        const double off = (pos - c.center).norm();
        if (off > 0.01) {
            std::ostringstream m;
            m << "satellite odometry origin [" << pos.x() << ", " << pos.y() << ", " << pos.z()
              << "] is " << off * 1000.0 << " mm from satellite_rotation_center (limit 10 mm)";
            throw std::runtime_error(m.str());
        }
        satellite_first_sample_seen_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "satellite odometry: first sample at stamp %.4f s, frame '%s', |origin - center| = %.3f mm",
                    stamp_s, msg->header.frame_id.c_str(), off * 1000.0);
    }
    const auto& q = msg->pose.pose.orientation;
    const si::PhaseSample s = phase_tracker_->update(Eigen::Quaterniond(q.w, q.x, q.y, q.z));
    if (cstate_ != ContinuousState::kRunning && cstate_ != ContinuousState::kFinished) {
        phase_checker_->update(stamp_s, s.theta_rad);  // throws if omega / time axis inconsistent
    }
    onPhaseSample(stamp_s, s.theta_rad, s.R_rel);
}

void ProDmpGazeboExecutorNode::onModelPhaseTick() {
    const auto& c = *cc_;
    const double now_s = this->now().seconds();
    const double theta = c.theta0_model_rad + c.omega_rad_s * now_s;
    onPhaseSample(now_s, theta, Eigen::AngleAxisd(theta, c.axis_unit).toRotationMatrix());
}

void ProDmpGazeboExecutorNode::onPhaseSample(double stamp_s, double theta_rad, const Eigen::Matrix3d& R_rel) {
    namespace si = core::satellite_intercept;
    const auto& c = *cc_;
    const double now_s = this->now().seconds();
    theta_sat_ = theta_rad;
    phase_stamp_s_ = stamp_s;
    phase_latency_s_ = now_s - stamp_s;
    latest_R_rel_ = R_rel;
    have_phase_ = true;
    if (cstate_ != ContinuousState::kWaiting) return;

    if (phase_coherence_pending_) {
        // grasp_command mode, once: the FIRST odometry sample after acceptance. Residual against the phase the
        // planner's snapshot predicts for this stamp; always logged.
        phase_coherence_pending_ = false;
        const double residual = si::phaseResidualRad(theta_rad, snapshot_theta_rad_, snapshot_t_s_,
                                                     command_omega_rad_s_, stamp_s);
        RCLCPP_INFO(this->get_logger(),
                    "phase coherence with the planner snapshot: measured theta %.4f deg at stamp %.4f s, snapshot "
                    "theta %.4f deg at %.4f s, residual %.5f deg (tol %.3f deg)",
                    core::radToDeg(theta_rad), stamp_s, core::radToDeg(snapshot_theta_rad_), snapshot_t_s_,
                    core::radToDeg(residual), core::radToDeg(grasp_command_phase_tol_rad_));
        try {
            si::checkPhaseCoherence(residual, grasp_command_phase_tol_rad_);
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "%s", e.what());
            throw;
        }
    }

    if (now_s - t_wait_start_ > c.trigger_timeout_s) {
        std::ostringstream m;
        m << "continuous: trigger not reached within timeout " << c.trigger_timeout_s << " s (360/|omega| + T_total + margin). "
          << "theta_sat=" << core::radToDeg(theta_rad) << " deg, theta_trig=" << core::radToDeg(trigger_->thetaTrigRad())
          << " deg. No motion was started.";
        RCLCPP_FATAL(this->get_logger(), "%s", m.str().c_str());
        throw std::runtime_error(m.str());
    }
    if (c.measured && !phase_checker_->validated()) return;  // no trigger before omega is verified

    const auto r = trigger_->update(theta_rad);
    trigger_e_rad_ = r.e_rad;
    if (!wait_expected_logged_) {
        wait_expected_logged_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "continuous: first phase sample consumed by the trigger: theta_sat=%.4f deg, e=%.4f deg, "
                    "expected wait %.1f s%s (phase-consistency error %.4f deg over >= %.1f s)",
                    core::radToDeg(theta_rad), core::radToDeg(r.e_rad), trigger_->expectedWaitS(theta_rad),
                    r.first_sample_past ? " [already past the threshold: waiting for the NEXT lap]" : "",
                    c.measured ? phase_checker_->lastErrorDeg() : 0.0, c.consistency_window_s);
    }
    if (r.fired) {
        theta_at_trigger_ = theta_rad;
        t_trigger_ = now_s;
        startContinuousRollout(theta_rad);
    }
}

void ProDmpGazeboExecutorNode::startContinuousRollout(double theta_at_trigger_rad) {
    namespace si = core::satellite_intercept;
    Eigen::Vector3d ee;
    if (!lookupEeNonBlocking(ee)) {
        const std::string m = "continuous: TF " + world_frame_ + " -> " + ee_frame_ +
                              " unavailable at the trigger; no motion started";
        RCLCPP_FATAL(this->get_logger(), "%s", m.c_str());
        throw std::runtime_error(m);
    }
    const double dev = (ee - ee_plan_).norm();
    if (dev > 0.005) {
        std::ostringstream m;
        m << "continuous: ee_start deviates " << dev * 1000.0 << " mm from ee_plan (limit 5 mm); no motion started";
        RCLCPP_FATAL(this->get_logger(), "%s", m.str().c_str());
        throw std::runtime_error(m.str());
    }
    ee_start_ = ee;
    if (grasp_command_mode_) {
        // The planner validated a rollout that starts at its start_position (FK at q0).
        try {
            si::checkStartPositionMatches(ee_start_, p_start_cmd_);
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "continuous: at the trigger: %s; no motion started", e.what());
            throw;
        }
    }

    // Same call chain as the frozen branch: relative goal, anchor init at the real EE pose, goal in world.
    prodmp_.setRelativeGoal(true);
    prodmp_.setInitialConditions(/*init_time=*/0.0, ee_start_, Eigen::Vector3d::Zero());
    prodmp_.setGoal(p_goal_);
    qdmp_.reset();  // orientation: demo-native, not rotated (as in frozen)

    rollout_clock_ = std::make_unique<si::MeasuredTimeBase>(dt_, prodmp_.tau());
    elapsed_ = 0.0;
    cstate_ = ContinuousState::kRunning;
    const Eigen::Vector3d rel_goal = p_goal_ - ee_start_;
    RCLCPP_INFO(this->get_logger(),
                "continuous TRIGGER at node clock t=%.4f s: theta_sat=%.4f deg (theta_trig %.4f deg, e=%.5f deg, "
                "phase-sample latency %.4f s)\n"
                "  ee_start=[%.4f, %.4f, %.4f] (|ee_start-ee_plan|=%.3f mm)  p_goal=[%.4f, %.4f, %.4f]  "
                "relative goal=[%.4f, %.4f, %.4f]\n"
                "  starting rollout: time base = node clock (elapsed = now - t_roll0), dt_nominal=%.4f s, tau=%.4f s, "
                "tick_count=0",
                t_trigger_, core::radToDeg(theta_at_trigger_rad), core::radToDeg(trigger_->thetaTrigRad()),
                core::radToDeg(trigger_e_rad_), phase_latency_s_, ee_start_.x(), ee_start_.y(), ee_start_.z(), dev * 1000.0,
                p_goal_.x(), p_goal_.y(), p_goal_.z(), rel_goal.x(), rel_goal.y(), rel_goal.z(), dt_, prodmp_.tau());

    step_timer_ = rclcpp::create_timer(
        this, this->get_clock(), rclcpp::Duration::from_seconds(dt_),
        std::bind(&ProDmpGazeboExecutorNode::stepCallback, this));
}

void ProDmpGazeboExecutorNode::finishContinuousRollout(const Eigen::Vector3d& commanded_after) {
    namespace si = core::satellite_intercept;
    const auto& c = *cc_;
    const double now_s = this->now().seconds();
    t_at_end_ = now_s;
    cmd_after_ = commanded_after;
    at_end_recorded_ = true;
    duration_sim_ = now_s - rollout_clock_->t0();
    timing_ok_ = std::fabs(duration_sim_ - prodmp_.tau()) <= c.rollout_time_tol_s;

    // Phase at at_end: model = analytic at the node clock; measured = latest odometry sample.
    double theta_end = theta_sat_;
    Eigen::Matrix3d R_end = latest_R_rel_;
    if (!c.measured) {
        theta_end = c.theta0_model_rad + c.omega_rad_s * now_s;
        R_end = Eigen::AngleAxisd(theta_end, c.axis_unit).toRotationMatrix();
    }
    dtheta_at_end_rad_ = core::wrapPi(theta_end - c.theta_int_rad);
    const Eigen::Vector3d p_handle = c.center + R_end * (p0_ - c.center);
    Eigen::Vector3d ee;
    const bool ee_ok = lookupEeNonBlocking(ee);
    const std::string handle_ee = ee_ok ? (fmtG((p_handle - ee).norm() * 1000.0) + " mm") : std::string("n/a (TF unavailable)");
    const double cmd_jump_mm = have_last_cmd_ ? (cmd_after_ - last_cmd_pos_).norm() * 1000.0 : std::nan("");

    const double mean_p = rollout_clock_->meanPeriod();
    RCLCPP_INFO(this->get_logger(),
                "continuous AT_END at node clock t=%.4f s: duration_sim=%.5f s (tau=%.5f, diff %.5f s, tol %.3f s) timing_ok=%s\n"
                "  T_total observed (trigger -> at_end) = %.5f s (parameter %.4f s)\n"
                "  theta_sat(t_at_end)=%.4f deg (phase sample stamp %.4f, age %.4f s)  theta_int=%.4f deg  dtheta=%.4f deg\n"
                "  |p_handle - p_goal| = %.3f mm  |p_handle - ee| = %s\n"
                "  ticks: count=%llu mean period=%.6f s max period=%.6f s (dt_nominal %.6f) skipped(>1.5 dt)=%llu\n"
                "  commanded pose before/after the at_end transition differs by %.4f mm",
                now_s, duration_sim_, prodmp_.tau(), duration_sim_ - prodmp_.tau(), c.rollout_time_tol_s,
                timing_ok_ ? "true" : "false", t_at_end_ - t_trigger_, c.contact_time_s,
                core::radToDeg(theta_end), phase_stamp_s_, now_s - phase_stamp_s_, core::radToDeg(c.theta_int_rad),
                core::radToDeg(dtheta_at_end_rad_), (p_handle - p_goal_).norm() * 1000.0, handle_ee.c_str(),
                static_cast<unsigned long long>(rollout_clock_->tickCount()), mean_p, rollout_clock_->maxPeriod(), dt_,
                static_cast<unsigned long long>(rollout_clock_->skippedTicks()), cmd_jump_mm);
    if (!timing_ok_) {
        RCLCPP_ERROR(this->get_logger(),
                     "continuous: rollout duration in sim time %.5f s differs from tau %.5f s by %.5f s (> rollout_time_tol_s %.3f s); "
                     "timing_ok=false in ~/continuous_status (rollout NOT interrupted).",
                     duration_sim_, prodmp_.tau(), duration_sim_ - prodmp_.tau(), c.rollout_time_tol_s);
    }
    if (mean_p > 0.0 && std::fabs(mean_p - dt_) / dt_ > 0.20) {
        RCLCPP_WARN(this->get_logger(),
                    "continuous: mean tick period %.6f s (sim) deviates %.1f%% from dt_=%.6f s: the executor timer "
                    "does not tick at dt_ in sim time; elapsed is MEASURED on the clock, so the rollout duration is unaffected.",
                    mean_p, 100.0 * std::fabs(mean_p - dt_) / dt_, dt_);
    }
    cstate_ = ContinuousState::kFinished;
}

void ProDmpGazeboExecutorNode::publishContinuousStatus() {
    namespace si = core::satellite_intercept;
    const auto& c = *cc_;
    const double now_s = this->now().seconds();
    const double nan = std::nan("");

    if (cstate_ == ContinuousState::kWaitingCommand &&
        now_s - t_cmd_wait_start_ > grasp_command_timeout_s_) {
        const std::string m = "continuous: no GraspCommand received on '" + grasp_command_topic_ + "' within " +
                              fmtG(grasp_command_timeout_s_) + " s. No motion was started.";
        RCLCPP_FATAL(this->get_logger(), "%s", m.c_str());
        throw std::runtime_error(m);
    }
    if (cstate_ == ContinuousState::kWaiting) {
        if (now_s - t_wait_start_ > c.trigger_timeout_s) {
            const std::string m = "continuous: trigger not reached within timeout " + fmtG(c.trigger_timeout_s) +
                                  " s (no phase sample or crossing missed). No motion was started.";
            RCLCPP_FATAL(this->get_logger(), "%s", m.c_str());
            throw std::runtime_error(m);
        }
        if (now_s - last_wait_log_ >= 10.0) {
            last_wait_log_ = now_s;
            const double e = have_phase_ ? core::wrapPi(theta_sat_ - trigger_->thetaTrigRad()) : nan;
            RCLCPP_INFO(this->get_logger(), "continuous: waiting, theta_sat=%.4f deg e=%.4f deg (armed=%s)",
                        have_phase_ ? core::radToDeg(theta_sat_) : nan, core::radToDeg(e), trigger_->armed() ? "yes" : "no");
        }
    }
    if (cstate_ == ContinuousState::kFinished && now_s >= t_at_end_ + 20.0) {
        if (continuous_status_timer_) continuous_status_timer_->cancel();
        return;
    }

    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "continuous_status";
    st.hardware_id = "prodmp_gazebo_executor_node";
    const char* state_name = cstate_ == ContinuousState::kWaitingCommand ? "waiting_command"
                             : cstate_ == ContinuousState::kWaiting ? "waiting"
                             : cstate_ == ContinuousState::kRunning ? "running"
                             : cstate_ == ContinuousState::kFinished ? "finished" : "idle";
    st.message = state_name;
    st.level = (cstate_ == ContinuousState::kFinished && !timing_ok_) ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                                                                      : diagnostic_msgs::msg::DiagnosticStatus::OK;
    auto kv = [&st](const std::string& k, const std::string& v) {
        diagnostic_msgs::msg::KeyValue e;
        e.key = k;
        e.value = v;
        st.values.push_back(e);
    };
    auto kvd = [&kv](const std::string& k, double v) { kv(k, fmtG(v)); };
    auto kv3 = [&kvd](const std::string& k, const Eigen::Vector3d& v) {
        kvd(k + "_x", v.x());
        kvd(k + "_y", v.y());
        kvd(k + "_z", v.z());
    };

    const double theta_now = have_phase_ ? theta_sat_ : nan;
    kv("state", state_name);
    if (grasp_command_mode_) kv("goal_source", "grasp_command");
    kvd("theta_sat_deg", core::radToDeg(theta_now));
    // trigger_ does not exist yet in grasp_command mode while waiting for the message.
    kvd("theta_trig_deg", trigger_ ? core::radToDeg(trigger_->thetaTrigRad()) : nan);
    kvd("e_deg", (have_phase_ && trigger_) ? core::radToDeg(core::wrapPi(theta_sat_ - trigger_->thetaTrigRad())) : nan);
    kvd("omega_deg_s", c.omega_deg_s);
    kvd("theta_int_deg", core::radToDeg(c.theta_int_rad));
    kvd("T_total_s", c.contact_time_s);
    kv("phase_source", c.measured ? "measured" : "model");
    kvd("phase_latency_s", have_phase_ ? phase_latency_s_ : nan);
    kvd("phase_consistency_error_deg", c.measured ? phase_checker_->lastErrorDeg() : nan);
    kv("phase_consistency_validated", (!c.measured || phase_checker_->validated()) ? "true" : "false");
    const bool ran = rollout_clock_ && rollout_clock_->tickCount() > 0;
    const bool trig = cstate_ == ContinuousState::kRunning || cstate_ == ContinuousState::kFinished;
    kvd("t_trigger_sim", trig ? t_trigger_ : nan);
    kvd("t_roll0", ran ? rollout_clock_->t0() : nan);
    kvd("t_at_end_sim", at_end_recorded_ ? t_at_end_ : nan);
    kvd("duration_sim", at_end_recorded_ ? duration_sim_ : nan);
    kv("timing_ok", timing_ok_ ? "true" : "false");
    kvd("tick_count", rollout_clock_ ? static_cast<double>(rollout_clock_->tickCount()) : 0.0);
    kvd("ticks_skipped", rollout_clock_ ? static_cast<double>(rollout_clock_->skippedTicks()) : 0.0);
    kvd("tick_period_mean_s", rollout_clock_ ? rollout_clock_->meanPeriod() : nan);
    kvd("tick_period_max_s", rollout_clock_ ? rollout_clock_->maxPeriod() : nan);
    kvd("elapsed_s", rollout_clock_ ? elapsed_ : nan);
    kvd("tau_s", prodmp_.tau());
    kv3("p0", p0_);
    kv3("p_goal", p_goal_);

    Eigen::Vector3d p_handle = Eigen::Vector3d::Constant(nan);
    if (have_phase_) p_handle = c.center + latest_R_rel_ * (p0_ - c.center);
    kv3("p_handle", p_handle);
    Eigen::Vector3d ee = Eigen::Vector3d::Constant(nan);
    const bool ee_ok = lookupEeNonBlocking(ee);
    if (!ee_ok) ee = Eigen::Vector3d::Constant(nan);
    kv("ee_valid", ee_ok ? "true" : "false");
    kv3("ee", ee);
    const Eigen::Vector3d miss = p_handle - ee;
    kv3("miss", miss);
    kvd("miss_norm", miss.norm());
    kvd("dtheta_at_end_deg", at_end_recorded_ ? core::radToDeg(dtheta_at_end_rad_) : nan);
    kv3("cmd_before_at_end", have_last_cmd_ ? last_cmd_pos_ : Eigen::Vector3d::Constant(nan));
    kv3("cmd_after_at_end", at_end_recorded_ ? cmd_after_ : Eigen::Vector3d::Constant(nan));
    kvd("cmd_jump_at_end_mm",
        (at_end_recorded_ && have_last_cmd_) ? (cmd_after_ - last_cmd_pos_).norm() * 1000.0 : nan);

    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = this->now();
    arr.status.push_back(st);
    continuous_status_pub_->publish(arr);
}

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning
