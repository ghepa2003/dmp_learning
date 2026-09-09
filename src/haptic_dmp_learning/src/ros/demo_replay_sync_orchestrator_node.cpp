#include "haptic_dmp_learning/ros/demo_replay_sync_orchestrator_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "haptic_dmp_learning/core/gripper_ramp.hpp"

#include <sys/types.h>
#include <sys/wait.h>
#include <sys/select.h>
#include <termios.h>
#include <signal.h>
#include <unistd.h>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/empty.hpp>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <stdexcept>

namespace haptic_dmp_learning {
namespace ros_wrapper {

namespace {
constexpr auto kTickPeriod = std::chrono::milliseconds(100);
// Gap between the seed [0,0] Joy and the [1,0] start edge on the synced topic,
// so the downstream recorder's "first message seeds prior state" rule does not
// swallow the edge (mirrors csv_master_pose_player_node::kStartEdgeDelaySec).
constexpr double kStartEdgeGapSec = 0.2;

std::string numArg(const std::string& key, double v) {
    return key + ":=" + std::to_string(v);
}
double secElapsed(std::chrono::steady_clock::time_point since) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - since).count();
}
}  // namespace

DemoReplaySyncOrchestratorNode::DemoReplaySyncOrchestratorNode()
    : Node("demo_replay_sync_orchestrator") {

    // 1. mode (mandatory, fail-loud) -----------------------------------------
    std::string mode_str;
    try {
        mode_str = this->declare_parameter<std::string>("mode");
    } catch (const std::exception& e) {
        throw std::runtime_error(
            "demo_replay_sync_orchestrator: required parameter 'mode' "
            "('demo' | 'replay') was not provided. Underlying error: " +
            std::string(e.what()));
    }
    if (mode_str == "demo") {
        mode_ = Mode::kDemo;
    } else if (mode_str == "replay") {
        mode_ = Mode::kReplay;
    } else {
        throw std::runtime_error(
            "demo_replay_sync_orchestrator: 'mode' must be 'demo' or 'replay', got '" +
            mode_str + "'.");
    }

    // 2. run_id: generated in demo mode, mandatory input in replay mode ------
    const std::string run_id_param = this->declare_parameter<std::string>("run_id", "");
    if (mode_ == Mode::kReplay) {
        if (run_id_param.empty()) {
            throw std::runtime_error(
                "demo_replay_sync_orchestrator: in replay mode 'run_id' is mandatory "
                "- it identifies the demonstration whose DMP weights and force "
                "calibration are replayed. It is NOT generated in this mode.");
        }
        run_id_ = run_id_param;
    } else {
        run_id_ = run_id_param.empty() ? generateRunId() : run_id_param;
    }

    // 3. Parameters (documented defaults) -----------------------------------
    clock_wait_timeout_sec_ = this->declare_parameter<double>("clock_wait_timeout_sec", 10.0);
    odom_wait_timeout_sec_ = this->declare_parameter<double>("odom_wait_timeout_sec", 30.0);
    sync_delay_sec_ = this->declare_parameter<double>("sync_delay_sec", 5.0);
    target_name_ = this->declare_parameter<std::string>("target_name", "free_target_object");
    tx_ = this->declare_parameter<double>("target_x", 0.6);
    ty_ = this->declare_parameter<double>("target_y", 0.0);
    tz_ = this->declare_parameter<double>("target_z", 0.5);
    tvx_ = this->declare_parameter<double>("target_vx", 0.0);
    tvy_ = this->declare_parameter<double>("target_vy", 0.0);
    tvz_ = this->declare_parameter<double>("target_vz", 0.0);
    twx_ = this->declare_parameter<double>("target_wx", 0.0);
    twy_ = this->declare_parameter<double>("target_wy", 0.0);
    twz_ = this->declare_parameter<double>("target_wz", 0.0);
    tsx_ = this->declare_parameter<double>("target_size_x", 0.045);
    tsy_ = this->declare_parameter<double>("target_size_y", 0.045);
    tsz_ = this->declare_parameter<double>("target_size_z", 0.045);
    tmass_ = this->declare_parameter<double>("target_mass", 5.0);
    buttons_topic_ = this->declare_parameter<std::string>("buttons_topic", "/touch0/buttons");
    buttons_synced_topic_ = this->declare_parameter<std::string>(
        "buttons_synced_topic", "/touch0/buttons_synced");
    use_csv_playback_ = this->declare_parameter<bool>("use_csv_playback", true);
    // Gripper close ramp (see core/gripper_ramp.hpp). Defaults reproduce the
    // previous single-step behaviour's endpoints: 0.06 = open, 0.0 = closed.
    gripper_open_position_ = this->declare_parameter<double>("gripper_open_position", 0.06);
    gripper_closed_position_ = this->declare_parameter<double>("gripper_closed_position", 0.0);
    gripper_close_ramp_duration_sec_ =
        this->declare_parameter<double>("gripper_close_ramp_duration_sec", 2.0);
    // Forwarded to grasp_state_machine in replay mode; mandatory there (absolute
    // safety limit, no silent default), unused in demo mode.
    hard_force_limit_n_ = this->declare_parameter<double>("hard_force_limit_n", -1.0);
    if (mode_ == Mode::kReplay && !(hard_force_limit_n_ > 0.0)) {
        throw std::runtime_error(
            "demo_replay_sync_orchestrator: in replay mode 'hard_force_limit_n' (> 0) "
            "is mandatory - it is forwarded to grasp_state_machine as the absolute "
            "|F| safety limit and must be set explicitly.");
    }
    // Forwarded to dmp_gazebo_executor_node in replay mode (see kLaunchReplay).
    target_odom_required_ = this->declare_pamemory_user_editsrameter<bool>("target_odom_required", false);
    target_odom_topic_ = this->declare_parameter<std::string>(
        "target_odom_topic", "/free_target_object/odometry");
    target_odom_timeout_sec_ = this->declare_parameter<double>("target_odom_timeout_sec", 5.0);

    buttons_synced_pub_ = this->create_publisher<sensor_msgs::msg::Joy>(
        buttons_synced_topic_, rclcpp::QoS(10));

    gripper_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/gripper_position_cmd", rclcpp::QoS(10));

    // One-shot announce that the gripper close ramp has finished (see header).
    gripper_close_complete_pub_ = this->create_publisher<std_msgs::msg::Empty>(
        "/gripper_close_complete", rclcpp::QoS(10));

    // One-shot wall-timer (1.0 s) to publish the initial gripper position
    // (gripper_open_position_), allowing the ROS2 <-> Ignition bridge time to
    // subscribe before the message.
    gripper_init_timer_ = this->create_wall_timer(
        std::chrono::seconds(1), [this]() {
            gripper_init_timer_->cancel();
            std_msgs::msg::Float64 cmd;
            cmd.data = gripper_open_position_;
            gripper_pub_->publish(cmd);
            RCLCPP_INFO(this->get_logger(),
                        "Initial gripper open command (%.3f) published on /gripper_position_cmd",
                        gripper_open_position_);
        });

    // Non-blocking keyboard reading setup via termios
    if (isatty(STDIN_FILENO)) {
        if (tcgetattr(STDIN_FILENO, &orig_termios_) == 0) {
            struct termios raw = orig_termios_;
            raw.c_lflag &= ~(ICANON | ECHO);
            raw.c_cc[VMIN] = 0;
            raw.c_cc[VTIME] = 0;
            if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
                termios_modified_ = true;
            }
        }
    }

    keyboard_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&DemoReplaySyncOrchestratorNode::checkKeyboard, this));

    // 4. /clock presence check subscription --------------------------------
    clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::QoS(10),
        std::bind(&DemoReplaySyncOrchestratorNode::clockCallback, this, std::placeholders::_1));

    node_start_ = std::chrono::steady_clock::now();
    phase_entered_ = node_start_;

    RCLCPP_INFO(this->get_logger(),
                "==== demo_replay_sync_orchestrator ====\n"
                "  mode                 = %s\n"
                "  run_id               = %s   (%s)\n"
                "  sync_delay_sec       = %.2f (anchored to target odometry sim-time t0)\n"
                "  target               = %s @ (%.3f, %.3f, %.3f)  v=(%.3f, %.3f, %.3f)  w=(%.3f, %.3f, %.3f)\n"
                "  target box size      = (%.3f, %.3f, %.3f) m   mass = %.3f kg\n"
                "  clock_wait_timeout   = %.1f s (wall clock)\n"
                "  buttons: %s -> %s",
                (mode_ == Mode::kDemo ? "demo" : "replay"),
                run_id_.c_str(),
                (mode_ == Mode::kReplay ? "provided" : "generated"),
                sync_delay_sec_, target_name_.c_str(),
                tx_, ty_, tz_, tvx_, tvy_, tvz_, twx_, twy_, twz_,
                tsx_, tsy_, tsz_, tmass_,
                clock_wait_timeout_sec_,
                buttons_topic_.c_str(), buttons_synced_topic_.c_str());

    // 5. Orchestration timer. Intentionally create_wall_timer / steady_clock:
    //    it must run BEFORE /clock is confirmed and it drives the wall-clock
    //    timeout of the /clock presence check itself.
    tick_timer_ = this->create_wall_timer(
        kTickPeriod, std::bind(&DemoReplaySyncOrchestratorNode::tick, this));
}

DemoReplaySyncOrchestratorNode::~DemoReplaySyncOrchestratorNode() {
    if (termios_modified_) {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
        termios_modified_ = false;
    }
    stopChildren();
}

// ---------------------------------------------------------------------------
void DemoReplaySyncOrchestratorNode::clockCallback(
    const rosgraph_msgs::msg::Clock::SharedPtr msg) {
    last_clock_sec_ =
        static_cast<double>(msg->clock.sec) + static_cast<double>(msg->clock.nanosec) * 1e-9;
    clock_seen_ = true;
}

void DemoReplaySyncOrchestratorNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (odom_seen_) return;
    t0_sim_ = rclcpp::Time(msg->header.stamp).seconds();
    odom_seen_ = true;
    RCLCPP_INFO(this->get_logger(),
                "target '%s' first odometry: sim-time t0 = %.4f s (frame_id='%s'); "
                "will trigger at t0 + %.2f s",
                target_name_.c_str(), t0_sim_, msg->header.frame_id.c_str(),
                sync_delay_sec_);
}

void DemoReplaySyncOrchestratorNode::buttonsCallback(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
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
    const bool rising0 = (msg->buttons[0] != 0) && (prev_buttons_[0] == 0);
    const bool rising1 = (msg->buttons[1] != 0) && (prev_buttons_[1] == 0);
    prev_buttons_.assign(msg->buttons.begin(), msg->buttons.end());

    if (rising0 && !recording_ && phase_ == Phase::kWaitButtonStart) {
        recording_ = true;
        gripper_trigger_sent_ = false;
        RCLCPP_INFO(this->get_logger(),
                    "start button: spawning target and starting the %0.2f s sync window",
                    sync_delay_sec_);
        phase_ = Phase::kSpawnTarget;
        phase_entered_ = std::chrono::steady_clock::now();
    } else if (rising1 && recording_) {
        // Stop: republish immediately on the synced topic, no delay.
        if (phase_ == Phase::kRunning || phase_ == Phase::kDone) {
            publishSyncedButtons(0, 1);
            RCLCPP_INFO(this->get_logger(),
                        "stop button: republished [0,1] on %s (immediate)",
                        buttons_synced_topic_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(),
                        "stop button received before the synced start was emitted "
                        "(phase not RUNNING); forwarding [0,1] anyway - the demo was "
                        "aborted mid-sync.");
            publishSyncedButtons(0, 1);
        }
        recording_ = false;
        phase_ = Phase::kDone;
    }
}

// ---------------------------------------------------------------------------
void DemoReplaySyncOrchestratorNode::tick() {
    switch (phase_) {
        case Phase::kCheckClock:
            if (clock_seen_) {
                RCLCPP_INFO(this->get_logger(), "/clock is live (t=%.3f s).", last_clock_sec_);
                phase_ = (mode_ == Mode::kDemo) ? Phase::kLaunchRecording : Phase::kSpawnTarget;
                phase_entered_ = std::chrono::steady_clock::now();
            } else if (secElapsed(node_start_) > clock_wait_timeout_sec_) {
                fatal("no message received on /clock within " +
                      std::to_string(clock_wait_timeout_sec_) + " s. /clock is NOT "
                      "bridged by any file in this workspace - it must be provided by "
                      "an external component (e.g. gz_ros2_control in the Franka Gazebo "
                      "bringup). Bring up Gazebo with sim time before this node.");
            }
            break;

        case Phase::kLaunchRecording: {  // demo only
            std::vector<std::string> argv = {
                "ros2", "launch", "haptic_dmp_learning", "demo_replay_sync.launch.py",
                "run_id:=" + run_id_,
                std::string("use_csv_playback:=") + (use_csv_playback_ ? "true" : "false")};
            launchChild(argv, "recording_launch");
            buttons_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
                buttons_topic_, rclcpp::QoS(10),
                std::bind(&DemoReplaySyncOrchestratorNode::buttonsCallback, this,
                          std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(),
                        "recording stack launched; waiting for start button on %s",
                        buttons_topic_.c_str());
            phase_ = Phase::kWaitButtonStart;
            phase_entered_ = std::chrono::steady_clock::now();
            break;
        }

        case Phase::kWaitButtonStart:  // demo only; driven by buttonsCallback
            break;

        case Phase::kSpawnTarget: {
            std::vector<std::string> argv = {
                "ros2", "launch", "free_target_object", "free_target_object.launch.py",
                "name:=" + target_name_,
                numArg("x", tx_), numArg("y", ty_), numArg("z", tz_),
                numArg("vx", tvx_), numArg("vy", tvy_), numArg("vz", tvz_),
                numArg("wx", twx_), numArg("wy", twy_), numArg("wz", twz_),
                numArg("size_x", tsx_), numArg("size_y", tsy_), numArg("size_z", tsz_),
                numArg("mass", tmass_)};
            launchChild(argv, "free_target_object");
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/" + target_name_ + "/odometry", rclcpp::QoS(10),
                std::bind(&DemoReplaySyncOrchestratorNode::odomCallback, this,
                          std::placeholders::_1));
            phase_ = Phase::kWaitOdom;
            phase_entered_ = std::chrono::steady_clock::now();
            break;
        }

        case Phase::kWaitOdom:
            if (odom_seen_) {
                phase_ = Phase::kWaitSync;
                phase_entered_ = std::chrono::steady_clock::now();
            } else if (secElapsed(phase_entered_) > odom_wait_timeout_sec_) {
                fatal("no message on /" + target_name_ + "/odometry within " +
                      std::to_string(odom_wait_timeout_sec_) +
                      " s after spawning the target.");
            }
            break;

        case Phase::kWaitSync:
            if (clock_seen_ && (last_clock_sec_ - t0_sim_) >= sync_delay_sec_) {
                RCLCPP_INFO(this->get_logger(),
                            "sync reached: sim-time %.4f s = t0 (%.4f) + %.2f s",
                            last_clock_sec_, t0_sim_, sync_delay_sec_);
                if (mode_ == Mode::kDemo) {
                    phase_ = Phase::kEmitStartSeed;
                } else {
                    phase_ = Phase::kLaunchReplay;
                }
                phase_entered_ = std::chrono::steady_clock::now();
            }
            break;

        case Phase::kEmitStartSeed:  // demo only
            publishSyncedButtons(0, 0);
            phase_ = Phase::kEmitStartEdge;
            phase_entered_ = std::chrono::steady_clock::now();
            break;

        case Phase::kEmitStartEdge:  // demo only
            if (secElapsed(phase_entered_) >= kStartEdgeGapSec) {
                publishSyncedButtons(1, 0);
                RCLCPP_INFO(this->get_logger(),
                            "synced start edge [1,0] emitted on %s; recorder is live.",
                            buttons_synced_topic_.c_str());
                phase_ = Phase::kRunning;
                phase_entered_ = std::chrono::steady_clock::now();
            }
            break;

        case Phase::kLaunchReplay: {  // replay only
            launchChild({"ros2", "run", "haptic_dmp_learning", "dmp_gazebo_executor_node",
                         "--ros-args",
                         "--params-file", hapticDmpParamsPath(),
                         "-p", std::string("target_odom_required:=") +
                                   (target_odom_required_ ? "true" : "false"),
                         "-p", "target_odom_topic:=" + target_odom_topic_,
                         "-p", "target_odom_timeout_sec:=" +
                                   std::to_string(target_odom_timeout_sec_),
                         "-p", "use_sim_time:=true",
                         "-p", "startup_delay_sec:=0.0",
                         "-p", "weights_yaml_path:=" + weightsPathForRunId(),
                         "-p", "demo_csv_path:=" + demoCsvPathForRunId()},
                        "dmp_executor");
            launchChild({"ros2", "run", "grasp_monitoring", "geometric_grasp_monitor",
                         "--ros-args",
                         "--params-file", graspMonitorParamsPath(),
                         "-p", "use_sim_time:=true"},
                        "geometric_grasp_monitor");
            launchChild({"ros2", "run", "haptic_dmp_learning", "grasp_force_calibration_node",
                         "--ros-args",
                         "--params-file", hapticDmpParamsPath(),
                         "-p", "use_sim_time:=true",
                         "-p", "mode:=verify",
                         "-p", "run_id:=" + run_id_},
                        "grasp_force_calibration(verify)");
            launchChild({"ros2", "run", "haptic_dmp_learning", "grasp_state_machine_node",
                         "--ros-args",
                         "-p", "use_sim_time:=true",
                         "-p", numArg("hard_force_limit_n", hard_force_limit_n_)},
                        "grasp_state_machine_node");
            RCLCPP_INFO(this->get_logger(),
                        "replay stack launched for run_id '%s' (weights: %s).",
                        run_id_.c_str(), weightsPathForRunId().c_str());
            phase_ = Phase::kRunning;
            phase_entered_ = std::chrono::steady_clock::now();
            break;
        }

        case Phase::kRunning:
            RCLCPP_INFO_ONCE(this->get_logger(),
                             "orchestration complete for run_id '%s'; child processes "
                             "are running. Stop this node (Ctrl-C) when finished - it "
                             "will terminate the children it launched.",
                             run_id_.c_str());
            break;

        case Phase::kDone:
        case Phase::kError:
            break;
    }
}

void DemoReplaySyncOrchestratorNode::fatal(const std::string& why) {
    if (failed_) return;
    failed_ = true;
    phase_ = Phase::kError;
    RCLCPP_FATAL(this->get_logger(), "demo_replay_sync_orchestrator ABORTED: %s", why.c_str());
    if (termios_modified_) {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
        termios_modified_ = false;
    }
    stopChildren();
    rclcpp::shutdown();
}

// ---------------------------------------------------------------------------
std::string DemoReplaySyncOrchestratorNode::generateRunId() const {
    std::time_t t = std::time(nullptr);
    std::tm tm_buf{};
    localtime_r(&t, &tm_buf);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%dT%H%M%S", &tm_buf);
    return std::string(buf);
}

std::string DemoReplaySyncOrchestratorNode::weightsPathForRunId() const {
    const char* home = std::getenv("HOME");
    const std::string ws_root = std::string(home ? home : "/root") + "/thesis_ws";
    return ws_root + "/dmp_weights_" + run_id_ + ".yaml";
}

std::string DemoReplaySyncOrchestratorNode::demoCsvPathForRunId() const {
    const char* home = std::getenv("HOME");
    const std::string ws_root = std::string(home ? home : "/root") + "/thesis_ws";
    return ws_root + "/demo_raw_" + run_id_ + ".csv";
}

std::string DemoReplaySyncOrchestratorNode::graspMonitorParamsPath() const {
    // grasp_monitoring/config/params.yaml carries geometric_grasp_monitor's
    // parameters (enable_alignment_check, ...). Passed with --params-file so the
    // node does not silently run on its hardcoded defaults. Resolve the installed
    // share copy, falling back to the source tree (same idiom as the feature
    // flags path in live_demo_recorder_node / haptic_dmp_wrapper_node).
    try {
        return ament_index_cpp::get_package_share_directory("grasp_monitoring") +
               "/config/params.yaml";
    } catch (const std::exception&) {
        const char* home = std::getenv("HOME");
        const std::string ws_root = std::string(home ? home : "/root") + "/thesis_ws";
        return ws_root + "/src/grasp_monitoring/config/params.yaml";
    }
}

std::string DemoReplaySyncOrchestratorNode::hapticDmpParamsPath() const {
    // haptic_dmp_learning/config/params.yaml is a single file with one section
    // per node name (dmp_gazebo_executor_node, demo_replay_sync_orchestrator,
    // ...); a node only reads its own section, so the same file is passed to
    // every haptic_dmp_learning child. Same resolve-share-then-source idiom as
    // graspMonitorParamsPath().
    try {
        return ament_index_cpp::get_package_share_directory("haptic_dmp_learning") +
               "/config/params.yaml";
    } catch (const std::exception&) {
        const char* home = std::getenv("HOME");
        const std::string ws_root = std::string(home ? home : "/root") + "/thesis_ws";
        return ws_root + "/src/haptic_dmp_learning/config/params.yaml";
    }
}

void DemoReplaySyncOrchestratorNode::publishSyncedButtons(int b0, int b1) {
    sensor_msgs::msg::Joy msg;
    msg.header.stamp = this->now();
    msg.buttons = {static_cast<int32_t>(b0), static_cast<int32_t>(b1)};
    buttons_synced_pub_->publish(msg);
}

void DemoReplaySyncOrchestratorNode::publishGripperTrigger() {
    sensor_msgs::msg::Joy msg;
    msg.header.stamp = this->now();
    msg.buttons = {0, 0, 1};
    buttons_synced_pub_->publish(msg);
}

void DemoReplaySyncOrchestratorNode::checkKeyboard() {
    if (!termios_modified_) return;

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    struct timeval tv{0, 0};
    int ret = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &tv);
    if (ret > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
        char c = 0;
        if (read(STDIN_FILENO, &c, 1) == 1) {
            if (c == ' ' && phase_ == Phase::kRunning && !gripper_trigger_sent_) {
                // 1. Mark the instant of intent on the synced buttons topic.
                publishGripperTrigger();
                // 2. Lock out repeat presses immediately.
                gripper_trigger_sent_ = true;
                // 3. Close along a timed ramp instead of one step command (a
                //    step slams the fingers shut and ejects the object). A
                //    dedicated 20 ms wall timer (same period as keyboard_timer_)
                //    walks core::gripper_ramp::interpolateGripperPosition from
                //    open to closed, then cancels itself.
                gripper_ramp_start_ = std::chrono::steady_clock::now();
                gripper_ramp_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(20),
                    std::bind(&DemoReplaySyncOrchestratorNode::gripperRampTick, this));
                RCLCPP_INFO(this->get_logger(),
                            "SPACE pressed: starting %.1fs gripper close ramp and trigger "
                            "Joy [0,0,1] on %s",
                            gripper_close_ramp_duration_sec_, buttons_synced_topic_.c_str());
            }
        }
    }
}

void DemoReplaySyncOrchestratorNode::gripperRampTick() {
    const double elapsed = secElapsed(gripper_ramp_start_);
    const bool done = elapsed >= gripper_close_ramp_duration_sec_;

    std_msgs::msg::Float64 cmd;
    // interpolateGripperPosition already clamps to the closed value once elapsed
    // reaches the duration; publish that exact value on the final tick so timer
    // drift cannot leave the gripper a hair open.
    cmd.data = done ? gripper_closed_position_
                    : core::gripper_ramp::interpolateGripperPosition(
                          elapsed, gripper_close_ramp_duration_sec_,
                          gripper_open_position_, gripper_closed_position_);
    gripper_pub_->publish(cmd);

    if (done) {
        gripper_ramp_timer_->cancel();
        gripper_ramp_timer_.reset();
        // Physical command first (above), then announce the ramp is complete so
        // grasp_force_calibration_node can start its force capture on real contact.
        gripper_close_complete_pub_->publish(std_msgs::msg::Empty());
        RCLCPP_INFO(this->get_logger(),
                    "gripper close ramp complete (%.3f).", gripper_closed_position_);
    }
}

pid_t DemoReplaySyncOrchestratorNode::launchChild(
    const std::vector<std::string>& argv, const std::string& tag) {
    // Build the argv array and the log line BEFORE fork(): between fork() and
    // execvp() the child may only call async-signal-safe functions (no malloc),
    // and rclcpp keeps background threads alive so a post-fork allocation could
    // deadlock on the malloc lock.
    std::vector<char*> cargv;
    cargv.reserve(argv.size() + 1);
    for (const auto& s : argv) cargv.push_back(const_cast<char*>(s.c_str()));
    cargv.push_back(nullptr);

    std::string joined;
    for (const auto& s : argv) { joined += s; joined += ' '; }

    pid_t pid = fork();
    if (pid < 0) {
        RCLCPP_ERROR(this->get_logger(), "fork() failed for child [%s]: %s",
                     tag.c_str(), std::strerror(errno));
        return -1;
    }
    if (pid == 0) {
        // Child: own process group so we can signal the whole ros2 launch tree.
        setpgid(0, 0);
        execvp(cargv[0], cargv.data());
        _exit(127);  // exec failed
    }
    setpgid(pid, pid);  // race-safe duplicate of the child's setpgid
    children_.emplace_back(pid, tag);
    RCLCPP_INFO(this->get_logger(), "launched child [%s] pid=%d: %s",
                tag.c_str(), pid, joined.c_str());
    return pid;
}

void DemoReplaySyncOrchestratorNode::stopChildren() {
    if (children_.empty()) return;
    for (const auto& [pid, tag] : children_) {
        RCLCPP_INFO(rclcpp::get_logger("demo_replay_sync_orchestrator"),
                    "SIGINT -> child [%s] pgid=%d", tag.c_str(), pid);
        kill(-pid, SIGINT);
    }
    // Give them a few seconds to shut down cleanly, then SIGKILL survivors.
    for (int i = 0; i < 30; ++i) {
        bool all_done = true;
        for (const auto& [pid, tag] : children_) {
            (void)tag;
            int status = 0;
            pid_t r = waitpid(pid, &status, WNOHANG);
            if (r == 0) all_done = false;
        }
        if (all_done) break;
        usleep(100 * 1000);
    }
    for (const auto& [pid, tag] : children_) {
        int status = 0;
        if (waitpid(pid, &status, WNOHANG) == 0) {
            RCLCPP_WARN(rclcpp::get_logger("demo_replay_sync_orchestrator"),
                        "SIGKILL -> child [%s] pgid=%d (did not exit on SIGINT)",
                        tag.c_str(), pid);
            kill(-pid, SIGKILL);
            waitpid(pid, &status, 0);
        }
    }
    children_.clear();
}

}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning
