#include "satellite_grasp_planner/ros/grasp_planner_node.hpp"

#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "haptic_dmp_learning/core/math_utils.hpp"
#include "haptic_dmp_learning/core/satellite_intercept.hpp"
#include "satellite_grasp_planner/core/grasp_launch.hpp"
#include "satellite_grasp_planner/core/selection_inputs.hpp"
#include "satellite_grasp_planner/ros/grasp_command_conversion.hpp"
#include "satellite_grasp_planner/ros/grasp_planner_checks.hpp"

namespace satellite_grasp_planner {
namespace ros_wrapper {

namespace {

Eigen::Vector3d toVector3(const std::vector<double>& v, const char* name) {
    if (v.size() != 3) {
        throw std::invalid_argument(std::string(name) + " must have exactly 3 elements");
    }
    return Eigen::Vector3d(v[0], v[1], v[2]);
}

/// x, y, z, w - same order and validation as prodmp_gazebo_executor_node's satellite_q_ref (the YAML
/// section is shared between the two nodes).
Eigen::Quaterniond toQuaternion(const std::vector<double>& v, const char* name) {
    if (v.size() != 4) {
        throw std::invalid_argument(std::string(name) + " must have exactly 4 elements (x, y, z, w)");
    }
    const Eigen::Quaterniond q(v[3], v[0], v[1], v[2]);
    const double n = q.norm();
    if (!(std::fabs(n - 1.0) <= 1e-6)) {
        throw std::invalid_argument(std::string(name) + " norm must be 1 +/- 1e-6");
    }
    return q.normalized();
}

}  // namespace

GraspPlannerNode::GraspPlannerNode() : rclcpp::Node("grasp_planner_node") {
    // --- use_sim_time / sim clock (fail loud - see class doc comment) ---
    // rclcpp::Node auto-declares "use_sim_time" itself (it drives the node's Clock); declaring it
    // again would throw ParameterAlreadyDeclaredException - read it back instead, same as
    // prodmp_gazebo_executor_node.cpp does.
    const bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
    if (!use_sim_time) {
        throw std::invalid_argument("grasp_planner_node requires use_sim_time=true");
    }
    if (!this->get_clock()->ros_time_is_active()) {
        throw std::invalid_argument("grasp_planner_node: ROS (sim) time is not active on the node clock");
    }

    // --- mandatory parameters, NO defaults, fail loud ---
    // Declared by TYPE ONLY (no default value): an unset one comes back PARAMETER_NOT_SET, named
    // explicitly in the exception (same pattern as prodmp_gazebo_executor_node::setupContinuous()'s
    // opt_double/opt_string/opt_vec).
    auto missing = [](const char* name) -> void {
        throw std::invalid_argument(std::string("mandatory parameter '") + name + "' was not provided");
    };
    auto require_string = [this, &missing](const char* name) {
        const auto v = this->declare_parameter(name, rclcpp::ParameterType::PARAMETER_STRING);
        if (v.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) missing(name);
        return v.get<std::string>();
    };
    auto require_double = [this, &missing](const char* name) {
        const auto v = this->declare_parameter(name, rclcpp::ParameterType::PARAMETER_DOUBLE);
        if (v.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) missing(name);
        return v.get<double>();
    };
    auto require_vec = [this, &missing](const char* name) {
        const auto v = this->declare_parameter(name, rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
        if (v.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) missing(name);
        return v.get<std::vector<double>>();
    };

    demo_params_path_ = require_string("demo_params_path");
    urdf_path_ = require_string("urdf_path");
    min_delay_s_ = require_double("min_delay_s");

    cube_geometry_.center_world = toVector3(require_vec("cube_geometry.center_world"), "cube_geometry.center_world");
    cube_geometry_.axis_world = toVector3(require_vec("cube_geometry.axis_world"), "cube_geometry.axis_world");
    cube_geometry_.cube_side_m = require_double("cube_geometry.cube_side_m");
    cube_geometry_.standoff_m = require_double("cube_geometry.standoff_m");
    cube_geometry_.collar_radius_m = require_double("cube_geometry.collar_radius_m");
    cube_geometry_.face_normal_body =
        toVector3(require_vec("cube_geometry.face_normal_body"), "cube_geometry.face_normal_body");
    cube_geometry_.wall_thickness_m = require_double("cube_geometry.wall_thickness_m");

    robot_base_world_ = toVector3(require_vec("robot_base_world"), "robot_base_world");
    satellite_rotation_center_ = toVector3(require_vec("satellite_rotation_center"), "satellite_rotation_center");
    satellite_rotation_axis_ = toVector3(require_vec("satellite_rotation_axis"), "satellite_rotation_axis");
    const double omega_deg_s = require_double("satellite_rotation_angular_velocity_deg_s");
    satellite_omega_rad_s_ = haptic_dmp_learning::core::degToRad(omega_deg_s);
    satellite_q_ref_ = toQuaternion(require_vec("satellite_q_ref"), "satellite_q_ref");
    const std::string odom_topic = require_string("satellite_odom_topic");
    const double omega_tol_deg_s = require_double("omega_consistency_tol_deg_per_s");
    omega_consistency_tol_rad_s_ = haptic_dmp_learning::core::degToRad(omega_tol_deg_s);

    // --- optional parameters ---
    const double q_ref_tol_deg = this->declare_parameter<double>("q_ref_consistency_tol_deg", 0.5);
    const double robot_base_world_tol_m = this->declare_parameter<double>("robot_base_world_tol_m", 1e-6);
    grasp_command_topic_ = this->declare_parameter<std::string>("grasp_command_topic", "/grasp_command");
    const double time_budget_s = this->declare_parameter<double>("time_budget_s", -1.0);
    if (time_budget_s >= 0.0) time_budget_s_ = time_budget_s;

    // SelectionParams overrides: declared by type only, so "not set" (keep the SelectionParams
    // default) is distinguishable from any value. Validated here so a bad value fails at startup.
    auto opt_double = [this](const char* name) -> std::optional<double> {
        const auto v = this->declare_parameter(name, rclcpp::ParameterType::PARAMETER_DOUBLE);
        if (v.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) return std::nullopt;
        return v.get<double>();
    };
    selection_overrides_.scan_step_deg = opt_double("scan_step_deg");
    selection_overrides_.w_hat_upper_bound = opt_double("w_hat_upper_bound");
    selection_overrides_.psi_tol_deg = opt_double("psi_tol_deg");
    {
        const auto v = this->declare_parameter("max_rows", rclcpp::ParameterType::PARAMETER_INTEGER);
        if (v.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
            const int64_t rows = v.get<int64_t>();
            if (rows > std::numeric_limits<int>::max()) {
                throw std::invalid_argument("parameter 'max_rows' is too large: " + std::to_string(rows));
            }
            selection_overrides_.max_rows = static_cast<int>(rows);  // <= 0 is rejected below
        }
    }
    validateSelectionOverrides(selection_overrides_);

    // --- startup validation (pure functions, see grasp_planner_checks.hpp) ---
    checkRobotBaseWorldIsZero(robot_base_world_, robot_base_world_tol_m);
    checkCubeAxisMatchesRotationAxis(cube_geometry_.axis_world, satellite_rotation_axis_);
    checkCubeCenterMatchesRotationCenter(cube_geometry_.center_world, satellite_rotation_center_);
    checkQRefOnAxis(satellite_q_ref_, satellite_rotation_axis_, q_ref_tol_deg);

    // --- publisher (QoS: reliable, transient_local, depth 1 - one-shot retained command) ---
    pub_ = this->create_publisher<satellite_grasp_msgs::msg::GraspCommand>(
        grasp_command_topic_, rclcpp::QoS(1).reliable().transient_local());

    // --- subscription (first sample only, see onOdom) ---
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::QoS(10).reliable(),
        std::bind(&GraspPlannerNode::onOdom, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "grasp_planner_node: ready, waiting for the first sample on '%s'.",
                odom_topic.c_str());
}

GraspPlannerNode::~GraspPlannerNode() {
    if (worker_.joinable()) worker_.join();
}

void GraspPlannerNode::onOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    if (dispatched_.exchange(true)) return;  // only the first sample is ever handled (one-shot v1)

    try {
        const Eigen::Vector3d position(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                       msg->pose.pose.position.z);
        checkFirstOdomSample(position, msg->header.frame_id, satellite_rotation_center_);

        const auto& q = msg->pose.pose.orientation;
        haptic_dmp_learning::core::satellite_intercept::PhaseTracker tracker(satellite_rotation_axis_,
                                                                             satellite_q_ref_);
        const auto phase = tracker.update(Eigen::Quaterniond(q.w, q.x, q.y, q.z));

        const Eigen::Vector3d twist_angular(msg->twist.twist.angular.x, msg->twist.twist.angular.y,
                                            msg->twist.twist.angular.z);
        checkOmegaConsistency(twist_angular, satellite_rotation_axis_, satellite_omega_rad_s_,
                             omega_consistency_tol_rad_s_);

        core::SatelliteSnapshot snapshot;
        snapshot.center = satellite_rotation_center_;
        snapshot.axis = satellite_rotation_axis_;
        snapshot.omega_rad_s = satellite_omega_rad_s_;
        snapshot.theta_rad = phase.theta_rad;
        snapshot.t_s = rclcpp::Time(msg->header.stamp).seconds();

        worker_ = std::thread(&GraspPlannerNode::runSelection, this, snapshot);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "grasp_planner_node: rejecting first odometry sample: %s", e.what());
        rclcpp::shutdown();
    }
}

void GraspPlannerNode::runSelection(const core::SatelliteSnapshot& snapshot) {
    try {
        core::SelectionInputs in =
            core::buildSelectionInputs(demo_params_path_, urdf_path_, snapshot, cube_geometry_, robot_base_world_);
        if (time_budget_s_) in.params.time_budget_s = time_budget_s_;
        applySelectionOverrides(in.params, selection_overrides_);
        RCLCPP_INFO(this->get_logger(), "grasp_planner_node: starting selection with %s",
                    describeSelectionParams(in.params, selection_overrides_).c_str());

        const core::GraspLaunchResult result = core::planGraspAndLaunch(
            in, snapshot, min_delay_s_, /*clock=*/nullptr, /*t_now_override_s=*/std::nullopt);

        const rclcpp::Time now = this->get_clock()->now();
        if (now.seconds() < snapshot.t_s) {
            std::ostringstream m;
            m << "node clock read " << now.seconds() << " s < snapshot.t_s " << snapshot.t_s
              << " s at the end of selection (sim clock went backwards or was never advancing)";
            throw std::runtime_error(m.str());
        }

        GraspCommandProvenance provenance;
        provenance.start_position = in.params.scan.p0;
        provenance.weights_sha256 = in.weights_sha256;
        const auto msg = toGraspCommandMsg(result, snapshot, now, provenance);
        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(),
                    "grasp_planner_node: published GraspCommand (status=%u %s, stop_reason=%u %s, "
                    "rows_evaluated=%d), now idle.",
                    static_cast<unsigned>(msg.status), statusName(msg.status),
                    static_cast<unsigned>(msg.stop_reason), stopReasonName(msg.stop_reason),
                    msg.rows_evaluated);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "grasp_planner_node: selection failed: %s", e.what());
        rclcpp::shutdown();
    }
}

}  // namespace ros_wrapper
}  // namespace satellite_grasp_planner
