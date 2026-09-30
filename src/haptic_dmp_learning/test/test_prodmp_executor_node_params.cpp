#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "haptic_dmp_learning/core/quaternion_dmp.hpp"
#include "haptic_dmp_learning/ros/prodmp_gazebo_executor_node.hpp"

// Constructor-level tests of ProDmpGazeboExecutorNode: parameter declaration and validation only.
// No spinning, no Gazebo, no TF, no controller. The weights file is generated here with the core
// writer. Parameters reach the node through rclcpp's global --ros-args (the node takes no options).

using haptic_dmp_learning::ros_wrapper::ProDmpGazeboExecutorNode;
namespace core = haptic_dmp_learning::core;

namespace {

std::string writeWeights() {
    const int N = 160;
    const double dt = 0.01;
    std::vector<core::Sample> demo(N);
    for (int i = 0; i < N; ++i) {
        const double s = static_cast<double>(i) / (N - 1);
        const double poly = s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
        demo[i].t = i * dt;
        demo[i].position = Eigen::Vector3d(0.0, 0.1, 0.0) + poly * Eigen::Vector3d(0.3, -0.2, 0.2);
        demo[i].orientation = Eigen::Quaterniond::Identity();
    }
    core::ProDMP mp(16);
    mp.learnFromDemonstration(demo);
    core::QuaternionDMP q(20, 4.6, 25.0, 6.25);
    q.learnFromDemonstration(demo);
    const std::string path =
        (std::filesystem::temp_directory_path() / "test_prodmp_executor_node_params.yaml").string();
    core::prodmp_io::saveProDmpToYaml(mp, &q, path);
    return path;
}

const std::vector<std::string> kContinuousBase = {
    "satellite_rotation_enabled:=true",
    "satellite_rotation_mode:=continuous",
    "satellite_rotation_axis:=[0.0,0.0,1.0]",
    "satellite_rotation_center:=[0.75,0.0,0.35]",
    "satellite_rotation_angular_velocity_deg_s:=-2.0",
    "satellite_phase_source:=measured",
    "satellite_odom_topic:=/free_target_object/odometry",
    "satellite_q_ref:=[0.0,0.0,0.0,1.0]",
    "use_sim_time:=true",
    "target_odom_required:=false",
};

class ExecutorNodeParams : public ::testing::Test {
protected:
    void SetUp() override { weights_ = writeWeights(); }
    void TearDown() override {
        if (rclcpp::ok()) rclcpp::shutdown();
        std::remove(weights_.c_str());
    }

    /// Initialises rclcpp with the given "-p"-style overrides (plus the weights path).
    void init(const std::vector<std::string>& overrides) {
        std::vector<std::string> args = {"test", "--ros-args", "-p", "weights_yaml_path:=" + weights_};
        // orientation_weights_yaml_path left empty: the node reads the inline quaternion section.
        for (const auto& o : overrides) {
            args.push_back("-p");
            args.push_back(o);
        }
        std::vector<const char*> argv;
        for (const auto& a : args) argv.push_back(a.c_str());
        rclcpp::init(static_cast<int>(argv.size()), argv.data());
    }

    static std::vector<std::string> with(std::vector<std::string> base, const std::vector<std::string>& extra) {
        base.insert(base.end(), extra.begin(), extra.end());
        return base;
    }

    std::string weights_;
};

}  // namespace

TEST_F(ExecutorNodeParams, DefaultSourceIsParametersAndDeclaresNothingElse) {
    init({"target_odom_required:=false"});
    auto node = std::make_shared<ProDmpGazeboExecutorNode>();
    EXPECT_EQ(node->get_parameter("grasp_goal_source").as_string(), "parameters");
    EXPECT_FALSE(node->has_parameter("robot_base_world"));
    EXPECT_FALSE(node->has_parameter("grasp_command_topic"));
}

TEST_F(ExecutorNodeParams, InvalidSourceThrows) {
    init({"grasp_goal_source:=planner"});
    EXPECT_THROW(ProDmpGazeboExecutorNode(), std::invalid_argument);
}

TEST_F(ExecutorNodeParams, GraspCommandRequiresContinuousMode) {
    init({"grasp_goal_source:=grasp_command", "target_odom_required:=false"});
    EXPECT_THROW(ProDmpGazeboExecutorNode(), std::invalid_argument);
}

TEST_F(ExecutorNodeParams, GraspCommandModeConstructsAndDeclaresItsParameters) {
    init(with(kContinuousBase, {"grasp_goal_source:=grasp_command", "robot_base_world:=[0.0,0.0,0.0]"}));
    auto node = std::make_shared<ProDmpGazeboExecutorNode>();
    EXPECT_EQ(node->get_parameter("grasp_goal_source").as_string(), "grasp_command");
    EXPECT_EQ(node->get_parameter("grasp_command_topic").as_string(), "/grasp_command");
    EXPECT_DOUBLE_EQ(node->get_parameter("grasp_command_timeout_s").as_double(), 300.0);
    EXPECT_DOUBLE_EQ(node->get_parameter("grasp_command_max_age_s").as_double(), 0.0);
    EXPECT_DOUBLE_EQ(node->get_parameter("omega_consistency_tol_deg_per_s").as_double(), 0.1);
    EXPECT_DOUBLE_EQ(node->get_parameter("grasp_command_phase_tol_deg").as_double(), 1.0);
    EXPECT_DOUBLE_EQ(node->get_parameter("robot_base_world_tol_m").as_double(), 1e-6);
    // Declared exactly once, as a double array.
    EXPECT_EQ(node->get_parameter("robot_base_world").get_type(), rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
}

TEST_F(ExecutorNodeParams, MissingRobotBaseWorldThrows) {
    init(with(kContinuousBase, {"grasp_goal_source:=grasp_command"}));
    try {
        ProDmpGazeboExecutorNode n;
        FAIL() << "expected std::invalid_argument";
    } catch (const std::invalid_argument& e) {
        EXPECT_NE(std::string(e.what()).find("robot_base_world"), std::string::npos);
    }
}

TEST_F(ExecutorNodeParams, RobotBaseWorldWrongLengthThrows) {
    init(with(kContinuousBase, {"grasp_goal_source:=grasp_command", "robot_base_world:=[0.0,0.0]"}));
    EXPECT_THROW(ProDmpGazeboExecutorNode(), std::invalid_argument);
}

TEST_F(ExecutorNodeParams, RobotBaseWorldNonZeroThrows) {
    init(with(kContinuousBase, {"grasp_goal_source:=grasp_command", "robot_base_world:=[0.1,0.0,0.0]"}));
    try {
        ProDmpGazeboExecutorNode n;
        FAIL() << "expected std::invalid_argument";
    } catch (const std::invalid_argument& e) {
        EXPECT_NE(std::string(e.what()).find("from the origin (limit "), std::string::npos);
    }
}

TEST_F(ExecutorNodeParams, InterceptPhaseParameterIsRejectedInGraspCommandMode) {
    init(with(kContinuousBase, {"grasp_goal_source:=grasp_command", "robot_base_world:=[0.0,0.0,0.0]",
                                "satellite_intercept_phase_deg:=90.0"}));
    EXPECT_THROW(ProDmpGazeboExecutorNode(), std::invalid_argument);
}

TEST_F(ExecutorNodeParams, ModelPhaseSourceIsRejectedInGraspCommandMode) {
    init(with(kContinuousBase, {"grasp_goal_source:=grasp_command", "robot_base_world:=[0.0,0.0,0.0]",
                                "satellite_phase_source:=model"}));
    EXPECT_THROW(ProDmpGazeboExecutorNode(), std::invalid_argument);
}
