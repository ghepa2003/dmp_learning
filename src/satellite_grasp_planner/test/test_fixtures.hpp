#pragma once

// Shared, header-only helpers of the satellite_grasp_planner tests (copied from the versions that used
// to be repeated in each test file). Everything is inline in satellite_grasp_planner::test_fixtures.

#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "haptic_dmp_learning/core/types.hpp"

namespace satellite_grasp_planner {
namespace test_fixtures {

using RobotModel = franka_cartesian_control::core::RobotModel;

/// Satellite "configuration B" (vertical axis, lateral face towards the robot base).
constexpr double kModelBCenterX = 0.75;
constexpr double kModelBCenterY = 0.0;
constexpr double kModelBCenterZ = 0.35;

/// $HOME/thesis_ws/fer_flat_effort.urdf (HOME falls back to /root).
inline std::string urdfPath() {
    const char* home = std::getenv("HOME");
    return std::string(home ? home : "/root") + "/thesis_ws/fer_flat_effort.urdf";
}

/// Panda RobotModel from the URDF above (frame fer_hand_tcp). Adds a test failure and returns nullptr if
/// the URDF cannot be opened.
inline std::shared_ptr<RobotModel> loadPandaRobotModel() {
    const std::string urdf_path = urdfPath();
    std::ifstream f(urdf_path);
    if (!f.is_open()) {
        ADD_FAILURE() << "Could not open URDF (required test fixture): " << urdf_path;
        return nullptr;
    }
    std::stringstream buffer;
    buffer << f.rdbuf();
    std::vector<std::string> joint_names;
    for (int i = 1; i <= 7; ++i) joint_names.push_back("fer_joint" + std::to_string(i));
    return std::make_shared<RobotModel>(buffer.str(), joint_names, "fer_hand_tcp");
}

/// Short synthetic ProDMP (2 s, straight 0.1 m line along x, 8 basis functions): only shape/duration
/// matter, goals and initial conditions are overridden by the code under test.
inline haptic_dmp_learning::core::ProDMP makeSyntheticProDmpTemplate() {
    std::vector<haptic_dmp_learning::core::Sample> demo;
    for (int i = 0; i <= 50; ++i) {
        haptic_dmp_learning::core::Sample s;
        s.t = 2.0 * i / 50.0;
        s.position = Eigen::Vector3d(0.1 * s.t / 2.0, 0.0, 0.0);
        demo.push_back(s);
    }
    haptic_dmp_learning::core::ProDMP p(/*num_basis=*/8);
    p.learnFromDemonstration(demo);
    return p;
}

/// Configuration B: center (0.75, 0, 0.35), axis +z, face_normal_body (-1, 0, 0), the rest default.
inline haptic_dmp_learning::core::CubeSatelliteModel::Params makeModelBParams() {
    haptic_dmp_learning::core::CubeSatelliteModel::Params p;
    p.center_world = Eigen::Vector3d(kModelBCenterX, kModelBCenterY, kModelBCenterZ);
    p.axis_world = Eigen::Vector3d(0.0, 0.0, 1.0);
    p.face_normal_body = Eigen::Vector3d(-1.0, 0.0, 0.0);
    return p;
}

inline haptic_dmp_learning::core::CubeSatelliteModel makeModelB() {
    return haptic_dmp_learning::core::CubeSatelliteModel(makeModelBParams());
}

/// Where the production weights.yaml is expected: $GRASP_PROBE_WEIGHTS, else the n80 fit run under
/// $HOME/thesis_ws/runs (whether or not the file exists).
inline std::string productionWeightsCandidatePath() {
    const char* home = std::getenv("HOME");
    const char* env_w = std::getenv("GRASP_PROBE_WEIGHTS");
    return env_w ? std::string(env_w)
                 : std::string(home ? home : "/root") +
                       "/thesis_ws/runs/20260914_150515_fit_reach_task_baseline_prodmp_n80_lam1e-10_w0.05/"
                       "weights.yaml";
}

/// The production weights path if that file exists, else an empty string.
inline std::string productionWeightsPath() {
    const std::string p = productionWeightsCandidatePath();
    return std::ifstream(p).good() ? p : std::string();
}

/// Loads the production ProDMP template from @p weights_path (prodmp_io::loadProDmpFromYaml).
inline haptic_dmp_learning::core::ProDMP loadProductionTemplate(const std::string& weights_path) {
    return haptic_dmp_learning::core::prodmp_io::loadProDmpFromYaml(weights_path);
}

/// robot->update(readyPose, zero velocities).
inline void resetRobotToReady(RobotModel& robot) {
    robot.update(RobotModel::readyPose(), RobotModel::JointVector::Zero());
}
template <class Ptr>
inline void resetRobotToReady(const Ptr& robot) {
    robot->update(RobotModel::readyPose(), RobotModel::JointVector::Zero());
}

}  // namespace test_fixtures
}  // namespace satellite_grasp_planner

/// First statements of a probe that needs the production weights: declares `const std::string var` with
/// the path and skips (GTEST_SKIP needs a void test body, hence a macro) if the file is missing.
#define SKIP_UNLESS_PRODUCTION_WEIGHTS(var)                                                        \
    const std::string var = ::satellite_grasp_planner::test_fixtures::productionWeightsCandidatePath(); \
    if (!std::ifstream(var).good()) {                                                              \
        GTEST_SKIP() << "Production weights.yaml not reachable: " << var                           \
                     << " (set GRASP_PROBE_WEIGHTS)";                                               \
    }
