// Tests for RobotModel joint limits / readyPose() and checkKinematicFeasibility().
// Like test_phase_selector.cpp these need Pinocchio and $HOME/thesis_ws/fer_flat_effort.urdf.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/types.hpp"
#include "satellite_grasp_planner/core/kinematic_feasibility.hpp"

using satellite_grasp_planner::core::checkKinematicFeasibility;
using RobotModel = franka_cartesian_control::core::RobotModel;

namespace {

std::shared_ptr<RobotModel> loadPandaRobotModel() {
    const char* home = std::getenv("HOME");
    const std::string urdf_path = std::string(home ? home : "/root") + "/thesis_ws/fer_flat_effort.urdf";
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

/// Short synthetic ProDMP (2 s, straight 0.1 m line): only shape/duration matter, the goal and
/// initial conditions are overridden by checkKinematicFeasibility().
haptic_dmp_learning::core::ProDMP makeSyntheticProDmpTemplate() {
    std::vector<haptic_dmp_learning::core::Sample> demo;
    const double tau = 2.0;
    const int n = 50;
    for (int i = 0; i <= n; ++i) {
        haptic_dmp_learning::core::Sample s;
        s.t = tau * static_cast<double>(i) / static_cast<double>(n);
        s.position = Eigen::Vector3d(0.1 * s.t / tau, 0.0, 0.0);
        demo.push_back(s);
    }
    haptic_dmp_learning::core::ProDMP prodmp(/*num_basis=*/8);
    prodmp.learnFromDemonstration(demo);
    return prodmp;
}

}  // namespace

TEST(RobotModelLimitsTest, JointLimitsMatchKnownFrankaValuesFromUrdf) {
    auto model = loadPandaRobotModel();
    ASSERT_NE(model, nullptr);
    // Values hardcoded as JOINT_LIMITS in tools/cpp_sweep_free_roll.cpp (4-decimal rounding).
    const double lo[7] = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
    const double hi[7] = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
    for (int j = 0; j < 7; ++j) {
        EXPECT_NEAR(model->jointLowerLimits()(j), lo[j], 1e-4) << "lower, joint " << j + 1;
        EXPECT_NEAR(model->jointUpperLimits()(j), hi[j], 1e-4) << "upper, joint " << j + 1;
    }
}

TEST(RobotModelReadyPoseTest, MatchesHomingValuesUsedInTools) {
    RobotModel::JointVector expected;
    expected << 0.0, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, 1.5707963267948966,
        0.7853981633974483;
    const RobotModel::JointVector q = RobotModel::readyPose();
    for (int j = 0; j < 7; ++j) EXPECT_DOUBLE_EQ(q(j), expected(j)) << "joint " << j + 1;
    // Also the documented closed form.
    EXPECT_NEAR(q(1), -M_PI / 4.0, 1e-15);
    EXPECT_NEAR(q(3), -3.0 * M_PI / 4.0, 1e-15);
}

TEST(KinematicFeasibilityTest, NearbyTargetFromReadyPoseIsFeasible) {
    auto model = loadPandaRobotModel();
    ASSERT_NE(model, nullptr);
    model->update(RobotModel::readyPose(), RobotModel::JointVector::Zero());
    const Eigen::Vector3d target = model->eePosition() + Eigen::Vector3d(0.1, 0.0, 0.0);
    const Eigen::Quaterniond quat = model->eeOrientation();

    const auto prodmp = makeSyntheticProDmpTemplate();
    const auto res = checkKinematicFeasibility(prodmp, target, quat, model);

    EXPECT_TRUE(res.feasible);
    EXPECT_LT(res.max_joint_violation_rad, 1e-4);
    // Sanity only (simulator tracking lag is a few mm); it does not decide `feasible`.
    EXPECT_LT(res.pos_err_final_mm, 50.0);
}

// Provable violation, independent of IK convergence: JointPathSimulator::simulate() documents
// output[0].q == q0 (evaluated, not integrated), so a q0 whose joint 4 lies beyond its upper
// limit (-0.0698 rad) makes the very first Step violate that limit by (q0(3) - upper) whatever the
// controller then does. The target equals the FK pose at q0 (tiny motion), so the test does not
// depend on the simulator's tracking behaviour.
TEST(KinematicFeasibilityTest, StartBeyondJointLimitIsInfeasibleWithExactViolation) {
    auto model = loadPandaRobotModel();
    ASSERT_NE(model, nullptr);

    RobotModel::JointVector q0 = RobotModel::readyPose();
    q0(3) = -0.02;  // upper limit of joint 4 is -0.0698 -> 0.0498 rad beyond it
    const double expected_min_violation = q0(3) - model->jointUpperLimits()(3);
    ASSERT_GT(expected_min_violation, 1e-4);

    model->update(q0, RobotModel::JointVector::Zero());
    const Eigen::Vector3d target = model->eePosition() + Eigen::Vector3d(0.01, 0.0, 0.0);
    const Eigen::Quaterniond quat = model->eeOrientation();

    const auto prodmp = makeSyntheticProDmpTemplate();
    const auto res = checkKinematicFeasibility(prodmp, target, quat, model, q0);

    EXPECT_FALSE(res.feasible);
    EXPECT_GE(res.max_joint_violation_rad, expected_min_violation - 1e-12);
}

TEST(KinematicFeasibilityTest, DoesNotModifyProDmpTemplate) {
    auto model = loadPandaRobotModel();
    ASSERT_NE(model, nullptr);
    const auto prodmp = makeSyntheticProDmpTemplate();
    const Eigen::Vector3d goal_before = prodmp.goal();
    const Eigen::Vector3d init_before = prodmp.initPos();
    checkKinematicFeasibility(prodmp, Eigen::Vector3d(0.5, 0.0, 0.5), Eigen::Quaterniond::Identity(),
                              model);
    EXPECT_EQ(prodmp.goal(), goal_before);
    EXPECT_EQ(prodmp.initPos(), init_before);
}
