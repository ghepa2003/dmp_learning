// Manipulability::wTrans() parity with the already-validated Python analysis
// (compute_manipulability_goals.py). Reads tools/gazebo_cartesian_eval/data/*.csv and
// $HOME/thesis_ws/fer_flat_effort.urdf; needs Pinocchio.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "satellite_grasp_planner/core/manipulability.hpp"
#include "test_fixtures.hpp"
#include "test_goal_data.hpp"

using satellite_grasp_planner::core::Manipulability;
using RobotModel = franka_cartesian_control::core::RobotModel;
using satellite_grasp_planner::test_fixtures::loadPandaRobotModel;
using satellite_grasp_planner::test_goal_data::dataDir;
using satellite_grasp_planner::test_goal_data::jointVectorFromRow;
using satellite_grasp_planner::test_goal_data::readCsv;

// ---------------------------------------------------------------------
// Test 1: C++ wTrans() matches the already-validated Python analysis
// ---------------------------------------------------------------------
TEST(ManipulabilityTest, MatchesValidatedPythonValuesForAllFiveGoals) {
    auto robot_model = loadPandaRobotModel();
    ASSERT_NE(robot_model, nullptr);
    Manipulability manip(robot_model);

    const std::vector<std::string> goal_runs = {
        "reach_task_goal_1_prodmp", "reach_task_goal_2_prodmp", "reach_task_goal_3_prodmp",
        "reach_task_goal_4_prodmp", "reach_task_goal_5_prodmp"};

    for (const auto& run : goal_runs) {
        auto rows = readCsv(dataDir() + "/manipulability_" + run + ".csv");
        ASSERT_FALSE(rows.empty()) << "run=" << run;

        // Last row = steady-state / pre-clamp configuration, the same one
        // compute_manipulability_goals.py reports as "pre-clamp" stats.
        const auto& last_row = rows.back();
        RobotModel::JointVector q = jointVectorFromRow(last_row);
        double w_trans_python = last_row.at("w_trans");

        double w_trans_cpp = manip.wTrans(q);

        EXPECT_NEAR(w_trans_cpp, w_trans_python, 1e-6)
            << "run=" << run << ": C++ wTrans() must match the validated Python "
            << "compute_manipulability_goals.py value bit-for-bit (same URDF/frame/formula).";
    }
}
