// JointPathSimulator: DLS-vs-impedance proxy gap against real impedance-control runs (Goal 2 / Goal 4)
// and the redundancy-resolution strategy factory. Reads tools/gazebo_cartesian_eval/data/*.csv and
// $HOME/thesis_ws/fer_flat_effort.urdf; needs Pinocchio.

#include <gtest/gtest.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "satellite_grasp_planner/core/joint_path_simulator.hpp"
#include "satellite_grasp_planner/core/manipulability.hpp"
#include "test_fixtures.hpp"
#include "test_goal_data.hpp"

using satellite_grasp_planner::core::CartesianSample;
using satellite_grasp_planner::core::JointPathSimulator;
using satellite_grasp_planner::core::Manipulability;
using satellite_grasp_planner::core::MinNormDlsResolution;
using satellite_grasp_planner::core::NullspaceBiasedResolution;
using satellite_grasp_planner::core::createStrategyForController;
using RobotModel = franka_cartesian_control::core::RobotModel;
using satellite_grasp_planner::test_fixtures::loadPandaRobotModel;
using satellite_grasp_planner::test_goal_data::dataDir;
using satellite_grasp_planner::test_goal_data::jointVectorFromRow;
using satellite_grasp_planner::test_goal_data::readCsv;

// ---------------------------------------------------------------------
// Test 2 (review point 2): Redundancy resolution strategies (MinNormDls vs
// NullspaceBiased) against real impedance-control runs for Goal 2 and Goal 4.
// ---------------------------------------------------------------------
TEST(JointPathSimulatorTest, StrategyComparisonVsRealImpedanceControlGoals) {
    auto robot_model = loadPandaRobotModel();
    ASSERT_NE(robot_model, nullptr);
    Manipulability manip(robot_model);
    const double kDeg2Rad = M_PI / 180.0;

    struct GoalTestCase {
        std::string name;
        std::string run_key;
        RobotModel::JointVector q_real_impedance;
    };

    std::vector<GoalTestCase> cases;
    {
        GoalTestCase g2;
        g2.name = "Goal 2 (critical zone)";
        g2.run_key = "reach_task_goal_2_prodmp";
        g2.q_real_impedance << -0.6 * kDeg2Rad, -40.5 * kDeg2Rad, -9.2 * kDeg2Rad, -154.8 * kDeg2Rad,
            -12.1 * kDeg2Rad, 115.7 * kDeg2Rad, 54.5 * kDeg2Rad;
        cases.push_back(g2);

        GoalTestCase g4;
        g4.name = "Goal 4 (nominal zone)";
        g4.run_key = "reach_task_goal_4_prodmp";
        g4.q_real_impedance << -0.4 * kDeg2Rad, -37.9 * kDeg2Rad, -16.8 * kDeg2Rad, -141.4 * kDeg2Rad,
            -15.4 * kDeg2Rad, 106.5 * kDeg2Rad, 47.7 * kDeg2Rad;
        cases.push_back(g4);
    }

    for (const auto& tc : cases) {
        auto target_rows = readCsv(dataDir() + "/target_aligned_" + tc.run_key + ".csv");
        auto js_rows = readCsv(dataDir() + "/joint_states_" + tc.run_key + ".csv");
        ASSERT_FALSE(target_rows.empty()) << tc.name;
        ASSERT_FALSE(js_rows.empty()) << tc.name;

        RobotModel::JointVector q0 = jointVectorFromRow(js_rows.front());
        const auto& target = target_rows.back();
        Eigen::Vector3d goal_pos(target.at("x"), target.at("y"), target.at("z"));
        Eigen::Quaterniond goal_quat(target.at("qw"), target.at("qx"), target.at("qy"), target.at("qz"));
        goal_quat.normalize();

        const double hold_duration = 8.0;
        const double dt = 0.005;
        const int n_steps = static_cast<int>(hold_duration / dt);
        std::vector<CartesianSample> trajectory(static_cast<size_t>(n_steps));
        for (auto& s : trajectory) {
            s.position = goal_pos;
            s.orientation = goal_quat;
        }

        // 1. MinNormDlsResolution (pure velocity control proxy)
        JointPathSimulator::Params dls_params;
        dls_params.dt = dt;
        dls_params.strategy = std::make_shared<MinNormDlsResolution>();
        JointPathSimulator dls_sim(robot_model, dls_params);
        auto dls_steps = dls_sim.simulate(trajectory, q0);
        ASSERT_FALSE(dls_steps.empty());
        RobotModel::JointVector q_dls = dls_steps.back().q;
        double w_dls = manip.wTrans(q_dls);

        // 2. NullspaceBiasedResolution (impedance control proxy)
        JointPathSimulator::Params ns_params;
        ns_params.dt = dt;
        ns_params.strategy = std::make_shared<NullspaceBiasedResolution>();
        JointPathSimulator ns_sim(robot_model, ns_params);
        auto ns_steps = ns_sim.simulate(trajectory, q0);
        ASSERT_FALSE(ns_steps.empty());
        RobotModel::JointVector q_ns = ns_steps.back().q;
        double w_ns = manip.wTrans(q_ns);

        double w_real = manip.wTrans(tc.q_real_impedance);
        double gap_dls_pct = 100.0 * (w_dls - w_real) / w_real;
        double gap_ns_pct = 100.0 * (w_ns - w_real) / w_real;

        std::cout << "\n=== Strategy Comparison vs Real Impedance Control (" << tc.name << ") ===\n";
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "  w_trans (Real Impedance)   : " << w_real << "\n";
        std::cout << "  w_trans (MinNormDls)       : " << w_dls << " (gap = " << gap_dls_pct << "%)\n";
        std::cout << "  w_trans (NullspaceBiased)  : " << w_ns  << " (gap = " << gap_ns_pct  << "%)\n";

        for (int i = 0; i < 7; ++i) {
            double real_deg = tc.q_real_impedance(i) / kDeg2Rad;
            double dls_deg  = q_dls(i) / kDeg2Rad;
            double ns_deg   = q_ns(i) / kDeg2Rad;
            std::cout << "  fer_joint" << (i + 1)
                      << ": Real=" << real_deg << " deg | DLS=" << dls_deg << " deg | NS=" << ns_deg << " deg\n";
        }
        std::cout << "=======================================================================\n";

        EXPECT_GT(w_dls, 0.0);
        EXPECT_GT(w_ns, 0.0);
    }
}

TEST(RedundancyStrategyTest, FactoryCreatesCorrectStrategies) {
    auto strat_imp = createStrategyForController("cartesian_impedance_controller");
    EXPECT_NE(dynamic_cast<NullspaceBiasedResolution*>(strat_imp.get()), nullptr);

    auto strat_imp_short = createStrategyForController("impedance");
    EXPECT_NE(dynamic_cast<NullspaceBiasedResolution*>(strat_imp_short.get()), nullptr);

    auto strat_vel = createStrategyForController("velocity_cartesian_controller");
    EXPECT_NE(dynamic_cast<MinNormDlsResolution*>(strat_vel.get()), nullptr);

    auto strat_vel_alt = createStrategyForController("cartesian_velocity_controller");
    EXPECT_NE(dynamic_cast<MinNormDlsResolution*>(strat_vel_alt.get()), nullptr);

    EXPECT_THROW(createStrategyForController("unknown_controller"), std::invalid_argument);
}
