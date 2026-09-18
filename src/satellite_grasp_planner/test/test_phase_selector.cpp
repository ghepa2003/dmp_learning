// Level-1 core tests: manipulability formula parity with the already-
// validated Python analysis, the JointPathSimulator DLS-vs-impedance proxy
// gap (point 2 of the Phase-1/2 review), and PhaseSelector's degenerate
// omega=0 behaviour against the 5 known static goals.
//
// NOTE ON FILE PATHS: these tests read the SAME data files the offline
// Python manipulability analysis already validated
// (tools/gazebo_cartesian_eval/data/*.csv, fer_flat_effort.urdf), resolved
// via $HOME/thesis_ws - the same "absolute default, CWD-independent" pattern
// already used by prodmp_gazebo_executor_node.cpp's weights_yaml_path_
// default. They therefore require those files to exist at that fixed
// workspace location (true in this repo) and require Pinocchio, which is
// only installed inside the project's Docker build environment - these
// tests cannot run on a bare host.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "satellite_grasp_planner/core/manipulability.hpp"
#include "satellite_grasp_planner/core/joint_path_simulator.hpp"
#include "satellite_grasp_planner/core/phase_selector.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/types.hpp"
#include "franka_cartesian_control/core/robot_model.hpp"

using satellite_grasp_planner::core::CartesianSample;
using satellite_grasp_planner::core::JointPathSimulator;
using satellite_grasp_planner::core::Manipulability;
using satellite_grasp_planner::core::MinNormDlsResolution;
using satellite_grasp_planner::core::NullspaceBiasedResolution;
using satellite_grasp_planner::core::createStrategyForController;
using satellite_grasp_planner::core::PhaseSelector;
using satellite_grasp_planner::core::SatelliteRotationModel;
using RobotModel = franka_cartesian_control::core::RobotModel;

namespace {

std::string workspaceRoot() {
    const char* home = std::getenv("HOME");
    return std::string(home ? home : "/root") + "/thesis_ws";
}

std::string dataDir() {
    return workspaceRoot() + "/tools/gazebo_cartesian_eval/data";
}

/// @brief Minimal CSV reader: header row -> column names, each following row
/// -> a name->value map. Good enough for the small, well-formed CSVs written
/// by compute_manipulability_goals.py / the Gazebo eval pipeline.
std::vector<std::unordered_map<std::string, double>> readCsv(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        ADD_FAILURE() << "Could not open CSV (required test fixture, see file-level NOTE): " << path;
        return {};
    }
    std::vector<std::unordered_map<std::string, double>> rows;
    std::string header_line;
    std::getline(f, header_line);
    if (!header_line.empty() && header_line.back() == '\r') header_line.pop_back();
    std::vector<std::string> cols;
    {
        std::stringstream ss(header_line);
        std::string col;
        while (std::getline(ss, col, ',')) {
            if (!col.empty() && col.back() == '\r') col.pop_back();
            cols.push_back(col);
        }
    }
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string cell;
        std::unordered_map<std::string, double> row;
        size_t i = 0;
        while (std::getline(ss, cell, ',') && i < cols.size()) {
            row[cols[i]] = std::stod(cell);
            ++i;
        }
        rows.push_back(std::move(row));
    }
    return rows;
}

std::shared_ptr<RobotModel> loadPandaRobotModel() {
    const std::string urdf_path = workspaceRoot() + "/fer_flat_effort.urdf";
    std::ifstream f(urdf_path);
    if (!f.is_open()) {
        ADD_FAILURE() << "Could not open URDF (required test fixture): " << urdf_path;
        return nullptr;
    }
    std::stringstream buffer;
    buffer << f.rdbuf();

    std::vector<std::string> joint_names;
    for (int i = 1; i <= 7; ++i) joint_names.push_back("fer_joint" + std::to_string(i));

    return std::make_shared<RobotModel>(buffer.str(), joint_names, "fer_link8");
}

RobotModel::JointVector jointVectorFromRow(const std::unordered_map<std::string, double>& row) {
    RobotModel::JointVector q;
    for (int i = 0; i < 7; ++i) {
        q(i) = row.at("fer_joint" + std::to_string(i + 1));
    }
    return q;
}

/// @brief Builds a short, self-contained synthetic ProDMP (2 s, few basis
/// functions) fitted on a trivial straight-line demo. Deliberately NOT one
/// of the project's real weights_*.yaml files: PhaseSelector overrides both
/// setInitialConditions() and setGoal() per candidate anyway (see
/// scoreCandidate()), so only the shape/duration matter here, and a short
/// synthetic demo keeps these tests fast (n_steps = tau/dt) and independent
/// of any external weights-file path.
haptic_dmp_learning::core::ProDMP makeSyntheticProDmpTemplate() {
    std::vector<haptic_dmp_learning::core::Sample> demo;
    const double tau = 2.0;
    const int n = 50;
    for (int i = 0; i <= n; ++i) {
        haptic_dmp_learning::core::Sample s;
        s.t = tau * static_cast<double>(i) / static_cast<double>(n);
        s.position = Eigen::Vector3d(0.1 * s.t / tau, 0.0, 0.0);  // arbitrary short straight line
        demo.push_back(s);
    }
    haptic_dmp_learning::core::ProDMP prodmp(/*num_basis=*/8);
    prodmp.learnFromDemonstration(demo);
    return prodmp;
}

}  // namespace

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

// ---------------------------------------------------------------------
// Test 3: PhaseSelector degenerate omega=0 case against the 5 known static
// goals - validates the coarse-to-fine plumbing collapses correctly to a
// single evaluation, and that the resulting score lands in the same
// manipulability regime as the already-recorded Gazebo run for that goal.
// ---------------------------------------------------------------------
class PhaseSelectorDegenerateTest : public ::testing::TestWithParam<std::string> {};

TEST_P(PhaseSelectorDegenerateTest, OmegaZeroMatchesRecordedGoalManipulability) {
    const std::string run = GetParam();

    auto robot_model = loadPandaRobotModel();
    ASSERT_NE(robot_model, nullptr);

    auto target_rows = readCsv(dataDir() + "/target_aligned_" + run + ".csv");
    auto js_rows = readCsv(dataDir() + "/joint_states_" + run + ".csv");
    auto manip_rows = readCsv(dataDir() + "/manipulability_" + run + ".csv");
    ASSERT_FALSE(target_rows.empty());
    ASSERT_FALSE(js_rows.empty());
    ASSERT_FALSE(manip_rows.empty());

    RobotModel::JointVector q0 = jointVectorFromRow(js_rows.front());
    robot_model->update(q0, RobotModel::JointVector::Zero());
    Eigen::Vector3d ee0_pos = robot_model->eePosition();
    Eigen::Quaterniond ee0_quat = robot_model->eeOrientation();

    const auto& target = target_rows.back();
    Eigen::Vector3d goal_pos(target.at("x"), target.at("y"), target.at("z"));
    Eigen::Quaterniond goal_quat(target.at("qw"), target.at("qx"), target.at("qy"), target.at("qz"));
    goal_quat.normalize();

    // Degenerate case: a "satellite" that does not rotate (omega=0). The
    // grasp point in the (irrelevant) body frame is placed directly at the
    // world goal pose with center=goal_pos, so transformPointAt()/
    // transformOrientationAt() return goal_pos/goal_quat unchanged for every
    // t - equivalent to a plain static target.
    SatelliteRotationModel rotation;
    rotation.center = goal_pos;
    rotation.axis = Eigen::Vector3d::UnitZ();
    rotation.phase0 = 0.0;
    rotation.omega = 0.0;
    Eigen::Vector3d grasp_pos_body = Eigen::Vector3d::Zero();
    Eigen::Quaterniond grasp_quat_body = goal_quat;

    auto prodmp_template = makeSyntheticProDmpTemplate();
    PhaseSelector selector(robot_model, prodmp_template);

    PhaseSelector::Result result = selector.select(rotation, grasp_pos_body, grasp_quat_body, q0,
                                                     ee0_pos, ee0_quat);

    // omega=0: PhaseSelector must recognize the degenerate case and collapse
    // to a SINGLE evaluation at t_start=0 - scanning phase is meaningless
    // when the phase never changes.
    EXPECT_EQ(result.t_start, 0.0) << run;
    EXPECT_EQ(result.candidates_evaluated, 1) << run;
    EXPECT_FALSE(result.budget_exceeded) << run;

    // Reference: percentile-10 of w_trans over the tail (steady-state) of
    // the ALREADY-VALIDATED recorded Gazebo run for this goal.
    std::vector<double> tail_w_trans;
    const size_t tail_len = std::min<size_t>(200, manip_rows.size());
    for (size_t i = manip_rows.size() - tail_len; i < manip_rows.size(); ++i) {
        tail_w_trans.push_back(manip_rows[i].at("w_trans"));
    }
    std::sort(tail_w_trans.begin(), tail_w_trans.end());
    double reference_score = tail_w_trans[tail_w_trans.size() / 10];  // ~10th percentile

    std::cout << "\n[PhaseSelector degenerate omega=0] " << run
              << ": selector.score=" << result.score << " | recorded-run reference (~p10)="
              << reference_score << "\n";

    // NOT a tight correctness bound: our from-scratch kinematic Euler+DLS
    // simulation and the real recorded Gazebo run follow different transient
    // paths/timing to reach the same fixed point (different control-loop
    // instantiations of the SAME DLS law), so exact numeric agreement was
    // not independently verified before this code was written (Pinocchio is
    // not available on the machine that authored it - see file header NOTE).
    // This asserts both numbers land in the same manipulability regime
    // (same order of magnitude, both well inside [0, 0.164] theoretical
    // Panda range) rather than a specific percentage - TIGHTEN this once run
    // inside the Docker build environment and the true gap is known.
    EXPECT_GT(result.score, 0.0) << run;
    EXPECT_LT(result.score, 0.2) << run;
    EXPECT_NEAR(result.score, reference_score, 0.5 * reference_score + 0.02)
        << run << ": selector score should be in the same manipulability regime as the "
        << "recorded run - if this fails, tighten/replace this bound with the real measured "
        << "gap (see file header NOTE, this could not be executed/calibrated in this session).";
}

INSTANTIATE_TEST_SUITE_P(FiveKnownGoals, PhaseSelectorDegenerateTest,
                          ::testing::Values("reach_task_goal_1_prodmp",
                                             "reach_task_goal_2_prodmp",
                                             "reach_task_goal_3_prodmp",
                                             "reach_task_goal_4_prodmp",
                                             "reach_task_goal_5_prodmp"));

// ---------------------------------------------------------------------
// Test 4 (review point 1): budget/timeout is respected and fails loud.
// ---------------------------------------------------------------------
TEST(PhaseSelectorTest, RespectsWallTimeBudgetAndFailsLoud) {
    auto robot_model = loadPandaRobotModel();
    ASSERT_NE(robot_model, nullptr);

    RobotModel::JointVector q0;
    q0 << 0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854;
    robot_model->update(q0, RobotModel::JointVector::Zero());
    Eigen::Vector3d ee0_pos = robot_model->eePosition();
    Eigen::Quaterniond ee0_quat = robot_model->eeOrientation();

    SatelliteRotationModel rotation;
    rotation.center = ee0_pos + Eigen::Vector3d(0.2, 0.0, 0.0);
    rotation.axis = Eigen::Vector3d::UnitZ();
    rotation.phase0 = 0.0;
    rotation.omega = 0.1;  // non-degenerate: forces a real coarse+fine scan

    auto prodmp_template = makeSyntheticProDmpTemplate();

    PhaseSelector::Params params;
    params.coarse_candidates = 24;
    params.fine_candidates = 24;
    params.max_wall_time_sec = -1.0;  // impossible budget: must abort immediately, fail loud

    PhaseSelector selector(robot_model, prodmp_template, params);
    PhaseSelector::Result result =
        selector.select(rotation, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), q0,
                         ee0_pos, ee0_quat);

    EXPECT_TRUE(result.budget_exceeded);
    EXPECT_FALSE(result.message.empty()) << "budget_exceeded must always carry an explanatory message";
}

// ---------------------------------------------------------------------
// Test 5: best_score_lower_bound is populated and matches formula
// ---------------------------------------------------------------------
TEST(PhaseSelectorTest, ComputesRealisticScoreLowerBoundFromUncertainty) {
    auto robot_model = loadPandaRobotModel();
    ASSERT_NE(robot_model, nullptr);

    RobotModel::JointVector q0;
    q0 << 0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854;
    robot_model->update(q0, RobotModel::JointVector::Zero());
    Eigen::Vector3d ee0_pos = robot_model->eePosition();
    Eigen::Quaterniond ee0_quat = robot_model->eeOrientation();

    SatelliteRotationModel rotation;
    rotation.center = ee0_pos + Eigen::Vector3d(0.1, 0.0, 0.0);
    rotation.axis = Eigen::Vector3d::UnitZ();
    rotation.phase0 = 0.0;
    rotation.omega = 0.0;  // static evaluation

    auto prodmp_template = makeSyntheticProDmpTemplate();

    // Default params (known_branch_uncertainty = 0.24)
    PhaseSelector selector_default(robot_model, prodmp_template);
    auto res_default = selector_default.select(rotation, Eigen::Vector3d::Zero(),
                                               Eigen::Quaterniond::Identity(), q0, ee0_pos, ee0_quat);

    EXPECT_GT(res_default.score, 0.0);
    EXPECT_NEAR(res_default.best_score_lower_bound, res_default.score * (1.0 - 0.24), 1e-6);

    // Custom uncertainty param (e.g. 0.30)
    PhaseSelector::Params params_custom;
    params_custom.known_branch_uncertainty = 0.30;
    PhaseSelector selector_custom(robot_model, prodmp_template, params_custom);
    auto res_custom = selector_custom.select(rotation, Eigen::Vector3d::Zero(),
                                             Eigen::Quaterniond::Identity(), q0, ee0_pos, ee0_quat);

    EXPECT_NEAR(res_custom.best_score_lower_bound, res_custom.score * (1.0 - 0.30), 1e-6);
}

// ---------------------------------------------------------------------
// Test 6: Indecision margin behavior with recalibrated 25% threshold
// ---------------------------------------------------------------------
TEST(PhaseSelectorTest, FlagsIndecisionMarginWhenCandidatesAreWithinThreshold) {
    auto robot_model = loadPandaRobotModel();
    ASSERT_NE(robot_model, nullptr);

    RobotModel::JointVector q0;
    q0 << 0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854;
    robot_model->update(q0, RobotModel::JointVector::Zero());
    Eigen::Vector3d ee0_pos = robot_model->eePosition();
    Eigen::Quaterniond ee0_quat = robot_model->eeOrientation();

    SatelliteRotationModel rotation;
    rotation.center = ee0_pos + Eigen::Vector3d(0.1, 0.0, 0.0);
    rotation.axis = Eigen::Vector3d::UnitZ();
    rotation.phase0 = 0.0;
    rotation.omega = 0.05;  // rotating target

    auto prodmp_template = makeSyntheticProDmpTemplate();

    PhaseSelector::Params params;
    params.coarse_candidates = 6;
    params.fine_candidates = 6;
    params.indecision_margin_fraction = 0.25;

    PhaseSelector selector(robot_model, prodmp_template, params);
    auto res = selector.select(rotation, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                               q0, ee0_pos, ee0_quat);

    EXPECT_GT(res.candidates_evaluated, 1);
    EXPECT_GT(res.score, 0.0);
    EXPECT_GT(res.runner_up_score, 0.0);
    EXPECT_NEAR(res.score_margin, (res.score - res.runner_up_score) / res.score, 1e-6);
    if (res.score_margin < 0.25) {
        EXPECT_TRUE(res.indecisive_margin);
    } else {
        EXPECT_FALSE(res.indecisive_margin);
    }
}

