// Tests for referenceWTrans()/selectGrasp() with a synthetic ProDMP (mechanism only), small scan
// (60 deg step, 1 eval per point) so they stay short, plus an empirical selection probe with the
// production template. Need Pinocchio, Coal and $HOME/thesis_ws/fer_flat_effort.urdf.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "probe_gate.hpp"
#include "test_fixtures.hpp"
#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/demo_csv_io.hpp"
#include "haptic_dmp_learning/core/demo_params.hpp"
#include "haptic_dmp_learning/core/grasp_cost.hpp"
#include "haptic_dmp_learning/core/math_utils.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "haptic_dmp_learning/core/types.hpp"
#include "satellite_grasp_planner/core/grasp_selection.hpp"
#include "satellite_grasp_planner/core/selection_inputs.hpp"

using namespace satellite_grasp_planner::core;
using haptic_dmp_learning::core::CubeSatelliteModel;
using haptic_dmp_learning::core::GraspPointId;
using haptic_dmp_learning::core::ProDMP;
using haptic_dmp_learning::core::degToRad;
using haptic_dmp_learning::core::radToDeg;
using RobotModel = franka_cartesian_control::core::RobotModel;
using satellite_grasp_planner::test_fixtures::loadPandaRobotModel;
using satellite_grasp_planner::test_fixtures::makeModelB;
using satellite_grasp_planner::test_fixtures::makeSyntheticProDmpTemplate;

namespace {

/// Satellite placed so that the approach axis z_tcp = -n equals the tool z-axis at q0 (axis = tool x,
/// n = -tool z) and the collar center sits 0.1 m ahead of the start position in world X, like the
/// "nearby target" case of test_kinematic_feasibility: the theta = 0 grasp targets are then close to
/// the start with the same approach direction, so reachability from readyPose is plausible.
CubeSatelliteModel makeNearbyModel(RobotModel& robot, const RobotModel::JointVector& q0) {
    robot.update(q0, RobotModel::JointVector::Zero());
    const Eigen::Vector3d p_ee = robot.eePosition();
    const Eigen::Matrix3d R = robot.eeOrientation().toRotationMatrix();
    CubeSatelliteModel::Params p;
    p.axis_world = R.col(0);
    p.face_normal_body = -R.col(2);
    p.center_world = p_ee + Eigen::Vector3d(0.1, 0.0, 0.0) -
                     (p.cube_side_m / 2.0 + p.standoff_m) * p.face_normal_body;
    return CubeSatelliteModel(p);
}

/// Position tolerance for the synthetic-template tests. The 2 s synthetic template has a final
/// tracking error of 20-58 mm; the production threshold (20 mm, SelectionParams default) is
/// calibrated on the production template, where reached candidates sit at 8-19 mm. Here we only
/// check the consistency of the selection, not reachability.
constexpr double kSyntheticPosTolMm = 100.0;

/// Small scan setup on the synthetic template: theta in {-60, 0, +60} deg (12 rows).
SelectionParams makeSmallParams(const CubeSatelliteModel& m, const RobotModel::JointVector& q0,
                                RobotModel& robot, double w_trans_demo) {
    robot.update(q0, RobotModel::JointVector::Zero());
    SelectionParams sp;
    sp.scan.theta_center_rad = 0.0;
    sp.scan.half_window_rad = degToRad(60.0);
    sp.scan.step_rad = degToRad(60.0);
    sp.scan.omega_rad_s = -degToRad(2.0);
    sp.scan.tau_contact_s = 1.0;
    sp.scan.p0 = robot.eePosition();
    sp.scan.delta_g_demo = Eigen::Vector3d(0.1, 0.0, 0.0);
    sp.scan.e_v_ref = haptic_dmp_learning::core::meanSurfaceSpeedSquared(m, sp.scan.omega_rad_s);
    sp.scan.e_g_ref = sp.scan.delta_g_demo.squaredNorm();
    sp.w_trans_demo = w_trans_demo;
    sp.max_rows = 40;
    sp.pos_tol_mm = kSyntheticPosTolMm;
    return sp;
}

}  // namespace

TEST(GraspSelectionTest, ReferenceWTransFinitePositive) {
    auto robot = loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    const double w = referenceWTrans(makeSyntheticProDmpTemplate(), robot, RobotModel::readyPose(),
                                     Eigen::Vector3d(0.1, 0.0, 0.0));
    EXPECT_TRUE(std::isfinite(w));
    EXPECT_GT(w, 0.0);
}

TEST(GraspSelectionTest, SelectionInvariants) {
    auto robot = loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    const ProDMP tmpl = makeSyntheticProDmpTemplate();
    const RobotModel::JointVector q0 = RobotModel::readyPose();
    const CubeSatelliteModel m = makeNearbyModel(*robot, q0);
    const double w_ref = referenceWTrans(tmpl, robot, q0, Eigen::Vector3d(0.1, 0.0, 0.0));
    const SelectionParams sp = makeSmallParams(m, q0, *robot, w_ref);
    const int n_rows = static_cast<int>(scanCandidates(m, tmpl, sp.scan).size());
    ASSERT_EQ(n_rows, 12);

    const GraspSelection sel = selectGrasp(m, tmpl, robot, q0, sp);
    std::cout << "[selection-test] found=" << sel.found << " rows_evaluated=" << sel.rows_evaluated
              << " skipped=" << sel.rows_skipped_by_bound << " total_rollouts=" << sel.total_rollouts
              << std::endl;
    // Diagnostics of why each evaluated row fails; only with GRASP_TEST_VERBOSE set. Printed BEFORE
    // the ASSERT so it is visible when found == false. Each entry is the variant retained by
    // selectGrasp (least joint-limit violation if none was feasible).
    if (std::getenv("GRASP_TEST_VERBOSE")) {
        const char* names[4] = {"kP0", "kP90", "kP180", "kP270"};
        int n_kin_fail = 0, n_not_reached = 0, n_coll_fail = 0;
        for (const auto& c : sel.evaluated) {
            std::cout << "[selection-test-diag] " << names[static_cast<int>(c.row.k)]
                      << " theta=" << radToDeg(c.row.theta_rad) << " psi=" << radToDeg(c.psi_rad)
                      << " kin_feasible=" << c.eval.kin_feasible << " reached=" << c.eval.reached
                      << " pos_err_final_mm=" << c.eval.pos_err_final_mm
                      << " max_joint_violation_rad=" << c.eval.max_joint_violation_rad
                      << " collision_feasible=" << c.eval.collision_feasible << std::endl;
            if (!c.eval.kin_feasible) ++n_kin_fail;
            if (!c.eval.reached) ++n_not_reached;
            if (!c.eval.collision_feasible) ++n_coll_fail;
        }
        std::cout << "[selection-test-diag] summary over " << sel.evaluated.size()
                  << " evaluated rows: kinematics failed=" << n_kin_fail
                  << " not reached=" << n_not_reached << " collision failed=" << n_coll_fail
                  << " (criteria overlap: a row can fail several)" << std::endl;
    }
    ASSERT_TRUE(sel.found) << "no feasible candidate on the nearby synthetic setup";

    int total = 0;
    for (const auto& c : sel.evaluated) {
        // 2 rollouts (psi in {0, pi}) or 6 (refinement added).
        EXPECT_TRUE(c.rollouts_used == 2 || c.rollouts_used == 6) << c.rollouts_used;
        const bool psi_basic =
            std::abs(c.psi_rad) < 1e-12 || std::abs(c.psi_rad - haptic_dmp_learning::core::kPi) < 1e-12;
        if (c.eval.feasible && psi_basic) {
            EXPECT_EQ(c.rollouts_used, 2);
        }
        if (c.eval.feasible) {
            EXPECT_LE(sel.best.cost.total, c.cost.total);
        }
        total += c.rollouts_used;
    }
    EXPECT_EQ(sel.total_rollouts, total);
    EXPECT_EQ(static_cast<int>(sel.evaluated.size()), sel.rows_evaluated);
    EXPECT_TRUE(sel.best.eval.feasible);
    EXPECT_EQ(sel.goal_position, sel.best.row.goal_param);
    EXPECT_NEAR(sel.goal_orientation.norm(), 1.0, 1e-12);
    if (!sel.truncated) {
        EXPECT_EQ(sel.rows_evaluated + sel.rows_skipped_by_bound, n_rows);
    }
}

// A huge bound never stops the search (all rows evaluated); a tiny one stops as soon as it can.
TEST(GraspSelectionTest, LargerBoundEvaluatesMoreRowsAndNeverWorse) {
    auto robot = loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    const ProDMP tmpl = makeSyntheticProDmpTemplate();
    const RobotModel::JointVector q0 = RobotModel::readyPose();
    const CubeSatelliteModel m = makeNearbyModel(*robot, q0);
    const double w_ref = referenceWTrans(tmpl, robot, q0, Eigen::Vector3d(0.1, 0.0, 0.0));
    SelectionParams sp = makeSmallParams(m, q0, *robot, w_ref);

    sp.w_hat_upper_bound = 1e6;
    const GraspSelection big = selectGrasp(m, tmpl, robot, q0, sp);
    sp.w_hat_upper_bound = 1e-6;
    const GraspSelection small = selectGrasp(m, tmpl, robot, q0, sp);
    ASSERT_TRUE(big.found);
    ASSERT_TRUE(small.found);
    EXPECT_GT(big.rows_evaluated, small.rows_evaluated);
    EXPECT_LE(big.best.cost.total, small.best.cost.total);
}

// With the production tolerance (20 mm) a target ~3 m from the base cannot be reached: the reached
// filter must reject it, so the row is infeasible and nothing is found. One row only (max_rows = 1)
// to keep the (long-distance) rollout cheap.
TEST(GraspSelectionTest, ReachedFilterRejectsUnreachedCandidates) {
    auto robot = loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    const ProDMP tmpl = makeSyntheticProDmpTemplate();
    const RobotModel::JointVector q0 = RobotModel::readyPose();
    CubeSatelliteModel::Params p;
    p.center_world = Eigen::Vector3d(3.0, 3.0, 1.0);  // metres away, as in test_satellite_collision
    const CubeSatelliteModel m(p);
    SelectionParams sp = makeSmallParams(m, q0, *robot, 1.0);
    sp.pos_tol_mm = 20.0;  // production value
    sp.max_rows = 1;

    const GraspSelection sel = selectGrasp(m, tmpl, robot, q0, sp);
    ASSERT_EQ(sel.evaluated.size(), 1u);
    const EvaluatedCandidate& c = sel.evaluated.front();
    EXPECT_GT(c.eval.pos_err_final_mm, sp.pos_tol_mm);
    EXPECT_FALSE(c.eval.reached);
    EXPECT_FALSE(c.eval.feasible);
    EXPECT_FALSE(sel.found);
}

TEST(GraspSelectionTest, InvalidParamsThrow) {
    auto robot = loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    const ProDMP tmpl = makeSyntheticProDmpTemplate();
    const RobotModel::JointVector q0 = RobotModel::readyPose();
    const CubeSatelliteModel m = makeNearbyModel(*robot, q0);
    SelectionParams sp = makeSmallParams(m, q0, *robot, 1.0);

    SelectionParams bad = sp;
    bad.max_rows = 0;
    EXPECT_THROW(selectGrasp(m, tmpl, robot, q0, bad), std::invalid_argument);
    bad = sp;
    bad.w_hat_upper_bound = 0.0;
    EXPECT_THROW(selectGrasp(m, tmpl, robot, q0, bad), std::invalid_argument);
    bad = sp;
    bad.w_trans_demo = 0.0;
    EXPECT_THROW(selectGrasp(m, tmpl, robot, q0, bad), std::invalid_argument);
    bad = sp;
    bad.w_trans_demo = -1.0;
    EXPECT_THROW(selectGrasp(m, tmpl, robot, q0, bad), std::invalid_argument);
}

TEST(GraspSelectionTest, InvalidTimeBudgetThrowsZeroDoesNot) {
    auto robot = loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    const ProDMP tmpl = makeSyntheticProDmpTemplate();
    const RobotModel::JointVector q0 = RobotModel::readyPose();
    const CubeSatelliteModel m = makeNearbyModel(*robot, q0);
    SelectionParams sp = makeSmallParams(m, q0, *robot, 1.0);

    SelectionParams bad = sp;
    bad.time_budget_s = std::nan("");
    EXPECT_THROW(selectGrasp(m, tmpl, robot, q0, bad), std::invalid_argument);
    bad.time_budget_s = -1.0;
    EXPECT_THROW(selectGrasp(m, tmpl, robot, q0, bad), std::invalid_argument);
    bad.time_budget_s = std::numeric_limits<double>::infinity();
    EXPECT_THROW(selectGrasp(m, tmpl, robot, q0, bad), std::invalid_argument);

    SelectionParams zero = sp;
    zero.time_budget_s = 0.0;
    EXPECT_NO_THROW(selectGrasp(m, tmpl, robot, q0, zero));
}

// time_budget_s = nullopt (default) and a practically unlimited budget must behave the same, field by
// field, aside from elapsed_s (real wall time, never expected to be bit-identical between two calls).
// This is NOT a claim that either of them matches selectGrasp()'s behavior before the budget feature
// was added - only that an unlimited budget is equivalent to no budget at all.
TEST(GraspSelectionTest, TimeBudgetNulloptEqualsHugeBudget) {
    auto robot = loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    const ProDMP tmpl = makeSyntheticProDmpTemplate();
    const RobotModel::JointVector q0 = RobotModel::readyPose();
    const CubeSatelliteModel m = makeNearbyModel(*robot, q0);
    const double w_ref = referenceWTrans(tmpl, robot, q0, Eigen::Vector3d(0.1, 0.0, 0.0));
    SelectionParams sp = makeSmallParams(m, q0, *robot, w_ref);

    sp.time_budget_s = std::nullopt;
    const GraspSelection a = selectGrasp(m, tmpl, robot, q0, sp);
    sp.time_budget_s = 1e9;
    const GraspSelection b = selectGrasp(m, tmpl, robot, q0, sp);

    EXPECT_EQ(a.found, b.found);
    EXPECT_EQ(a.rows_evaluated, b.rows_evaluated);
    EXPECT_EQ(a.total_rollouts, b.total_rollouts);
    EXPECT_EQ(a.stopped_by_bound, b.stopped_by_bound);
    EXPECT_EQ(a.truncated, b.truncated);
    EXPECT_EQ(a.rows_skipped_by_bound, b.rows_skipped_by_bound);
    EXPECT_FALSE(a.budget_exhausted);
    EXPECT_FALSE(b.budget_exhausted);
    EXPECT_EQ(a.rows_skipped_by_budget, 0);
    EXPECT_EQ(b.rows_skipped_by_budget, 0);
    EXPECT_EQ(a.stop_reason, b.stop_reason);
    EXPECT_EQ(a.goal_position, b.goal_position);
    ASSERT_EQ(a.evaluated.size(), b.evaluated.size());
    for (std::size_t i = 0; i < a.evaluated.size(); ++i) {
        EXPECT_EQ(a.evaluated[i].rollouts_used, b.evaluated[i].rollouts_used);
        EXPECT_DOUBLE_EQ(a.evaluated[i].psi_rad, b.evaluated[i].psi_rad);
        EXPECT_DOUBLE_EQ(a.evaluated[i].cost.total, b.evaluated[i].cost.total);
        EXPECT_FALSE(a.evaluated[i].row_interrupted_by_budget);
        EXPECT_FALSE(b.evaluated[i].row_interrupted_by_budget);
    }
}

// A fake clock (advances by a fixed step per call, decoupled from evaluateCandidate()'s actual - small,
// synthetic-template - runtime) lets this test be deterministic on any machine. It deliberately does
// NOT assume how many clock() calls a single evaluateCandidate() attempt costs (an implementation
// detail): it first calibrates with an unlimited budget on the SAME kind of clock to learn the total
// "tick time" of the full search, then re-runs with half that budget on a fresh clock instance, which
// is guaranteed to cut the search short by construction, and only asserts inequalities.
TEST(GraspSelectionTest, TimeBudgetFakeClockCutsSearchShort) {
    auto robot = loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    const ProDMP tmpl = makeSyntheticProDmpTemplate();
    const RobotModel::JointVector q0 = RobotModel::readyPose();
    const CubeSatelliteModel m = makeNearbyModel(*robot, q0);
    const double w_ref = referenceWTrans(tmpl, robot, q0, Eigen::Vector3d(0.1, 0.0, 0.0));
    SelectionParams sp = makeSmallParams(m, q0, *robot, w_ref);
    sp.w_hat_upper_bound = 1e6;  // never stop by bound: every row is attempted (as in
                                 // LargerBoundEvaluatesMoreRowsAndNeverWorse's "big" case).

    auto makeFakeClock = []() {
        auto ticks = std::make_shared<int>(0);
        constexpr double kStepS = 1.0;
        return ClockFn([ticks]() { return kStepS * (*ticks)++; });
    };

    // A practically unlimited budget (not nullopt) so the timing code path still runs - clock() gets
    // called around every rollout, not just once at entry and once at exit - and full.elapsed_s
    // reflects the actual "tick time" of the whole search, proportional to its rollout count.
    sp.time_budget_s = 1e9;
    const GraspSelection full = selectGrasp(m, tmpl, robot, q0, sp, makeFakeClock());
    ASSERT_TRUE(full.found);
    ASSERT_GE(full.total_rollouts, 4) << "need enough rollouts for a meaningful halfway cutoff";

    sp.time_budget_s = full.elapsed_s / 2.0;
    const GraspSelection cut = selectGrasp(m, tmpl, robot, q0, sp, makeFakeClock());

    EXPECT_TRUE(cut.budget_exhausted);
    EXPECT_EQ(cut.stop_reason, GraspSelection::StopReason::kTimeBudgetExhausted);
    ASSERT_FALSE(cut.evaluated.empty());
    EXPECT_GE(cut.total_rollouts, 1);
    EXPECT_LT(cut.total_rollouts, full.total_rollouts);
    EXPECT_LE(cut.rows_evaluated, full.rows_evaluated);
    EXPECT_GT(cut.rows_skipped_by_budget, 0);
    int rollouts_sum = 0;
    for (const auto& c : cut.evaluated) rollouts_sum += c.rollouts_used;
    EXPECT_EQ(rollouts_sum, cut.total_rollouts);
    // Only the LAST pushed row can be interrupted mid-way; every earlier one ran to completion.
    for (std::size_t i = 0; i + 1 < cut.evaluated.size(); ++i) {
        EXPECT_FALSE(cut.evaluated[i].row_interrupted_by_budget);
    }
}

// EMPIRICAL probe, NO asserts on values: full selection on configuration B with the production
// template. Expected duration 5-10 minutes.
TEST(SatelliteSelectionProbe, PrintsSelection) {
    SKIP_UNLESS_PROBES_ENABLED();
    SKIP_UNLESS_PRODUCTION_WEIGHTS(weights_path);
    auto robot = loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    const ProDMP tmpl = haptic_dmp_learning::core::prodmp_io::loadProDmpFromYaml(weights_path);
    const Eigen::Vector3d delta_g_demo = tmpl.demoDisplacement();  // before any setInitialConditions

    const CubeSatelliteModel m = makeModelB();
    const RobotModel::JointVector q0 = RobotModel::readyPose();
    robot->update(q0, RobotModel::JointVector::Zero());
    const Eigen::Vector3d p0 = robot->eePosition();
    const Eigen::Vector3d base = robot->framePose("fer_link0").position;

    SelectionParams sp;
    sp.scan.theta_center_rad = windowCenterTheta(m, base);
    sp.scan.step_rad = degToRad(10.0);
    sp.scan.omega_rad_s = -degToRad(2.0);
    sp.scan.tau_contact_s = 0.0;
    sp.scan.p0 = p0;
    sp.scan.delta_g_demo = delta_g_demo;
    sp.scan.contact_to_end_offset = Eigen::Vector3d::Zero();
    sp.scan.e_v_ref = haptic_dmp_learning::core::meanSurfaceSpeedSquared(m, sp.scan.omega_rad_s);
    sp.scan.e_g_ref = delta_g_demo.squaredNorm();
    sp.w_trans_demo = referenceWTrans(tmpl, robot, q0, delta_g_demo);
    sp.max_rows = 40;
    std::cout << "[sel-ref] w_trans_demo=" << sp.w_trans_demo << std::endl;

    const GraspSelection sel = selectGrasp(m, tmpl, robot, q0, sp);
    const char* names[4] = {"kP0", "kP90", "kP180", "kP270"};
    auto line = [&](const char* tag, const EvaluatedCandidate& c) {
        std::cout << tag << " " << names[static_cast<int>(c.row.k)] << " theta="
                  << radToDeg(c.row.theta_rad) << " psi=" << radToDeg(c.psi_rad)
                  << " feasible=" << c.eval.feasible << " e_v_hat=" << c.cost.e_v_hat
                  << " e_g_hat=" << c.cost.e_g_hat << " w_hat=" << c.cost.w_hat
                  << " p_psi=" << c.cost.p_psi << " total=" << c.cost.total
                  << " max_joint_violation_rad=" << c.eval.max_joint_violation_rad
                  << " pos_err_final_mm=" << c.eval.pos_err_final_mm
                  << " min_distance_arm_m=" << c.eval.min_distance_arm_m
                  << " min_distance_cube_m=" << c.eval.min_distance_cube_m
                  << " rollouts_used=" << c.rollouts_used << std::endl;
    };
    for (const auto& c : sel.evaluated) line("[sel]", c);
    if (sel.found) {
        line("[sel-best]", sel.best);
    } else {
        std::cout << "[sel-best] none found" << std::endl;
    }
    std::cout << "[sel-time] total_rollouts=" << sel.total_rollouts << " elapsed=" << sel.elapsed_s
              << " s" << std::endl;
    std::cout << "[sel-stop] stopped_by_bound=" << sel.stopped_by_bound << " truncated=" << sel.truncated
              << " rows_evaluated=" << sel.rows_evaluated
              << " rows_skipped_by_bound=" << sel.rows_skipped_by_bound
              << " max_w_hat_seen=" << sel.max_w_hat_seen << " bound_violated=" << sel.bound_violated
              << std::endl;

    // Expected total 1.3255 for (kP270, theta=-60, psi=0), from the feasibility map.
    const EvaluatedCandidate* target = nullptr;
    for (const auto& c : sel.evaluated) {
        if (c.row.k == GraspPointId::kP270 && std::abs(c.row.theta_rad - degToRad(-60.0)) < 1e-6) {
            target = &c;
        }
    }
    if (!target) {
        std::cout << "[sel-check] (kP270, theta=-60) is NOT among the evaluated rows" << std::endl;
    } else if (std::abs(target->psi_rad) > 1e-12) {
        std::cout << "[sel-check] (kP270, theta=-60) evaluated, but the retained variant has psi="
                  << radToDeg(target->psi_rad) << " deg (psi=0 variant not kept); its total="
                  << target->cost.total << " feasible=" << target->eval.feasible << std::endl;
    } else {
        const double expected = 1.3255;
        std::cout << "[sel-check] (kP270, theta=-60, psi=0) total=" << target->cost.total
                  << " expected=" << expected
                  << " rel_diff=" << (target->cost.total - expected) / expected
                  << " feasible=" << target->eval.feasible << std::endl;
    }
}

// EMPIRICAL probe, NO asserts on values: same selection as SatelliteSelectionProbe, but the inputs come
// from buildSelectionInputs() reading a demo_params file generated (like fit_prodmp does) next to a COPY
// of the production weights in /tmp/sel_check. No satellite options: satellite_at_demo is null.
// Expected duration 5-10 minutes.
TEST(SatelliteSelectionFromDemoParamsProbe, PrintsSelection) {
    SKIP_UNLESS_PROBES_ENABLED();
    const char* home = std::getenv("HOME");
    const std::string root = std::string(home ? home : "/root") + "/thesis_ws/";
    const std::string csv = root + "reach_task_baseline.csv";
    const std::string prod_weights =
        root + "runs/20260914_150515_fit_reach_task_baseline_prodmp_n80_lam1e-10_w0.05/weights.yaml";
    if (!std::ifstream(csv).good() || !std::ifstream(prod_weights).good()) {
        GTEST_SKIP() << "production demo/weights not reachable: " << csv << " / " << prod_weights;
    }
    namespace dp = haptic_dmp_learning::core::demo_params;

    // Copy of the production weights + demo_params generated next to it.
    const std::string dir = "/tmp/sel_check/";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    const std::string weights_copy = dir + "weights.yaml";
    {
        std::ifstream in(prod_weights, std::ios::binary);
        std::ofstream out(weights_copy, std::ios::binary);
        out << in.rdbuf();
    }
    const std::vector<haptic_dmp_learning::core::Sample> demo = haptic_dmp_learning::core::demo_csv_io::readDemoCsv(csv);
    dp::FitInfo fit;  // production fit configuration (prodmp_features.yaml)
    fit.num_basis = 80;
    fit.ridge_lambda = 1e-10;
    fit.position_filter_window_s = 0.05;
    fit.fix_goal_to_demo_endpoint = true;
    std::ostringstream wlog;
    dp::DemoParams written;
    const std::string params_path =
        dp::writeForWeights(csv, demo, fit, weights_copy, std::nullopt, wlog, &written);
    std::cout << "[sel2-setup] " << wlog.str() << "[sel2-setup] demo_params hash=" << written.weights_sha256
              << " production weights hash=" << dp::sha256FileHex(prod_weights)
              << (written.weights_sha256 == dp::sha256FileHex(prod_weights) ? " (MATCH)" : " (DIFFERENT)")
              << std::endl;

    auto probe_robot = loadPandaRobotModel();
    ASSERT_NE(probe_robot, nullptr);
    probe_robot->update(RobotModel::readyPose(), RobotModel::JointVector::Zero());
    const Eigen::Vector3d base = probe_robot->framePose("fer_link0").position;

    SatelliteSnapshot snap;
    snap.center = Eigen::Vector3d(0.75, 0.0, 0.35);
    snap.axis = Eigen::Vector3d(0.0, 0.0, 1.0);
    snap.omega_rad_s = -degToRad(2.0);
    snap.theta_rad = 0.0;
    snap.t_s = 0.0;
    CubeSatelliteModel::Params geometry;
    geometry.face_normal_body = Eigen::Vector3d(-1.0, 0.0, 0.0);

    const char* home2 = std::getenv("HOME");
    const std::string urdf = std::string(home2 ? home2 : "/root") + "/thesis_ws/fer_flat_effort.urdf";
    SelectionInputs in = buildSelectionInputs(params_path, urdf, snap, geometry, base);
    for (const auto& l : in.provenance) std::cout << "[sel2-inputs] " << l << std::endl;
    std::cout << "[sel2-inputs] w_trans_demo=" << in.params.w_trans_demo
              << " w_trans_demo_is_proxy=" << in.w_trans_demo_is_proxy
              << " contact_assumed_at_end=" << in.contact_assumed_at_end << std::endl;

    // Same search settings as SatelliteSelectionProbe.
    in.params.scan.step_rad = degToRad(10.0);
    in.params.w_hat_upper_bound = 2.5;
    in.params.max_rows = 40;

    const CubeSatelliteModel model(in.cube_params);
    const GraspSelection sel = selectGrasp(model, in.prodmp_template, in.robot, in.q0, in.params);

    const char* names[4] = {"kP0", "kP90", "kP180", "kP270"};
    auto line = [&](const char* tag, const EvaluatedCandidate& c) {
        std::cout << tag << " " << names[static_cast<int>(c.row.k)] << " theta="
                  << radToDeg(c.row.theta_rad) << " psi=" << radToDeg(c.psi_rad)
                  << " feasible=" << c.eval.feasible << " e_v_hat=" << c.cost.e_v_hat
                  << " e_g_hat=" << c.cost.e_g_hat << " w_hat=" << c.cost.w_hat
                  << " p_psi=" << c.cost.p_psi << " total=" << c.cost.total
                  << " max_joint_violation_rad=" << c.eval.max_joint_violation_rad
                  << " pos_err_final_mm=" << c.eval.pos_err_final_mm
                  << " min_distance_arm_m=" << c.eval.min_distance_arm_m
                  << " min_distance_cube_m=" << c.eval.min_distance_cube_m
                  << " rollouts_used=" << c.rollouts_used << std::endl;
    };
    for (const auto& c : sel.evaluated) line("[sel2]", c);
    if (sel.found) {
        line("[sel2-best]", sel.best);
    } else {
        std::cout << "[sel2-best] none found" << std::endl;
    }
    std::cout << "[sel2-stop] stopped_by_bound=" << sel.stopped_by_bound << " truncated=" << sel.truncated
              << " rows_evaluated=" << sel.rows_evaluated
              << " rows_skipped_by_bound=" << sel.rows_skipped_by_bound
              << " max_w_hat_seen=" << sel.max_w_hat_seen << " bound_violated=" << sel.bound_violated
              << std::endl;
    std::cout << "[sel2-time] total_rollouts=" << sel.total_rollouts << " elapsed=" << sel.elapsed_s << " s"
              << std::endl;

    // Compare the best with the previous probe's: (kP270, theta=-60, psi=0), total 1.32548,
    // w_trans_demo 0.0912627 there.
    const double expected_total = 1.32548, old_w_ref = 0.0912627;
    const double w_ref_new = in.params.w_trans_demo;
    std::cout << "[sel2-check] w_trans_demo here=" << w_ref_new << " previous probe=" << old_w_ref
              << " |delta_g| here=" << in.params.scan.delta_g_demo.norm() << " (template demoDisplacement "
              << in.prodmp_template.demoDisplacement().norm() << ", used by the previous probe)" << std::endl;
    if (!sel.found) {
        std::cout << "[sel2-check] no best found: nothing to compare" << std::endl;
    } else {
        const EvaluatedCandidate& b = sel.best;
        const bool same = b.row.k == GraspPointId::kP270 && std::abs(b.row.theta_rad - degToRad(-60.0)) < 1e-6 &&
                          std::abs(b.psi_rad) < 1e-12;
        std::cout << "[sel2-check] best here: " << names[static_cast<int>(b.row.k)] << " theta="
                  << radToDeg(b.row.theta_rad) << " psi=" << radToDeg(b.psi_rad) << " total=" << b.cost.total
                  << (same ? " (same candidate as the previous probe)" : " (DIFFERENT candidate from the previous probe)")
                  << " expected_total=" << expected_total
                  << " rel_diff=" << (b.cost.total - expected_total) / expected_total << std::endl;
        // How much of the total difference is explained by w_trans_demo: recompute the total with the old
        // reference, w_hat_old = w_trans_final / old_w_ref, using this candidate's own w_trans_final.
        const double w_trans_final = b.cost.w_hat * w_ref_new;
        const double w_hat_old = w_trans_final / old_w_ref;
        const double total_with_old_ref = b.cost.total + in.params.scan.weights.w_m * (b.cost.w_hat - w_hat_old);
        std::cout << "[sel2-check] w_trans_final=" << w_trans_final << " w_hat here=" << b.cost.w_hat
                  << " w_hat with previous reference=" << w_hat_old << " -> total recomputed with the previous "
                  << "w_trans_demo=" << total_with_old_ref << " (rel_diff vs expected "
                  << (total_with_old_ref - expected_total) / expected_total << ")" << std::endl;
    }
}

// EMPIRICAL probe, NO asserts on values: same setup as SatelliteSelectionFromDemoParamsProbe, but with
// a 30 s time budget on the real (steady_clock-backed) clock, to see the budget feature's effect on the
// production template/geometry. Expected duration: up to ~30 s plus the last (uninterruptible) rollout.
TEST(SatelliteSelectionBudgetProbe, PrintsSelection) {
    SKIP_UNLESS_PROBES_ENABLED();
    const char* home = std::getenv("HOME");
    const std::string root = std::string(home ? home : "/root") + "/thesis_ws/";
    const std::string csv = root + "reach_task_baseline.csv";
    const std::string prod_weights =
        root + "runs/20260914_150515_fit_reach_task_baseline_prodmp_n80_lam1e-10_w0.05/weights.yaml";
    if (!std::ifstream(csv).good() || !std::ifstream(prod_weights).good()) {
        GTEST_SKIP() << "production demo/weights not reachable: " << csv << " / " << prod_weights;
    }
    namespace dp = haptic_dmp_learning::core::demo_params;

    const std::string dir = "/tmp/sel_budget_check/";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    const std::string weights_copy = dir + "weights.yaml";
    {
        std::ifstream in(prod_weights, std::ios::binary);
        std::ofstream out(weights_copy, std::ios::binary);
        out << in.rdbuf();
    }
    const std::vector<haptic_dmp_learning::core::Sample> demo = haptic_dmp_learning::core::demo_csv_io::readDemoCsv(csv);
    dp::FitInfo fit;  // production fit configuration (prodmp_features.yaml)
    fit.num_basis = 80;
    fit.ridge_lambda = 1e-10;
    fit.position_filter_window_s = 0.05;
    fit.fix_goal_to_demo_endpoint = true;
    std::ostringstream wlog;
    dp::DemoParams written;
    const std::string params_path =
        dp::writeForWeights(csv, demo, fit, weights_copy, std::nullopt, wlog, &written);

    auto probe_robot = loadPandaRobotModel();
    ASSERT_NE(probe_robot, nullptr);
    probe_robot->update(RobotModel::readyPose(), RobotModel::JointVector::Zero());
    const Eigen::Vector3d base = probe_robot->framePose("fer_link0").position;

    SatelliteSnapshot snap;
    snap.center = Eigen::Vector3d(0.75, 0.0, 0.35);
    snap.axis = Eigen::Vector3d(0.0, 0.0, 1.0);
    snap.omega_rad_s = -degToRad(2.0);
    snap.theta_rad = 0.0;
    snap.t_s = 0.0;
    CubeSatelliteModel::Params geometry;
    geometry.face_normal_body = Eigen::Vector3d(-1.0, 0.0, 0.0);

    const std::string urdf = root + "fer_flat_effort.urdf";
    SelectionInputs in = buildSelectionInputs(params_path, urdf, snap, geometry, base);

    // Same search settings as SatelliteSelectionProbe/SatelliteSelectionFromDemoParamsProbe, plus the
    // 30 s time budget under test. Real (default) clock: clock = nullptr.
    in.params.scan.step_rad = degToRad(10.0);
    in.params.w_hat_upper_bound = 2.5;
    in.params.max_rows = 40;
    in.params.time_budget_s = 30.0;

    const CubeSatelliteModel model(in.cube_params);
    const GraspSelection sel = selectGrasp(model, in.prodmp_template, in.robot, in.q0, in.params);

    const char* stop_reason_name = "?";
    switch (sel.stop_reason) {
        case GraspSelection::StopReason::kCompleted: stop_reason_name = "kCompleted"; break;
        case GraspSelection::StopReason::kBoundSatisfied: stop_reason_name = "kBoundSatisfied"; break;
        case GraspSelection::StopReason::kMaxRowsReached: stop_reason_name = "kMaxRowsReached"; break;
        case GraspSelection::StopReason::kTimeBudgetExhausted: stop_reason_name = "kTimeBudgetExhausted"; break;
    }
    std::cout << "[sel-budget] rows_evaluated=" << sel.rows_evaluated
              << " total_rollouts=" << sel.total_rollouts << " elapsed_s=" << sel.elapsed_s
              << " stop_reason=" << stop_reason_name << " budget_exhausted=" << sel.budget_exhausted
              << " found=" << sel.found << std::endl;
    if (!sel.evaluated.empty()) {
        std::cout << "[sel-budget] last row row_interrupted_by_budget="
                  << sel.evaluated.back().row_interrupted_by_budget << std::endl;
    } else {
        std::cout << "[sel-budget] no row was evaluated" << std::endl;
    }
}
