// Tests for referenceWTrans()/selectGrasp() with a synthetic ProDMP (mechanism only), small scan
// (60 deg step, 1 eval per point) so they stay short, plus an empirical selection probe with the
// production template. Need Pinocchio, Coal and $HOME/thesis_ws/fer_flat_effort.urdf.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/grasp_cost.hpp"
#include "haptic_dmp_learning/core/math_utils.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "haptic_dmp_learning/core/types.hpp"
#include "satellite_grasp_planner/core/grasp_selection.hpp"

using namespace satellite_grasp_planner::core;
using haptic_dmp_learning::core::CubeSatelliteModel;
using haptic_dmp_learning::core::GraspPointId;
using haptic_dmp_learning::core::ProDMP;
using haptic_dmp_learning::core::degToRad;
using haptic_dmp_learning::core::radToDeg;
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

/// 2 s straight 0.1 m demo, 8 basis functions.
ProDMP makeSyntheticProDmpTemplate() {
    std::vector<haptic_dmp_learning::core::Sample> demo;
    for (int i = 0; i <= 50; ++i) {
        haptic_dmp_learning::core::Sample s;
        s.t = 2.0 * i / 50.0;
        s.position = Eigen::Vector3d(0.1 * s.t / 2.0, 0.0, 0.0);
        demo.push_back(s);
    }
    ProDMP p(/*num_basis=*/8);
    p.learnFromDemonstration(demo);
    return p;
}

CubeSatelliteModel makeModelB() {
    CubeSatelliteModel::Params p;
    p.center_world = Eigen::Vector3d(0.75, 0.0, 0.35);
    p.axis_world = Eigen::Vector3d(0.0, 0.0, 1.0);
    p.face_normal_body = Eigen::Vector3d(-1.0, 0.0, 0.0);
    return CubeSatelliteModel(p);
}

std::string productionWeightsPath() {
    const char* home = std::getenv("HOME");
    const char* env_w = std::getenv("GRASP_PROBE_WEIGHTS");
    return env_w ? std::string(env_w)
                 : std::string(home ? home : "/root") +
                       "/thesis_ws/runs/20260914_150515_fit_reach_task_baseline_prodmp_n80_lam1e-10_w0.05/"
                       "weights.yaml";
}

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
        if (c.eval.feasible && psi_basic) EXPECT_EQ(c.rollouts_used, 2);
        if (c.eval.feasible) EXPECT_LE(sel.best.cost.total, c.cost.total);
        total += c.rollouts_used;
    }
    EXPECT_EQ(sel.total_rollouts, total);
    EXPECT_EQ(static_cast<int>(sel.evaluated.size()), sel.rows_evaluated);
    EXPECT_TRUE(sel.best.eval.feasible);
    EXPECT_EQ(sel.goal_position, sel.best.row.goal_param);
    EXPECT_NEAR(sel.goal_orientation.norm(), 1.0, 1e-12);
    if (!sel.truncated) EXPECT_EQ(sel.rows_evaluated + sel.rows_skipped_by_bound, n_rows);
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

// EMPIRICAL probe, NO asserts on values: full selection on configuration B with the production
// template. Expected duration 5-10 minutes.
TEST(SatelliteSelectionProbe, PrintsSelection) {
    const std::string weights_path = productionWeightsPath();
    if (!std::ifstream(weights_path).good()) {
        GTEST_SKIP() << "Production weights.yaml not reachable: " << weights_path
                     << " (set GRASP_PROBE_WEIGHTS)";
    }
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
