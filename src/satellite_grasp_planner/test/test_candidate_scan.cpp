// Pure tests for the candidate scan (no IK / RobotModel / collision). A synthetic ProDMP is used:
// they check the mechanism, not a result. See core/candidate_scan.hpp.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "probe_gate.hpp"
#include "test_fixtures.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/grasp_cost.hpp"
#include "haptic_dmp_learning/core/math_utils.hpp"
#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "haptic_dmp_learning/core/types.hpp"
#include "satellite_grasp_planner/core/candidate_scan.hpp"

using namespace satellite_grasp_planner::core;
using haptic_dmp_learning::core::CubeSatelliteModel;
using haptic_dmp_learning::core::GraspPointId;
using haptic_dmp_learning::core::ProDMP;
using haptic_dmp_learning::core::degToRad;
using haptic_dmp_learning::core::radToDeg;
using RobotModel = franka_cartesian_control::core::RobotModel;
using satellite_grasp_planner::test_fixtures::makeModelBParams;
using satellite_grasp_planner::test_fixtures::makeSyntheticProDmpTemplate;
using satellite_grasp_planner::test_fixtures::resetRobotToReady;

namespace {

ScanParams makeScan(const CubeSatelliteModel& m) {
    ScanParams sp;
    sp.theta_center_rad = 0.3;
    sp.omega_rad_s = -degToRad(2.0);
    sp.tau_contact_s = 1.0;
    sp.p0 = Eigen::Vector3d(0.2, -0.1, 0.3);
    sp.delta_g_demo = Eigen::Vector3d(0.1, 0.0, 0.0);
    sp.e_v_ref = haptic_dmp_learning::core::meanSurfaceSpeedSquared(m, sp.omega_rad_s);
    sp.e_g_ref = 0.01;
    return sp;
}

}  // namespace

TEST(CandidateScanWindow, CenterThetaKnownCases) {
    {
        auto p = makeModelBParams();
        EXPECT_NEAR(windowCenterTheta(CubeSatelliteModel(p), Eigen::Vector3d::Zero()), 0.0, 1e-12);
    }
    {
        CubeSatelliteModel::Params p;
        p.center_world = Eigen::Vector3d(0.3928, -0.165, 0.1188);
        p.axis_world = Eigen::Vector3d(0.0, 1.0, 0.0);
        p.face_normal_body = Eigen::Vector3d(0.0, 0.0, 1.0);
        EXPECT_NEAR(radToDeg(windowCenterTheta(CubeSatelliteModel(p), Eigen::Vector3d::Zero())),
                    -106.83, 0.01);
    }
    // Base on the spin axis: undefined.
    CubeSatelliteModel::Params p;
    const CubeSatelliteModel m(p);
    EXPECT_THROW(windowCenterTheta(m, Eigen::Vector3d(0.0, 0.0, 5.0)), std::invalid_argument);
}

TEST(CandidateScanRows, CountAndThetaRange) {
    const CubeSatelliteModel m{CubeSatelliteModel::Params{}};
    const ProDMP tmpl = makeSyntheticProDmpTemplate();
    const ScanParams sp = makeScan(m);
    const auto rows = scanCandidates(m, tmpl, sp);
    ASSERT_EQ(rows.size(), 4u * 37u);
    EXPECT_NEAR(rows.front().theta_rad, sp.theta_center_rad - M_PI / 2.0, 1e-12);
    EXPECT_NEAR(rows.back().theta_rad, sp.theta_center_rad + M_PI / 2.0, 1e-12);
}

TEST(CandidateScanRows, EvMatchesIndependentComputation) {
    const CubeSatelliteModel m{CubeSatelliteModel::Params{}};
    const ProDMP tmpl = makeSyntheticProDmpTemplate();
    const ScanParams sp = makeScan(m);
    const auto rows = scanCandidates(m, tmpl, sp);

    // Independent advance: same recipe, done by hand (200 steps of 5 ms = tau_contact 1 s).
    ProDMP ref = tmpl;
    ref.setRelativeGoal(false);
    ref.setInitialConditions(0.0, sp.p0, Eigen::Vector3d::Zero());
    ref.setGoal(sp.p0 + sp.delta_g_demo);
    for (int i = 0; i < 200; ++i) ref.step(0.005);

    for (const CandidateRow& r : rows) {
        const Eigen::Vector3d g = m.graspPoseAt(r.k, r.theta_rad).position_world;
        const double e_v = haptic_dmp_learning::core::velocityTerm(
            ref.velocityForCandidateGoal(g),
            haptic_dmp_learning::core::surfaceVelocity(m, r.k, r.theta_rad, sp.omega_rad_s));
        EXPECT_NEAR(r.e_v, e_v, 1e-12);
        EXPECT_NEAR(r.e_v_hat, e_v / sp.e_v_ref, 1e-9);
        EXPECT_NEAR(r.partial_cost, sp.weights.w_v * r.e_v_hat + sp.weights.w_g * r.e_g_hat, 1e-12);
    }
}

TEST(CandidateScanRows, TemplateIsNotModified) {
    const CubeSatelliteModel m{CubeSatelliteModel::Params{}};
    const ProDMP tmpl = makeSyntheticProDmpTemplate();
    const Eigen::Vector3d v0 = tmpl.velocity(), p0 = tmpl.position();
    scanCandidates(m, tmpl, makeScan(m));
    EXPECT_EQ(tmpl.velocity(), v0);
    EXPECT_EQ(tmpl.position(), p0);
}

TEST(CandidateScanRows, ContactToEndOffset) {
    const CubeSatelliteModel m{CubeSatelliteModel::Params{}};
    const ProDMP tmpl = makeSyntheticProDmpTemplate();
    ScanParams sp = makeScan(m);
    const auto base = scanCandidates(m, tmpl, sp);
    sp.contact_to_end_offset = Eigen::Vector3d(0.05, -0.02, 0.01);
    const auto shifted = scanCandidates(m, tmpl, sp);
    ASSERT_EQ(base.size(), shifted.size());
    for (std::size_t i = 0; i < base.size(); ++i) {
        EXPECT_NEAR((shifted[i].goal_param - (shifted[i].contact_point + sp.contact_to_end_offset)).norm(),
                    0.0, 1e-15);
        EXPECT_NEAR((base[i].goal_param - base[i].contact_point).norm(), 0.0, 1e-15);
        const double e_g = haptic_dmp_learning::core::goalTerm(shifted[i].goal_param - sp.p0,
                                                               sp.delta_g_demo);
        EXPECT_NEAR(shifted[i].e_g, e_g, 1e-15);
        EXPECT_NE(shifted[i].e_g, base[i].e_g);
    }
}

TEST(CandidateScanRows, SortingAndBestPerPoint) {
    const CubeSatelliteModel m{CubeSatelliteModel::Params{}};
    const auto rows = scanCandidates(m, makeSyntheticProDmpTemplate(), makeScan(m));
    const auto sorted = sortedByPartialCost(rows);
    ASSERT_EQ(sorted.size(), rows.size());
    for (std::size_t i = 1; i < sorted.size(); ++i) {
        EXPECT_LE(sorted[i - 1].partial_cost, sorted[i].partial_cost);
    }
    const auto best = bestPerPoint(rows, 3);
    ASSERT_EQ(best.size(), 12u);
    int i = 0;
    for (GraspPointId k : {GraspPointId::kP0, GraspPointId::kP90, GraspPointId::kP180,
                           GraspPointId::kP270}) {
        for (int j = 0; j < 3; ++j, ++i) {
            EXPECT_EQ(best[i].k, k);
            if (j > 0) {
                EXPECT_LE(best[i - 1].partial_cost, best[i].partial_cost);
            }
        }
    }
    EXPECT_THROW(bestPerPoint(rows, 0), std::invalid_argument);
}

TEST(CandidateScanLaunchDelay, HandComputedAndRange) {
    const double omega = -degToRad(2.0);  // T = 180 s
    // (-60 deg - 0)/(-2 deg/s) = 30 s, minus tau_contact 30 -> 0.
    EXPECT_NEAR(launchDelay(0.0, -degToRad(60.0), omega, 30.0), 0.0, 1e-9);
    // (-30)/(-2) = 15 s, minus 30 = -15 -> +180 = 165.
    EXPECT_NEAR(launchDelay(0.0, -degToRad(30.0), omega, 30.0), 165.0, 1e-9);
    // min_delay shifts the interval: -15 into [10, 190) -> 165 stays, 5 -> 185.
    EXPECT_NEAR(launchDelay(0.0, -degToRad(30.0), omega, 30.0, 10.0), 165.0, 1e-9);
    EXPECT_NEAR(launchDelay(0.0, -degToRad(70.0), omega, 30.0, 10.0), 185.0, 1e-9);

    for (double now : {0.0, 1.0, -2.5}) {
        for (double star : {-3.0, -0.4, 0.7, 2.9}) {
            for (double md : {0.0, 12.5}) {
                const double d = launchDelay(now, star, omega, 30.0, md);
                EXPECT_GE(d, md);
                EXPECT_LT(d, md + 180.0);
            }
        }
    }
    EXPECT_THROW(launchDelay(0.0, 1.0, 0.0, 30.0), std::invalid_argument);
}

TEST(CandidateScanRows, InvalidArgumentsThrow) {
    const CubeSatelliteModel m{CubeSatelliteModel::Params{}};
    const ProDMP tmpl = makeSyntheticProDmpTemplate();
    ScanParams sp = makeScan(m);
    sp.step_rad = 0.0;
    EXPECT_THROW(scanCandidates(m, tmpl, sp), std::invalid_argument);
    sp = makeScan(m);
    sp.half_window_rad = 0.0;
    EXPECT_THROW(scanCandidates(m, tmpl, sp), std::invalid_argument);
    sp = makeScan(m);
    sp.tau_contact_s = tmpl.tau() + 0.5;
    EXPECT_THROW(scanCandidates(m, tmpl, sp), std::invalid_argument);
    sp = makeScan(m);
    sp.e_v_ref = 0.0;
    EXPECT_THROW(scanCandidates(m, tmpl, sp), std::invalid_argument);
    sp = makeScan(m);
    sp.e_g_ref = -1.0;
    EXPECT_THROW(scanCandidates(m, tmpl, sp), std::invalid_argument);
}

// EMPIRICAL probe, NO asserts on values. Convenience configuration: the real starting pose is not
// decided yet. Prints the whole scan (production ProDMP template, satellite at (0.75,0,0.35),
// vertical axis, face towards the robot base) so it can be read, not judged.
TEST(SatelliteScanProbe, PrintsScanOverProductionTemplate) {
    SKIP_UNLESS_PROBES_ENABLED();
    const char* home = std::getenv("HOME");
    const std::string urdf_path =
        std::string(home ? home : "/root") + "/thesis_ws/fer_flat_effort.urdf";
    std::ifstream urdf(urdf_path);
    if (!urdf.is_open()) {
        GTEST_SKIP() << "Could not open URDF: " << urdf_path;
    }
    std::stringstream buffer;
    buffer << urdf.rdbuf();
    std::vector<std::string> joint_names;
    for (int i = 1; i <= 7; ++i) joint_names.push_back("fer_joint" + std::to_string(i));
    RobotModel robot(buffer.str(), joint_names, "fer_hand_tcp");

    SKIP_UNLESS_PRODUCTION_WEIGHTS(weights_path);
    // demoDisplacement() must be read BEFORE any setInitialConditions().
    const ProDMP tmpl = haptic_dmp_learning::core::prodmp_io::loadProDmpFromYaml(weights_path);
    const Eigen::Vector3d delta_g_demo = tmpl.demoDisplacement();

    auto p = makeModelBParams();
    const CubeSatelliteModel model(p);

    resetRobotToReady(robot);
    const Eigen::Vector3d p0 = robot.eePosition();
    Eigen::Vector3d base = Eigen::Vector3d::Zero();
    try {
        base = robot.framePose("fer_link0").position;
    } catch (const std::exception& e) {
        std::cout << "[scan-config] fer_link0 not available (" << e.what() << "), base = origin"
                  << std::endl;
    }

    ScanParams sp;
    sp.theta_center_rad = windowCenterTheta(model, base);
    sp.half_window_rad = M_PI / 2.0;
    sp.step_rad = degToRad(10.0);
    sp.omega_rad_s = -degToRad(2.0);
    sp.tau_contact_s = 0.0;  // template tau
    sp.p0 = p0;
    sp.delta_g_demo = delta_g_demo;
    sp.contact_to_end_offset = Eigen::Vector3d::Zero();
    sp.e_v_ref = haptic_dmp_learning::core::meanSurfaceSpeedSquared(model, sp.omega_rad_s);
    sp.e_g_ref = delta_g_demo.squaredNorm();

    const auto t_start = std::chrono::steady_clock::now();
    const auto rows = scanCandidates(model, tmpl, sp);
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start).count();

    std::cout << "[scan-config] theta_c=" << radToDeg(sp.theta_center_rad) << " deg base=("
              << base.x() << "," << base.y() << "," << base.z() << ") rows=" << rows.size()
              << " (expected 76) e_v_ref=" << sp.e_v_ref * 1e6 << " (mm/s)^2 e_g_ref="
              << sp.e_g_ref * 1e6 << " mm^2 tau=" << tmpl.tau() << " s scanCandidates="
              << ms << " ms" << std::endl;

    // Sanity reference: e_v at theta = 0 from the velocity probe (omega = -2 deg/s column).
    const double expected_ev[4] = {78.5107, 80.3359, 88.2633, 80.3457};
    const char* names[4] = {"kP0", "kP90", "kP180", "kP270"};
    const GraspPointId ids[4] = {GraspPointId::kP0, GraspPointId::kP90, GraspPointId::kP180,
                                 GraspPointId::kP270};
    for (int i = 0; i < 4; ++i) {
        const CandidateRow* found = nullptr;
        double best_gap = 1e300;
        for (const auto& r : rows) {
            if (r.k == ids[i] && std::abs(r.theta_rad) < best_gap) {
                best_gap = std::abs(r.theta_rad);
                found = &r;
            }
        }
        if (!found) continue;
        const double ev = found->e_v * 1e6;
        std::cout << "[scan-check] " << names[i] << " theta=" << radToDeg(found->theta_rad)
                  << " deg" << (best_gap > 1e-9 ? " (NOT exactly 0: grid does not contain 0)" : "")
                  << " e_v=" << ev << " (mm/s)^2 expected=" << expected_ev[i]
                  << " rel_diff=" << (ev - expected_ev[i]) / expected_ev[i] << std::endl;
    }

    auto line = [&](const char* tag, const CandidateRow& r) {
        const char* kname = names[static_cast<int>(r.k)];
        std::cout << tag << " " << kname << " theta=" << radToDeg(r.theta_rad)
                  << " deg e_v_hat=" << r.e_v_hat << " e_g_hat=" << r.e_g_hat
                  << " partial_cost=" << r.partial_cost << " |v_ee|=" << r.v_ee.norm() * 1000.0
                  << " mm/s |v_target|=" << r.v_target.norm() * 1000.0
                  << " mm/s |goal_param-p0|=" << (r.goal_param - p0).norm() * 1000.0 << " mm"
                  << std::endl;
    };
    const auto sorted = sortedByPartialCost(rows);
    for (std::size_t i = 0; i < std::min<std::size_t>(10, sorted.size()); ++i) {
        line("[scan-top]", sorted[i]);
    }
    for (const auto& r : bestPerPoint(rows, 1)) line("[scan-best]", r);

    if (!rows.empty()) {
        auto mm = [&](auto get) {
            const auto [lo, hi] = std::minmax_element(
                rows.begin(), rows.end(),
                [&](const CandidateRow& a, const CandidateRow& b) { return get(a) < get(b); });
            return std::make_pair(get(*lo), get(*hi));
        };
        const auto ev = mm([](const CandidateRow& r) { return r.e_v_hat; });
        const auto eg = mm([](const CandidateRow& r) { return r.e_g_hat; });
        const auto pc = mm([](const CandidateRow& r) { return r.partial_cost; });
        const CandidateRow& b = sorted.front();
        std::cout << "[scan-range] e_v_hat=[" << ev.first << ", " << ev.second << "] e_g_hat=["
                  << eg.first << ", " << eg.second << "] partial_cost=[" << pc.first << ", "
                  << pc.second << "] | best row: w_v*e_v_hat=" << sp.weights.w_v * b.e_v_hat
                  << " w_g*e_g_hat=" << sp.weights.w_g * b.e_g_hat << std::endl;
    }
}
