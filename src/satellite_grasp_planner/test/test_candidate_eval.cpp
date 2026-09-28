// Tests for evaluateCandidate() (synthetic ProDMP, cheap) plus an empirical feasibility-map probe
// with the production template. Need Pinocchio, Coal and $HOME/thesis_ws/fer_flat_effort.urdf.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/grasp_roll.hpp"
#include "haptic_dmp_learning/core/math_utils.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "haptic_dmp_learning/core/types.hpp"
#include "satellite_grasp_planner/core/candidate_eval.hpp"
#include "satellite_grasp_planner/core/candidate_scan.hpp"
#include "satellite_grasp_planner/core/kinematic_feasibility.hpp"
#include "satellite_grasp_planner/core/satellite_collision.hpp"

using namespace satellite_grasp_planner::core;
using haptic_dmp_learning::core::CubeSatelliteModel;
using haptic_dmp_learning::core::GraspPointId;
using haptic_dmp_learning::core::ProDMP;
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

/// 2 s straight 0.1 m demo, 8 basis functions (mechanism only, not a result).
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

}  // namespace

TEST(CandidateEvalTest, MatchesManualComposition) {
    auto robot = loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    const ProDMP tmpl = makeSyntheticProDmpTemplate();
    CubeSatelliteModel::Params p;
    p.center_world = Eigen::Vector3d(0.75, 0.0, 0.35);
    p.axis_world = Eigen::Vector3d(0.0, 0.0, 1.0);
    p.face_normal_body = Eigen::Vector3d(-1.0, 0.0, 0.0);
    const CubeSatelliteModel model(p);
    const GraspPointId k = GraspPointId::kP90;
    const double theta = 0.3, psi = 0.2;
    const RobotModel::JointVector q0 = RobotModel::readyPose();

    const CandidateEval ev = evaluateCandidate(model, k, theta, psi, tmpl, robot, q0);

    // Same inputs, by hand.
    const auto target = model.graspPoseAt(k, theta);
    const auto kin = checkKinematicFeasibility(
        tmpl, target.position_world,
        haptic_dmp_learning::core::applyRoll(target.orientation_nominal_world, psi), robot, q0);
    robot->update(kin.q_final, RobotModel::JointVector::Zero());
    const auto col = checkSatelliteCollision(*robot, model, theta, 0.010);

    EXPECT_EQ(ev.reached, kin.pos_err_final_mm < 20.0);
    EXPECT_EQ(ev.feasible, ev.kin_feasible && ev.reached && ev.collision_feasible);
    EXPECT_EQ(ev.kin_feasible, kin.feasible);
    EXPECT_DOUBLE_EQ(ev.max_joint_violation_rad, kin.max_joint_violation_rad);
    EXPECT_DOUBLE_EQ(ev.w_trans_final, kin.w_trans_final);
    EXPECT_DOUBLE_EQ(ev.pos_err_final_mm, kin.pos_err_final_mm);
    EXPECT_EQ(ev.collision_feasible, col.feasible);
    EXPECT_DOUBLE_EQ(ev.min_distance_arm_m, col.min_distance_arm_m);
    EXPECT_DOUBLE_EQ(ev.min_distance_cube_m, col.min_distance_cube_m);
    EXPECT_EQ(ev.closest_capsule, col.closest_capsule);
    EXPECT_EQ(ev.closest_capsule_cube, col.closest_capsule_cube);

    // (b) finite outputs.
    EXPECT_TRUE(std::isfinite(ev.w_trans_final));
    EXPECT_TRUE(kin.q_final.allFinite());
}

// psi and psi + pi: same target position, different target orientation (recomputed here exactly as
// evaluateCandidate does). Both evaluations must also run.
TEST(CandidateEvalTest, RollAndRollPlusPiShareTargetPositionButNotOrientation) {
    CubeSatelliteModel::Params p;
    p.center_world = Eigen::Vector3d(0.75, 0.0, 0.35);
    p.axis_world = Eigen::Vector3d(0.0, 0.0, 1.0);
    p.face_normal_body = Eigen::Vector3d(-1.0, 0.0, 0.0);
    const CubeSatelliteModel model(p);
    const double theta = 0.3, psi = 0.2;
    const auto t0 = model.graspPoseAt(GraspPointId::kP0, theta);
    const Eigen::Matrix3d R_a =
        haptic_dmp_learning::core::applyRoll(t0.orientation_nominal_world, psi).toRotationMatrix();
    const Eigen::Matrix3d R_b =
        haptic_dmp_learning::core::applyRoll(t0.orientation_nominal_world, psi + M_PI).toRotationMatrix();
    EXPECT_GT((R_a - R_b).norm(), 1.0);                        // x and y flipped: not equal
    EXPECT_NEAR((R_a.col(2) - R_b.col(2)).norm(), 0.0, 1e-12);  // approach axis identical
    EXPECT_NEAR((R_a.col(0) + R_b.col(0)).norm(), 0.0, 1e-12);

    auto robot = loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    const ProDMP tmpl = makeSyntheticProDmpTemplate();
    const RobotModel::JointVector q0 = RobotModel::readyPose();
    EXPECT_NO_THROW(evaluateCandidate(model, GraspPointId::kP0, theta, psi, tmpl, robot, q0));
    EXPECT_NO_THROW(evaluateCandidate(model, GraspPointId::kP0, theta, psi + M_PI, tmpl, robot, q0));
}

// EMPIRICAL probe, NO asserts on values: feasibility map over theta x k for two satellite
// configurations, production template, psi = 0. ~56 rollouts, ~7 minutes.
TEST(SatelliteFeasibilityMapProbe, PrintsFeasibilityMap) {
    const char* home = std::getenv("HOME");
    const char* env_w = std::getenv("GRASP_PROBE_WEIGHTS");
    const std::string weights_path =
        env_w ? std::string(env_w)
              : std::string(home ? home : "/root") +
                    "/thesis_ws/runs/20260914_150515_fit_reach_task_baseline_prodmp_n80_lam1e-10_w0.05/"
                    "weights.yaml";
    if (!std::ifstream(weights_path).good()) {
        GTEST_SKIP() << "Production weights.yaml not reachable: " << weights_path
                     << " (set GRASP_PROBE_WEIGHTS)";
    }
    auto robot = loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    const ProDMP tmpl = haptic_dmp_learning::core::prodmp_io::loadProDmpFromYaml(weights_path);

    struct Config {
        const char* name;
        Eigen::Vector3d center, axis, normal;
    };
    const std::vector<Config> configs = {
        {"A", {0.3928, -0.165, 0.1188}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}},
        {"B", {0.75, 0.0, 0.35}, {0.0, 0.0, 1.0}, {-1.0, 0.0, 0.0}}};
    const std::vector<std::pair<const char*, GraspPointId>> points = {
        {"kP0", GraspPointId::kP0}, {"kP90", GraspPointId::kP90},
        {"kP180", GraspPointId::kP180}, {"kP270", GraspPointId::kP270}};

    const RobotModel::JointVector q0 = RobotModel::readyPose();
    robot->update(q0, RobotModel::JointVector::Zero());
    const Eigen::Vector3d base = robot->framePose("fer_link0").position;

    std::map<std::string, int> feasible_count;  // "<config> <k>" -> count
    int n_total = 0;
    const auto t_start = std::chrono::steady_clock::now();
    for (const Config& c : configs) {
        CubeSatelliteModel::Params p;
        p.center_world = c.center;
        p.axis_world = c.axis;
        p.face_normal_body = c.normal;
        const CubeSatelliteModel model(p);
        const double theta_c = windowCenterTheta(model, base);
        for (const auto& pt : points) {
            feasible_count[std::string(c.name) + " " + pt.first] += 0;
            for (int i = 0; i < 7; ++i) {
                const double theta = theta_c + haptic_dmp_learning::core::degToRad(-90.0 + 30.0 * i);
                const CandidateEval ev = evaluateCandidate(model, pt.second, theta, 0.0, tmpl, robot, q0);
                ++n_total;
                if (ev.feasible) ++feasible_count[std::string(c.name) + " " + pt.first];
                std::cout << "[map] " << c.name << " " << pt.first << " theta="
                          << haptic_dmp_learning::core::radToDeg(theta) << " feasible=" << ev.feasible
                          << " kin_feasible=" << ev.kin_feasible
                          << " max_joint_violation_rad=" << ev.max_joint_violation_rad
                          << " w_trans_final=" << ev.w_trans_final
                          << " pos_err_final_mm=" << ev.pos_err_final_mm
                          << " collision_feasible=" << ev.collision_feasible
                          << " min_distance_arm_m=" << ev.min_distance_arm_m
                          << " min_distance_cube_m=" << ev.min_distance_cube_m
                          << " closest_capsule=" << ev.closest_capsule << std::endl;
            }
        }
    }
    const double total_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    for (const Config& c : configs) {
        for (const auto& pt : points) {
            std::cout << "[map-summary] " << c.name << " " << pt.first << ": "
                      << feasible_count[std::string(c.name) + " " + pt.first] << "/7 feasible"
                      << std::endl;
        }
    }
    std::cout << "[map-time] total=" << total_s << " s, per candidate="
              << (n_total ? total_s / n_total : 0.0) << " s (" << n_total << " candidates)"
              << std::endl;
}

// EMPIRICAL probe, NO asserts on values: can a roll psi rescue candidates that fail at psi = 0?
// Configuration B, production template. 20 rollouts, ~150 s. Theta is ABSOLUTE (not relative to
// theta_c).
TEST(SatelliteRollRescueProbe, PrintsRollRescue) {
    const char* home = std::getenv("HOME");
    const char* env_w = std::getenv("GRASP_PROBE_WEIGHTS");
    const std::string weights_path =
        env_w ? std::string(env_w)
              : std::string(home ? home : "/root") +
                    "/thesis_ws/runs/20260914_150515_fit_reach_task_baseline_prodmp_n80_lam1e-10_w0.05/"
                    "weights.yaml";
    if (!std::ifstream(weights_path).good()) {
        GTEST_SKIP() << "Production weights.yaml not reachable: " << weights_path
                     << " (set GRASP_PROBE_WEIGHTS)";
    }
    auto robot = loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    const ProDMP tmpl = haptic_dmp_learning::core::prodmp_io::loadProDmpFromYaml(weights_path);

    CubeSatelliteModel::Params p;
    p.center_world = Eigen::Vector3d(0.75, 0.0, 0.35);
    p.axis_world = Eigen::Vector3d(0.0, 0.0, 1.0);
    p.face_normal_body = Eigen::Vector3d(-1.0, 0.0, 0.0);
    const CubeSatelliteModel model(p);
    const RobotModel::JointVector q0 = RobotModel::readyPose();

    struct Cand {
        const char* name;
        GraspPointId k;
        double theta_deg;
    };
    const std::vector<Cand> cands = {{"kP180", GraspPointId::kP180, 0.0},
                                     {"kP90", GraspPointId::kP90, 0.0},
                                     {"kP270", GraspPointId::kP270, 0.0},
                                     {"kP270", GraspPointId::kP270, 60.0}};
    const std::vector<double> psis_deg = {-10.0, 10.0, 170.0, 180.0, -170.0};

    const auto t_start = std::chrono::steady_clock::now();
    int n_total = 0;
    std::vector<std::string> summary;
    for (const Cand& c : cands) {
        std::ostringstream ok;
        for (double psi_deg : psis_deg) {
            const double theta = haptic_dmp_learning::core::degToRad(c.theta_deg);
            const double psi = haptic_dmp_learning::core::degToRad(psi_deg);
            const CandidateEval ev =
                evaluateCandidate(model, c.k, theta, psi, tmpl, robot, q0, 0.010, 20.0);
            ++n_total;
            if (ev.feasible) ok << " " << psi_deg;
            std::cout << "[rescue] " << c.name << " theta=" << c.theta_deg << " psi=" << psi_deg
                      << " feasible=" << ev.feasible << " kin_feasible=" << ev.kin_feasible
                      << " reached=" << ev.reached
                      << " max_joint_violation_rad=" << ev.max_joint_violation_rad
                      << " pos_err_final_mm=" << ev.pos_err_final_mm
                      << " w_trans_final=" << ev.w_trans_final
                      << " collision_feasible=" << ev.collision_feasible
                      << " min_distance_arm_m=" << ev.min_distance_arm_m
                      << " min_distance_cube_m=" << ev.min_distance_cube_m << std::endl;
        }
        std::ostringstream line;
        line << "[rescue-summary] " << c.name << " theta=" << c.theta_deg
             << ": feasible psi(deg) =" << (ok.str().empty() ? " none" : ok.str());
        summary.push_back(line.str());
    }
    for (const auto& l : summary) std::cout << l << std::endl;
    const double total_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    std::cout << "[rescue-time] total=" << total_s << " s, per candidate="
              << (n_total ? total_s / n_total : 0.0) << " s (" << n_total << " candidates)"
              << std::endl;
}

// EMPIRICAL probe, NO asserts on values: which joint / where along the path does each candidate
// violate its limit? Configuration B, production template. Calls checkKinematicFeasibility directly
// (no collision, no reached criterion). Theta is ABSOLUTE.
TEST(SatelliteViolationDiagnosisProbe, PrintsWorstJointViolation) {
    const char* home = std::getenv("HOME");
    const char* env_w = std::getenv("GRASP_PROBE_WEIGHTS");
    const std::string weights_path =
        env_w ? std::string(env_w)
              : std::string(home ? home : "/root") +
                    "/thesis_ws/runs/20260914_150515_fit_reach_task_baseline_prodmp_n80_lam1e-10_w0.05/"
                    "weights.yaml";
    if (!std::ifstream(weights_path).good()) {
        GTEST_SKIP() << "Production weights.yaml not reachable: " << weights_path
                     << " (set GRASP_PROBE_WEIGHTS)";
    }
    auto robot = loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    const ProDMP tmpl = haptic_dmp_learning::core::prodmp_io::loadProDmpFromYaml(weights_path);

    CubeSatelliteModel::Params p;
    p.center_world = Eigen::Vector3d(0.75, 0.0, 0.35);
    p.axis_world = Eigen::Vector3d(0.0, 0.0, 1.0);
    p.face_normal_body = Eigen::Vector3d(-1.0, 0.0, 0.0);
    const CubeSatelliteModel model(p);
    const RobotModel::JointVector q0 = RobotModel::readyPose();

    struct Cand {
        const char* name;
        GraspPointId k;
        double theta_deg, psi_deg;
    };
    const std::vector<Cand> cands = {
        {"kP0", GraspPointId::kP0, -30, 0},     {"kP0", GraspPointId::kP0, 0, 0},
        {"kP90", GraspPointId::kP90, -60, 0},   {"kP90", GraspPointId::kP90, -30, 0},
        {"kP90", GraspPointId::kP90, 0, 0},     {"kP180", GraspPointId::kP180, 0, 0},
        {"kP270", GraspPointId::kP270, 0, 0},   {"kP270", GraspPointId::kP270, 30, 0},
        {"kP270", GraspPointId::kP270, 60, 0},  {"kP270", GraspPointId::kP270, 60, 10}};

    const auto t_start = std::chrono::steady_clock::now();
    for (const Cand& c : cands) {
        const double theta = haptic_dmp_learning::core::degToRad(c.theta_deg);
        const double psi = haptic_dmp_learning::core::degToRad(c.psi_deg);
        const auto target = model.graspPoseAt(c.k, theta);
        const auto kin = checkKinematicFeasibility(
            tmpl, target.position_world,
            haptic_dmp_learning::core::applyRoll(target.orientation_nominal_world, psi), robot, q0);
        std::cout << "[diag] " << c.name << " theta=" << c.theta_deg << " psi=" << c.psi_deg
                  << " max_joint_violation_rad=" << kin.max_joint_violation_rad << " worst_joint="
                  << (kin.worst_joint >= 0 ? std::to_string(kin.worst_joint + 1) : std::string("none"))
                  << " worst_step_fraction=" << kin.worst_step_fraction
                  << " worst_joint_value_rad=" << kin.worst_joint_value_rad
                  << " worst_joint_limit_rad=" << kin.worst_joint_limit_rad << " q_final=("
                  << kin.q_final.transpose() << ")" << std::endl;
    }
    const double total_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    std::cout << "[diag-time] total=" << total_s << " s, per candidate="
              << total_s / static_cast<double>(cands.size()) << " s (" << cands.size()
              << " candidates)" << std::endl;
}

// EMPIRICAL probe, NO asserts on values: roll basins. Where does the worst joint-limit violation
// sit for psi around 180 / -170 / -20 on candidates that fail at psi = 0? Configuration B,
// production template, checkKinematicFeasibility called directly (no collision, no reached
// criterion). Theta is ABSOLUTE. 8 rollouts, ~60 s.
TEST(SatelliteRollBasinProbe, PrintsRollBasins) {
    const char* home = std::getenv("HOME");
    const char* env_w = std::getenv("GRASP_PROBE_WEIGHTS");
    const std::string weights_path =
        env_w ? std::string(env_w)
              : std::string(home ? home : "/root") +
                    "/thesis_ws/runs/20260914_150515_fit_reach_task_baseline_prodmp_n80_lam1e-10_w0.05/"
                    "weights.yaml";
    if (!std::ifstream(weights_path).good()) {
        GTEST_SKIP() << "Production weights.yaml not reachable: " << weights_path
                     << " (set GRASP_PROBE_WEIGHTS)";
    }
    auto robot = loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    const ProDMP tmpl = haptic_dmp_learning::core::prodmp_io::loadProDmpFromYaml(weights_path);

    CubeSatelliteModel::Params p;
    p.center_world = Eigen::Vector3d(0.75, 0.0, 0.35);
    p.axis_world = Eigen::Vector3d(0.0, 0.0, 1.0);
    p.face_normal_body = Eigen::Vector3d(-1.0, 0.0, 0.0);
    const CubeSatelliteModel model(p);
    const RobotModel::JointVector q0 = RobotModel::readyPose();

    struct Cand {
        const char* name;
        GraspPointId k;
        double theta_deg, psi_deg;
    };
    const std::vector<Cand> cands = {
        {"kP0", GraspPointId::kP0, -30, 180},   {"kP0", GraspPointId::kP0, -30, -170},
        {"kP0", GraspPointId::kP0, -30, -20},   {"kP90", GraspPointId::kP90, -60, 180},
        {"kP90", GraspPointId::kP90, -60, -170}, {"kP0", GraspPointId::kP0, 0, 180},
        {"kP90", GraspPointId::kP90, -30, 180}, {"kP270", GraspPointId::kP270, 30, 180}};

    const auto t_start = std::chrono::steady_clock::now();
    for (const Cand& c : cands) {
        const double theta = haptic_dmp_learning::core::degToRad(c.theta_deg);
        const double psi = haptic_dmp_learning::core::degToRad(c.psi_deg);
        const auto target = model.graspPoseAt(c.k, theta);
        const auto kin = checkKinematicFeasibility(
            tmpl, target.position_world,
            haptic_dmp_learning::core::applyRoll(target.orientation_nominal_world, psi), robot, q0);
        std::cout << "[basin] " << c.name << " theta=" << c.theta_deg << " psi=" << c.psi_deg
                  << " max_joint_violation_rad=" << kin.max_joint_violation_rad << " worst_joint="
                  << (kin.worst_joint >= 0 ? std::to_string(kin.worst_joint + 1) : std::string("none"))
                  << " worst_step_fraction=" << kin.worst_step_fraction
                  << " worst_joint_value_rad=" << kin.worst_joint_value_rad
                  << " worst_joint_limit_rad=" << kin.worst_joint_limit_rad
                  << " pos_err_final_mm=" << kin.pos_err_final_mm << " q_final=("
                  << kin.q_final.transpose() << ")" << std::endl;
    }
    const double total_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    std::cout << "[basin-time] total=" << total_s << " s, per candidate="
              << total_s / static_cast<double>(cands.size()) << " s (" << cands.size()
              << " candidates)" << std::endl;
}
