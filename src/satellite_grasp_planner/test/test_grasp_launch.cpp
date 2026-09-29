// Tests for planGraspAndLaunch() (grasp_launch.hpp): the single-entry-point wrapper chaining
// selectGrasp() and planLaunch() with one output struct that carries an explicit status instead of a
// silently-zero goal. All tests build SelectionInputs BY HAND with the synthetic ProDMP template
// (test_fixtures.hpp) - no dependency on production weights/demo files, nothing is skipped.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <optional>

#include <Eigen/Dense>

#include "test_fixtures.hpp"
#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/grasp_cost.hpp"
#include "haptic_dmp_learning/core/math_utils.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "satellite_grasp_planner/core/grasp_launch.hpp"
#include "satellite_grasp_planner/core/selection_inputs.hpp"

using namespace satellite_grasp_planner::core;
using haptic_dmp_learning::core::CubeSatelliteModel;
using haptic_dmp_learning::core::ProDMP;
using haptic_dmp_learning::core::degToRad;
using haptic_dmp_learning::core::wrapPi;
using RobotModel = franka_cartesian_control::core::RobotModel;
using satellite_grasp_planner::test_fixtures::loadPandaRobotModel;
using satellite_grasp_planner::test_fixtures::makeSyntheticProDmpTemplate;

namespace {

/// Satellite placed so the theta = 0 grasp targets are close to readyPose with a plausible approach
/// direction (same placement as test_grasp_selection.cpp's makeNearbyModel; duplicated here since that
/// one lives in that file's anonymous namespace, not in the shared test_fixtures.hpp).
CubeSatelliteModel::Params makeNearbyCubeParams(RobotModel& robot, const RobotModel::JointVector& q0) {
    robot.update(q0, RobotModel::JointVector::Zero());
    const Eigen::Vector3d p_ee = robot.eePosition();
    const Eigen::Matrix3d R = robot.eeOrientation().toRotationMatrix();
    CubeSatelliteModel::Params p;
    p.axis_world = R.col(0);
    p.face_normal_body = -R.col(2);
    p.center_world = p_ee + Eigen::Vector3d(0.1, 0.0, 0.0) -
                     (p.cube_side_m / 2.0 + p.standoff_m) * p.face_normal_body;
    return p;
}

/// Position tolerance for the synthetic-template tests (see test_grasp_selection.cpp: the 2 s
/// synthetic template has a final tracking error of 20-58 mm, well above the 20 mm production default).
constexpr double kSyntheticPosTolMm = 100.0;

/// Builds a SelectionInputs by hand (no buildSelectionInputs(), no demo_params/weights file): the
/// synthetic template, the "nearby" satellite placement above, and a small scan (60 deg step, 3 thetas
/// x 4 points = 12 rows) so the tests stay quick.
SelectionInputs makeSyntheticInputs(const std::shared_ptr<RobotModel>& robot,
                                    bool contact_assumed_at_end) {
    const RobotModel::JointVector q0 = RobotModel::readyPose();
    const CubeSatelliteModel::Params cube_params = makeNearbyCubeParams(*robot, q0);
    const CubeSatelliteModel m(cube_params);
    const ProDMP tmpl = makeSyntheticProDmpTemplate();

    SelectionInputs in(tmpl);
    in.cube_params = cube_params;
    in.robot = robot;
    in.q0 = q0;
    robot->update(q0, RobotModel::JointVector::Zero());
    in.p0 = robot->eePosition();
    in.contact_assumed_at_end = contact_assumed_at_end;

    in.params.scan.theta_center_rad = 0.0;
    in.params.scan.half_window_rad = degToRad(60.0);
    in.params.scan.step_rad = degToRad(60.0);
    in.params.scan.omega_rad_s = -degToRad(2.0);
    in.params.scan.tau_contact_s = contact_assumed_at_end ? 0.0 : 1.0;
    in.params.scan.p0 = in.p0;
    in.params.scan.delta_g_demo = Eigen::Vector3d(0.1, 0.0, 0.0);
    in.params.scan.e_v_ref =
        haptic_dmp_learning::core::meanSurfaceSpeedSquared(m, in.params.scan.omega_rad_s);
    in.params.scan.e_g_ref = in.params.scan.delta_g_demo.squaredNorm();
    in.params.w_trans_demo = referenceWTrans(tmpl, robot, q0, in.params.scan.delta_g_demo);
    in.params.max_rows = 40;
    in.params.pos_tol_mm = kSyntheticPosTolMm;
    return in;
}

SatelliteSnapshot snapshotFrom(const SelectionInputs& in, double t_s = 0.0) {
    SatelliteSnapshot snap;
    snap.center = in.cube_params.center_world;
    snap.axis = in.cube_params.axis_world;
    snap.omega_rad_s = in.params.scan.omega_rad_s;
    snap.theta_rad = 0.0;
    snap.t_s = t_s;
    return snap;
}

}  // namespace

TEST(GraspLaunchTest, NoFeasibleCandidate) {
    auto robot = loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    const RobotModel::JointVector q0 = RobotModel::readyPose();
    const ProDMP tmpl = makeSyntheticProDmpTemplate();

    CubeSatelliteModel::Params cube_params;
    cube_params.center_world = Eigen::Vector3d(3.0, 3.0, 1.0);  // unreachable, as in
                                                                 // test_grasp_selection.cpp's
                                                                 // ReachedFilterRejectsUnreachedCandidates
    const CubeSatelliteModel m(cube_params);

    SelectionInputs in(tmpl);
    in.cube_params = cube_params;
    in.robot = robot;
    in.q0 = q0;
    robot->update(q0, RobotModel::JointVector::Zero());
    in.p0 = robot->eePosition();
    in.contact_assumed_at_end = true;

    in.params.scan.theta_center_rad = 0.0;
    in.params.scan.half_window_rad = degToRad(60.0);
    in.params.scan.step_rad = degToRad(60.0);
    in.params.scan.omega_rad_s = -degToRad(2.0);
    in.params.scan.tau_contact_s = 0.0;
    in.params.scan.p0 = in.p0;
    in.params.scan.delta_g_demo = Eigen::Vector3d(0.1, 0.0, 0.0);
    in.params.scan.e_v_ref =
        haptic_dmp_learning::core::meanSurfaceSpeedSquared(m, in.params.scan.omega_rad_s);
    in.params.scan.e_g_ref = in.params.scan.delta_g_demo.squaredNorm();
    in.params.w_trans_demo = 1.0;
    in.params.pos_tol_mm = 20.0;  // production value: nothing 3-4 m away can be "reached"
    in.params.max_rows = 1;       // keep the (long-distance) rollout cheap

    const GraspLaunchResult result = planGraspAndLaunch(in, snapshotFrom(in), /*min_delay_s=*/0.1);

    EXPECT_EQ(result.status, GraspLaunchResult::Status::kNoFeasibleCandidate);
    EXPECT_FALSE(result.selection.found);
    EXPECT_FALSE(result.k.has_value());
    EXPECT_FALSE(result.theta_star_rad.has_value());
    EXPECT_FALSE(result.psi_rad.has_value());
    EXPECT_FALSE(result.goal_position.has_value());
    EXPECT_FALSE(result.goal_orientation.has_value());
    EXPECT_FALSE(result.launch.has_value());
}

// min_delay_s is validated BEFORE selectGrasp() runs: the injected clock must never be called.
TEST(GraspLaunchTest, InvalidMinDelayThrowsBeforeAnyRollout) {
    auto robot = loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    SelectionInputs in = makeSyntheticInputs(robot, /*contact_assumed_at_end=*/true);
    const SatelliteSnapshot snap = snapshotFrom(in);

    ClockFn clock_must_not_be_called = []() -> double {
        ADD_FAILURE() << "clock() must not be called: no rollout should be attempted";
        return 0.0;
    };

    EXPECT_THROW(planGraspAndLaunch(in, snap, std::nan(""), clock_must_not_be_called),
                std::invalid_argument);
    EXPECT_THROW(planGraspAndLaunch(in, snap, -1.0, clock_must_not_be_called),
                std::invalid_argument);
}

// A snapshot whose omega disagrees with the one 'in' was built from is not the acquisition snapshot:
// rejected BEFORE selectGrasp() runs, so the injected clock must never be called either.
TEST(GraspLaunchTest, SnapshotOmegaMismatchThrowsBeforeAnyRollout) {
    auto robot = loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    SelectionInputs in = makeSyntheticInputs(robot, /*contact_assumed_at_end=*/true);
    SatelliteSnapshot snap = snapshotFrom(in);
    snap.omega_rad_s = in.params.scan.omega_rad_s * 2.0;  // deliberately inconsistent with 'in'

    ClockFn clock_must_not_be_called = []() -> double {
        ADD_FAILURE() << "clock() must not be called: no rollout should be attempted";
        return 0.0;
    };

    EXPECT_THROW(planGraspAndLaunch(in, snap, /*min_delay_s=*/0.1, clock_must_not_be_called),
                std::invalid_argument);
}

TEST(GraspLaunchTest, BudgetZeroEndToEnd) {
    auto robot = loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    SelectionInputs in = makeSyntheticInputs(robot, /*contact_assumed_at_end=*/true);
    in.params.time_budget_s = 0.0;

    const GraspLaunchResult result = planGraspAndLaunch(in, snapshotFrom(in), /*min_delay_s=*/0.1);

    EXPECT_EQ(result.selection.rows_evaluated, 0);
    EXPECT_EQ(result.selection.total_rollouts, 0);
    EXPECT_FALSE(result.selection.found);
    EXPECT_EQ(result.selection.stop_reason, GraspSelection::StopReason::kTimeBudgetExhausted);
    EXPECT_TRUE(result.selection.budget_exhausted);
    EXPECT_EQ(result.status, GraspLaunchResult::Status::kBudgetExhaustedNoCandidate);
    EXPECT_FALSE(result.launch.has_value());
}

// t_now defaults to snapshot.t_s + selection.elapsed_s: verified by independently calling planLaunch()
// with that exact value and comparing, rather than reaching into private state. A fake clock with
// small, deterministic steps keeps elapsed_s (and therefore the comparison) reproducible.
TEST(GraspLaunchTest, TNowDefaultsToSnapshotPlusElapsedAndCanBeOverridden) {
    auto robot = loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    SelectionInputs in = makeSyntheticInputs(robot, /*contact_assumed_at_end=*/true);
    const SatelliteSnapshot snap = snapshotFrom(in, /*t_s=*/100.0);

    auto ticks = std::make_shared<int>(0);
    ClockFn fake_clock = [ticks]() { return 0.01 * (*ticks)++; };

    const GraspLaunchResult result = planGraspAndLaunch(in, snap, /*min_delay_s=*/0.2, fake_clock);
    ASSERT_EQ(result.status, GraspLaunchResult::Status::kSelected);
    ASSERT_TRUE(result.launch.has_value());

    const double t_now_expected = snap.t_s + result.selection.elapsed_s;
    const LaunchPlan expected =
        planLaunch(snap, t_now_expected, result.theta_star_rad.value(), result.tau_launch_s, 0.2);
    EXPECT_DOUBLE_EQ(result.launch->delay_s, expected.delay_s);
    EXPECT_DOUBLE_EQ(result.launch->launch_time_s, expected.launch_time_s);
    EXPECT_DOUBLE_EQ(result.launch->contact_time_s, expected.contact_time_s);
    EXPECT_DOUBLE_EQ(result.launch->phase_at_contact_rad, expected.phase_at_contact_rad);

    // t_now_override_s, when given, takes precedence over snapshot.t_s + elapsed_s.
    auto ticks2 = std::make_shared<int>(0);
    ClockFn fake_clock2 = [ticks2]() { return 0.01 * (*ticks2)++; };
    const GraspLaunchResult overridden =
        planGraspAndLaunch(in, snap, 0.2, fake_clock2, /*t_now_override_s=*/500.0);
    ASSERT_TRUE(overridden.launch.has_value());
    const LaunchPlan expected_override =
        planLaunch(snap, 500.0, overridden.theta_star_rad.value(), overridden.tau_launch_s, 0.2);
    EXPECT_DOUBLE_EQ(overridden.launch->delay_s, expected_override.delay_s);
}

TEST(GraspLaunchTest, ContactAssumedAtEndUsesTemplateTauAndMatchesPhase) {
    auto robot = loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    SelectionInputs in = makeSyntheticInputs(robot, /*contact_assumed_at_end=*/true);
    const SatelliteSnapshot snap = snapshotFrom(in);

    const GraspLaunchResult result = planGraspAndLaunch(in, snap, /*min_delay_s=*/0.1);
    ASSERT_EQ(result.status, GraspLaunchResult::Status::kSelected);
    ASSERT_TRUE(result.launch.has_value());

    EXPECT_DOUBLE_EQ(result.tau_launch_s, in.prodmp_template.tau());

    const double phase_at_contact = phaseAt(snap, result.launch->contact_time_s);
    EXPECT_NEAR(wrapPi(phase_at_contact - result.theta_star_rad.value()), 0.0, 1e-9);
}
