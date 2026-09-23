#include <gtest/gtest.h>
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "haptic_dmp_learning/core/satellite_intercept.hpp"
#include <vector>
#include <cmath>
#include <cstdio>

using namespace haptic_dmp_learning::core;

namespace {

/// @brief Builds a smooth 5th-order minimum-jerk demo from p_start to p_goal.
std::vector<Sample> makeMinJerkDemo(const Eigen::Vector3d& p_start, const Eigen::Vector3d& p_goal,
                                    int N, double dt) {
    std::vector<Sample> demo(N);
    for (int i = 0; i < N; ++i) {
        double s = static_cast<double>(i) / (N - 1);
        double poly = s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
        demo[i].t = i * dt;
        demo[i].position = p_start + poly * (p_goal - p_start);
        demo[i].orientation = Eigen::Quaterniond::Identity();
    }
    return demo;
}

/// @brief Rolls a freshly re-initialized ProDMP for N-1 steps, returning the trajectory.
std::vector<Eigen::Vector3d> rollout(ProDMP& mp, double init_time, const Eigen::Vector3d& p0,
                                     const Eigen::Vector3d& v0, int N, double dt) {
    mp.setInitialConditions(init_time, p0, v0);
    std::vector<Eigen::Vector3d> traj;
    traj.reserve(N);
    traj.push_back(mp.position());
    for (int i = 0; i < N - 1; ++i) traj.push_back(mp.step(dt));
    return traj;
}

}  // namespace

/**
 * @brief Learning from a known synthetic demo then rolling out must reproduce the
 *        start exactly and converge to the demonstrated goal.
 */
TEST(ProDMPCoreTest, LearnsSyntheticMinJerkDemo) {
    const int N = 200;
    const double dt = 0.01;

    Eigen::Vector3d p_start(0.0, 0.0, 0.0);
    Eigen::Vector3d p_goal(0.3, -0.2, 0.4);
    auto demo = makeMinJerkDemo(p_start, p_goal, N, dt);

    ProDMP mp(20);
    mp.learnFromDemonstration(demo);

    EXPECT_TRUE(mp.isLearned());
    EXPECT_NEAR(mp.tau(), (N - 1) * dt, 1e-9);
    EXPECT_EQ(mp.diagnostics().dropped_non_monotonic_samples, 0);
    EXPECT_LT(mp.diagnostics().learn_residual_rms, 1e-3);

    auto traj = rollout(mp, 0.0, p_start, Eigen::Vector3d::Zero(), N, dt);

    // Start reproduced exactly.
    EXPECT_NEAR(traj.front().x(), p_start.x(), 1e-9);
    EXPECT_NEAR(traj.front().y(), p_start.y(), 1e-9);
    EXPECT_NEAR(traj.front().z(), p_start.z(), 1e-9);

    // Converged to the demonstrated goal.
    EXPECT_NEAR(traj.back().x(), p_goal.x(), 0.02);
    EXPECT_NEAR(traj.back().y(), p_goal.y(), 0.02);
    EXPECT_NEAR(traj.back().z(), p_goal.z(), 0.02);

    // The whole rollout should track the demo closely.
    double max_err = 0.0;
    for (int i = 0; i < N; ++i) max_err = std::max(max_err, (traj[i] - demo[i].position).norm());
    EXPECT_LT(max_err, 0.02);
}

/**
 * @brief setGoal() must overwrite ONLY the goal parameter: the shape weights w_i
 *        must be bit-for-bit unchanged, and the rollout must converge to the new
 *        goal (no kMinDG / amplitude-ratio guard involved).
 */
TEST(ProDMPCoreTest, SetGoalPreservesShapeWeights) {
    const int N = 200;
    const double dt = 0.01;

    Eigen::Vector3d p_start(0.1, -0.1, 0.2);
    Eigen::Vector3d p_goal_demo(0.4, 0.1, 0.5);
    Eigen::Vector3d p_goal_new(0.9, -0.6, 0.05);
    auto demo = makeMinJerkDemo(p_start, p_goal_demo, N, dt);

    ProDMP mp(20);
    mp.learnFromDemonstration(demo);

    // Snapshot the learned shape weights.
    std::array<Eigen::VectorXd, 3> w_before = mp.weights();

    mp.setInitialConditions(0.0, p_start, Eigen::Vector3d::Zero());
    mp.setGoal(p_goal_new);

    // Explicit numerical check: not a single shape weight moved.
    for (int d = 0; d < 3; ++d) {
        ASSERT_EQ(mp.weights()[d].size(), w_before[d].size());
        for (int i = 0; i < mp.numBasis(); ++i) {
            EXPECT_DOUBLE_EQ(mp.weights()[d](i), w_before[d](i))
                << "shape weight changed at dim " << d << " basis " << i;
        }
    }
    EXPECT_NEAR(mp.goal().x(), p_goal_new.x(), 1e-12);
    EXPECT_NEAR(mp.goal().y(), p_goal_new.y(), 1e-12);
    EXPECT_NEAR(mp.goal().z(), p_goal_new.z(), 1e-12);

    Eigen::Vector3d pos = mp.position();
    for (int i = 0; i < N - 1; ++i) pos = mp.step(dt);

    EXPECT_NEAR(pos.x(), p_goal_new.x(), 0.03);
    EXPECT_NEAR(pos.y(), p_goal_new.y(), 0.03);
    EXPECT_NEAR(pos.z(), p_goal_new.z(), 0.03);
}

/**
 * @brief Position and velocity must be continuous at t = 0: before the first
 *        step the reported state equals (init_pos, init_vel) exactly, and the
 *        first finite-difference step matches init_vel for a non-zero init_vel.
 */
TEST(ProDMPCoreTest, InitialVelocityContinuityAtStart) {
    const int N = 200;
    const double dt = 0.01;

    Eigen::Vector3d p_start(0.0, 0.0, 0.0);
    Eigen::Vector3d p_goal(0.25, 0.15, -0.1);
    auto demo = makeMinJerkDemo(p_start, p_goal, N, dt);

    ProDMP mp(20);
    mp.learnFromDemonstration(demo);

    Eigen::Vector3d p0(0.05, -0.02, 0.03);
    Eigen::Vector3d v0(0.4, -0.3, 0.2);  // deliberately non-zero
    mp.setInitialConditions(1.234, p0, v0);

    // Exact continuity before stepping.
    EXPECT_NEAR((mp.position() - p0).norm(), 0.0, 1e-12);
    EXPECT_NEAR((mp.velocity() - v0).norm(), 0.0, 1e-12);

    // First step: y(dt) ~ p0 + v0 * dt to O(dt^2), and the analytic velocity
    // stays close to v0 right after the start.
    Eigen::Vector3d p1 = mp.step(dt);
    EXPECT_NEAR((p1 - (p0 + v0 * dt)).norm(), 0.0, 5e-3);
    EXPECT_NEAR((mp.velocity() - v0).norm(), 0.0, 0.15);

    // Finite-difference velocity estimate matches the reported one.
    Eigen::Vector3d fd_vel = (p1 - p0) / dt;
    EXPECT_NEAR((fd_vel - v0).norm(), 0.0, 0.1);
}

/**
 * @brief resetIntegrationState() must be applied on every setInitialConditions():
 *        two consecutive rollouts (identical, and interleaved with a different
 *        one) must be bit-for-bit reproducible, i.e. no stale accumulated
 *        integral leaks across rollouts.
 */
TEST(ProDMPCoreTest, ResetIntegrationStateOnRepeatedRollouts) {
    const int N = 150;
    const double dt = 0.01;

    Eigen::Vector3d p_start(0.0, 0.0, 0.0);
    Eigen::Vector3d p_goal(0.2, -0.3, 0.1);
    auto demo = makeMinJerkDemo(p_start, p_goal, N, dt);

    ProDMP mp(18);
    mp.learnFromDemonstration(demo);

    Eigen::Vector3d p0_a(0.0, 0.0, 0.0);
    Eigen::Vector3d v0_a(0.0, 0.0, 0.0);
    Eigen::Vector3d p0_b(0.3, 0.2, -0.1);
    Eigen::Vector3d v0_b(-0.5, 0.1, 0.25);

    auto traj_a1 = rollout(mp, 0.0, p0_a, v0_a, N, dt);
    auto traj_b = rollout(mp, 0.0, p0_b, v0_b, N, dt);   // contaminating run in between
    auto traj_a2 = rollout(mp, 0.0, p0_a, v0_a, N, dt);

    (void)traj_b;
    ASSERT_EQ(traj_a1.size(), traj_a2.size());
    double max_diff = 0.0;
    for (size_t i = 0; i < traj_a1.size(); ++i) {
        max_diff = std::max(max_diff, (traj_a1[i] - traj_a2[i]).norm());
    }
    EXPECT_NEAR(max_diff, 0.0, 1e-12);
}

/**
 * @brief relative_goal mode: the goal parameter is stored relative to init_pos,
 *        setGoal still generalizes to a new absolute target, and the rollout
 *        converges there.
 */
TEST(ProDMPCoreTest, RelativeGoalGeneralization) {
    const int N = 200;
    const double dt = 0.01;

    Eigen::Vector3d p_start(0.2, 0.1, -0.1);
    Eigen::Vector3d p_goal_demo(0.5, -0.2, 0.3);
    auto demo = makeMinJerkDemo(p_start, p_goal_demo, N, dt);

    ProDMP mp(20);
    mp.setRelativeGoal(true);
    mp.learnFromDemonstration(demo);
    EXPECT_TRUE(mp.relativeGoal());

    // Learned absolute goal recovers the demonstrated end.
    EXPECT_NEAR((mp.goal() - p_goal_demo).norm(), 0.0, 1e-2);

    Eigen::Vector3d p_goal_new(0.8, 0.4, 0.6);
    mp.setInitialConditions(0.0, p_start, Eigen::Vector3d::Zero());
    mp.setGoal(p_goal_new);

    Eigen::Vector3d pos = mp.position();
    for (int i = 0; i < N - 1; ++i) pos = mp.step(dt);
    EXPECT_NEAR((pos - p_goal_new).norm(), 0.0, 0.04);
}

/**
 * @brief demoDisplacement() must equal goal() - initPos() for known values, both when the goal
 *        is stored absolute (relative_goal=false, e.g. loaded straight from weights.yaml) and
 *        when it is stored relative to init_pos_ - the same physical displacement either way.
 */
TEST(ProDMPCoreTest, DemoDisplacementIsGoalMinusInitPos) {
    const Eigen::Vector3d init_pos(0.2, 0.1, -0.1);
    const Eigen::Vector3d goal_param(0.5, -0.2, 0.3);  // absolute, matches relative_goal=false
    const std::array<Eigen::VectorXd, 3> weights = {Eigen::VectorXd::Zero(10), Eigen::VectorXd::Zero(10),
                                                     Eigen::VectorXd::Zero(10)};
    Eigen::VectorXd centers(10), widths(10);
    centers.setLinSpaced(10, 0.05, 1.0);
    widths.setConstant(10, 20.0);

    ProDMP mp_abs(10);
    mp_abs.setLearnedParameters(1.0, init_pos, Eigen::Vector3d::Zero(), goal_param, centers, widths,
                                weights, /*relative_goal=*/false);
    EXPECT_NEAR((mp_abs.goal() - goal_param).norm(), 0.0, 1e-12);  // absolute goal == goal_param as loaded
    EXPECT_NEAR((mp_abs.demoDisplacement() - (goal_param - init_pos)).norm(), 0.0, 1e-12);

    ProDMP mp_rel(10);
    mp_rel.setLearnedParameters(1.0, init_pos, Eigen::Vector3d::Zero(), goal_param, centers, widths,
                                weights, /*relative_goal=*/true);
    EXPECT_NEAR((mp_rel.goal() - (init_pos + goal_param)).norm(), 0.0, 1e-12);
    // Same physical displacement is recovered regardless of the internal storage convention.
    EXPECT_NEAR((mp_rel.demoDisplacement() - goal_param).norm(), 0.0, 1e-12);
}

/**
 * @brief Regression for the anchorAndRotate() ~192mm bug (satellite rotation frozen/continuous
 *        branches, prodmp_gazebo_executor_node.cpp): at theta=0 (no rotation), anchoring the demo
 *        via ee_anchor + ProDMP::demoDisplacement() and feeding it through
 *        core::satellite_intercept::anchorAndRotate() must land on the SAME absolute goal as the
 *        pure baseline path (relativeGoal=true, init_pos_=demo start, setGoal(goal)) - both
 *        express the identical physical goal.
 *
 *        Both models are seeded directly via setLearnedParameters() (as prodmp_io's YAML loader
 *        does) rather than fit from a demonstration via learnFromDemonstration(): a least-squares
 *        fit of a synthetic (e.g. min-jerk) trajectory against a finite RBF basis has a nonzero
 *        residual on the fitted goal by construction, which would swamp the ~mm-scale invariant
 *        this test checks with fitting noise unrelated to the anchorAndRotate() bug (baseline's
 *        setGoal() call assigns the goal parameter exactly, while a fitted satellite model's
 *        goal() carries that residual - comparing the two would not be an apples-to-apples check).
 *        Loading identical, exact parameters into both models instead makes this an exact
 *        algebraic identity: only float rounding remains, hence the tight tolerance.
 */
TEST(ProDMPCoreTest, AnchorAndRotateAtThetaZeroMatchesBaselineGoal) {
    using haptic_dmp_learning::core::satellite_intercept::anchorAndRotate;

    const Eigen::Vector3d demo_init_pos(0.3, -0.1, 0.2);
    const Eigen::Vector3d demo_goal(0.55, 0.05, 0.45);  // absolute, as loaded from weights.yaml
    const std::array<Eigen::VectorXd, 3> weights = {Eigen::VectorXd::Zero(10), Eigen::VectorXd::Zero(10),
                                                     Eigen::VectorXd::Zero(10)};
    Eigen::VectorXd centers(10), widths(10);
    centers.setLinSpaced(10, 0.05, 1.0);
    widths.setConstant(10, 20.0);

    // Baseline: relativeGoal=true, setGoal(demo_goal) from demo_init_pos_ - the one trusted place
    // that computes (new_goal - init_pos_).
    ProDMP baseline(10);
    baseline.setLearnedParameters(1.0, demo_init_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                  centers, widths, weights, /*relative_goal=*/true);
    baseline.setInitialConditions(0.0, demo_init_pos, Eigen::Vector3d::Zero());
    baseline.setGoal(demo_goal);
    const Eigen::Vector3d baseline_absolute_goal = baseline.goal();

    // Satellite branch: model loaded exactly as from weights.yaml (relative_goal=false, goal_param
    // is the absolute demo_goal). ee_anchor happens to equal demo_init_pos_ here (the "no
    // re-anchoring" case), anchored via the fixed demoDisplacement() and rotated by theta=0.
    ProDMP satellite(10);
    satellite.setLearnedParameters(1.0, demo_init_pos, Eigen::Vector3d::Zero(), demo_goal, centers, widths,
                                   weights, /*relative_goal=*/false);
    const Eigen::Vector3d ee_anchor = demo_init_pos;
    const auto anchored = anchorAndRotate(ee_anchor, satellite.demoDisplacement(), Eigen::Vector3d::Zero(),
                                          Eigen::Vector3d::UnitZ(), /*theta_rad=*/0.0);

    EXPECT_NEAR((anchored.p_rotated_world - baseline_absolute_goal).norm(), 0.0, 1e-9);
}

/**
 * @brief Degenerate Wronskian guard: a non-positive alpha makes W(0) collapse
 *        and setInitialConditions() must refuse rather than divide by ~0.
 */
TEST(ProDMPCoreTest, RejectsDegenerateWronskian) {
    ProDMP mp(10, /*alpha=*/0.0);
    EXPECT_THROW(mp.setInitialConditions(0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
                 std::runtime_error);
}

/**
 * @brief YAML round-trip: save a trained ProDMP, load it back, and check that
 *        weights, goal, tau, centers and widths survive; the reloaded model must
 *        reproduce the same rollout.
 */
TEST(ProDMPCoreTest, YamlRoundtrip) {
    const int N = 160;
    const double dt = 0.01;

    Eigen::Vector3d p_start(0.0, 0.1, 0.0);
    Eigen::Vector3d p_goal(0.3, -0.1, 0.2);
    auto demo = makeMinJerkDemo(p_start, p_goal, N, dt);

    ProDMP mp(16);
    mp.learnFromDemonstration(demo);

    const std::string path = "/tmp/test_prodmp_roundtrip.yaml";
    ASSERT_NO_THROW(prodmp_io::saveProDmpToYaml(mp, path));

    ProDMP loaded = prodmp_io::loadProDmpFromYaml(path);
    std::remove(path.c_str());

    EXPECT_EQ(loaded.numBasis(), mp.numBasis());
    EXPECT_NEAR(loaded.tau(), mp.tau(), 1e-12);
    EXPECT_NEAR((loaded.goal() - mp.goal()).norm(), 0.0, 1e-9);
    for (int d = 0; d < 3; ++d) {
        for (int i = 0; i < mp.numBasis(); ++i) {
            EXPECT_NEAR(loaded.weights()[d](i), mp.weights()[d](i), 1e-12);
        }
    }

    auto traj_ref = rollout(mp, 0.0, p_start, Eigen::Vector3d::Zero(), N, dt);
    auto traj_new = rollout(loaded, 0.0, p_start, Eigen::Vector3d::Zero(), N, dt);
    double max_diff = 0.0;
    for (int i = 0; i < N; ++i) max_diff = std::max(max_diff, (traj_ref[i] - traj_new[i]).norm());
    EXPECT_NEAR(max_diff, 0.0, 1e-9);
}
