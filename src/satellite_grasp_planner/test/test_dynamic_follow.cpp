// Essential tests of the L2 forward dynamics (core/dynamic_follow.hpp). They need Pinocchio and the URDF fixture. The parity with
// inertial_pred.py on the 36 runs is NOT here: it needs the campaign data, see analysis/dynamic_follow_parity.cpp.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "satellite_grasp_planner/core/dynamic_follow.hpp"
#include "test_fixtures.hpp"

using namespace satellite_grasp_planner::core;
namespace tf = satellite_grasp_planner::test_fixtures;

#define REQUIRE_URDF_OR_SKIP()                                      \
  do {                                                              \
    if (!std::ifstream(tf::urdfPath()).good()) {                    \
      GTEST_SKIP() << "URDF fixture not found: " << tf::urdfPath(); \
    }                                                               \
  } while (0)

namespace {

using JV = RobotModel::JointVector;

std::vector<std::string> jointNames() {
  std::vector<std::string> n;
  for (int i = 1; i <= 7; ++i) n.push_back("fer_joint" + std::to_string(i));
  return n;
}

struct Scene {
  std::shared_ptr<RobotModel> robot = tf::loadPandaRobotModel();
  JointDynamics dyn = loadJointDynamicsFromUrdfFile(tf::urdfPath(), jointNames());
  JV q0 = RobotModel::readyPose();
  Scene() { robot->setPassiveJointPositions({0.04}); }
  /// Pose of the TCP at q0 as a fixed target.
  DynamicFollowTarget holdTarget(const Eigen::Vector3d& shift = Eigen::Vector3d::Zero()) {
    robot->update(q0, JV::Zero());
    CartesianSample s;
    s.position = robot->eePosition() + shift;
    s.orientation = robot->eeOrientation();
    return [s](double) { return s; };
  }
};

}  // namespace

TEST(DynamicFollow, UrdfDynamicsAreRead) {
  REQUIRE_URDF_OR_SKIP();
  const JointDynamics d = loadJointDynamicsFromUrdfFile(tf::urdfPath(), jointNames());
  EXPECT_NEAR(d.damping(0), 0.003, 1e-12);  // README of the Python: damping 0.003, friction 0.2 on the 7 joints
  EXPECT_NEAR(d.friction(6), 0.2, 1e-12);
  EXPECT_THROW(parseJointDynamicsFromUrdfXml("<robot><joint name=\"fer_joint1\" type=\"revolute\"></joint></robot>", jointNames()),
               std::runtime_error);
}

// Slew: after the (unlimited) first cycle no torque changes by more than delta_tau_max between two consecutive cycles. The target
// jumps by 0.2 m at t = 10 ms, so the demanded torque changes far faster than 0.5 Nm per cycle and the limit is the active one.
TEST(DynamicFollow, TorqueSlewIsLimitedFromTheSecondCycle) {
  REQUIRE_URDF_OR_SKIP();
  Scene sc;
  DynamicFollowParams p;
  p.delta_tau_max = 0.5;
  DynamicFollowSimulator sim(sc.robot, sc.dyn, p);
  const DynamicFollowTarget hold = sc.holdTarget();
  const DynamicFollowTarget jump = [hold](double t) {
    CartesianSample s = hold(t);
    if (t >= 0.0095) s.position += Eigen::Vector3d(0.2, 0.0, 0.0);
    return s;
  };
  std::vector<JV> tau;
  const DynamicFollowObserver obs = [&tau](const DynamicFollowStepInfo& s) { tau.push_back(s.tau); };
  sim.simulate(sc.q0, JV::Zero(), sc.q0, jump, 0.05, &obs);
  ASSERT_GE(tau.size(), 40u);
  double max_step = 0.0;
  for (std::size_t i = 1; i < tau.size(); ++i) max_step = std::max(max_step, (tau[i] - tau[i - 1]).cwiseAbs().maxCoeff());
  EXPECT_LE(max_step, p.delta_tau_max + 1e-12);
  EXPECT_GT(max_step, 0.99 * p.delta_tau_max) << "the limit should be active in this scenario";
}

// Friction: odd function of v, same sign as v, zero at rest, d v + f_c far from the regularization, switched off by the flags.
TEST(DynamicFollow, FrictionSignAndSymmetry) {
  REQUIRE_URDF_OR_SKIP();
  Scene sc;
  DynamicFollowParams p;
  p.friction_tanh_eps_rad_s = 0.1;
  DynamicFollowSimulator sim(sc.robot, sc.dyn, p);
  JV v;
  v << 0.3, -0.02, 0.0, 1.0, -2.0, 0.05, -0.001;
  const JV f = sim.frictionTorque(v);
  const JV fm = sim.frictionTorque(-v);
  EXPECT_LT((f + fm).cwiseAbs().maxCoeff(), 1e-15);  // odd
  for (int j = 0; j < 7; ++j) {
    if (v(j) > 0.0) {
      EXPECT_GT(f(j), 0.0);
    }
    if (v(j) < 0.0) {
      EXPECT_LT(f(j), 0.0);
    }
    if (v(j) == 0.0) {
      EXPECT_EQ(f(j), 0.0);
    }
  }
  // |v| >> eps: d v + f_c (tanh -> 1)
  EXPECT_NEAR(f(4), sc.dyn.damping(4) * -2.0 - sc.dyn.friction(4), 1e-8);
  EXPECT_NEAR(f(3), sc.dyn.damping(3) * 1.0 + sc.dyn.friction(3), 1e-8);
  // flags
  p.apply_urdf_friction = false;
  p.apply_urdf_damping = false;
  EXPECT_EQ(DynamicFollowSimulator(sc.robot, sc.dyn, p).frictionTorque(v).cwiseAbs().maxCoeff(), 0.0);
}

// Gravity-free plant and fixed target at the current pose, rest: nothing moves (inertial_pred.py selftest T2), and the run is
// reproducible bit for bit.
TEST(DynamicFollow, StillCaseNoGravityIsStationaryAndReproducible) {
  REQUIRE_URDF_OR_SKIP();
  Scene sc;
  DynamicFollowParams p;
  p.plant_gravity_z_m_s2 = 0.0;
  p.compensate_gravity_internally = false;
  DynamicFollowSimulator sim(sc.robot, sc.dyn, p);
  const DynamicFollowTarget tgt = sc.holdTarget();
  const DynamicFollowResult r1 = sim.simulate(sc.q0, JV::Zero(), sc.q0, tgt, 0.5);
  const DynamicFollowResult r2 = sim.simulate(sc.q0, JV::Zero(), sc.q0, tgt, 0.5);
  ASSERT_FALSE(r1.diverged);
  ASSERT_EQ(r1.q.size(), 501u);
  EXPECT_LT((r1.q.back() - sc.q0).cwiseAbs().maxCoeff() * 180.0 / M_PI, 1e-3);  // deg
  EXPECT_TRUE(r1.q.back() == r2.q.back());                                      // deterministic
  EXPECT_EQ(r1.min_margin_rad, r2.min_margin_rad);
  // margin of a configuration at rest = the margin of q0
  const JV lo = sc.robot->jointLowerLimits(), hi = sc.robot->jointUpperLimits();
  EXPECT_NEAR(r1.min_margin_rad, (sc.q0 - lo).cwiseMin(hi - sc.q0).minCoeff(), 1e-9);
}

// Plant gravity = model gravity and compensation on: the gravity torque cancels exactly, the arm holds.
TEST(DynamicFollow, StillCaseCompensatedGravityHolds) {
  REQUIRE_URDF_OR_SKIP();
  Scene sc;
  DynamicFollowParams p;
  p.plant_gravity_z_m_s2 = p.model_gravity_z_m_s2;
  p.compensate_gravity_internally = true;
  DynamicFollowSimulator sim(sc.robot, sc.dyn, p);
  const DynamicFollowResult r = sim.simulate(sc.q0, JV::Zero(), sc.q0, sc.holdTarget(), 0.5);
  ASSERT_FALSE(r.diverged);
  EXPECT_LT((r.q.back() - sc.q0).cwiseAbs().maxCoeff() * 180.0 / M_PI, 1e-3);
}

// Explosion forced with artificial parameters: enormous gains, slew limit effectively off and a step of 10 ms (omega dt far above the
// stability limit of the explicit scheme). Whatever the way it blows up (non-finite or beyond divergence_rad), the result is flagged.
TEST(DynamicFollow, ArtificialExplosionIsFlagged) {
  REQUIRE_URDF_OR_SKIP();
  Scene sc;
  DynamicFollowParams p;
  p.translational_stiffness = 1e7;
  p.rotational_stiffness = 1e6;
  p.delta_tau_max = 1e9;
  p.dt_s = 0.01;
  DynamicFollowSimulator sim(sc.robot, sc.dyn, p);
  const DynamicFollowTarget tgt = sc.holdTarget(Eigen::Vector3d(0.05, 0.0, 0.0));
  const DynamicFollowResult r = sim.simulate(sc.q0, JV::Zero(), sc.q0, tgt, 1.0);
  EXPECT_TRUE(r.diverged);
  EXPECT_TRUE(std::isfinite(r.t_div_s));
  EXPECT_GT(r.t_div_s, 0.0);
  EXPECT_LE(r.t_div_s, 1.0 + 1e-9);
  EXPECT_LT(r.steps_done, 100);  // aborted
}

// The excursion guard alone. The reference run has the guard off (divergence_rad = 1e300) and must be finite; its largest excursion
// E = max_k max_j |q_k - q0| is measured, and the thresholds are taken relative to it: 0.5 E must flag the run at the first step
// beyond it (aborted before that row is stored, rows identical to the reference up to there), 2 E must change nothing.
TEST(DynamicFollow, ExcursionGuardFlagsAndLargeThresholdChangesNothing) {
  REQUIRE_URDF_OR_SKIP();
  Scene sc;
  const DynamicFollowTarget tgt = sc.holdTarget(Eigen::Vector3d(0.02, 0.0, 0.0));

  DynamicFollowParams pb;
  pb.divergence_rad = 1e300;
  const DynamicFollowResult rb = DynamicFollowSimulator(sc.robot, sc.dyn, pb).simulate(sc.q0, JV::Zero(), sc.q0, tgt, 0.2);
  ASSERT_FALSE(rb.diverged);
  ASSERT_EQ(rb.q.size(), 201u);
  double excursion = 0.0;
  for (const JV& q : rb.q) excursion = std::max(excursion, (q - sc.q0).cwiseAbs().maxCoeff());
  std::cout << "[ excursion ] max |q - q0| of the unguarded run = " << excursion << " rad\n";
  ASSERT_GT(excursion, 0.0);

  DynamicFollowParams pt;
  pt.divergence_rad = 0.5 * excursion;
  const DynamicFollowResult rt = DynamicFollowSimulator(sc.robot, sc.dyn, pt).simulate(sc.q0, JV::Zero(), sc.q0, tgt, 0.2);
  ASSERT_TRUE(rt.diverged);
  EXPECT_GT(rt.t_div_s, 0.0);
  EXPECT_LE(rt.t_div_s, 0.2 + 1e-9);
  EXPECT_EQ(rt.q.size(), static_cast<std::size_t>(rt.steps_done) + 1u);
  EXPECT_LT(rt.q.size(), rb.q.size());
  EXPECT_TRUE(rt.q.back().allFinite());
  for (std::size_t k = 0; k < rt.q.size(); ++k) {
    EXPECT_TRUE(rt.q[k] == rb.q[k]) << "row " << k;                            // same trajectory up to the abort
    EXPECT_LE((rt.q[k] - sc.q0).cwiseAbs().maxCoeff(), pt.divergence_rad);     // stored rows are inside the guard
  }

  DynamicFollowParams pl;
  pl.divergence_rad = 2.0 * excursion;
  const DynamicFollowResult rl = DynamicFollowSimulator(sc.robot, sc.dyn, pl).simulate(sc.q0, JV::Zero(), sc.q0, tgt, 0.2);
  EXPECT_FALSE(rl.diverged);
  ASSERT_EQ(rl.q.size(), rb.q.size());
  EXPECT_TRUE(rl.q.back() == rb.q.back());
  EXPECT_EQ(rl.min_margin_rad, rb.min_margin_rad);
}

TEST(DynamicFollow, PoseSeriesIsZeroOrderHold) {
  const std::vector<double> t{1.0, 0.0, 2.0};  // unsorted on purpose
  const std::vector<Eigen::Vector3d> p{{1, 0, 0}, {0, 0, 0}, {2, 0, 0}};
  const std::vector<Eigen::Quaterniond> q(3, Eigen::Quaterniond::Identity());
  const PoseSeries s(t, p, q);
  EXPECT_EQ(s.at(-5.0).position.x(), 0.0);  // before the first sample: the first one
  EXPECT_EQ(s.at(0.999).position.x(), 0.0);
  EXPECT_EQ(s.at(1.0).position.x(), 1.0);
  EXPECT_EQ(s.at(1.5).position.x(), 1.0);
  EXPECT_EQ(s.at(9.0).position.x(), 2.0);
}

TEST(DynamicFollow, ParamsAreValidated) {
  DynamicFollowParams p;
  EXPECT_NO_THROW(validateDynamicFollowParams(p));
  p.friction_tanh_eps_rad_s = 0.0;
  EXPECT_THROW(validateDynamicFollowParams(p), std::invalid_argument);
  p = DynamicFollowParams();
  p.dt_s = -1.0;
  EXPECT_THROW(validateDynamicFollowParams(p), std::invalid_argument);
  p = DynamicFollowParams();
  p.finger_pos_m = 0.2;
  EXPECT_THROW(validateDynamicFollowParams(p), std::invalid_argument);
  p = DynamicFollowParams();
  p.divergence_rad = 0.0;
  EXPECT_THROW(validateDynamicFollowParams(p), std::invalid_argument);
}
