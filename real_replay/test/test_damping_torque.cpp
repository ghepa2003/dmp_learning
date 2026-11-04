// Torque-level tests of the per-axis damping against the unmodified CartesianImpedanceSolver.
//   test_damping_torque <config.yaml> <urdf>      (exit 77 = URDF not generated yet)

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <random>
#include <sstream>

#include "real_replay/config.hpp"
#include "real_replay/controller_core.hpp"

using namespace real_replay;
using JointVec = core::RobotModel::JointVector;

static int g_fail = 0;
static int g_checks = 0;
#define CHECK(cond, ...)                                              \
  do {                                                                \
    ++g_checks;                                                       \
    if (!(cond)) {                                                    \
      ++g_fail;                                                       \
      std::printf("  CHECK FAILED (line %d): %s  ", __LINE__, #cond); \
      std::printf(__VA_ARGS__);                                       \
      std::printf("\n");                                              \
    }                                                                 \
  } while (0)

namespace {

core::CartesianImpedanceSolver::Params paramsFrom(const GainsConfig& g, double dt_trans, double dt_rot) {
  core::CartesianImpedanceSolver::Params p;
  p.translational_stiffness = g.translational_stiffness;
  p.rotational_stiffness = g.rotational_stiffness;
  p.translational_damping = dt_trans;
  p.rotational_damping = dt_rot;
  p.nullspace_stiffness = g.nullspace_stiffness;
  p.joint1_nullspace_stiffness = g.joint1_nullspace_stiffness;
  p.nullspace_pinv_damping = g.nullspace_pinv_damping;
  p.delta_tau_max = 1e9;  // slew limiter out of the way
  p.compensate_gravity_internally = false;
  return p;
}

JointVec readyPlus(std::mt19937& rng, double amp) {
  std::uniform_real_distribution<double> u(-amp, amp);
  JointVec q = core::RobotModel::readyPose();
  for (int i = 0; i < 7; ++i) q(i) += u(rng);
  return q;
}

void testHelperEquivalence(const Config& cfg, const std::string& urdf) {
  std::printf("[solver with Dt/Dr  ==  solver with Dt=Dr=0  +  axisDampingTorque(D=(Dt,Dt,Dt,Dr,Dr,Dr))]\n");
  core::RobotModel model(urdf, cfg.robot.joint_names, cfg.robot.ee_frame);
  std::mt19937 rng(99);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  const double Dt = 37.0, Dr = 5.5;  // arbitrary, not the config values
  core::CartesianImpedanceSolver with_d(paramsFrom(cfg.gains, Dt, Dr));
  core::CartesianImpedanceSolver no_d(paramsFrom(cfg.gains, 0.0, 0.0));
  Vector6 D;
  D << Dt, Dt, Dt, Dr, Dr, Dr;

  double worst = 0.0;
  for (int n = 0; n < 500; ++n) {
    const JointVec q = readyPlus(rng, 0.4);
    JointVec dq;
    for (int i = 0; i < 7; ++i) dq(i) = 0.8 * u(rng);
    model.update(q, dq);
    core::CartesianError err;
    err.linear = Eigen::Vector3d(0.05 * u(rng), 0.05 * u(rng), 0.05 * u(rng));
    err.angular = Eigen::Vector3d(0.3 * u(rng), 0.3 * u(rng), 0.3 * u(rng));
    const JointVec qn = readyPlus(rng, 0.3);
    with_d.setNullspaceTarget(qn);
    no_d.setNullspaceTarget(qn);
    JointVec prev;
    for (int i = 0; i < 7; ++i) prev(i) = 3.0 * u(rng);

    const JointVec a = with_d.computeTorque(model, err, q, dq, prev);
    const JointVec b = no_d.computeTorque(model, err, q, dq, prev) + axisDampingTorque(model.jacobian(), dq, D);
    worst = std::max(worst, (a - b).lpNorm<Eigen::Infinity>());
  }
  std::printf("  500 random states: worst |difference| %.3e Nm\n", worst);
  CHECK(worst < 1e-9, "worst difference %.3e", worst);

  // The damping term itself: zero velocity -> zero torque; linear in D.
  model.update(readyPlus(rng, 0.1), JointVec::Zero());
  CHECK(axisDampingTorque(model.jacobian(), JointVec::Zero(), D).norm() == 0.0, "no velocity, no damping torque");
}

Config relaxedConfig(Config cfg, const std::vector<double>& override_d) {
  cfg.damping.override_d = override_d;
  cfg.gains.gain_ramp_s = 0.0;       // alpha = 1 from the first cycle
  cfg.gains.delta_tau_max = 1e9;
  cfg.safety.torque_limit_fraction = 1.0;
  for (double& e : cfg.safety.effort_limits_nm) e = 1e6;
  cfg.safety.max_torque_step_nm = 1e9;
  cfg.safety.max_joint_speed_rad_s = 1e3;
  cfg.safety.callback_budget_us = 1e9;
  cfg.safety.callback_budget_consecutive = 1000000;
  cfg.safety.period_abort_s = 1.0;
  return cfg;
}

void testLoopEquivalence(const Config& base, const std::string& urdf) {
  std::printf("[ReplayLoop torque with damping.override_d = (Dt,Dt,Dt,Dr,Dr,Dr) == solver with Dt/Dr]\n");
  const double Dt = 21.0, Dr = 6.5;
  const Config cfg = relaxedConfig(base, {Dt, Dt, Dt, Dr, Dr, Dr});
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> u(-1.0, 1.0);

  double worst = 0.0;
  for (int trial = 0; trial < 20; ++trial) {
    ReplayLoop loop(cfg, urdf, std::make_unique<HoldSource>(30.0));
    loop.setTrackingGuardEnabled(false);
    const JointVec q0 = readyPlus(rng, 0.3);
    Array7 qa{};
    for (int i = 0; i < 7; ++i) qa[static_cast<std::size_t>(i)] = q0(i);
    loop.prepareDamping(qa, nullptr, nullptr);

    LoopInput in;
    in.q = qa;
    in.period_s = 1e-3;
    LoopOutput out;
    loop.step(in, out);  // activation: target = activation pose

    JointVec q1 = q0, dq1;
    for (int i = 0; i < 7; ++i) {
      q1(i) += 0.01 * u(rng);
      dq1(i) = 0.5 * u(rng);
      in.q[static_cast<std::size_t>(i)] = q1(i);
      in.dq[static_cast<std::size_t>(i)] = dq1(i);
    }
    loop.step(in, out);

    // Reference: the unmodified solver with Dt/Dr on the same state, same target, same nullspace target.
    core::RobotModel ref(urdf, cfg.robot.joint_names, cfg.robot.ee_frame);
    ref.update(q1, dq1);
    const core::CartesianError err = core::computePoseError(ref.eePosition(), ref.eeOrientation(),
                                                            loop.activationPose().p, loop.activationPose().q);
    core::CartesianImpedanceSolver solver(paramsFrom(cfg.gains, Dt, Dr));
    solver.setNullspaceTarget(q0);
    const JointVec expected = solver.computeTorque(ref, err, q1, dq1, JointVec::Zero());
    double d = 0.0;
    for (int i = 0; i < 7; ++i) d = std::max(d, std::fabs(expected(i) - out.tau[static_cast<std::size_t>(i)]));
    worst = std::max(worst, d);
  }
  std::printf("  20 trials through ReplayLoop::step: worst |difference| %.3e Nm\n", worst);
  CHECK(worst < 1e-9, "worst difference %.3e", worst);
}

void testLoopBehaviour(const Config& base, const std::string& urdf) {
  std::printf("[ReplayLoop: designed damping respects the cap; drift guard; fallback design]\n");
  Config cfg = relaxedConfig(base, {});
  {
    ReplayLoop loop(cfg, urdf, std::make_unique<HoldSource>(30.0));
    loop.prepareDamping(cfg.dry_run.q, nullptr, nullptr);
    const DampingDesign& d = loop.dampingDesign();
    for (std::size_t i = 0; i < 6; ++i) CHECK(d.a[i] <= cfg.damping.max_damping_per_sample * (1.0 + 1e-12), "axis %zu a=%.4f", i, d.a[i]);
    CHECK(d.capped[5], "rz is capped at the ready pose (flange, no hand)");
    std::printf("  ready pose: rz D = %.4f (capped, a = %.3f, zeta = %.3f)\n", d.d[5], d.a[5], d.zeta[5]);
  }
  {  // activation drift: design made at q_a, control started at q_b
    ReplayLoop loop(cfg, urdf, std::make_unique<HoldSource>(30.0));
    loop.prepareDamping(cfg.dry_run.q, nullptr, nullptr);
    LoopInput in;
    in.q = cfg.dry_run.q;
    in.q[0] += 2.0 * cfg.damping.max_q_drift_rad;
    in.period_s = 1e-3;
    LoopOutput out;
    loop.step(in, out);
    CHECK(out.finished && loop.stopReason() == StopReason::kActivationDrift, "drift guard: %s", toString(loop.stopReason()));
    double max_tau = 0.0;
    for (double t : out.tau) max_tau = std::max(max_tau, std::fabs(t));
    CHECK(max_tau == 0.0, "zero torque on drift");
  }
  {  // fallback: no prepareDamping() call, the first cycle designs it
    ReplayLoop loop(cfg, urdf, std::make_unique<HoldSource>(30.0));
    LoopInput in;
    in.q = cfg.dry_run.q;
    in.period_s = 1e-3;
    LoopOutput out;
    loop.step(in, out);
    CHECK(loop.dampingPrepared() && !out.finished, "fallback design in the first cycle");
  }
  {  // stopping phase uses the stop damping with the cap re-applied: a_i <= cap on the stop values
    ReplayLoop loop(cfg, urdf, std::make_unique<HoldSource>(30.0));
    loop.prepareDamping(cfg.dry_run.q, nullptr, nullptr);
    const DampingDesign& d = loop.dampingDesign();
    for (std::size_t i = 0; i < 6; ++i) {
      CHECK(d.d_stop[i] * kNominalCycleS / d.lambda_min[i] <= cfg.damping.max_damping_per_sample * (1.0 + 1e-12), "stop a axis %zu", i);
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::fprintf(stderr, "usage: %s <config.yaml> <urdf>\n", argv[0]);
    return 2;
  }
  {
    std::ifstream probe(argv[2]);
    if (!probe) {
      std::printf("SKIP: URDF '%s' not found (generate it with scripts/gen_urdf_nohand.py)\n", argv[2]);
      return 77;
    }
  }
  try {
    const Config cfg = loadConfig(argv[1]);
    std::ifstream f(argv[2]);
    std::stringstream ss;
    ss << f.rdbuf();
    testHelperEquivalence(cfg, ss.str());
    testLoopEquivalence(cfg, ss.str());
    testLoopBehaviour(cfg, ss.str());
  } catch (const std::exception& e) {
    std::printf("EXCEPTION: %s\n", e.what());
    return 2;
  }
  std::printf("\ntest_damping_torque: %d check(s), %d failure(s)\n", g_checks, g_fail);
  return g_fail == 0 ? 0 : 1;
}
