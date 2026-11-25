// Controlled stop and nullspace reference (F6 of the 2026-11-25 analysis).
//   test_stop_nullspace <config.yaml> <urdf>      (exit 77 = URDF not generated yet)
//
// safety.stop_nullspace_to_current = true: on entering the controlled stop the nullspace target becomes the measured q at that
// instant, so afterwards the nullspace torque depends on (q - q_stop), not on (q - q_activation). false (default): unchanged.
// The expected torque is recomputed here from the model Jacobian: tau_ns = N (K_ns a (q_ref - q)[J1 x joint1_ns] - 2 sqrt(K_ns a) dq...).
// Also checks the guard detail text of the guards (joint speed, joint limit, tracking, success rate).

#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include "real_replay/config.hpp"
#include "real_replay/controller_core.hpp"
#include "real_replay/safety.hpp"
#include "real_replay/target_source.hpp"

using namespace real_replay;
using JointVec = core::RobotModel::JointVector;

static int g_fail = 0;
static void check(bool ok, const std::string& what) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}

static JointVec toVec(const Array7& a) {
  JointVec v;
  for (int i = 0; i < 7; ++i) v(i) = a[static_cast<std::size_t>(i)];
  return v;
}
static Array7 toArr(const JointVec& v) {
  Array7 a{};
  for (int i = 0; i < 7; ++i) a[static_cast<std::size_t>(i)] = v(i);
  return a;
}

/// tau_ns = N (K_ns (q_ref - q) with joint 1 x joint1_ns  -  2 sqrt(K_ns) dq with joint 1 x 2 sqrt(joint1_ns)), alpha = 1
static JointVec expectedNullspace(const Config& cfg, core::RobotModel& m, const JointVec& qref, const JointVec& q, const JointVec& dq) {
  m.update(q, dq);
  const auto& J = m.jacobian();
  Eigen::Matrix<double, 6, 6> A = J * J.transpose();
  A.diagonal().array() += cfg.gains.nullspace_pinv_damping * cfg.gains.nullspace_pinv_damping;
  const Eigen::Matrix<double, 7, 7> N = Eigen::Matrix<double, 7, 7>::Identity() - J.transpose() * A.inverse() * J;
  JointVec qe = qref - q;
  qe(0) *= cfg.gains.joint1_nullspace_stiffness;
  JointVec dqe = dq;
  dqe(0) *= 2.0 * std::sqrt(cfg.gains.joint1_nullspace_stiffness);
  return N * (cfg.gains.nullspace_stiffness * qe - 2.0 * std::sqrt(cfg.gains.nullspace_stiffness) * dqe);
}

struct Result {
  JointVec target_before, target_after, tau_after, tau_expected_stop_ref, tau_expected_act_ref, q_stop, q_act;
  StopReason reason;
  std::string text;
};

static Result run(Config cfg, bool flag, const std::string& urdf) {
  cfg.safety.stop_nullspace_to_current = flag;
  cfg.gains.nullspace_stiffness = 0.2;
  cfg.gains.joint1_nullspace_stiffness = 100.0;
  cfg.trajectory.stop_at_fraction = 1.0;
  cfg.run.max_duration_s = 100.0;
  ReplayLoop loop(cfg, urdf, std::make_unique<HoldSource>(60.0));
  loop.setTrackingGuardEnabled(false);
  loop.prepareDamping(cfg.dry_run.q, nullptr, nullptr);
  volatile std::sig_atomic_t flag_stop = 0;
  loop.setExternalStopFlag(&flag_stop);

  const JointVec q_act = toVec(cfg.dry_run.q);
  JointVec q_stop = q_act;
  q_stop(0) -= 0.30;  // the arm has drifted away from the activation posture (joint 1 -17 deg, joint 3 +10 deg, joint 7 +12 deg)
  q_stop(2) += 0.18;
  q_stop(6) += 0.20;
  LoopInput in;
  in.period_s = 1e-3;
  in.success_rate = 1.0;
  LoopOutput out;
  in.q = toArr(q_act);
  for (int i = 0; i < 3500; ++i) loop.step(in, out);  // past the 3 s gain ramp, at the activation posture
  in.q = toArr(q_stop);
  for (int i = 0; i < 100; ++i) loop.step(in, out);   // drifted, still RUNNING
  Result r;
  r.q_act = q_act;
  r.q_stop = q_stop;
  r.target_before = loop.nullspaceTarget();
  flag_stop = 1;                                      // controlled stop requested while the arm is at q_stop
  loop.step(in, out);
  r.target_after = loop.nullspaceTarget();
  // after the stop the arm keeps moving: q2 = q_stop + delta (still inside the limits), dq = 0 to isolate the spring
  JointVec q2 = q_stop;
  q2(4) += 0.15;
  q2(6) -= 0.10;
  in.q = toArr(q2);
  loop.step(in, out);
  r.tau_after = loop.lastNullspaceTorque();
  core::RobotModel m(urdf, cfg.robot.joint_names, cfg.robot.ee_frame);
  r.tau_expected_stop_ref = expectedNullspace(cfg, m, q_stop, q2, JointVec::Zero());
  r.tau_expected_act_ref = expectedNullspace(cfg, m, q_act, q2, JointVec::Zero());
  r.reason = loop.stopReason();
  r.text = loop.stopText();
  return r;
}

int main(int argc, char** argv) {
  if (argc != 3) {
    std::fprintf(stderr, "usage: test_stop_nullspace <config.yaml> <urdf>\n");
    return 2;
  }
  {
    std::ifstream u(argv[2]);
    if (!u.good()) {
      std::printf("SKIP: URDF %s not found\n", argv[2]);
      return 77;
    }
  }
  std::ifstream f(argv[2]);
  std::stringstream ss;
  ss << f.rdbuf();
  const Config cfg = loadConfig(argv[1]);

  std::printf("default: safety.stop_nullspace_to_current is OFF\n");
  check(!cfg.safety.stop_nullspace_to_current, "loadConfig default false");

  std::printf("flag OFF: the nullspace target stays at the activation posture\n");
  const Result off = run(cfg, false, ss.str());
  check(off.reason == StopReason::kUserSignal, "stop entered by the user signal");
  check((off.target_after - off.q_act).norm() < 1e-12, "target after the stop == q at activation");
  check((off.tau_after - off.tau_expected_act_ref).norm() < 1e-9, "nullspace torque == N K (q_act - q)");
  check((off.tau_after - off.tau_expected_stop_ref).norm() > 1e-2, "... and differs from the one with q_ref = q_stop (the feature matters)");

  std::printf("flag ON: the nullspace target becomes the measured q at the stop instant\n");
  const Result on = run(cfg, true, ss.str());
  check((on.target_before - on.q_act).norm() < 1e-12, "before the stop: target == q at activation");
  check((on.target_after - on.q_stop).norm() < 1e-12, "at the stop: target == measured q (|diff| < 1e-12)");
  check((on.tau_after - on.tau_expected_stop_ref).norm() < 1e-9, "after the stop: nullspace torque == N K (q_stop - q), depends on q - q_stop");
  check((on.tau_after - on.tau_expected_act_ref).norm() > 1e-2, "... and NOT on q - q_activation");
  check(on.tau_after.norm() < off.tau_after.norm(), "torque norm smaller than with the activation reference (the posture spring no longer pulls 17 deg of joint 1)");

  std::printf("stop text of the guards (snprintf, no allocation)\n");
  {
    SafetyConfig sc = cfg.safety;
    sc.joint_limit_margin_rad = 0.1;
    sc.max_joint_speed_rad_s = 1.0;
    sc.joint_speed_consecutive = 3;
    Array7 lo, hi;
    for (int i = 0; i < 7; ++i) {
      lo[static_cast<std::size_t>(i)] = -2.0;
      hi[static_cast<std::size_t>(i)] = 2.0;
    }
    SafetyGuard g(sc, lo, hi);
    Array7 q{}, dq{};
    dq[4] = -1.2;
    dq[1] = 0.5;
    StopReason r = StopReason::kNone;
    for (int i = 0; i < 3; ++i) r = g.checkState(q, dq, 0.0, 0.0);
    char b[256];
    formatStopReason(b, sizeof b, r, g.lastDetail());
    std::printf("    %s\n", b);
    check(r == StopReason::kJointSpeed && std::strstr(b, "joint 5") && std::strstr(b, "1.200") && std::strstr(b, "3 consecutive"), "joint speed: joint 5, 1.200 rad/s, 3 consecutive cycles");
    SafetyGuard g2(sc, lo, hi);
    Array7 q2{}, dz{};
    q2[6] = 1.95;
    r = g2.checkState(q2, dz, 0.0, 0.0);
    formatStopReason(b, sizeof b, r, g2.lastDetail());
    std::printf("    %s\n", b);
    check(r == StopReason::kJointLimit && std::strstr(b, "joint 7") && std::strstr(b, "upper"), "joint limit: joint 7, upper bound");
    SafetyGuard g3(sc, lo, hi);
    r = StopReason::kNone;
    for (int i = 0; i < sc.tracking_consecutive; ++i) r = g3.checkState(Array7{}, Array7{}, 0.05, 0.0);
    formatStopReason(b, sizeof b, r, g3.lastDetail());
    std::printf("    %s\n", b);
    check(r == StopReason::kTrackingError && std::strstr(b, "position") && std::strstr(b, "50"), "tracking: position error 50 mm");
    SafetyGuard g4(sc, lo, hi);
    r = StopReason::kNone;
    for (int i = 0; i < sc.success_rate_consecutive + 1; ++i) r = g4.checkCycle(1e-3, 10.0, 0.5, sc.watchdog_grace_cycles + 1);
    formatStopReason(b, sizeof b, r, g4.lastDetail());
    std::printf("    %s\n", b);
    check(r == StopReason::kSuccessRate && std::strstr(b, "0.500"), "success rate: 0.500 below the minimum");
  }

  std::printf("%s (%d failure(s))\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
