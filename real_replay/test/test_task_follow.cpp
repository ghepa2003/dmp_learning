// Stage 3 (C, D, I): TaskSource follow target, reference velocity (friction feedforward input) in APPROACH, FOLLOW and
// the satellite stop ramp, stale-sample fallback, and the friction_ff configuration of the task config.
//   test_task_follow <weights.yaml> <task_robot1_hand_ff.yaml> <replay_..._ffa100.yaml> <task_robot1_hand.yaml>
#include <cmath>
#include <cstdio>
#include <string>

#include "real_replay/config.hpp"
#include "real_replay/satellite_phase.hpp"
#include "real_replay/task_runtime.hpp"

using namespace real_replay;
using namespace real_replay::task;

static int g_fail = 0;
static void check(bool ok, const std::string& what) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}
static std::string f(const char* fmt, double a = 0, double b = 0, double c = 0) {
  char buf[300];
  std::snprintf(buf, sizeof buf, fmt, a, b, c);
  return buf;
}
constexpr double D = sat::kDeg;
static double g_now = 1000.0;
static double fakeClock() { return g_now; }

static const Eigen::Vector3d kC(0.0, 0.72, 0.70);
static const Eigen::Vector3d kStart(0.000002, 0.432494, 0.772246);
static const Eigen::Quaterniond kQs(0.494585, -0.505353, -0.505353, -0.494593);
static const Eigen::Vector3d kGoal(-0.046991784, 0.516475991, 0.7);
static const Eigen::Quaterniond kQg(-0.34281217, 0.618449526, 0.34281217, 0.618449526);

static void publish(TaskShared& sh, double theta_wrapped, double omega_cmd, double t_stamp) {
  SatSample s;
  s.seq = 1;
  s.state = 2;
  s.t_mono_ns = static_cast<std::uint64_t>(t_stamp * 1e9);
  s.theta = sat::unwrapPhase(theta_wrapped);
  s.omega_cmd = omega_cmd;
  s.omega_valid = true;
  s.omega_meas = omega_cmd;
  s.rx_s = t_stamp;
  s.center[0] = kC.x();
  s.center[1] = kC.y();
  s.center[2] = kC.z();
  sh.sat_latest.store(s);
}

/// Freezes / releases the writer of a LatestValue mid-store (seq left odd), as if the rx thread were preempted inside
/// store(): every read fails with kContended meanwhile. Pointer to the protected member, formed through a derived class.
struct SeqAccess : sat::LatestValue<SatSample> {
  static std::atomic<std::uint64_t> sat::LatestValue<SatSample>::*seq() { return &SeqAccess::seq_; }
};
static void holdWriter(sat::LatestValue<SatSample>& v) { (v.*SeqAccess::seq()).fetch_add(1); }
static void releaseWriter(sat::LatestValue<SatSample>& v) { (v.*SeqAccess::seq()).fetch_add(1); }

/// Runs the approach to its end. publish_during = false: no datagram is stored during the approach (writer frozen).
static Pose runApproach(TaskSource& src, TaskShared& sh, double& theta, double omega, bool publish_during) {
  sh.source_cmd.store(static_cast<int>(SourceCmd::kApproach));
  Pose p = src.sample(0.0);
  long n = 0;
  while (!sh.approach_done.load() && n < 40000) {
    g_now += 0.001;
    theta += omega * 0.001;
    if (publish_during && n % 10 == 0) publish(sh, theta, omega, g_now);
    p = src.sample(0.001);
    ++n;
  }
  return p;
}

static void testFollowContention(const std::string& weights) {
  std::printf("TaskSource: seqlock read failing at the first FOLLOW cycle\n");
  const double omega = -2.0 * D;
  auto make = [&](TaskShared& sh) {
    auto ap = std::make_unique<ProDmpSource>(weights, "", 1.0, 0.0);
    ap->setGoal(kGoal, kQg);
    auto src = std::make_unique<TaskSource>(std::move(ap), sh, 100.0, &fakeClock, omega, 0.05);
    Pose act;
    act.p = kStart;
    act.q = kQs;
    src->start(act);
    return src;
  };
  {  // 1) valid copy kept during the approach, writer frozen exactly at the follow start -> follows (no FREEZE)
    TaskShared sh;
    auto src = make(sh);
    g_now = 1000.0;
    double theta = -122.0 * D;
    publish(sh, theta, omega, g_now);
    const Pose end = runApproach(*src, sh, theta, omega, true);
    holdWriter(sh.sat_latest);
    sh.source_cmd.store(static_cast<int>(SourceCmd::kFollow));
    const double t_f0 = g_now;
    Pose pf = end;
    for (int i = 0; i < 20; ++i) {  // 20 ms of failing reads
      g_now += 0.001;
      theta += omega * 0.001;
      pf = src->sample(0.001);
    }
    releaseWriter(sh.sat_latest);
    for (int i = 0; i < 2000; ++i) {
      g_now += 0.001;
      theta += omega * 0.001;
      if (i % 10 == 0) publish(sh, theta, omega, g_now);
      pf = src->sample(0.001);
    }
    const double dth = omega * (g_now - t_f0);
    check(sh.follow_freeze_reason.load() == 0 && sh.follow_start_kind.load() == static_cast<int>(FollowStartKind::kKeptCopy) &&
              sh.follow_read_failures.load() == 1,
          "(1) contention at the first follow cycle with a kept copy: FOLLOW started on the kept copy, not frozen");
    check(std::fabs(src->followDtheta() - dth) < 0.03 * D && !src->followStale(),
          f("    ... and it follows: dtheta after 2 s %.4f deg (expected %.4f)", src->followDtheta() / D, dth / D));
    check((pf.p - end.p).norm() > 1e-3, f("    ... the target moved %.2f mm from the end pose", (pf.p - end.p).norm() * 1e3));
  }
  {  // 2) no copy kept (writer frozen for the whole approach), released after 20 ms < follow_read_hold_s -> waits, then follows
    TaskShared sh;
    auto src = make(sh);
    g_now = 1000.0;
    double theta = -122.0 * D;
    publish(sh, theta, omega, g_now);
    holdWriter(sh.sat_latest);
    const Pose end = runApproach(*src, sh, theta, omega, false);
    sh.source_cmd.store(static_cast<int>(SourceCmd::kFollow));
    bool still = true;
    for (int i = 0; i < 20; ++i) {
      g_now += 0.001;
      theta += omega * 0.001;
      const Pose p = src->sample(0.001);
      still = still && (p.p - end.p).norm() < 1e-12;
    }
    const bool waiting = sh.follow_start_kind.load() == 0 && sh.follow_freeze_reason.load() == 0;
    releaseWriter(sh.sat_latest);
    publish(sh, theta, omega, g_now);
    g_now += 0.001;
    src->sample(0.001);
    check(still && waiting, "(2) no kept copy: holds the goal pose while the reads fail (20 ms), neither started nor frozen");
    check(sh.follow_start_kind.load() == static_cast<int>(FollowStartKind::kFresh) && sh.follow_freeze_reason.load() == 0 &&
              sh.follow_wait_s.load() > 0.015 && sh.follow_wait_s.load() < 0.05,
          f("    ... released before follow_read_hold_s: FOLLOW started on a fresh read after %.1f ms", sh.follow_wait_s.load() * 1e3));
  }
  {  // 3) no copy kept, reads failing beyond follow_read_hold_s -> FREEZE with reason kReadContention
    TaskShared sh;
    auto src = make(sh);
    g_now = 1000.0;
    double theta = -122.0 * D;
    publish(sh, theta, omega, g_now);
    holdWriter(sh.sat_latest);
    const Pose end = runApproach(*src, sh, theta, omega, false);
    sh.source_cmd.store(static_cast<int>(SourceCmd::kFollow));
    Pose p = end;
    for (int i = 0; i < 80; ++i) {
      g_now += 0.001;
      p = src->sample(0.001);
    }
    releaseWriter(sh.sat_latest);
    publish(sh, theta, omega, g_now);
    for (int i = 0; i < 100; ++i) {  // FREEZE is sticky: a later fresh sample does not restart the follow
      g_now += 0.001;
      p = src->sample(0.001);
    }
    Eigen::Vector3d v, w;
    src->twist(v, w);
    const int fr = sh.follow_freeze_reason.load();
    check(fr == static_cast<int>(FollowFreezeReason::kReadContention) && sh.follow_wait_s.load() > 0.05 && sh.follow_wait_s.load() < 0.053,
          std::string("(3) reads failing beyond follow_read_hold_s (50 ms): FREEZE, reason '") + toString(static_cast<FollowFreezeReason>(fr)) +
              f("' after %.1f ms", sh.follow_wait_s.load() * 1e3));
    check((p.p - end.p).norm() < 1e-12 && v.norm() == 0.0 && w.norm() == 0.0 && sh.follow_start_kind.load() == 0,
          "    ... target stays at the end pose, zero twist, FREEZE sticky");
  }
  {  // 4) never any sample: FREEZE at once, reason kNoSample (unchanged behaviour)
    TaskShared sh;
    auto src = make(sh);
    g_now = 1000.0;
    double theta = -122.0 * D;
    runApproach(*src, sh, theta, omega, false);
    sh.source_cmd.store(static_cast<int>(SourceCmd::kFollow));
    g_now += 0.001;
    src->sample(0.001);
    check(sh.follow_freeze_reason.load() == static_cast<int>(FollowFreezeReason::kNoSample), "(4) no sample ever stored: FREEZE at once (kNoSample)");
  }
  {  // 5) a kept copy older than kFollowFreshAgeS is not used: FREEZE kStaleSample (unchanged freshness rule)
    TaskShared sh;
    auto src = make(sh);
    g_now = 1000.0;
    double theta = -122.0 * D;
    publish(sh, theta, omega, g_now);
    runApproach(*src, sh, theta, omega, false);  // copy kept from the first approach cycle, then no new sample
    holdWriter(sh.sat_latest);
    sh.source_cmd.store(static_cast<int>(SourceCmd::kFollow));
    g_now += 0.001;
    src->sample(0.001);
    check(sh.follow_freeze_reason.load() == static_cast<int>(FollowFreezeReason::kStaleSample),
          "(5) contention, kept copy older than kFollowFreshAgeS: FREEZE (kStaleSample)");
  }
}

static void testFollow(const std::string& weights, double theta_end_deg) {
  std::printf("TaskSource: APPROACH -> FOLLOW (theta_end %.0f deg)\n", theta_end_deg);
  TaskShared sh;
  auto ap = std::make_unique<ProDmpSource>(weights, "", 1.0, 0.0);
  ap->setGoal(kGoal, kQg);
  const double omega = -2.0 * D;
  TaskSource src(std::move(ap), sh, 100.0, &fakeClock, omega);
  Pose act;
  act.p = kStart;
  act.q = kQs;
  src.start(act);
  g_now = 1000.0;
  double theta = theta_end_deg * D;
  publish(sh, theta, omega, g_now);
  sh.source_cmd.store(static_cast<int>(SourceCmd::kApproach));
  // ---- APPROACH: the reference twist (friction ff input) is non-zero in the middle and zero at the end
  double vmax = 0.0, vsum = 0.0;
  Pose p = src.sample(0.0);
  long n = 0;
  while (!src.done() && !sh.approach_done.load() && n < 40000) {
    g_now += 0.001;
    theta += omega * 0.001;
    if (n % 10 == 0) publish(sh, theta, omega, g_now);
    p = src.sample(0.001);
    Eigen::Vector3d v, w;
    src.twist(v, w);
    vmax = std::max(vmax, v.norm());
    vsum += v.norm();
    ++n;
  }
  check(sh.approach_done.load(), "approach completed");
  check(vmax > 0.01 && vsum / static_cast<double>(n) > 0.002,
        f("APPROACH: reference velocity non-zero (max %.4f m/s, mean %.4f m/s) -> friction ff has a velocity to act on", vmax, vsum / static_cast<double>(n)));
  check((p.p - kGoal).norm() < 1e-12, "approach ends exactly at the goal position");
  Eigen::Vector3d v0, w0;
  src.twist(v0, w0);
  check(v0.norm() == 0.0 && w0.norm() == 0.0, "twist is zero at the end of the approach");
  const Pose end = p;

  // ---- FOLLOW
  sh.source_cmd.store(static_cast<int>(SourceCmd::kFollow));
  g_now += 0.001;
  theta += omega * 0.001;
  publish(sh, theta, omega, g_now);
  Pose pf = src.sample(0.001);
  check((pf.p - end.p).norm() < 1e-9 && pf.q.angularDistance(end.q) < 1e-9, "follow starts at the end pose (no jump)");
  const double t_f0 = g_now;
  const double theta_f0 = theta;
  for (int i = 0; i < 5000; ++i) {  // 5 s
    g_now += 0.001;
    theta += omega * 0.001;
    if (i % 10 == 0) publish(sh, theta, omega, g_now);
    pf = src.sample(0.001);
  }
  const double dth = omega * (g_now - t_f0);
  const Eigen::Quaterniond rot(Eigen::AngleAxisd(dth, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d p_exp = kC + rot * (end.p - kC);
  const Eigen::Quaterniond q_exp = (rot * end.q).normalized();
  const double dth_src = src.followDtheta();
  check(std::fabs(dth_src - dth) < 0.03 * D, f("dtheta after 5 s = %.4f deg (expected %.4f, error < 0.03 deg incl. the 10 ms sample hold)", dth_src / D, dth / D));
  check((pf.p - p_exp).norm() < 1e-3 * 0.5, f("position = c + Rz(dtheta)(p_end - c), error %.4f mm (sample hold of up to 10 ms)", (pf.p - p_exp).norm() * 1e3));
  check(pf.q.angularDistance(q_exp) < 0.03 * D * 1.0 + 1e-9, "orientation = Rz(dtheta) q_end");
  check(std::fabs((pf.p - kC).head<2>().norm() - (end.p - kC).head<2>().norm()) < 1e-9 && std::fabs(pf.p.z() - end.p.z()) < 1e-12,
        "rigid: horizontal radius and height about the axis are conserved");
  Eigen::Vector3d v, w;
  src.twist(v, w);
  const double r_xy = (pf.p - kC).head<2>().norm();
  check(std::fabs(v.norm() - std::fabs(omega) * r_xy) < 1e-9 && std::fabs(w.z() - omega) < 1e-12,
        f("FOLLOW: |v| = |omega| r = %.5f m/s, w_z = omega (friction ff input)", v.norm()));
  check(!src.followStale(), "fresh samples: not stale");

  // ---- stale: the samples stop; the estimate continues at the nominal omega (open loop)
  const double dth_before = src.followDtheta();
  const double t_stop = g_now;
  for (int i = 0; i < 2000; ++i) {  // 2 s without samples
    g_now += 0.001;
    pf = src.sample(0.001);
  }
  const double dth_after = src.followDtheta();
  check(src.followStale(), "no sample for 2 s -> open-loop mode (> 0.5 s)");
  check(std::fabs((dth_after - dth_before) - omega * (g_now - t_stop)) < 0.03 * D,
        f("open loop: dtheta advanced %.3f deg in 2 s (expected %.3f, omega*t)", (dth_after - dth_before) / D, omega * (g_now - t_stop) / D));

  // ---- satellite stop ramp: omega_cmd goes to 0, the target follows the MEASURED phase and the reference velocity falls
  double th = theta + omega * (g_now - t_stop);  // the satellite kept moving meanwhile
  double prev_v = 1e9;
  bool mono = true;
  double dth_end_ramp = 0.0;
  const double T2 = 2.0;
  double d_ramp = 0.0;
  for (int i = 0; i < 2500; ++i) {  // 2.5 s
    g_now += 0.001;
    const double ts = (i + 1) * 0.001;
    const double y = std::min(1.0, ts / T2);
    const double rate = omega * (1.0 - (3 * y * y - 2 * y * y * y));  // smoothstep to 0
    d_ramp += rate * 0.001;
    if (i % 10 == 0) publish(sh, th + d_ramp, rate, g_now);
    pf = src.sample(0.001);
    src.twist(v, w);
    if (i > 100 && v.norm() > prev_v + 1e-7) mono = false;
    if (i > 100) prev_v = v.norm();
    dth_end_ramp = src.followDtheta();
  }
  src.twist(v, w);
  check(mono, "during the stop ramp |v_ref| is non-increasing");
  check(v.norm() < 1e-6, f("after the ramp |v_ref| = %.2e m/s (omega_cmd = 0)", v.norm()));
  check(!src.followStale(), "samples resumed: fresh again");
  // ---- FREEZE (machine: satellite STOPPED): target frozen, zero twist
  sh.source_cmd.store(static_cast<int>(SourceCmd::kFreeze));
  const Pose pz = src.sample(0.001);
  g_now += 1.0;
  const Pose pz2 = src.sample(0.001);
  src.twist(v, w);
  check((pz.p - pz2.p).norm() == 0.0 && v.norm() == 0.0 && w.norm() == 0.0, "FREEZE: target constant, zero twist");
  (void)dth_end_ramp;
  (void)theta_f0;
}

static void testFrictionConfig(const char* ff, const char* ffa100, const char* plain) {
  std::printf("friction_ff of the task config\n");
  const Config a = loadConfig(ff), b = loadConfig(ffa100), c = loadConfig(plain);
  check(a.friction_ff.enabled && std::fabs(a.friction_ff.alpha - 1.0) < 1e-12, "task_robot1_hand_ff: friction_ff enabled, alpha 1.0");
  bool same = std::fabs(a.friction_ff.eps_v_rad_s - b.friction_ff.eps_v_rad_s) < 1e-15 && std::fabs(a.friction_ff.dls_lambda - b.friction_ff.dls_lambda) < 1e-15;
  for (int i = 0; i < 7; ++i) same = same && a.friction_ff.tau_c[i] == b.friction_ff.tau_c[i] && a.friction_ff.b[i] == b.friction_ff.b[i];
  check(same, "eps_v 0.005, DLS 0.05, tau_c and b equal to the validated *_ffa100 config");
  check(std::fabs(a.friction_ff.tau_c[0] - 0.615) < 1e-12 && std::fabs(a.friction_ff.b[3] - 1.6) < 1e-12 && a.friction_ff.tau_c[4] == 0.0 && a.friction_ff.tau_c[5] == 0.0,
        "tau_c [0.615 0.361 0.361 0.562 0 0 0.154], b [2.83 1.89 2.71 1.6 0 0 0.18] (J5, J6 off)");
  check(a.gains.gain_ramp_s == b.gains.gain_ramp_s && std::fabs(a.gains.gain_ramp_initial - 0.1) < 1e-12 && std::fabs(a.gains.gain_ramp_s - 3.0) < 1e-12,
        "gain_alpha ramp 0.1 -> 1 over 3 s");
  check(!c.friction_ff.enabled, "the existing task_robot1_hand.yaml has NO friction feedforward (unchanged)");
}

int main(int argc, char** argv) {
  if (argc < 5) {
    std::fprintf(stderr, "usage: %s <weights> <task_ff.yaml> <ffa100.yaml> <task.yaml>\n", argv[0]);
    return 2;
  }
  try {
    testFollow(argv[1], -122.0);
    testFollow(argv[1], -265.0);  // crossing of the branch edge -270 deg: dtheta must stay small
    testFollowContention(argv[1]);
    testFrictionConfig(argv[2], argv[3], argv[4]);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "exception: %s\n", e.what());
    return 2;
  }
  std::printf("%s (%d failure(s))\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
