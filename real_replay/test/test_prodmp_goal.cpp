// Equivalence of ProDmpSource in GOAL mode (setGoal before start) with the simulation executor's chain.
//   test_prodmp_goal <weights.yaml>
//
// Reference = the library itself, used exactly as prodmp_gazebo_executor_node.cpp startContinuousRollout() /
// stepCallback() do in grasp_command mode with grasp_orientation_mode = planner_goal:
//   ProDMP: setRelativeGoal(true); setInitialConditions(0, p_start, 0); setGoal(p_goal); step(dt) per tick,
//           first tick dt = 0 (MeasuredTimeBase), at elapsed >= tau the goal is published.
//   orientation: slerpAt(q_start, alignShortestPath(q_start, q_goal), elapsed, tau).
// The reference is rolled out at 1 ms, 5 ms (executor dt) and 0.1 ms; ProDmpSource at 1 ms (1 kHz loop).
// Criterion (whole trajectory, compared at the common instants): max |dpos| <= 0.1 mm, max |dangle| <= 0.1 deg;
// at t = tau: position and orientation EXACTLY the goal, twist zero.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "haptic_dmp_learning/core/grasp_orientation.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "haptic_dmp_learning/core/quaternion_dmp.hpp"
#include "real_replay/prodmp_source.hpp"

using namespace real_replay;
namespace hcore = haptic_dmp_learning::core;
namespace go = haptic_dmp_learning::core::grasp_orientation;

static int g_fail = 0;
static void check(bool ok, const std::string& what) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}

struct Sample {
  double t;
  Eigen::Vector3d p;
  Eigen::Quaterniond q;
};

/// Executor chain at step dt; samples at t = k*dt (k = 0 .. until elapsed >= tau, last = goal).
static std::vector<Sample> reference(const std::string& weights, const Eigen::Vector3d& p0, const Eigen::Quaterniond& q0,
                                     const Eigen::Vector3d& pg, const Eigen::Quaterniond& qg, double dt) {
  hcore::QuaternionDMP qd(20, 4.6, 25.0, 6.25);
  bool has_o = false;
  hcore::ProDMP pd = hcore::prodmp_io::loadProDmpFromYaml(weights, qd, has_o);
  pd.setRelativeGoal(true);
  pd.setInitialConditions(0.0, p0, Eigen::Vector3d::Zero());
  pd.setGoal(pg);
  const double tau = pd.tau();
  const Eigen::Quaterniond qga = go::alignShortestPath(q0, qg);
  std::vector<Sample> out;
  const long n = static_cast<long>(std::ceil(tau / dt - 1e-9));
  double prev = 0.0;
  for (long k = 0; k <= n; ++k) {
    const double t = std::min(tau, static_cast<double>(k) * dt);  // MeasuredTimeBase elapsed
    if (t >= tau) {
      out.push_back({tau, pd.goal(), qga});
      break;
    }
    const Eigen::Vector3d p = pd.step(t - prev);  // first tick: dt_step = 0
    prev = t;
    out.push_back({t, p, go::slerpAt(q0, qga, t, tau)});
  }
  return out;
}

static double angDeg(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
  return a.angularDistance(b) * 180.0 / M_PI;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <weights.yaml>\n", argv[0]);
    return 2;
  }
  const std::string W = argv[1];
  std::printf("weights (TEST data, not the nominal demo): %s\n", W.c_str());

  // activation pose: TCP of the robot-1 start configuration (approximately), hand pointing down
  const Eigen::Vector3d p_act(0.0, 0.41, 0.45);
  const Eigen::Quaterniond q_act = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())) *
                                   Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI / 4, Eigen::Vector3d::UnitZ()));
  hcore::QuaternionDMP qd(20, 4.6, 25.0, 6.25);
  bool has_o = false;
  const hcore::ProDMP tmpl = hcore::prodmp_io::loadProDmpFromYaml(W, qd, has_o);
  const Eigen::Vector3d disp = tmpl.demoDisplacement();
  std::printf("tau %.6f s, demo displacement [%.4f %.4f %.4f] m\n", tmpl.tau(), disp.x(), disp.y(), disp.z());

  struct Goal {
    const char* name;
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
  };
  const std::vector<Goal> goals = {
      {"demo displacement, +25 deg about z", p_act + disp, q_act * Eigen::Quaterniond(Eigen::AngleAxisd(25.0 * M_PI / 180, Eigen::Vector3d::UnitZ()))},
      {"displacement + (5,-3,2) cm, 40 deg about (1,1,0)", p_act + disp + Eigen::Vector3d(0.05, -0.03, 0.02),
       Eigen::Quaterniond(Eigen::AngleAxisd(40.0 * M_PI / 180, Eigen::Vector3d(1, 1, 0).normalized())) * q_act},
      {"(-4,6,-3) cm, -q (sign flip), 10 deg about y", p_act + Eigen::Vector3d(-0.04, 0.06, -0.03),
       Eigen::Quaterniond(-(q_act * Eigen::Quaterniond(Eigen::AngleAxisd(10.0 * M_PI / 180, Eigen::Vector3d::UnitY()))).coeffs())},
  };

  // default mode untouched: same source without setGoal behaves as before (rigid anchoring)
  {
    ProDmpSource s(W, "", 1.0, 0.0);
    Pose a;
    a.p = p_act;
    a.q = q_act;
    s.start(a);
    const Pose t0 = s.sample(0.0);
    check(!s.hasGoal() && (t0.p - p_act).norm() < 1e-12 && t0.q.angularDistance(q_act) < 1e-12,
          "default mode (no setGoal): first target = activation pose (rigid anchoring unchanged)");
  }

  for (const Goal& g : goals) {
    std::printf("goal: %s\n", g.name);
    ProDmpSource src(W, "", 1.0, 0.0);
    src.setGoal(g.p, g.q);
    Pose a;
    a.p = p_act;
    a.q = q_act;
    src.start(a);
    check(src.positionOffset().norm() == 0.0 && src.orientationOffset().angularDistance(Eigen::Quaterniond::Identity()) == 0.0,
          "rigid offsets are the identity in goal mode");
    // 1 kHz rollout as the loop does it: first sample dt = 0, then 1 ms
    std::vector<Sample> s1;
    s1.push_back({0.0, src.sample(0.0).p, src.sample(0.0).q});  // second call with dt=0 changes nothing
    double t = 0.0;
    long k = 0;
    while (!src.done() && k < 100000) {
      ++k;
      const Pose p = src.sample(1e-3);
      t = src.elapsed();
      s1.push_back({t, p.p, p.q});
    }
    const Sample& last = s1.back();
    Eigen::Vector3d v, w;
    src.twist(v, w);
    check(std::fabs(last.t - src.tau()) < 1e-9, "rollout ends at t = tau (" + std::to_string(last.t) + " s)");
    check(last.p == g.p, "final position EXACTLY the goal (bitwise)");
    check(last.q.coeffs() == src.alignedGoalOrientation().coeffs() && angDeg(last.q, g.q) < 1e-9,
          "final orientation EXACTLY the (aligned) goal quaternion");
    check(v.norm() == 0.0 && w.norm() == 0.0, "twist zero at t = tau");
    check((s1.front().p - p_act).norm() < 1e-12 && angDeg(s1.front().q, q_act) < 1e-9, "first target = activation pose");

    for (double dt_ref : {1e-3, 5e-3, 1e-4}) {
      const std::vector<Sample> ref = reference(W, p_act, q_act, g.p, g.q, dt_ref);
      // compare at the common instants t = j * max(dt_ref, 1ms)
      double max_dp = 0.0, max_da = 0.0;
      std::size_t n = 0;
      const double step = std::max(dt_ref, 1e-3);
      const long stride_src = static_cast<long>(std::llround(step / 1e-3));
      const long stride_ref = static_cast<long>(std::llround(step / dt_ref));
      for (std::size_t i = 0, j = 0; i < s1.size() && j < ref.size(); i += static_cast<std::size_t>(stride_src), j += static_cast<std::size_t>(stride_ref)) {
        if (std::fabs(s1[i].t - ref[j].t) > 1e-6) continue;  // only the exactly common instants
        max_dp = std::max(max_dp, (s1[i].p - ref[j].p).norm());
        max_da = std::max(max_da, angDeg(s1[i].q, ref[j].q));
        ++n;
      }
      // always include the final sample
      max_dp = std::max(max_dp, (s1.back().p - ref.back().p).norm());
      max_da = std::max(max_da, angDeg(s1.back().q, ref.back().q));
      char buf[200];
      std::snprintf(buf, sizeof buf, "vs reference at dt = %.1f ms (%zu common instants): max |dpos| %.3e mm, max |dang| %.3e deg",
                    dt_ref * 1e3, n, max_dp * 1e3, max_da);
      check(n > 1000 && max_dp <= 1e-4 && max_da <= 0.1, buf);
    }
  }
  std::printf("%s (%d failure(s))\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
