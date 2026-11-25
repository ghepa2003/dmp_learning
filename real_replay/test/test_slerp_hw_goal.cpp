// Orientation path of ProDmpSource (goal mode) for the HARDWARE nominal GraspCommand
// (hw/examples/grasp_command_hw_j1p90_w2_th0m10.yaml): q_start . q_goal = -0.961, short path ~32 deg.
//   test_slerp_hw_goal <weights.yaml>
// Run with q_goal and -q_goal: total swept angle must be the SHORT one (not 360 - 32), monotone, final = goal up to sign.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "real_replay/prodmp_source.hpp"

using namespace real_replay;
static int g_fail = 0;
static void check(bool ok, const std::string& what) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}
static double angDeg(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
  const double d = std::min(1.0, std::fabs(a.normalized().coeffs().dot(b.normalized().coeffs())));
  return 2.0 * std::acos(d) * 180.0 / M_PI;
}

int main(int argc, char** argv) {
  if (argc < 2) { std::fprintf(stderr, "usage: %s <weights.yaml>\n", argv[0]); return 2; }
  const std::string W = argv[1];
  const Eigen::Quaterniond q_start(0.494585, -0.505353, -0.505353, -0.494593);   // w, x, y, z
  const Eigen::Quaterniond q_goal(-0.34281217, 0.618449526, 0.34281217, 0.618449526);
  const Eigen::Vector3d p0(2.273e-06, 0.432493844, 0.772246186), pg(-0.046991784, 0.516475991, 0.7);
  const double dot = q_start.normalized().coeffs().dot(q_goal.normalized().coeffs());
  std::printf("dot(q_start, q_goal) = %.4f\n", dot);
  check(dot < -0.9, "premise: dot < 0 (raw interpolation would take the long way)");
  const double expected = angDeg(q_start, q_goal);   // |dot| based: shortest angle
  for (int sign : {+1, -1}) {
    Eigen::Quaterniond qg = q_goal;
    if (sign < 0) qg.coeffs() = -qg.coeffs();
    std::printf("goal sign %+d\n", sign);
    ProDmpSource src(W, "", 1.0, 0.0);
    src.setGoal(pg, qg);
    Pose a; a.p = p0; a.q = q_start;
    src.start(a);
    Eigen::Quaterniond prev = q_start, last = q_start;
    double total = 0.0, max_step = 0.0, min_step = 1e9;
    bool monotone = true; double from_start_prev = 0.0;
    long k = 0;
    src.sample(0.0);
    while (!src.done() && k++ < 100000) {
      const Pose p = src.sample(1e-3);
      const double step = angDeg(prev, p.q), fs = angDeg(q_start, p.q);
      total += step; max_step = std::max(max_step, step);
      if (fs < from_start_prev - 1e-6) monotone = false;
      from_start_prev = fs; prev = p.q; last = p.q;
    }
    std::printf("  total swept angle %.4f deg (short path %.4f deg)\n", total, expected);
    check(std::fabs(total - expected) < 0.05, "total swept angle = short path (~32 deg, not ~328)");
    check(total < 90.0, "total swept angle < 90 deg");
    check(monotone, "angle from q_start monotone non-decreasing");
    check(angDeg(last, q_goal) < 1e-6, "final orientation = goal up to sign");
  }
  std::printf("%s (%d failure(s))\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
