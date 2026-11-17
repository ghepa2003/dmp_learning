// Unit tests of the joint friction feedforward.
//   test_friction_ff <config.yaml> <urdf>      (exit 77 = URDF not generated yet)
//   (1) enabled=false: tau identical to the build without the section (and with alpha = 0)
//   (2) vhat = 0  ->  tau_ff = 0
//   (3) |tau_ff,j| <= alpha*(tau_c,j + b_j*|vhat_j|)
//   (4) sign(tau_ff,j) == sign(vhat_j)

#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <random>
#include <vector>

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

std::string readFile(const std::string& p) {
  std::ifstream f(p);
  return std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

/// Runs `n` cycles of a step source (the reference moves) on a fixed measured state; returns the sent
/// torques (n x 7), and the logged tau_ff / vhat.
struct Run {
  std::vector<Array7> tau, tau_ff, vhat;
  std::vector<double> gain;
};

Run runLoop(const Config& cfg, const std::string& urdf, int n) {
  ReplayLoop loop(cfg, urdf, std::make_unique<StepSource>(cfg.step));
  loop.prepareDamping(cfg.dry_run.q, nullptr, nullptr);
  loop.setTrackingGuardEnabled(false);
  Run r;
  LoopInput in;
  in.q = cfg.dry_run.q;
  in.success_rate = 1.0;
  for (int i = 0; i < n; ++i) {
    in.period_s = i == 0 ? 0.0 : 1e-3;
    LoopOutput out;
    loop.step(in, out);
    r.tau.push_back(out.tau);
  }
  for (std::size_t i = 0; i < loop.logger().size(); ++i) {
    const LogRecord& l = loop.logger().at(i);
    Array7 a{}, b{};
    for (int j = 0; j < 7; ++j) {
      a[static_cast<std::size_t>(j)] = l.tau_ff[j];
      b[static_cast<std::size_t>(j)] = l.vhat[j];
    }
    r.tau_ff.push_back(a);
    r.vhat.push_back(b);
    r.gain.push_back(l.gain_alpha);
  }
  return r;
}

bool identical(const Run& a, const Run& b) {
  if (a.tau.size() != b.tau.size()) return false;
  for (std::size_t i = 0; i < a.tau.size(); ++i)
    for (std::size_t j = 0; j < 7; ++j)
      if (a.tau[i][j] != b.tau[i][j]) return false;
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::printf("usage: test_friction_ff <config.yaml> <urdf>\n");
    return 2;
  }
  if (readFile(argv[2]).empty()) {
    std::printf("SKIP: URDF '%s' not found\n", argv[2]);
    return 77;
  }
  const Config base = loadConfig(argv[1]);
  const std::string urdf = readFile(base.paths.urdf);
  if (urdf.empty()) {
    std::printf("SKIP: URDF '%s' not found\n", base.paths.urdf.c_str());
    return 77;
  }

  // ---- (1) enabled = false -------------------------------------------------------------------
  std::printf("[1] enabled=false gives the same torques as without the section\n");
  {
    CHECK(!base.friction_ff.enabled, "config must not enable the feedforward by default");
    Config off = base;
    off.friction_ff.enabled = false;
    off.friction_ff.alpha = 1.0;  // must be ignored
    Config zero = base;
    zero.friction_ff.enabled = true;
    zero.friction_ff.alpha = 0.0;  // enabled but zero gain
    const Run r0 = runLoop(base, urdf, 3000);
    const Run r1 = runLoop(off, urdf, 3000);
    const Run r2 = runLoop(zero, urdf, 3000);
    CHECK(identical(r0, r1), "tau differs with enabled=false, alpha=1");
    CHECK(identical(r0, r2), "tau differs with enabled=true, alpha=0");
    double mx = 0.0;
    for (const auto& t : r0.tau)
      for (double v : t) mx = std::max(mx, std::fabs(v));
    CHECK(mx > 1e-3, "reference torque is trivially zero (test is vacuous)");
    for (const auto& t : r1.tau_ff)
      for (double v : t) CHECK(v == 0.0, "tau_ff logged non-zero while disabled");
    std::printf("    max |tau| %.3f Nm over 3000 cycles: identical bitwise\n", mx);
  }

  // ---- (2)(3)(4) on the pure functions, random J and random xdot -----------------------------
  core::RobotModel model(urdf, base.robot.joint_names, base.robot.ee_frame);
  FrictionFfConfig f;
  f.enabled = true;
  f.alpha = 0.7;
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(-1.0, 1.0);

  std::printf("[2] vhat = 0 -> tau_ff = 0\n");
  {
    model.update(core::RobotModel::readyPose(), JointVec::Zero());
    const JointVec vh = dlsJointVelocity(model.jacobian(), Vector6::Zero(), f.dls_lambda);
    CHECK(vh.norm() == 0.0, "vhat for xdot_ref = 0 is %g", vh.norm());
    const JointVec t = frictionTorque(vh, f, f.alpha);
    CHECK(t.norm() == 0.0, "tau_ff at vhat = 0 is %g", t.norm());
    CHECK(frictionTorque(JointVec::Zero(), f, 1.0).norm() == 0.0, "tau_ff at vhat = 0 (direct)");
    std::printf("    ok\n");
  }

  std::printf("[3][4] bound and sign on 2000 random (q, xdot_ref) and 7 joints, + the loop's own log\n");
  {
    int n_signed = 0;
    for (int it = 0; it < 2000; ++it) {
      JointVec q = core::RobotModel::readyPose();
      for (int i = 0; i < 7; ++i) q(i) += 0.4 * u(rng);
      model.update(q, JointVec::Zero());
      Vector6 xd;
      const double sc = std::pow(10.0, 3.0 * u(rng) - 1.0);  // 1e-4 .. 1e2 spread, to cross eps_v
      for (int i = 0; i < 6; ++i) xd(i) = sc * 0.1 * u(rng);
      const JointVec vh = dlsJointVelocity(model.jacobian(), xd, f.dls_lambda);
      const double scale = f.alpha * (0.1 + 0.9 * std::fabs(u(rng)));  // alpha * gain_alpha
      const JointVec t = frictionTorque(vh, f, scale);
      for (int j = 0; j < 7; ++j) {
        const std::size_t k = static_cast<std::size_t>(j);
        const double bound = scale * (f.tau_c[k] + f.b[k] * std::fabs(vh(j)));
        CHECK(std::fabs(t(j)) <= bound + 1e-12, "|tau_ff| %.9g > bound %.9g", std::fabs(t(j)), bound);
        if (vh(j) != 0.0 && std::fabs(t(j)) > 0.0) {
          ++n_signed;
          CHECK((t(j) > 0.0) == (vh(j) > 0.0), "sign mismatch: tau_ff %.6g vhat %.6g", t(j), vh(j));
        }
      }
    }
    CHECK(n_signed > 10000, "too few sign checks (%d)", n_signed);
    std::printf("    %d sign checks\n", n_signed);

    Config on = base;
    on.friction_ff = f;
    on.friction_ff.alpha = 1.0;
    const Run r = runLoop(on, urdf, 3000);
    double mx_ff = 0.0;
    int n_nz = 0;
    for (std::size_t i = 0; i < r.tau_ff.size(); ++i) {
      for (std::size_t j = 0; j < 7; ++j) {
        const double v = r.vhat[i][j], t = r.tau_ff[i][j];
        const double bound = on.friction_ff.alpha * r.gain[i] * (on.friction_ff.tau_c[j] + on.friction_ff.b[j] * std::fabs(v));
        CHECK(std::fabs(t) <= bound + 1e-12, "loop: |tau_ff| %.9g > bound %.9g (cycle %zu joint %zu)", std::fabs(t), bound, i, j + 1);
        if (v != 0.0 && t != 0.0) {
          ++n_nz;
          CHECK((t > 0.0) == (v > 0.0), "loop: sign mismatch (cycle %zu joint %zu)", i, j + 1);
        }
        mx_ff = std::max(mx_ff, std::fabs(t));
      }
    }
    CHECK(n_nz > 1000 && mx_ff > 1e-3, "loop feedforward trivially zero (nz %d, max %.3g)", n_nz, mx_ff);
    std::printf("    loop log: %d non-zero samples, max |tau_ff| %.4f Nm\n", n_nz, mx_ff);
  }

  std::printf("%d checks, %d failed\n", g_checks, g_fail);
  return g_fail == 0 ? 0 : 1;
}
