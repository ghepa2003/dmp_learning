// Synthetic tests of the gravity preflight logic and of the mass scaling.
//   test_preflight_checks <config.yaml> <urdf>     (exit 77 = URDF not generated yet)

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

#include "real_replay/config.hpp"
#include "real_replay/controller_core.hpp"
#include "real_replay/preflight_checks.hpp"

using namespace real_replay;
using V = MeasuredGravityResult::Verdict;

static int g_fail = 0, g_checks = 0;
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

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  std::ifstream f(argv[2]);
  if (!f) {
    std::printf("SKIP: URDF not found\n");
    return 77;
  }
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string urdf = ss.str();
  try {
    const Config cfg = loadConfig(argv[1]);
    const double tol = cfg.preflight.gravity_vs_measured_tol_nm;
    const double still = cfg.preflight.gravity_still_max_dq_rad_s;
    const double warn_u = cfg.preflight.gravity_urdf_warn_nm;
    // Realistic gravity vector (joint4 18.57, joint2 -2.08 from the real robot).
    const Array7 g = {0.0, -2.08, 0.1, 18.57, 0.2, 1.4, 0.0};
    const Array7 rest{};

    std::printf("[franka gravity vs measured tau_J]\n");
    {  // consistent: 0.27 Nm on joint 4
      Array7 t = g;
      t[3] = 18.84;
      const auto r = checkGravityVsMeasured(g, t, rest, tol, still);
      CHECK(r.verdict == V::kPass && std::fabs(r.max_abs_diff - 0.27) < 1e-9, "diff %.3f", r.max_abs_diff);
      std::printf("  0.27 Nm at rest: PASS (max diff %.3f)\n", r.max_abs_diff);
    }
    {  // wrong mass: 2 Nm on joint 4
      Array7 t = g;
      t[3] += 2.0;
      const auto r = checkGravityVsMeasured(g, t, rest, tol, still);
      CHECK(r.verdict == V::kFail && r.worst_joint == 3, "verdict");
      std::printf("  2.0 Nm on joint4 at rest: FAIL\n");
    }
    {  // moving: the same 2 Nm, but |dq| above the threshold -> WARN, not FAIL
      Array7 t = g;
      t[3] += 2.0;
      Array7 dq{};
      dq[0] = 0.1;
      const auto r = checkGravityVsMeasured(g, t, dq, tol, still);
      CHECK(r.verdict == V::kWarnMoving && r.moving, "verdict");
      std::printf("  2.0 Nm while moving (|dq| 0.1): WARN (moving)\n");
      // moving but consistent: plain PASS
      const auto r2 = checkGravityVsMeasured(g, g, dq, tol, still);
      CHECK(r2.verdict == V::kPass, "consistent while moving");
    }
    {  // boundary
      Array7 t = g;
      t[1] += tol;
      CHECK(checkGravityVsMeasured(g, t, rest, tol, still).verdict == V::kPass, "exactly at tol passes");
      t[1] += 1e-6;
      CHECK(checkGravityVsMeasured(g, t, rest, tol, still).verdict == V::kFail, "just above tol fails");
    }

    std::printf("[franka gravity vs URDF: informative]\n");
    {
      Array7 u = g;
      u[3] -= 3.0;
      CHECK(!checkGravityVsUrdf(g, u, warn_u).warn, "3 Nm: no warning");
      u[3] = g[3] - 5.0;
      const auto r = checkGravityVsUrdf(g, u, warn_u);
      CHECK(r.warn && r.worst_joint == 3 && std::fabs(r.max_abs_diff - 5.0) < 1e-9, "5 Nm: warning");
      std::printf("  URDF off by 3 Nm: no WARN; by 5 Nm: WARN (threshold %.1f)\n", warn_u);
    }

    std::printf("[scaleLinkMasses]\n");
    {
      const std::vector<std::string> links = {"fer_link3", "fer_link4", "fer_link5", "fer_link6"};
      auto totalMass = [&](const std::string& x) {
        core::RobotModel m(x, cfg.robot.joint_names, cfg.robot.ee_frame);
        // gravity at the ready pose is linear in the masses: use it as an observable
        m.update(core::RobotModel::readyPose(), core::RobotModel::JointVector::Zero());
        return m.gravity();
      };
      const std::string up = scaleLinkMasses(urdf, links, 1.15);
      CHECK(up != urdf, "changed");
      CHECK(scaleLinkMasses(urdf, links, 1.0).size() > 0, "identity runs");
      const auto g0 = totalMass(urdf), g1 = totalMass(scaleLinkMasses(urdf, links, 1.0));
      CHECK((g0 - g1).cwiseAbs().maxCoeff() < 1e-9, "factor 1 keeps the model");
      const auto gu = totalMass(up), gd = totalMass(scaleLinkMasses(urdf, links, 0.85));
      // links 3..6 only: gravity changes, symmetric in +-15% (linear): gu - g0 == g0 - gd * (0.15/0.15)
      CHECK(((gu - g0) + (gd - g0)).cwiseAbs().maxCoeff() < 1e-9, "gravity is linear in the scaled masses");
      CHECK((gu - g0).cwiseAbs().maxCoeff() > 0.1, "gravity does change");
      // untouched links: link1 / link7 / link0 text is identical
      for (const char* keep : {"fer_link0", "fer_link1", "fer_link2", "fer_link7"}) {
        const std::string tag = std::string("<link name=\"") + keep + "\">";
        const auto b0 = urdf.find(tag), b1 = up.find(tag);
        CHECK(b0 != std::string::npos && b1 != std::string::npos &&
                  urdf.substr(b0, urdf.find("</link>", b0) - b0) == up.substr(b1, up.find("</link>", b1) - b1),
              "%s untouched", keep);
      }
      bool threw = false;
      try { scaleLinkMasses(urdf, {"nope"}, 1.1); } catch (const std::runtime_error&) { threw = true; }
      CHECK(threw, "missing link throws");
    }
  } catch (const std::exception& e) {
    std::printf("EXCEPTION: %s\n", e.what());
    return 2;
  }
  std::printf("\ntest_preflight_checks: %d check(s), %d failure(s)\n", g_checks, g_fail);
  return g_fail == 0 ? 0 : 1;
}
