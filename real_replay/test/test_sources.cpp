// Tests of the target sources and of the SHA-256 helper. No robot, no libfranka.
//   test_sources <config.yaml>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>

#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "real_replay/config.hpp"
#include "real_replay/prodmp_source.hpp"
#include "real_replay/sha256.hpp"
#include "real_replay/target_source.hpp"

using namespace real_replay;
namespace hcore = haptic_dmp_learning::core;

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

Eigen::Quaterniond randomQuat(std::mt19937& rng) {
  std::normal_distribution<double> n(0.0, 1.0);
  Eigen::Quaterniond q(n(rng), n(rng), n(rng), n(rng));
  q.normalize();
  return q;
}

/// Independent reference: the same loader and models, stepped by hand.
struct Reference {
  hcore::QuaternionDMP qdmp{20, 4.6, 25.0, 6.25};
  hcore::ProDMP prodmp{20};
  Eigen::Vector3d p0;
  Eigen::Quaterniond q0;
  explicit Reference(const std::string& path) {
    bool has = false;
    hcore::QuaternionDMP loaded(20, 4.6, 25.0, 6.25);
    prodmp = hcore::prodmp_io::loadProDmpFromYaml(path, loaded, has);
    qdmp = loaded;
    prodmp.setInitialConditions(0.0, prodmp.initPos(), prodmp.initVel());
    qdmp.reset();
    p0 = prodmp.step(0.0);
    q0 = qdmp.step(0.0).normalized();
  }
};

void testSha256(const Config& cfg) {
  std::printf("[sha256]\n");
  CHECK(sha256Hex("") == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855", "empty");
  CHECK(sha256Hex("abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad", "abc");
  CHECK(sha256Hex("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq") ==
            "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1",
        "two-block padding");
  CHECK(sha256Hex(std::string(1000000, 'a')) == "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0",
        "one million 'a'");
  if (!cfg.replay.expected_sha256.empty()) {
    const std::string h = sha256FileHex(cfg.replay.weights_path);
    CHECK(h == cfg.replay.expected_sha256, "weights file: %s", h.c_str());
    std::printf("  weights sha256 %s\n", h.c_str());
  }
}

void testAnchoring(const Config& cfg) {
  std::printf("[anchoring: target(0) == activation pose]\n");
  std::mt19937 rng(12345);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  double worst_p = 0.0, worst_a = 0.0;
  for (int k = 0; k < 8; ++k) {
    Pose act;
    act.p = k == 0 ? Eigen::Vector3d::Zero() : Eigen::Vector3d(u(rng), u(rng), u(rng));
    act.q = k == 0 ? Eigen::Quaterniond::Identity() : randomQuat(rng);  // includes w < 0 representatives
    ProDmpSource src(cfg.replay.weights_path, cfg.replay.expected_sha256, 1.0, 1.0);
    src.start(act);
    const Pose t0 = src.sample(0.0);
    const double dp = (t0.p - act.p).norm();
    const double da = t0.q.angularDistance(act.q);
    worst_p = std::max(worst_p, dp);
    worst_a = std::max(worst_a, da);
    CHECK(dp < 1e-9, "case %d: |dp| = %.3e", k, dp);
    CHECK(da < 1e-9, "case %d: angle = %.3e", k, da);
  }
  std::printf("  worst |dp| %.3e m, worst angle %.3e rad over 8 activation poses\n", worst_p, worst_a);

  Reference ref(cfg.replay.weights_path);
  std::printf("  step(0.0) leaves the models at their initial state: |p_raw0 - init_pos| = %.3e m\n",
              (ref.p0 - ref.prodmp.initPos()).norm());
}

/// Drives a ProDmpSource and the independent reference with the same dt sequence and compares.
void runIncrements(const Config& cfg, const Pose& act, const char* label, bool irregular, std::size_t n_steps) {
  ProDmpSource src(cfg.replay.weights_path, cfg.replay.expected_sha256, 1.0, 1.0);
  Reference ref(cfg.replay.weights_path);
  src.start(act);
  const Pose t0 = src.sample(0.0);

  const Eigen::Quaterniond q_off = src.orientationOffset();
  double max_dp = 0.0, max_body = 0.0, max_world = 0.0, max_world_vs_raw = 0.0;
  for (std::size_t k = 1; k <= n_steps; ++k) {
    const double dt = (irregular && k % 3 == 0) ? 2e-3 : 1e-3;
    const Pose t = src.sample(dt);
    const Eigen::Vector3d p_raw = ref.prodmp.step(dt);
    const Eigen::Quaterniond q_raw = ref.qdmp.step(dt).normalized();

    max_dp = std::max(max_dp, ((t.p - t0.p) - (p_raw - ref.p0)).norm());
    // Right (body-frame) increment q(0)^-1 q(t): invariant under the LEFT anchoring q_off.
    const Eigen::Quaterniond body_des = t0.q.conjugate() * t.q;
    const Eigen::Quaterniond body_raw = ref.q0.conjugate() * q_raw;
    max_body = std::max(max_body, body_des.angularDistance(body_raw));
    // Left (world-frame) increment q(t) q(0)^-1: conjugated by q_off.
    const Eigen::Quaterniond world_des = t.q * t0.q.conjugate();
    const Eigen::Quaterniond world_raw = q_raw * ref.q0.conjugate();
    const Eigen::Quaterniond world_expected = q_off * world_raw * q_off.conjugate();
    max_world = std::max(max_world, world_des.angularDistance(world_expected));
    max_world_vs_raw = std::max(max_world_vs_raw, world_des.angularDistance(world_raw));
  }
  const double q_off_angle = Eigen::Quaterniond::Identity().angularDistance(q_off);
  std::printf("  %-34s |q_off| = %6.3f rad: max |dp - dp_raw| %.2e m | body incr. vs raw %.2e rad | "
              "world incr. vs q_off*raw*q_off^-1 %.2e rad | world incr. vs raw %.2e rad\n",
              label, q_off_angle, max_dp, max_body, max_world, max_world_vs_raw);
  CHECK(max_dp < 1e-9, "%s: position increment mismatch %.3e", label, max_dp);
  CHECK(max_body < 1e-9, "%s: body-frame rotation increment mismatch %.3e", label, max_body);
  CHECK(max_world < 1e-9, "%s: world-frame rotation increment mismatch %.3e", label, max_world);
  if (q_off_angle < 1e-9) {
    CHECK(max_world_vs_raw < 1e-9, "%s: with q_off = identity the world increment must equal the raw one (%.3e)",
          label, max_world_vs_raw);
  }
}

void testIncrements(const Config& cfg) {
  std::printf("[increments along the trajectory]\n");
  std::mt19937 rng(777);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  Reference ref(cfg.replay.weights_path);

  Pose generic;
  generic.p = Eigen::Vector3d(0.31, -0.12, 0.47);
  generic.q = randomQuat(rng);
  runIncrements(cfg, generic, "generic q_act, dt = 1 ms", false, 50000);
  runIncrements(cfg, generic, "generic q_act, dt = 1/2 ms mix", true, 25000);

  Pose same;  // q_act = q_raw0  =>  q_off = identity
  same.p = Eigen::Vector3d(u(rng), u(rng), u(rng));
  same.q = ref.q0;
  runIncrements(cfg, same, "q_act = q_raw0 (q_off = I)", false, 20000);
}

void testEndBehaviour(const Config& cfg) {
  std::printf("[end of the trajectory / stop_at_fraction / time = sum of dt]\n");
  Reference ref(cfg.replay.weights_path);
  const double tau = ref.prodmp.tau();
  Pose act;
  act.p = Eigen::Vector3d(0.3, 0.0, 0.5);
  act.q = Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()));

  // (1) fraction 0.25, hold 0.5 s: frozen target, done() only after the hold.
  {
    ProDmpSource src(cfg.replay.weights_path, cfg.replay.expected_sha256, 0.25, 0.5);
    src.start(act);
    src.sample(0.0);
    CHECK(std::fabs(src.nominalDurationS() - (0.25 * tau + 0.5)) < 1e-9, "nominal duration");
    Pose last;
    std::size_t n = 0;
    std::size_t frozen_from = 0;
    while (!src.done() && n < 200000) {
      last = src.sample(1e-3);
      ++n;
      if (frozen_from == 0 && src.progress() >= 0.25 - 1e-12) frozen_from = n;
    }
    const double expected_n = (0.25 * tau + 0.5) / 1e-3;
    std::printf("  fraction 0.25: done after %zu samples (expected ~%.0f), target frozen from sample %zu\n", n,
                expected_n, frozen_from);
    CHECK(std::fabs(static_cast<double>(n) - expected_n) <= 2.0, "samples to done");
    const Pose again = src.sample(1e-3);
    CHECK((again.p - last.p).norm() == 0.0 && again.q.coeffs() == last.q.coeffs(), "target stays frozen");
    CHECK(std::fabs(src.progress() - 0.25) < 1e-6, "progress at freeze = %.9f", src.progress());
  }
  // (2) fraction 1.0: final sample is the attractor, then hold, then done.
  {
    ProDmpSource src(cfg.replay.weights_path, cfg.replay.expected_sha256, 1.0, 0.3);
    src.start(act);
    const Pose t0 = src.sample(0.0);
    Pose last = t0;
    std::size_t n = 0;
    while (!src.done() && n < 200000) {
      last = src.sample(1e-3);
      ++n;
    }
    const Eigen::Vector3d goal_expected = src.positionOffset() + ref.prodmp.goal();
    const Eigen::Quaterniond qgoal_expected = (src.orientationOffset() * ref.qdmp.goal()).normalized();
    std::printf("  fraction 1.0: done after %zu samples (expected ~%.0f); |p_end - (p_off+goal)| = %.2e m\n", n,
                (tau + 0.3) / 1e-3, (last.p - goal_expected).norm());
    CHECK((last.p - goal_expected).norm() < 1e-12, "final position is the goal");
    CHECK(last.q.angularDistance(qgoal_expected) < 1e-12, "final orientation is the goal");
    CHECK(std::fabs(static_cast<double>(n) - (tau + 0.3) / 1e-3) <= 2.0, "samples to done");
  }
  // (3) one huge dt lands exactly on the end (no overshoot), and elapsed time is the sum of dt.
  {
    ProDmpSource src(cfg.replay.weights_path, cfg.replay.expected_sha256, 1.0, 0.0);
    src.start(act);
    src.sample(0.0);
    src.sample(tau * 2.0);
    CHECK(std::fabs(src.progress() - 1.0) < 1e-12, "progress after a huge dt");
    CHECK(src.done(), "done with zero hold");
  }
  {
    ProDmpSource a(cfg.replay.weights_path, cfg.replay.expected_sha256, 1.0, 1.0);
    ProDmpSource b(cfg.replay.weights_path, cfg.replay.expected_sha256, 1.0, 1.0);
    a.start(act);
    b.start(act);
    a.sample(0.0);
    b.sample(0.0);
    for (int i = 0; i < 1000; ++i) a.sample(2e-3);
    for (int i = 0; i < 2000; ++i) b.sample(1e-3);
    CHECK(std::fabs(a.progress() - b.progress()) < 1e-9, "progress is a function of the summed dt: %.12f vs %.12f",
          a.progress(), b.progress());
  }
}

void testStepRotSource() {
  std::printf("[StepSource rotational]\n");
  StepConfig c{};
  c.axis = {1.0, 0.0, 0.0};
  c.amplitude_m = 0.05;  // must be ignored in rotational mode
  c.move_duration_s = 2.0;
  c.dwell_s = 1.0;
  c.return_to_start = true;
  c.return_duration_s = 3.0;
  c.rot_axis = {0.0, 0.0, 1.0};
  c.rot_angle_rad = 0.087;
  const Eigen::Vector3d axis(0.0, 0.0, 1.0);

  Pose act;
  act.p = Eigen::Vector3d(0.3, -0.2, 0.4);
  act.q = Eigen::Quaterniond(Eigen::AngleAxisd(0.8, Eigen::Vector3d(1, 1, 0).normalized()));

  StepSource s(c, /*rotational=*/true);
  s.start(act);
  const Pose t0 = s.sample(0.0);
  CHECK((t0.p - act.p).norm() < 1e-12 && t0.q.angularDistance(act.q) < 1e-12, "rot: target(0) is the activation pose");
  CHECK(std::fabs(s.nominalDurationS() - 6.0) < 1e-12, "rot: nominal duration");

  double t = 0.0, max_ang = 0.0, max_fd_err = 0.0, max_pos = 0.0, max_axis_err = 0.0;
  Pose prev = t0, p = t0;
  Pose at2, at3;
  std::size_t n = 0;
  while (!s.done() && n < 100000) {
    p = s.sample(1e-3);
    t += 1e-3;
    ++n;
    max_pos = std::max(max_pos, (p.p - act.p).norm());
    const Eigen::Quaterniond dq = p.q * act.q.inverse();  // base-frame rotation
    const Eigen::AngleAxisd aa(dq);
    max_ang = std::max(max_ang, aa.angle());
    if (aa.angle() > 1e-6) max_axis_err = std::max(max_axis_err, (aa.axis() - axis).norm());
    Eigen::Vector3d v, w;
    s.twist(v, w);
    // finite difference of the base-frame angular velocity: log(q_k q_{k-1}^-1)/dt
    const Eigen::AngleAxisd d(p.q * prev.q.inverse());
    max_fd_err = std::max(max_fd_err, (w - d.axis() * d.angle() / 1e-3).norm() * (d.angle() > 1e-12 ? 1.0 : 0.0) +
                                          (d.angle() > 1e-12 ? 0.0 : w.norm()));
    if (v.norm() != 0.0) max_fd_err = 1e9;
    if (std::fabs(t - 2.0) < 5e-4) at2 = p;
    if (std::fabs(t - 3.0) < 5e-4) at3 = p;
    prev = p;
  }
  std::printf("  %zu samples; peak angle %.6f rad (cfg %.3f); max |w - fd| %.2e rad/s\n", n, max_ang, c.rot_angle_rad,
              max_fd_err);
  CHECK(std::fabs(max_ang - c.rot_angle_rad) < 1e-9, "rot: reaches the angle");
  CHECK(max_pos < 1e-12, "rot: position held");
  CHECK(max_axis_err < 1e-6, "rot: rotation axis is the fixed base axis");
  CHECK(at2.q.angularDistance(Eigen::Quaterniond(Eigen::AngleAxisd(c.rot_angle_rad, axis)) * act.q) < 1e-6,
        "rot: end of move = Exp(theta*axis)*q_act");
  CHECK(at3.q.angularDistance(at2.q) < 1e-6, "rot: dwell");
  CHECK(p.q.angularDistance(act.q) < 1e-6 && (p.p - act.p).norm() < 1e-12, "rot: back at the activation pose");
  CHECK(max_fd_err < 1e-3, "rot: twist vs finite difference");
}

void testStepSource() {
  std::printf("[StepSource]\n");
  StepConfig c{};
  c.axis = {1.0, 2.0, 2.0};
  c.amplitude_m = 0.05;
  c.move_duration_s = 2.0;
  c.dwell_s = 1.0;
  c.return_to_start = true;
  c.return_duration_s = 3.0;
  const Eigen::Vector3d axis = Eigen::Vector3d(1.0, 2.0, 2.0).normalized();

  Pose act;
  act.p = Eigen::Vector3d(0.3, -0.2, 0.4);
  act.q = Eigen::Quaterniond(Eigen::AngleAxisd(0.8, Eigen::Vector3d(1, 1, 0).normalized()));

  StepSource s(c);
  s.start(act);
  const Pose t0 = s.sample(0.0);
  CHECK((t0.p - act.p).norm() < 1e-12 && t0.q.angularDistance(act.q) < 1e-12, "target(0) is the activation pose");
  CHECK(std::fabs(s.nominalDurationS() - 6.0) < 1e-12, "nominal duration");

  double t = 0.0;
  Pose prev = t0;
  double max_fd_err = 0.0, max_disp = 0.0;
  bool monotone_out = true, monotone_back = true, orientation_ok = true;
  Eigen::Vector3d at2, at3, at6;
  std::size_t n = 0;
  while (!s.done() && n < 100000) {
    const Pose p = s.sample(1e-3);
    t += 1e-3;
    ++n;
    const double disp = (p.p - act.p).dot(axis);
    max_disp = std::max(max_disp, disp);
    if (t <= 2.0 && disp < (prev.p - act.p).dot(axis) - 1e-15) monotone_out = false;
    if (t > 3.0 && disp > (prev.p - act.p).dot(axis) + 1e-15) monotone_back = false;
    if (p.q.angularDistance(act.q) > 1e-12) orientation_ok = false;
    Eigen::Vector3d v, w;
    s.twist(v, w);
    max_fd_err = std::max(max_fd_err, (v - (p.p - prev.p) / 1e-3).norm());
    if (std::fabs(t - 2.0) < 5e-4) at2 = p.p;
    if (std::fabs(t - 3.0) < 5e-4) at3 = p.p;
    if (std::fabs(t - 6.0) < 5e-4) at6 = p.p;
    prev = p;
  }
  std::printf("  %zu samples; peak displacement %.6f m (amplitude %.3f); max |twist - finite difference| %.2e m/s\n", n,
              max_disp, c.amplitude_m, max_fd_err);
  CHECK(std::fabs(n - 6000.0) <= 1.0, "samples to done: %zu", n);
  CHECK(std::fabs(max_disp - c.amplitude_m) < 1e-9, "reaches the amplitude");
  CHECK(((at2 - act.p) - axis * c.amplitude_m).norm() < 1e-6, "end of move");
  CHECK(((at3 - act.p) - axis * c.amplitude_m).norm() < 1e-6, "end of dwell");
  CHECK((at6 - act.p).norm() < 1e-6, "back at the activation pose");
  CHECK(monotone_out && monotone_back, "monotone profile");
  CHECK(orientation_ok, "orientation held");
  CHECK(max_fd_err < 1e-4, "twist vs finite difference");

  StepConfig nr = c;
  nr.return_to_start = false;
  StepSource s2(nr);
  s2.start(act);
  s2.sample(0.0);
  Pose end;
  n = 0;
  while (!s2.done() && n < 100000) {
    end = s2.sample(1e-3);
    ++n;
  }
  CHECK(std::fabs(s2.nominalDurationS() - 3.0) < 1e-12, "no-return nominal duration");
  CHECK(((end.p - act.p) - axis * c.amplitude_m).norm() < 1e-9, "no-return ends displaced");
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: %s <config.yaml>\n", argv[0]);
    return 2;
  }
  try {
    const Config cfg = loadConfig(argv[1]);
    testSha256(cfg);
    testAnchoring(cfg);
    testIncrements(cfg);
    testEndBehaviour(cfg);
    testStepSource();
    testStepRotSource();
  } catch (const std::exception& e) {
    std::printf("EXCEPTION: %s\n", e.what());
    return 2;
  }
  std::printf("\ntest_sources: %d check(s), %d failure(s)\n", g_checks, g_fail);
  return g_fail == 0 ? 0 : 1;
}
