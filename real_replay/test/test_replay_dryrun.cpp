// Complete dry-run of the ProDMP replay at nominal tau through ReplayLoop (fixed robot state, no
// robot, no libfranka): NaN check, callback duration statistics and heap-allocation count.
//   test_replay_dryrun <config.yaml> <urdf> [--paced]
// Exit code 77 = skipped (URDF not generated yet).

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <new>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "real_replay/config.hpp"
#include "real_replay/controller_core.hpp"
#include "real_replay/prodmp_source.hpp"

// ---------------------------------------------------------------------------------------------
// Allocation counting: operator new is replaced; malloc-family calls made by the code compiled into
// this binary (Eigen, Pinocchio templates) are routed through --wrap (see CMakeLists.txt).
// Allocations made by shared libraries through malloc directly are NOT seen.
// ---------------------------------------------------------------------------------------------
extern "C" {
void* __real_malloc(std::size_t);
void* __real_calloc(std::size_t, std::size_t);
void* __real_realloc(void*, std::size_t);
int __real_posix_memalign(void**, std::size_t, std::size_t);
void* __real_aligned_alloc(std::size_t, std::size_t);
}

namespace {
std::atomic<bool> g_count{false};
std::atomic<unsigned long> g_n_new{0};
std::atomic<unsigned long> g_n_malloc{0};
inline void bumpNew() {
  if (g_count.load(std::memory_order_relaxed)) g_n_new.fetch_add(1, std::memory_order_relaxed);
}
inline void bumpMalloc() {
  if (g_count.load(std::memory_order_relaxed)) g_n_malloc.fetch_add(1, std::memory_order_relaxed);
}
}  // namespace

extern "C" {
void* __wrap_malloc(std::size_t n) {
  bumpMalloc();
  return __real_malloc(n);
}
void* __wrap_calloc(std::size_t a, std::size_t b) {
  bumpMalloc();
  return __real_calloc(a, b);
}
void* __wrap_realloc(void* p, std::size_t n) {
  bumpMalloc();
  return __real_realloc(p, n);
}
int __wrap_posix_memalign(void** out, std::size_t al, std::size_t n) {
  bumpMalloc();
  return __real_posix_memalign(out, al, n);
}
void* __wrap_aligned_alloc(std::size_t al, std::size_t n) {
  bumpMalloc();
  return __real_aligned_alloc(al, n);
}
}

void* operator new(std::size_t n) {
  bumpNew();
  void* p = __real_malloc(n);
  if (!p) throw std::bad_alloc();
  return p;
}
void* operator new[](std::size_t n) {
  bumpNew();
  void* p = __real_malloc(n);
  if (!p) throw std::bad_alloc();
  return p;
}
void* operator new(std::size_t n, std::align_val_t al) {
  bumpNew();
  const std::size_t a = static_cast<std::size_t>(al);
  void* p = __real_aligned_alloc(a, (n + a - 1) / a * a);
  if (!p) throw std::bad_alloc();
  return p;
}
void* operator new[](std::size_t n, std::align_val_t al) { return operator new(n, al); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }

// ---------------------------------------------------------------------------------------------

using namespace real_replay;
namespace hcore = haptic_dmp_learning::core;

namespace {

int g_fail = 0;
#define CHECK(cond, ...)                                              \
  do {                                                                \
    if (!(cond)) {                                                    \
      ++g_fail;                                                       \
      std::printf("  CHECK FAILED (line %d): %s  ", __LINE__, #cond); \
      std::printf(__VA_ARGS__);                                       \
      std::printf("\n");                                              \
    }                                                                 \
  } while (0)

double percentile(std::vector<double> v, double q) {  // q in [0, 1]
  if (v.empty()) return 0.0;
  std::size_t k = static_cast<std::size_t>(std::ceil(q * static_cast<double>(v.size())));
  k = std::min(v.size(), std::max<std::size_t>(1, k)) - 1;
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(k), v.end());
  return v[k];
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s <config.yaml> <urdf> [--paced]\n", argv[0]);
    return 2;
  }
  const bool paced = argc > 3 && std::string(argv[3]) == "--paced";
  {
    std::ifstream probe(argv[2]);
    if (!probe) {
      std::printf("SKIP: URDF '%s' not found (generate it with scripts/gen_urdf_nohand.py)\n", argv[2]);
      return 77;
    }
  }

  try {
    Config cfg = loadConfig(argv[1]);
    cfg.trajectory.stop_at_fraction = 1.0;          // nominal tau
    cfg.trajectory.target_update_divisor = 1;
    cfg.safety.callback_budget_us = 1e9;            // measure, do not let the watchdog end the run
    cfg.safety.callback_budget_consecutive = 1000000;

    std::ifstream f(argv[2]);
    std::stringstream ss;
    ss << f.rdbuf();

    auto src = std::make_unique<ProDmpSource>(cfg.replay.weights_path, cfg.replay.expected_sha256, 1.0,
                                              cfg.replay.hold_after_end_s);
    const double tau = src->tau();
    const double nominal = src->nominalDurationS();
    std::printf("weights sha256 %s\ntau %.6f s, nominal length %.3f s (+ %.2f s controlled stop)\n",
                src->sha256().c_str(), tau, nominal, cfg.safety.stop_duration_s);
    if (nominal + cfg.safety.stop_duration_s >= cfg.run.max_duration_s) {
      std::printf("run.max_duration_s (%.1f s) too small for this replay\n", cfg.run.max_duration_s);
      return 2;
    }

    // Independent expectation for the final tracking error: |goal - init_pos| (the state never moves).
    hcore::QuaternionDMP qtmp(20, 4.6, 25.0, 6.25);
    bool has = false;
    const hcore::ProDMP pref = hcore::prodmp_io::loadProDmpFromYaml(cfg.replay.weights_path, qtmp, has);
    const double expected_disp = (pref.goal() - pref.initPos()).norm();

    ReplayLoop loop(cfg, ss.str(), std::move(src));
    loop.setTrackingGuardEnabled(false);
    // As in main(): the per-axis damping design (non-real-time code) is prepared BEFORE the loop and
    // therefore outside the counted window.
    loop.prepareDamping(cfg.dry_run.q, nullptr, nullptr);

    LoopInput in;
    in.q = cfg.dry_run.q;
    in.period_s = 1e-3;
    in.success_rate = 1.0;
    LoopOutput out;

    const std::size_t max_cycles = static_cast<std::size_t>((nominal + cfg.safety.stop_duration_s) / 1e-3) + 200;
    std::vector<double> cb_us(max_cycles, 0.0);  // preallocated, filled in the loop

    std::printf("running the replay (%s)...\n", paced ? "paced at 1 kHz" : "unpaced");
    // Warm-up on the first cycle (activation) outside the counted window is NOT done on purpose:
    // the count below includes everything from the first cycle on.
    std::size_t n = 0;
    auto next = std::chrono::steady_clock::now();
    g_n_new = 0;
    g_n_malloc = 0;
    g_count = true;
    for (; n < max_cycles; ++n) {
      const auto t0 = std::chrono::steady_clock::now();
      loop.step(in, out);
      const auto t1 = std::chrono::steady_clock::now();
      const double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
      loop.setLastCallbackUs(us);
      cb_us[n] = us;
      if (out.finished) {
        ++n;
        break;
      }
      if (paced) {
        next += std::chrono::microseconds(1000);
        std::this_thread::sleep_until(next);
      }
    }
    g_count = false;
    cb_us.resize(n);

    const unsigned long allocs = g_n_new.load() + g_n_malloc.load();
    std::printf("cycles: %zu   stop reason: %s\n", n, toString(loop.stopReason()));
    std::printf("callback duration [us]: min %.2f  mean %.2f  p99 %.2f  p99.9 %.2f  max %.2f\n",
                *std::min_element(cb_us.begin(), cb_us.end()),
                std::accumulate(cb_us.begin(), cb_us.end(), 0.0) / static_cast<double>(n), percentile(cb_us, 0.99),
                percentile(cb_us, 0.999), *std::max_element(cb_us.begin(), cb_us.end()));
    std::printf("heap allocations during the loop: %lu (operator new %lu + malloc-family %lu) = %.2f per cycle\n",
                allocs, g_n_new.load(), g_n_malloc.load(), static_cast<double>(allocs) / static_cast<double>(n));

    // The state never moves, so |e_pos| is the distance of the target from the activation pose. The
    // ProDMP may overshoot its endpoint along the way, so compare the LAST Running-phase sample
    // (the target frozen on the goal) with |goal - init_pos|, and report the maximum separately.
    double max_epos = 0.0, max_tau = 0.0, final_epos = 0.0;
    for (std::size_t i = 0; i < loop.logger().size(); ++i) {
      const LogRecord& r = loop.logger().at(i);
      const double e = std::sqrt(r.e_pos[0] * r.e_pos[0] + r.e_pos[1] * r.e_pos[1] + r.e_pos[2] * r.e_pos[2]);
      max_epos = std::max(max_epos, e);
      if (r.phase == 1.0) final_epos = e;
      for (int j = 0; j < 7; ++j) max_tau = std::max(max_tau, std::fabs(r.tau_cmd[j]));
    }
    std::printf("|e_pos| at the end of the replay %.6f m (expected |goal - init| = %.6f m), max along the path %.6f m\n"
                "max |tau_cmd| %.3f Nm   cycles rate-limited %ld, clamped %ld (expected: fixed state, moving target)\n",
                final_epos, expected_disp, max_epos, max_tau, loop.rateLimitedCycles(), loop.clampedCycles());

    CHECK(loop.logger().allFinite(), "NaN/inf in the log");
    // Baseline measured BEFORE the per-axis damping was added, same weights/config: 63474 cycles,
    // 375847 allocations (4 per cycle in RobotModel::update + 2 per cycle while the DMPs step).
    // The damping term must not add any.
    constexpr std::size_t kBaselineCycles = 63474;
    constexpr unsigned long kBaselineAllocs = 375847;
    if (n == kBaselineCycles) {
      CHECK(allocs == kBaselineAllocs, "allocations %lu, baseline %lu", allocs, kBaselineAllocs);
      std::printf("allocation count equals the pre-damping baseline (%lu)\n", kBaselineAllocs);
    } else {
      CHECK(static_cast<double>(allocs) / static_cast<double>(n) <= 6.0 + 1e-9, "allocations per cycle %.3f",
            static_cast<double>(allocs) / static_cast<double>(n));
    }
    CHECK(loop.stopReason() == StopReason::kCompleted, "stop reason %s", toString(loop.stopReason()));
    CHECK(loop.hasFirstTarget() && (loop.firstTarget().p - loop.activationPose().p).norm() < 1e-9 &&
              loop.firstTarget().q.angularDistance(loop.activationPose().q) < 1e-9,
          "target_0 != activation pose");
    const double expected_cycles = (nominal + cfg.safety.stop_duration_s) / 1e-3;
    CHECK(std::fabs(static_cast<double>(n) - expected_cycles) <= 5.0, "cycles %zu, expected ~%.0f", n, expected_cycles);
    CHECK(std::fabs(final_epos - expected_disp) < 1e-9, "end-to-end displacement %.9f vs %.9f", final_epos, expected_disp);
  } catch (const std::exception& e) {
    std::printf("EXCEPTION: %s\n", e.what());
    return 2;
  }
  std::printf("\ntest_replay_dryrun: %d failure(s)\n", g_fail);
  return g_fail == 0 ? 0 : 1;
}
