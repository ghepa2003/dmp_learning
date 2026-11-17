// Standalone Franka Hand grasp test with a rigid object. Only libfranka (franka::Gripper).
//
// usage: ./gripper_grasp_test <host> --object-width <m> [--force 5] [--speed 0.02] [--hold-s 10]
//                             [--eps-in 0.003] [--eps-out 0.003] [--homing 0] [--countdown-s 3] [--log <file.csv>]
//
// Safety limits (hard-coded, refused with exit code 2 before any motion):
//   force in (0, 10] N, speed in (0, 0.03] m/s, object-width in [0.005, 0.060] m,
//   hold-s in [1, 30] s, eps-in and eps-out in (0, 0.01] m, countdown-s integer in [1, 30].
// Exit codes: 0 ok, 1 runtime failure (grasp failed / exception / release not ok), 2 refused arguments or not homed.

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr double kMaxForce = 10.0;         // [N]
constexpr double kMaxSpeed = 0.03;         // [m/s]
constexpr double kMinObjectWidth = 0.005;  // [m]
constexpr double kMaxObjectWidth = 0.060;  // [m]
constexpr double kMinHold = 1.0;           // [s]
constexpr double kMaxHold = 30.0;          // [s]
constexpr double kMaxEps = 0.01;           // [m]
constexpr double kMinCountdown = 1.0;    // [s]
constexpr double kMaxCountdown = 30.0;   // [s]
constexpr double kMinMaxWidth = 0.05;      // [m] below this the fingers are not calibrated
constexpr double kReleaseTol = 0.002;      // [m]
constexpr double kAlreadyOpenTol = 0.002;  // [m]

using Clock = std::chrono::steady_clock;

void usage(const char* prog) {
  std::fprintf(stderr,
               "usage: %s <host> --object-width <m> [--force 5] [--speed 0.02] [--hold-s 10]\n"
               "          [--eps-in 0.003] [--eps-out 0.003] [--homing 0] [--countdown-s 3] [--log <file.csv>]\n"
               "limits: force in (0,%.0f] N, speed in (0,%.2f] m/s, object-width in [%.3f,%.3f] m,\n"
               "        hold-s in [%.0f,%.0f] s, eps-in/eps-out in (0,%.2f] m, countdown-s integer in [%.0f,%.0f]\n",
               prog, kMaxForce, kMaxSpeed, kMinObjectWidth, kMaxObjectWidth, kMinHold, kMaxHold, kMaxEps, kMinCountdown, kMaxCountdown);
}

bool parseNumber(const char* s, double& out) {
  char* end = nullptr;
  out = std::strtod(s, &end);
  return end != s && *end == '\0' && std::isfinite(out);
}

void printState(const char* label, const franka::GripperState& s) {
  std::printf("  [%s] width=%.4f m  max_width=%.4f m  is_grasped=%d  temperature=%u C\n", label, s.width,
              s.max_width, s.is_grasped ? 1 : 0, static_cast<unsigned>(s.temperature));
}

std::string defaultLogPath() {
  const std::time_t t = std::time(nullptr);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&t));
  return std::string("build/logs/") + buf + "_gripper_grasp.csv";
}

struct Sample {
  double t, width;
  bool grasped;
  unsigned temp;
};

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2 || std::string(argv[1]).rfind("--", 0) == 0) {
    usage(argv[0]);
    return 2;
  }
  const std::string host = argv[1];
  double object_width = -1.0, force = 5.0, speed = 0.02, hold_s = 10.0, eps_in = 0.003, eps_out = 0.003;
  double homing_d = 0.0, countdown_d = 3.0;
  bool have_object_width = false;
  std::string log_path;

  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--log") {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "error: --log needs a file name\n");
        return 2;
      }
      log_path = argv[++i];
      continue;
    }
    double* target = a == "--object-width" ? &object_width
                     : a == "--force"      ? &force
                     : a == "--speed"      ? &speed
                     : a == "--hold-s"     ? &hold_s
                     : a == "--eps-in"     ? &eps_in
                     : a == "--eps-out"    ? &eps_out
                     : a == "--homing"     ? &homing_d
                     : a == "--countdown-s" ? &countdown_d
                                           : nullptr;
    if (target == nullptr) {
      std::fprintf(stderr, "error: unknown argument '%s'\n", a.c_str());
      usage(argv[0]);
      return 2;
    }
    if (i + 1 >= argc || !parseNumber(argv[i + 1], *target)) {
      std::fprintf(stderr, "error: %s needs a numeric value\n", a.c_str());
      return 2;
    }
    ++i;
    if (a == "--object-width") have_object_width = true;
  }

  auto refuse = [](const char* msg, double v) {
    std::fprintf(stderr, "error: %s refused (got %.5f)\n", msg, v);
    return 2;
  };
  if (!have_object_width) {
    std::fprintf(stderr, "error: --object-width is required\n");
    usage(argv[0]);
    return 2;
  }
  if (!(force > 0.0) || force > kMaxForce) return refuse("--force must be in (0, 10] N", force);
  if (!(speed > 0.0) || speed > kMaxSpeed) return refuse("--speed must be in (0, 0.03] m/s", speed);
  if (object_width < kMinObjectWidth || object_width > kMaxObjectWidth)
    return refuse("--object-width must be in [0.005, 0.060] m", object_width);
  if (hold_s < kMinHold || hold_s > kMaxHold) return refuse("--hold-s must be in [1, 30] s", hold_s);
  if (!(eps_in > 0.0) || eps_in > kMaxEps) return refuse("--eps-in must be in (0, 0.01] m", eps_in);
  if (!(eps_out > 0.0) || eps_out > kMaxEps) return refuse("--eps-out must be in (0, 0.01] m", eps_out);
  if (homing_d != 0.0 && homing_d != 1.0) return refuse("--homing must be 0 or 1", homing_d);
  if (countdown_d < kMinCountdown || countdown_d > kMaxCountdown || countdown_d != std::floor(countdown_d))
    return refuse("--countdown-s must be an integer in [1, 30]", countdown_d);
  const int countdown_s = static_cast<int>(countdown_d);
  const bool do_homing = homing_d == 1.0;
  if (log_path.empty()) log_path = defaultLogPath();

  std::unique_ptr<franka::Gripper> gripper;
  double max_width = 0.0;
  bool motion_started = false;  // true once a move/grasp command may have been sent

  try {
    gripper = std::make_unique<franka::Gripper>(host);
    franka::GripperState st = gripper->readOnce();
    printState("initial", st);
    if (st.max_width < kMinMaxWidth) {
      if (!do_homing) {
        std::fprintf(stderr, "error: gripper not homed (max_width=%.4f m < %.3f m)\n", st.max_width, kMinMaxWidth);
        return 2;
      }
    }
    if (do_homing) {
      motion_started = true;
      if (!gripper->homing()) throw std::runtime_error("homing() returned false");
      st = gripper->readOnce();
      printState("after homing", st);
      if (st.max_width < kMinMaxWidth) {
        std::fprintf(stderr, "error: gripper not homed (max_width=%.4f m < %.3f m)\n", st.max_width, kMinMaxWidth);
        return 2;
      }
    }
    max_width = st.max_width;
    if (object_width >= max_width) {
      std::fprintf(stderr, "error: --object-width %.4f >= max_width %.4f\n", object_width, max_width);
      return 2;
    }

    // Open the fingers if they are not already open.
    if (st.width < max_width - kAlreadyOpenTol) {
      motion_started = true;
      if (!gripper->move(max_width, speed)) throw std::runtime_error("move(max_width) returned false");
      st = gripper->readOnce();
      printState("opened", st);
    }

    std::printf("object-width=%.4f m force=%.2f N speed=%.3f m/s hold=%.1f s eps-in=%.4f eps-out=%.4f\n",
                object_width, force, speed, hold_s, eps_in, eps_out);
    std::printf("Place the object between the open fingers, take your hands away, then press Enter\n");
    std::fflush(stdout);
    std::cin.get();
    for (int k = countdown_s; k >= 1; --k) {
      std::printf("closing in %d...\n", k);
      std::fflush(stdout);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::printf("closing now\n");
    std::fflush(stdout);

    // Grasp.
    motion_started = true;
    const auto g0 = Clock::now();
    const bool grasp_ok = gripper->grasp(object_width, speed, force, eps_in, eps_out);
    const double grasp_duration = std::chrono::duration<double>(Clock::now() - g0).count();
    std::printf("grasp: ok=%d duration=%.2f s\n", grasp_ok ? 1 : 0, grasp_duration);
    if (!grasp_ok) {
      std::fprintf(stderr, "error: grasp() returned false: reopening\n");
      try {
        gripper->move(max_width, speed);
      } catch (const std::exception& e) {
        std::fprintf(stderr, "reopen failed: %s\n", e.what());
      }
      std::printf("SUMMARY: success=0 grasp_duration_s=%.2f width_mean=nan width_std=nan drift=nan "
                  "is_grasped_all=0 release_ok=0 temperature=%u\n",
                  grasp_duration, static_cast<unsigned>(st.temperature));
      return 1;
    }

    // Hold phase, sampled about every 50 ms.
    std::vector<Sample> samples;
    samples.reserve(static_cast<std::size_t>(hold_s * 25));
    const auto h0 = Clock::now();
    auto next = h0;
    for (;;) {
      const double t = std::chrono::duration<double>(Clock::now() - h0).count();
      if (t > hold_s) break;
      const franka::GripperState s = gripper->readOnce();
      samples.push_back({t, s.width, s.is_grasped, static_cast<unsigned>(s.temperature)});
      next += std::chrono::milliseconds(50);
      std::this_thread::sleep_until(next);
    }

    // Statistics.
    double sum = 0.0, sum_first = 0.0, sum_last = 0.0;
    std::size_t n_first = 0, n_last = 0;
    bool grasped_all = true;
    for (const Sample& s : samples) {
      sum += s.width;
      grasped_all = grasped_all && s.grasped;
      if (s.t < 1.0) { sum_first += s.width; ++n_first; }
      if (s.t >= hold_s - 1.0) { sum_last += s.width; ++n_last; }
    }
    const double n = static_cast<double>(samples.size());
    const double mean = n > 0 ? sum / n : std::nan("");
    double var = 0.0;
    for (const Sample& s : samples) var += (s.width - mean) * (s.width - mean);
    const double stddev = n > 0 ? std::sqrt(var / n) : std::nan("");
    const double drift = (n_first > 0 && n_last > 0) ? sum_last / n_last - sum_first / n_first : std::nan("");
    const unsigned temperature = samples.empty() ? st.temperature : samples.back().temp;

    // CSV log.
    try {
      const std::filesystem::path p(log_path);
      if (p.has_parent_path()) std::filesystem::create_directories(p.parent_path());
      std::ofstream f(log_path);
      f << "t_s,width,is_grasped,temperature\n";
      for (const Sample& s : samples) f << s.t << ',' << s.width << ',' << (s.grasped ? 1 : 0) << ',' << s.temp << '\n';
      std::printf("log: %s (%zu samples)\n", log_path.c_str(), samples.size());
    } catch (const std::exception& e) {
      std::fprintf(stderr, "warning: cannot write log '%s': %s\n", log_path.c_str(), e.what());
    }

    // Release.
    if (!gripper->move(max_width, speed)) throw std::runtime_error("release move(max_width) returned false");
    const franka::GripperState fin = gripper->readOnce();
    printState("released", fin);
    const bool release_ok = fin.width >= max_width - kReleaseTol;

    std::printf("SUMMARY: success=%d grasp_duration_s=%.2f width_mean=%.5f width_std=%.5f drift=%.5f "
                "is_grasped_all=%d release_ok=%d temperature=%u\n",
                release_ok ? 1 : 0, grasp_duration, mean, stddev, drift, grasped_all ? 1 : 0, release_ok ? 1 : 0,
                temperature);
    return release_ok ? 0 : 1;
  } catch (const std::exception& e) {  // includes franka::Exception
    std::fprintf(stderr, "error: %s\n", e.what());
    if (gripper && motion_started) {
      if (max_width >= kMinMaxWidth) {
        try {
          gripper->move(max_width, speed);
          std::fprintf(stderr, "reopen sent\n");
        } catch (const std::exception& e2) {
          std::fprintf(stderr, "reopen failed: %s\n", e2.what());
        }
      }
      try {
        gripper->stop();
        std::fprintf(stderr, "gripper.stop() sent\n");
      } catch (const std::exception& e3) {
        std::fprintf(stderr, "gripper.stop() failed: %s\n", e3.what());
      }
    }
    std::printf("SUMMARY: success=0 error=exception\n");
    return 1;
  }
}
