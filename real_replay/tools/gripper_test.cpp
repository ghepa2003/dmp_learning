// Standalone Franka Hand open/close test WITHOUT an object. Only libfranka (franka::Gripper).
// Never calls grasp(): only homing() and move().
//
// usage: ./gripper_test <host> [--speed 0.03] [--closed-width 0.03] [--cycles 2] [--no-homing]
//
// Safety limits (hard-coded): speed <= 0.05 m/s, closed-width >= 0.015 m, cycles <= 5.

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

namespace {

constexpr double kMaxSpeed = 0.05;        // [m/s]
constexpr double kMinClosedWidth = 0.015;  // [m]
constexpr int kMaxCycles = 5;
constexpr double kMinMaxWidth = 0.05;  // [m] below this the fingers are not calibrated

void usage(const char* prog) {
  std::fprintf(stderr, "usage: %s <host> [--speed 0.03] [--closed-width 0.03] [--cycles 2] [--no-homing]\n", prog);
  std::fprintf(stderr, "limits: speed <= %.3f m/s, closed-width >= %.3f m, cycles <= %d\n", kMaxSpeed,
               kMinClosedWidth, kMaxCycles);
}

bool parseNumber(const char* s, double& out) {
  char* end = nullptr;
  out = std::strtod(s, &end);
  return end != s && *end == '\0';
}

void printState(const char* label, const franka::GripperState& s) {
  std::printf("  [%s] width=%.4f m  max_width=%.4f m  is_grasped=%d  temperature=%u C\n", label, s.width,
              s.max_width, s.is_grasped ? 1 : 0, static_cast<unsigned>(s.temperature));
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2 || std::string(argv[1]).rfind("--", 0) == 0) {
    usage(argv[0]);
    return 2;
  }
  const std::string host = argv[1];
  double speed = 0.03;
  double closed_width = 0.03;
  double cycles_d = 2.0;
  bool do_homing = true;

  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--no-homing") {
      do_homing = false;
    } else if (a == "--speed" || a == "--closed-width" || a == "--cycles") {
      double v = 0.0;
      if (i + 1 >= argc || !parseNumber(argv[i + 1], v)) {
        std::fprintf(stderr, "error: %s needs a numeric value\n", a.c_str());
        return 2;
      }
      ++i;
      (a == "--speed" ? speed : a == "--closed-width" ? closed_width : cycles_d) = v;
    } else {
      std::fprintf(stderr, "error: unknown argument '%s'\n", a.c_str());
      usage(argv[0]);
      return 2;
    }
  }

  if (!(speed > 0.0) || speed > kMaxSpeed) {
    std::fprintf(stderr, "error: --speed %.4f refused (must be in (0, %.3f] m/s)\n", speed, kMaxSpeed);
    return 2;
  }
  if (closed_width < kMinClosedWidth) {
    std::fprintf(stderr, "error: --closed-width %.4f refused (must be >= %.3f m)\n", closed_width, kMinClosedWidth);
    return 2;
  }
  if (cycles_d < 1.0 || cycles_d > kMaxCycles || cycles_d != static_cast<int>(cycles_d)) {
    std::fprintf(stderr, "error: --cycles %.2f refused (integer in [1, %d])\n", cycles_d, kMaxCycles);
    return 2;
  }
  const int cycles = static_cast<int>(cycles_d);

  using Clock = std::chrono::steady_clock;
  int steps_ok = 0;
  int steps_total = 0;
  franka::Gripper* gp = nullptr;

  try {
    franka::Gripper gripper(host);
    gp = &gripper;

    franka::GripperState st = gripper.readOnce();
    printState("initial", st);

    std::printf("speed=%.3f m/s  closed_width=%.4f m  cycles=%d  homing=%s\n", speed, closed_width, cycles,
                do_homing ? "yes" : "no");
    std::printf("Press Enter to continue (Ctrl+C to abort)...");
    std::fflush(stdout);
    std::cin.get();

    if (do_homing) {
      const auto t0 = Clock::now();
      const bool ok = gripper.homing();
      const double dt = std::chrono::duration<double>(Clock::now() - t0).count();
      std::printf("homing: ok=%d duration=%.2f s\n", ok ? 1 : 0, dt);
      if (!ok) throw std::runtime_error("homing() returned false");
      st = gripper.readOnce();
      printState("after homing", st);
    }

    // max_width is only valid once the fingers are calibrated (homing). Check it before any move.
    const double max_width = st.max_width;
    if (max_width < kMinMaxWidth) {
      std::fprintf(stderr, do_homing ? "error: homing did not calibrate the fingers (max_width=%.4f m < %.3f m)\n"
                                     : "error: gripper not homed: run without --no-homing (max_width=%.4f m < %.3f m)\n",
                   max_width, kMinMaxWidth);
      return 2;
    }
    if (closed_width >= max_width) {
      std::fprintf(stderr, "error: --closed-width %.4f >= max_width %.4f\n", closed_width, max_width);
      return 2;
    }

    for (int c = 1; c <= cycles; ++c) {
      for (int phase = 0; phase < 2; ++phase) {
        const double target = phase == 0 ? closed_width : max_width;
        const char* name = phase == 0 ? "close" : "open";
        ++steps_total;
        const auto t0 = Clock::now();
        const bool ok = gripper.move(target, speed);
        const double dt = std::chrono::duration<double>(Clock::now() - t0).count();
        if (!ok) {
          std::fprintf(stderr, "error: move(%.4f, %.3f) returned false (cycle %d, %s)\n", target, speed, c, name);
          throw std::runtime_error("move() returned false");
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        st = gripper.readOnce();
        std::printf("cycle %d %-5s commanded=%.4f m  measured=%.4f m  duration=%.2f s  is_grasped=%d\n", c, name,
                    target, st.width, dt, st.is_grasped ? 1 : 0);
        ++steps_ok;
      }
    }
    std::printf("SUMMARY: OK  cycles=%d steps=%d/%d speed=%.3f closed_width=%.4f final_width=%.4f\n", cycles,
                steps_ok, steps_total, speed, closed_width, st.width);
    return 0;
  } catch (const std::exception& e) {  // includes franka::Exception
    std::fprintf(stderr, "error: %s\n", e.what());
    if (gp != nullptr) {
      try {
        gp->stop();
        std::fprintf(stderr, "gripper.stop() sent\n");
      } catch (const std::exception& e2) {
        std::fprintf(stderr, "gripper.stop() failed: %s\n", e2.what());
      }
    }
    std::printf("SUMMARY: FAILED  steps=%d/%d\n", steps_ok, steps_total);
    return 1;
  }
}
