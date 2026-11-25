// satellite_node: robot 2 ("satellite") process of the two-robot set-up. Rotates ONLY J7 of robot 2 at the phase
// speed scene.omega_deg_s of hw/hw_config.yaml, inside the admitted phase window, and publishes the satellite state
// over UDP for the task layer. Same control scheme as tools/satellite_j7.cpp (joint PD in torque at 1 kHz, J1-J6
// held at q0, J7 on the smoothstep profile of include/real_replay/j7_profile.hpp, SafetyGuard on every sent torque).
//
// usage: ./satellite_node --config <hw_config.yaml> [--dry-run] [--autostart] [--yes] [--max-travel-deg D]
//                         [--dry-q-deg q1,..,q7] [--pub-port P] [--monitor-port P] [--cmd-port P]
//                         [--log-dir DIR] [--log-tag TAG] [--countdown-s N] [--max-wait-start-s S]
//
// Sequence: preflight -> (robot: confirmation unless --yes, countdown) -> control loop in WAIT_START (holds q0)
//   -> START received over UDP (or --autostart, dry-run only) -> RUNNING (ramp, constant speed, nominal stop ramp
//   ending window_margin before the window edge) -> HOLD -> exit.
// A STOP command, Ctrl-C/SIGTERM, a heartbeat timeout (T_hb_s after the first heartbeat; START counts as one), a
// tracking error, a window violation (q7 or theta beyond the window EDGE) or a SafetyGuard trigger ALWAYS start a
// controlled stop ramp (stop_ramp_s) from the current reference velocity. In WAIT_START the robot is at rest: the
// node ends at once (zero motion).
//
// Phase: ONE way, the Panda DH forward kinematics of q (include/real_replay/satellite_phase.hpp), identical in robot
// and dry-run mode. In robot mode the preflight checks the DH flange pose against O_T_EE * F_T_EE^-1.
//
// Threads: the 1 kHz loop never allocates and never calls network syscalls. It writes the latest state into a
// seqlock (LatestValue) and reads the command counters (atomics). A publisher thread sends the state datagram at
// udp.rate_hz (non-blocking sendto); a receiver thread reads the command port.
//
// Exit codes (same meaning as satellite_j7): 0 completed (window end reached) or STOP command, orderly;
//   1 runtime/preflight failure; 2 refused arguments/config; 3 stopped by Ctrl-C / heartbeat / tracking / window /
//   safety / START timeout, after a controlled ramp (or at rest); 4 the stop ramp could NOT be completed (libfranka
//   error, non-finite state): the robot may be in reflex.

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/lowpass_filter.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "real_replay/config.hpp"
#include "real_replay/hw_config.hpp"
#include "real_replay/j7_profile.hpp"
#include "real_replay/preflight_checks.hpp"
#include "real_replay/safety.hpp"
#include "real_replay/satellite_phase.hpp"
#include "real_replay/satellite_udp.hpp"
#include "real_replay/sha256.hpp"

using real_replay::Array7;
using real_replay::J7Profile;
using real_replay::J7ProfileParams;
using real_replay::J7Ref;
namespace sat = real_replay::sat;

namespace {

constexpr double kDeg = sat::kDeg;
constexpr double kDt = 1e-3;
constexpr double kMaxSpeedDegS = 5.0;  // same hard maximum as satellite_j7

volatile std::sig_atomic_t g_sig = 0;
void onSignal(int) { g_sig = 1; }

void installSignals() {
  struct sigaction sa {};
  sa.sa_handler = onSignal;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;  // no SA_RESTART: std::cin.get() and recvfrom() must be interrupted
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);
}

std::uint64_t monoNs() noexcept {  // CLOCK_MONOTONIC (vDSO, no real syscall)
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ull + static_cast<std::uint64_t>(ts.tv_nsec);
}

std::string fmt(const char* f, double a = 0.0, double b = 0.0, double c = 0.0, double d = 0.0) {
  char buf[400];
  std::snprintf(buf, sizeof buf, f, a, b, c, d);
  return buf;
}

// ---- CLI --------------------------------------------------------------------------------------------
struct Cli {
  std::string config;
  bool dry_run = false;
  bool autostart = false;
  bool yes = false;
  double max_travel_deg = 0.0;  // 0 = from the config
  bool dry_q_given = false;
  Array7 dry_q{};
  int pub_port = -1, mon_port = -1, cmd_port = -1;  // -1 = from the config
  std::string log_dir, log_tag;
  int countdown_s = -1;
  double max_wait_start_s = 0.0;  // 0 = from the config
  std::string argv_line;
};

void usage(const char* p) {
  std::fprintf(stderr,
               "usage: %s --config <hw_config.yaml> [--dry-run] [--autostart] [--yes] [--max-travel-deg D]\n"
               "          [--dry-q-deg q1,..,q7] [--pub-port P] [--monitor-port P (0 = off)] [--cmd-port P]\n"
               "          [--log-dir DIR] [--log-tag TAG] [--countdown-s N] [--max-wait-start-s S]\n"
               "--autostart is accepted only with --dry-run. --dry-q-deg: initial q of the dry-run (default robot2.start.q_deg).\n",
               p);
}

bool parseNumber(const std::string& s, double& out) {
  char* end = nullptr;
  out = std::strtod(s.c_str(), &end);
  return !s.empty() && end != s.c_str() && *end == '\0' && std::isfinite(out);
}

bool parseList7(const std::string& s, Array7& out) {
  std::stringstream ss(s);
  std::string tok;
  std::size_t n = 0;
  Array7 tmp{};
  while (std::getline(ss, tok, ',')) {
    if (n >= 7 || !parseNumber(tok, tmp[n++])) return false;
  }
  if (n != 7) return false;
  out = tmp;
  return true;
}

std::string parseCli(int argc, char** argv, Cli& cli) {
  for (int i = 1; i < argc; ++i) {
    if (i > 1) cli.argv_line += ' ';
    cli.argv_line += argv[i];
  }
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--dry-run") {
      cli.dry_run = true;
      continue;
    }
    if (a == "--autostart") {
      cli.autostart = true;
      continue;
    }
    if (a == "--yes") {
      cli.yes = true;
      continue;
    }
    if (a == "-h" || a == "--help") return "help";
    if (i + 1 >= argc) return a + " needs a value";
    const std::string v = argv[++i];
    double d = 0.0;
    auto port = [&](int& dst, bool allow_zero) {
      if (!parseNumber(v, d) || d != std::floor(d) || d < (allow_zero ? 0 : 1) || d > 65535) return false;
      dst = static_cast<int>(d);
      return true;
    };
    bool ok = true;
    if (a == "--config") {
      cli.config = v;
    } else if (a == "--max-travel-deg") {
      ok = parseNumber(v, d) && d > 0.0 && d <= 360.0;
      cli.max_travel_deg = d;
    } else if (a == "--dry-q-deg") {
      ok = parseList7(v, cli.dry_q);
      for (double& x : cli.dry_q) x *= kDeg;
      cli.dry_q_given = ok;
    } else if (a == "--pub-port") {
      ok = port(cli.pub_port, false);
    } else if (a == "--monitor-port") {
      ok = port(cli.mon_port, true);
    } else if (a == "--cmd-port") {
      ok = port(cli.cmd_port, false);
    } else if (a == "--log-dir") {
      cli.log_dir = v;
    } else if (a == "--log-tag") {
      cli.log_tag = v;
    } else if (a == "--countdown-s") {
      ok = parseNumber(v, d) && d == std::floor(d) && d >= 0 && d <= 30;
      cli.countdown_s = static_cast<int>(d);
    } else if (a == "--max-wait-start-s") {
      ok = parseNumber(v, d) && d > 0.0 && d <= 3600.0;
      cli.max_wait_start_s = d;
    } else {
      return "unknown argument '" + a + "'";
    }
    if (!ok) return "invalid value '" + v + "' for " + a;
  }
  if (cli.config.empty()) return "--config is required";
  if (cli.autostart && !cli.dry_run) return "--autostart is allowed only with --dry-run";
  return "";
}

// ---- Communication (outside the 1 kHz loop) ---------------------------------------------------------
struct Comms {
  std::atomic<std::uint32_t> hb_count{0};
  std::atomic<std::uint32_t> start_count{0};
  std::atomic<std::uint32_t> stop_count{0};
  std::atomic<std::uint32_t> bad_datagrams{0};
  std::atomic<std::uint64_t> send_errors{0};
  std::atomic<std::uint64_t> sent{0};
  std::atomic<bool> quit{false};
  sat::LatestValue<sat::StatePacket> latest;

  int pub_fd = -1, cmd_fd = -1;
  sockaddr_in pub_addr{}, mon_addr{};
  bool monitor = false;
  double rate_hz = 100.0;
  std::thread pub_thread, rx_thread;

  ~Comms() { stop(); }

  void open(const std::string& host, int pub_port, int mon_port, int cmd_port, double rate) {
    rate_hz = rate;
    auto addr = [&](int port) {
      sockaddr_in a{};
      a.sin_family = AF_INET;
      a.sin_port = htons(static_cast<std::uint16_t>(port));
      if (inet_pton(AF_INET, host.c_str(), &a.sin_addr) != 1) throw std::runtime_error("udp.host '" + host + "' is not an IPv4 address");
      return a;
    };
    pub_addr = addr(pub_port);
    monitor = mon_port > 0;
    if (monitor) mon_addr = addr(mon_port);
    pub_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (pub_fd < 0) throw std::runtime_error("socket() failed for the publisher");
    cmd_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (cmd_fd < 0) throw std::runtime_error("socket() failed for the command port");
    const sockaddr_in ca = addr(cmd_port);
    if (::bind(cmd_fd, reinterpret_cast<const sockaddr*>(&ca), sizeof ca) != 0) {
      throw std::runtime_error("cannot bind the command port " + host + ":" + std::to_string(cmd_port) + " (" + std::strerror(errno) +
                               "): another satellite_node running?");
    }
    timeval tv{};
    tv.tv_usec = 50000;
    setsockopt(cmd_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
  }

  /// Drops every datagram already queued on the command port (e.g. a START/STOP sent before this run), so that
  /// only commands sent after this point count. Returns how many were dropped.
  int drainCommands() {
    std::uint8_t buf[256];
    int n = 0;
    while (::recv(cmd_fd, buf, sizeof buf, MSG_DONTWAIT) >= 0) ++n;  // until EAGAIN/EWOULDBLOCK
    return n;
  }

  void start() {
    const int dropped = drainCommands();
    std::printf("udp: command port drained before start: %d stale datagram(s) discarded\n", dropped);
    std::fflush(stdout);
    pub_thread = std::thread([this] { publishLoop(); });
    rx_thread = std::thread([this] { receiveLoop(); });
  }

  void stop() {
    quit.store(true);
    if (pub_thread.joinable()) pub_thread.join();
    if (rx_thread.joinable()) rx_thread.join();
    if (pub_fd >= 0) ::close(pub_fd);
    if (cmd_fd >= 0) ::close(cmd_fd);
    pub_fd = cmd_fd = -1;
  }

  void sendOne(std::uint32_t& seq) {
    sat::StatePacket p;
    if (!latest.load(p)) return;
    p.magic = sat::kStateMagic;
    p.version = sat::kProtocolVersion;
    p.size = static_cast<std::uint16_t>(sat::kStatePacketSize);
    p.seq = ++seq;
    std::uint8_t buf[sat::kStatePacketSize];
    sat::packState(p, buf);
    // Non-blocking: a full socket buffer drops the datagram (the consumer checks seq and age).
    if (::sendto(pub_fd, buf, sizeof buf, MSG_DONTWAIT, reinterpret_cast<const sockaddr*>(&pub_addr), sizeof pub_addr) < 0) ++send_errors;
    if (monitor && ::sendto(pub_fd, buf, sizeof buf, MSG_DONTWAIT, reinterpret_cast<const sockaddr*>(&mon_addr), sizeof mon_addr) < 0) {
      ++send_errors;
    }
    ++sent;
  }

  void publishLoop() {
    std::uint32_t seq = 0;
    const auto period = std::chrono::nanoseconds(static_cast<long long>(1e9 / rate_hz));
    auto next = std::chrono::steady_clock::now();
    while (!quit.load()) {
      next += period;
      std::this_thread::sleep_until(next);
      sendOne(seq);
    }
    // a few more datagrams so that listeners see the final state (DONE)
    for (int i = 0; i < 5; ++i) {
      sendOne(seq);
      std::this_thread::sleep_for(period);
    }
  }

  void receiveLoop() {
    std::uint8_t buf[256];
    while (!quit.load()) {
      const ssize_t n = ::recv(cmd_fd, buf, sizeof buf, 0);
      if (n < 0) continue;  // timeout or EINTR
      sat::CommandPacket c;
      if (!sat::unpackCommand(buf, static_cast<std::size_t>(n), c)) {
        ++bad_datagrams;
        continue;
      }
      switch (static_cast<sat::CommandType>(c.type)) {
        case sat::CommandType::kHeartbeat: ++hb_count; break;
        case sat::CommandType::kStart:
          ++start_count;
          ++hb_count;  // START also arms / feeds the heartbeat monitor
          break;
        case sat::CommandType::kStop: ++stop_count; break;
      }
    }
  }
};

// ---- Log --------------------------------------------------------------------------------------------
struct Rec {
  double t, period, cb_us, state;
  double q[7];
  double dq7, q7_ref, dq7_ref, e_q7, tau7;
  double theta_flange, theta_q7, omega_meas, omega_cmd, omega_valid;
  double center[3];
  double faults;
};
static_assert(sizeof(Rec) % sizeof(double) == 0, "Rec must be a flat array of doubles");
constexpr std::size_t kRecFields = sizeof(Rec) / sizeof(double);

std::string csvHeader() {
  std::string h = "t,period,cb_us,state";
  for (int k = 1; k <= 7; ++k) h += ",q" + std::to_string(k);
  h += ",dq7,q7_ref,dq7_ref,e_q7,tau7,theta_flange_deg,theta_q7_deg,omega_meas_deg_s,omega_cmd_deg_s,omega_valid,cx,cy,cz,faults";
  return h;
}

std::string writeCsv(const std::string& path, const std::vector<Rec>& rec) {
  std::FILE* f = std::fopen(path.c_str(), "wx");
  if (!f) return "cannot create '" + path + "' (exists already, or not writable)";
  std::fprintf(f, "%s\n", csvHeader().c_str());
  for (const Rec& r : rec) {
    const double* d = reinterpret_cast<const double*>(&r);
    for (std::size_t k = 0; k < kRecFields; ++k) std::fprintf(f, k == 0 ? "%.9g" : ",%.9g", d[k]);
    std::fputc('\n', f);
  }
  if (std::fclose(f) != 0) return "close error on '" + path + "'";
  return "";
}

std::string timestampMs() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  const long ms = static_cast<long>(std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000);
  std::tm tmv{};
  localtime_r(&t, &tmv);
  char buf[48];
  std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d_%03ld", tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday, tmv.tm_hour,
                tmv.tm_min, tmv.tm_sec, ms);
  return buf;
}

std::string logPath(const std::string& dir, const std::string& tag, const char* kind) {
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  std::string name = timestampMs() + "_satellite_node_";
  for (char c : tag) name += (std::isalnum(static_cast<unsigned char>(c)) || c == '-' || c == '_') ? c : '_';
  if (!tag.empty()) name += "_";
  name += std::string(kind) + ".csv";
  const std::filesystem::path p = std::filesystem::path(dir) / name;
  if (std::filesystem::exists(p)) throw std::runtime_error("log file already exists, refusing to overwrite: " + p.string());
  return p.string();
}

// ---- Controller (robot-free; robot callback and dry-run) ---------------------------------------------
struct StepIn {
  Array7 q{}, dq{};
  double period_s = 0.0;
  double success_rate = 1.0;
};

enum class Stop { kNone, kCompleted, kStopCommand, kUserSignal, kSafety, kTracking, kWindow, kHeartbeat, kWaitTimeout, kLogFull, kNonFinite };

const char* toString(Stop s) {
  switch (s) {
    case Stop::kNone: return "none";
    case Stop::kCompleted: return "completed";
    case Stop::kStopCommand: return "stop_command";
    case Stop::kUserSignal: return "user_signal";
    case Stop::kSafety: return "safety_guard";
    case Stop::kTracking: return "tracking_error";
    case Stop::kWindow: return "window";
    case Stop::kHeartbeat: return "heartbeat_timeout";
    case Stop::kWaitTimeout: return "start_timeout";
    case Stop::kLogFull: return "log_full";
    case Stop::kNonFinite: return "non_finite_state";
  }
  return "unknown";
}

std::uint32_t faultBit(Stop s) {
  switch (s) {
    case Stop::kStopCommand: return sat::kFaultStopCommand;
    case Stop::kUserSignal: return sat::kFaultSignal;
    case Stop::kSafety: return sat::kFaultSafety;
    case Stop::kTracking: return sat::kFaultTracking;
    case Stop::kWindow: return sat::kFaultWindow;
    case Stop::kHeartbeat: return sat::kFaultHeartbeat;
    case Stop::kWaitTimeout: return sat::kFaultWaitTimeout;
    case Stop::kLogFull: return sat::kFaultLogFull;
    case Stop::kNonFinite: return sat::kFaultNonFinite;
    default: return 0;
  }
}

struct Params {
  int sign = +1;                ///< q7_ref = q7_0 + sign * d
  J7ProfileParams prof;
  Array7 kp{}, kd{};
  double max_track_err = 0.05;
  double q7_lo = 0.0, q7_hi = 0.0;    ///< window EDGES [rad] (guard); the profile ends margin before
  double psi_lo = 0.0, psi_hi = 0.0;  ///< phase window edges [rad]
  sat::PhaseParams phase;
  double omega_window_s = 5.0, omega_sample_s = 0.01;
  double mismatch_warn = 0.5 * kDeg;
  double hb_timeout_s = 0.5;
  double max_wait_s = 120.0;
  bool autostart = false;
  double omega_cmd_nominal = 0.0;  ///< scene omega [rad/s]
};

class Controller {
 public:
  Controller(const real_replay::SafetyConfig& safety, const Params& p, const Array7& q0, Comms& comms)
      : p_(p),
        prof_(p.prof),
        guard_(safety, sat::kQMin, sat::kQMax),
        q0_(q0),
        comms_(comms),
        omega_(p.omega_window_s, p.omega_sample_s),
        hb_(p.hb_timeout_s) {
    guard_.reset();
    cap_ = static_cast<std::size_t>((p.max_wait_s + prof_.totalDurationS() + p.prof.stop_ramp_s + 5.0) * 1100.0) + 1000;
    rec_.reserve(cap_);
    publish(sat::SatState::kInit, sat::computeSatGeom(p_.phase, q0_), q0_[6], 0.0, 0u);
  }

  bool step(const StepIn& in, Array7& tau) {
    tau.fill(0.0);
    if (phase_ == Phase::kDone) return false;
    for (std::size_t k = 0; k < 7; ++k) {
      if (!std::isfinite(in.q[k]) || !std::isfinite(in.dq[k])) return fatal("non-finite q/dq");
    }
    const double dt = (std::isfinite(in.period_s) && in.period_s >= 0.0) ? in.period_s : 0.0;
    if (cycle_ > 0) t_ += dt;

    // geometry / phase / omega (same code in robot and dry-run)
    const sat::SatGeom g = sat::computeSatGeom(p_.phase, in.q);
    if (t_ >= next_omega_t_) {
      omega_.push(t_, g.theta_flange);
      next_omega_t_ += p_.omega_sample_s;
      if (next_omega_t_ < t_) next_omega_t_ = t_ + p_.omega_sample_s;
    }
    const double mismatch = std::fabs(sat::angleDiff(g.theta_flange, g.theta_q7));
    max_mismatch_ = std::max(max_mismatch_, mismatch);
    if (mismatch > p_.mismatch_warn) faults_ |= sat::kFaultPhaseMismatch;

    // ---- supervision ----
    const double t_run = phase_ == Phase::kWait ? 0.0 : t_ - t_start_;
    const J7Ref nom = prof_.sample(t_run);
    const bool hb_timeout = hb_.update(t_, comms_.hb_count.load(std::memory_order_relaxed));
    if (phase_ == Phase::kWait || phase_ == Phase::kRun) {
      Stop trig = Stop::kNone;
      std::string why;
      auto set = [&](Stop s, const std::string& w) {
        if (trig == Stop::kNone) {
          trig = s;
          why = w;
        }
      };
      if (g_sig != 0) set(Stop::kUserSignal, "Ctrl-C / SIGTERM");
      if (comms_.stop_count.load(std::memory_order_relaxed) != 0) set(Stop::kStopCommand, "STOP command received");
      if (hb_timeout) set(Stop::kHeartbeat, "no heartbeat for more than T_hb_s");
      {
        const real_replay::StopReason r = guard_.checkCycle(in.period_s, last_cb_us_, in.success_rate, cycle_);
        if (r != real_replay::StopReason::kNone) set(Stop::kSafety, std::string("SafetyGuard: ") + real_replay::toString(r));
      }
      {
        const real_replay::StopReason r = guard_.checkState(in.q, in.dq, 0.0, 0.0);
        if (r != real_replay::StopReason::kNone) set(Stop::kSafety, std::string("SafetyGuard: ") + real_replay::toString(r));
      }
      for (std::size_t k = 0; k < 6; ++k) {
        if (std::fabs(q0_[k] - in.q[k]) > p_.max_track_err) set(Stop::kTracking, "held joint " + std::to_string(k + 1) + " deviates");
      }
      const double e7n = (q0_[6] + p_.sign * nom.d) - in.q[6];
      if (std::fabs(e7n) > p_.max_track_err) set(Stop::kTracking, "J7 tracking error " + fmt("%.4f", e7n) + " rad");
      if (in.q[6] < p_.q7_lo || in.q[6] > p_.q7_hi) set(Stop::kWindow, "q7 " + fmt("%.2f", in.q[6] / kDeg) + " deg beyond the window edge");
      if (g.theta_flange < p_.psi_lo || g.theta_flange > p_.psi_hi) {
        set(Stop::kWindow, "theta " + fmt("%.2f", g.theta_flange / kDeg) + " deg beyond the window edge");
      }
      if (phase_ == Phase::kWait && t_ > p_.max_wait_s) set(Stop::kWaitTimeout, "no START within max_wait_start_s");
      if (rec_.size() + 2 >= cap_) set(Stop::kLogFull, "log buffer full");

      if (trig != Stop::kNone) {
        stop_ = trig;
        stop_msg_ = why;
        faults_ |= faultBit(trig);
        if (phase_ == Phase::kWait) {  // at rest: end now
          phase_ = Phase::kHold;
          d_hold_ = 0.0;
          t_hold_ = t_;
          hold_len_ = 0.0;
        } else {
          phase_ = Phase::kStopping;
          d_stop_ = nom.d;
          v_stop_ = nom.v;
          t_stop_ = t_;
          hold_len_ = 0.0;
        }
      } else if (phase_ == Phase::kWait && (p_.autostart || comms_.start_count.load(std::memory_order_relaxed) != 0)) {
        phase_ = Phase::kRun;
        t_start_ = t_;
      }
    }

    // ---- reference ----
    J7Ref ref;
    bool finish = false;
    if (phase_ == Phase::kWait) {
      ref = {0.0, 0.0};
    } else if (phase_ == Phase::kRun) {
      ref = prof_.sample(t_ - t_start_);
      if (t_ - t_start_ >= prof_.motionDurationS()) {
        phase_ = Phase::kHold;
        stop_ = Stop::kCompleted;
        d_hold_ = prof_.params().travel_rad;
        t_hold_ = t_;
        hold_len_ = prof_.params().hold_s;
        ref = {d_hold_, 0.0};
      }
    } else if (phase_ == Phase::kStopping) {
      const double ts = t_ - t_stop_;
      ref = real_replay::stopFrom(d_stop_, v_stop_, prof_.params().stop_ramp_s, ts);
      if (ts >= prof_.params().stop_ramp_s) {
        phase_ = Phase::kHold;
        d_hold_ = ref.d;
        t_hold_ = t_;
      }
    } else {
      ref = {d_hold_, 0.0};
    }
    if (phase_ == Phase::kHold && (t_ - t_hold_ >= hold_len_ || g_sig != 0)) finish = true;

    // ---- joint PD ----
    Array7 des{};
    for (std::size_t k = 0; k < 6; ++k) {
      const double e = q0_[k] - in.q[k];
      max_held_err_ = std::max(max_held_err_, std::fabs(e));
      des[k] = p_.kp[k] * e - p_.kd[k] * in.dq[k];
    }
    const double q7_ref = q0_[6] + p_.sign * ref.d;
    const double dq7_ref = p_.sign * ref.v;
    const double e7 = q7_ref - in.q[6];
    max_e7_ = std::max(max_e7_, std::fabs(e7));
    des[6] = p_.kp[6] * e7 + p_.kd[6] * (dq7_ref - in.dq[6]);
    for (std::size_t k = 0; k < 7; ++k) {
      if (!std::isfinite(des[k])) return fatal("non-finite torque demand");
    }
    guard_.limitTorque(des, tau);

    const double omega_cmd = static_cast<double>(p_.phase.q7_sign) * dq7_ref;  // d(theta_q7)/dt of the reference
    const sat::SatState st = finish ? sat::SatState::kDone : stateOf();
    // omega_valid: only while the nominal profile runs and the whole regression window is at constant speed
    const bool omega_valid = sat::omegaValid(phase_ == Phase::kRun && !finish, t_ - t_start_, p_.prof.ramp_s, p_.omega_window_s,
                                             prof_.constantEndS(), omega_.last());
    publish(st, g, in.q[6], omega_cmd, omega_valid ? sat::kFlagOmegaValid : 0u);

    if (rec_.size() < cap_) {
      Rec r{};
      r.t = t_;
      r.period = in.period_s;
      r.state = static_cast<double>(static_cast<std::uint32_t>(st));
      for (std::size_t k = 0; k < 7; ++k) r.q[k] = in.q[k];
      r.dq7 = in.dq[6];
      r.q7_ref = q7_ref;
      r.dq7_ref = dq7_ref;
      r.e_q7 = e7;
      r.tau7 = tau[6];
      r.theta_flange = g.theta_flange / kDeg;
      r.theta_q7 = g.theta_q7 / kDeg;
      r.omega_meas = omega_.last() / kDeg;
      r.omega_cmd = omega_cmd / kDeg;
      r.omega_valid = omega_valid ? 1.0 : 0.0;
      for (int k = 0; k < 3; ++k) r.center[k] = g.center[k];
      r.faults = static_cast<double>(faults_);
      rec_.push_back(r);
    }
    ++cycle_;
    if (finish) {
      phase_ = Phase::kDone;
      return false;
    }
    return true;
  }

  void setLastCallbackUs(double us) {
    last_cb_us_ = us;
    max_cb_us_ = std::max(max_cb_us_, us);
    if (!rec_.empty()) rec_.back().cb_us = us;
  }
  void markAbnormal(const std::string& msg) {
    stop_ = Stop::kNonFinite;
    stop_msg_ = msg;
    faults_ |= sat::kFaultNonFinite;
    phase_ = Phase::kDone;
    publishFinal();
  }
  /// Final DONE datagram content (after the control loop ended).
  void publishFinal() {
    sat::StatePacket p;
    if (comms_.latest.load(p)) {
      p.state = static_cast<std::uint32_t>(sat::SatState::kDone);
      p.faults = faults_;
      p.flags = 0;
      p.t_mono_ns = monoNs();
      comms_.latest.store(p);
    }
  }

  Stop stopReason() const { return stop_; }
  const std::string& stopMessage() const { return stop_msg_; }
  bool orderlyEnd() const { return phase_ == Phase::kDone && stop_ != Stop::kNonFinite; }
  const std::vector<Rec>& records() const { return rec_; }
  long cycles() const { return cycle_; }
  const J7Profile& profile() const { return prof_; }
  std::uint32_t faults() const { return faults_; }
  double maxMismatch() const { return max_mismatch_; }
  double maxHeldErr() const { return max_held_err_; }
  double maxE7() const { return max_e7_; }
  double maxCbUs() const { return max_cb_us_; }
  double startTime() const { return t_start_; }

 private:
  enum class Phase { kWait, kRun, kStopping, kHold, kDone };

  sat::SatState stateOf() const {
    switch (phase_) {
      case Phase::kWait: return sat::SatState::kWaitStart;
      case Phase::kRun: return sat::SatState::kRunning;
      case Phase::kStopping: return sat::SatState::kStopping;
      case Phase::kHold: return sat::SatState::kHold;
      case Phase::kDone: return sat::SatState::kDone;
    }
    return sat::SatState::kDone;
  }

  void publish(sat::SatState st, const sat::SatGeom& g, double q7, double omega_cmd, std::uint32_t flags) noexcept {
    sat::StatePacket p;
    p.state = static_cast<std::uint32_t>(st);
    p.t_mono_ns = monoNs();
    p.theta = g.theta_flange;
    p.omega_meas = omega_.last();
    p.theta_q7 = g.theta_q7;
    p.q7 = q7;
    p.omega_cmd = omega_cmd;
    for (int k = 0; k < 3; ++k) {
      p.center[k] = g.center[k];
      p.axis[k] = g.axis_up[k];
    }
    p.faults = faults_;
    p.flags = flags;
    comms_.latest.store(p);
  }

  bool fatal(const char* msg) {
    stop_ = Stop::kNonFinite;
    stop_msg_ = msg;
    faults_ |= sat::kFaultNonFinite;
    phase_ = Phase::kDone;
    return false;
  }

  Params p_;
  J7Profile prof_;
  real_replay::SafetyGuard guard_;
  Array7 q0_;
  Comms& comms_;
  sat::OmegaRegressor omega_;
  sat::HeartbeatMonitor hb_;
  std::vector<Rec> rec_;
  std::size_t cap_ = 0;

  Phase phase_ = Phase::kWait;
  Stop stop_ = Stop::kNone;
  std::string stop_msg_;
  std::uint32_t faults_ = 0;
  long cycle_ = 0;
  double t_ = 0.0, t_start_ = 0.0, next_omega_t_ = 0.0;
  double last_cb_us_ = 0.0, max_cb_us_ = 0.0;
  double t_stop_ = 0.0, d_stop_ = 0.0, v_stop_ = 0.0;
  double d_hold_ = 0.0, t_hold_ = 0.0, hold_len_ = 0.0;
  double max_mismatch_ = 0.0, max_held_err_ = 0.0, max_e7_ = 0.0;
};

int exitCodeFor(const Controller& c) {
  if (c.stopReason() == Stop::kCompleted || c.stopReason() == Stop::kStopCommand) return c.orderlyEnd() ? 0 : 4;
  if (!c.orderlyEnd()) return 4;
  return 3;
}

// ---- Parameters and the preflight shared by robot and dry-run ----------------------------------------
sat::PhaseParams phaseParams(const real_replay::HwConfig& hw) {
  sat::PhaseParams p;
  p.collar_offset_rad = hw.scene.collar_phase_offset_rad;
  p.q7_offset_rad = hw.scene.q7_phase_offset_rad;
  p.q7_sign = hw.scene.q7_phase_sign;
  p.flange_to_center_m = hw.scene.flange_to_center_m;
  p.base2_in_base1 = sat::BaseTransform::fromYaw(hw.scene.base2_t_m, hw.scene.base2_yaw_rad);
  return p;
}

/// Builds the parameters for the start configuration q0 and prints the checks. Returns the number of failures.
int makeParams(const Cli& cli, const real_replay::HwConfig& hw, const real_replay::Config& rr, const Array7& q0, Params& p) {
  int failures = 0;
  auto check = [&](const char* name, bool ok, const std::string& detail) {
    std::printf("  [%s] %s: %s\n", ok ? "PASS" : "FAIL", name, detail.c_str());
    if (!ok) ++failures;
  };
  const real_replay::HwRobot2& r2 = hw.robot2;
  p.phase = phaseParams(hw);
  p.kp = r2.kp;
  p.kd = r2.kd;
  p.max_track_err = r2.max_track_err_rad;
  p.q7_lo = r2.q7_lo_rad;
  p.q7_hi = r2.q7_hi_rad;
  p.psi_lo = hw.scene.psi_lo_rad;
  p.psi_hi = hw.scene.psi_hi_rad;
  p.omega_window_s = r2.omega_window_s;
  p.omega_sample_s = 1.0 / r2.omega_sample_hz;
  p.mismatch_warn = r2.phase_mismatch_warn_rad;
  p.hb_timeout_s = hw.udp.T_hb_s;
  p.max_wait_s = cli.max_wait_start_s > 0.0 ? cli.max_wait_start_s : r2.max_wait_start_s;
  p.autostart = cli.autostart;
  p.omega_cmd_nominal = hw.scene.omega_rad_s;

  const double w = hw.scene.omega_rad_s;
  std::printf("preflight (geometry, window, profile):\n");
  check("omega != 0 and |omega| <= 5 deg/s", w != 0.0 && std::fabs(w) <= kMaxSpeedDegS * kDeg, fmt("omega = %.3f deg/s", w / kDeg));
  if (failures) return failures;
  // d(theta)/dt = q7_sign * dq7  ->  dq7 = omega * q7_sign  ->  sign = sgn(omega) * q7_sign
  p.sign = (w > 0.0 ? 1 : -1) * hw.scene.q7_phase_sign;
  p.prof.speed_rad_s = std::fabs(w);
  p.prof.ramp_s = r2.ramp_s;
  p.prof.stop_ramp_s = r2.stop_ramp_s;
  p.prof.hold_s = r2.hold_s;

  const sat::SatGeom g = sat::computeSatGeom(p.phase, q0);
  const Eigen::Vector3d c_cfg(hw.scene.center_m[0], hw.scene.center_m[1], hw.scene.center_m[2]);
  const Eigen::Vector3d ax_cfg = sat::axisFromTilt(hw.scene.tilt_x_rad, hw.scene.tilt_y_rad);
  std::printf("  axis from scene.axis_tilt_deg: [%.5f %.5f %.5f] (frame 1, upwards)\n", ax_cfg[0], ax_cfg[1], ax_cfg[2]);
  std::printf("  q0 [deg] = [");
  for (double x : q0) std::printf(" %.3f", x / kDeg);
  std::printf(" ]\n  flange (frame 1) [%.4f %.4f %.4f] m, centre [%.4f %.4f %.4f] m, axis [%.5f %.5f %.5f]\n", g.flange[0], g.flange[1],
              g.flange[2], g.center[0], g.center[1], g.center[2], g.axis_up[0], g.axis_up[1], g.axis_up[2]);
  const double dc = (g.center - c_cfg).norm();
  check("centre from FK vs scene.center_m", dc <= r2.tol_center_m, fmt("|diff| %.2f mm (tol %.2f mm)", dc * 1e3, r2.tol_center_m * 1e3));
  const double dax = std::acos(std::clamp(g.axis_up.dot(ax_cfg), -1.0, 1.0));
  check("axis from FK vs scene.axis_tilt_deg", dax <= r2.tol_axis_rad, fmt("angle %.3f deg (tol %.3f deg)", dax / kDeg, r2.tol_axis_rad / kDeg));
  const double mm = std::fabs(sat::angleDiff(g.theta_flange, g.theta_q7));
  std::printf("  theta0 from flange %.3f deg, from q7 %.3f deg, |diff| %.3f deg%s\n", g.theta_flange / kDeg, g.theta_q7 / kDeg, mm / kDeg,
              mm > r2.phase_mismatch_warn_rad ? "  [WARN] above phase_mismatch_warn_deg: the q7 formula does not hold here" : "");
  const double m = r2.window_margin_rad;
  check("theta0 inside the phase window with margin", sat::insideWindow(g.theta_flange, p.psi_lo, p.psi_hi, m),
        fmt("theta0 %.2f deg, admitted [%.2f, %.2f] deg", g.theta_flange / kDeg, (p.psi_lo + m) / kDeg, (p.psi_hi - m) / kDeg));

  const double max_travel = cli.max_travel_deg > 0.0 ? cli.max_travel_deg * kDeg : r2.max_travel_rad;
  const double min_travel = 0.5 * p.prof.speed_rad_s * (p.prof.ramp_s + p.prof.stop_ramp_s);
  const sat::TravelPlan tp = sat::planTravelInWindow(q0[6], p.sign, p.q7_lo, p.q7_hi, m, max_travel, min_travel);
  check("q7 window and travel", tp.ok,
        tp.ok ? fmt("q7_0 %.2f deg -> q7_end %.2f deg, travel %.2f deg", q0[6] / kDeg, tp.q7_end / kDeg, tp.travel / kDeg) +
                    (tp.capped ? " (capped by max_travel)" : " (limited by the window)")
              : tp.why + fmt(" (q7_0 %.2f deg, window with margin [%.2f, %.2f] deg)", q0[6] / kDeg, tp.inner_lo / kDeg, tp.inner_hi / kDeg));
  if (tp.ok) {
    p.prof.travel_rad = tp.travel;
    const double th_end = sat::phaseFromQ7(p.phase, tp.q7_end);
    std::printf("  theta at the nominal end of travel (from q7): %.2f deg\n", th_end / kDeg);
    // Worst position after a window-guard trip at the edge: edge + sign * (v*T2/2 + tracking tolerance).
    const double edge = p.sign > 0 ? p.q7_hi : p.q7_lo;
    const double worst = edge + p.sign * (0.5 * p.prof.speed_rad_s * p.prof.stop_ramp_s + p.max_track_err);
    const double jm = rr.safety.joint_limit_margin_rad;
    check("overshoot after a window-guard trip stays inside the J7 limits", worst >= sat::kQMin[6] + jm && worst <= sat::kQMax[6] - jm,
          fmt("worst q7 %.2f deg, limits with margin [%.2f, %.2f] deg", worst / kDeg, (sat::kQMin[6] + jm) / kDeg, (sat::kQMax[6] - jm) / kDeg));
    try {
      const J7Profile prof(p.prof);
      std::printf("  profile: |omega| %.3f deg/s, sign %+d (q7_ref = q7_0 + sign*d), ramp %.2f s, constant %.2f s, stop ramp %.2f s, hold %.2f s\n",
                  p.prof.speed_rad_s / kDeg, p.sign, p.prof.ramp_s, prof.constantS(), p.prof.stop_ramp_s, p.prof.hold_s);
    } catch (const std::exception& e) {
      check("profile", false, e.what());
    }
  }
  {
    bool ok = true;
    std::string det;
    for (std::size_t k = 0; k < 7; ++k) {
      if (q0[k] < sat::kQMin[k] + rr.safety.joint_limit_margin_rad || q0[k] > sat::kQMax[k] - rr.safety.joint_limit_margin_rad) {
        ok = false;
        det += "joint" + std::to_string(k + 1) + " ";
      }
    }
    check("all joints within limits (SafetyGuard margin)", ok, det.empty() ? "ok" : det);
  }
  std::printf("  heartbeat timeout %.2f s (armed by the first heartbeat or START), START wait max %.0f s%s\n", p.hb_timeout_s, p.max_wait_s,
              p.autostart ? ", AUTOSTART (dry-run)" : "");
  return failures;
}

void printSummary(const Controller& c, const Params& p, const Comms& comms) {
  const std::vector<Rec>& r = c.records();
  std::printf("\n=== SUMMARY ===\n");
  std::printf("stop reason: %s%s%s\n", toString(c.stopReason()), c.stopMessage().empty() ? "" : "  -- ", c.stopMessage().c_str());
  std::printf("faults (bits): 0x%x   cycles %ld   datagrams sent %llu (send errors %llu)   bad command datagrams %u\n", c.faults(), c.cycles(),
              static_cast<unsigned long long>(comms.sent.load()), static_cast<unsigned long long>(comms.send_errors.load()), comms.bad_datagrams.load());
  if (r.empty()) return;
  std::vector<double> om;
  for (const Rec& x : r) {
    if (x.omega_valid != 0.0) om.push_back(x.omega_meas);  // same rule as the datagram flag omega_valid
  }
  if (om.empty()) {
    std::printf("steady omega: n/a (omega_valid never true: constant-speed phase shorter than the %.1f s window, or not reached)\n",
                p.omega_window_s);
  } else {
    double mean = 0.0;
    for (double x : om) mean += x;
    mean /= static_cast<double>(om.size());
    std::printf("steady omega_meas (omega_valid, LS over %.1f s): mean %.4f deg/s over %zu samples, commanded %.4f deg/s (rel. error %.3f %%)\n",
                p.omega_window_s, mean, om.size(), p.omega_cmd_nominal / kDeg, 100.0 * (mean - p.omega_cmd_nominal / kDeg) / (p.omega_cmd_nominal / kDeg));
  }
  std::printf("theta: start %.2f deg, end %.2f deg (from flange)   max |theta_flange - theta_q7| %.4f deg\n", r.front().theta_flange,
              r.back().theta_flange, c.maxMismatch() / kDeg);
  std::printf("max |e_q7| %.5f rad   max |error| held joints %.5f rad   max callback %.1f us\n", c.maxE7(), c.maxHeldErr(), c.maxCbUs());
}

void writeMeta(const std::string& csv, const Cli& cli, const real_replay::HwConfig& hw, const Params& p, const Controller& c, const char* kind) {
  try {
    const std::string meta = csv + ".meta.txt";
    std::FILE* f = std::fopen(meta.c_str(), "wx");
    if (!f) throw std::runtime_error("cannot create '" + meta + "'");
    std::fprintf(f, "csv: %s\nmode: %s\nargv: %s\nhw_config: %s\nhw_config_sha256: %s\n", csv.c_str(), kind, cli.argv_line.c_str(), hw.path.c_str(),
                 real_replay::sha256FileHex(hw.path).c_str());
    std::fprintf(f, "robot2_ip: %s\nomega_deg_s: %.9g\nsign: %d\ntravel_deg: %.9g\nramp_s: %.9g\nstop_ramp_s: %.9g\nhold_s: %.9g\n",
                 cli.dry_run ? "(dry-run, none)" : hw.robot2.ip.c_str(), p.omega_cmd_nominal / kDeg, p.sign, p.prof.travel_rad / kDeg, p.prof.ramp_s,
                 p.prof.stop_ramp_s, p.prof.hold_s);
    std::fprintf(f, "collar_phase_offset_deg: %.9g\nq7_phase_offset_deg: %.9g\nq7_phase_sign: %d\nbase2_t_m: %.9g %.9g %.9g\nbase2_yaw_deg: %.9g\n",
                 hw.scene.collar_phase_offset_rad / kDeg, hw.scene.q7_phase_offset_rad / kDeg, hw.scene.q7_phase_sign, hw.scene.base2_t_m[0],
                 hw.scene.base2_t_m[1], hw.scene.base2_t_m[2], hw.scene.base2_yaw_rad / kDeg);
    std::fprintf(f, "mockup_mass_kg: %.9g\nstop_reason: %s\nstop_message: %s\nfaults: 0x%x\n", hw.mockup.mass_kg, toString(c.stopReason()),
                 c.stopMessage().c_str(), c.faults());
    std::fclose(f);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "WARNING: log meta not written: %s\n", e.what());
  }
}

void finishLog(const std::string& csv, const Cli& cli, const real_replay::HwConfig& hw, const Params& p, const Controller& c, const char* kind) {
  const std::string err = writeCsv(csv, c.records());
  if (err.empty()) {
    std::printf("log written: %s (%zu rows)\n", csv.c_str(), c.records().size());
    writeMeta(csv, cli, hw, p, c, kind);
  } else {
    std::fprintf(stderr, "ERROR: log not written: %s\n", err.c_str());
  }
}

// ---- Dry-run ----------------------------------------------------------------------------------------
// Synthetic plant (same as satellite_j7 / friction_id): I*ddq = tau - f(dq); gravity compensated by the robot.
void simStep(Array7& q, Array7& dq, const Array7& tau, double dt) {
  static const double I[7] = {1.2, 1.5, 1.0, 1.0, 0.4, 0.3, 0.15};
  static const double fc[7] = {1.5, 1.5, 1.2, 1.2, 0.8, 0.8, 0.6};
  static const double bv[7] = {2.0, 2.0, 1.5, 1.5, 0.8, 0.8, 0.5};
  for (std::size_t k = 0; k < 7; ++k) {
    const double f = fc[k] * std::tanh(dq[k] / 2e-3) + bv[k] * dq[k];
    dq[k] += (tau[k] - f) / I[k] * dt;
    q[k] += dq[k] * dt;
  }
}

int runDry(const Cli& cli, const real_replay::HwConfig& hw, const real_replay::Config& rr, Comms& comms) {
  std::printf("=== DRY RUN (synthetic plant, no robot connection, paced at 1 kHz) ===\n");
  Array7 q = cli.dry_q_given ? cli.dry_q : hw.robot2.start_q_rad;
  if (!cli.dry_q_given && hw.robot2.start_mode != "q") {
    std::printf("note: robot2.start.mode = %s: the dry-run starts from robot2.start.q_deg (pass --dry-q-deg for the IK pose; hw.py run does)\n",
                hw.robot2.start_mode.c_str());
  }
  Params p;
  if (makeParams(cli, hw, rr, q, p) > 0) {
    std::printf("preflight failed -> exit\n");
    return 1;
  }
  Controller c(rr.safety, p, q, comms);
  const std::string csv = logPath(cli.log_dir.empty() ? hw.log_dir : cli.log_dir, cli.log_tag, "dryrun");
  comms.start();
  std::printf("%s\n", p.autostart ? "AUTOSTART: rotating at once" : "WAIT_START: send START to the command port");
  std::fflush(stdout);

  std::mt19937 rng(12345);
  std::uniform_real_distribution<double> jit(-hw.robot2.dry_period_jitter_s, hw.robot2.dry_period_jitter_s);
  const int delay = hw.robot2.dry_meas_delay_cycles;
  std::vector<Array7> qh(static_cast<std::size_t>(delay + 1), q), dqh(static_cast<std::size_t>(delay + 1), Array7{});
  Array7 dq{}, tau{};
  StepIn in{};
  auto next = std::chrono::steady_clock::now();
  for (long i = 0;; ++i) {
    const double period = i == 0 ? 0.0 : kDt + jit(rng);
    const std::size_t idx = static_cast<std::size_t>(i % (delay + 1));  // oldest sample of the delay line
    in.q = qh[idx];
    in.dq = dqh[idx];
    in.period_s = period;
    const auto t0 = std::chrono::steady_clock::now();
    const bool go = c.step(in, tau);
    c.setLastCallbackUs(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count());
    if (!go) break;
    simStep(q, dq, tau, i == 0 ? kDt : period);
    qh[idx] = q;
    dqh[idx] = dq;
    next += std::chrono::nanoseconds(static_cast<long long>((i == 0 ? kDt : period) * 1e9));
    std::this_thread::sleep_until(next);
  }
  c.publishFinal();
  comms.stop();
  finishLog(csv, cli, hw, p, c, "dryrun");
  printSummary(c, p, comms);
  return exitCodeFor(c);
}

// ---- Robot run --------------------------------------------------------------------------------------
Eigen::Matrix4d colMajor(const std::array<double, 16>& a) {
  Eigen::Matrix4d m;
  for (int c = 0; c < 4; ++c)
    for (int r = 0; r < 4; ++r) m(r, c) = a[static_cast<std::size_t>(c * 4 + r)];
  return m;
}

int runRobot(const Cli& cli, const real_replay::HwConfig& hw, const real_replay::Config& rr, Comms& comms) {
  const std::string ip = hw.robot2.ip;
  std::printf("=== ROBOT RUN: robot 2 at %s ===\n", ip.c_str());
  franka::Robot* rp = nullptr;
  std::unique_ptr<Controller> ctl;
  Params p;
  std::string csv;
  int exit_code = 0;
  bool control_started = false;
  try {
    franka::Robot robot(ip, franka::RealtimeConfig::kIgnore);
    rp = &robot;
    robot.setCollisionBehavior(rr.collision.lower_torque_acc, rr.collision.upper_torque_acc, rr.collision.lower_torque_nom,
                               rr.collision.upper_torque_nom, rr.collision.lower_force_acc, rr.collision.upper_force_acc,
                               rr.collision.lower_force_nom, rr.collision.upper_force_nom);
    robot.setJointImpedance(rr.collision.joint_impedance);
    robot.setCartesianImpedance(rr.collision.cartesian_impedance);
    if (rr.end_effector.set_ee) robot.setEE(rr.end_effector.ne_t_ee);
    // The mock-up is a PAYLOAD (m_load), never an end effector (m_ee: Desk must have no hand on robot 2).
    const std::array<double, 9> zero_inertia{};
    robot.setLoad(hw.mockup.mass_kg, hw.mockup.com_m, zero_inertia);
    std::printf("setLoad: %.3f kg, com [%.4f %.4f %.4f] m (flange frame), zero inertia\n", hw.mockup.mass_kg, hw.mockup.com_m[0], hw.mockup.com_m[1],
                hw.mockup.com_m[2]);

    const franka::RobotState s0 = robot.readOnce();
    int failures = 0;
    auto check = [&](const char* name, bool ok, const std::string& detail) {
      std::printf("  [%s] %s: %s\n", ok ? "PASS" : "FAIL", name, detail.c_str());
      if (!ok) ++failures;
    };
    std::printf("preflight (robot):\n");
    check("robot_mode == Idle", s0.robot_mode == franka::RobotMode::kIdle, "robot_mode=" + std::to_string(static_cast<int>(s0.robot_mode)));
    check("m_ee <= robot2.m_ee_max_kg (no hand in Desk)", s0.m_ee <= hw.robot2.m_ee_max_kg, fmt("m_ee=%.3f kg, limit %.3f", s0.m_ee, hw.robot2.m_ee_max_kg));
    const double dm = std::fabs(s0.m_load - hw.mockup.mass_kg);
    std::printf("  [%s] m_load vs mockup.mass_kg: m_load=%.3f kg, mockup %.3f kg (tol %.3f; WARN only: the state may not reflect setLoad yet)\n",
                dm <= hw.robot2.m_load_tol_kg ? "PASS" : "WARN", s0.m_load, hw.mockup.mass_kg, hw.robot2.m_load_tol_kg);
    Array7 q0{};
    for (std::size_t k = 0; k < 7; ++k) q0[k] = s0.q[k];
    {  // DH flange vs the robot's own O_T_EE * F_T_EE^-1 (frame 2)
      const Eigen::Matrix4d O_T_F = colMajor(s0.O_T_EE) * colMajor(s0.F_T_EE).inverse();
      const Eigen::Isometry3d fk = sat::pandaFlangeFk(q0);
      const double dp = (O_T_F.block<3, 1>(0, 3) - fk.translation()).norm();
      const Eigen::AngleAxisd da(Eigen::Matrix3d(O_T_F.block<3, 3>(0, 0).transpose() * fk.linear()));
      check("DH flange FK vs O_T_EE * F_T_EE^-1", dp <= hw.robot2.tol_fk_m && std::fabs(da.angle()) <= hw.robot2.tol_fk_rad,
            fmt("%.3f mm, %.3f deg (tol %.3f mm, %.3f deg)", dp * 1e3, std::fabs(da.angle()) / kDeg, hw.robot2.tol_fk_m * 1e3, hw.robot2.tol_fk_rad / kDeg));
    }
    if (rr.preflight.check_gravity) {
      franka::Model fm = robot.loadModel();
      const std::array<double, 7> gf = fm.gravity(s0);
      Array7 g{}, tj{}, dqv{};
      for (std::size_t k = 0; k < 7; ++k) {
        g[k] = gf[k];
        tj[k] = s0.tau_J[k];
        dqv[k] = s0.dq[k];
      }
      const auto r = real_replay::checkGravityVsMeasured(g, tj, dqv, rr.preflight.gravity_vs_measured_tol_nm, rr.preflight.gravity_still_max_dq_rad_s);
      std::printf("  [%s] gravity: franka model vs measured tau_J: max diff %.3f Nm on joint %d (tol %.3f; WARN only, as satellite_j7 with a load)\n",
                  r.verdict == real_replay::MeasuredGravityResult::Verdict::kPass ? "PASS" : "WARN", r.max_abs_diff, r.worst_joint + 1,
                  rr.preflight.gravity_vs_measured_tol_nm);
    }
    failures += makeParams(cli, hw, rr, q0, p);
    if (failures > 0) {
      std::printf("preflight: %d failure(s) -> aborting before control\n", failures);
      return 1;
    }
    ctl = std::make_unique<Controller>(rr.safety, p, q0, comms);
    csv = logPath(cli.log_dir.empty() ? hw.log_dir : cli.log_dir, cli.log_tag, "robot");

    std::printf("\n================================================================================\n"
                "  CARICO DICHIARATO: %.3f kg, com [%.4f %.4f %.4f] m (frame flangia) -- il mockup e' montato?\n"
                "================================================================================\n",
                hw.mockup.mass_kg, hw.mockup.com_m[0], hw.mockup.com_m[1], hw.mockup.com_m[2]);
    if (!cli.yes) {
      std::printf("Digita la massa del mockup montato in kg per confermare (deve coincidere con %.3f kg entro 1 g): ", hw.mockup.mass_kg);
      std::fflush(stdout);
      std::string line;
      if (!std::getline(std::cin, line) || g_sig != 0 || !real_replay::massConfirmationOk(line, hw.mockup.mass_kg)) {
        std::printf("\ncarico NON confermato ('%s'): esco senza prendere il controllo.\n", line.c_str());
        return 2;
      }
      std::printf("carico confermato.\n");
    }
    std::printf("\nWARNING: robot 2 will be torque-controlled; after START J7 rotates by up to %.1f deg (sign %+d). Keep the user stop at hand.\n",
                p.prof.travel_rad / kDeg, p.sign);
    if (!cli.yes) {
      std::printf("Press Enter to take control (Ctrl-C to abort)...");
      std::fflush(stdout);
      const int ch = std::cin.get();
      if (g_sig != 0 || ch == EOF) {
        std::printf("\naborted before control (%s).\n", g_sig != 0 ? "signal" : "no confirmation on stdin");
        return 3;
      }
    }
    const int countdown = cli.countdown_s >= 0 ? cli.countdown_s : hw.robot2.countdown_s;
    for (int k = countdown; k >= 1; --k) {
      std::printf("control in %d...\n", k);
      std::fflush(stdout);
      for (int i = 0; i < 10 && g_sig == 0; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (g_sig != 0) {
        std::printf("aborted during the countdown; no control started.\n");
        return 3;
      }
    }
    comms.start();
    Controller& c = *ctl;
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)> cb = [&c](const franka::RobotState& s, franka::Duration d) {
      const auto t0 = std::chrono::steady_clock::now();
      StepIn in{};
      for (std::size_t k = 0; k < 7; ++k) {
        in.q[k] = s.q[k];
        in.dq[k] = s.dq[k];
      }
      in.period_s = d.toSec();
      in.success_rate = s.control_command_success_rate;
      Array7 tau{};
      const bool go = c.step(in, tau);
      franka::Torques t(tau);
      t.motion_finished = !go;
      c.setLastCallbackUs(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count());
      return t;
    };
    control_started = true;
    std::printf("control started: WAIT_START (holding q0), waiting for START on the command port\n");
    std::fflush(stdout);
    robot.control(cb, /*limit_rate=*/true, franka::kMaxCutoffFrequency);
    c.publishFinal();
    std::printf("control loop ended after %ld cycles; robot released.\n", c.cycles());
  } catch (const franka::ControlException& e) {
    std::fprintf(stderr,
                 "\nERROR: franka::ControlException: %s\n"
                 "       The controlled stop ramp was NOT completed by this program. The robot is probably in reflex: check it,\n"
                 "       then run the error recovery or restart from Desk.\n",
                 e.what());
    if (ctl) ctl->markAbnormal(std::string("franka::ControlException: ") + e.what());
    exit_code = 4;
  } catch (const franka::Exception& e) {
    std::fprintf(stderr, "ERROR: franka::Exception: %s\n", e.what());
    if (rp != nullptr && control_started) {
      try {
        rp->stop();
      } catch (...) {
      }
    }
    if (ctl) ctl->markAbnormal(std::string("franka::Exception: ") + e.what());
    exit_code = control_started ? 4 : 1;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "ERROR: %s\n", e.what());
    if (ctl) ctl->markAbnormal(std::string("exception: ") + e.what());
    exit_code = control_started ? 4 : 1;
  }
  comms.stop();
  if (ctl && control_started) {
    if (!csv.empty()) finishLog(csv, cli, hw, p, *ctl, "robot");
    printSummary(*ctl, p, comms);
    if (exit_code == 0) exit_code = exitCodeFor(*ctl);
  }
  return exit_code;
}

}  // namespace

int main(int argc, char** argv) {
  Cli cli;
  const std::string perr = parseCli(argc, argv, cli);
  if (perr == "help") {
    usage(argv[0]);
    return 0;
  }
  if (!perr.empty()) {
    std::fprintf(stderr, "error: %s\n", perr.c_str());
    usage(argv[0]);
    return 2;
  }
  real_replay::HwConfig hw;
  real_replay::Config rr;
  try {
    hw = real_replay::loadHwConfig(cli.config);
    rr = real_replay::loadConfig(hw.robot2.real_replay_config);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 2;
  }
  if (rr.robot.ip != hw.robot2.ip) {
    std::printf("note: %s has robot.ip %s; satellite_node uses robot2.ip %s from %s (only safety/collision/set_ee/gravity tolerances are taken "
                "from that file)\n",
                hw.robot2.real_replay_config.c_str(), rr.robot.ip.c_str(), hw.robot2.ip.c_str(), cli.config.c_str());
  }
  const int pub = cli.pub_port > 0 ? cli.pub_port : hw.udp.publish_port;
  const int mon = cli.mon_port >= 0 ? cli.mon_port : hw.udp.monitor_port;
  const int cmd = cli.cmd_port > 0 ? cli.cmd_port : hw.udp.command_port;
  installSignals();
  Comms comms;
  try {
    comms.open(hw.udp.host, pub, mon, cmd, hw.udp.rate_hz);
    std::printf("udp: state -> %s:%d%s at %.0f Hz, commands <- %s:%d\n", hw.udp.host.c_str(), pub,
                mon > 0 ? (" and :" + std::to_string(mon)).c_str() : "", hw.udp.rate_hz, hw.udp.host.c_str(), cmd);
    return cli.dry_run ? runDry(cli, hw, rr, comms) : runRobot(cli, hw, rr, comms);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
