// Satellite mock-up driver: rotate ONLY joint J7 of one Franka at a constant, imposed speed, with smooth start and
// stop ramps. The other six joints are held at the values read at the start. libfranka torque control at 1 kHz,
// same scheme as tools/friction_id.cpp (joint PD with a ramp reference, SafetyGuard on every sent torque).
// Does not use ReplayLoop.
//
// usage: ./satellite_j7 <robot-ip> <config.yaml> --travel-deg D [--speed-deg-s 2] [--sign +1|-1]
//                       [--ramp-s 2] [--stop-ramp-s 2] [--hold-s 1] [--margin-deg 10] [--countdown-s 3]
//                       [--phase-map -1|+1] [--theta0-deg 0] [--omega-window-s 0.2] [--max-track-err-rad 0.05]
//                       [--load-kg M [--load-com-m x,y,z]] [--kp k1..k7] [--kd d1..d7]
//                       [--log-dir DIR] [--log-tag TAG] [--dry-run]
//
// --sign      = direction of the COMMAND of J7:   q7_ref = q7_0 + sign * d(t).
// --phase-map = physical map J7 -> satellite phase: theta = theta0 + phase_map * (q7 - q7_0); omega_meas = d(theta)/dt.
//               Project convention: omega < 0 = clockwise seen from above. The default (-1) is only a hypothesis:
//               to be verified physically.
//
// Reference (include/real_replay/j7_profile.hpp): q7_ref = q7_0 + sign * d(t), d(t) from a smoothstep velocity ramp
// (T1 = ramp-s), a constant-speed phase and a smoothstep stop ramp (T2 = stop-ramp-s); total rotation = travel-deg exactly.
// Ctrl-C, a SafetyGuard trigger, a tracking error or a J7 limit trigger ALWAYS start a controlled stop ramp of T2 seconds
// from the current reference velocity (never an abrupt stop). Only a non-finite state (no valid reference possible)
// ends the control with zero torque immediately.
//
// Exit codes: 0 completed normally; 1 runtime/preflight failure; 2 refused arguments;
//             3 stopped before completion (Ctrl-C / safety) after a controlled stop ramp;
//             4 the stop ramp could NOT be completed (libfranka error, non-finite state): the robot may be in reflex.

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/lowpass_filter.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <signal.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "real_replay/git_version.hpp"
#include "real_replay/config.hpp"
#include "real_replay/j7_profile.hpp"
#include "real_replay/preflight_checks.hpp"
#include "real_replay/safety.hpp"
#include "real_replay/sha256.hpp"

using real_replay::Array7;
using real_replay::J7Profile;
using real_replay::J7ProfileParams;
using real_replay::J7Ref;

namespace {

constexpr double kDeg = M_PI / 180.0;
constexpr double kDt = 1e-3;

// ---- Defaults and hard limits -----------------------------------------------------------------------
const Array7 kDefaultKp = {300.0, 300.0, 300.0, 300.0, 150.0, 100.0, 40.0};  // [Nm/rad]  (same as friction_id)
const Array7 kDefaultKd = {15.0, 15.0, 15.0, 15.0, 8.0, 6.0, 3.0};            // [Nms/rad] (same as friction_id)
constexpr double kMaxSpeedDegS = 5.0;     // hard maximum of --speed-deg-s
constexpr double kDefaultMaxTrackErr = 0.05;  // [rad] default of --max-track-err-rad (J7 and held joints)
constexpr double kMaxPayloadKg = 3.0;         // hard maximum of --load-kg (Franka FR3 rated payload)
constexpr double kMaxRampS = 60.0;        // sanity bound on --ramp-s, --stop-ramp-s, --hold-s
constexpr int kMaxCountdownS = 30;

// FER joint limits (franka-research-3 datasheet) [rad]; same constants as friction_id.
const Array7 kQMin = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
const Array7 kQMax = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};

volatile std::sig_atomic_t g_sig = 0;
void onSignal(int) { g_sig = 1; }

void installSignals() {
  // No SA_RESTART: a blocking read of the confirmation (std::cin) must be interrupted by Ctrl-C.
  struct sigaction sa {};
  sa.sa_handler = onSignal;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);
}

// ---- CLI --------------------------------------------------------------------------------------------
struct Cli {
  std::string robot_ip;
  std::string config;
  double speed_deg_s = 2.0;
  int sign = +1;
  int phase_map = -1;  // hypothesis, to be verified physically
  double max_track_err_rad = kDefaultMaxTrackErr;
  double load_kg = std::nan("");  // NaN = --load-kg absent: the load already configured on the robot is not touched
  std::array<double, 3> load_com = {0.0, 0.0, 0.05};
  double travel_deg = std::nan("");  // required
  double ramp_s = 2.0;
  double stop_ramp_s = 2.0;
  double hold_s = 1.0;
  double margin_deg = 10.0;
  int countdown_s = 3;
  double theta0_deg = 0.0;
  double omega_window_s = 0.2;
  Array7 kp = kDefaultKp;
  Array7 kd = kDefaultKd;
  std::string log_dir;
  std::string log_tag;
  bool dry_run = false;
  std::string argv_line;
};

void usage(const char* p) {
  std::fprintf(stderr,
               "usage: %s <robot-ip> <config.yaml> --travel-deg D [--speed-deg-s 2] [--sign +1|-1] [--ramp-s 2]\n"
               "          [--stop-ramp-s 2] [--hold-s 1] [--margin-deg 10] [--countdown-s 3] [--phase-map -1|+1] [--theta0-deg 0]\n"
               "          [--omega-window-s 0.2] [--max-track-err-rad 0.05] [--load-kg M [--load-com-m x,y,z]] [--kp k1,..,k7] [--kd d1,..,d7] [--log-dir DIR] [--log-tag TAG] [--dry-run]\n"
               "limits: speed in (0, %.1f] deg/s, ramps in (0, %.0f] s, hold in [0, %.0f] s, margin >= 0, countdown integer in [0, %d]\n",
               p, kMaxSpeedDegS, kMaxRampS, kMaxRampS, kMaxCountdownS);
}

bool parseNumber(const std::string& s, double& out) {
  char* end = nullptr;
  out = std::strtod(s.c_str(), &end);
  return !s.empty() && end != s.c_str() && *end == '\0' && std::isfinite(out);
}

bool parseVec3(const std::string& s, std::array<double, 3>& out) {
  std::stringstream ss(s);
  std::string tok;
  std::size_t n = 0;
  std::array<double, 3> tmp{};
  while (std::getline(ss, tok, ',')) {
    if (n >= 3) return false;
    if (!parseNumber(tok, tmp[n++])) return false;
  }
  if (n != 3) return false;
  out = tmp;
  return true;
}

bool parseList7(const std::string& s, Array7& out) {
  std::stringstream ss(s);
  std::string tok;
  std::size_t n = 0;
  Array7 tmp{};
  while (std::getline(ss, tok, ',')) {
    if (n >= 7) return false;
    double v = 0.0;
    if (!parseNumber(tok, v) || v < 0.0 || v > 1000.0) return false;
    tmp[n++] = v;
  }
  if (n != 7) return false;
  out = tmp;
  return true;
}

/// Returns "" if ok, otherwise the reason.
std::string parseCli(int argc, char** argv, Cli& cli) {
  std::vector<std::string> positional;
  for (int i = 1; i < argc; ++i) {
    if (i > 1) cli.argv_line += ' ';
    cli.argv_line += argv[i];
  }
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a.rfind("--", 0) != 0) {
      positional.push_back(a);
      continue;
    }
    if (a == "--dry-run") {
      cli.dry_run = true;
      continue;
    }
    if (i + 1 >= argc) return a + " needs a value";
    const std::string v = argv[++i];
    double d = 0.0;
    auto num = [&](double& dst) {
      if (!parseNumber(v, d)) return false;
      dst = d;
      return true;
    };
    bool ok = true;
    if (a == "--speed-deg-s") {
      ok = num(cli.speed_deg_s);
    } else if (a == "--travel-deg") {
      ok = num(cli.travel_deg);
    } else if (a == "--ramp-s") {
      ok = num(cli.ramp_s);
    } else if (a == "--stop-ramp-s") {
      ok = num(cli.stop_ramp_s);
    } else if (a == "--hold-s") {
      ok = num(cli.hold_s);
    } else if (a == "--margin-deg") {
      ok = num(cli.margin_deg);
    } else if (a == "--theta0-deg") {
      ok = num(cli.theta0_deg);
    } else if (a == "--omega-window-s") {
      ok = num(cli.omega_window_s);
    } else if (a == "--sign") {
      ok = parseNumber(v, d) && (d == 1.0 || d == -1.0);
      if (ok) cli.sign = static_cast<int>(d);
    } else if (a == "--phase-map") {
      ok = parseNumber(v, d) && (d == 1.0 || d == -1.0);
      if (ok) cli.phase_map = static_cast<int>(d);
    } else if (a == "--max-track-err-rad") {
      ok = num(cli.max_track_err_rad);
    } else if (a == "--load-kg") {
      ok = num(cli.load_kg);
    } else if (a == "--load-com-m") {
      ok = parseVec3(v, cli.load_com);
    } else if (a == "--countdown-s") {
      ok = parseNumber(v, d) && d == std::floor(d) && d >= 0.0 && d <= kMaxCountdownS;
      if (ok) cli.countdown_s = static_cast<int>(d);
    } else if (a == "--kp") {
      ok = parseList7(v, cli.kp);
    } else if (a == "--kd") {
      ok = parseList7(v, cli.kd);
    } else if (a == "--log-dir") {
      cli.log_dir = v;
    } else if (a == "--log-tag") {
      cli.log_tag = v;
    } else {
      return "unknown argument '" + a + "'";
    }
    if (!ok) return "invalid value '" + v + "' for " + a;
  }
  if (positional.size() != 2) return "expected exactly two positional arguments: <robot-ip> <config.yaml>";
  cli.robot_ip = positional[0];
  cli.config = positional[1];
  if (!std::isfinite(cli.travel_deg)) return "--travel-deg is required";
  if (!(cli.travel_deg > 0.0)) return "--travel-deg must be > 0";
  if (!(cli.speed_deg_s > 0.0) || cli.speed_deg_s > kMaxSpeedDegS) return "--speed-deg-s must be in (0, 5]";
  if (!(cli.ramp_s > 0.0) || cli.ramp_s > kMaxRampS) return "--ramp-s must be in (0, 60]";
  if (!(cli.stop_ramp_s > 0.0) || cli.stop_ramp_s > kMaxRampS) return "--stop-ramp-s must be in (0, 60]";
  if (!(cli.hold_s >= 0.0) || cli.hold_s > kMaxRampS) return "--hold-s must be in [0, 60]";
  if (!(cli.margin_deg >= 0.0) || cli.margin_deg > 90.0) return "--margin-deg must be in [0, 90]";
  if (!(cli.max_track_err_rad > 0.0) || cli.max_track_err_rad > 1.0) return "--max-track-err-rad must be in (0, 1]";
  if (!std::isnan(cli.load_kg) && (!(cli.load_kg >= 0.0) || cli.load_kg > kMaxPayloadKg)) return "--load-kg must be in [0, 3]";
  if (!(cli.omega_window_s >= 0.01) || cli.omega_window_s > 5.0) return "--omega-window-s must be in [0.01, 5]";
  return "";
}

// ---- Helpers ----------------------------------------------------------------------------------------
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

std::string sanitizeTag(const std::string& s) {
  std::string out;
  for (char c : s) out += (std::isalnum(static_cast<unsigned char>(c)) || c == '-' || c == '_') ? c : '_';
  return out;
}

std::string fmt(const char* f, double a = 0.0, double b = 0.0, double c = 0.0, double d = 0.0) {
  char buf[400];
  std::snprintf(buf, sizeof buf, f, a, b, c, d);
  return buf;
}

// ---- Log --------------------------------------------------------------------------------------------
struct Rec {
  double t, period, cb_us;
  double q[7], dq[7], tau_cmd[7];
  double q7_ref, dq7_ref, e_q7, theta, omega_meas;
  // appended at the end (existing columns unchanged); NaN in the dry-run (no robot state)
  double tau_J[7];     ///< state.tau_J: measured joint torques
  double tau_ext[7];   ///< state.tau_ext_hat_filtered
  double F_ext_K[6];   ///< state.O_F_ext_hat_K
};
static_assert(sizeof(Rec) % sizeof(double) == 0, "Rec must be a flat array of doubles");
constexpr std::size_t kRecFields = sizeof(Rec) / sizeof(double);

std::string csvHeader() {
  std::string h = "t,period,cb_us";
  for (const char* g : {"q", "dq", "tau_cmd"})
    for (int k = 1; k <= 7; ++k) h += "," + std::string(g) + std::to_string(k);
  h += ",q7_ref,dq7_ref,e_q7,theta,omega_meas";
  for (int k = 1; k <= 7; ++k) h += ",tau_J" + std::to_string(k);
  for (int k = 1; k <= 7; ++k) h += ",tau_ext" + std::to_string(k);
  for (int k = 1; k <= 6; ++k) h += ",F_ext_K" + std::to_string(k);
  return h;
}

/// Exclusive open ("wx"): never overwrites. Returns "" on success, otherwise the error.
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

// ---- Controller (robot-free; used by the robot callback and by the dry-run) --------------------------
struct StepIn {
  Array7 q{};
  Array7 dq{};
  double period_s = 0.0;     ///< libfranka period of this cycle (0 on the very first call)
  double success_rate = 1.0;
  // logged only (never used by the control law); NaN when there is no robot (dry-run)
  Array7 tau_J = {std::nan(""), std::nan(""), std::nan(""), std::nan(""), std::nan(""), std::nan(""), std::nan("")};
  Array7 tau_ext = {std::nan(""), std::nan(""), std::nan(""), std::nan(""), std::nan(""), std::nan(""), std::nan("")};
  std::array<double, 6> F_ext_K = {std::nan(""), std::nan(""), std::nan(""), std::nan(""), std::nan(""), std::nan("")};
};

enum class Stop { kNone, kCompleted, kUserSignal, kSafety, kTrackingError, kJ7Limit, kLogFull, kNonFinite };

const char* toString(Stop s) {
  switch (s) {
    case Stop::kNone: return "none";
    case Stop::kCompleted: return "completed";
    case Stop::kUserSignal: return "user_signal";
    case Stop::kSafety: return "safety_guard";
    case Stop::kTrackingError: return "tracking_error";
    case Stop::kJ7Limit: return "j7_limit";
    case Stop::kLogFull: return "log_full";
    case Stop::kNonFinite: return "non_finite_state";
  }
  return "unknown";
}

struct Params {
  int sign = +1;
  int phase_map = -1;
  double max_track_err = kDefaultMaxTrackErr;
  std::string load_note;  ///< what is known about the mock-up load (printed in the plan, summary and meta)
  J7ProfileParams prof;
  Array7 kp = kDefaultKp;
  Array7 kd = kDefaultKd;
  double theta0_rad = 0.0;
  double omega_window_s = 0.2;
  double q7_lo = 0.0;  ///< lower limit + margin [rad]
  double q7_hi = 0.0;  ///< upper limit - margin [rad]
};

class Controller {
 public:
  Controller(const real_replay::Config& cfg, const Params& p, const Array7& q0)
      : p_(p),
        prof_(p.prof),
        guard_(cfg.safety, kQMin, kQMax),
        q0_(q0),
        slope_(p.omega_window_s, static_cast<std::size_t>(std::ceil(p.omega_window_s / 0.0005)) + 8) {
    guard_.reset();
    cap_ = static_cast<std::size_t>((prof_.totalDurationS() + p.prof.stop_ramp_s + 5.0) * 1100.0) + 1000;
    rec_.reserve(cap_);
  }

  /// One 1 kHz cycle. Returns true while the control must go on; false = motion_finished (the torque in `tau`
  /// is still sent in that same cycle, except after a non-finite state, where it is zero).
  bool step(const StepIn& in, Array7& tau) {
    tau.fill(0.0);
    if (phase_ == Phase::kDone) return false;
    for (std::size_t k = 0; k < 7; ++k) {
      if (!std::isfinite(in.q[k]) || !std::isfinite(in.dq[k])) return fatal(Stop::kNonFinite, "non-finite q/dq");
    }
    const double dt = (std::isfinite(in.period_s) && in.period_s >= 0.0) ? in.period_s : 0.0;
    if (cycle_ > 0) t_ += dt;
    max_period_ = std::max(max_period_, in.period_s);

    // ---- supervision (only while running the nominal profile): every trigger starts the stop ramp ----
    const J7Ref nom = prof_.sample(t_);
    if (phase_ == Phase::kRun) {
      Stop trig = Stop::kNone;
      std::string why;
      if (g_sig != 0) {
        trig = Stop::kUserSignal;
        why = "Ctrl-C / SIGTERM";
      }
      if (trig == Stop::kNone) {
        const real_replay::StopReason r = guard_.checkCycle(in.period_s, last_cb_us_, in.success_rate, static_cast<long>(cycle_));
        if (r != real_replay::StopReason::kNone) {
          trig = Stop::kSafety;
          why = std::string("SafetyGuard: ") + real_replay::toString(r);
        }
      }
      if (trig == Stop::kNone) {
        const real_replay::StopReason r = guard_.checkState(in.q, in.dq, 0.0, 0.0);  // tracking is checked below
        if (r != real_replay::StopReason::kNone) {
          trig = Stop::kSafety;
          why = std::string("SafetyGuard: ") + real_replay::toString(r);
        }
      }
      if (trig == Stop::kNone) {
        for (std::size_t k = 0; k < 6; ++k) {
          if (std::fabs(q0_[k] - in.q[k]) > p_.max_track_err) {
            trig = Stop::kTrackingError;
            why = "held joint " + std::to_string(k + 1) + " deviates more than " + fmt("%.3f", p_.max_track_err) + " rad from its start value";
          }
        }
        const double e7 = (q0_[6] + p_.sign * nom.d) - in.q[6];
        if (trig == Stop::kNone && std::fabs(e7) > p_.max_track_err) {
          trig = Stop::kTrackingError;
          why = "J7 tracking error " + fmt("%.4f", e7) + " rad exceeds " + fmt("%.3f", p_.max_track_err);
        }
      }
      if (trig == Stop::kNone && (in.q[6] < p_.q7_lo || in.q[6] > p_.q7_hi)) {
        trig = Stop::kJ7Limit;
        why = "q7 = " + fmt("%.4f", in.q[6]) + " rad outside [" + fmt("%.4f", p_.q7_lo) + ", " + fmt("%.4f", p_.q7_hi) + "]";
      }
      if (trig == Stop::kNone && rec_.size() + 2 >= cap_) {
        trig = Stop::kLogFull;
        why = "log buffer full";
      }
      if (trig != Stop::kNone) {
        phase_ = Phase::kStopping;
        stop_ = trig;
        stop_msg_ = why;
        d_stop_ = nom.d;
        v_stop_ = nom.v;
        t_stop_ = t_;
        hold_len_ = 0.0;  // aborted run: no final hold, finish right after the ramp
      }
    }

    // ---- reference of this cycle ----
    J7Ref ref;
    bool finish = false;
    if (phase_ == Phase::kRun) {
      ref = nom;
      if (t_ >= prof_.motionDurationS()) {  // normal end of the motion: hold
        phase_ = Phase::kHold;
        stop_ = Stop::kCompleted;
        d_hold_ = prof_.params().travel_rad;
        t_hold_ = t_;
        hold_len_ = prof_.params().hold_s;
        ref = {d_hold_, 0.0};
      }
    } else if (phase_ == Phase::kStopping) {
      const double tau_s = t_ - t_stop_;
      ref = real_replay::stopFrom(d_stop_, v_stop_, prof_.params().stop_ramp_s, tau_s);
      if (tau_s >= prof_.params().stop_ramp_s) {
        phase_ = Phase::kHold;
        d_hold_ = ref.d;
        t_hold_ = t_;
      }
    } else {
      ref = {d_hold_, 0.0};
    }
    // Final hold: done after hold_s, or at once on Ctrl-C (the motion is already at rest).
    if (phase_ == Phase::kHold && (t_ - t_hold_ >= hold_len_ || g_sig != 0)) finish = true;

    // ---- joint PD: J1-J6 hold q0, J7 follows the reference ----
    Array7 des{};
    for (std::size_t k = 0; k < 6; ++k) {
      const double e = q0_[k] - in.q[k];
      max_held_err_ = std::max(max_held_err_, std::fabs(e));
      des[k] = p_.kp[k] * e + p_.kd[k] * (0.0 - in.dq[k]);
    }
    const double q7_ref = q0_[6] + p_.sign * ref.d;
    const double dq7_ref = p_.sign * ref.v;
    const double e7 = q7_ref - in.q[6];
    des[6] = p_.kp[6] * e7 + p_.kd[6] * (dq7_ref - in.dq[6]);
    for (std::size_t k = 0; k < 7; ++k) {
      if (!std::isfinite(des[k])) return fatal(Stop::kNonFinite, "non-finite torque demand");
    }
    guard_.limitTorque(des, tau);
    if (guard_.lastRateLimited()) ++n_rate_limited_;
    if (guard_.lastClamped()) ++n_clamped_;

    // ---- phase and omega ----
    const double theta = real_replay::phaseTheta(p_.theta0_rad, p_.phase_map, in.q[6], q0_[6]);
    const double omega = slope_.push(t_, theta);

    if (rec_.size() < cap_) {
      Rec r{};
      r.t = t_;
      r.period = in.period_s;
      r.cb_us = 0.0;
      for (std::size_t k = 0; k < 7; ++k) {
        r.q[k] = in.q[k];
        r.dq[k] = in.dq[k];
        r.tau_cmd[k] = tau[k];
      }
      r.q7_ref = q7_ref;
      r.dq7_ref = dq7_ref;
      r.e_q7 = e7;
      r.theta = theta;
      r.omega_meas = omega;
      for (std::size_t k = 0; k < 7; ++k) {
        r.tau_J[k] = in.tau_J[k];
        r.tau_ext[k] = in.tau_ext[k];
      }
      for (std::size_t k = 0; k < 6; ++k) r.F_ext_K[k] = in.F_ext_K[k];
      rec_.push_back(r);
    }
    ++cycle_;
    if (finish) {
      phase_ = Phase::kDone;
      return false;
    }
    return true;
  }

  /// Duration of the callback that just ran (set by the wrapper after step()).
  void setLastCallbackUs(double us) {
    last_cb_us_ = us;
    if (!rec_.empty()) rec_.back().cb_us = us;
  }

  bool done() const { return phase_ == Phase::kDone; }
  Stop stopReason() const { return stop_; }
  const std::string& stopMessage() const { return stop_msg_; }
  /// True if the controlled stop ramp (or the final hold) was completed, i.e. the run ended in an orderly way.
  bool orderlyEnd() const { return phase_ == Phase::kDone && stop_ != Stop::kNonFinite; }
  const std::vector<Rec>& records() const { return rec_; }
  long cycles() const { return cycle_; }
  double stopTime() const { return t_stop_; }
  double maxHeldErr() const { return max_held_err_; }
  double maxPeriod() const { return max_period_; }
  long rateLimited() const { return n_rate_limited_; }
  long clamped() const { return n_clamped_; }
  const J7Profile& profile() const { return prof_; }
  const Array7& q0() const { return q0_; }
  void markAbnormal(const std::string& msg) {  // exception around the control loop
    stop_ = Stop::kNonFinite;
    stop_msg_ = msg;
    phase_ = Phase::kDone;
  }

 private:
  enum class Phase { kRun, kStopping, kHold, kDone };

  bool fatal(Stop s, const char* msg) {
    stop_ = s;
    stop_msg_ = msg;
    phase_ = Phase::kDone;
    return false;  // zero torque
  }

  Params p_;
  J7Profile prof_;
  real_replay::SafetyGuard guard_;
  Array7 q0_;
  real_replay::SlopeFilter slope_;
  std::vector<Rec> rec_;
  std::size_t cap_ = 0;

  Phase phase_ = Phase::kRun;
  Stop stop_ = Stop::kNone;
  std::string stop_msg_;
  long cycle_ = 0;
  double t_ = 0.0;
  double last_cb_us_ = 0.0;
  double t_stop_ = std::numeric_limits<double>::infinity();
  double d_stop_ = 0.0, v_stop_ = 0.0;
  double d_hold_ = 0.0, t_hold_ = 0.0, hold_len_ = 0.0;
  double max_held_err_ = 0.0;
  double max_period_ = 0.0;
  long n_rate_limited_ = 0, n_clamped_ = 0;
};

// ---- Summary ----------------------------------------------------------------------------------------
struct Stats {
  std::size_t n = 0;
  double mean = 0.0, sd = 0.0, pp = 0.0;
};

Stats stats(const std::vector<double>& v) {
  Stats s;
  s.n = v.size();
  if (v.empty()) return s;
  double sum = 0.0;
  for (double x : v) sum += x;
  s.mean = sum / static_cast<double>(v.size());
  double var = 0.0;
  for (double x : v) var += (x - s.mean) * (x - s.mean);
  s.sd = std::sqrt(var / static_cast<double>(v.size()));  // population standard deviation
  s.pp = *std::max_element(v.begin(), v.end()) - *std::min_element(v.begin(), v.end());
  return s;
}

void printSummary(const Controller& c, const Params& p) {
  const std::vector<Rec>& r = c.records();
  std::printf("\n=== SUMMARY ===\n");
  std::printf("stop reason: %s%s%s\n", toString(c.stopReason()), c.stopMessage().empty() ? "" : "  -- ", c.stopMessage().c_str());
  std::printf("cycles: %ld  logged: %zu  rate-limited: %ld  clamped: %ld\n", c.cycles(), r.size(), c.rateLimited(), c.clamped());
  if (r.empty()) return;
  const J7Profile& prof = c.profile();
  const double rot = p.sign * (r.back().q[6] - r.front().q[6]);
  std::printf("command: sign %+d (q7_ref = q7_0 + sign*d).  phase_map %+d (theta = theta0 + phase_map*(q7 - q7_0)): da verificare fisicamente\n"
              "  convention: omega < 0 = clockwise seen from above\n", p.sign, p.phase_map);
  std::printf("gains used: joint |  kp [Nm/rad] | kd [Nms/rad]\n");
  for (std::size_t k = 0; k < 7; ++k) std::printf("            J%zu   | %10.1f   | %9.2f\n", k + 1, p.kp[k], p.kd[k]);
  std::printf("omega window (--omega-window-s): %.3f s\n", p.omega_window_s);
  std::printf("max tracking error allowed (--max-track-err-rad): %.4f rad (J7 and held joints)\n", p.max_track_err);
  std::printf("predicted static error on J7 = tau_friction / kp: non stimato (no friction estimate for this robot/mock-up is used)\n");
  std::printf("load: %s\n", p.load_note.c_str());
  std::printf("J7 total rotation measured (in the + sign direction): %.4f deg   (planned %.4f deg%s)\n", rot / kDeg,
              prof.params().travel_rad / kDeg, c.stopReason() == Stop::kCompleted ? "" : "; run stopped early");

  // steady (constant-speed) rows: the omega window must lie entirely inside the constant phase
  const double t_lo = prof.constantStartS() + p.omega_window_s;
  const double t_hi = std::min(prof.constantEndS(), c.stopTime());
  std::vector<double> om, raw, e7;
  for (const Rec& x : r) {
    if (x.t >= t_lo && x.t <= t_hi) {
      om.push_back(x.omega_meas);
      raw.push_back(p.phase_map * x.dq[6]);
      e7.push_back(x.e_q7);
    }
  }
  if (om.empty()) {
    std::printf("steady-state omega: n/a (no sample with the %.2f s window inside the constant-speed phase [%.2f, %.2f] s)\n",
                p.omega_window_s, prof.constantStartS(), prof.constantEndS());
  } else {
    const Stats a = stats(om);
    const Stats b = stats(raw);
    std::printf("steady-state window t in [%.3f, %.3f] s, %zu samples, nominal omega %.4f deg/s\n", t_lo, t_hi, a.n,
                prof.params().speed_rad_s / kDeg);
    std::printf("  omega_meas (finite difference of theta over %.2f s): mean %.4f deg/s  std %.5f deg/s  peak-peak %.5f deg/s\n",
                p.omega_window_s, a.mean / kDeg, a.sd / kDeg, a.pp / kDeg);
    const Stats ee = stats(e7);
    std::printf("  mean J7 tracking error e_q7 = q7_ref - q7 (steady window): %.5f deg (%.6f rad);  equivalent torque kp*e = %.4f Nm (kp = %.1f Nm/rad)\n",
                ee.mean / kDeg, ee.mean, p.kp[6] * ee.mean, p.kp[6]);
    std::printf("  omega_raw  (phase_map * dq7 from libfranka):         mean %.4f deg/s  std %.5f deg/s  peak-peak %.5f deg/s\n",
                b.mean / kDeg, b.sd / kDeg, b.pp / kDeg);
  }
  double emax = 0.0, pmax = 0.0, cbmax = 0.0;
  for (const Rec& x : r) {
    emax = std::max(emax, std::fabs(x.e_q7));
    pmax = std::max(pmax, x.period);
    cbmax = std::max(cbmax, x.cb_us);
  }
  std::printf("max |tracking error| J7: %.5f rad (%.4f deg)   max |error| of the held joints J1-J6: %.5f rad\n", emax, emax / kDeg,
              c.maxHeldErr());
  std::printf("max cycle period: %.4f ms   max callback duration: %.1f us\n", pmax * 1e3, cbmax);
}

void writeMeta(const std::string& csv, const Cli& cli, const real_replay::Config& cfg, const Params& p, const Controller& c,
               const char* kind) {
  try {
    const std::string meta = csv + ".meta.txt";
    std::FILE* f = std::fopen(meta.c_str(), "wx");
    if (!f) throw std::runtime_error("cannot create '" + meta + "'");
    std::fprintf(f, "csv: %s\nmode: %s\nargv: %s\nconfig_path: %s\nconfig_sha256: %s\n", csv.c_str(), kind, cli.argv_line.c_str(),
                 cfg.config_path.c_str(), real_replay::sha256FileHex(cfg.config_path).c_str());
    std::fprintf(f, "git_head: %s\ngit_dirty: %s\ngit_diff_sha256: %s\n", RR_GIT_HEAD, RR_GIT_DIRTY, RR_GIT_DIFF_SHA256);
    std::fprintf(f, "robot_ip: %s\nsign: %d\nspeed_deg_s: %.9g\ntravel_deg: %.9g\nramp_s: %.9g\nstop_ramp_s: %.9g\nhold_s: %.9g\n",
                 cli.dry_run ? "(dry-run, none)" : cli.robot_ip.c_str(), p.sign, p.prof.speed_rad_s / kDeg, p.prof.travel_rad / kDeg,
                 p.prof.ramp_s, p.prof.stop_ramp_s, p.prof.hold_s);
    std::fprintf(f, "phase_map: %d   (da verificare fisicamente; convention: omega < 0 = clockwise seen from above)\n", p.phase_map);
    std::fprintf(f, "max_track_err_rad: %.9g\nload: %s\nstatic_error_J7: non stimato\n", p.max_track_err, p.load_note.c_str());
    std::fprintf(f, "theta0_deg: %.9g\nomega_window_s: %.9g\nq7_lo_rad: %.9g\nq7_hi_rad: %.9g\nconstant_phase_s: %.9g\n", p.theta0_rad / kDeg,
                 p.omega_window_s, p.q7_lo, p.q7_hi, c.profile().constantS());
    std::fprintf(f, "kp:");
    for (double k : p.kp) std::fprintf(f, " %.9g", k);
    std::fprintf(f, "\nkd:");
    for (double k : p.kd) std::fprintf(f, " %.9g", k);
    std::fprintf(f, "\nq0:");
    for (double k : c.q0()) std::fprintf(f, " %.9g", k);
    std::fprintf(f, "\nstop_reason: %s\nstop_message: %s\n", toString(c.stopReason()), c.stopMessage().c_str());
    std::fclose(f);
    std::printf("log meta written: %s\n", meta.c_str());
  } catch (const std::exception& e) {
    std::fprintf(stderr, "WARNING: log meta not written: %s\n", e.what());
  }
}

std::string logPath(const Cli& cli, const real_replay::Config& cfg, const char* kind) {
  const std::string dir = cli.log_dir.empty() ? cfg.paths.log_dir : cli.log_dir;
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  std::string name = timestampMs() + "_satellite_j7_";
  if (!cli.log_tag.empty()) name += sanitizeTag(cli.log_tag) + "_";
  name += std::string(kind) + ".csv";
  const std::filesystem::path p = std::filesystem::path(dir) / name;
  if (std::filesystem::exists(p)) throw std::runtime_error("log file already exists, refusing to overwrite: " + p.string());
  return p.string();
}

int exitCodeFor(const Controller& c) {
  if (c.stopReason() == Stop::kCompleted) return 0;
  if (!c.orderlyEnd()) return 4;
  return 3;
}

// ---- Limits / parameters ----------------------------------------------------------------------------
/// Builds the Params (profile + limits) or throws with a clear message. Prints the available interval.
Params makeParams(const Cli& cli, const real_replay::Config& cfg, double q7_0, bool& limits_ok) {
  Params p;
  p.sign = cli.sign;
  p.phase_map = cli.phase_map;
  p.max_track_err = cli.max_track_err_rad;
  p.kp = cli.kp;
  p.kd = cli.kd;
  p.theta0_rad = cli.theta0_deg * kDeg;
  p.omega_window_s = cli.omega_window_s;
  p.prof.speed_rad_s = cli.speed_deg_s * kDeg;
  p.prof.travel_rad = cli.travel_deg * kDeg;
  p.prof.ramp_s = cli.ramp_s;
  p.prof.stop_ramp_s = cli.stop_ramp_s;
  p.prof.hold_s = cli.hold_s;
  const double margin = std::max(cli.margin_deg * kDeg, cfg.safety.joint_limit_margin_rad);
  const real_replay::J7TravelCheck tc =
      real_replay::checkJ7Travel(q7_0, cli.sign, p.prof.travel_rad, kQMin[6], kQMax[6], margin);
  p.q7_lo = tc.lo;
  p.q7_hi = tc.hi;
  std::printf("J7 limits (datasheet) [%.4f, %.4f] rad; margin %.2f deg%s -> available [%.4f, %.4f] rad = [%.2f, %.2f] deg\n",
              kQMin[6], kQMax[6], margin / kDeg,
              margin > cli.margin_deg * kDeg + 1e-12 ? " (raised to the SafetyGuard margin of the config)" : "", tc.lo, tc.hi, tc.lo / kDeg,
              tc.hi / kDeg);
  std::printf("q7 at start = %.4f rad (%.2f deg); max rotation possible: %.2f deg in the + direction, %.2f deg in the - direction\n",
              q7_0, q7_0 / kDeg, std::max(0.0, tc.max_pos) / kDeg, std::max(0.0, tc.max_neg) / kDeg);
  std::printf("requested: sign %+d, travel %.2f deg -> q7 end = %.4f rad (%.2f deg)  [%s]\n", cli.sign, cli.travel_deg, tc.q_end,
              tc.q_end / kDeg, tc.ok ? "PASS" : "FAIL: outside the available interval");
  limits_ok = tc.ok;
  return p;
}

void printPlan(const J7Profile& prof, const Params& p) {
  std::printf("profile: speed %.3f deg/s, ramp %.2f s, constant %.3f s, stop ramp %.2f s, hold %.2f s -> motion %.2f s, total %.2f s\n",
              p.prof.speed_rad_s / kDeg, p.prof.ramp_s, prof.constantS(), p.prof.stop_ramp_s, p.prof.hold_s, prof.motionDurationS(),
              prof.totalDurationS());
  std::printf("joint PD kp=[");
  for (double k : p.kp) std::printf(" %.0f", k);
  std::printf(" ] kd=[");
  for (double k : p.kd) std::printf(" %.1f", k);
  std::printf(" ]; theta0 = %.2f deg, omega window %.2f s\n", p.theta0_rad / kDeg, p.omega_window_s);
  std::printf("sign %+d (command direction of J7); phase_map %+d (J7 -> satellite phase; da verificare fisicamente; omega < 0 = clockwise "
              "seen from above); max tracking error %.4f rad\n", p.sign, p.phase_map, p.max_track_err);
}

// ---- Dry-run ----------------------------------------------------------------------------------------
// Synthetic plant (same as friction_id): I*ddq = tau_cmd - f(dq); the robot compensates gravity itself.
void simStep(Array7& q, Array7& dq, const Array7& tau_cmd) {
  static const double I[7] = {1.2, 1.5, 1.0, 1.0, 0.4, 0.3, 0.15};
  static const double fc[7] = {1.5, 1.5, 1.2, 1.2, 0.8, 0.8, 0.6};
  static const double bv[7] = {2.0, 2.0, 1.5, 1.5, 0.8, 0.8, 0.5};
  for (std::size_t k = 0; k < 7; ++k) {
    const double f = fc[k] * std::tanh(dq[k] / 2e-3) + bv[k] * dq[k];
    dq[k] += (tau_cmd[k] - f) / I[k] * kDt;
    q[k] += dq[k] * kDt;
  }
}

int runDry(const Cli& cli, const real_replay::Config& cfg) {
  std::printf("=== DRY RUN (synthetic plant, no robot connection, no pacing) ===\n");
  Array7 q = cfg.dry_run.q;
  Array7 dq{}, tau{};
  bool limits_ok = false;
  Params p = makeParams(cli, cfg, q[6], limits_ok);
  if (!limits_ok) {
    std::fprintf(stderr, "error: travel refused: J7 would leave the available interval\n");
    return 2;
  }
  p.load_note = std::isnan(cli.load_kg) ? "dry-run: setLoad skipped, no robot connected (no --load-kg given: the robot's configured load would be left untouched)"
                                        : "dry-run: setLoad SKIPPED, no robot connected (requested " + fmt("%.3f", cli.load_kg) + " kg, com [" +
                                              fmt("%.3f", cli.load_com[0]) + " " + fmt("%.3f", cli.load_com[1]) + " " +
                                              fmt("%.3f", cli.load_com[2]) + "] m, zero inertia)";
  std::printf("load: %s\n", p.load_note.c_str());
  Controller c(cfg, p, q);
  printPlan(c.profile(), p);
  std::printf("dry-run: the CSV columns tau_J1..7, tau_ext1..7 and F_ext_K1..6 are written as NaN (no robot state)\n");
  const std::string csv = logPath(cli, cfg, "dryrun");
  const long max_cycles = static_cast<long>((c.profile().totalDurationS() + p.prof.stop_ramp_s + 5.0) / kDt) + 100;
  StepIn in{};
  for (long i = 0; i < max_cycles; ++i) {
    in.q = q;
    in.dq = dq;
    in.period_s = i == 0 ? 0.0 : kDt;
    in.success_rate = 1.0;
    if (!c.step(in, tau)) break;
    simStep(q, dq, tau);
  }
  const std::string err = writeCsv(csv, c.records());
  if (err.empty()) {
    std::printf("log written: %s (%zu rows)\n", csv.c_str(), c.records().size());
    writeMeta(csv, cli, cfg, p, c, "dryrun");
  } else {
    std::fprintf(stderr, "ERROR: log not written: %s\n", err.c_str());
  }
  printSummary(c, p);
  return exitCodeFor(c);
}

// ---- Robot run --------------------------------------------------------------------------------------
int runRobot(const Cli& cli, const real_replay::Config& cfg) {
  const std::string ip = cli.robot_ip;
  std::printf("=== ROBOT RUN: %s ===\n", ip.c_str());
  franka::Robot* rp = nullptr;
  std::unique_ptr<Controller> ctl;
  Params p;
  std::string csv;
  int exit_code = 0;
  bool control_started = false;
  try {
    franka::Robot robot(ip, franka::RealtimeConfig::kIgnore);
    rp = &robot;
    // Settings go BEFORE the control loop (same order as real_replay main.cpp / friction_id).
    robot.setCollisionBehavior(cfg.collision.lower_torque_acc, cfg.collision.upper_torque_acc, cfg.collision.lower_torque_nom,
                               cfg.collision.upper_torque_nom, cfg.collision.lower_force_acc, cfg.collision.upper_force_acc,
                               cfg.collision.lower_force_nom, cfg.collision.upper_force_nom);
    robot.setJointImpedance(cfg.collision.joint_impedance);
    robot.setCartesianImpedance(cfg.collision.cartesian_impedance);
    if (cfg.end_effector.set_ee) robot.setEE(cfg.end_effector.ne_t_ee);
    if (cfg.end_effector.set_load) robot.setLoad(cfg.end_effector.load_mass, cfg.end_effector.f_x_cload, cfg.end_effector.load_inertia);
    const bool load_cli = !std::isnan(cli.load_kg);
    if (load_cli) {
      // Mock-up load given on the command line: overrides the config's setLoad, sent BEFORE the control loop.
      const std::array<double, 9> zero_inertia{};
      robot.setLoad(cli.load_kg, cli.load_com, zero_inertia);
      std::printf("setLoad: mass %.3f kg, center of mass [%.3f %.3f %.3f] m (flange frame), inertia all zeros (sent before control)\n",
                  cli.load_kg, cli.load_com[0], cli.load_com[1], cli.load_com[2]);
    }

    const franka::RobotState s0 = robot.readOnce();
    int failures = 0;
    auto check = [&](const char* name, bool ok, const std::string& detail) {
      std::printf("  [%s] %s: %s\n", ok ? "PASS" : "FAIL", name, detail.c_str());
      if (!ok) ++failures;
    };
    std::printf("preflight:\n");
    check("robot_mode == Idle", s0.robot_mode == franka::RobotMode::kIdle, "robot_mode=" + std::to_string(static_cast<int>(s0.robot_mode)));
    check("m_ee <= limit", s0.m_ee <= cfg.preflight.m_ee_max_kg, fmt("m_ee=%.3f kg, limit %.3f", s0.m_ee, cfg.preflight.m_ee_max_kg));
    // With --load-kg the state may or may not reflect the new payload yet (not verified): the m_load and gravity checks
    // then only warn, they do not block.
    auto checkOrWarn = [&](const char* name, bool ok, const std::string& detail) {
      if (!load_cli || ok) {
        check(name, ok, detail);
      } else {
        std::printf("  [WARN] %s: %s (non-fatal because --load-kg was given)\n", name, detail.c_str());
      }
    };
    checkOrWarn("m_load <= limit", s0.m_load <= cfg.preflight.m_load_max_kg, fmt("m_load=%.3f kg, limit %.3f", s0.m_load, cfg.preflight.m_load_max_kg));
    if (cfg.end_effector.set_ee) {
      double d = 0.0;
      for (std::size_t i = 0; i < 16; ++i) d = std::max(d, std::fabs(s0.NE_T_EE[i] - cfg.end_effector.ne_t_ee[i]));
      check("NE_T_EE in the state == config", d < 1e-9, fmt("max |diff| %.3e", d));
    }
    Array7 q0{};
    for (std::size_t k = 0; k < 7; ++k) q0[k] = s0.q[k];
    {  // every joint inside its limits with the SafetyGuard margin
      bool ok = true;
      std::string det;
      for (std::size_t k = 0; k < 7; ++k) {
        if (q0[k] < kQMin[k] + cfg.safety.joint_limit_margin_rad || q0[k] > kQMax[k] - cfg.safety.joint_limit_margin_rad) {
          ok = false;
          det += "joint" + std::to_string(k + 1) + " q=" + fmt("%.4f", q0[k]) + "; ";
        }
      }
      check("all joints within limits (with the config margin)", ok, det.empty() ? "ok" : det);
    }
    if (cfg.preflight.check_gravity) {
      franka::Model fm = robot.loadModel();
      const std::array<double, 7> gf = fm.gravity(s0);
      Array7 g{}, tj{}, dqv{};
      for (std::size_t k = 0; k < 7; ++k) {
        g[k] = gf[k];
        tj[k] = s0.tau_J[k];
        dqv[k] = s0.dq[k];
      }
      const auto r = real_replay::checkGravityVsMeasured(g, tj, dqv, cfg.preflight.gravity_vs_measured_tol_nm,
                                                         cfg.preflight.gravity_still_max_dq_rad_s);
      checkOrWarn("gravity: franka model vs measured tau_J", r.verdict == real_replay::MeasuredGravityResult::Verdict::kPass,
            fmt("max |franka - tau_J| %.3f Nm on joint%.0f (tol %.3f), max|dq| %.4f rad/s", r.max_abs_diff, r.worst_joint + 1.0,
                cfg.preflight.gravity_vs_measured_tol_nm, r.max_abs_dq));
    }
    std::printf("  info: load seen in the robot state: m_ee=%.3f kg, m_load=%.3f kg, F_x_Cload=[%.3f %.3f %.3f] m, m_total=%.3f kg\n", s0.m_ee,
                s0.m_load, s0.F_x_Cload[0], s0.F_x_Cload[1], s0.F_x_Cload[2], s0.m_total);
    bool limits_ok = false;
    p = makeParams(cli, cfg, q0[6], limits_ok);
    if (load_cli) {
      p.load_note = "setLoad sent: " + fmt("%.3f", cli.load_kg) + " kg, com [" + fmt("%.3f", cli.load_com[0]) + " " + fmt("%.3f", cli.load_com[1]) +
                    " " + fmt("%.3f", cli.load_com[2]) + "] m, zero inertia; state after setLoad: m_load " + fmt("%.3f", s0.m_load) +
                    " kg (reflected in the state: not verified)";
    } else {
      p.load_note = "no --load-kg: load already configured on the robot left as is (m_load " + fmt("%.3f", s0.m_load) + " kg, m_ee " +
                    fmt("%.3f", s0.m_ee) + " kg, com [" + fmt("%.3f", s0.F_x_Cload[0]) + " " + fmt("%.3f", s0.F_x_Cload[1]) + " " +
                    fmt("%.3f", s0.F_x_Cload[2]) + "] m)" + (cfg.end_effector.set_load ? "; the config's setLoad was applied" : "");
    }
    std::printf("load: %s\n", p.load_note.c_str());
    check("J7 travel inside the available interval", limits_ok, "see above");
    if (failures > 0) {
      std::printf("preflight: %d failure(s) -> aborting before control\n", failures);
      return 1;
    }
    ctl = std::make_unique<Controller>(cfg, p, q0);
    printPlan(ctl->profile(), p);
    csv = logPath(cli, cfg, "robot");

    std::printf("\nWARNING: the robot will be torque-controlled and J7 will rotate by %.1f deg (sign %+d). Keep the user stop button at "
                "hand. Ctrl-C starts a controlled stop ramp of %.1f s.\nPress Enter to start (Ctrl-C to abort)...",
                cli.travel_deg, cli.sign, cli.stop_ramp_s);
    std::fflush(stdout);
    const int ch = std::cin.get();
    if (g_sig != 0 || ch == EOF) {
      std::printf("\naborted before start (%s); no control started.\n", g_sig != 0 ? "signal" : "no confirmation read from stdin");
      return 3;
    }
    for (int k = cli.countdown_s; k >= 1; --k) {
      std::printf("starting in %d...\n", k);
      std::fflush(stdout);
      for (int i = 0; i < 10 && g_sig == 0; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (g_sig != 0) {
        std::printf("aborted during the countdown; no control started.\n");
        return 3;
      }
    }

    Controller& c = *ctl;
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)> cb = [&c](const franka::RobotState& s,
                                                                                         franka::Duration d) {
      const auto t0 = std::chrono::steady_clock::now();
      StepIn in{};
      for (std::size_t k = 0; k < 7; ++k) {
        in.q[k] = s.q[k];
        in.dq[k] = s.dq[k];
        in.tau_J[k] = s.tau_J[k];
        in.tau_ext[k] = s.tau_ext_hat_filtered[k];
      }
      for (std::size_t k = 0; k < 6; ++k) in.F_ext_K[k] = s.O_F_ext_hat_K[k];
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
    robot.control(cb, /*limit_rate=*/true, franka::kMaxCutoffFrequency);  // returns when motion_finished -> robot released
    std::printf("control loop ended after %ld cycles; robot released.\n", c.cycles());
  } catch (const franka::ControlException& e) {
    std::fprintf(stderr,
                 "\nERROR: franka::ControlException: %s\n"
                 "       The controlled stop ramp was NOT completed by this program (libfranka/robot reflex or error). The robot is\n"
                 "       probably in a reflex/error state: check it, then run automatic error recovery or restart from Desk.\n",
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

  if (ctl && control_started) {
    if (!csv.empty()) {
      const std::string err = writeCsv(csv, ctl->records());
      if (err.empty()) {
        std::printf("log written: %s (%zu rows)\n", csv.c_str(), ctl->records().size());
        writeMeta(csv, cli, cfg, p, *ctl, "robot");
      } else {
        std::fprintf(stderr, "ERROR: log not written: %s\n", err.c_str());
      }
    }
    printSummary(*ctl, p);
    if (exit_code == 0) exit_code = exitCodeFor(*ctl);
  }
  return exit_code;
}

}  // namespace

int main(int argc, char** argv) {
  Cli cli;
  const std::string perr = parseCli(argc, argv, cli);
  if (!perr.empty()) {
    std::fprintf(stderr, "error: %s\n", perr.c_str());
    usage(argv[0]);
    return 2;
  }
  // The profile must be valid before anything else (also catches a travel shorter than the two ramps).
  try {
    J7ProfileParams pp;
    pp.speed_rad_s = cli.speed_deg_s * kDeg;
    pp.travel_rad = cli.travel_deg * kDeg;
    pp.ramp_s = cli.ramp_s;
    pp.stop_ramp_s = cli.stop_ramp_s;
    pp.hold_s = cli.hold_s;
    const J7Profile test(pp);
    (void)test;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 2;
  }
  real_replay::Config cfg;
  try {
    cfg = real_replay::loadConfig(cli.config);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: config '%s': %s\n", cli.config.c_str(), e.what());
    return 2;
  }
  installSignals();
  try {
    return cli.dry_run ? runDry(cli, cfg) : runRobot(cli, cfg);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
