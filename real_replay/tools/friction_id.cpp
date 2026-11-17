// Standalone joint friction identification (Coulomb tau_c,j + viscous b_j, from the difference between the
// two directions of motion). libfranka torque control at 1 kHz; never touches controller_core.
//
// Per joint j (one at a time, the other six held by the same joint PD): reference
//   q_j(t) = q_start + s * v * t,   v in {0.02, 0.05, 0.10} rad/s, s in {+1, -1}.
// tau_cmd = Kp (q_ref - q) + Kd (dq_ref - dq)   (the robot adds gravity itself in FCI torque mode)
// goes through SafetyGuard (rate limit max_torque_step_nm, clamp torque_limit_fraction * effort).
//
// usage: ./friction_id [--config <yaml>] [--dry-run] [--yes] [--joints 1,2,..,7]
//                      [--kp k1,..,k7] [--kd d1,..,d7] [--out <file.csv>] [--robot-ip IP]
// See the "Defaults" block below. Exit codes: 0 ok, 1 runtime/preflight failure, 2 refused arguments.

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "real_replay/config.hpp"
#include "real_replay/preflight_checks.hpp"
#include "real_replay/safety.hpp"

using real_replay::Array7;

namespace {

// ---- Defaults ---------------------------------------------------------------------------------------
constexpr const char* kDefaultConfig = "config/replay_demo1014_f10_hand_kr30.yaml";
const Array7 kDefaultKp = {300.0, 300.0, 300.0, 300.0, 150.0, 100.0, 40.0};  // [Nm/rad]
const Array7 kDefaultKd = {15.0, 15.0, 15.0, 15.0, 8.0, 6.0, 3.0};            // [Nms/rad]
constexpr std::array<double, 3> kSpeeds = {0.02, 0.05, 0.10};                 // [rad/s]
constexpr double kSegmentS = 4.0;       // nominal segment duration (shortened if the excursion would exceed kMaxExcursion)
constexpr double kDiscardS = 1.5;       // transient discarded at the start of each segment
constexpr double kPauseS = 1.0;         // hold between segments (after the repositioning move)
constexpr double kMaxExcursion = 0.25;  // [rad] max distance of the reference from the initial q_j
constexpr double kLimitMargin = 0.30;   // [rad] required distance of q0_j +- kMaxExcursion... from the joint limits
constexpr double kBinRad = 0.02;        // q bin for pairing the two directions
constexpr double kReposSpeed = 0.03;    // [rad/s] peak-ish speed of the repositioning moves
constexpr double kMaxDq = 0.15;         // [rad/s] abort threshold on any joint
constexpr double kMaxHoldErr = 0.15;    // [rad] abort if any joint is further than this from its reference
constexpr double kStopFractionOfEffort = 0.3;  // abort if the PD demand exceeds this * effort (before the clamp)
constexpr double kDt = 1e-3;

// FER joint limits (franka-research-3 datasheet) [rad]
const Array7 kQMin = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
const Array7 kQMax = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};

// ---- CLI --------------------------------------------------------------------------------------------
struct Cli {
  std::string config = kDefaultConfig;
  std::string out;
  std::string robot_ip;
  bool dry_run = false;
  bool yes = false;
  std::vector<int> joints = {1, 2, 3, 4, 5, 6, 7};
  Array7 kp = kDefaultKp;
  Array7 kd = kDefaultKd;
  bool breakaway = false;       // static-friction (breakaway) mode instead of the kinetic one
  std::string kinetic_csv;      // breakaway report: CSV of a previous kinetic run -> tau_c,j
  Array7 tauc = {std::nan(""), std::nan(""), std::nan(""), std::nan(""), std::nan(""), std::nan(""), std::nan("")};
};

void usage(const char* p) {
  std::fprintf(stderr,
               "usage: %s [--config <yaml>] [--dry-run] [--yes] [--joints 1,2,..] [--kp k1..k7] [--kd d1..d7]\n"
               "          [--out file.csv] [--robot-ip IP] [--breakaway [--kinetic-csv kinetic.csv | --tauc t1..t7]]\n"
               "defaults: config=%s joints=1..7 kp=[300 300 300 300 150 100 40] kd=[15 15 15 15 8 6 3]\n",
               p, kDefaultConfig);
}

bool parseList(const std::string& s, std::vector<double>& out) {
  out.clear();
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    char* end = nullptr;
    const double v = std::strtod(tok.c_str(), &end);
    if (tok.empty() || end == tok.c_str() || *end != '\0') return false;
    out.push_back(v);
  }
  return !out.empty();
}

bool parseCli(int argc, char** argv, Cli& cli) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto val = [&](std::string& dst) {
      if (i + 1 >= argc) return false;
      dst = argv[++i];
      return true;
    };
    std::string v;
    std::vector<double> l;
    if (a == "--dry-run") {
      cli.dry_run = true;
    } else if (a == "--yes") {
      cli.yes = true;
    } else if (a == "--config") {
      if (!val(cli.config)) return false;
    } else if (a == "--out") {
      if (!val(cli.out)) return false;
    } else if (a == "--breakaway") {
      cli.breakaway = true;
    } else if (a == "--kinetic-csv") {
      if (!val(cli.kinetic_csv)) return false;
    } else if (a == "--tauc") {
      if (!val(v) || !parseList(v, l) || l.size() != 7) return false;
      for (std::size_t k = 0; k < 7; ++k) cli.tauc[k] = l[k];
    } else if (a == "--robot-ip") {
      if (!val(cli.robot_ip)) return false;
    } else if (a == "--joints") {
      if (!val(v) || !parseList(v, l)) return false;
      cli.joints.clear();
      for (double d : l) {
        if (d < 1 || d > 7 || d != std::floor(d)) return false;
        cli.joints.push_back(static_cast<int>(d));
      }
    } else if (a == "--kp" || a == "--kd") {
      if (!val(v) || !parseList(v, l) || l.size() != 7) return false;
      Array7& dst = a == "--kp" ? cli.kp : cli.kd;
      for (std::size_t k = 0; k < 7; ++k) {
        if (!(l[k] >= 0.0) || l[k] > 1000.0) return false;
        dst[k] = l[k];
      }
    } else {
      std::fprintf(stderr, "error: unknown or incomplete argument '%s'\n", a.c_str());
      return false;
    }
  }
  return true;
}

// ---- Experiment schedule ----------------------------------------------------------------------------
struct Seg {
  int joint;  // 0-based
  double v;
  int s;
  double dur;      // segment duration [s]
  double q_start;  // reference at the segment start
};

enum class Mode { kRepos, kHold, kSeg, kFinalRepos, kFinalHold, kDone };

struct Record {
  double t;
  int j;  // 1-based
  double v;
  int s;
  double q[7], dq[7], tau_cmd[7], tau_J[7];
};

struct StepIn {
  Array7 q, dq, tau_J;
};

enum class Abort { kNone, kSpeed, kTorque, kHoldError, kNonFinite };

class Experiment {
 public:
  Experiment(const real_replay::Config& cfg, const Cli& cli, const Array7& q0) : cli_(cli), guard_(cfg.safety, kQMin, kQMax), q0_(q0) {
    guard_.reset();
    for (std::size_t k = 0; k < 7; ++k) {
      tau_lim_[k] = kStopFractionOfEffort * cfg.safety.effort_limits_nm[k];
      ref_[k] = q0[k];
    }
    for (int j1 : cli.joints) {
      const std::size_t j = static_cast<std::size_t>(j1 - 1);
      for (double v : kSpeeds) {
        // excursion of the reference from q0: v*(T+kDiscard)/2 <= kMaxExcursion
        const double dur = std::min(kSegmentS, 2.0 * kMaxExcursion / v - kDiscardS);
        const double h = 0.5 * v * (dur - kDiscardS);  // half range of the steady window, centred on q0
        for (int s : {+1, -1}) {
          segs_.push_back({static_cast<int>(j), v, s, dur, q0[j] - s * (h + kDiscardS * v)});
        }
      }
    }
    records_.reserve(static_cast<std::size_t>(segs_.size() * (kSegmentS + 0.2) * 1000.0) + 1000);
    beginRepos(segs_.empty() ? q0_ : startPose(0), 0);
  }

  /// One 1 kHz cycle. Returns true while the experiment goes on.
  bool step(const StepIn& in, Array7& tau_out) {
    tau_out.fill(0.0);
    if (done()) return false;
    for (std::size_t k = 0; k < 7; ++k) {
      if (!std::isfinite(in.q[k]) || !std::isfinite(in.dq[k])) return abort(Abort::kNonFinite, k);
      if (std::fabs(in.dq[k]) > kMaxDq) return abort(Abort::kSpeed, k);
    }
    advanceReference();
    Array7 des{};
    for (std::size_t k = 0; k < 7; ++k) {
      const double e = ref_[k] - in.q[k];
      if (std::fabs(e) > kMaxHoldErr) return abort(Abort::kHoldError, k);
      des[k] = cli_.kp[k] * e + cli_.kd[k] * (dref_[k] - in.dq[k]);
      if (std::fabs(des[k]) > tau_lim_[k]) return abort(Abort::kTorque, k);
    }
    guard_.limitTorque(des, tau_out);
    if (mode_ == Mode::kSeg && records_.size() < records_.capacity()) {
      Record r{};
      r.t = t_in_mode_;
      r.j = segs_[idx_].joint + 1;
      r.v = segs_[idx_].v;
      r.s = segs_[idx_].s;
      for (std::size_t k = 0; k < 7; ++k) {
        r.q[k] = in.q[k];
        r.dq[k] = in.dq[k];
        r.tau_cmd[k] = tau_out[k];
        r.tau_J[k] = in.tau_J[k];
      }
      records_.push_back(r);
    }
    t_in_mode_ += kDt;
    ++cycles_;
    return true;
  }

  bool done() const { return mode_ == Mode::kDone; }
  bool aborted() const { return abort_ != Abort::kNone; }
  const std::string& abortMsg() const { return abort_msg_; }
  const std::vector<Record>& records() const { return records_; }
  long cycles() const { return cycles_; }
  const std::vector<Seg>& segs() const { return segs_; }
  double nominalDurationS() const {
    double t = 0.0;
    double ref_j_prev_q = 0.0;
    (void)ref_j_prev_q;
    for (std::size_t i = 0; i < segs_.size(); ++i) {
      const double from = i == 0 ? q0_[segs_[0].joint] : segs_[i - 1].q_start + segs_[i - 1].s * segs_[i - 1].v * segs_[i - 1].dur;
      const bool same = i > 0 && segs_[i - 1].joint == segs_[i].joint;
      const double a = same ? from : q0_[segs_[i].joint];
      t += reposDur(std::fabs(segs_[i].q_start - a)) + kPauseS + segs_[i].dur;
    }
    return t + 5.0;
  }

 private:
  static double reposDur(double dist) { return std::max(2.0, 1.5 * dist / kReposSpeed); }

  Array7 startPose(std::size_t i) const {
    Array7 p = q0_;
    p[static_cast<std::size_t>(segs_[i].joint)] = segs_[i].q_start;
    return p;
  }

  void beginRepos(const Array7& to, std::size_t idx) {
    idx_ = idx;
    from_ = ref_;
    to_ = to;
    double dist = 0.0;
    for (std::size_t k = 0; k < 7; ++k) dist = std::max(dist, std::fabs(to_[k] - from_[k]));
    repos_dur_ = reposDur(dist);
    mode_ = idx < segs_.size() ? Mode::kRepos : Mode::kFinalRepos;
    t_in_mode_ = 0.0;
  }

  void setMode(Mode m) {
    mode_ = m;
    t_in_mode_ = 0.0;
  }

  void advanceReference() {
    switch (mode_) {
      case Mode::kRepos:
      case Mode::kFinalRepos: {
        const double x = std::min(1.0, t_in_mode_ / repos_dur_);
        const double s = x * x * x * (10.0 - 15.0 * x + 6.0 * x * x);                 // min-jerk
        const double ds = 30.0 * x * x * (1.0 - x) * (1.0 - x) / repos_dur_;
        for (std::size_t k = 0; k < 7; ++k) {
          ref_[k] = from_[k] + (to_[k] - from_[k]) * s;
          dref_[k] = x < 1.0 ? (to_[k] - from_[k]) * ds : 0.0;
        }
        if (t_in_mode_ >= repos_dur_) setMode(mode_ == Mode::kRepos ? Mode::kHold : Mode::kFinalHold);
        break;
      }
      case Mode::kHold:
      case Mode::kFinalHold:
        dref_.fill(0.0);
        if (t_in_mode_ >= (mode_ == Mode::kHold ? kPauseS : 0.5)) {
          if (mode_ == Mode::kHold) {
            setMode(Mode::kSeg);
          } else {
            setMode(Mode::kDone);
          }
        }
        break;
      case Mode::kSeg: {
        const Seg& g = segs_[idx_];
        const std::size_t j = static_cast<std::size_t>(g.joint);
        ref_[j] = g.q_start + g.s * g.v * t_in_mode_;
        dref_[j] = g.s * g.v;
        if (t_in_mode_ >= g.dur) {
          ref_[j] = g.q_start + g.s * g.v * g.dur;
          dref_[j] = 0.0;
          const std::size_t next = idx_ + 1;
          // next segment: reposition (only the joint j changes; the target of the other joints is q0)
          Array7 to = q0_;
          if (next < segs_.size()) to = startPose(next);
          beginRepos(to, next);
        }
        break;
      }
      case Mode::kDone:
        break;
    }
  }

  bool abort(Abort a, std::size_t k) {
    abort_ = a;
    const char* what = a == Abort::kSpeed ? "|dq| > 0.15 rad/s" : a == Abort::kTorque ? "|tau_cmd| demand > 0.3*effort"
                       : a == Abort::kHoldError ? "tracking error > 0.15 rad" : "non-finite state";
    char b[160];
    std::snprintf(b, sizeof b, "ABORT: %s on joint %zu (cycle %ld, mode %d, segment %zu)", what, k + 1, cycles_,
                  static_cast<int>(mode_), idx_);
    abort_msg_ = b;
    mode_ = Mode::kDone;
    return false;
  }

  const Cli& cli_;
  real_replay::SafetyGuard guard_;
  Array7 q0_{};
  Array7 tau_lim_{};
  Array7 ref_{}, dref_{}, from_{}, to_{};
  std::vector<Seg> segs_;
  std::vector<Record> records_;
  std::size_t idx_ = 0;
  Mode mode_ = Mode::kRepos;
  double t_in_mode_ = 0.0;
  double repos_dur_ = 2.0;
  long cycles_ = 0;
  Abort abort_ = Abort::kNone;
  std::string abort_msg_;
};

// ---- Analysis ---------------------------------------------------------------------------------------
void writeCsv(const std::string& path, const std::vector<Record>& rec) {
  std::ofstream f(path);
  f << "t,j,v,s";
  for (const char* g : {"q", "dq", "tau_cmd", "tau_J"})
    for (int k = 1; k <= 7; ++k) f << ',' << g << k;
  f << '\n';
  f.precision(10);
  for (const Record& r : rec) {
    f << r.t << ',' << r.j << ',' << r.v << ',' << r.s;
    for (int k = 0; k < 7; ++k) f << ',' << r.q[k];
    for (int k = 0; k < 7; ++k) f << ',' << r.dq[k];
    for (int k = 0; k < 7; ++k) f << ',' << r.tau_cmd[k];
    for (int k = 0; k < 7; ++k) f << ',' << r.tau_J[k];
    f << '\n';
  }
}

void report(const std::vector<Record>& rec, const std::vector<int>& joints) {
  std::printf("\n=== FRICTION RESULT (D = mean(tau_cmd - tau_J) over steady samples t >= %.1f s of the segment, bins of %.2f rad "
              "present in BOTH directions) ===\n", kDiscardS, kBinRad);
  std::printf("convention: moving +: tau_J = tau_cmd + g - f  =>  D(+v) = -g + f, D(-v) = -g - f  =>  tau_c = (D(+v) - D(-v))/2 >= 0\n");
  std::printf("joint |   v    | D(+v)   D(-v)   tau_c(v) | bins  n(+) n(-)\n");
  for (int j1 : joints) {
    const std::size_t j = static_cast<std::size_t>(j1 - 1);
    std::vector<double> vs, tcs;
    for (double v : kSpeeds) {
      std::map<long, std::vector<double>> bins[2];
      for (const Record& r : rec) {
        if (r.j != j1 || std::fabs(r.v - v) > 1e-9 || r.t < kDiscardS) continue;
        bins[r.s > 0 ? 0 : 1][static_cast<long>(std::floor(r.q[j] / kBinRad))].push_back(r.tau_cmd[j] - r.tau_J[j]);
      }
      double sum[2] = {0, 0};
      long n[2] = {0, 0};
      long nb = 0;
      for (const auto& kv : bins[0]) {
        const auto it = bins[1].find(kv.first);
        if (it == bins[1].end()) continue;
        ++nb;
        for (double d : kv.second) { sum[0] += d; ++n[0]; }
        for (double d : it->second) { sum[1] += d; ++n[1]; }
      }
      if (nb == 0 || n[0] == 0 || n[1] == 0) {
        std::printf("  J%d  | %.3f  |  no common q bins between the two directions\n", j1, v);
        continue;
      }
      const double dp = sum[0] / static_cast<double>(n[0]);
      const double dm = sum[1] / static_cast<double>(n[1]);
      const double tc = 0.5 * (dp - dm);
      std::printf("  J%d  | %.3f  | %+7.3f %+7.3f %+7.3f  | %3ld  %5ld %5ld\n", j1, v, dp, dm, tc, nb, n[0], n[1]);
      vs.push_back(v);
      tcs.push_back(tc);
    }
    if (vs.size() >= 2) {
      double mv = 0, mt = 0;
      for (std::size_t i = 0; i < vs.size(); ++i) { mv += vs[i]; mt += tcs[i]; }
      mv /= static_cast<double>(vs.size());
      mt /= static_cast<double>(vs.size());
      double sxy = 0, sxx = 0;
      for (std::size_t i = 0; i < vs.size(); ++i) { sxy += (vs[i] - mv) * (tcs[i] - mt); sxx += (vs[i] - mv) * (vs[i] - mv); }
      const double b = sxy / sxx;
      std::printf("  J%d  => b_j = %.3f Nms/rad (slope of tau_c(v), %zu speeds), tau_c,j = %.3f Nm (intercept at v=0)\n", j1, b,
                  vs.size(), mt - b * mv);
    }
  }
}

// ---- Robot / dry-run --------------------------------------------------------------------------------
std::string timestamp() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&t, &tm);
  char b[32];
  std::strftime(b, sizeof b, "%Y%m%d_%H%M%S", &tm);
  return b;
}

std::string outPath(const Cli& cli, const real_replay::Config& cfg) {
  if (!cli.out.empty()) return cli.out;
  std::error_code ec;
  std::filesystem::create_directories(cfg.paths.log_dir, ec);
  return (std::filesystem::path(cfg.paths.log_dir) / (timestamp() + "_friction_id_" + (cli.dry_run ? "dryrun" : "robot") + ".csv")).string();
}

bool marginsOk(const Cli& cli, const Array7& q0, double joint_limit_margin) {
  bool ok = true;
  for (int j1 : cli.joints) {
    const std::size_t j = static_cast<std::size_t>(j1 - 1);
    const double lo = q0[j] - kQMin[j], hi = kQMax[j] - q0[j];
    const bool good = lo >= kLimitMargin && hi >= kLimitMargin && lo >= kMaxExcursion + joint_limit_margin &&
                      hi >= kMaxExcursion + joint_limit_margin;
    std::printf("  [%s] joint%d: q0=%+.3f, distance to limits %.3f / %.3f rad (need >= %.2f and >= %.2f + margin %.2f)\n",
                good ? "PASS" : "FAIL", j1, q0[j], lo, hi, kLimitMargin, kMaxExcursion, joint_limit_margin);
    ok = ok && good;
  }
  return ok;
}

// Simulated plant for --dry-run: 1 kHz, per joint I*ddq = tau_cmd - f(dq), tau_J = tau_cmd - f - I*ddq + g_syn (the
// "robot" compensates gravity: g_syn only shows in tau_J). True friction: Coulomb 1.5/1.0/0.8 Nm, viscous 2.0 Nms/rad.
void simStep(Array7& q, Array7& dq, Array7& tauJ, const Array7& tau_cmd) {
  static const double I[7] = {1.2, 1.5, 1.0, 1.0, 0.4, 0.3, 0.15};
  static const double gs[7] = {0.0, -9.0, 1.5, 19.0, 0.6, 1.2, 0.0};
  static const double fc[7] = {1.5, 1.5, 1.2, 1.2, 0.8, 0.8, 0.6};
  static const double bv[7] = {2.0, 2.0, 1.5, 1.5, 0.8, 0.8, 0.5};
  for (std::size_t k = 0; k < 7; ++k) {
    const double f = fc[k] * std::tanh(dq[k] / 2e-3) + bv[k] * dq[k];
    const double acc = (tau_cmd[k] - f) / I[k];
    dq[k] += acc * kDt;
    q[k] += dq[k] * kDt;
    tauJ[k] = tau_cmd[k] - f - I[k] * acc + gs[k];
  }
}

int runBreakawayDry(const Cli& cli, const real_replay::Config& cfg, Array7 q);
int runBreakawayRobot(franka::Robot& robot, const Cli& cli, const real_replay::Config& cfg, const Array7& q0);

int runDry(const Cli& cli, const real_replay::Config& cfg) {
  std::printf("=== DRY RUN (simulated plant, no robot) ===\n");
  Array7 q = {0.0, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, 1.5707963267948966, 0.7853981633974483};
  Array7 dq{}, tauJ{}, tau{};
  std::printf("preflight (joint margins):\n");
  if (!marginsOk(cli, q, cfg.safety.joint_limit_margin_rad)) return 1;
  if (cli.breakaway) return runBreakawayDry(cli, cfg, q);
  Experiment ex(cfg, cli, q);
  std::printf("segments: %zu, nominal duration %.0f s (simulated, not real time)\n", ex.segs().size(), ex.nominalDurationS());
  StepIn in{};
  while (!ex.done()) {
    in.q = q;
    in.dq = dq;
    in.tau_J = tauJ;
    if (!ex.step(in, tau)) break;
    simStep(q, dq, tauJ, tau);
  }
  const std::string path = outPath(cli, cfg);
  writeCsv(path, ex.records());
  std::printf("%ld cycles, %zu logged samples, CSV: %s\n", ex.cycles(), ex.records().size(), path.c_str());
  if (ex.aborted()) std::printf("%s\n", ex.abortMsg().c_str());
  report(ex.records(), cli.joints);
  std::printf("(plant truth: tau_c = 1.5 1.5 1.2 1.2 0.8 0.8 0.6 Nm, b = 2.0 2.0 1.5 1.5 0.8 0.8 0.5 Nms/rad)\n");
  return ex.aborted() ? 1 : 0;
}

int runRobot(const Cli& cli, const real_replay::Config& cfg) {
  const std::string ip = cli.robot_ip.empty() ? cfg.robot.ip : cli.robot_ip;
  std::printf("=== ROBOT RUN: %s ===\n", ip.c_str());
  franka::Robot* rp = nullptr;
  try {
    franka::Robot robot(ip, franka::RealtimeConfig::kIgnore);
    rp = &robot;
    // Settings go BEFORE the control loop (same order as real_replay main.cpp).
    robot.setCollisionBehavior(cfg.collision.lower_torque_acc, cfg.collision.upper_torque_acc, cfg.collision.lower_torque_nom,
                               cfg.collision.upper_torque_nom, cfg.collision.lower_force_acc, cfg.collision.upper_force_acc,
                               cfg.collision.lower_force_nom, cfg.collision.upper_force_nom);
    robot.setJointImpedance(cfg.collision.joint_impedance);
    robot.setCartesianImpedance(cfg.collision.cartesian_impedance);
    if (cfg.end_effector.set_ee) robot.setEE(cfg.end_effector.ne_t_ee);
    if (cfg.end_effector.set_load) {
      robot.setLoad(cfg.end_effector.load_mass, cfg.end_effector.f_x_cload, cfg.end_effector.load_inertia);
    }

    const franka::RobotState s0 = robot.readOnce();
    int failures = 0;
    auto check = [&](const char* name, bool ok, const std::string& detail) {
      std::printf("  [%s] %s: %s\n", ok ? "PASS" : "FAIL", name, detail.c_str());
      if (!ok) ++failures;
    };
    char b[200];
    std::printf("preflight:\n");
    check("robot_mode == Idle", s0.robot_mode == franka::RobotMode::kIdle, "robot_mode=" + std::to_string(static_cast<int>(s0.robot_mode)));
    std::snprintf(b, sizeof b, "m_ee=%.3f kg, limit %.3f", s0.m_ee, cfg.preflight.m_ee_max_kg);
    check("m_ee <= limit", s0.m_ee <= cfg.preflight.m_ee_max_kg, b);
    std::snprintf(b, sizeof b, "m_load=%.3f kg, limit %.3f", s0.m_load, cfg.preflight.m_load_max_kg);
    check("m_load <= limit", s0.m_load <= cfg.preflight.m_load_max_kg, b);
    if (cfg.end_effector.set_ee) {
      double d = 0.0;
      for (std::size_t i = 0; i < 16; ++i) d = std::max(d, std::fabs(s0.NE_T_EE[i] - cfg.end_effector.ne_t_ee[i]));
      std::snprintf(b, sizeof b, "max |diff| %.3e", d);
      check("NE_T_EE in the state == config", d < 1e-9, b);
    }
    Array7 q0{};
    for (std::size_t k = 0; k < 7; ++k) q0[k] = s0.q[k];
    std::printf("  joint limit margins:\n");
    check("joint margins", marginsOk(cli, q0, cfg.safety.joint_limit_margin_rad), "see above");
    if (cfg.preflight.check_gravity) {
      franka::Model fm = robot.loadModel();
      const std::array<double, 7> gf = fm.gravity(s0);
      Array7 g{}, tj{}, dqv{};
      for (std::size_t k = 0; k < 7; ++k) { g[k] = gf[k]; tj[k] = s0.tau_J[k]; dqv[k] = s0.dq[k]; }
      const auto r = real_replay::checkGravityVsMeasured(g, tj, dqv, cfg.preflight.gravity_vs_measured_tol_nm,
                                                         cfg.preflight.gravity_still_max_dq_rad_s);
      std::snprintf(b, sizeof b, "max |franka - tau_J| %.3f Nm on joint%d (tol %.3f), max|dq| %.4f rad/s", r.max_abs_diff,
                    r.worst_joint + 1, cfg.preflight.gravity_vs_measured_tol_nm, r.max_abs_dq);
      check("gravity: franka model vs measured tau_J", r.verdict == real_replay::MeasuredGravityResult::Verdict::kPass, b);
    }
    if (failures > 0) {
      std::printf("preflight: %d failure(s) -> aborting before control\n", failures);
      return 1;
    }
    if (cli.breakaway) return runBreakawayRobot(robot, cli, cfg, q0);

    Experiment ex(cfg, cli, q0);
    std::printf("\nsegments: %zu, nominal duration %.0f s. Joint PD kp=[", ex.segs().size(), ex.nominalDurationS());
    for (double k : cli.kp) std::printf(" %.0f", k);
    std::printf(" ] kd=[");
    for (double k : cli.kd) std::printf(" %.0f", k);
    std::printf(" ]\nWARNING: the robot will be torque-controlled and will move each joint slowly (+-%.2f rad max). Keep the user stop "
                "button at hand.\n", kMaxExcursion);
    if (!cli.yes) {
      std::printf("Press Enter to start (Ctrl+C to abort)...");
      std::fflush(stdout);
      std::cin.get();
    }

    std::function<franka::Torques(const franka::RobotState&, franka::Duration)> cb =
        [&ex](const franka::RobotState& s, franka::Duration) {
          StepIn in{};
          for (std::size_t k = 0; k < 7; ++k) {
            in.q[k] = s.q[k];
            in.dq[k] = s.dq[k];
            in.tau_J[k] = s.tau_J[k];
          }
          Array7 tau{};
          const bool go = ex.step(in, tau);
          franka::Torques t(tau);
          t.motion_finished = !go;
          return t;
        };
    robot.control(cb, /*limit_rate=*/true, franka::kMaxCutoffFrequency);  // returns when motion_finished -> robot released
    std::printf("control loop ended after %ld cycles; robot released.\n", ex.cycles());
    const std::string path = outPath(cli, cfg);
    writeCsv(path, ex.records());
    std::printf("%zu logged samples, CSV: %s\n", ex.records().size(), path.c_str());
    if (ex.aborted()) std::printf("%s\n", ex.abortMsg().c_str());
    report(ex.records(), cli.joints);
    return ex.aborted() ? 1 : 0;
  } catch (const franka::Exception& e) {
    std::fprintf(stderr, "franka::Exception: %s\n", e.what());
    if (rp != nullptr) {
      try { rp->stop(); } catch (...) {}
    }
    return 1;
  }
}

// ---- Breakaway (static friction) mode ---------------------------------------------------------------
// For each joint, rate r in {0.002, 0.005, 0.01} rad/s, 5 repetitions: [+ then -] from the same q0: hold 2 s, ramp the
// reference at rate r until |q_j - q_j(start)| > 0.002 rad (breakaway), D_break = mean(tau_cmd - tau_J) over the 20 ms
// before the breakaway cycle, then stop the ramp and go back slowly to q0.
constexpr double kBaWaitS = 2.0;
constexpr double kBaThresh = 0.002;   // [rad] movement that defines the breakaway
constexpr double kBaAvgS = 0.020;     // averaging window before the breakaway
constexpr double kBaMaxRefErr = 0.05; // [rad] abort if the reference gets this far from q_j without breakaway
constexpr int kBaLog = 500;           // samples (0.5 s) logged before each breakaway
constexpr int kBaReps = 5;
constexpr std::array<double, 3> kBaRates = {0.002, 0.005, 0.01};

struct BaSamp {
  double q[7], dq[7], tc[7], tj[7];
};
struct BaEvent {
  int joint, s, rep;
  double r, D, q_break, Dpk;
};
struct BaRow {
  int trial, joint, s, rep;
  double r, t, D, q_break, Dpk;
  BaSamp x;
};

class BreakExperiment {
 public:
  struct Trial {
    int joint;  // 0-based
    double r;
    int s, rep;
  };
  BreakExperiment(const real_replay::Config& cfg, const Cli& cli, const Array7& q0)
      : cli_(cli), guard_(cfg.safety, kQMin, kQMax), q0_(q0) {
    guard_.reset();
    for (std::size_t k = 0; k < 7; ++k) {
      tau_lim_[k] = kStopFractionOfEffort * cfg.safety.effort_limits_nm[k];
      ref_[k] = q0[k];
    }
    for (int j1 : cli.joints)
      for (double r : kBaRates)
        for (int rep = 0; rep < kBaReps; ++rep)
          for (int s : {+1, -1}) trials_.push_back({j1 - 1, r, s, rep + 1});
    rows_.reserve(trials_.size() * kBaLog + 10);
    mode_ = Mode::kWait;
  }

  bool step(const StepIn& in, Array7& tau_out) {
    tau_out.fill(0.0);
    if (mode_ == Mode::kDone) return false;
    for (std::size_t k = 0; k < 7; ++k) {
      if (!std::isfinite(in.q[k]) || !std::isfinite(in.dq[k])) return abort("non-finite state", k);
      if (std::fabs(in.dq[k]) > kMaxDq) return abort("|dq| > 0.15 rad/s", k);
    }
    const Trial& tr = trials_[idx_];
    const std::size_t j = static_cast<std::size_t>(tr.joint);
    bool broke = false;
    switch (mode_) {
      case Mode::kWait:
        dref_.fill(0.0);
        if (t_ >= kBaWaitS) {
          mode_ = Mode::kRamp;
          t_ = 0.0;
          q_start_ = in.q[j];
        }
        break;
      case Mode::kRamp:
        if (std::fabs(in.q[j] - q_start_) > kBaThresh) {
          broke = true;
        } else {
          ref_[j] = q0_[j] + tr.s * tr.r * t_;
          dref_[j] = tr.s * tr.r;
          if (std::fabs(ref_[j] - in.q[j]) > kBaMaxRefErr) return abort("no breakaway within 0.05 rad of reference error", j);
        }
        break;
      case Mode::kReturn: {
        const double x = std::min(1.0, t_ / ret_dur_);
        const double sm = x * x * x * (10.0 - 15.0 * x + 6.0 * x * x);
        const double ds = 30.0 * x * x * (1.0 - x) * (1.0 - x) / ret_dur_;
        ref_[j] = from_ + (q0_[j] - from_) * sm;
        dref_[j] = x < 1.0 ? (q0_[j] - from_) * ds : 0.0;
        if (t_ >= ret_dur_) {
          ref_[j] = q0_[j];
          dref_[j] = 0.0;
          if (++idx_ >= trials_.size()) {
            mode_ = Mode::kDone;
            return false;
          }
          mode_ = Mode::kWait;
          t_ = -kDt;  // incremented to 0 at the end of this cycle
        }
        break;
      }
      case Mode::kDone:
        break;
    }
    if (broke) {  // record the event from the samples BEFORE this cycle, then freeze the reference and return
      const std::size_t n = std::min<std::size_t>(count_, kBaLog);
      const std::size_t na = std::min<std::size_t>(n, static_cast<std::size_t>(kBaAvgS / kDt));
      double D = 0.0;
      for (std::size_t i = 0; i < na; ++i) {
        const BaSamp& sm = ring_[(head_ + kBaLog - 1 - i) % kBaLog];
        D += sm.tc[j] - sm.tj[j];
      }
      D = na ? D / static_cast<double>(na) : std::nan("");
      // Peak of D = tau_cmd - tau_J over the whole logged window, on a centred 20-sample moving average
      // (max for the + direction, min for the -): only windows fully inside the buffer are used.
      double Dpk = std::nan("");
      {
        constexpr std::size_t kW = 20, kHalf = 10;
        std::array<double, kBaLog> d{};
        for (std::size_t i = 0; i < n; ++i) {
          const BaSamp& sm = ring_[(head_ + kBaLog - n + i) % kBaLog];
          d[i] = sm.tc[j] - sm.tj[j];
        }
        for (std::size_t i = kHalf; i + (kW - kHalf) <= n; ++i) {  // window [i-10, i+9]
          double a = 0.0;
          for (std::size_t k = i - kHalf; k < i - kHalf + kW; ++k) a += d[k];
          a /= static_cast<double>(kW);
          if (!std::isfinite(Dpk) || (tr.s > 0 ? a > Dpk : a < Dpk)) Dpk = a;
        }
      }
      events_.push_back({tr.joint + 1, tr.s, tr.rep, tr.r, D, in.q[j], Dpk});
      for (std::size_t i = 0; i < n; ++i) {
        const BaSamp& sm = ring_[(head_ + kBaLog - n + i) % kBaLog];
        rows_.push_back({static_cast<int>(idx_) + 1, tr.joint + 1, tr.s, tr.rep, tr.r, -kDt * static_cast<double>(n - i), D, in.q[j], Dpk, sm});
      }
      from_ = ref_[j];
      dref_.fill(0.0);
      ret_dur_ = std::max(2.0, 1.5 * std::fabs(q0_[j] - from_) / kReposSpeed);
      mode_ = Mode::kReturn;
      t_ = -kDt;
    }
    Array7 des{};
    for (std::size_t k = 0; k < 7; ++k) {
      const double e = ref_[k] - in.q[k];
      if (std::fabs(e) > kMaxHoldErr) return abort("tracking error > 0.15 rad", k);
      des[k] = cli_.kp[k] * e + cli_.kd[k] * (dref_[k] - in.dq[k]);
      if (std::fabs(des[k]) > tau_lim_[k]) return abort("|tau_cmd| demand > 0.3*effort", k);
    }
    guard_.limitTorque(des, tau_out);
    BaSamp& s = ring_[head_];
    for (std::size_t k = 0; k < 7; ++k) {
      s.q[k] = in.q[k];
      s.dq[k] = in.dq[k];
      s.tc[k] = tau_out[k];
      s.tj[k] = in.tau_J[k];
    }
    head_ = (head_ + 1) % kBaLog;
    ++count_;
    t_ += kDt;
    ++cycles_;
    return true;
  }

  bool done() const { return mode_ == Mode::kDone; }
  bool aborted() const { return !abort_msg_.empty(); }
  const std::string& abortMsg() const { return abort_msg_; }
  long cycles() const { return cycles_; }
  const std::vector<BaEvent>& events() const { return events_; }
  const std::vector<BaRow>& rows() const { return rows_; }
  std::size_t trials() const { return trials_.size(); }
  /// Rough duration of one joint: 2 s wait + ramp until the PD error reaches ~(f_guess/Kp + 2 mrad) + >= 2 s return.
  static double estimatedJointS(double kp, double f_guess = 2.0) {
    double t = 0.0;
    for (double r : kBaRates) t += 2.0 * kBaReps * (kBaWaitS + (f_guess / kp + kBaThresh) / r + 2.0);
    return t;
  }

 private:
  enum class Mode { kWait, kRamp, kReturn, kDone };
  bool abort(const char* what, std::size_t k) {
    char b[200];
    std::snprintf(b, sizeof b, "ABORT: %s on joint %zu (cycle %ld, trial %zu/%zu)", what, k + 1, cycles_, idx_ + 1, trials_.size());
    abort_msg_ = b;
    mode_ = Mode::kDone;
    return false;
  }

  const Cli& cli_;
  real_replay::SafetyGuard guard_;
  Array7 q0_{}, tau_lim_{}, ref_{}, dref_{};
  std::vector<Trial> trials_;
  std::vector<BaEvent> events_;
  std::vector<BaRow> rows_;
  std::array<BaSamp, kBaLog> ring_{};
  std::size_t head_ = 0, count_ = 0, idx_ = 0;
  Mode mode_ = Mode::kWait;
  double t_ = 0.0, q_start_ = 0.0, from_ = 0.0, ret_dur_ = 2.0;
  long cycles_ = 0;
  std::string abort_msg_;
};

void writeBaCsv(const std::string& path, const std::vector<BaRow>& rows) {
  std::ofstream f(path);
  f << "trial,j,r,s,rep,t,D_break,q_break";
  for (const char* g : {"q", "dq", "tau_cmd", "tau_J"})
    for (int k = 1; k <= 7; ++k) f << ',' << g << k;
  f << ",D_break_peak\n";
  f.precision(10);
  for (const BaRow& r : rows) {
    f << r.trial << ',' << r.joint << ',' << r.r << ',' << r.s << ',' << r.rep << ',' << r.t << ',' << r.D << ',' << r.q_break;
    for (int k = 0; k < 7; ++k) f << ',' << r.x.q[k];
    for (int k = 0; k < 7; ++k) f << ',' << r.x.dq[k];
    for (int k = 0; k < 7; ++k) f << ',' << r.x.tc[k];
    for (int k = 0; k < 7; ++k) f << ',' << r.x.tj[k];
    f << ',' << r.Dpk << '\n';
  }
}

/// Kinetic tau_c,j (intercept of tau_c(v), same pairing as report()) from the CSV of a previous kinetic run.
std::array<double, 7> kineticTauC(const std::string& csv) {
  std::array<double, 7> out;
  out.fill(std::nan(""));
  std::ifstream f(csv);
  if (!f) throw std::runtime_error("cannot open kinetic CSV " + csv);
  std::string line;
  std::getline(f, line);
  std::vector<Record> rec;
  while (std::getline(f, line)) {
    std::vector<double> v;
    if (!parseList(line, v) || v.size() < 32) continue;
    Record r{};
    r.t = v[0]; r.j = static_cast<int>(v[1]); r.v = v[2]; r.s = static_cast<int>(v[3]);
    for (int k = 0; k < 7; ++k) { r.q[k] = v[4 + k]; r.dq[k] = v[11 + k]; r.tau_cmd[k] = v[18 + k]; r.tau_J[k] = v[25 + k]; }
    rec.push_back(r);
  }
  for (int j1 = 1; j1 <= 7; ++j1) {
    const std::size_t j = static_cast<std::size_t>(j1 - 1);
    std::vector<double> vs, tcs;
    for (double v : kSpeeds) {
      std::map<long, std::vector<double>> bins[2];
      for (const Record& r : rec) {
        if (r.j != j1 || std::fabs(r.v - v) > 1e-9 || r.t < kDiscardS) continue;
        bins[r.s > 0 ? 0 : 1][static_cast<long>(std::floor(r.q[j] / kBinRad))].push_back(r.tau_cmd[j] - r.tau_J[j]);
      }
      double sum[2] = {0, 0};
      long n[2] = {0, 0};
      for (const auto& kv : bins[0]) {
        const auto it = bins[1].find(kv.first);
        if (it == bins[1].end()) continue;
        for (double d : kv.second) { sum[0] += d; ++n[0]; }
        for (double d : it->second) { sum[1] += d; ++n[1]; }
      }
      if (n[0] && n[1]) { vs.push_back(v); tcs.push_back(0.5 * (sum[0] / static_cast<double>(n[0]) - sum[1] / static_cast<double>(n[1]))); }
    }
    if (vs.size() < 2) continue;
    double mv = 0, mt = 0;
    for (std::size_t i = 0; i < vs.size(); ++i) { mv += vs[i]; mt += tcs[i]; }
    mv /= static_cast<double>(vs.size());
    mt /= static_cast<double>(vs.size());
    double sxy = 0, sxx = 0;
    for (std::size_t i = 0; i < vs.size(); ++i) { sxy += (vs[i] - mv) * (tcs[i] - mt); sxx += (vs[i] - mv) * (vs[i] - mv); }
    out[j] = mt - sxy / sxx * mv;
  }
  return out;
}

void meanStd(const std::vector<double>& v, double& m, double& sd) {
  m = 0.0;
  for (double x : v) m += x;
  m /= static_cast<double>(v.size());
  double s2 = 0.0;
  for (double x : v) s2 += (x - m) * (x - m);
  sd = v.size() > 1 ? std::sqrt(s2 / static_cast<double>(v.size() - 1)) : 0.0;
}

void reportBreakaway(const Cli& cli, const std::vector<BaEvent>& ev) {
  Array7 tauc;
  tauc.fill(std::nan(""));
  if (!cli.kinetic_csv.empty()) tauc = kineticTauC(cli.kinetic_csv);
  for (std::size_t k = 0; k < 7; ++k)
    if (std::isfinite(cli.tauc[k])) tauc[k] = cli.tauc[k];
  std::printf("\n=== BREAKAWAY RESULT (D_peak = peak of tau_cmd - tau_J before the breakaway, centred 20-sample moving average over the"
              " logged 0.5 s: max for +, min for -; breakaway = |q_j - q_start| > %.3f rad; mean +- std over %d repetitions) ===\n", kBaThresh, kBaReps);
  std::printf("f_s,j = (D_peak(+) - D_peak(-))/2 ; f_s[mean20ms] (reference only) uses D_break = mean(tau_cmd - tau_J) over the last 20 ms;"
              " tau_c,j kinetic %s\n",
              cli.kinetic_csv.empty() && !std::isfinite(cli.tauc[0]) ? "not given (use --kinetic-csv or --tauc)" : "from --kinetic-csv/--tauc (intercept at v=0)");
  std::printf("joint |   r    | n+ n- |   D_peak(+)       D_peak(-)    |  f_s   | tau_c,j  f_s/f_c | f_s[mean20ms]  f_s/f_c\n");
  for (int j1 : cli.joints) {
    for (double r : kBaRates) {
      std::vector<double> d[2], dm[2];
      for (const BaEvent& e : ev)
        if (e.joint == j1 && std::fabs(e.r - r) < 1e-12 && std::isfinite(e.Dpk) && std::isfinite(e.D)) {
          d[e.s > 0 ? 0 : 1].push_back(e.Dpk);
          dm[e.s > 0 ? 0 : 1].push_back(e.D);
        }
      if (d[0].empty() || d[1].empty()) {
        std::printf("  J%d  | %.3f  | %zu  %zu  | incomplete\n", j1, r, d[0].size(), d[1].size());
        continue;
      }
      double mp, sp, mm, sm, mp2, sp2, mm2, sm2;
      meanStd(d[0], mp, sp);
      meanStd(d[1], mm, sm);
      meanStd(dm[0], mp2, sp2);
      meanStd(dm[1], mm2, sm2);
      const double fs = 0.5 * (mp - mm);
      const double fs2 = 0.5 * (mp2 - mm2);
      const double fc = tauc[static_cast<std::size_t>(j1 - 1)];
      std::printf("  J%d  | %.3f  | %zu  %zu  | %+7.3f +-%.3f  %+7.3f +-%.3f | %+6.3f |", j1, r, d[0].size(), d[1].size(), mp, sp, mm, sm, fs);
      if (std::isfinite(fc)) {
        std::printf(" %6.3f  %6.3f | %+6.3f  %6.3f\n", fc, fs / fc, fs2, fs2 / fc);
      } else {
        std::printf("   n/a      n/a | %+6.3f    n/a\n", fs2);
      }
    }
  }
}

std::string baOutPath(const Cli& cli, const real_replay::Config& cfg) {
  if (!cli.out.empty()) return cli.out;
  std::error_code ec;
  std::filesystem::create_directories(cfg.paths.log_dir, ec);
  return (std::filesystem::path(cfg.paths.log_dir) / (timestamp() + "_friction_id_breakaway_" + (cli.dry_run ? "dryrun" : "robot") + ".csv")).string();
}

int finishBreakaway(const Cli& cli, const real_replay::Config& cfg, const BreakExperiment& ex) {
  const std::string path = baOutPath(cli, cfg);
  writeBaCsv(path, ex.rows());
  std::printf("%ld cycles, %zu breakaway events, %zu logged samples, CSV: %s\n", ex.cycles(), ex.events().size(), ex.rows().size(), path.c_str());
  if (ex.aborted()) std::printf("%s\n", ex.abortMsg().c_str());
  reportBreakaway(cli, ex.events());
  return ex.aborted() ? 1 : 0;
}

void printBaPlan(const Cli& cli, const BreakExperiment& ex) {
  std::printf("breakaway: %zu trials (%zu joints x %zu rates x %d reps x 2 directions)\n", ex.trials(), cli.joints.size(), kBaRates.size(), kBaReps);
  for (int j1 : cli.joints)
    std::printf("  J%d: estimated ~%.0f s (kp=%.0f, assuming ~2 Nm breakaway torque)\n", j1,
                BreakExperiment::estimatedJointS(cli.kp[static_cast<std::size_t>(j1 - 1)]), cli.kp[static_cast<std::size_t>(j1 - 1)]);
}

// Dry-run plant with static friction f_s = 1.3 f_c: stuck while |tau_cmd| <= f_s (then tau_J = g_syn, i.e. D = tau_cmd - g).
void simStepStatic(Array7& q, Array7& dq, Array7& tauJ, const Array7& tau_cmd) {
  static const double I[7] = {1.2, 1.5, 1.0, 1.0, 0.4, 0.3, 0.15};
  static const double gs[7] = {0.0, -9.0, 1.5, 19.0, 0.6, 1.2, 0.0};
  static const double fc[7] = {1.5, 1.5, 1.2, 1.2, 0.8, 0.8, 0.6};
  static const double bv[7] = {2.0, 2.0, 1.5, 1.5, 0.8, 0.8, 0.5};
  for (std::size_t k = 0; k < 7; ++k) {
    if (std::fabs(dq[k]) < 1e-7 && std::fabs(tau_cmd[k]) <= 1.3 * fc[k]) {
      dq[k] = 0.0;
      tauJ[k] = gs[k];
      continue;
    }
    const double sg = std::fabs(dq[k]) < 1e-7 ? (tau_cmd[k] > 0 ? 1.0 : -1.0) : std::tanh(dq[k] / 2e-3);
    const double f = fc[k] * sg + bv[k] * dq[k];
    const double acc = (tau_cmd[k] - f) / I[k];
    dq[k] += acc * kDt;
    q[k] += dq[k] * kDt;
    tauJ[k] = tau_cmd[k] - f - I[k] * acc + gs[k];
  }
}

int runBreakawayDry(const Cli& cli, const real_replay::Config& cfg, Array7 q) {
  Array7 dq{}, tauJ{}, tau{};
  BreakExperiment ex(cfg, cli, q);
  printBaPlan(cli, ex);
  StepIn in{};
  while (!ex.done()) {
    in.q = q;
    in.dq = dq;
    in.tau_J = tauJ;
    if (!ex.step(in, tau)) break;
    simStepStatic(q, dq, tauJ, tau);
  }
  const int rc = finishBreakaway(cli, cfg, ex);
  std::printf("(plant truth: f_s = 1.3 * f_c = 1.95 1.95 1.56 1.56 1.04 1.04 0.78 Nm)\n");
  return rc;
}

int runBreakawayRobot(franka::Robot& robot, const Cli& cli, const real_replay::Config& cfg, const Array7& q0) {
  BreakExperiment ex(cfg, cli, q0);
  std::printf("\n");
  printBaPlan(cli, ex);
  std::printf("WARNING: the robot will be torque-controlled; each joint is moved by a few mrad at breakaway (reference error limit %.2f rad). "
              "Keep the user stop button at hand.\n", kBaMaxRefErr);
  if (!cli.yes) {
    std::printf("Press Enter to start (Ctrl+C to abort)...");
    std::fflush(stdout);
    std::cin.get();
  }
  std::function<franka::Torques(const franka::RobotState&, franka::Duration)> cb = [&ex](const franka::RobotState& s, franka::Duration) {
    StepIn in{};
    for (std::size_t k = 0; k < 7; ++k) {
      in.q[k] = s.q[k];
      in.dq[k] = s.dq[k];
      in.tau_J[k] = s.tau_J[k];
    }
    Array7 tau{};
    const bool go = ex.step(in, tau);
    franka::Torques t(tau);
    t.motion_finished = !go;
    return t;
  };
  robot.control(cb, /*limit_rate=*/true, franka::kMaxCutoffFrequency);  // returns when motion_finished -> robot released
  std::printf("control loop ended after %ld cycles; robot released.\n", ex.cycles());
  return finishBreakaway(cli, cfg, ex);
}

}  // namespace

int main(int argc, char** argv) {
  Cli cli;
  if (!parseCli(argc, argv, cli)) {
    usage(argv[0]);
    return 2;
  }
  real_replay::Config cfg;
  try {
    cfg = real_replay::loadConfig(cli.config);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: config '%s': %s\n", cli.config.c_str(), e.what());
    return 2;
  }
  try {
    return cli.dry_run ? runDry(cli, cfg) : runRobot(cli, cfg);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
