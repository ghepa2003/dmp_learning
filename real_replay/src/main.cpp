// real_replay: standalone libfranka executable running the Cartesian impedance controller in the
// 1 kHz torque callback. Modes: hold the activation pose, small smoothstep step, ProDMP replay.
//
//   real_replay --config <yaml> [--dry-run] [--duration S] [--mode hold] [--robot-ip IP]
//               [--urdf FILE] [--log-out FILE] [--no-pace] [--yes]
//
// Exit codes: 0 completed; 1 usage/config/preflight failure; 2 run stopped by the safety layer or
// by the user; 3 exception from libfranka or from the control code.

#include <sys/mman.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <franka/control_tools.h>
#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/lowpass_filter.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "real_replay/config.hpp"
#include "real_replay/controller_core.hpp"
#include "real_replay/logger.hpp"
#include "real_replay/path_check.hpp"
#include "real_replay/preflight_checks.hpp"
#include "real_replay/prodmp_source.hpp"
#include "real_replay/safety.hpp"
#include "real_replay/target_source.hpp"

namespace {

using namespace real_replay;
using JointVec = core::RobotModel::JointVector;
using SteadyClock = std::chrono::steady_clock;

volatile std::sig_atomic_t g_stop_requested = 0;
void onSignal(int) { g_stop_requested = 1; }

// ---------------------------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------------------------

struct Cli {
  std::string config;
  std::string mode = "hold";
  std::string robot_ip;
  std::string urdf;
  std::string log_out;
  double duration_s = -1.0;
  bool dry_run = false;
  bool no_pace = false;
  bool yes = false;
  bool check_joint_limits = false;
  bool preflight_only = false;
  bool mass_sensitivity = false;
  bool help = false;
};

void printUsage(const char* argv0) {
  std::printf(
      "Usage: %s --config <yaml> [options]\n"
      "  --dry-run          same loop, fixed RobotState, no robot connection\n"
      "  --duration S       override hold.duration_s (trajectory seconds)\n"
      "  --mode M           hold | step | step_rot | replay (ProDMP replay of replay.weights_path)\n"
      "  --check-joint-limits  offline joint-path check of the chosen mode (JointPathSimulator)\n"
      "  --preflight-only   connect, read the state (no control(), no setEE/setLoad/collision settings),\n"
      "                     print the preflight and exit\n"
      "  --mass-sensitivity damping design with mass/inertia of fer_link3..6 scaled by +15%% and -15%%\n"
      "                     (dry-run or --preflight-only)\n"
      "  --robot-ip IP      override robot.ip\n"
      "  --urdf FILE        override paths.urdf\n"
      "  --log-out FILE     CSV output (default: <log_dir>/<start time with ms>_<mode>_<dryrun|robot>.csv; never overwritten)\n"
      "  --no-pace          dry-run: do not sleep to a 1 kHz cadence\n"
      "  --yes              skip the 'press Enter' prompt (robot mode)\n",
      argv0);
}

bool parseCli(int argc, char** argv, Cli& cli, std::string& err) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto value = [&](std::string& dst) {
      if (i + 1 >= argc) {
        err = "missing value for " + a;
        return false;
      }
      dst = argv[++i];
      return true;
    };
    if (a == "-h" || a == "--help") {
      cli.help = true;
    } else if (a == "--dry-run") {
      cli.dry_run = true;
    } else if (a == "--no-pace") {
      cli.no_pace = true;
    } else if (a == "--yes") {
      cli.yes = true;
    } else if (a == "--mass-sensitivity") {
      cli.mass_sensitivity = true;
    } else if (a == "--preflight-only") {
      cli.preflight_only = true;
    } else if (a == "--check-joint-limits") {
      cli.check_joint_limits = true;
    } else if (a == "--config") {
      if (!value(cli.config)) return false;
    } else if (a == "--mode") {
      if (!value(cli.mode)) return false;
    } else if (a == "--robot-ip") {
      if (!value(cli.robot_ip)) return false;
    } else if (a == "--urdf") {
      if (!value(cli.urdf)) return false;
    } else if (a == "--log-out") {
      if (!value(cli.log_out)) return false;
    } else if (a == "--duration") {
      std::string v;
      if (!value(v)) return false;
      try {
        cli.duration_s = std::stod(v);
      } catch (const std::exception&) {
        err = "invalid number for --duration: " + v;
        return false;
      }
    } else {
      err = "unknown option: " + a;
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------------------------

std::string readFile(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open file '" + path + "'");
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

/// Local wall-clock time of the call, with milliseconds: YYYYmmdd_HHMMSS_mmm.
std::string timestampMs() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  const long ms = static_cast<long>(
      std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000);
  std::tm tmv{};
  localtime_r(&t, &tmv);
  char buf[48];
  std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d_%03ld", tmv.tm_year + 1900, tmv.tm_mon + 1,
                tmv.tm_mday, tmv.tm_hour, tmv.tm_min, tmv.tm_sec, ms);
  return buf;
}

struct Stats {
  std::size_t n = 0;
  double min = 0.0, mean = 0.0, p99 = 0.0, max = 0.0;
};

Stats computeStats(std::vector<double> v) {
  Stats s;
  if (v.empty()) return s;
  s.n = v.size();
  s.min = *std::min_element(v.begin(), v.end());
  s.max = *std::max_element(v.begin(), v.end());
  s.mean = std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
  const std::size_t k = static_cast<std::size_t>(std::ceil(0.99 * static_cast<double>(v.size()))) - 1;
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(k), v.end());
  s.p99 = v[k];
  return s;
}

void printStats(const char* label, const Stats& s, const char* unit) {
  std::printf("  %-28s min %.3f  mean %.3f  p99 %.3f  max %.3f %s  (n=%zu)\n", label, s.min, s.mean,
              s.p99, s.max, unit, s.n);
}

struct Report {
  int failures = 0;
  int warnings = 0;
  void check(const std::string& name, bool ok, const std::string& detail, bool fatal = true) {
    if (ok) {
      std::printf("  [PASS] %s%s%s\n", name.c_str(), detail.empty() ? "" : ": ", detail.c_str());
    } else if (fatal) {
      ++failures;
      std::printf("  [FAIL] %s%s%s\n", name.c_str(), detail.empty() ? "" : ": ", detail.c_str());
    } else {
      ++warnings;
      std::printf("  [WARN] %s%s%s\n", name.c_str(), detail.empty() ? "" : ": ", detail.c_str());
    }
  }
};

std::string fmt(const char* f, ...) __attribute__((format(printf, 1, 2)));
std::string fmt(const char* f, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof(buf), f, ap);
  va_end(ap);
  return buf;
}

// ---------------------------------------------------------------------------------------------
// libfranka provenance: print which libfranka*.so is mapped in this process
// ---------------------------------------------------------------------------------------------

std::vector<std::string> loadedFrankaLibs() {
  std::ifstream maps("/proc/self/maps");
  std::set<std::string> libs;
  std::string line;
  while (std::getline(maps, line)) {
    if (line.find("libfranka") == std::string::npos) continue;
    const auto pos = line.find('/');
    if (pos == std::string::npos) continue;
    libs.insert(line.substr(pos));
  }
  return std::vector<std::string>(libs.begin(), libs.end());
}

void checkLibfranka(const Config& cfg, Report& report, bool fatal) {
  const auto libs = loadedFrankaLibs();
  std::printf("libfranka mapped in this process:\n");
  if (libs.empty()) {
    std::printf("  (none found in /proc/self/maps)\n");
    report.check("libfranka is the expected non-rt build", false, "no libfranka mapping found", fatal);
    return;
  }
  std::error_code ec;
  const std::filesystem::path expected = std::filesystem::canonical(cfg.runtime.expected_libfranka_dir, ec);
  bool ok = !ec;
  const std::string prefix = ok ? expected.string() + "/" : std::string();
  for (const auto& lib : libs) {
    std::error_code ec2;
    const std::filesystem::path canon = std::filesystem::canonical(lib, ec2);
    const std::string shown = ec2 ? lib : canon.string();
    std::printf("  %s\n", shown.c_str());
    if (ec2 || !ok || shown.rfind(prefix, 0) != 0) ok = false;
  }
  report.check("libfranka is the expected non-rt build",
               ok, "expected under '" + cfg.runtime.expected_libfranka_dir + "'", fatal);
}

// ---------------------------------------------------------------------------------------------
// Runtime flags (default off)
// ---------------------------------------------------------------------------------------------

void applyRuntimeFlags(const Config& cfg) {
  if (cfg.runtime.mlockall) {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
      std::printf("  [WARN] mlockall requested but failed: %s\n", std::strerror(errno));
    } else {
      std::printf("  mlockall(MCL_CURRENT|MCL_FUTURE): ok\n");
    }
  }
  if (cfg.runtime.sched_fifo) {
    std::string msg;
    if (!franka::setCurrentThreadToHighestSchedulerPriority(&msg)) {
      std::printf("  [WARN] SCHED_FIFO requested but failed: %s\n", msg.c_str());
    } else {
      std::printf("  SCHED_FIFO (highest priority) set on the control thread: ok\n");
    }
  }
}

// ---------------------------------------------------------------------------------------------
// Preflight
// ---------------------------------------------------------------------------------------------

std::unique_ptr<ITargetSource> makeSource(const Config& cfg, const std::string& mode, bool quiet);

JointVec toVec(const Array7& a) {
  JointVec v;
  for (int i = 0; i < 7; ++i) v(i) = a[static_cast<std::size_t>(i)];
  return v;
}

/// Checks that need only the model: joint margins, effective inertia, per-axis damping design.
/// `robot_mode`: a_i above preflight.fail_damping_per_sample is a FAIL (otherwise only a WARN).
void preflightModel(ReplayLoop& loop, const Config& cfg, const Array7& qa, Report& report, bool robot_mode,
                    bool moves) {
  core::RobotModel& m = loop.model();
  const JointVec q = toVec(qa);
  m.update(q, JointVec::Zero());

  // Joint limits with margin.
  bool ok = true;
  std::string detail;
  for (int i = 0; i < 7; ++i) {
    const double lo = m.jointLowerLimits()(i) + cfg.safety.joint_limit_margin_rad;
    const double hi = m.jointUpperLimits()(i) - cfg.safety.joint_limit_margin_rad;
    if (q(i) < lo || q(i) > hi) {
      ok = false;
      detail += fmt("joint%d q=%.4f outside [%.4f, %.4f]; ", i + 1, q(i), lo, hi);
    }
  }
  report.check("q within joint limits (with margin)", ok, detail);

  // Effective Cartesian inertia at the activation configuration.
  const core::RobotModel::Matrix6d L = m.reflectedCartesianInertia();
  std::printf("Effective inertia Lambda at the control frame '%s' (6x6, [linear; angular]):\n",
              cfg.robot.ee_frame.c_str());
  std::cout << std::fixed << std::setprecision(4) << L << std::defaultfloat << "\n";

  // Per-axis damping design (the loop applies it; see controller_core.hpp).
  const DampingDesign& dd = loop.dampingDesign();
  if (!loop.dampingPrepared()) {
    report.check("damping design available", false, "prepareDamping() was not called");
    return;
  }
  std::printf("Per-axis damping%s, dt = %.0f ms, targets zeta_trans %.2f / zeta_rot %.2f, cap a_i <= %.2f%s:\n",
              dd.overridden ? " (OVERRIDDEN by damping.override_d)" : "", kNominalCycleS * 1e3,
              cfg.damping.zeta_trans, cfg.damping.zeta_rot, cfg.damping.max_damping_per_sample,
              dd.path_known ? "" : "   [Lambda_min/max = Lambda_act: no joint path computed]");
  std::printf("  axis | K        | Lam_act   Lam_min   Lam_max   |  D_i        D_i(stop) |  zeta   zeta@Lmax |  a_i     b_i    | max|z|  g*(1)    g*(2)\n");
  const char* names[6] = {"x ", "y ", "z ", "rx", "ry", "rz"};
  bool warn = false, fail = false, gwarn = false, gfail = false;
  std::string wd, fd, gwd, gfd;
  for (std::size_t i = 0; i < 6; ++i) {
    std::printf("  %-4s | %8.2f | %8.4f  %8.4f  %8.4f  | %9.4f  %9.4f  | %6.3f  %6.3f  | %6.3f  %6.4f | %6.4f  %7.3f  %7.3f%s\n", names[i],
                dd.k[i], dd.lambda_act[i], dd.lambda_min[i], dd.lambda_max[i], dd.d[i], dd.d_stop[i], dd.zeta[i],
                dd.zeta_lmax[i], dd.a[i], dd.b[i], dd.rho[i], dd.g1[i], dd.g2[i], dd.capped[i] ? "  (capped)" : "");
    // relative tolerance: a_i sits exactly ON the cap when the cap binds, and floating point may land a hair above
    if (dd.a[i] > cfg.preflight.max_damping_per_sample * (1.0 + 1e-9)) {
      warn = true;
      wd += fmt("%s a=%.2f; ", names[i], dd.a[i]);
    }
    if (dd.a[i] > cfg.preflight.fail_damping_per_sample * (1.0 + 1e-9)) {
      fail = true;
      fd += fmt("%s a=%.2f; ", names[i], dd.a[i]);
    }
    if (dd.g1[i] < cfg.preflight.warn_gain_margin) {
      gwarn = true;
      gwd += fmt("%s g*=%.2f; ", names[i], dd.g1[i]);
    }
    if (dd.g1[i] < cfg.preflight.fail_gain_margin || dd.rho[i] >= cfg.preflight.fail_spectral_radius) {
      gfail = true;
      gfd += fmt("%s g*=%.2f max|z|=%.4f; ", names[i], dd.g1[i], dd.rho[i]);
    }
  }
  std::printf("  max|z|: roots of z^3 - 2z^2 + (1+a)z + (b-a) = 0, a = D dt/Lambda_min, b = K dt^2/Lambda_min.\n"
              "  g*(n): gain margin = largest g with max|z|<1 for (g a, g b), n cycles of delay (n=2: z^2(z-1)^2 + a(z-1) + b).\n"
              "  MODEL: RIGID double integrator, ONE cycle of delay between state and applied torque,\n"
              "         NO rotor inertia, NO filters (cutoff, torque low-pass, friction). It is a SCREENING\n"
              "         criterion, not a proof of stability of the real loop. zeta@Lmax: D fixed, Lambda = Lam_max.\n");
  // NOTE: the URDF has no rotor (armature) inertia, so Lambda is optimistic-low (pessimistic for a_i).
  if (!dd.path_known && moves) {
    std::printf("  [WARN] Lambda_min = Lambda_act: run with --check-joint-limits to use the minimum along the path\n");
  }
  report.check(fmt("a_i = D_i*dt/Lambda_min_i <= %.2f on all axes", cfg.preflight.max_damping_per_sample), !warn,
               wd, /*fatal=*/false);
  report.check(fmt("a_i <= %.2f on all axes (stability)", cfg.preflight.fail_damping_per_sample), !fail, fd,
               /*fatal=*/robot_mode);
  report.check(fmt("gain margin g*(1 cycle) >= %.2f on all axes (screening)", cfg.preflight.warn_gain_margin), !gwarn,
               gwd, /*fatal=*/false);
  report.check(fmt("g*(1 cycle) >= %.2f and max|z| < %.2f on all axes (screening)", cfg.preflight.fail_gain_margin,
                   cfg.preflight.fail_spectral_radius),
               !gfail, gfd, /*fatal=*/robot_mode);
}

/// Checks that need the robot.
/// `settings_applied`: false in --preflight-only (setEE/setLoad/collision settings were NOT sent, so
/// the state shows whatever the controller currently has).
void preflightRobot(franka::Robot& robot, const franka::RobotState& s, ReplayLoop& loop,
                    const Config& cfg, Report& report, bool moves, bool settings_applied) {
  report.check("robot_mode == Idle", s.robot_mode == franka::RobotMode::kIdle,
               fmt("robot_mode=%d", static_cast<int>(s.robot_mode)));
  report.check("m_ee <= limit (no hand configured in Desk)", s.m_ee <= cfg.preflight.m_ee_max_kg,
               fmt("m_ee=%.3f kg, limit %.3f", s.m_ee, cfg.preflight.m_ee_max_kg));
  report.check("m_load <= limit", s.m_load <= cfg.preflight.m_load_max_kg,
               fmt("m_load=%.3f kg, limit %.3f", s.m_load, cfg.preflight.m_load_max_kg));
  std::printf("  info: m_total=%.3f kg, F_T_EE translation=[%.4f %.4f %.4f] m\n", s.m_total,
              s.F_T_EE[12], s.F_T_EE[13], s.F_T_EE[14]);
  auto printMat = [](const char* name, const std::array<double, 16>& M) {
    std::printf("  %s (row-major view of the column-major 4x4):\n", name);
    for (int r = 0; r < 4; ++r) {
      std::printf("    % .5f % .5f % .5f % .5f\n", M[static_cast<std::size_t>(r)], M[static_cast<std::size_t>(4 + r)],
                  M[static_cast<std::size_t>(8 + r)], M[static_cast<std::size_t>(12 + r)]);
    }
  };
  printMat("NE_T_EE", s.NE_T_EE);
  printMat("F_T_EE ", s.F_T_EE);
  printMat("O_T_EE ", s.O_T_EE);
  if (cfg.end_effector.set_ee) {
    double dne = 0.0;
    for (std::size_t i = 0; i < 16; ++i) dne = std::max(dne, std::fabs(s.NE_T_EE[i] - cfg.end_effector.ne_t_ee[i]));
    report.check("NE_T_EE in the state == end_effector.ne_t_ee from the config", dne < 1e-9,
                 fmt("max |diff| %.3e%s", dne,
                     settings_applied ? "" : " (setEE NOT sent in this mode: the state shows the current setting)"),
                 /*fatal=*/settings_applied);
  }

  preflightModel(loop, cfg, s.q, report, /*robot_mode=*/true, moves);

  // FK of the model (control frame) against the flange pose reconstructed from the robot state.
  core::RobotModel& m = loop.model();
  m.update(toVec(s.q), JointVec::Zero());
  Eigen::Map<const Eigen::Matrix4d> O_T_EE(s.O_T_EE.data());
  Eigen::Map<const Eigen::Matrix4d> F_T_EE(s.F_T_EE.data());
  const Eigen::Matrix4d O_T_F = O_T_EE * F_T_EE.inverse();
  const Eigen::Vector3d p_state = O_T_F.block<3, 1>(0, 3);
  const Eigen::Matrix3d R_state = O_T_F.block<3, 3>(0, 0);
  const double dp = (p_state - m.eePosition()).norm();
  const Eigen::Matrix3d R_model = m.eeOrientation().toRotationMatrix();
  const double dang_deg = Eigen::AngleAxisd(R_model.transpose() * R_state).angle() * 180.0 / M_PI;
  report.check("model FK == O_T_EE * F_T_EE^-1 (position)", dp <= cfg.preflight.fk_position_tol_m,
               fmt("%.6f m (tol %.6f)", dp, cfg.preflight.fk_position_tol_m));
  report.check("model FK == O_T_EE * F_T_EE^-1 (orientation)", dang_deg <= cfg.preflight.fk_rotation_tol_deg,
               fmt("%.4f deg (tol %.4f)", dang_deg, cfg.preflight.fk_rotation_tol_deg));

  if (cfg.preflight.check_gravity) {
    franka::Model fm = robot.loadModel();
    const Array7 g_f = fm.gravity(s);
    const JointVec& g_p = m.gravity();
    Array7 g_u{};
    for (int i = 0; i < 7; ++i) g_u[static_cast<std::size_t>(i)] = g_p(i);

    std::printf("  gravity [Nm]: tau_J = measured joint torque (state), franka = franka::Model::gravity(state),\n"
                "                pinocchio = gravity of the URDF in the control code. max|dq| = %.4f rad/s\n",
                [&] { double v = 0; for (double x : s.dq) v = std::max(v, std::fabs(x)); return v; }());
    std::printf("    joint |  tau_J   | franka   | |franka-tau_J| | pinocchio | |franka-pin|\n");
    for (int i = 0; i < 7; ++i) {
      const std::size_t k = static_cast<std::size_t>(i);
      std::printf("    %d     | %8.3f | %8.3f | %8.3f      | %8.3f  | %8.3f\n", i + 1, s.tau_J[k], g_f[k],
                  std::fabs(g_f[k] - s.tau_J[k]), g_u[k], std::fabs(g_f[k] - g_u[k]));
    }

    // (1) Principal: Desk/robot side. Independent of the URDF.
    std::printf("  [what it checks] franka vs tau_J: Desk's mass/COM/payload settings and the robot's own gravity model\n"
                "                   agree with what the joint torque sensors measure at rest (does NOT involve the URDF).\n");
    const MeasuredGravityResult mr = checkGravityVsMeasured(g_f, s.tau_J, s.dq, cfg.preflight.gravity_vs_measured_tol_nm,
                                                            cfg.preflight.gravity_still_max_dq_rad_s);
    const std::string mdetail = fmt("max |franka - tau_J| %.3f Nm on joint%d (tol %.3f), max|dq| %.4f rad/s (at rest below %.3f)",
                                    mr.max_abs_diff, mr.worst_joint + 1, cfg.preflight.gravity_vs_measured_tol_nm,
                                    mr.max_abs_dq, cfg.preflight.gravity_still_max_dq_rad_s);
    if (mr.verdict == MeasuredGravityResult::Verdict::kWarnMoving) {
      report.check("gravity: franka model vs measured tau_J",
                   false, "robot is MOVING, so tau_J includes dynamic torque and the comparison is not valid: " + mdetail,
                   /*fatal=*/false);
    } else {
      report.check("gravity: franka model vs measured tau_J", mr.verdict == MeasuredGravityResult::Verdict::kPass, mdetail);
    }

    // (2) Informative: how good the URDF inertial parameters are.
    std::printf("  [what it checks] franka vs pinocchio: how far the URDF inertial parameters are from the robot's own model\n"
                "                   (informative; the control law uses the robot's gravity compensation, not the URDF's).\n");
    const UrdfGravityResult ur = checkGravityVsUrdf(g_f, g_u, cfg.preflight.gravity_urdf_warn_nm);
    report.check("gravity: franka model vs Pinocchio (URDF, informative)", !ur.warn,
                 fmt("max diff %.3f Nm on joint%d (warn above %.3f)", ur.max_abs_diff, ur.worst_joint + 1,
                     cfg.preflight.gravity_urdf_warn_nm),
                 /*fatal=*/false);

    // Coriolis at the measured dq (franka::Model includes m_total/F_x_Ctotal/I_total, Pinocchio is arm only).
    m.update(toVec(s.q), toVec(s.dq));
    const std::array<double, 7> c_f = fm.coriolis(s);
    const JointVec& c_p = m.coriolis();
    double cworst = 0.0, dqmax = 0.0;
    for (int i = 0; i < 7; ++i) {
      cworst = std::max(cworst, std::fabs(c_f[static_cast<std::size_t>(i)] - c_p(i)));
      dqmax = std::max(dqmax, std::fabs(s.dq[static_cast<std::size_t>(i)]));
    }
    std::printf("  [what it checks] Coriolis franka vs pinocchio: same as above for the velocity-dependent term (weak at rest).\n");
    report.check("Coriolis: franka model vs Pinocchio (URDF, informative)", cworst <= cfg.preflight.gravity_urdf_warn_nm,
                 fmt("max diff %.4f Nm at max|dq| %.4f rad/s (warn above %.3f)", cworst, dqmax,
                     cfg.preflight.gravity_urdf_warn_nm),
                 /*fatal=*/false);
  }
}

// ---------------------------------------------------------------------------------------------
// Sensitivity of the damping design to the URDF inertial parameters
// ---------------------------------------------------------------------------------------------

/// Re-computes Lambda with the mass (and inertia) of fer_link3..6 scaled by +15 % and -15 % and prints, per axis,
/// D_i, zeta_i, g*(1) of (A) a design redone on the scaled model and (B) the NOMINAL D applied to the scaled
/// plant (the real situation if the URDF is wrong and D was designed from it).
void massSensitivity(const Cli& cli, const Config& cfg, const std::string& urdf_xml, const Array7& qa) {
  const std::vector<std::string> links = {"fer_link3", "fer_link4", "fer_link5", "fer_link6"};
  const double factors[3] = {1.0, 1.15, 0.85};
  const char* names[6] = {"x ", "y ", "z ", "rx", "ry", "rz"};
  std::printf("Mass sensitivity: mass and inertia of fer_link3..fer_link6 scaled; Lambda_min/max from %s.\n"
              "  A = design redone on the scaled model;  B = NOMINAL D (scale 1.00) on the scaled plant.\n"
              "  MODEL of g*: rigid double integrator, one cycle of delay, no rotor inertia, no filters (screening).\n",
              cli.check_joint_limits ? "the joint path of each scaled model" : "the activation configuration only");
  DampingDesign nominal;
  for (int vi = 0; vi < 3; ++vi) {
    const double f = factors[vi];
    const std::string urdf = f == 1.0 ? urdf_xml : scaleLinkMasses(urdf_xml, links, f);
    core::RobotModel mv(urdf, cfg.robot.joint_names, cfg.robot.ee_frame);
    mv.update(toVec(qa), JointVec::Zero());
    const core::RobotModel::Matrix6d L = mv.reflectedCartesianInertia();
    Array6 act{}, mn{}, mx{};
    for (std::size_t i = 0; i < 6; ++i) act[i] = mn[i] = mx[i] = L(static_cast<int>(i), static_cast<int>(i));
    bool path = false;
    if (cli.check_joint_limits) {
      auto src = makeSource(cfg, cli.mode, /*quiet=*/true);
      const PathCheckResult r = checkJointPath(cfg, urdf, *src, qa);
      mn = r.lambda_min;
      mx = r.lambda_max;
      path = true;
    }
    const DampingDesign d = designDamping(cfg.gains, cfg.damping, cfg.safety.stop_damping_scale, kNominalCycleS, act, mn, mx, path);
    if (vi == 0) nominal = d;
    std::printf("  masses x %.2f   axis | Lam_act  Lam_min  | A: D_i       zeta   g*(1)  | B: D_nom     zeta   g*(1)\n", f);
    for (std::size_t i = 0; i < 6; ++i) {
      const double lmin = d.lambda_min[i];
      const double a_b = nominal.d[i] * kNominalCycleS / lmin;
      const double b_b = d.k[i] * kNominalCycleS * kNominalCycleS / lmin;
      const double zeta_b = nominal.d[i] / (2.0 * std::sqrt(d.k[i] * act[i]));
      std::printf("                  %-4s | %8.4f %8.4f | %10.4f %6.3f %7.2f | %10.4f %6.3f %7.2f\n", names[i], act[i], lmin,
                  d.d[i], d.zeta[i], d.g1[i], nominal.d[i], zeta_b, gainMargin(a_b, b_b, 1));
    }
  }
}

// ---------------------------------------------------------------------------------------------
// One control cycle (shared by the robot callback and the dry-run)
// ---------------------------------------------------------------------------------------------

LoopInput toInput(const franka::RobotState& s, double period_s) {
  LoopInput in;
  in.q = s.q;
  in.dq = s.dq;
  in.tau_J = s.tau_J;
  in.period_s = period_s;
  in.success_rate = s.control_command_success_rate;
  return in;
}

franka::Torques runCycle(ReplayLoop& loop, const franka::RobotState& s, double period_s) {
  const auto t0 = SteadyClock::now();
  LoopOutput out;
  try {
    loop.step(toInput(s, period_s), out);
  } catch (const std::exception& e) {
    loop.markException(e.what());
    out.tau.fill(0.0);
    out.finished = true;
  }
  franka::Torques tq(out.tau);
  tq.motion_finished = out.finished;
  const auto t1 = SteadyClock::now();
  loop.setLastCallbackUs(std::chrono::duration<double, std::micro>(t1 - t0).count());
  return tq;
}

// ---------------------------------------------------------------------------------------------
// Reporting
// ---------------------------------------------------------------------------------------------

struct RunStats {
  Stats cb_us;
  Stats period_ms;
  double max_abs_tau = 0.0;
  double max_abs_e_pos = 0.0;
  double max_abs_e_rot = 0.0;
};

RunStats collectStats(const ReplayLoop& loop) {
  RunStats r;
  std::vector<double> cb;
  std::vector<double> per;
  cb.reserve(loop.logger().size());
  per.reserve(loop.logger().size());
  for (std::size_t i = 0; i < loop.logger().size(); ++i) {
    const LogRecord& rec = loop.logger().at(i);
    cb.push_back(rec.cb_us);
    if (i > 0) per.push_back(rec.period * 1e3);
    for (int j = 0; j < 7; ++j) r.max_abs_tau = std::max(r.max_abs_tau, std::fabs(rec.tau_cmd[j]));
    r.max_abs_e_pos = std::max(r.max_abs_e_pos, std::sqrt(rec.e_pos[0] * rec.e_pos[0] + rec.e_pos[1] * rec.e_pos[1] + rec.e_pos[2] * rec.e_pos[2]));
    r.max_abs_e_rot = std::max(r.max_abs_e_rot, std::sqrt(rec.e_rot[0] * rec.e_rot[0] + rec.e_rot[1] * rec.e_rot[1] + rec.e_rot[2] * rec.e_rot[2]));
  }
  r.cb_us = computeStats(std::move(cb));
  r.period_ms = computeStats(std::move(per));
  return r;
}

void printSummary(const ReplayLoop& loop, const RunStats& st) {
  std::printf("  cycles: %ld   logged: %zu   stop reason: %s\n", loop.cycles(), loop.logger().size(),
              toString(loop.stopReason()));
  if (!loop.exceptionMessage().empty()) std::printf("  exception: %s\n", loop.exceptionMessage().c_str());
  printStats("callback duration", st.cb_us, "us");
  std::printf("  max |tau_cmd| %.3f Nm   max |e_pos| %.6f m   max |e_rot| %.6f rad\n", st.max_abs_tau,
              st.max_abs_e_pos, st.max_abs_e_rot);
  std::printf("  cycles rate-limited: %ld   clamped: %ld\n", loop.rateLimitedCycles(), loop.clampedCycles());
}

/// <log_dir>/<start-of-run time with ms>_<mode>_<dryrun|robot>.csv (or --log-out). Throws if the file
/// already exists: a log is never overwritten.
std::string logPath(const Cli& cli, const Config& cfg, const char* kind) {
  std::filesystem::path p;
  if (!cli.log_out.empty()) {
    p = cli.log_out;
  } else {
    std::error_code ec;
    std::filesystem::create_directories(cfg.paths.log_dir, ec);
    p = std::filesystem::path(cfg.paths.log_dir) / (timestampMs() + "_" + cli.mode + "_" + kind + ".csv");
  }
  if (std::filesystem::exists(p)) throw std::runtime_error("log file already exists, refusing to overwrite: " + p.string());
  return p.string();
}

void writeLog(const ReplayLoop& loop, const std::string& path) {
  std::string err;
  if (loop.logger().writeCsv(path, &err)) {
    std::printf("log written: %s (%zu rows)\n", path.c_str(), loop.logger().size());
  } else {
    std::fprintf(stderr, "ERROR: log not written: %s\n", err.c_str());
  }
}

// ---------------------------------------------------------------------------------------------
// Target sources
// ---------------------------------------------------------------------------------------------

std::unique_ptr<ITargetSource> makeSource(const Config& cfg, const std::string& mode, bool quiet) {
  if (mode == "hold") return std::make_unique<HoldSource>(cfg.hold.duration_s);
  if (mode == "step") return std::make_unique<StepSource>(cfg.step);
  if (mode == "step_rot") return std::make_unique<StepSource>(cfg.step, /*rotational=*/true);
  if (mode == "replay") {
    auto src = std::make_unique<ProDmpSource>(cfg.replay.weights_path, cfg.replay.expected_sha256,
                                              cfg.trajectory.stop_at_fraction, cfg.replay.hold_after_end_s);
    if (!quiet) {
      std::printf("replay weights: %s\n  sha256 %s%s\n  tau %.6f s, ends at fraction %.3f (%.3f s), then holds %.2f s\n",
                  cfg.replay.weights_path.c_str(), src->sha256().c_str(),
                  cfg.replay.expected_sha256.empty() ? " (not checked: replay.expected_sha256 is empty)"
                                                     : " (matches replay.expected_sha256)",
                  src->tau(), cfg.trajectory.stop_at_fraction, src->endTime(), cfg.replay.hold_after_end_s);
    }
    return src;
  }
  throw std::runtime_error("unknown mode '" + mode + "' (hold | step | step_rot | replay)");
}

/// The run must fit in run.max_duration_s together with the controlled stop.
void checkDurations(const Config& cfg, double nominal_s) {
  if (nominal_s + cfg.safety.stop_duration_s >= cfg.run.max_duration_s) {
    throw std::runtime_error(fmt("source length %.1f s + stop %.1f s does not fit in run.max_duration_s = %.1f s",
                                 nominal_s, cfg.safety.stop_duration_s, cfg.run.max_duration_s));
  }
}

PathCheckResult runPathCheck(const Cli& cli, const Config& cfg, const std::string& urdf_xml, const Array7& q_act,
                             Report& report) {
  auto src = makeSource(cfg, cli.mode, /*quiet=*/true);  // separate instance: the check consumes it
  const PathCheckResult r = checkJointPath(cfg, urdf_xml, *src, q_act);
  std::printf("Joint-path check (JointPathSimulator, kinematic proxy of the impedance controller, %zu samples, %.1f s):\n",
              r.samples, r.duration_s);
  for (int j = 0; j < 7; ++j) {
    std::printf("  joint%d: min distance to nearest limit %.4f rad%s\n", j + 1, r.min_margin_rad[static_cast<std::size_t>(j)],
                j == r.worst_joint ? "   <- smallest" : "");
  }
  std::printf("  min translational manipulability %.5f\n", r.min_w_trans);
  report.check("joint limits along the path (margin " + fmt("%.3f", r.margin_rad) + " rad)", r.ok,
               fmt("worst joint%d, margin %.4f rad", r.worst_joint + 1,
                   r.worst_joint >= 0 ? r.min_margin_rad[static_cast<std::size_t>(r.worst_joint)] : 0.0));
  return r;
}

/// Damping design for the loop: Lambda extrema from the joint path when --check-joint-limits was
/// requested, Lambda_act otherwise. Must run before the control loop starts.
void prepareDampingForRun(const Cli& cli, const Config& cfg, const std::string& urdf_xml, const Array7& q_act,
                          ReplayLoop& loop, Report& report) {
  if (cli.check_joint_limits) {
    const PathCheckResult r = runPathCheck(cli, cfg, urdf_xml, q_act, report);
    loop.prepareDamping(q_act, &r.lambda_min, &r.lambda_max);
  } else {
    loop.prepareDamping(q_act, nullptr, nullptr);
  }
}

// ---------------------------------------------------------------------------------------------
// Dry-run
// ---------------------------------------------------------------------------------------------

int runDry(const Cli& cli, const Config& cfg, const std::string& urdf_xml) {
  ReplayLoop loop(cfg, urdf_xml, makeSource(cfg, cli.mode, /*quiet=*/false));
  const double nominal_s = loop.sourceNominalDurationS();
  checkDurations(cfg, nominal_s);
  loop.setExternalStopFlag(&g_stop_requested);
  const std::string csv_path = logPath(cli, cfg, "dryrun");  // name from the run start time
  // The state is fixed, so a moving target legitimately leaves it: do not let the tracking guard end the run.
  if (cfg.dry_run.ignore_tracking_error) loop.setTrackingGuardEnabled(false);

  std::printf("=== DRY-RUN (no robot connection) ===\n");
  Report report;
  checkLibfranka(cfg, report, /*fatal=*/false);
  applyRuntimeFlags(cfg);
  prepareDampingForRun(cli, cfg, urdf_xml, cfg.dry_run.q, loop, report);
  preflightModel(loop, cfg, cfg.dry_run.q, report, /*robot_mode=*/false, cli.mode != "hold");
  if (cli.mass_sensitivity) massSensitivity(cli, cfg, urdf_xml, cfg.dry_run.q);

  franka::RobotState state{};
  state.q = cfg.dry_run.q;
  state.control_command_success_rate = 1.0;

  const bool paced = cfg.dry_run.paced && !cli.no_pace;
  const double dt = kNominalCycleS;  // dry-run period fed to the loop (the robot reports 1e-3 s)
  const std::size_t max_cycles =
      static_cast<std::size_t>((cfg.run.max_duration_s + cfg.safety.stop_duration_s) / dt) + 100;
  std::vector<double> lateness_us;
  lateness_us.reserve(paced ? max_cycles : 0);

  std::printf("mode %s: running %.1f s of trajectory time, %s\n", cli.mode.c_str(), nominal_s,
              paced ? "paced at 1 kHz" : "unpaced (as fast as possible)");
  auto next = SteadyClock::now();
  for (std::size_t i = 0; i < max_cycles; ++i) {
    const franka::Torques tq = runCycle(loop, state, dt);
    if (tq.motion_finished) break;
    if (paced) {
      next += std::chrono::microseconds(1000);
      std::this_thread::sleep_until(next);
      lateness_us.push_back(
          std::chrono::duration<double, std::micro>(SteadyClock::now() - next).count());
    }
  }

  std::printf("=== dry-run results ===\n");
  const RunStats st = collectStats(loop);
  printSummary(loop, st);
  if (paced) printStats("wake-up lateness", computeStats(lateness_us), "us");

  report.check("no NaN/inf in the log", loop.logger().allFinite(), "");
  bool target_ok = loop.hasFirstTarget();
  std::string detail = "no first target";
  if (target_ok) {
    const double dp = (loop.firstTarget().p - loop.activationPose().p).norm();
    const double da = loop.firstTarget().q.angularDistance(loop.activationPose().q);
    target_ok = dp < 1e-9 && da < 1e-9;
    detail = fmt("|dp|=%.3e m, angle=%.3e rad", dp, da);
  }
  report.check("target_0 == activation pose", target_ok, detail);
  report.check("run completed normally", loop.stopReason() == StopReason::kCompleted,
               toString(loop.stopReason()));
  const double expected = (nominal_s + cfg.safety.stop_duration_s) / dt;
  report.check("cycle count as expected", std::fabs(static_cast<double>(loop.cycles()) - expected) <= 5.0,
               fmt("%ld cycles, expected ~%.0f", loop.cycles(), expected));
  report.check("callback p99 below budget", st.cb_us.p99 < cfg.safety.callback_budget_us,
               fmt("p99 %.1f us, budget %.1f us", st.cb_us.p99, cfg.safety.callback_budget_us));
  if (cli.mode == "hold") {
    report.check("safety layer never intervened", loop.rateLimitedCycles() == 0 && loop.clampedCycles() == 0,
                 fmt("%ld rate-limited, %ld clamped", loop.rateLimitedCycles(), loop.clampedCycles()));
  } else {
    // The fixed state makes the error grow with the target, so torque limiting here is expected.
    std::printf("  info: %ld cycles rate-limited, %ld clamped (expected with a fixed state and a moving target)\n",
                loop.rateLimitedCycles(), loop.clampedCycles());
  }

  writeLog(loop, csv_path);
  std::printf("dry-run %s (%d failure(s), %d warning(s))\n", report.failures == 0 ? "OK" : "FAILED",
              report.failures, report.warnings);
  return report.failures == 0 ? 0 : 1;
}

// ---------------------------------------------------------------------------------------------
// Preflight only: read the state, never start control()
// ---------------------------------------------------------------------------------------------

int runPreflightOnly(const Cli& cli, const Config& cfg, const std::string& urdf_xml) {
  ReplayLoop loop(cfg, urdf_xml, makeSource(cfg, cli.mode, /*quiet=*/false));
  std::printf("=== PREFLIGHT ONLY: %s, mode %s (no control(), no setEE/setLoad/setCollisionBehavior) ===\n",
              cfg.robot.ip.c_str(), cli.mode.c_str());
  Report report;
  checkLibfranka(cfg, report, /*fatal=*/true);
  if (report.failures > 0 && cfg.preflight.abort_on_failure) {
    std::printf("preflight failed, not connecting.\n");
    return 1;
  }
  try {
    franka::Robot robot(cfg.robot.ip, franka::RealtimeConfig::kIgnore);
    const franka::RobotState s0 = robot.readOnce();
    std::printf("preflight:\n");
    prepareDampingForRun(cli, cfg, urdf_xml, s0.q, loop, report);
    preflightRobot(robot, s0, loop, cfg, report, cli.mode != "hold", /*settings_applied=*/false);
    if (cli.mass_sensitivity) massSensitivity(cli, cfg, urdf_xml, s0.q);
    std::printf(
        "NOT checked in this mode (they need setEE/setLoad/setCollisionBehavior, i.e. a write to the controller):\n"
        "  - the load actually in effect during a run: m_load, F_T_EE and the gravity/Coriolis comparison reflect the\n"
        "    CURRENT controller setting (Desk, or a previous setEE/setLoad if it persists: not documented), not the\n"
        "    end_effector.* values of the config;\n"
        "  - collision thresholds and joint/Cartesian impedance of the config (not sent).\n");
  } catch (const franka::Exception& e) {
    std::fprintf(stderr, "franka::Exception: %s\n", e.what());
    return 3;
  }
  std::printf("preflight-only %s (%d failure(s), %d warning(s))\n", report.failures == 0 ? "OK" : "FAILED",
              report.failures, report.warnings);
  return report.failures == 0 ? 0 : 1;
}

// ---------------------------------------------------------------------------------------------
// Robot run
// ---------------------------------------------------------------------------------------------

int runRobot(const Cli& cli, const Config& cfg, const std::string& urdf_xml) {
  ReplayLoop loop(cfg, urdf_xml, makeSource(cfg, cli.mode, /*quiet=*/false));
  const double nominal_s = loop.sourceNominalDurationS();
  checkDurations(cfg, nominal_s);
  loop.setExternalStopFlag(&g_stop_requested);
  const std::string csv_path = logPath(cli, cfg, "robot");
  int exit_code = 0;

  std::printf("=== ROBOT RUN: %s, mode %s, %.1f s ===\n", cfg.robot.ip.c_str(), cli.mode.c_str(), nominal_s);
  Report report;
  checkLibfranka(cfg, report, /*fatal=*/true);
  if (report.failures > 0 && cfg.preflight.abort_on_failure) {
    std::printf("preflight failed, not connecting.\n");
    return 1;
  }

  try {
    // Explicit kIgnore: the non-rt build already defaults to it, but this must not depend on defaults.
    franka::Robot robot(cfg.robot.ip, franka::RealtimeConfig::kIgnore);

    // Settings go BEFORE the control loop (libfranka examples: "NEVER in the control loop").
    robot.setCollisionBehavior(cfg.collision.lower_torque_acc, cfg.collision.upper_torque_acc,
                               cfg.collision.lower_torque_nom, cfg.collision.upper_torque_nom,
                               cfg.collision.lower_force_acc, cfg.collision.upper_force_acc,
                               cfg.collision.lower_force_nom, cfg.collision.upper_force_nom);
    robot.setJointImpedance(cfg.collision.joint_impedance);
    robot.setCartesianImpedance(cfg.collision.cartesian_impedance);
    if (cfg.end_effector.set_ee) robot.setEE(cfg.end_effector.ne_t_ee);
    if (cfg.end_effector.set_load) {
      robot.setLoad(cfg.end_effector.load_mass, cfg.end_effector.f_x_cload, cfg.end_effector.load_inertia);
    }

    const franka::RobotState s0 = robot.readOnce();
    std::printf("preflight:\n");
    prepareDampingForRun(cli, cfg, urdf_xml, s0.q, loop, report);
    preflightRobot(robot, s0, loop, cfg, report, cli.mode != "hold", /*settings_applied=*/true);
    if (report.failures > 0) {
      std::printf("preflight: %d failure(s)%s\n", report.failures,
                  cfg.preflight.abort_on_failure ? " -> aborting before control" : " (continuing: abort_on_failure=false)");
      if (cfg.preflight.abort_on_failure) return 1;
    }

    if (!cli.yes) {
      std::printf("\nWARNING: the robot will be torque-controlled. Keep the user stop button at hand.\n"
                  "Press Enter to start (Ctrl+C to abort)...\n");
      std::cin.get();
      if (g_stop_requested) {
        std::printf("aborted by user before start.\n");
        return 2;
      }
    }

    applyRuntimeFlags(cfg);

    std::function<franka::Torques(const franka::RobotState&, franka::Duration)> callback =
        [&loop](const franka::RobotState& s, franka::Duration d) { return runCycle(loop, s, d.toSec()); };
    robot.control(callback, /*limit_rate=*/true, franka::kMaxCutoffFrequency);
  } catch (const franka::ControlException& e) {
    std::fprintf(stderr, "franka::ControlException: %s (%zu recorded states)\n", e.what(), e.log.size());
    loop.markException(e.what());
    exit_code = 3;
  } catch (const franka::Exception& e) {
    std::fprintf(stderr, "franka::Exception: %s\n", e.what());
    loop.markException(e.what());
    exit_code = 3;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "exception: %s\n", e.what());
    loop.markException(e.what());
    exit_code = 3;
  }

  std::printf("=== run results ===\n");
  const RunStats st = collectStats(loop);
  printSummary(loop, st);
  printStats("period", st.period_ms, "ms");
  writeLog(loop, csv_path);

  if (exit_code != 0) return exit_code;
  return loop.stopReason() == StopReason::kCompleted ? 0 : 2;
}

}  // namespace

int main(int argc, char** argv) {
  Cli cli;
  std::string err;
  if (!parseCli(argc, argv, cli, err)) {
    std::fprintf(stderr, "error: %s\n", err.c_str());
    printUsage(argv[0]);
    return 1;
  }
  if (cli.help || cli.config.empty()) {
    printUsage(argv[0]);
    return cli.help ? 0 : 1;
  }
  if (cli.mode != "hold" && cli.mode != "step" && cli.mode != "step_rot" && cli.mode != "replay") {
    std::fprintf(stderr, "error: unknown mode '%s' (hold | step | step_rot | replay)\n", cli.mode.c_str());
    return 1;
  }

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  try {
    Config cfg = loadConfig(cli.config);
    if (!cli.robot_ip.empty()) cfg.robot.ip = cli.robot_ip;
    if (!cli.urdf.empty()) cfg.paths.urdf = cli.urdf;
    if (cli.duration_s > 0.0) {
      if (cli.mode == "hold") {
        cfg.hold.duration_s = cli.duration_s;
      } else {
        std::fprintf(stderr, "warning: --duration only applies to --mode hold; ignored\n");
      }
    }
    const std::string urdf_xml = readFile(cfg.paths.urdf);
    std::printf("config: %s\nurdf:   %s (control frame '%s')\n", cfg.config_path.c_str(),
                cfg.paths.urdf.c_str(), cfg.robot.ee_frame.c_str());
    if (cli.preflight_only && cli.dry_run) throw std::runtime_error("--preflight-only and --dry-run are mutually exclusive");
    if (cli.preflight_only) return runPreflightOnly(cli, cfg, urdf_xml);
    return cli.dry_run ? runDry(cli, cfg, urdf_xml) : runRobot(cli, cfg, urdf_xml);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
