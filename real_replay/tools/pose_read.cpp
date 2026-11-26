// pose_read: READ-ONLY probe of robot 1 (and optionally robot 2). Only franka::Robot::readOnce(): no control loop, no
// torque / position / velocity command, no setter (no setEE, setLoad, setCollisionBehavior...), no gripper.
//
// usage:
//   pose_read --hw-config hw/hw_config_j1ns1.yaml [--robot1-ip IP] [--samples N] [--period-ms M] [--log-dir D]
//             [--theta-from-robot2 [--robot2-ip IP]] [--phase-source q7|flange]
//             [--grasp-command F] [--planner-config F]
//   pose_read --hw-config F --fk q1,...,q7 [deg] [--f-t-ee 16 numbers, column-major] [--grasp-command F]   (offline, no robot)
//
// What it prints (robot 1, frame = base of robot 1): robot_mode, current_errors / last_motion_errors, q [rad, deg],
// O_T_EE position + quaternion (the TCP of the controller when NE_T_EE is the identity, as in the task configs),
// the flange from O_T_EE * F_T_EE^-1 against the Panda DH FK of q (same FK as satellite_node), O_F_ext_hat_K,
// tau_ext_hat_filtered, F_T_EE / NE_T_EE / m_ee / m_load as read. With --samples N: mean and std over N reads and a CSV
// (+ .meta.txt) in the log dir.
//
// --theta-from-robot2: reads robot 2 once per sample and computes, with the SAME functions as satellite_node
// (sat::computeSatGeom / phaseFromQ7, parameters from hw_config scene.*), theta_q7, theta_flange and the FK centre.
// With --grasp-command: the nominal grasp pose for that theta is the command's goal pose (planned at theta*) rotated
// rigidly about the configured axis through the configured centre (scene.center_m, scene.axis_tilt_deg) by
// theta - theta* (the same rigid rotation the FOLLOW target uses). The deviation read - nominal is printed as radial,
// tangential, vertical components and an orientation angle. --planner-config only adds the planner's rim
// (cube_side_m/2 + standoff_m) for comparison with the radial distances; the nominal pose does not use it.
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "real_replay/git_version.hpp"
#include "real_replay/hw_config.hpp"
#include "real_replay/satellite_phase.hpp"
#include "real_replay/task_logic.hpp"

namespace sat = real_replay::sat;
using Eigen::Isometry3d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

namespace {
constexpr double kDeg = M_PI / 180.0;

struct Cli {
  std::string hw_config, robot1_ip, robot2_ip, log_dir, grasp_command, planner_config, phase_source = "q7";
  int samples = 1, period_ms = 100;
  bool theta = false, fk_only = false, have_ftee = false;
  std::array<double, 7> fk_q{};
  std::array<double, 16> f_t_ee{};
};

bool parseList(const std::string& s, double* out, int n) {
  std::stringstream ss(s);
  std::string tok;
  int k = 0;
  while (std::getline(ss, tok, ',')) {
    if (k >= n) return false;
    char* end = nullptr;
    out[k++] = std::strtod(tok.c_str(), &end);
    if (end == tok.c_str()) return false;
  }
  return k == n;
}

void usage(const char* a0) {
  std::fprintf(stderr,
               "usage: %s --hw-config F [--robot1-ip IP] [--samples N] [--period-ms M] [--log-dir D]\n"
               "          [--theta-from-robot2 [--robot2-ip IP]] [--phase-source q7|flange] [--grasp-command F] [--planner-config F]\n"
               "       %s --hw-config F --fk q1,..,q7(deg) [--f-t-ee m0,..,m15 (column-major)] [--grasp-command F]   (offline)\n"
               "READ ONLY: readOnce() only, no control, no setter, no gripper.\n",
               a0, a0);
}

bool parseCli(int argc, char** argv, Cli& c) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto val = [&]() -> std::string {
      if (i + 1 >= argc) throw std::runtime_error("missing value after " + a);
      return argv[++i];
    };
    if (a == "--hw-config") c.hw_config = val();
    else if (a == "--robot1-ip") c.robot1_ip = val();
    else if (a == "--robot2-ip") c.robot2_ip = val();
    else if (a == "--samples") c.samples = std::atoi(val().c_str());
    else if (a == "--period-ms") c.period_ms = std::atoi(val().c_str());
    else if (a == "--log-dir") c.log_dir = val();
    else if (a == "--theta-from-robot2") c.theta = true;
    else if (a == "--phase-source") c.phase_source = val();
    else if (a == "--grasp-command") c.grasp_command = val();
    else if (a == "--planner-config") c.planner_config = val();
    else if (a == "--fk") {
      double d[7];
      if (!parseList(val(), d, 7)) throw std::runtime_error("--fk needs 7 comma-separated angles in degrees");
      for (int k = 0; k < 7; ++k) c.fk_q[k] = d[k] * kDeg;
      c.fk_only = true;
    } else if (a == "--f-t-ee") {
      if (!parseList(val(), c.f_t_ee.data(), 16)) throw std::runtime_error("--f-t-ee needs 16 comma-separated numbers (column-major)");
      c.have_ftee = true;
    } else if (a == "-h" || a == "--help") {
      return false;
    } else {
      throw std::runtime_error("unknown argument " + a);
    }
  }
  if (c.hw_config.empty()) throw std::runtime_error("--hw-config is required (centre, axis, phase parameters, IPs, log dir)");
  if (c.samples < 1 || c.samples > 100000) throw std::runtime_error("--samples in [1, 100000]");
  if (c.period_ms < 1 || c.period_ms > 60000) throw std::runtime_error("--period-ms in [1, 60000]");
  if (c.phase_source != "q7" && c.phase_source != "flange") throw std::runtime_error("--phase-source q7|flange");
  return true;
}

Isometry3d fromColMajor(const std::array<double, 16>& m) {
  Eigen::Matrix4d M;
  for (int r = 0; r < 4; ++r)
    for (int col = 0; col < 4; ++col) M(r, col) = m[static_cast<std::size_t>(col * 4 + r)];
  Isometry3d T;
  T.matrix() = M;
  return T;
}

std::string fmtQuat(const Quaterniond& q) {
  char b[128];
  std::snprintf(b, sizeof b, "[w %.6f x %.6f y %.6f z %.6f]", q.w(), q.x(), q.y(), q.z());
  return b;
}

const char* modeName(franka::RobotMode m) {
  switch (m) {
    case franka::RobotMode::kOther: return "Other";
    case franka::RobotMode::kIdle: return "Idle";
    case franka::RobotMode::kMove: return "Move";
    case franka::RobotMode::kGuiding: return "Guiding";
    case franka::RobotMode::kReflex: return "Reflex";
    case franka::RobotMode::kUserStopped: return "UserStopped";
    case franka::RobotMode::kAutomaticErrorRecovery: return "AutomaticErrorRecovery";
  }
  return "?";
}

/// Mean of unit quaternions (sign-aligned to the first, normalised sum): fine for the small spread of a static read.
Quaterniond meanQuat(const std::vector<Quaterniond>& v) {
  Eigen::Vector4d s = Eigen::Vector4d::Zero();
  for (const Quaterniond& q : v) {
    Eigen::Vector4d c = q.coeffs();
    if (c.dot(v.front().coeffs()) < 0.0) c = -c;
    s += c;
  }
  Quaterniond m;
  m.coeffs() = s.normalized();
  return m;
}

struct Nominal {
  bool ok = false;
  double theta_star = NAN, theta_used = NAN;
  Vector3d c, axis, p;
  Quaterniond q;
};

Nominal nominalPose(const real_replay::task::GraspCommand& cmd, const real_replay::HwConfig& hw, double theta) {
  Nominal n;
  n.c = Vector3d(hw.scene.center_m[0], hw.scene.center_m[1], hw.scene.center_m[2]);
  n.axis = sat::axisFromTilt(hw.scene.tilt_x_rad, hw.scene.tilt_y_rad);
  n.theta_star = cmd.theta_star;
  n.theta_used = theta;
  const double d = sat::wrapPi(theta - cmd.theta_star);
  const Eigen::AngleAxisd R(d, n.axis);
  n.p = n.c + R * (cmd.goal_p - n.c);
  n.q = (Quaterniond(R) * cmd.goal_q).normalized();
  n.ok = n.p.allFinite() && n.q.coeffs().allFinite();
  return n;
}

void printDeviation(const char* what, const Vector3d& p, const Quaterniond& q, const Nominal& n, double rim) {
  const Vector3d dp = p - n.p;
  Vector3d r = (n.p - n.c) - (n.p - n.c).dot(n.axis) * n.axis;  // radial direction at the nominal point
  const double rn = r.norm();
  r /= rn;
  const Vector3d t = n.axis.cross(r);
  const Quaterniond dq = n.q.inverse() * q;  // rotation from nominal to read, in the nominal TCP frame
  const Eigen::AngleAxisd aa(dq);
  const Vector3d rv = aa.angle() * aa.axis();
  const Vector3d pr = (p - n.c) - (p - n.c).dot(n.axis) * n.axis;
  std::printf("  %s vs nominal at theta %.3f deg (theta* %.3f deg, rotation %+.3f deg about the axis through c):\n", what,
              n.theta_used / kDeg, n.theta_star / kDeg, sat::wrapPi(n.theta_used - n.theta_star) / kDeg);
  std::printf("    nominal p [%.4f %.4f %.4f] m  q %s\n", n.p.x(), n.p.y(), n.p.z(), fmtQuat(n.q).c_str());
  std::printf("    deviation: radial %+.2f mm, tangential %+.2f mm (+ = direction of +theta), vertical %+.2f mm, |dp| %.2f mm\n",
              dp.dot(r) * 1e3, dp.dot(t) * 1e3, dp.dot(n.axis) * 1e3, dp.norm() * 1e3);
  std::printf("    orientation: angle %.3f deg; rotation vector in the nominal TCP frame [%+.3f %+.3f %+.3f] deg\n",
              q.angularDistance(n.q) / kDeg, rv.x() / kDeg, rv.y() / kDeg, rv.z() / kDeg);
  std::printf("    distance from the axis: read %.4f m, nominal %.4f m", pr.norm(), rn);
  if (std::isfinite(rim)) std::printf(", planner rim (cube_side/2 + standoff) %.4f m", rim);
  std::printf("\n");
}

double plannerRim(const std::string& path) {
  if (path.empty()) return NAN;
  const YAML::Node r = YAML::LoadFile(path);
  const YAML::Node p = r["grasp_planner_node"]["ros__parameters"];
  if (!p || !p["cube_geometry.cube_side_m"] || !p["cube_geometry.standoff_m"]) {
    throw std::runtime_error(path + ": grasp_planner_node.ros__parameters.cube_geometry.{cube_side_m, standoff_m} not found");
  }
  return 0.5 * p["cube_geometry.cube_side_m"].as<double>() + p["cube_geometry.standoff_m"].as<double>();
}

std::string nowStamp() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t tt = std::chrono::system_clock::to_time_t(now);
  const long ms = static_cast<long>(std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000);
  std::tm tm{};
  localtime_r(&tt, &tm);
  char b[64];
  std::snprintf(b, sizeof b, "%04d%02d%02d_%02d%02d%02d_%03ld", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ms);
  return b;
}

// ---- offline FK check ---------------------------------------------------------------------------------------------
int runFk(const Cli& c, const real_replay::HwConfig& hw) {
  sat::Q7 q{};
  for (int k = 0; k < 7; ++k) q[static_cast<std::size_t>(k)] = c.fk_q[static_cast<std::size_t>(k)];
  const Isometry3d F = sat::pandaFlangeFk(q);
  const Quaterniond qf(F.linear());
  std::printf("FK (Panda DH, same as satellite_node) of q = [");
  for (int k = 0; k < 7; ++k) std::printf("%s%.4f", k ? ", " : "", c.fk_q[static_cast<std::size_t>(k)] / kDeg);
  std::printf("] deg\n  flange p [%.6f %.6f %.6f] m  q %s\n", F.translation().x(), F.translation().y(), F.translation().z(), fmtQuat(qf).c_str());
  if (!c.have_ftee) {
    std::printf("  (no --f-t-ee: the TCP needs F_T_EE as configured in Desk; read it with pose_read on the robot)\n");
    return 0;
  }
  const Isometry3d T = F * fromColMajor(c.f_t_ee);
  const Quaterniond qt(T.linear());
  std::printf("  TCP = flange * F_T_EE: p [%.6f %.6f %.6f] m  q %s\n", T.translation().x(), T.translation().y(), T.translation().z(),
              fmtQuat(qt).c_str());
  if (!c.grasp_command.empty()) {
    const real_replay::task::GraspCommand cmd = real_replay::task::loadGraspCommand(c.grasp_command);
    std::printf("  vs command start_pose: |dp| %.4f mm, angle %.4f deg\n", (T.translation() - cmd.start_p).norm() * 1e3,
                qt.angularDistance(cmd.start_q.normalized()) / kDeg);
  }
  (void)hw;
  return 0;
}

std::string robot1Ip(const Cli& c) {
  if (!c.robot1_ip.empty()) return c.robot1_ip;
  const YAML::Node r = YAML::LoadFile(c.hw_config);
  if (!r["robot1"] || !r["robot1"]["ip"]) throw std::runtime_error("robot1.ip not in " + c.hw_config + " (or pass --robot1-ip)");
  return r["robot1"]["ip"].as<std::string>();
}
}  // namespace

int main(int argc, char** argv) {
  Cli c;
  try {
    if (!parseCli(argc, argv, c)) {
      usage(argv[0]);
      return 0;
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    usage(argv[0]);
    return 2;
  }
  try {
    const real_replay::HwConfig hw = real_replay::loadHwConfig(c.hw_config);
    if (c.fk_only) return runFk(c, hw);

    sat::PhaseParams ph;  // as satellite_node phaseParams()
    ph.collar_offset_rad = hw.scene.collar_phase_offset_rad;
    ph.q7_offset_rad = hw.scene.q7_phase_offset_rad;
    ph.q7_sign = hw.scene.q7_phase_sign;
    ph.flange_to_center_m = hw.scene.flange_to_center_m;
    ph.base2_in_base1 = sat::BaseTransform::fromYaw(hw.scene.base2_t_m, hw.scene.base2_yaw_rad);
    real_replay::task::GraspCommand cmd;
    const bool have_cmd = !c.grasp_command.empty();
    if (have_cmd) cmd = real_replay::task::loadGraspCommand(c.grasp_command);
    const double rim = plannerRim(c.planner_config);

    const std::string ip1 = robot1Ip(c);
    const std::string ip2 = c.robot2_ip.empty() ? hw.robot2.ip : c.robot2_ip;
    std::printf("pose_read: READ ONLY (readOnce), robot 1 %s%s, %d sample(s) every %d ms\n", ip1.c_str(),
                c.theta ? (", robot 2 " + ip2).c_str() : "", c.samples, c.period_ms);
    franka::Robot r1(ip1, franka::RealtimeConfig::kIgnore);
    std::unique_ptr<franka::Robot> r2;
    if (c.theta) r2 = std::make_unique<franka::Robot>(ip2, franka::RealtimeConfig::kIgnore);

    const std::string log_dir = c.log_dir.empty() ? hw.log_dir : c.log_dir;
    const std::string base = log_dir + "/" + nowStamp() + "_pose_read";
    std::FILE* csv = nullptr;
    if (c.samples > 1) {
      csv = std::fopen((base + ".csv").c_str(), "wx");
      if (!csv) throw std::runtime_error("cannot create " + base + ".csv");
      std::fprintf(csv, "t_s,robot_mode,q1,q2,q3,q4,q5,q6,q7,px,py,pz,qw,qx,qy,qz,fx,fy,fz,mx,my,mz,theta_q7_deg,theta_flange_deg,q7_robot2\n");
    }

    std::vector<std::array<double, 7>> qs;
    std::vector<Vector3d> ps;
    std::vector<Quaterniond> qq;
    std::vector<std::array<double, 6>> fs;
    std::vector<double> th_q7, th_fl;
    franka::RobotState s1, s2;
    sat::SatGeom g{};
    const auto t0 = std::chrono::steady_clock::now();
    int mode_changes = 0;
    franka::RobotMode mode0 = franka::RobotMode::kOther;
    for (int i = 0; i < c.samples; ++i) {
      if (i > 0) std::this_thread::sleep_for(std::chrono::milliseconds(c.period_ms));
      s1 = r1.readOnce();
      if (i == 0) mode0 = s1.robot_mode;
      if (s1.robot_mode != mode0) ++mode_changes;
      const Isometry3d T = fromColMajor(s1.O_T_EE);
      qs.push_back(s1.q);
      ps.push_back(T.translation());
      qq.push_back(Quaterniond(T.linear()).normalized());
      fs.push_back(s1.O_F_ext_hat_K);
      double q72 = NAN;
      if (r2) {
        s2 = r2->readOnce();
        sat::Q7 q2{};
        for (std::size_t k = 0; k < 7; ++k) q2[k] = s2.q[k];
        g = sat::computeSatGeom(ph, q2);
        th_q7.push_back(g.theta_q7);
        th_fl.push_back(g.theta_flange);
        q72 = s2.q[6];
      }
      if (csv) {
        const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        std::fprintf(csv, "%.4f,%d", t, static_cast<int>(s1.robot_mode));
        for (double v : s1.q) std::fprintf(csv, ",%.9g", v);
        std::fprintf(csv, ",%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g", ps.back().x(), ps.back().y(), ps.back().z(), qq.back().w(), qq.back().x(),
                     qq.back().y(), qq.back().z());
        for (double v : s1.O_F_ext_hat_K) std::fprintf(csv, ",%.6g", v);
        std::fprintf(csv, ",%.6f,%.6f,%.9g\n", r2 ? g.theta_q7 / kDeg : NAN, r2 ? g.theta_flange / kDeg : NAN, q72);
      }
    }
    const double n = static_cast<double>(c.samples);

    // ---- robot 1
    std::array<double, 7> qm{}, qsd{};
    for (const auto& q : qs)
      for (std::size_t k = 0; k < 7; ++k) qm[k] += q[k] / n;
    for (const auto& q : qs)
      for (std::size_t k = 0; k < 7; ++k) qsd[k] += (q[k] - qm[k]) * (q[k] - qm[k]) / n;
    Vector3d pm = Vector3d::Zero();
    for (const Vector3d& p : ps) pm += p / n;
    double psd = 0.0;
    for (const Vector3d& p : ps) psd += (p - pm).squaredNorm() / n;
    const Quaterniond qmean = meanQuat(qq);
    double qspread = 0.0;
    for (const Quaterniond& q : qq) qspread = std::max(qspread, q.angularDistance(qmean));
    std::array<double, 6> fm{};
    for (const auto& f : fs)
      for (std::size_t k = 0; k < 6; ++k) fm[k] += f[k] / n;

    std::printf("== robot 1 (%s) ==\n", ip1.c_str());
    std::printf("  robot_mode %s (%d mode change(s) during the reads)\n", modeName(s1.robot_mode), mode_changes);
    std::printf("  current_errors: %s\n", std::string(s1.current_errors).c_str());
    std::printf("  last_motion_errors: %s\n", std::string(s1.last_motion_errors).c_str());
    std::printf("  q [rad]: ");
    for (std::size_t k = 0; k < 7; ++k) std::printf("%s%.6f", k ? ", " : "", qm[k]);
    std::printf("\n  q [deg]: ");
    for (std::size_t k = 0; k < 7; ++k) std::printf("%s%.4f", k ? ", " : "", qm[k] / kDeg);
    std::printf("\n  q std over the samples [deg]: ");
    for (std::size_t k = 0; k < 7; ++k) std::printf("%s%.4f", k ? ", " : "", std::sqrt(qsd[k]) / kDeg);
    std::printf("\n  O_T_EE p [%.5f %.5f %.5f] m (rms spread %.3f mm)  q %s (max spread %.4f deg)\n", pm.x(), pm.y(), pm.z(),
                std::sqrt(psd) * 1e3, fmtQuat(qmean).c_str(), qspread / kDeg);
    std::printf("  O_F_ext_hat_K [N, Nm]: [%.2f %.2f %.2f | %.3f %.3f %.3f]  |F| %.2f N\n", fm[0], fm[1], fm[2], fm[3], fm[4], fm[5],
                std::sqrt(fm[0] * fm[0] + fm[1] * fm[1] + fm[2] * fm[2]));
    std::printf("  tau_ext_hat_filtered [Nm]: ");
    for (std::size_t k = 0; k < 7; ++k) std::printf("%s%.3f", k ? ", " : "", s1.tau_ext_hat_filtered[k]);
    std::printf("\n  m_ee %.3f kg, m_load %.3f kg\n", s1.m_ee, s1.m_load);
    const Isometry3d FtEE = fromColMajor(s1.F_T_EE), NEtEE = fromColMajor(s1.NE_T_EE);
    std::printf("  F_T_EE (Desk end effector + NE_T_EE): t [%.5f %.5f %.5f] m, q %s; NE_T_EE %s\n", FtEE.translation().x(),
                FtEE.translation().y(), FtEE.translation().z(), fmtQuat(Quaterniond(FtEE.linear())).c_str(),
                NEtEE.isApprox(Isometry3d::Identity(), 1e-9) ? "identity" : "NOT identity (set by setEE: the TCP is not the Desk one)");
    {  // FK check on the robot: flange from the state vs Panda DH FK of q
      sat::Q7 q{};
      for (std::size_t k = 0; k < 7; ++k) q[k] = qm[k];
      const Isometry3d Ffk = sat::pandaFlangeFk(q);
      const Isometry3d Frob = Isometry3d(Eigen::Translation3d(pm) * qmean) * FtEE.inverse();
      std::printf("  flange: from the state [%.5f %.5f %.5f] m, Panda DH FK [%.5f %.5f %.5f] m: |dp| %.3f mm, angle %.4f deg\n",
                  Frob.translation().x(), Frob.translation().y(), Frob.translation().z(), Ffk.translation().x(), Ffk.translation().y(),
                  Ffk.translation().z(), (Frob.translation() - Ffk.translation()).norm() * 1e3,
                  Quaterniond(Frob.linear()).angularDistance(Quaterniond(Ffk.linear())) / kDeg);
    }

    // ---- robot 2 / satellite
    if (r2) {
      double mq = 0.0, mf = 0.0;
      for (double v : th_q7) mq += v / n;
      for (double v : th_fl) mf += v / n;
      std::printf("== robot 2 (%s): satellite phase (satellite_node formulas, hw_config scene.*) ==\n", ip2.c_str());
      std::printf("  robot_mode %s, current_errors: %s\n", modeName(s2.robot_mode), std::string(s2.current_errors).c_str());
      std::printf("  q7 %.4f deg -> theta_q7 %.4f deg; theta_flange %.4f deg (published by satellite_node); diff %.4f deg\n",
                  s2.q[6] / kDeg, mq / kDeg, mf / kDeg, sat::angleDiff(mf, mq) / kDeg);
      std::printf("  centre from FK [%.4f %.4f %.4f] m vs scene.center_m [%.4f %.4f %.4f] m: |d| %.2f mm\n", g.center.x(), g.center.y(),
                  g.center.z(), hw.scene.center_m[0], hw.scene.center_m[1], hw.scene.center_m[2],
                  (g.center - Vector3d(hw.scene.center_m[0], hw.scene.center_m[1], hw.scene.center_m[2])).norm() * 1e3);
      if (have_cmd) {
        const double th = c.phase_source == "q7" ? mq : mf;
        const Nominal nom = nominalPose(cmd, hw, th);
        std::printf("== nominal grasp pose from %s (phase from %s) ==\n", c.grasp_command.c_str(), c.phase_source.c_str());
        if (nom.ok) printDeviation("robot 1 O_T_EE", pm, qmean, nom, rim);
      }
    } else if (have_cmd) {
      std::printf("(--grasp-command without --theta-from-robot2: no phase, no nominal pose)\n");
    }
    if (csv) {
      std::fclose(csv);
      std::FILE* m = std::fopen((base + ".csv.meta.txt").c_str(), "wx");
      if (m) {
        std::fprintf(m, "csv: %s.csv\ntool: pose_read (readOnce only)\nhw_config: %s\nrobot1_ip: %s\nrobot2_ip: %s\nsamples: %d\nperiod_ms: %d\n",
                     base.c_str(), c.hw_config.c_str(), ip1.c_str(), r2 ? ip2.c_str() : "-", c.samples, c.period_ms);
        std::fprintf(m, "grasp_command: %s\nphase_source: %s\n", c.grasp_command.c_str(), c.phase_source.c_str());
        std::fprintf(m, "git_head: %s\ngit_dirty: %s\ngit_diff_sha256: %s\n", RR_GIT_HEAD, RR_GIT_DIRTY, RR_GIT_DIFF_SHA256);
        std::fclose(m);
      }
      std::printf("CSV: %s.csv\n", base.c_str());
    }
    return 0;
  } catch (const franka::Exception& e) {
    std::fprintf(stderr, "franka: %s\n", e.what());
    return 1;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
