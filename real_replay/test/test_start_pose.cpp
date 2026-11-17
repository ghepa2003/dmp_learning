// start_pose_check: config parsing and comparison of O_T_EE with the expected pose.
//   test_start_pose <config with start_pose_check> <config without it>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "real_replay/config.hpp"
#include "real_replay/preflight_checks.hpp"

using namespace real_replay;

static int g_fail = 0;
static void check(bool ok, const char* what) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) ++g_fail;
}

static std::array<double, 16> pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = q.toRotationMatrix();
  T.block<3, 1>(0, 3) = p;
  std::array<double, 16> a{};
  Eigen::Map<Eigen::Matrix4d>(a.data()) = T;
  return a;
}

static std::string slurp(const std::string& p) {
  std::ifstream f(p);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

/// Writes `text` next to the original config (relative paths keep resolving) and loads it; returns
/// the error message, or "" if it loaded.
static std::string loadText(const std::string& dir_of, const std::string& text, Config* out) {
  const std::string path = dir_of.substr(0, dir_of.find_last_of('/') + 1) + "_tmp_start_pose_test.yaml";
  { std::ofstream(path) << text; }
  std::string err;
  try {
    *out = loadConfig(path);
  } catch (const std::exception& e) {
    err = e.what();
  }
  std::remove(path.c_str());
  return err;
}

int main(int argc, char** argv) {
  if (argc < 3) return 2;
  const Config with = loadConfig(argv[1]);
  const Config without = loadConfig(argv[2]);
  check(!without.start_pose_check.present, "key absent: no check (behaviour unchanged)");
  check(with.start_pose_check.present, "key present in the config");
  const StartPoseCheckConfig& e = with.start_pose_check;
  check(e.quaternion_wxyz[0] == 0.0 && e.quaternion_wxyz[1] == 0.70711 && e.quaternion_wxyz[2] == 0.0 && e.quaternion_wxyz[3] == 0.70711,
        "quaternion_wxyz [0, 0.70711, 0, 0.70711] (explicit)");
  check(e.position_m[0] == 0.4293 && e.position_m[1] == 0.0 && e.position_m[2] == 0.7854, "position_m [0.4293, 0, 0.7854]");
  check(e.tol_position_m == 0.005 && e.tol_rotation_rad == 0.02, "tolerances 0.005 m / 0.02 rad");

  const Eigen::Vector3d p0(e.position_m[0], e.position_m[1], e.position_m[2]);
  const Eigen::Quaterniond q0(e.quaternion_wxyz[0], e.quaternion_wxyz[1], e.quaternion_wxyz[2], e.quaternion_wxyz[3]);
  StartPoseResult r = checkStartPose(e, pose(p0, q0));
  check(r.ok && r.dist_m < 1e-12 && r.angle_rad < 1e-6, "exact pose: ok");
  r = checkStartPose(e, pose(p0 + Eigen::Vector3d(0.004, 0, 0), q0));
  check(r.ok && std::fabs(r.dist_m - 0.004) < 1e-9, "4 mm off: ok (inside 5 mm)");
  r = checkStartPose(e, pose(p0 + Eigen::Vector3d(0.003, 0.004, 0.003), q0));
  check(!r.ok && std::fabs(r.dist_m - std::sqrt(0.000034)) < 1e-9, "5.83 mm off (norm): refused, distance reported");
  r = checkStartPose(e, pose(p0 + Eigen::Vector3d(0, 0, -0.1), q0));
  check(!r.ok && std::fabs(r.dist_m - 0.1) < 1e-9, "10 cm off: refused");
  r = checkStartPose(e, pose(p0, Eigen::Quaterniond(Eigen::AngleAxisd(0.015, Eigen::Vector3d::UnitZ())) * q0));
  check(r.ok && std::fabs(r.angle_rad - 0.015) < 1e-9, "0.015 rad off: ok");
  r = checkStartPose(e, pose(p0, Eigen::Quaterniond(Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitX())) * q0));
  check(!r.ok && std::fabs(r.angle_rad - 0.03) < 1e-9, "0.03 rad off: refused, angle reported");
  r = checkStartPose(e, pose(p0, Eigen::Quaterniond(-q0.coeffs()(3), -q0.coeffs()(0), -q0.coeffs()(1), -q0.coeffs()(2))));
  check(r.ok && r.angle_rad < 1e-6, "q and -q are the same orientation");
  // q = -q, also when the CONFIG holds the negated quaternion.
  r = checkStartPose(e, pose(p0, Eigen::Quaterniond(-q0.w(), -q0.x(), -q0.y(), -q0.z())));
  check(r.ok && r.angle_rad < 1e-6 && r.dist_m < 1e-12, "q vs -q: distance 0");
  StartPoseCheckConfig neg = e;
  for (double& v : neg.quaternion_wxyz) v = -v;
  r = checkStartPose(neg, pose(p0, q0));
  check(r.ok && r.angle_rad < 1e-6, "expected -q vs read q: distance 0");
  r = checkStartPose(e, pose(p0, Eigen::Quaterniond(Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitX())) * Eigen::Quaterniond(-q0.w(), -q0.x(), -q0.y(), -q0.z())));
  check(!r.ok && std::fabs(r.angle_rad - 0.03) < 1e-9, "-q rotated by 0.03 rad: refused with the right angle");

  // quaternion_wxyz missing while position_m is present: config error
  std::string text = slurp(argv[1]);
  const std::string qkey = "quaternion_wxyz: [0.0, 0.70711, 0.0, 0.70711], ";
  check(text.find(qkey) != std::string::npos, "(fixture) quaternion entry found in the config text");
  std::string no_q = text;
  no_q.erase(no_q.find(qkey), qkey.size());
  Config tmp;
  std::string err = loadText(argv[1], no_q, &tmp);
  check(err.find("quaternion_wxyz") != std::string::npos, "quaternion_wxyz missing: config error naming the key");
  // position_m missing: config error
  std::string no_p = text;
  const std::string pkey = "position_m: [0.4293, 0.0, 0.7854], ";
  no_p.erase(no_p.find(pkey), pkey.size());
  err = loadText(argv[1], no_p, &tmp);
  check(err.find("position_m") != std::string::npos, "position_m missing: config error naming the key");
  // the unmodified text loads
  check(loadText(argv[1], text, &tmp).empty(), "(fixture) unmodified text loads");
  std::printf("%s\n", g_fail ? "FAILED" : "all passed");
  return g_fail ? 1 : 0;
}
