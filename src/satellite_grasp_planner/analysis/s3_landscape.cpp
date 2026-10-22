// s3_landscape: exports the "landscape" of the S3 selector, one CSV row per (k, theta, psi), also for rejected
// candidates, for OFFLINE evaluation of alternative cost formulations (tools/s3_landscape_report.py).
// Nothing here changes the planner: selectGraspS3 is called as is (with S3Context::eval_log on), psi-rolled
// evaluations call the same evaluateGraspPoseDetailed / s3AprioriRejectMask / s3DPos / planLaunch /
// checkKinematicFeasibility / solvePoseIk the planner uses. Build/run: analysis/README.md (BUILD_ANALYSIS_TOOLS).
//
// psi = deliberate roll about the LOCAL z of the TCP = the approach axis z_tcp = -R(theta) n_body
// (CubeSatelliteModel::graspPoseAt, cube_satellite_model.cpp:61,72). The TCP position does not depend on psi.
// At psi = 0 the construction is "in the collar frame": z_tcp = -face normal, y_tcp = -r_k (radial); the Franka
// fingers close along the hand y (URDF fer_finger_joint1 axis 0 1 0; fer_hand_tcp_joint has rpy 0), i.e. along the
// radial direction, so the pads are parallel to the (tangential) wall. The tool checks this numerically
// (collar_frame_check) and prints the result.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/math_utils.hpp"
#include "haptic_dmp_learning/core/pose_rotation.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/surface_velocity.hpp"
#include "s3_landscape_util.hpp"
#include "satellite_grasp_planner/core/grasp_pose_eval.hpp"
#include "satellite_grasp_planner/core/grasp_s3_launch.hpp"
#include "satellite_grasp_planner/core/grasp_s3_selection.hpp"
#include "satellite_grasp_planner/core/kinematic_feasibility.hpp"
#include "satellite_grasp_planner/core/launch_plan.hpp"
#include "satellite_grasp_planner/core/manipulability.hpp"
#include "satellite_grasp_planner/core/selection_inputs.hpp"
#include "satellite_grasp_planner/core/start_state.hpp"

using namespace satellite_grasp_planner::core;
using haptic_dmp_learning::core::CubeSatelliteModel;
using haptic_dmp_learning::core::GraspPointId;
using haptic_dmp_learning::core::RotatablePose;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr int kFollowSamples = 7;
// reject_mask bits: 1..8 are S3RejectBit (a-priori, decide `accepted`); 16..64 are diagnostics added by this tool.
constexpr unsigned kBitVelocity = 16u;  // |v_target - v_ee| > delta_v_max (post-hoc velocity check, evaluated for all rows)
constexpr unsigned kBitPath = 32u;      // checkKinematicFeasibility infeasible (joint limits along the rollout)
constexpr unsigned kBitFollow = 64u;    // IK failed on one of the follow-arc samples

// Run of 2026-10-15 (tools/s1_run.py:57-60, analysis_s1_1015/calc_top5_final2.cpp:31-46).
const char* kRunQStart = "-0.000009,-0.786572,0.000002,-2.378914,-0.000006,3.141609,0.785398";

// RUN15 snapshot. The tool evaluates at t_now = snap.t_s (it passes snap.t_s as t_now to planGraspAndLaunchS1), so
// --theta-snap-rad must be theta_NOW = theta_snap + omega*elapsed, NOT the raw snapshot angle:
// -1.626193073 + (-0.034906585 * 1.334242) = -1.6727670737402633 at t_snap = 18.318 s.
constexpr double kRun15ThetaSnapRad = -1.6727670737402633;
constexpr double kRun15TSnapS = 18.318;
constexpr double kRun15GraspRadiusM = 0.068;  // config/grasp_planner_example.yaml, cube_geometry.grasp_radius_m
const char* kDefaultParamsRel = "config/grasp_planner_example.yaml";

struct Args {
  std::string weights_dir = "runs/20261015_095418_fit_demo_20261014T150851";
  std::string params;            // empty after parseArgs never happens: default resolved there
  bool params_explicit = false;  // --params given on the command line
  std::string urdf;
  std::string q_start = kRunQStart;  // "ready" = RobotModel::readyPose()
  double omega_deg_s = -2.0;
  double depth = 0.0125;
  double follow_s = 3.0;
  double psi_max_deg = 5.0;
  double psi_step_deg = 1.0;
  double theta_step_deg = 2.0;
  std::string theta_extra_deg;  // comma list, evaluated for all k (grid = extra)
  int refine_top_n = 6;         // per k: top-N coarse candidates by d_pos (psi = 0, IK ok) are refined
  double refine_step_deg = 0.1;
  double refine_halfwidth_deg = 2.0;
  int seed_spread = 0;
  std::string out = "landscape.csv";
  std::string rollout = "all";  // all | psi0 | none
  bool selftest = false;
  // scenario of the 15/10 run (not in the requested argument list, needed to reproduce its winner)
  std::string center = "0.70,0.0,0.77";
  double theta_snap_rad = kRun15ThetaSnapRad;  // = theta_now at t_snap_s (see kRun15ThetaSnapRad)
  double t_snap_s = kRun15TSnapS;
  double min_delay_s = 5.0;
  int posthoc_velocity = 0;  // the S1 campaigns ran with s1.posthoc_velocity_check=false
};

std::vector<double> parseList(const std::string& s, std::size_t n, const char* what) {
  std::vector<double> v;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    std::size_t used = 0;
    v.push_back(std::stod(tok, &used));
  }
  if (v.size() != n) throw std::invalid_argument(std::string(what) + ": expected " + std::to_string(n) + " values");
  return v;
}

std::string trim(const std::string& s) {
  const auto a = s.find_first_not_of(" \t\r\n");
  if (a == std::string::npos) return "";
  return s.substr(a, s.find_last_not_of(" \t\r\n") - a + 1);
}

/// Minimal reader of the planner YAML: flat `key: value` lines (dotted keys) of the `grasp_planner_node:` block.
/// No yaml-cpp (flat scalars/arrays only; comments after '#').
std::map<std::string, std::string> readPlannerYaml(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open --params " + path);
  std::map<std::string, std::string> kv;
  std::string line;
  bool in_block = false;
  while (std::getline(f, line)) {
    const auto h = line.find('#');
    if (h != std::string::npos) line = line.substr(0, h);
    if (trim(line).empty()) continue;
    if (line[0] != ' ' && line[0] != '\t') {
      in_block = (trim(line) == "grasp_planner_node:");
      continue;
    }
    if (!in_block) continue;
    const auto c = line.find(':');
    if (c == std::string::npos) continue;
    const std::string key = trim(line.substr(0, c)), val = trim(line.substr(c + 1));
    if (!val.empty()) kv[key] = val;
  }
  return kv;
}

std::vector<double> yamlList(const std::string& v) {
  std::string t = v;
  t.erase(std::remove(t.begin(), t.end(), '['), t.end());
  t.erase(std::remove(t.begin(), t.end(), ']'), t.end());
  std::vector<double> out;
  std::stringstream ss(t);
  std::string tok;
  while (std::getline(ss, tok, ',')) out.push_back(std::stod(tok));
  return out;
}

/// Default --params: installed package share (AMENT_PREFIX_PATH), then relative to this source file, then cwd.
/// Throws if not found: the code's default grasp_radius_m (0.0725) must never be used silently.
std::string resolveDefaultParams() {
  std::vector<std::string> cand;
  if (const char* ap = std::getenv("AMENT_PREFIX_PATH")) {
    std::stringstream ss(ap);
    std::string pre;
    while (std::getline(ss, pre, ':')) {
      if (!pre.empty()) cand.push_back(pre + "/share/satellite_grasp_planner/" + kDefaultParamsRel);
    }
  }
  cand.push_back((std::filesystem::path(__FILE__).parent_path().parent_path() / kDefaultParamsRel).string());
  cand.push_back(kDefaultParamsRel);
  for (const auto& c : cand) {
    std::error_code ec;
    if (std::filesystem::is_regular_file(c, ec)) return c;
  }
  std::string msg = std::string("default --params ") + kDefaultParamsRel + " not found; tried:";
  for (const auto& c : cand) msg += "\n  " + c;
  throw std::runtime_error(msg + "\npass --params <file> explicitly");
}

Args parseArgs(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    auto val = [&]() -> std::string {
      if (i + 1 >= argc) throw std::invalid_argument("missing value for " + k);
      return argv[++i];
    };
    if (k == "--weights-dir")
      a.weights_dir = val();
    else if (k == "--params") {
      a.params = val();
      a.params_explicit = true;
    } else if (k == "--urdf")
      a.urdf = val();
    else if (k == "--q-start")
      a.q_start = val();
    else if (k == "--omega-deg-s")
      a.omega_deg_s = std::stod(val());
    else if (k == "--depth")
      a.depth = std::stod(val());
    else if (k == "--follow-s")
      a.follow_s = std::stod(val());
    else if (k == "--psi-max-deg")
      a.psi_max_deg = std::stod(val());
    else if (k == "--psi-step-deg")
      a.psi_step_deg = std::stod(val());
    else if (k == "--theta-step-deg")
      a.theta_step_deg = std::stod(val());
    else if (k == "--theta-extra-deg")
      a.theta_extra_deg = val();
    else if (k == "--refine-top-n")
      a.refine_top_n = std::stoi(val());
    else if (k == "--refine-step-deg")
      a.refine_step_deg = std::stod(val());
    else if (k == "--refine-halfwidth-deg")
      a.refine_halfwidth_deg = std::stod(val());
    else if (k == "--seed-spread")
      a.seed_spread = std::stoi(val());
    else if (k == "--out")
      a.out = val();
    else if (k == "--rollout")
      a.rollout = val();
    else if (k == "--selftest")
      a.selftest = true;
    else if (k == "--center")
      a.center = val();
    else if (k == "--theta-snap-rad")
      a.theta_snap_rad = std::stod(val());
    else if (k == "--t-snap-s")
      a.t_snap_s = std::stod(val());
    else if (k == "--min-delay-s")
      a.min_delay_s = std::stod(val());
    else if (k == "--posthoc-velocity")
      a.posthoc_velocity = std::stoi(val());
    else
      throw std::invalid_argument("unknown argument " + k);
  }
  if (a.rollout != "all" && a.rollout != "psi0" && a.rollout != "none") {
    throw std::invalid_argument("--rollout must be all|psi0|none");
  }
  if (!(a.psi_step_deg > 0.0) || !(a.psi_max_deg >= 0.0) || !(a.theta_step_deg > 0.0) || a.seed_spread < 0 ||
      a.refine_top_n < 0 || !(a.refine_step_deg > 0.0) || !(a.refine_halfwidth_deg >= 0.0)) {
    throw std::invalid_argument("invalid psi/theta/refine step, refine-top-n or seed-spread");
  }
  if (a.params_explicit && a.params.empty()) {
    throw std::invalid_argument("--params is empty: the planner parameters (grasp_radius_m) are mandatory");
  }
  if (!a.params_explicit) a.params = resolveDefaultParams();
  if (a.urdf.empty()) {
    const char* home = std::getenv("HOME");
    a.urdf = std::string(home ? home : "/root") + "/thesis_ws/fer_flat_effort.urdf";
  }
  return a;
}

struct Row {
  int k = 0;
  double theta_deg = 0, psi_deg = 0;
  unsigned mask = 0;
  bool accepted = false;
  int ik_A = 0, ik_B = 0, branch = 0;
  double d_pos_mm = kNaN, t_wait_s = kNaN, J_current = kNaN, w_hat = kNaN, w_trans = kNaN, w_rot = kNaN,
         w_full = kNaN, smin_trans = kNaN, smin_rot = kNaN, margin_rollout = kNaN, margin_follow = kNaN;
  int follow_ik_ok = 0, path_ok = 0;
  double w_full_other = kNaN, margin_other = kNaN, margin_ik = kNaN, dv = kNaN;
  double w_full_seedmin = kNaN, w_full_seedmax = kNaN, margin_seedmin = kNaN, margin_seedmax = kNaN;
  int n_seed_conv = 0;
  double theta_rad = 0;         // exact value used (theta_deg is derived)
  const char* grid = "coarse";  // coarse | fine | extra
};

const char* firstReason(unsigned mask) {
  if (mask & kRejectIk) return "ik";
  if (mask & kRejectWHat) return "w_hat";
  if (mask & kRejectMargin) return "margine";
  if (mask & kRejectRTrust) return "r_trust";
  if (mask & kBitVelocity) return "velocita";
  if (mask & kBitPath) return "percorso";
  if (mask & kBitFollow) return "follow_ik";
  return "none";
}

std::uint64_t bits(double x) {
  std::uint64_t u;
  std::memcpy(&u, &x, sizeof u);
  return u;
}

bool close(double a, double b, double tol = 1e-9) {
  if (std::isnan(a) && std::isnan(b)) return true;
  return std::abs(a - b) <= tol * std::max(1.0, std::max(std::abs(a), std::abs(b)));
}

void writeNum(std::ostream& o, double v) {
  if (std::isnan(v))
    o << "nan";
  else
    o << v;
}

const char* kCsvHeader =
    "k,theta_deg,psi_deg,accepted,reject_first,reject_mask,ik_ok_A,ik_ok_B,branch_selected,d_pos_mm,t_wait_s,"
    "J_current,w_hat,w_trans,w_rot,w_full,smin_trans,smin_rot,margin_rollout_rad,margin_follow_rad,follow_ik_ok,"
    "path_ok,w_full_other,margin_other,margin_ik_rad,dv_m_s,w_full_seedmin,w_full_seedmax,margin_seedmin,"
    "margin_seedmax,n_seed_conv,grid";

void writeRow(std::ostream& o, const Row& r) {
  o << r.k << ',' << r.theta_deg << ',' << r.psi_deg << ',' << (r.accepted ? 1 : 0) << ',' << firstReason(r.mask)
    << ',' << r.mask << ',' << r.ik_A << ',' << r.ik_B << ',' << r.branch << ',';
  const double v[] = {r.d_pos_mm, r.t_wait_s, r.J_current, r.w_hat, r.w_trans, r.w_rot, r.w_full, r.smin_trans,
                      r.smin_rot, r.margin_rollout, r.margin_follow};
  for (std::size_t i = 0; i < sizeof v / sizeof v[0]; ++i) {
    writeNum(o, v[i]);
    o << ',';
  }
  o << r.follow_ik_ok << ',' << r.path_ok << ',';
  const double w[] = {r.w_full_other, r.margin_other, r.margin_ik, r.dv, r.w_full_seedmin, r.w_full_seedmax,
                      r.margin_seedmin, r.margin_seedmax};
  for (std::size_t i = 0; i < sizeof w / sizeof w[0]; ++i) {
    writeNum(o, w[i]);
    o << ',';
  }
  o << r.n_seed_conv << ',' << r.grid << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Args a = parseArgs(argc, argv);
    const double kPi = haptic_dmp_learning::core::kPi;

    // ---- planner parameters: defaults < --params YAML < CLI ----
    S3Params s3;
    CubeSatelliteModel::Params cube;
    double min_delay = a.min_delay_s;
    {
      const auto kv = readPlannerYaml(a.params);
      if (!kv.count("cube_geometry.grasp_radius_m")) {
        throw std::runtime_error("--params " + a.params + " has no cube_geometry.grasp_radius_m: refusing the code default");
      }
      auto d = [&](const char* key, double& dst) {
        const auto it = kv.find(key);
        if (it != kv.end()) dst = std::stod(it->second);
      };
      auto b = [&](const char* key, bool& dst) {
        const auto it = kv.find(key);
        if (it != kv.end()) dst = (it->second == "true");
      };
      d("s1.length_scale_m", s3.length_scale_m);
      d("s1.orientation_weight", s3.orientation_weight);
      d("s1.angle_scale_rad", s3.angle_scale_rad);
      d("s1.epsilon_wait_per_s", s3.epsilon_wait_per_s);
      d("s1.w_hat_min", s3.w_hat_min);
      d("s1.margin_min_rad", s3.margin_min_rad);
      d("s1.r_trust_m", s3.r_trust_m);
      d("s1.delta_v_max_m_s", s3.delta_v_max_m_s);
      d("s1.theta_step_deg", s3.theta_step_deg);
      d("s1.theta_refine_step_deg", s3.theta_refine_step_deg);
      d("s1.posthoc_cluster_deg", s3.posthoc_cluster_deg);
      d("s1.ik.pos_tol_m", s3.ik.pos_tol_m);
      d("s1.ik.ori_tol_rad", s3.ik.ori_tol_rad);
      d("s1.ik.damping", s3.ik.damping);
      d("s1.ik.nullspace_gain", s3.ik.nullspace_gain);
      d("s1.ik.max_step_rad", s3.ik.max_step_rad);
      b("s1.posthoc_path_check", s3.posthoc_path_check);
      if (kv.count("s1.ik.max_iter")) s3.ik.max_iter = std::stoi(kv.at("s1.ik.max_iter"));
      if (kv.count("s1.max_posthoc_candidates")) s3.max_posthoc_candidates = std::stoi(kv.at("s1.max_posthoc_candidates"));
      d("cube_geometry.cube_side_m", cube.cube_side_m);
      d("cube_geometry.standoff_m", cube.standoff_m);
      d("cube_geometry.collar_radius_m", cube.collar_radius_m);
      d("cube_geometry.wall_thickness_m", cube.wall_thickness_m);
      if (kv.count("cube_geometry.grasp_radius_m")) cube.grasp_radius_m = std::stod(kv.at("cube_geometry.grasp_radius_m"));
      if (kv.count("cube_geometry.face_normal_body")) {
        const auto v = yamlList(kv.at("cube_geometry.face_normal_body"));
        if (v.size() == 3) cube.face_normal_body = Eigen::Vector3d(v[0], v[1], v[2]);
      }
    }
    s3.theta_step_deg = a.theta_step_deg;
    s3.posthoc_velocity_check = a.posthoc_velocity != 0;
    cube.grasp_depth_m = a.depth;

    const auto cvec = parseList(a.center, 3, "--center");
    SatelliteSnapshot snap;
    snap.center = Eigen::Vector3d(cvec[0], cvec[1], cvec[2]);
    snap.axis = Eigen::Vector3d::UnitZ();
    snap.omega_rad_s = a.omega_deg_s * kPi / 180.0;
    snap.theta_rad = a.theta_snap_rad;
    snap.t_s = a.t_snap_s;
    cube.center_world = snap.center;
    cube.axis_world = snap.axis;

    StartState start;
    if (a.q_start == "ready") {
      start.q = RobotModel::readyPose();
      start.source = "RobotModel::readyPose()";
    } else {
      const auto qv = parseList(a.q_start, 7, "--q-start");
      start = makeStartStateFromOverride(qv, a.urdf);
    }

    // ---- scenario header (stderr, and <out>.meta next to the CSV: a CSV comment line would break the parsers) ----
    {
      std::ostringstream m;
      m << std::setprecision(17) << "params=" << a.params << "\ngrasp_radius_m=" << cube.grasp_radius_m.value()
        << "\ndepth=" << a.depth << "\nomega_deg_s=" << a.omega_deg_s << "\ncenter=" << a.center
        << "\ntheta_snap_rad=" << a.theta_snap_rad << "\nt_snap_s=" << a.t_snap_s << "\nq_start=" << a.q_start
        << "\ndemo=" << a.weights_dir << "\n";
      std::cerr << "---- scenario ----\n"
                << m.str() << "------------------\n";
      if (!a.selftest) {
        std::ofstream mf(a.out + ".meta");
        if (!mf) throw std::runtime_error("cannot write " + a.out + ".meta");
        mf << m.str();
      }
    }

    SelectionInputs in = buildSelectionInputs(a.weights_dir + "/weights_demo_params.yaml", a.urdf, snap, cube, start);
    std::cerr << describeStartState(in) << "\n";

    // ---- the real selector, with the per-candidate log on ----
    std::vector<S3EvalRecord> log;
    S3Result res;
    const GraspLaunchResult lr = planGraspAndLaunchS1(in, snap, min_delay, s3, snap.t_s, &res, &log);
    std::cerr << "selectGraspS3: status=" << res.status << " evaluations=" << res.evaluations
              << " log_rows=" << log.size() << " posthoc_checks=" << res.posthoc_checks << " note: " << res.note << "\n";
    if (res.status == S3Result::kSelected) {
      const S3Candidate& c = res.chosen;
      std::cerr << std::setprecision(10) << "WINNER k=" << static_cast<int>(c.eval.k)
                << " theta_deg=" << c.eval.theta_rad * 180.0 / kPi << " d_pos_mm=" << c.d_pos_m * 1000.0
                << " w_hat=" << c.eval.w_hat << " margin_rad=" << c.eval.margins.min_margin_rad
                << " wrist_flip=" << c.eval.wrist_flip << " cost=" << c.cost << " wait_s=" << c.wait_s << "\n"
                << "RUN15 reference: k=0 theta=-163.5 deg d_pos=7.171 mm w_hat=0.8238 margin=0.5255 rad "
                   "(informative; depends on --theta-snap-rad/--t-snap-s/--center)\n";
    }

    // ---- context for the psi-rolled evaluations (same glue as planGraspAndLaunchS1, grasp_s3_launch.cpp:36-52) ----
    const CubeSatelliteModel model(in.cube_params);
    RobotModel& robot = *in.robot;
    robot.update(in.q0, RobotModel::JointVector::Zero());
    DemoReference demo;
    demo.position = in.p0 + in.delta_g_demo;
    demo.orientation = robot.eeOrientation();
    demo.w_trans_demo = in.w_trans_demo;
    const double tau_launch = in.contact_assumed_at_end ? in.prodmp_template.tau() : in.tau_contact_s;

    // ProDMP at contact for the velocity diagnostic (copy of grasp_s3_selection.cpp velocity setup).
    haptic_dmp_learning::core::ProDMP pd(in.prodmp_template);
    pd.setRelativeGoal(false);
    pd.setInitialConditions(0.0, in.p0, Eigen::Vector3d::Zero());
    pd.setGoal(in.p0 + in.delta_g_demo);
    {
      const double tc = in.tau_contact_s > 0.0 ? in.tau_contact_s : in.prodmp_template.tau();
      const int n = std::max(1, static_cast<int>(std::ceil(tc / 0.005 - 1e-9)));
      for (int i = 0; i < n; ++i) pd.step(0.005);
    }

    // ---- collar frame / approach axis checks (psi = 0 construction) ----
    bool frame_ok = true;
    for (int ki = 0; ki < 4; ++ki) {
      const auto k = static_cast<GraspPointId>(ki);
      const auto k2 = static_cast<GraspPointId>((ki + 2) % 4);
      const double th = 0.37;
      const auto p = model.graspPoseAt(k, th), p2 = model.graspPoseAt(k2, th);
      const Eigen::Vector3d radial = (p.position_world - 0.5 * (p.position_world + p2.position_world)).normalized();
      const Eigen::Matrix3d R = p.orientation_nominal_world.toRotationMatrix();
      frame_ok = frame_ok && (R.col(2) - p.approach_axis_world).norm() < 1e-9 &&
                 (p.approach_axis_world + model.faceNormalWorld(th)).norm() < 1e-9 &&
                 (R.col(1) + radial).norm() < 1e-9;  // y_tcp = -radial: fingers close along the radial direction
    }
    std::cerr << "approach axis = local z of the TCP = z_tcp = -R(theta) n_body (cube_satellite_model.cpp:61,72)\n"
              << "collar_frame_check (z_tcp = -face normal, y_tcp = -radial, hence pads parallel to the wall): "
              << (frame_ok ? "OK" : "FAIL: at psi=0 the walls are NOT parallel to the pads") << "\n";

    // ---- sweep ----
    std::vector<double> psis_deg;
    if (a.selftest) {
      psis_deg = {0.0};
    } else {
      const int n = static_cast<int>(std::lround(a.psi_max_deg / a.psi_step_deg));
      for (int j = -n; j <= n; ++j) psis_deg.push_back(j * a.psi_step_deg);
    }
    const double step = s3.theta_step_deg * kPi / 180.0;
    const int n_theta = static_cast<int>(std::ceil(2.0 * kPi / step - 1e-9));  // same as grasp_s3_selection.cpp
    const std::vector<double> t_follow = s3_landscape::followSampleTimes(a.follow_s, kFollowSamples);
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> uni(-0.3, 0.3);

    std::vector<Row> rows;
    double max_pos_dev = 0.0, max_fk_dev = 0.0;

    // One (k, theta, psi) evaluation = one CSV row (identical computation for every grid).
    auto evalRow = [&](int ki, double theta, double psi_deg, const char* grid) {
      const auto k = static_cast<GraspPointId>(ki);
      const Eigen::Vector3d pos0 = model.graspPoseAt(k, theta).position_world;
      const double psi = psi_deg * kPi / 180.0;
      const PoseEvaluationDetail det = evaluateGraspPoseDetailed(
          model, k, theta, psi, robot, in.q0, demo, s3.length_scale_m, s3.angle_scale_rad, s3.ik, true);
      const PoseEvaluation& ev = det.eval;
      const BranchDetail& bs = det.branch[det.selected];
      const BranchDetail& bo = det.branch[1 - det.selected];
      max_pos_dev = std::max(max_pos_dev, (ev.pose.position_world - pos0).norm());
      if (bs.converged) {
        robot.update(bs.q, RobotModel::JointVector::Zero());
        max_fk_dev = std::max(max_fk_dev, (robot.eePosition() - pos0).norm());
      }

      Row r;
      r.k = ki;
      r.theta_rad = theta;
      r.theta_deg = theta * 180.0 / kPi;
      r.psi_deg = psi_deg;
      r.ik_A = det.branch[0].converged ? 1 : 0;
      r.ik_B = det.branch[1].converged ? 1 : 0;
      r.branch = det.selected;
      const double d_pos = s3DPos(ev.pose.position_world, in.contact_to_end_offset, in.p0, in.delta_g_demo);
      r.d_pos_mm = d_pos * 1000.0;
      r.mask = s3AprioriRejectMask(ev, d_pos, s3);
      r.accepted = (r.mask & (kRejectIk | kRejectWHat | kRejectMargin | kRejectRTrust)) == 0;
      r.t_wait_s = planLaunch(snap, snap.t_s, theta, tau_launch, min_delay).delay_s;
      const double dd = s3Distance(d_pos, ev.dist_to_demo.ori_rad, s3.length_scale_m,
                                   s3.orientation_weight, s3.angle_scale_rad);
      r.J_current = dd + s3.epsilon_wait_per_s * r.t_wait_s;
      r.w_hat = ev.w_hat;
      r.w_trans = ev.w_trans;
      r.w_rot = bs.w_rot;
      r.w_full = bs.w_full;
      r.smin_trans = bs.smin_trans;
      r.smin_rot = bs.smin_rot;
      r.margin_ik = ev.margins.min_margin_rad;
      r.w_full_other = bo.w_full;
      r.margin_other = bo.min_margin_rad;

      const Eigen::Vector3d goal = ev.pose.position_world + in.contact_to_end_offset;
      const Eigen::Vector3d v_ee = pd.velocityForCandidateGoal(goal);
      const Eigen::Vector3d v_t =
          haptic_dmp_learning::core::surfaceVelocity(model, k, theta, snap.omega_rad_s);
      r.dv = (v_t - v_ee).norm();
      if (!(r.dv <= s3.delta_v_max_m_s)) r.mask |= kBitVelocity;

      const bool want_rollout =
          !a.selftest && (a.rollout == "all" || (a.rollout == "psi0" && psi_deg == 0.0));
      if (want_rollout && ev.feasible_basic) {
        const FeasibilityResult fr = checkKinematicFeasibility(
            in.prodmp_template, ev.pose.position_world, bs.orientation, in.robot, in.q0);
        r.margin_rollout = fr.min_joint_margin_rad;
        r.path_ok = fr.feasible ? 1 : 0;
        if (!fr.feasible) r.mask |= kBitPath;
      }

      // follow arc: grasp pose rotated by omega*t about the satellite axis, hot-started IK (same branch)
      if (!a.selftest && ev.feasible_basic) {
        bool ok = true;
        double mmin = ev.margins.min_margin_rad;  // t = 0: the candidate's own solution
        RobotModel::JointVector q_prev = bs.q;
        const RotatablePose base{ev.pose.position_world, bs.orientation};
        for (std::size_t j = 1; j < t_follow.size(); ++j) {
          const RotatablePose pj = s3_landscape::rotateAboutSatelliteAxis(
              snap.center, snap.axis, base, snap.omega_rad_s, t_follow[j]);
          const IkResult ik = solvePoseIk(robot, pj.position, pj.orientation, q_prev, in.q0, s3.ik);
          if (!ik.converged) {
            ok = false;
            break;
          }
          mmin = std::min(mmin, jointMargins(robot, ik.q).min_margin_rad);
          q_prev = ik.q;
        }
        r.follow_ik_ok = ok ? 1 : 0;
        if (ok) {
          r.margin_follow = std::isnan(r.margin_rollout) ? mmin : std::min(mmin, r.margin_rollout);
        } else {
          r.mask |= kBitFollow;
        }
      }

      // seed spread (psi = 0, accepted only)
      if (!a.selftest && a.seed_spread > 0 && psi_deg == 0.0 && r.accepted) {
        double wmin = kNaN, wmax = kNaN, mmin = kNaN, mmax = kNaN;
        int nconv = 0;
        for (int s = 0; s < a.seed_spread; ++s) {
          RobotModel::JointVector qs = in.q0;
          for (int jj = 0; jj < RobotModel::kNumJoints; ++jj) qs(jj) += uni(rng);
          const IkResult ik = solvePoseIk(robot, ev.pose.position_world, bs.orientation, qs, in.q0, s3.ik);
          if (!ik.converged) continue;
          robot.update(ik.q, RobotModel::JointVector::Zero());
          const double wf = fullManipulability(robot.jacobian()).w_full;
          const double mg = jointMargins(robot, ik.q).min_margin_rad;
          wmin = std::isnan(wmin) ? wf : std::min(wmin, wf);
          wmax = std::isnan(wmax) ? wf : std::max(wmax, wf);
          mmin = std::isnan(mmin) ? mg : std::min(mmin, mg);
          mmax = std::isnan(mmax) ? mg : std::max(mmax, mg);
          ++nconv;
        }
        r.w_full_seedmin = wmin;
        r.w_full_seedmax = wmax;
        r.margin_seedmin = mmin;
        r.margin_seedmax = mmax;
        r.n_seed_conv = nconv;
      }
      r.grid = grid;
      rows.push_back(r);
    };

    // (1) coarse grid: exactly the selector's grid (theta = -pi + i*step), all psi
    for (int ki = 0; ki < 4; ++ki) {
      for (int i = 0; i < n_theta; ++i) {
        const double theta = -kPi + i * step;
        for (double psi_deg : psis_deg) evalRow(ki, theta, psi_deg, "coarse");
      }
    }
    const std::size_t n_coarse_rows = rows.size();

    // theta already evaluated per k (key = theta rounded to 1e-9 rad)
    std::set<std::pair<int, long long>> seen;
    auto tkey = [](double th) { return std::llround(th * 1e9); };
    for (const Row& r : rows) seen.insert({r.k, tkey(r.theta_rad)});

    // (2) fine refinement around the top-N coarse candidates per k (by d_pos, psi = 0, IK ok on the chosen branch),
    //     same theta expression as the selector's refinement (theta_c + j*refine), all psi of the band
    const double refine = a.refine_step_deg * kPi / 180.0;
    const int n_ref = static_cast<int>(std::floor(a.refine_halfwidth_deg / a.refine_step_deg + 1e-9));
    for (int ki = 0; ki < 4 && a.refine_top_n > 0; ++ki) {
      std::vector<std::pair<double, double>> cand;  // (d_pos_mm, theta_rad)
      for (std::size_t q = 0; q < n_coarse_rows; ++q) {
        const Row& r = rows[q];
        if (r.k == ki && r.psi_deg == 0.0 && (r.mask & kRejectIk) == 0u) cand.emplace_back(r.d_pos_mm, r.theta_rad);
      }
      std::stable_sort(cand.begin(), cand.end(),
                       [](const std::pair<double, double>& x, const std::pair<double, double>& y) { return x.first < y.first; });
      const std::size_t top = std::min<std::size_t>(static_cast<std::size_t>(a.refine_top_n), cand.size());
      for (std::size_t c = 0; c < top; ++c) {
        for (int j = -n_ref; j <= n_ref; ++j) {
          if (j == 0) continue;
          const double theta = cand[c].second + j * refine;
          if (!seen.insert({ki, tkey(theta)}).second) continue;
          for (double psi_deg : psis_deg) evalRow(ki, theta, psi_deg, "fine");
        }
      }
    }

    // (3) extra thetas (all k)
    if (!a.theta_extra_deg.empty()) {
      std::stringstream ss(a.theta_extra_deg);
      std::string tok;
      while (std::getline(ss, tok, ',')) {
        if (trim(tok).empty()) continue;
        const double theta = std::stod(tok) * kPi / 180.0;
        for (int ki = 0; ki < 4; ++ki) {
          if (!seen.insert({ki, tkey(theta)}).second) continue;
          for (double psi_deg : psis_deg) evalRow(ki, theta, psi_deg, "extra");
        }
      }
    }
    std::cerr << std::setprecision(6) << "tcp_position_deviation_vs_psi_max_m=" << max_pos_dev
              << " fk_deviation_of_converged_ik_max_m=" << max_fk_dev << " rows=" << rows.size() << "\n";

    // ---- selftest ----
    if (a.selftest) {
      int fails = 0;
      auto fail = [&](const std::string& m) {
        ++fails;
        if (fails <= 20) std::cerr << "SELFTEST FAIL: " << m << "\n";
      };
      std::map<std::pair<int, std::uint64_t>, const S3EvalRecord*> by_key;
      std::size_t n_coarse = 0;
      for (const auto& l : log) {
        by_key[{static_cast<int>(l.k), bits(l.theta_rad)}] = &l;
        if (!l.refine_stage) ++n_coarse;
      }
      if (static_cast<int>(log.size()) != res.evaluations) fail("log size != evaluations");
      if (n_coarse != n_coarse_rows) fail("coarse log rows != tool coarse psi=0 rows (theta step mismatch?)");
      // rows were generated with theta = -pi + i*step, identical to the selector: rebuild the key from the grid
      std::size_t idx = 0;
      for (int ki = 0; ki < 4; ++ki) {
        for (int i = 0; i < n_theta; ++i, ++idx) {
          if (idx >= n_coarse_rows) break;
          const double theta = -kPi + i * step;
          const Row& r = rows[idx];
          const auto it = by_key.find({ki, bits(theta)});
          if (it == by_key.end()) {
            fail("no log row for k=" + std::to_string(ki) + " theta_idx=" + std::to_string(i));
            continue;
          }
          const S3EvalRecord& l = *it->second;
          const unsigned apriori = r.mask & 15u;
          if (apriori != l.reject_mask) fail("mask differs k=" + std::to_string(ki) + " i=" + std::to_string(i));
          if (!close(r.d_pos_mm / 1000.0, l.d_pos_m)) fail("d_pos differs");
          if (!close(r.t_wait_s, l.wait_s)) fail("t_wait differs");
          if (!close(r.J_current, l.cost_current)) fail("J_current differs");
          if (!close(r.w_hat, l.w_hat)) fail("w_hat differs");
          if (!close(r.w_trans, l.w_trans)) fail("w_trans differs");
          if (!close(r.margin_ik, l.margin_rad)) fail("margin differs");
          if (r.branch != (l.wrist_flip ? 1 : 0)) fail("branch differs");
          if (!close(r.w_full, l.branch[l.wrist_flip ? 1 : 0].w_full)) fail("w_full differs");
          if (r.accepted != l.passes_constraints) fail("accepted differs");
        }
      }
      // winner over ALL psi = 0 rows (coarse + fine + extra) by J_current, informative and cross-checked with the selector
      {
        const Row* best = nullptr;
        std::vector<const Row*> pass0;
        for (const Row& r : rows)
          if (r.psi_deg == 0.0 && (r.mask & 15u) == 0u) pass0.push_back(&r);
        std::stable_sort(pass0.begin(), pass0.end(), [](const Row* x, const Row* y) {
          if (x->J_current != y->J_current) return x->J_current < y->J_current;
          if (x->k != y->k) return x->k < y->k;
          return x->theta_rad < y->theta_rad;
        });
        if (!pass0.empty()) best = pass0.front();
        std::cerr << "rows: coarse=" << n_coarse_rows << " all=" << rows.size() << " (fine/extra at psi=0)\n";
        if (best) {
          std::cerr << std::setprecision(10) << "ALL-ROWS psi=0 winner by J_current: k=" << best->k
                    << " theta_deg=" << best->theta_deg << " d_pos_mm=" << best->d_pos_mm << " w_hat=" << best->w_hat
                    << " margin_rad=" << best->margin_ik << " grid=" << best->grid << "\n";
        }
        if (res.status == S3Result::kSelected) {
          std::size_t rank = pass0.size();
          for (std::size_t q = 0; q < pass0.size(); ++q) {
            if (pass0[q]->k == static_cast<int>(res.chosen.eval.k) &&
                std::abs(pass0[q]->theta_rad - res.chosen.eval.theta_rad) < 1e-9) {
              rank = q;
              break;
            }
          }
          if (rank == pass0.size())
            fail("selector's chosen (k,theta) is not among the tool's psi=0 rows (add --theta-extra-deg or raise --refine-top-n)");
          else
            std::cerr << "selector's chosen candidate has rank " << rank << " among the tool's psi=0 passing rows (0 = first)\n";
        }
      }
      // winner: first ranked passing row, or every earlier row was rejected post-hoc / skipped by the cluster
      if (res.status == S3Result::kSelected) {
        std::vector<const S3EvalRecord*> pass;
        for (const auto& l : log)
          if (l.passes_constraints) pass.push_back(&l);
        std::stable_sort(pass.begin(), pass.end(), [](const S3EvalRecord* x, const S3EvalRecord* y) {
          if (x->cost_current != y->cost_current) return x->cost_current < y->cost_current;
          if (x->k != y->k) return static_cast<int>(x->k) < static_cast<int>(y->k);
          return x->theta_rad < y->theta_rad;
        });
        const S3Candidate& c = res.chosen;
        std::size_t pos = pass.size();
        for (std::size_t q = 0; q < pass.size(); ++q) {
          if (pass[q]->k == c.eval.k && bits(pass[q]->theta_rad) == bits(c.eval.theta_rad)) {
            pos = q;
            break;
          }
        }
        if (pos == pass.size())
          fail("winner not among the log's passing rows");
        else {
          if (!close(pass[pos]->cost_current, c.cost)) fail("winner cost differs from log");
          if (!close(pass[pos]->d_pos_m, c.d_pos_m)) fail("winner d_pos differs from log");
          for (std::size_t q = 0; q < pos; ++q) {
            bool covered = false;
            for (const auto& e : res.posthoc_log) {
              if (e.k != pass[q]->k) continue;
              double dth = std::fmod(std::abs(e.theta_rad - pass[q]->theta_rad), 2.0 * kPi);
              if (dth > kPi) dth = 2.0 * kPi - dth;
              if (dth < s3.posthoc_cluster_deg * kPi / 180.0 || bits(e.theta_rad) == bits(pass[q]->theta_rad)) {
                covered = true;
                break;
              }
            }
            if (!covered) {
              fail("row ranked before the winner is neither checked nor cluster-skipped");
              break;
            }
          }
        }
        if (!lr.k || *lr.k != c.eval.k || !lr.theta_star_rad || !close(*lr.theta_star_rad, c.eval.theta_rad) ||
            !close(lr.d_pos_m, c.d_pos_m)) {
          fail("GraspLaunchResult differs from S3Result::chosen");
        }
        std::cerr << "winner rank among log passing rows: " << pos << " (0 = first)\n";
      }
      // ---- RUN15 winner reference (only for the RUN15 scenario: default params file/snapshot/center/q-start/depth/omega) ----
      {
        auto same = [](const std::vector<double>& x, const std::vector<double>& y) {
          if (x.size() != y.size()) return false;
          for (std::size_t i = 0; i < x.size(); ++i)
            if (!(std::abs(x[i] - y[i]) < 1e-12)) return false;
          return true;
        };
        const Args def;
        bool q_same = (a.q_start == def.q_start);
        if (!q_same && a.q_start != "ready") q_same = same(parseList(a.q_start, 7, "--q-start"), parseList(def.q_start, 7, "q"));
        bool params_same = !a.params_explicit;
        if (!params_same) {
          std::error_code ec;
          params_same = std::filesystem::equivalent(a.params, resolveDefaultParams(), ec) && !ec;
        }
        const bool run15 = params_same && q_same && std::abs(a.theta_snap_rad - kRun15ThetaSnapRad) < 1e-12 &&
                           std::abs(a.t_snap_s - kRun15TSnapS) < 1e-12 && std::abs(a.depth - def.depth) < 1e-12 &&
                           std::abs(a.omega_deg_s - def.omega_deg_s) < 1e-12 &&
                           same(parseList(a.center, 3, "--center"), parseList(def.center, 3, "c"));
        if (!run15) {
          std::cerr << "RUN15 reference non applicabile (scenario modificato)\n";
        } else if (res.status != S3Result::kSelected) {
          fail("RUN15: selector did not select a candidate (status=" + std::to_string(static_cast<int>(res.status)) + ")");
        } else {
          const S3Candidate& c = res.chosen;
          struct Ref {
            const char* name;
            double got, want, tol;
          };
          const Ref refs[] = {
              {"grasp_radius_m", cube.grasp_radius_m.value(), kRun15GraspRadiusM, 1e-9},
              {"k", static_cast<double>(static_cast<int>(c.eval.k)), 0.0, 0.0},
              {"theta_deg", c.eval.theta_rad * 180.0 / kPi, -163.5, 0.05},
              {"d_pos_mm", c.d_pos_m * 1000.0, 7.1713, 0.001},
              {"wait_s", c.wait_s, 9.3548, 0.01},
              {"w_hat", c.eval.w_hat, 0.8238, 1e-3},
          };
          std::cerr << "RUN15 reference check (got / want / tol):\n";
          for (const Ref& r : refs) {
            const bool ok = std::abs(r.got - r.want) <= r.tol;
            std::cerr << std::setprecision(10) << "  " << (ok ? "ok   " : "FAIL ") << r.name << ": " << r.got << " / "
                      << r.want << " / " << r.tol << "\n";
            if (!ok) fail(std::string("RUN15 ") + r.name + " = " + std::to_string(r.got) + ", expected " + std::to_string(r.want));
          }
        }
      }
      std::cerr << (fails == 0 ? "SELFTEST OK" : "SELFTEST FAILED") << " (" << fails << " failures)\n";
      return fails == 0 ? 0 : 3;
    }

    std::ofstream out(a.out);
    if (!out) throw std::runtime_error("cannot write " + a.out);
    out << std::setprecision(10) << kCsvHeader << '\n';
    for (const Row& r : rows) writeRow(out, r);
    std::cerr << "wrote " << a.out << " (" << rows.size() << " rows)\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "s3_landscape: " << e.what() << "\n";
    return 2;
  }
}
