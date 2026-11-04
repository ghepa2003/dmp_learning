// s3_landscape: exports the "landscape" of the S3 selector, one CSV row per (k, theta, psi), also for rejected
// candidates, for OFFLINE evaluation of alternative cost formulations (tools/s3_landscape_report.py).
// Nothing here changes the planner: selectGraspS3 is called as is (with S3Context::eval_log on), psi-rolled
// evaluations call the same evaluateGraspPoseDetailed / s3AprioriRejectMaskV2 / s3DPos / planLaunch /
// checkKinematicFeasibility / solvePoseIk the planner uses. Build/run: analysis/README.md (BUILD_ANALYSIS_TOOLS).
//
// psi = deliberate roll about the LOCAL z of the TCP = the approach axis z_tcp = -R(theta) n_body
// (CubeSatelliteModel::graspPoseAt, cube_satellite_model.cpp:61,72). The TCP position does not depend on psi.
// At psi = 0 the construction is "in the collar frame": z_tcp = -face normal, y_tcp = -r_k (radial); the Franka
// fingers close along the hand y (URDF fer_finger_joint1 axis 0 1 0; fer_hand_tcp_joint has rpy 0), i.e. along the
// radial direction, so the pads are parallel to the (tangential) wall. The tool checks this numerically
// (collar_frame_check) and prints the result.

#include <algorithm>
#include <chrono>
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
#include <regex>
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

// Run of 2026-10-15 (tools/s1_run.py:74, _data/analysis/analysis_s1_1015/calc_top5_final2.cpp:31-46).
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
  int dynamic_follow = -1;   // --dynamic-follow 0|1; -1 = s1.use_dynamic_follow of --params (absent = false)
  // ---- grasp_planner_node configuration (all off/absent = the behaviour of the tool before these options existed) ----
  bool node_config = false;     // --node-config: read every relevant s1.* key of --params as grasp_planner_node does (see readNodeS1)
  bool all_candidates = false;  // --all-candidates: every finalist with every constraint recorded, not used to discard (implies --node-config)
  std::string axis = "0,0,1";   // --axis x,y,z: rotation axis of the satellite (normalized; the cube face normal must be perpendicular to it)
  bool axis_given = false;      // --axis on the command line
  // --axis-tilt-deg ALPHA / --axis-lean-deg PHI: axis and face normal with the formulas of tools/s1_run.py (s3_landscape::axisTilt); exclusive with --axis
  double axis_tilt_deg = 0.0, axis_lean_deg = 0.0;
  bool tilt_given = false;      // --axis-tilt-deg or --axis-lean-deg on the command line
  std::string scenario_tag;     // --scenario-tag: value of the `scenario` column; empty = built from the scenario options
  int force_k = -1;             // --force-k 0..3 (with --force-theta-deg): evaluate only this candidate, constraints recorded not obeyed
  double force_theta_deg = kNaN;
  int force_flip = -1;          // --force-flip -1|0|1
  std::string meas_margins;          // --meas-margins <csv> (with --all-candidates): measured L2 margins, format of out17/inputs/meas_margins.csv
  double margin_threshold_deg = 0.0;  // --margin-threshold-deg: threshold of false_safe / false_alarm
  int finalists_n_scan = -1;          // --finalists-n N (with --all-candidates): finalists_n of the scan; -1 = s1.finalists_n
  bool min_delay_given = false, depth_given = false, q_start_given = false, posthoc_given = false, theta_step_given = false;
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
    else if (k == "--q-start") {
      a.q_start = val();
      a.q_start_given = true;
    }
    else if (k == "--omega-deg-s")
      a.omega_deg_s = std::stod(val());
    else if (k == "--depth") {
      a.depth = std::stod(val());
      a.depth_given = true;
    }
    else if (k == "--follow-s")
      a.follow_s = std::stod(val());
    else if (k == "--psi-max-deg")
      a.psi_max_deg = std::stod(val());
    else if (k == "--psi-step-deg")
      a.psi_step_deg = std::stod(val());
    else if (k == "--theta-step-deg") {
      a.theta_step_deg = std::stod(val());
      a.theta_step_given = true;
    }
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
    else if (k == "--min-delay-s") {
      a.min_delay_s = std::stod(val());
      a.min_delay_given = true;
    }
    else if (k == "--dynamic-follow")
      a.dynamic_follow = std::stoi(val());
    else if (k == "--posthoc-velocity") {
      a.posthoc_velocity = std::stoi(val());
      a.posthoc_given = true;
    } else if (k == "--node-config")
      a.node_config = true;
    else if (k == "--all-candidates")
      a.all_candidates = true;
    else if (k == "--axis") {
      a.axis = val();
      a.axis_given = true;
    } else if (k == "--axis-tilt-deg") {
      a.axis_tilt_deg = std::stod(val());
      a.tilt_given = true;
    } else if (k == "--axis-lean-deg") {
      a.axis_lean_deg = std::stod(val());
      a.tilt_given = true;
    }
    else if (k == "--scenario-tag")
      a.scenario_tag = val();
    else if (k == "--meas-margins")
      a.meas_margins = val();
    else if (k == "--margin-threshold-deg")
      a.margin_threshold_deg = std::stod(val());
    else if (k == "--finalists-n")
      a.finalists_n_scan = std::stoi(val());
    else if (k == "--force-k")
      a.force_k = std::stoi(val());
    else if (k == "--force-theta-deg")
      a.force_theta_deg = std::stod(val());
    else if (k == "--force-flip")
      a.force_flip = std::stoi(val());
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
  if (a.all_candidates) a.node_config = true;
  if (a.axis_given && a.tilt_given) {
    throw std::invalid_argument("--axis and --axis-tilt-deg/--axis-lean-deg are alternatives: give only one way to set the satellite axis");
  }
  if (a.tilt_given) (void)s3_landscape::axisTilt(a.axis_tilt_deg, a.axis_lean_deg);  // range check at parse time
  if ((a.force_k >= 0) != std::isfinite(a.force_theta_deg) || a.force_k > 3 || a.force_flip < -1 || a.force_flip > 1) {
    throw std::invalid_argument("--force-k (0..3) and --force-theta-deg go together; --force-flip is -1, 0 or 1");
  }
  if (!a.all_candidates && (!a.meas_margins.empty() || a.finalists_n_scan >= 0)) {
    throw std::invalid_argument("--meas-margins and --finalists-n need --all-candidates");
  }
  if (a.finalists_n_scan == 0) throw std::invalid_argument("--finalists-n must be >= 1");
  if (a.all_candidates && a.force_k >= 0) throw std::invalid_argument("--all-candidates and --force-k are alternatives");
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

/// Per-finalist tail of the CSV, only with the L2 dynamic follow on (S3FinalistRecord::dyn_*). Selection-level values
/// (dynamic_evals_done, selection_ms, ...) go to <out>.meta and stderr.
const char* kCsvHeaderDyn =
    ",dyn_evaluated,dyn_approach_margin_deg,dyn_hold_margin_deg,dyn_total_margin_deg,dyn_discard_reason,dyn_winner,"
    "dyn_q_end_j1_deg,dyn_q_end_j2_deg,dyn_q_end_j3_deg,dyn_q_end_j4_deg,dyn_q_end_j5_deg,dyn_q_end_j6_deg,dyn_q_end_j7_deg,"
    "dyn_q_h1p5_j1_deg,dyn_q_h1p5_j2_deg,dyn_q_h1p5_j3_deg,dyn_q_h1p5_j4_deg,dyn_q_h1p5_j5_deg,dyn_q_h1p5_j6_deg,dyn_q_h1p5_j7_deg,"
    "dyn_q_h3_j1_deg,dyn_q_h3_j2_deg,dyn_q_h3_j3_deg,dyn_q_h3_j4_deg,dyn_q_h3_j5_deg,dyn_q_h3_j6_deg,dyn_q_h3_j7_deg,"
    "dyn_pose_err_pos_t0_m,dyn_pose_err_rot_t0_deg,dyn_max_abs_dq_from_qend_deg,dyn_t_first_exceed_s,dyn_elapsed_ms";
constexpr int kCsvDynFields = 32;

/// Tail of one row: the finalist with the same (k, theta) of the psi = 0 coarse grid, else empty fields.
std::string dynTail(const S3FinalistRecord* f) {
  std::ostringstream o;
  o << std::setprecision(10);
  if (!f) {
    for (int i = 0; i < kCsvDynFields; ++i) o << ',';
    return o.str();
  }
  o << ',' << (f->dyn_done ? 1 : 0) << ',';
  writeNum(o, f->dyn_approach_margin_deg);
  o << ',';
  writeNum(o, f->dyn_hold_margin_deg);
  o << ',';
  writeNum(o, f->dyn_total_margin_deg);
  o << ',' << f->discard_reason << ',' << (f->winner ? 1 : 0);
  for (const RobotModel::JointVector* q : {&f->dyn_q_end_deg, &f->dyn_q_h1p5_deg, &f->dyn_q_h3_deg}) {
    for (int j = 0; j < RobotModel::kNumJoints; ++j) {
      o << ',';
      writeNum(o, (*q)(j));
    }
  }
  for (const double x : {f->dyn_pose_err_pos_t0_m, f->dyn_pose_err_rot_t0_deg, f->dyn_max_abs_dq_from_qend_deg,
                         f->dyn_t_first_exceed_s, f->dyn_elapsed_ms}) {
    o << ',';
    writeNum(o, x);
  }
  return o.str();
}


/// --node-config: the s1.* keys grasp_planner_node.cpp declares (lines 148-222 there), read from the `grasp_planner_node:` block of
/// --params into @p p. A key that is absent keeps the code default (the node's own default); a value that is not a number /
/// true|false is an error. s1.rollout_q_ref is mandatory, as in the node. min_delay_s and cube_geometry.grasp_depth_m are read by main().
void readNodeS1(const std::map<std::string, std::string>& kv, S3Params& p) {
  auto dbl = [&](const char* key, double& dst) {
    const auto it = kv.find(key);
    if (it != kv.end()) dst = std::stod(it->second);
  };
  auto bol = [&](const char* key, bool& dst) {
    const auto it = kv.find(key);
    if (it == kv.end()) return;
    if (it->second != "true" && it->second != "false") throw std::invalid_argument(std::string(key) + " must be true or false");
    dst = it->second == "true";
  };
  auto integer = [&](const char* key, int& dst) {
    const auto it = kv.find(key);
    if (it != kv.end()) dst = std::stoi(it->second);
  };
  dbl("s1.length_scale_m", p.length_scale_m);
  dbl("s1.orientation_weight", p.orientation_weight);
  dbl("s1.angle_scale_rad", p.angle_scale_rad);
  dbl("s1.r_trust_m", p.r_trust_m);
  dbl("s1.delta_v_max_m_s", p.delta_v_max_m_s);
  dbl("s1.theta_step_deg", p.theta_step_deg);
  bol("s1.posthoc_velocity_check", p.posthoc_velocity_check);
  dbl("s1.posthoc_cluster_deg", p.posthoc_cluster_deg);
  dbl("s1.ik.pos_tol_m", p.ik.pos_tol_m);
  dbl("s1.ik.ori_tol_rad", p.ik.ori_tol_rad);
  dbl("s1.ik.damping", p.ik.damping);
  dbl("s1.ik.nullspace_gain", p.ik.nullspace_gain);
  dbl("s1.ik.max_step_rad", p.ik.max_step_rad);
  integer("s1.ik.max_iter", p.ik.max_iter);
  integer("s1.max_posthoc_candidates", p.max_posthoc_candidates);
  if (kv.count("s1.branch_rule")) p.branch_rule = kv.at("s1.branch_rule");
  dbl("s1.max_start_goal_deg", p.max_start_goal_deg);
  integer("s1.finalists_n", p.finalists_n);
  if (!kv.count("s1.rollout_q_ref")) throw std::runtime_error("--node-config: --params has no s1.rollout_q_ref (mandatory, as in the node)");
  p.rollout_q_ref = yamlList(kv.at("s1.rollout_q_ref"));
  bol("s1.use_w", p.use_w);
  dbl("s1.kappa_w", p.kappa_w_mm);
  dbl("s1.kappa_t", p.kappa_t_mm_s);
  dbl("s1.tau_j_mm", p.tau_j_mm);
  bol("s1.use_clearance", p.use_clearance);
  dbl("s1.min_clearance_mm", p.min_clearance_mm);
  dbl("s1.clearance_dt_s", p.clearance_dt_s);
  dbl("s1.gripper_open_m", p.gripper_open_m);
  bol("s1.use_margin", p.use_margin);
  dbl("s1.min_margin_deg", p.min_margin_deg);
  dbl("s1.margin_follow_s", p.margin_follow_s);
  dbl("s1.margin_follow_dt_s", p.margin_follow_dt_s);
  bol("s1.lazy_static_order", p.lazy_static_order);
  bol("s1.use_dynamic_follow", p.use_dynamic_follow);
  dbl("s1.margin_delta_deg", p.margin_delta_deg);
  bol("s1.use_hold_end", p.use_hold_end);
  dbl("s1.hold_follow_s", p.hold_follow_s);
  DynamicFollowParams& df = p.dynamic_follow;
  dbl("s1.follow_translational_stiffness", df.translational_stiffness);
  dbl("s1.follow_rotational_stiffness", df.rotational_stiffness);
  dbl("s1.follow_translational_damping", df.translational_damping);
  dbl("s1.follow_rotational_damping", df.rotational_damping);
  dbl("s1.follow_nullspace_stiffness", df.nullspace_stiffness);
  dbl("s1.follow_joint1_nullspace_stiffness", df.joint1_nullspace_stiffness);
  dbl("s1.follow_nullspace_pinv_damping", df.nullspace_pinv_damping);
  dbl("s1.follow_delta_tau_max", df.delta_tau_max);
  bol("s1.follow_compensate_gravity", df.compensate_gravity_internally);
  dbl("s1.follow_plant_gravity_z", df.plant_gravity_z_m_s2);
  dbl("s1.follow_friction_eps", df.friction_tanh_eps_rad_s);
  bol("s1.follow_apply_urdf_damping", df.apply_urdf_damping);
  bol("s1.follow_apply_urdf_friction", df.apply_urdf_friction);
  dbl("s1.follow_dt_s", df.dt_s);
  bol("s1.follow_abort_on_divergence", df.abort_on_divergence);
  dbl("s1.follow_divergence_rad", df.divergence_rad);
  dbl("s1.follow_finger_pos_m", df.finger_pos_m);
  bol("s1.follow_dq0_zero", df.dq0_zero);
}

/// Why a finalist would have been discarded by the production (lazy) flow, from the constraints recorded for it, in the lazy order:
/// rollout (V1), then with L2: clearance (V2), V1', velocity, L2; without L2: V1', margin, clearance, velocity. "admissible" if none.
/// @p velocity_ok: -1 not evaluated, 0 failed, 1 passed.
std::string lazyReason(const S3FinalistRecord& f, const S3Params& s3, int velocity_ok) {
  if (!f.rollout_feasible) return "rollout";
  const bool hold_end_bad = s3.use_hold_end && f.hold_end_done && !f.end_ok;
  const bool clearance_bad = s3.use_clearance && f.clearance_done && f.clearance_rejected;
  if (s3.use_dynamic_follow) {
    if (clearance_bad) return "clearance";
    if (hold_end_bad) return "hold_end_infeasible";
    if (velocity_ok == 0) return "velocity";
    if (f.dyn_done && f.dyn_diverged) return "dynamic_diverged";
    if (f.dyn_done && f.dyn_rejected) return f.dyn_approach_margin_deg >= s3.margin_delta_deg ? "dynamic_hold" : "dynamic_approach";
    return "admissible";
  }
  if (hold_end_bad) return "hold_end_infeasible";
  if (s3.use_margin && f.margin_done && f.margin_rejected) return "margin";
  if (clearance_bad) return "clearance";
  if (velocity_ok == 0) return "velocity";
  return "admissible";
}

/// One measured row of meas_margins.csv (out17/inputs/meas_margins.csv). Columns read: `run_id` and `tot_min_margin_deg` (the minimum
/// joint margin over approach and hold). The candidate is taken from run_id: `k<0..3><A|B>_th<deg>` (e.g. forced_k1A_th-156,
/// g20_run1_k1A_th-166); a run_id without that pattern (e.g. clr_w-5.0) is skipped and counted.
struct MeasMargin {
  int k = 0;
  char branch = 'A';
  double theta_deg = 0.0;
  double total_deg = 0.0;
  bool sat = false;  // saturation_real == YES
};

std::vector<MeasMargin> readMeasMargins(const std::string& path, int& n_skipped, int& n_sat_blank) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open --meas-margins " + path);
  std::string line;
  std::getline(f, line);
  auto split = [](const std::string& l) {
    std::vector<std::string> v;
    std::stringstream ss(l);
    std::string t;
    while (std::getline(ss, t, ',')) v.push_back(trim(t));
    return v;
  };
  const auto head = split(line);
  const auto col = [&](const char* name) {
    const auto it = std::find(head.begin(), head.end(), name);
    if (it == head.end()) throw std::runtime_error(path + ": column '" + name + "' not found in the header");
    return static_cast<std::size_t>(it - head.begin());
  };
  const std::size_t c_run = col("run_id"), c_tot = col("tot_min_margin_deg"), c_sat = col("saturation_real");
  const std::regex re("k([0-3])([AB])_th(-?[0-9]+(\\.[0-9]+)?)");
  std::vector<MeasMargin> out;
  n_skipped = 0;
  n_sat_blank = 0;
  while (std::getline(f, line)) {
    if (trim(line).empty()) continue;
    const auto v = split(line);
    std::smatch m;
    if (v.size() <= std::max({c_run, c_tot, c_sat}) || !std::regex_search(v[c_run], m, re)) {
      ++n_skipped;
      continue;
    }
    if (v[c_sat] != "YES" && v[c_sat] != "NO") {  // blank / other: not associable, counted apart
      ++n_sat_blank;
      continue;
    }
    MeasMargin r;
    r.k = std::stoi(m[1].str());
    r.branch = m[2].str()[0];
    r.theta_deg = std::stod(m[3].str());
    r.total_deg = std::stod(v[c_tot]);
    r.sat = v[c_sat] == "YES";
    out.push_back(r);
  }
  return out;
}

/// --all-candidates: one CSV row per finalist of @p scan (the finalists of the scan with finalists_n = @p n_scan; the production
/// selection @p res_prod ran with @p n_prod). Each finalist is re-evaluated alone with S3Params::force (k, theta, wrist branch of the
/// finalist; force_ignore_constraints), which runs the NON-lazy branch of selectGraspS3: rollout, V1', L2 (also for a finalist the lazy
/// flow would have discarded earlier), clearance and the velocity check are computed and recorded, none discards. Not computed by that
/// branch: V2 / V1' / velocity of a finalist whose V1 rollout fails (cells left empty). lazy_outcome is read from @p res_prod (the
/// production selection already run), not recomputed. The d_mm / w_static of the forced run are written next to the scan's (repro_*).
/// With --meas-margins: err_margin_deg = l2_total_deg - measured; measured_violation = (saturation_real == YES) if T == 0.0 exactly
/// (the measured margin is truncated at 0, the CSV flags it with tot_min_margin_deg <= 0.05, extract_inputs_36.py:293), else
/// (measured < T); false_safe = (l2_total >= T) and measured_violation; false_alarm = (l2_total < T) and not measured_violation;
/// T = --margin-threshold-deg. Key of the association: (k, branch, theta within 0.5 deg of the
/// theta in the run_id); with several measured rows (repetitions) the minimum is used and meas_n is their number.
void writeAllCandidates(const Args& a, const SelectionInputs& in, const SatelliteSnapshot& snap, double min_delay, const S3Params& s3,
                        const ApproachClearanceModel* clearance, const JointDynamics* dynamics, const S3Result& res_prod,
                        const std::vector<S3FinalistRecord>& scan, int n_prod, int n_scan, const std::string& scenario) {
  const double kPi = haptic_dmp_learning::core::kPi;
  static const char* kCols[] = {
      "scenario", "finalists_n_production", "finalists_n_scan", "in_production_list", "rank", "k", "branch", "theta_deg", "J_s", "d_mm",
      "w_static", "w_rollout", "w_end", "w_eff", "v1_ok", "start_goal_deg", "clearance_mm", "clearance_pair", "clearance_t_rel_s",
      "min_clearance_mm", "v2_ok", "use_hold_end", "v1p_ok", "vel_dv_m_s", "vel_ok", "l2_approach_deg", "l2_hold_deg", "l2_total_deg",
      "margin_delta_deg", "dyn_vs_rollout_qend_deg", "dyn_initial_rot_err_deg", "l2_diverged", "lazy_reason", "lazy_outcome",
      "v1_ms", "v1p_ms", "clearance_ms", "l2_ms", "call_ms", "repro_d_mm_diff", "repro_w_static_diff", "meas_margin_deg", "meas_sat", "meas_n",
      "err_margin_deg", "false_safe", "false_alarm", "error"};
  int n_meas_skipped = 0, n_sat_blank = 0, n_assoc = 0, n_false_safe = 0, n_false_alarm = 0;
  std::vector<MeasMargin> meas;
  if (!a.meas_margins.empty()) meas = readMeasMargins(a.meas_margins, n_meas_skipped, n_sat_blank);
  const double thr = a.margin_threshold_deg;

  auto fmt = [](double v) {
    std::ostringstream o;
    if (std::isfinite(v)) o << std::setprecision(10) << v;
    return o.str();
  };
  auto yn = [](bool v) { return std::string(v ? "1" : "0"); };
  struct Out {
    std::map<std::string, std::string> v;
    int prod_index = -1;
  };
  std::vector<Out> rows;
  int rank = 0;
  for (const S3FinalistRecord& f0 : scan) {
    Out o;
    auto& v = o.v;
    S3Params pf = s3;
    pf.force = true;
    pf.force_k = static_cast<int>(f0.cand.eval.k);
    pf.force_theta_deg = f0.cand.eval.theta_rad * 180.0 / kPi;
    pf.force_psi_deg = 0.0;
    pf.force_flip = f0.cand.eval.wrist_flip ? 1 : 0;
    pf.force_ignore_constraints = true;
    S3Result r;
    std::string err;
    const auto t0 = std::chrono::steady_clock::now();
    try {
      planGraspAndLaunchS1(in, snap, min_delay, pf, snap.t_s, &r, nullptr, clearance, dynamics);
    } catch (const std::exception& e) {
      err = e.what();
      std::replace(err.begin(), err.end(), ',', ';');
      std::replace(err.begin(), err.end(), '\n', ' ');
    }
    const double call_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    const S3FinalistRecord* f = r.v2_finalists.empty() ? nullptr : &r.v2_finalists[0];
    if (!f && err.empty()) err = "no finalist record";
    v["scenario"] = scenario;
    v["finalists_n_production"] = std::to_string(n_prod);
    v["finalists_n_scan"] = std::to_string(n_scan);
    v["rank"] = std::to_string(rank++);
    v["k"] = std::to_string(static_cast<int>(f0.cand.eval.k));
    v["branch"] = std::string(1, f0.cand.eval.wrist_flip ? 'B' : 'A');
    const double theta_deg = f0.cand.eval.theta_rad * 180.0 / kPi;
    v["theta_deg"] = fmt(theta_deg);
    v["J_s"] = fmt(s3CostV2(f0.d_mm, f0.w_static, 0.0, s3));
    v["d_mm"] = fmt(f0.d_mm);
    v["w_static"] = fmt(f0.w_static);
    v["min_clearance_mm"] = fmt(s3.min_clearance_mm);
    v["use_hold_end"] = yn(s3.use_hold_end);
    v["margin_delta_deg"] = fmt(s3.margin_delta_deg);
    v["error"] = err;
    // the finalist of the production selection (same k, branch, theta), if any
    for (std::size_t i = 0; i < res_prod.v2_finalists.size(); ++i) {
      const S3FinalistRecord& p = res_prod.v2_finalists[i];
      if (p.cand.eval.k == f0.cand.eval.k && p.cand.eval.wrist_flip == f0.cand.eval.wrist_flip &&
          std::fabs(p.cand.eval.theta_rad - f0.cand.eval.theta_rad) < 1e-9) {
        o.prod_index = static_cast<int>(i);
        break;
      }
    }
    v["in_production_list"] = yn(o.prod_index >= 0);
    if (o.prod_index < 0) {
      v["lazy_outcome"] = "not_in_production_list";
    } else {
      const S3FinalistRecord& p = res_prod.v2_finalists[static_cast<std::size_t>(o.prod_index)];
      if (p.winner)
        v["lazy_outcome"] = "winner";
      else if (!p.discard_reason.empty())
        v["lazy_outcome"] = "discarded:" + p.discard_reason;
      else if (!p.rollout_done)
        v["lazy_outcome"] = "not_reached";
      else
        v["lazy_outcome"] = "evaluated_not_selected";
    }
    if (f) {
      int vel = -1;  // -1 not evaluated
      double dv = kNaN;
      if (s3.posthoc_velocity_check && !r.posthoc_log.empty()) {
        vel = r.posthoc_log.front().reject_reason.empty() ? 1 : 0;
        dv = r.posthoc_log.front().dv_m_s;
      }
      if (f->rollout_done) {
        v["w_rollout"] = fmt(f->w_rollout);
        v["v1_ok"] = yn(f->rollout_feasible);
      }
      v["start_goal_deg"] = fmt(f->start_goal_deg);
      if (f->hold_end_done) {
        v["w_end"] = fmt(f->w_end);
        v["v1p_ok"] = yn(f->end_ok);
        if (f->end_ok) v["w_eff"] = fmt(f->w_eff);
      }
      if (f->clearance_done) {
        v["clearance_mm"] = fmt(f->clearance_mm);
        v["clearance_pair"] = f->clearance_pair;
        v["clearance_t_rel_s"] = fmt(f->clearance_t_rel_s);
        v["v2_ok"] = yn(!f->clearance_rejected);
      }
      v["vel_dv_m_s"] = fmt(dv);
      if (vel >= 0) v["vel_ok"] = yn(vel == 1);
      if (f->dyn_done) {
        v["l2_approach_deg"] = fmt(f->dyn_approach_margin_deg);
        v["l2_hold_deg"] = fmt(f->dyn_hold_margin_deg);
        v["l2_total_deg"] = fmt(f->dyn_total_margin_deg);
        v["l2_diverged"] = yn(f->dyn_diverged);
        v["l2_ms"] = fmt(f->dyn_elapsed_ms);
      }
      v["dyn_vs_rollout_qend_deg"] = fmt(f->dyn_vs_rollout_qend_deg);
      v["dyn_initial_rot_err_deg"] = fmt(f->dyn_initial_rot_err_deg);
      v["lazy_reason"] = lazyReason(*f, s3, vel);
      v["v1_ms"] = fmt(f->rollout_ms);
      if (f->hold_end_done) v["v1p_ms"] = fmt(f->hold_end_ms);
      v["clearance_ms"] = fmt(r.clearance_stage_ms);
      v["call_ms"] = fmt(call_ms);
      v["repro_d_mm_diff"] = fmt(f->d_mm - f0.d_mm);
      v["repro_w_static_diff"] = fmt(f->w_static - f0.w_static);
      if (!meas.empty()) {
        int n = 0;
        double m = 0.0;
        bool sat = false;
        for (const MeasMargin& mm : meas) {
          if (mm.k == static_cast<int>(f0.cand.eval.k) && mm.branch == (f0.cand.eval.wrist_flip ? 'B' : 'A') &&
              std::fabs(mm.theta_deg - theta_deg) <= 0.5) {
            m = n == 0 ? mm.total_deg : std::min(m, mm.total_deg);
            sat = sat || mm.sat;
            ++n;
          }
        }
        v["meas_n"] = std::to_string(n);
        if (n > 0) {
          v["meas_margin_deg"] = fmt(m);
          v["meas_sat"] = sat ? "YES" : "NO";
          ++n_assoc;
          if (f->dyn_done && std::isfinite(f->dyn_total_margin_deg)) {
            const double pred = f->dyn_total_margin_deg;
            v["err_margin_deg"] = fmt(pred - m);
            const bool violated = thr == 0.0 ? sat : (m < thr);
            const bool fs = pred >= thr && violated, fa = pred < thr && !violated;
            v["false_safe"] = yn(fs);
            v["false_alarm"] = yn(fa);
            n_false_safe += fs ? 1 : 0;
            n_false_alarm += fa ? 1 : 0;
          }
        }
      }
    }
    rows.push_back(std::move(o));
  }

  std::ofstream out(a.out);
  if (!out) throw std::runtime_error("cannot write " + a.out);
  for (std::size_t i = 0; i < sizeof(kCols) / sizeof(kCols[0]); ++i) out << (i ? "," : "") << kCols[i];
  out << '\n';
  for (const Out& o : rows) {
    for (std::size_t i = 0; i < sizeof(kCols) / sizeof(kCols[0]); ++i) {
      const auto it = o.v.find(kCols[i]);
      out << (i ? "," : "") << (it == o.v.end() ? "" : it->second);
    }
    out << '\n';
  }

  // Self-check: the production winner is the first finalist (J_static order, among those of the production list) whose columns all pass.
  auto tag = [&](const S3FinalistRecord& p) {
    std::ostringstream t;
    t << "k" << static_cast<int>(p.cand.eval.k) << (p.cand.eval.wrist_flip ? 'B' : 'A') << "_th" << std::fixed << std::setprecision(1)
      << p.cand.eval.theta_rad * 180.0 / kPi;
    return t.str();
  };
  std::string winner_tag = "none", first_tag = "none";
  for (const S3FinalistRecord& p : res_prod.v2_finalists) {
    if (p.winner) winner_tag = tag(p);
  }
  for (const Out& o : rows) {
    if (o.prod_index < 0) continue;
    const auto it = o.v.find("lazy_reason");
    if (it != o.v.end() && it->second == "admissible") {
      first_tag = tag(res_prod.v2_finalists[static_cast<std::size_t>(o.prod_index)]);
      break;
    }
  }
  const std::string check = std::string(winner_tag == first_tag ? "OK" : "MISMATCH") + " production_winner=" + winner_tag +
                            " first_admissible_in_scan=" + first_tag;
  std::cerr << "autocheck: " << check << "\n";
  std::ofstream mf(a.out + ".meta", std::ios::app);
  mf << "autocheck=" << check << "\nfinalists_n_production=" << n_prod << "\nfinalists_n_scan=" << n_scan
     << "\nmin_clearance_mm=" << s3.min_clearance_mm << "\nmargin_threshold_deg=" << thr << "\n";
  if (!a.meas_margins.empty()) {
    mf << "meas_margins=" << a.meas_margins << "\nmeas_rows_used=" << meas.size() << "\nmeas_rows_skipped_run_id=" << n_meas_skipped
       << "\nmeas_rows_skipped_saturation_blank=" << n_sat_blank << "\nfinalists_associated=" << n_assoc
       << "\nfalse_safe=" << n_false_safe << "\nfalse_alarm=" << n_false_alarm
       << "\nmeasured_violation_definition=" << (thr == 0.0 ? "saturation_real==YES" : "meas_margin_deg<T")
       << " (predicted_ok: l2_total>=T)\n";
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    Args a = parseArgs(argc, argv);
    const double kPi = haptic_dmp_learning::core::kPi;

    // ---- planner parameters: defaults < --params YAML < CLI ----
    S3Params s3;
    CubeSatelliteModel::Params cube;
    double min_delay = a.min_delay_s;
    bool use_dyn = false;  // L2 dynamic follow (--dynamic-follow, else s1.use_dynamic_follow of --params); default off
    {
      const auto kv = readPlannerYaml(a.params);
      if (!kv.count("cube_geometry.grasp_radius_m")) {
        throw std::runtime_error("--params " + a.params + " has no cube_geometry.grasp_radius_m: refusing the code default");
      }
      auto d = [&](const char* key, double& dst) {
        const auto it = kv.find(key);
        if (it != kv.end()) dst = std::stod(it->second);
      };
      d("s1.length_scale_m", s3.length_scale_m);
      d("s1.orientation_weight", s3.orientation_weight);
      d("s1.angle_scale_rad", s3.angle_scale_rad);
      d("s1.r_trust_m", s3.r_trust_m);
      d("s1.delta_v_max_m_s", s3.delta_v_max_m_s);
      d("s1.theta_step_deg", s3.theta_step_deg);
      d("s1.posthoc_cluster_deg", s3.posthoc_cluster_deg);
      d("s1.ik.pos_tol_m", s3.ik.pos_tol_m);
      d("s1.ik.ori_tol_rad", s3.ik.ori_tol_rad);
      d("s1.ik.damping", s3.ik.damping);
      d("s1.ik.nullspace_gain", s3.ik.nullspace_gain);
      d("s1.ik.max_step_rad", s3.ik.max_step_rad);
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
      // ---- L2 dynamic follow: nothing below is read, and nothing changes, unless it is on ----
      const auto df_it = kv.find("s1.use_dynamic_follow");
      use_dyn = a.dynamic_follow >= 0 ? a.dynamic_follow != 0 : (df_it != kv.end() && df_it->second == "true");
      if (use_dyn) {
        s3.use_dynamic_follow = true;
        if (!kv.count("s1.rollout_q_ref")) {
          throw std::runtime_error("--params " + a.params + " has no s1.rollout_q_ref: mandatory for the selector (use_dynamic_follow)");
        }
        s3.rollout_q_ref = yamlList(kv.at("s1.rollout_q_ref"));
        if (kv.count("s1.finalists_n")) s3.finalists_n = std::stoi(kv.at("s1.finalists_n"));
        if (kv.count("s1.branch_rule")) s3.branch_rule = kv.at("s1.branch_rule");
        d("s1.margin_delta_deg", s3.margin_delta_deg);
        d("s1.margin_follow_s", s3.margin_follow_s);
        DynamicFollowParams& df = s3.dynamic_follow;
        d("s1.follow_translational_stiffness", df.translational_stiffness);
        d("s1.follow_rotational_stiffness", df.rotational_stiffness);
        d("s1.follow_translational_damping", df.translational_damping);
        d("s1.follow_rotational_damping", df.rotational_damping);
        d("s1.follow_nullspace_stiffness", df.nullspace_stiffness);
        d("s1.follow_joint1_nullspace_stiffness", df.joint1_nullspace_stiffness);
        d("s1.follow_nullspace_pinv_damping", df.nullspace_pinv_damping);
        d("s1.follow_delta_tau_max", df.delta_tau_max);
        d("s1.follow_plant_gravity_z", df.plant_gravity_z_m_s2);
        d("s1.follow_friction_eps", df.friction_tanh_eps_rad_s);
        d("s1.follow_dt_s", df.dt_s);
        d("s1.follow_divergence_rad", df.divergence_rad);
        d("s1.follow_finger_pos_m", df.finger_pos_m);
        auto bl = [&](const char* key, bool& dst) {
          const auto it = kv.find(key);
          if (it != kv.end()) dst = (it->second == "true");
        };
        bl("s1.follow_compensate_gravity", df.compensate_gravity_internally);
        bl("s1.follow_apply_urdf_damping", df.apply_urdf_damping);
        bl("s1.follow_apply_urdf_friction", df.apply_urdf_friction);
        bl("s1.follow_abort_on_divergence", df.abort_on_divergence);
        bl("s1.follow_dq0_zero", df.dq0_zero);
      }
      // ---- --node-config: every s1.* key of the node, then the scenario inputs the node takes from the YAML ----
      if (a.node_config) {
        readNodeS1(kv, s3);
        use_dyn = a.dynamic_follow >= 0 ? a.dynamic_follow != 0 : s3.use_dynamic_follow;
        s3.use_dynamic_follow = use_dyn;
        if (!a.min_delay_given) {
          // mandatory in the node (no default): no silent 5.0 either
          if (!kv.count("min_delay_s")) throw std::runtime_error("--node-config: --params has no min_delay_s (mandatory in the node); pass --min-delay-s");
          min_delay = std::stod(kv.at("min_delay_s"));
        }
        if (!a.depth_given) a.depth = kv.count("cube_geometry.grasp_depth_m") ? std::stod(kv.at("cube_geometry.grasp_depth_m")) : 0.0;
        if (!a.q_start_given) {
          const auto qi = kv.find(kStartJointsParameterName);
          if (qi != kv.end()) {
            std::string q = qi->second;
            q.erase(std::remove(q.begin(), q.end(), '['), q.end());
            q.erase(std::remove(q.begin(), q.end(), ']'), q.end());
            a.q_start = q;
          } else {
            a.q_start = "ready";
          }
        }
      }
    }
    if (!a.node_config || a.theta_step_given) s3.theta_step_deg = a.theta_step_deg;
    if (!a.node_config || a.posthoc_given) s3.posthoc_velocity_check = a.posthoc_velocity != 0;
    cube.grasp_depth_m = a.depth;
    if (a.force_k >= 0) {
      s3.force = true;
      s3.force_k = a.force_k;
      s3.force_theta_deg = a.force_theta_deg;
      s3.force_flip = a.force_flip;
      s3.force_ignore_constraints = true;  // the violated constraints are listed in forced_violations, the candidate is evaluated anyway
    }
    if (a.node_config) validateS3Params(s3);  // at startup, as the node does

    const auto cvec = parseList(a.center, 3, "--center");
    SatelliteSnapshot snap;
    snap.center = Eigen::Vector3d(cvec[0], cvec[1], cvec[2]);
    const bool tilt_active = a.tilt_given && a.axis_tilt_deg != 0.0;  // alpha = 0: axis z and the YAML normal, as tools/s1_run.py
    const s3_landscape::AxisTilt tilt = s3_landscape::axisTilt(a.tilt_given ? a.axis_tilt_deg : 0.0, a.tilt_given ? a.axis_lean_deg : 0.0);
    if (tilt_active) {
      snap.axis = tilt.axis;
      cube.face_normal_body = tilt.face_normal;
    } else {
      const auto av = parseList(a.axis, 3, "--axis");
      const Eigen::Vector3d ax(av[0], av[1], av[2]);
      if (!(ax.norm() > 1e-9)) throw std::invalid_argument("--axis has ~zero norm");
      snap.axis = ax.normalized();  // (0,0,1) -> UnitZ exactly
    }
    // CubeSatelliteModel throws unless face_normal_body is perpendicular to the axis (|n . axis| <= 1e-6, cube_satellite_model.hpp:70). For an
    // inclined --axis the normal of the YAML is projected on the plane perpendicular to the axis and normalized (reported on stderr / .meta).
    bool face_normal_projected = false;
    {
      Eigen::Vector3d n = cube.face_normal_body.normalized();
      const double dot = n.dot(snap.axis);
      if (std::fabs(dot) > 1e-9) {
        Eigen::Vector3d np = n - dot * snap.axis;
        if (!(np.norm() > 1e-6)) throw std::invalid_argument("face_normal_body is parallel to --axis: no lateral normal to project");
        cube.face_normal_body = np.normalized();
        face_normal_projected = true;
        std::cerr << "face_normal_body projected perpendicular to --axis: " << cube.face_normal_body.transpose() << "\n";
      }
    }
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
        << "\ndepth=" << cube.grasp_depth_m << "\naxis=" << snap.axis.transpose() << "\nface_normal_body=" << cube.face_normal_body.transpose() << "\nface_normal_projected=" << (face_normal_projected ? 1 : 0)
        << "\naxis_tilt_deg=" << a.axis_tilt_deg << "\naxis_lean_deg=" << a.axis_lean_deg << "\nq_ref_xyzw=" << tilt.q_ref.x() << " "
        << tilt.q_ref.y() << " " << tilt.q_ref.z() << " " << tilt.q_ref.w()
        << "\nnode_config=" << (a.node_config ? 1 : 0)
        << "\nomega_deg_s=" << a.omega_deg_s << "\ncenter=" << a.center
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
    // Per-joint URDF <dynamics> for the L2 follow, from --urdf (same default as grasp_planner_node's urdf_path); only if it is on.
    JointDynamics joint_dynamics;
    if (use_dyn) {
      std::vector<std::string> arm_joints;
      for (int j = 1; j <= 7; ++j) arm_joints.push_back("fer_joint" + std::to_string(j));
      joint_dynamics = loadJointDynamicsFromUrdfFile(a.urdf, arm_joints);
    }
    // fer_hand collision mesh + clearance model, built as grasp_planner_node.cpp:268-283 does (only with s1.use_clearance, i.e. --node-config).
    std::unique_ptr<ApproachClearanceModel> clearance_model;
    if (s3.use_clearance) {
      clearance_model = std::make_unique<ApproachClearanceModel>(cube, loadHandCollisionMesh(a.urdf));
    }
    const auto sel_t0 = std::chrono::steady_clock::now();
    const GraspLaunchResult lr = planGraspAndLaunchS1(in, snap, min_delay, s3, snap.t_s, &res, &log, clearance_model.get(),
                                                      use_dyn ? &joint_dynamics : nullptr);
    const double selection_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - sel_t0).count();
    std::cerr << "selectGraspS3: status=" << res.status << " evaluations=" << res.evaluations
              << " log_rows=" << log.size() << " posthoc_checks=" << res.posthoc_checks << " note: " << res.note << "\n";
    if (use_dyn) {
      int n_discarded = 0;
      for (const S3FinalistRecord& f : res.v2_finalists) n_discarded += f.discard_reason.empty() ? 0 : 1;
      std::cerr << "dynamic follow: dynamic_evals_done=" << res.dynamic_evals_done << " rollouts_done=" << res.rollouts_done
                << " finalists=" << res.v2_finalists.size() << " discarded=" << n_discarded << " selection_ms=" << selection_ms
                << " margin_delta_deg=" << s3.margin_delta_deg << "\n";
      for (std::size_t i = 0; i < res.v2_finalists.size(); ++i) {
        const S3FinalistRecord& f = res.v2_finalists[i];
        if (!f.rollout_done) continue;
        std::cerr << "  finalist " << i << " k=" << static_cast<int>(f.cand.eval.k) << " theta_deg="
                  << f.cand.eval.theta_rad * 180.0 / kPi << " dyn_done=" << f.dyn_done << " approach=" << f.dyn_approach_margin_deg
                  << " hold=" << f.dyn_hold_margin_deg << " total=" << f.dyn_total_margin_deg << " reason='" << f.discard_reason
                  << "'" << (f.winner ? " WINNER" : "") << "\n";
      }
      {
        const S3Result::DynamicCounters& n = res.dyn_counters;
        std::cerr << "counters: rolled_out=" << n.rolled_out << " discarded_v1_rollout=" << n.discarded_v1_rollout
                  << " discarded_v2_clearance=" << n.discarded_v2_clearance << " discarded_v1p_hold_end=" << n.discarded_v1p_hold_end
                  << " discarded_velocity=" << n.discarded_velocity << " evaluated_l2=" << n.evaluated_l2
                  << " discarded_l2_approach=" << n.discarded_l2_approach << " discarded_l2_hold=" << n.discarded_l2_hold
                  << " discarded_l2_diverged=" << n.discarded_l2_diverged << " posthoc_checks=" << res.posthoc_checks
                  << " l2_stage_ms=" << res.dynamic_stage_ms << " clearance_stage_ms=" << res.clearance_stage_ms
                  << " winner_l2_clearance_mm=" << res.winner_l2_clearance_mm << "\n";
      }
      if (!a.selftest) {
        std::ofstream mf(a.out + ".meta", std::ios::app);
        {
          const S3Result::DynamicCounters& n = res.dyn_counters;
          mf << "counter_rolled_out=" << n.rolled_out << "\ncounter_discarded_v1_rollout=" << n.discarded_v1_rollout
             << "\ncounter_discarded_v2_clearance=" << n.discarded_v2_clearance
             << "\ncounter_discarded_v1p_hold_end=" << n.discarded_v1p_hold_end << "\ncounter_discarded_velocity=" << n.discarded_velocity
             << "\ncounter_evaluated_l2=" << n.evaluated_l2 << "\ncounter_discarded_l2_approach=" << n.discarded_l2_approach
             << "\ncounter_discarded_l2_hold=" << n.discarded_l2_hold << "\ncounter_discarded_l2_diverged=" << n.discarded_l2_diverged
             << "\nposthoc_checks=" << res.posthoc_checks << "\nl2_stage_ms=" << res.dynamic_stage_ms
             << "\nclearance_stage_ms=" << res.clearance_stage_ms << "\n";
        }
        mf << std::setprecision(17) << "use_dynamic_follow=1\nmargin_delta_deg=" << s3.margin_delta_deg
           << "\nmargin_follow_s=" << s3.margin_follow_s << "\ndynamic_evals_done=" << res.dynamic_evals_done
           << "\nrollouts_done=" << res.rollouts_done << "\nfinalists=" << res.v2_finalists.size()
           << "\ndiscarded=" << n_discarded << "\nselection_ms=" << selection_ms << "\nstatus=" << res.status << "\n";
        int fi = 0;
        for (const S3FinalistRecord& f : res.v2_finalists) {
          if (f.rollout_done) mf << "finalist_" << fi << "_discard_reason=" << (f.discard_reason.empty() ? "none" : f.discard_reason) << "\n";
          ++fi;
        }
      }
    }
    if (!use_dyn && !a.selftest) {  // the L2 block above already wrote them; without L2 the campaign dry run needs them too
      std::ofstream mf(a.out + ".meta", std::ios::app);
      mf << std::setprecision(17) << "selection_ms=" << selection_ms << "\nstatus=" << res.status << "\n";  // S3Result::Status: 0 selected, 1 no candidate, 2 post-hoc failed
    }
    if (!a.selftest) {  // per-stage times (sum over the finalists examined, see GraspLaunchResult::StageTimesMs); same names as the node's "s1 timing:" line
      std::ofstream mf(a.out + ".meta", std::ios::app);
      mf << std::setprecision(17) << "scan_ms=" << lr.stage_ms.scan << "\nrollout_ms=" << lr.stage_ms.rollout
         << "\nhold_end_ms=" << lr.stage_ms.hold_end << "\nposthoc_ms=" << lr.stage_ms.posthoc << "\n";
    }
    if (res.status == S3Result::kSelected) {
      const S3Candidate& c = res.chosen;
      std::cerr << std::setprecision(10) << "WINNER k=" << static_cast<int>(c.eval.k)
                << " theta_deg=" << c.eval.theta_rad * 180.0 / kPi << " d_pos_mm=" << c.d_pos_m * 1000.0
                << " w_hat=" << c.eval.w_hat << " margin_rad=" << c.eval.margins.min_margin_rad
                << " wrist_flip=" << c.eval.wrist_flip << " cost=" << c.cost << " wait_s=" << c.wait_s << "\n"
                << "RUN15 reference: k=0 theta=-163.5 deg d_pos=7.171 mm w_hat=0.8238 margin=0.5255 rad "
                   "(informative; depends on --theta-snap-rad/--t-snap-s/--center)\n";
    }

    for (const std::string& w : res.warnings) std::cerr << w << "\n";
    if (!a.selftest && !res.warnings.empty()) {
      std::ofstream mf(a.out + ".meta", std::ios::app);
      for (std::size_t i = 0; i < res.warnings.size(); ++i) mf << "warning_" << i << "=" << res.warnings[i] << "\n";
    }
    if (!res.forced_violations.empty()) std::cerr << "forced candidate violates (ignored): " << res.forced_violations << "\n";
    if (a.all_candidates) {
      std::string scenario = a.scenario_tag;
      if (scenario.empty()) {
        std::ostringstream sc;
        sc << std::setprecision(10) << "omega=" << a.omega_deg_s << ";center=" << a.center << ";axis=" << snap.axis.transpose()
           << ";theta=" << a.theta_snap_rad << ";t=" << a.t_snap_s << ";q=" << a.q_start;
        scenario = sc.str();
        std::replace(scenario.begin(), scenario.end(), ',', ' ');
        std::replace(scenario.begin(), scenario.end(), '\n', ' ');
      }
      // finalists of the scan: those of the production selection, or (--finalists-n different from s1.finalists_n) the first N groups of the
      // same J_static ranking, listed by a light run (no L2 / clearance / V1' / velocity: the list is built before any of them).
      const int n_prod = s3.finalists_n;
      const int n_scan = a.finalists_n_scan >= 0 ? a.finalists_n_scan : n_prod;
      std::vector<S3FinalistRecord> scan = res.v2_finalists;
      if (n_scan != n_prod) {
        S3Params pl = s3;
        pl.finalists_n = n_scan;
        pl.use_dynamic_follow = false;
        pl.use_hold_end = false;
        pl.use_margin = false;
        pl.lazy_static_order = false;
        pl.use_clearance = false;
        pl.posthoc_velocity_check = false;
        S3Result rl;
        planGraspAndLaunchS1(in, snap, min_delay, pl, snap.t_s, &rl, nullptr, nullptr, nullptr);
        scan = rl.v2_finalists;
      }
      writeAllCandidates(a, in, snap, min_delay, s3, clearance_model.get(), use_dyn ? &joint_dynamics : nullptr, res, scan, n_prod, n_scan,
                         scenario);
      if (!a.selftest) {
        std::ofstream mf(a.out + ".meta", std::ios::app);
        mf << "all_candidates=1\nfinalists=" << res.v2_finalists.size() << "\nscenario=" << scenario << "\n";
      }
      std::cerr << "all-candidates: " << res.v2_finalists.size() << " finalists written to " << a.out << "\n";
      return 0;
    }

    // ---- context for the psi-rolled evaluations (same glue as planGraspAndLaunchS1, grasp_s3_launch.cpp:34-51) ----
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
      const int n = std::max(1, static_cast<int>(std::ceil(tc / kDefaultSimDtS - 1e-9)));
      for (int i = 0; i < n; ++i) pd.step(kDefaultSimDtS);
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
      r.mask = s3AprioriRejectMaskV2(ev, d_pos, s3);
      r.accepted = (r.mask & (kRejectIk | kRejectRTrust | kRejectStartGoal)) == 0;
      r.t_wait_s = planLaunch(snap, snap.t_s, theta, tau_launch, min_delay).delay_s;
      const double dd = s3Distance(d_pos, ev.dist_to_demo.ori_rad, s3.length_scale_m,
                                   s3.orientation_weight, s3.angle_scale_rad);
      // CSV column kept under its old name (tools/s3_landscape_report.py reads it); the value is now J_static, as in the selector.
      r.J_current = s3CostV2(dd * s3.length_scale_m * 1000.0, bs.w_full, 0.0, s3);
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
          const RotatablePose pj = haptic_dmp_learning::core::rotatePoseAboutAxis(
              snap.center, snap.axis, base, snap.omega_rad_s * t_follow[j]);
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
    //     the tool's own fine grid (theta_c + j*refine; the selector itself no longer refines), all psi of the band
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
        ++n_coarse;  // the selector has a single theta grid: every log row is a coarse row
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
          if (!close(r.J_current, l.j_static)) fail("J_current (= J_static) differs");
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
          if (x->j_static != y->j_static) return x->j_static < y->j_static;
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
          if (!close(pass[pos]->d_pos_m, c.d_pos_m)) fail("winner d_pos differs from log");
          for (std::size_t q = 0; q < pos; ++q) {
            bool covered = false;
            for (const auto& e : res.posthoc_log) {
              if (e.k != pass[q]->k) continue;
              if (thetaDistRad(e.theta_rad, pass[q]->theta_rad) < s3.posthoc_cluster_deg * kPi / 180.0 || bits(e.theta_rad) == bits(pass[q]->theta_rad)) {
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
        const bool axis_same = !a.tilt_given && parseList(a.axis, 3, "--axis") == std::vector<double>{0.0, 0.0, 1.0};
        const bool run15 = axis_same && params_same && q_same && std::abs(a.theta_snap_rad - kRun15ThetaSnapRad) < 1e-12 &&
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
    out << std::setprecision(10) << kCsvHeader;
    if (use_dyn) out << kCsvHeaderDyn;
    out << '\n';
    for (const Row& r : rows) {
      if (!use_dyn) {
        writeRow(out, r);  // unchanged
        continue;
      }
      std::ostringstream line;
      line << std::setprecision(10);
      writeRow(line, r);
      std::string text = line.str();
      text.pop_back();  // the '\n' of writeRow: the tail goes before it
      const S3FinalistRecord* fin = nullptr;
      if (std::string(r.grid) == "coarse" && r.psi_deg == 0.0) {
        for (const S3FinalistRecord& f : res.v2_finalists) {
          if (f.rollout_done && static_cast<int>(f.cand.eval.k) == r.k && bits(f.cand.eval.theta_rad) == bits(r.theta_rad)) {
            fin = &f;
            break;
          }
        }
      }
      out << text << dynTail(fin) << '\n';
    }
    std::cerr << "wrote " << a.out << " (" << rows.size() << " rows)\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "s3_landscape: " << e.what() << "\n";
    return 2;
  }
}
