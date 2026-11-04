// dynamic_follow_parity: parity of the C++ L2 (core::DynamicFollowSimulator) with inertial_pred.py on the 36 runs of the
// inertial validation.
//
// Inputs: the four CSVs of inertial_pred.py in one directory (--in): runs_inertial.csv (run_id, omega_deg_s, q_start_1..7,
// kt_N_per_m, kr_Nm_per_rad, dt_Ns_per_m, dr_Nms_per_rad), state_at_end.csv (run_id, q1..7, dq1..7, f1, f2),
// target_series.csv (run_id, t_rel, px, py, pz, qx, qy, qz, qw); the reference: results_per_run.csv of the Python run (rows with
// model == L2); the URDF of the simulated model (--urdf; the one whose sha256 the params json records).
// Angles of the CSVs in --angle-unit (default rad = params_A.json units.csv_angle).
//
// Per run, three variants (each: minimum margin over [0, 3 s] and q at +1.5 s and +3.0 s):
//   a  strict parity: measured dq, measured target series, measured fingers (f1, f2)
//   b  as (a) but dq0 = 0
//   c  as (a) but the target is the ideal rotation of the pose of the series at t_rel = 0 about --axis through --center at the
//      run's omega (followTargetSample, evaluated at every 1 ms step)
// The nullspace target of the controller is q_start of the run in all of them (inertial_pred.py:617). Parameters = set A / eps 0.1
// of out2 unless overridden.
//
//   ros2 run satellite_grasp_planner dynamic_follow_parity --in DIR --ref results_per_run.csv --urdf fer_rendered.urdf --out parity.csv
//        [--angle-unit rad|deg] [--eps 0.1] [--comp-gravity 1] [--plant-gz -9.80665] [--dt 0.001]
//        [--center 0.70,0,0.77] [--axis 0,0,1] [--tol-deg 0.01] [--trace RUN_ID]
//
// Output CSV, one row per (run, variant); the reference columns are those of the L2 row of the run. Exit code 0 if variant (a)
// matches the reference within --tol-deg on the minimum margin and on q at +3.0 s for every run, 1 otherwise, 2 for bad input.
// --trace RUN_ID: per-step t, q, dq, tau, acc of variant (a) of that run to <out>.trace.csv (the Python has no such dump).
// Not run by the author (no build / docker allowed in the session that wrote it).
//
// --from-approach (separate mode; --ref is not used, variants a/b/c are not run): ONE continuous L2 simulation per run, from the
// measured state at the start of the approach to AT_END + 3 s, the state is never reset at AT_END. Inputs in --in:
//   runs_inertial.csv       impedance gains (kt_N_per_m, kr_Nm_per_rad, dt_Ns_per_m, dr_Nms_per_rad) and q_start_1..7
//   state_at_start.csv      run_id, q1..7, dq1..7, f1, f2, t_start_rel_s [, t_at_end_rel_s]   (measured at the start of the approach)
//   target_series_full.csv  run_id, t_rel, px, py, pz, qx, qy, qz, qw   (from t_start_rel_s to >= AT_END + 3 s)
// Time base (ASSUMED, same as target_series.csv): t_rel = 0 is AT_END, so t_start_rel_s < 0. t_at_end_rel_s is the instant of AT_END
// in that same time base; the column is optional and defaults to 0. Simulation time tau = t_rel - t_start_rel_s; the target is
// sampled (zero-order hold) at t_rel = t_start_rel_s + tau. AT_END is snapped to the dt grid (row round((t_at_end - t_start)/dt)).
// Nullspace target: q_start of the run (--ns-target qstart, as the other modes) or the initial q of the simulation (state0).
//   dynamic_follow_parity --from-approach --in DIR --urdf fer_rendered.urdf --out approach.csv [--angle-unit rad|deg] [--eps 0.1]
//        [--comp-gravity 1] [--plant-gz -9.80665] [--dt 0.001] [--ns-target qstart|state0] [--divergence-rad 6.2832]
// Output, one row per run: status, q at AT_END (7 joints) and at AT_END + 3 s, minimum joint margin on the approach window
// [t_start, AT_END] and on the hold window [AT_END, AT_END + 3 s] (value, joint, t_rel), the first row at which a joint limit is
// exceeded (L2 does not apply limits: only the fact is reported), compute time. Exit code 0 if at least one run was simulated and
// none diverged, 1 otherwise, 2 for bad input.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/pose_rotation.hpp"
#include "satellite_grasp_planner/core/dynamic_follow.hpp"

using namespace satellite_grasp_planner::core;
using RobotModel = franka_cartesian_control::core::RobotModel;
using JV = RobotModel::JointVector;

namespace {

constexpr double kRad2Deg = 180.0 / M_PI;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

struct Csv {
  std::vector<std::string> header;
  std::vector<std::vector<std::string>> rows;
  int col(const std::string& name) const {
    for (std::size_t i = 0; i < header.size(); ++i)
      if (header[i] == name) return static_cast<int>(i);
    return -1;
  }
  int need(const std::string& name, const std::string& file) const {
    const int c = col(name);
    if (c < 0) throw std::runtime_error(file + ": missing column " + name);
    return c;
  }
};

std::vector<std::string> splitLine(std::string line) {
  if (!line.empty() && line.back() == '\r') line.pop_back();
  std::vector<std::string> out;
  std::string cur;
  for (char ch : line) {
    if (ch == ',') {
      out.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(ch);
    }
  }
  out.push_back(cur);
  return out;
}

Csv readCsv(const std::string& path) {
  std::ifstream f(path);
  if (!f.is_open()) throw std::runtime_error("cannot open " + path);
  Csv c;
  std::string line;
  if (!std::getline(f, line)) throw std::runtime_error(path + ": empty");
  c.header = splitLine(line);
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    c.rows.push_back(splitLine(line));
  }
  return c;
}

double num(const std::vector<std::string>& row, int col) {
  if (col < 0 || static_cast<std::size_t>(col) >= row.size() || row[static_cast<std::size_t>(col)].empty()) return kNaN;
  return std::stod(row[static_cast<std::size_t>(col)]);
}

std::vector<double> parseList(const std::string& s, std::size_t n, const char* what) {
  std::vector<double> v;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ',')) v.push_back(std::stod(tok));
  if (v.size() != n) throw std::runtime_error(std::string(what) + ": expected " + std::to_string(n) + " values");
  return v;
}

struct Args {
  std::string in, ref, urdf, out, trace_run;
  std::string angle_unit = "rad";
  std::string ns_target = "qstart";  // --from-approach only: qstart | state0
  bool from_approach = false;
  double divergence_rad = DynamicFollowParams().divergence_rad;  // --from-approach only
  double eps = 0.1, plant_gz = -9.80665, dt = 0.001, tol_deg = 0.01;
  bool comp_gravity = true;
  std::vector<double> center{0.70, 0.0, 0.77};
  std::vector<double> axis{0.0, 0.0, 1.0};
};

Args parseArgs(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    auto val = [&]() -> std::string {
      if (i + 1 >= argc) throw std::runtime_error("missing value for " + k);
      return argv[++i];
    };
    if (k == "--in") a.in = val();
    else if (k == "--ref") a.ref = val();
    else if (k == "--urdf") a.urdf = val();
    else if (k == "--out") a.out = val();
    else if (k == "--trace") a.trace_run = val();
    else if (k == "--angle-unit") a.angle_unit = val();
    else if (k == "--from-approach") a.from_approach = true;
    else if (k == "--ns-target") a.ns_target = val();
    else if (k == "--divergence-rad") a.divergence_rad = std::stod(val());
    else if (k == "--eps") a.eps = std::stod(val());
    else if (k == "--plant-gz") a.plant_gz = std::stod(val());
    else if (k == "--dt") a.dt = std::stod(val());
    else if (k == "--tol-deg") a.tol_deg = std::stod(val());
    else if (k == "--comp-gravity") a.comp_gravity = std::stoi(val()) != 0;
    else if (k == "--center") a.center = parseList(val(), 3, "--center");
    else if (k == "--axis") a.axis = parseList(val(), 3, "--axis");
    else throw std::runtime_error("unknown argument " + k);
  }
  if (a.in.empty() || (a.ref.empty() && !a.from_approach) || a.urdf.empty() || a.out.empty()) {
    throw std::runtime_error("--in, --ref (not with --from-approach), --urdf and --out are required");
  }
  if (a.ns_target != "qstart" && a.ns_target != "state0") throw std::runtime_error("--ns-target in {qstart, state0}");
  if (a.angle_unit != "rad" && a.angle_unit != "deg") throw std::runtime_error("--angle-unit in {rad, deg}");
  return a;
}

struct Row {
  std::string run_id;
  char variant = 'a';
  bool diverged = false;
  double t_div = kNaN;
  double margin_deg = kNaN;
  int margin_joint = 0;  // 1-based
  double margin_t = kNaN;
  JV q15 = JV::Constant(kNaN), q30 = JV::Constant(kNaN);
  // reference
  bool has_ref = false;
  double ref_margin_deg = kNaN;
  double ref_margin_joint = kNaN;
  JV ref_q15 = JV::Constant(kNaN), ref_q30 = JV::Constant(kNaN);
};

JV qAtOrNaN(const DynamicFollowResult& r, double t) {
  try {
    return r.qAt(t);
  } catch (const std::out_of_range&) {
    return JV::Constant(kNaN);
  }
}

}  // namespace

// ---------------------------------------------------------------------------------------------------------------------
// --from-approach
// ---------------------------------------------------------------------------------------------------------------------
namespace {

struct WindowMargin {
  double margin_rad = kNaN;
  int joint = 0;  // 1-based, 0 = no row
  double t_rel = kNaN;
};

/// Minimum over rows [k0, k1] (clipped to the rows that exist) and the 7 joints of min(q - lo, hi - q); ties: first row, first joint.
WindowMargin windowMargin(const DynamicFollowResult& r, const JV& lo, const JV& hi, long k0, long k1, double t_start_rel) {
  WindowMargin w;
  const long last = static_cast<long>(r.q.size()) - 1;
  for (long k = std::max(0L, k0); k <= std::min(k1, last); ++k) {
    const JV& q = r.q[static_cast<std::size_t>(k)];
    const JV m = (q - lo).cwiseMin(hi - q);
    Eigen::Index j = 0;
    const double mm = m.minCoeff(&j);
    if (w.joint == 0 || mm < w.margin_rad) {
      w.margin_rad = mm;
      w.joint = static_cast<int>(j) + 1;
      w.t_rel = t_start_rel + static_cast<double>(k) * r.dt_s;
    }
  }
  return w;
}

int runFromApproach(const Args& args) {
  const double ang = args.angle_unit == "deg" ? M_PI / 180.0 : 1.0;  // input angle -> rad
  const double hold_s = 3.0;

  std::ifstream uf(args.urdf);
  if (!uf.is_open()) throw std::runtime_error("cannot open " + args.urdf);
  std::stringstream ubuf;
  ubuf << uf.rdbuf();
  const std::string urdf_xml = ubuf.str();
  std::vector<std::string> joint_names;
  for (int j = 1; j <= 7; ++j) joint_names.push_back("fer_joint" + std::to_string(j));
  const JointDynamics dyn = parseJointDynamicsFromUrdfXml(urdf_xml, joint_names);

  const std::string runs_f = "runs_inertial.csv", state_f = "state_at_start.csv", tgt_f = "target_series_full.csv";
  const Csv runs = readCsv(args.in + "/" + runs_f);
  const Csv state = readCsv(args.in + "/" + state_f);
  const Csv tgt = readCsv(args.in + "/" + tgt_f);
  const int r_id = runs.need("run_id", runs_f);
  const int r_kt = runs.need("kt_N_per_m", runs_f), r_kr = runs.need("kr_Nm_per_rad", runs_f);
  const int r_dt = runs.need("dt_Ns_per_m", runs_f), r_dr = runs.need("dr_Nms_per_rad", runs_f);
  int r_qs[7], s_q[7], s_dq[7];
  for (int j = 0; j < 7; ++j) {
    r_qs[j] = runs.need("q_start_" + std::to_string(j + 1), runs_f);
    s_q[j] = state.need("q" + std::to_string(j + 1), state_f);
    s_dq[j] = state.need("dq" + std::to_string(j + 1), state_f);
  }
  const int s_id = state.need("run_id", state_f), s_f1 = state.need("f1", state_f), s_f2 = state.need("f2", state_f);
  const int s_t0 = state.need("t_start_rel_s", state_f);
  const int s_te = state.col("t_at_end_rel_s");  // optional, default 0
  const int t_id = tgt.need("run_id", tgt_f), t_t = tgt.need("t_rel", tgt_f);
  const int t_p[3] = {tgt.need("px", tgt_f), tgt.need("py", tgt_f), tgt.need("pz", tgt_f)};
  const int t_q[4] = {tgt.need("qx", tgt_f), tgt.need("qy", tgt_f), tgt.need("qz", tgt_f), tgt.need("qw", tgt_f)};
  if (s_te < 0) std::cerr << "note: no t_at_end_rel_s column in " << state_f << ": AT_END assumed at t_rel = 0 for every run\n";

  std::map<std::string, const std::vector<std::string>*> state_rows;
  for (const auto& r : state.rows) state_rows[r[static_cast<std::size_t>(s_id)]] = &r;
  std::map<std::string, std::vector<const std::vector<std::string>*>> tgt_rows;
  for (const auto& r : tgt.rows) tgt_rows[r[static_cast<std::size_t>(t_id)]].push_back(&r);

  DynamicFollowParams P;
  P.friction_tanh_eps_rad_s = args.eps;
  P.plant_gravity_z_m_s2 = args.plant_gz;
  P.compensate_gravity_internally = args.comp_gravity;
  P.dt_s = args.dt;
  P.abort_on_divergence = true;
  P.divergence_rad = args.divergence_rad;

  std::ofstream of(args.out);
  if (!of.is_open()) throw std::runtime_error("cannot write " + args.out);
  of << std::setprecision(17);
  of << "run_id,status,t_div_s,t_start_rel_s,t_at_end_rel_s,steps_done";
  for (int j = 1; j <= 7; ++j) of << ",q_at_end_j" << j << "_rad";
  for (int j = 1; j <= 7; ++j) of << ",q_hold_end_j" << j << "_rad";
  of << ",approach_min_margin_deg,approach_min_margin_joint,approach_min_margin_t_rel_s"
        ",hold_min_margin_deg,hold_min_margin_joint,hold_min_margin_t_rel_s"
        ",limit_exceeded,first_viol_joint,first_viol_side,first_viol_t_rel_s,compute_ms\n";

  int n_sim = 0, n_div = 0, n_skipped = 0;
  for (const auto& rr : runs.rows) {
    const std::string rid = rr[static_cast<std::size_t>(r_id)];
    const auto sit = state_rows.find(rid);
    const auto tit = tgt_rows.find(rid);
    if (sit == state_rows.end() || tit == tgt_rows.end() || tit->second.size() < 2) {
      std::cerr << "skip " << rid << ": missing state or target rows\n";
      ++n_skipped;
      continue;
    }
    const std::vector<std::string>& st = *sit->second;
    JV q0, dq0, q_start;
    for (int j = 0; j < 7; ++j) {
      q0(j) = num(st, s_q[j]) * ang;
      dq0(j) = num(st, s_dq[j]) * ang;
      q_start(j) = num(rr, r_qs[j]) * ang;
    }
    const double t_start = num(st, s_t0);
    const double t_end = s_te >= 0 ? num(st, s_te) : 0.0;
    if (!std::isfinite(t_start) || !std::isfinite(t_end) || !(t_end > t_start) || !q0.allFinite() || !dq0.allFinite() ||
        !q_start.allFinite()) {
      std::cerr << "skip " << rid << ": non-finite state or t_start_rel_s=" << t_start << " not before t_at_end_rel_s=" << t_end << "\n";
      ++n_skipped;
      continue;
    }
    std::vector<double> ts;
    std::vector<Eigen::Vector3d> ps;
    std::vector<Eigen::Quaterniond> qs;
    for (const auto* tr : tit->second) {
      ts.push_back(num(*tr, t_t));
      ps.emplace_back(num(*tr, t_p[0]), num(*tr, t_p[1]), num(*tr, t_p[2]));
      qs.emplace_back(num(*tr, t_q[3]), num(*tr, t_q[0]), num(*tr, t_q[1]), num(*tr, t_q[2]));  // Eigen ctor: w, x, y, z
    }
    const auto series = std::make_shared<const PoseSeries>(ts, ps, qs);

    // AT_END and the end of the hold, snapped to the dt grid; the horizon is exactly their sum so no rounding mismatch with simulate().
    const long k_end = std::lround((t_end - t_start) / args.dt);
    const long k_hold = std::lround(hold_s / args.dt);
    const double horizon = static_cast<double>(k_end + k_hold) * args.dt;
    if (k_end < 1) {
      std::cerr << "skip " << rid << ": AT_END less than one step after t_start\n";
      ++n_skipped;
      continue;
    }
    if (series->tEnd() < t_start + horizon - 1e-9) {
      std::cerr << "skip " << rid << ": target series ends at t_rel=" << series->tEnd() << " < " << t_start + horizon << "\n";
      ++n_skipped;
      continue;
    }
    if (series->tBegin() > t_start + 1e-6) {
      std::cerr << "warn " << rid << ": target series starts at t_rel=" << series->tBegin() << " > t_start_rel_s=" << t_start
                << " (the first sample is held backwards)\n";
    }

    DynamicFollowParams Pr = P;
    Pr.translational_stiffness = num(rr, r_kt);
    Pr.rotational_stiffness = num(rr, r_kr);
    Pr.translational_damping = num(rr, r_dt);
    Pr.rotational_damping = num(rr, r_dr);

    auto robot = std::make_shared<RobotModel>(urdf_xml, joint_names, "fer_hand_tcp");
    const double f1 = num(st, s_f1), f2 = num(st, s_f2);
    if (robot->numPassiveJoints() == 2) robot->setPassiveJointPositions({f1, f2});
    else if (robot->numPassiveJoints() == 1) robot->setPassiveJointPositions({f1});
    else if (robot->numPassiveJoints() != 0) throw std::runtime_error("unexpected number of passive joints");
    DynamicFollowSimulator sim(robot, dyn, Pr);

    const std::shared_ptr<const PoseSeries> sp = series;
    const DynamicFollowTarget target = [sp, t_start](double tau) { return sp->at(t_start + tau); };
    const JV q_ns = args.ns_target == "state0" ? q0 : q_start;

    const auto c0 = std::chrono::steady_clock::now();
    const DynamicFollowResult res = sim.simulate(q0, dq0, q_ns, target, horizon);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - c0).count();
    ++n_sim;
    if (res.diverged) ++n_div;

    const JV lo = robot->jointLowerLimits();
    const JV hi = robot->jointUpperLimits();
    const WindowMargin wa = windowMargin(res, lo, hi, 0, k_end, t_start);
    const WindowMargin wh = windowMargin(res, lo, hi, k_end, k_end + k_hold, t_start);

    // First row with a joint beyond its limit (the most violated joint of that row). L2 itself does not apply limits.
    int v_joint = 0;
    const char* v_side = "";
    double v_t = kNaN;
    for (std::size_t k = 0; k < res.q.size() && v_joint == 0; ++k) {
      const JV& q = res.q[k];
      const JV m = (q - lo).cwiseMin(hi - q);
      Eigen::Index j = 0;
      if (m.minCoeff(&j) < 0.0) {
        v_joint = static_cast<int>(j) + 1;
        v_side = q(j) < lo(j) ? "lower" : "upper";
        v_t = t_start + static_cast<double>(k) * res.dt_s;
      }
    }

    const JV q_end = qAtOrNaN(res, static_cast<double>(k_end) * args.dt);
    const JV q_hold = qAtOrNaN(res, static_cast<double>(k_end + k_hold) * args.dt);
    of << rid << "," << (res.diverged ? "diverged" : "ok") << "," << (res.diverged ? res.t_div_s : kNaN) << "," << t_start << ","
       << t_end << "," << res.steps_done;
    for (int j = 0; j < 7; ++j) of << "," << q_end(j);
    for (int j = 0; j < 7; ++j) of << "," << q_hold(j);
    of << "," << wa.margin_rad * kRad2Deg << "," << wa.joint << "," << wa.t_rel << "," << wh.margin_rad * kRad2Deg << "," << wh.joint
       << "," << wh.t_rel << "," << (v_joint != 0 ? 1 : 0) << "," << v_joint << "," << v_side << "," << v_t << "," << ms << "\n";
    std::cout << rid << ": " << (res.diverged ? "DIVERGED" : "ok") << ", " << res.steps_done << " steps, " << std::fixed
              << std::setprecision(1) << ms << " ms" << std::defaultfloat << std::setprecision(17) << "\n";
  }
  std::cout << "simulated " << n_sim << ", diverged " << n_div << ", skipped " << n_skipped << "\nwritten " << args.out << "\n";
  return n_sim > 0 && n_div == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  Args args;
  try {
    args = parseArgs(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 2;
  }
  if (args.from_approach) {
    try {
      return runFromApproach(args);
    } catch (const std::exception& e) {
      std::cerr << "error: " << e.what() << "\n";
      return 2;
    }
  }
  try {
    const double ang = args.angle_unit == "deg" ? M_PI / 180.0 : 1.0;  // input angle -> rad
    const double horizon = 3.0;

    // ---- URDF ----
    std::ifstream uf(args.urdf);
    if (!uf.is_open()) throw std::runtime_error("cannot open " + args.urdf);
    std::stringstream ubuf;
    ubuf << uf.rdbuf();
    const std::string urdf_xml = ubuf.str();
    std::vector<std::string> joint_names;
    for (int j = 1; j <= 7; ++j) joint_names.push_back("fer_joint" + std::to_string(j));
    const JointDynamics dyn = parseJointDynamicsFromUrdfXml(urdf_xml, joint_names);

    // ---- inputs ----
    const Csv runs = readCsv(args.in + "/runs_inertial.csv");
    const Csv state = readCsv(args.in + "/state_at_end.csv");
    const Csv tgt = readCsv(args.in + "/target_series.csv");
    const Csv refcsv = readCsv(args.ref);
    const std::string runs_f = "runs_inertial.csv", state_f = "state_at_end.csv", tgt_f = "target_series.csv";
    const int r_id = runs.need("run_id", runs_f), r_om = runs.need("omega_deg_s", runs_f);
    const int r_kt = runs.need("kt_N_per_m", runs_f), r_kr = runs.need("kr_Nm_per_rad", runs_f);
    const int r_dt = runs.need("dt_Ns_per_m", runs_f), r_dr = runs.need("dr_Nms_per_rad", runs_f);
    int r_qs[7], s_q[7], s_dq[7];
    for (int j = 0; j < 7; ++j) {
      r_qs[j] = runs.need("q_start_" + std::to_string(j + 1), runs_f);
      s_q[j] = state.need("q" + std::to_string(j + 1), state_f);
      s_dq[j] = state.need("dq" + std::to_string(j + 1), state_f);
    }
    const int s_id = state.need("run_id", state_f), s_f1 = state.need("f1", state_f), s_f2 = state.need("f2", state_f);
    const int t_id = tgt.need("run_id", tgt_f), t_t = tgt.need("t_rel", tgt_f);
    const int t_p[3] = {tgt.need("px", tgt_f), tgt.need("py", tgt_f), tgt.need("pz", tgt_f)};
    const int t_q[4] = {tgt.need("qx", tgt_f), tgt.need("qy", tgt_f), tgt.need("qz", tgt_f), tgt.need("qw", tgt_f)};

    // reference rows (model == L2)
    const std::string ref_f = "results_per_run.csv";
    const int f_id = refcsv.need("run_id", ref_f), f_model = refcsv.need("model", ref_f);
    const std::string u = args.angle_unit;
    const int f_mm = refcsv.need("pred_min_margin_" + u, ref_f), f_mj = refcsv.need("pred_min_margin_joint", ref_f);
    int f_q15[7], f_q30[7];
    for (int j = 0; j < 7; ++j) {
      f_q15[j] = refcsv.need("q_pred_h1p5_j" + std::to_string(j + 1) + "_" + u, ref_f);
      f_q30[j] = refcsv.need("q_pred_h3_j" + std::to_string(j + 1) + "_" + u, ref_f);
    }
    std::map<std::string, const std::vector<std::string>*> ref_rows;
    for (const auto& r : refcsv.rows)
      if (r[static_cast<std::size_t>(f_model)] == "L2") ref_rows[r[static_cast<std::size_t>(f_id)]] = &r;

    std::map<std::string, const std::vector<std::string>*> state_rows;
    for (const auto& r : state.rows) state_rows[r[static_cast<std::size_t>(s_id)]] = &r;
    std::map<std::string, std::vector<const std::vector<std::string>*>> tgt_rows;
    for (const auto& r : tgt.rows) tgt_rows[r[static_cast<std::size_t>(t_id)]].push_back(&r);

    DynamicFollowParams P;  // defaults = set A, eps 0.1
    P.friction_tanh_eps_rad_s = args.eps;
    P.plant_gravity_z_m_s2 = args.plant_gz;
    P.compensate_gravity_internally = args.comp_gravity;
    P.dt_s = args.dt;
    P.abort_on_divergence = true;

    std::vector<Row> out;
    std::ofstream trace;
    int n_skipped = 0;
    for (const auto& rr : runs.rows) {
      const std::string rid = rr[static_cast<std::size_t>(r_id)];
      const auto sit = state_rows.find(rid);
      const auto tit = tgt_rows.find(rid);
      if (sit == state_rows.end() || tit == tgt_rows.end() || tit->second.size() < 2) {
        std::cerr << "skip " << rid << ": missing state or target rows\n";
        ++n_skipped;
        continue;
      }
      const std::vector<std::string>& st = *sit->second;
      JV q0, dq0, q_start;
      for (int j = 0; j < 7; ++j) {
        q0(j) = num(st, s_q[j]) * ang;
        dq0(j) = num(st, s_dq[j]) * ang;
        q_start(j) = num(rr, r_qs[j]) * ang;
      }
      std::vector<double> ts;
      std::vector<Eigen::Vector3d> ps;
      std::vector<Eigen::Quaterniond> qs;
      for (const auto* tr : tit->second) {
        ts.push_back(num(*tr, t_t));
        ps.emplace_back(num(*tr, t_p[0]), num(*tr, t_p[1]), num(*tr, t_p[2]));
        qs.emplace_back(num(*tr, t_q[3]), num(*tr, t_q[0]), num(*tr, t_q[1]), num(*tr, t_q[2]));  // Eigen ctor: w, x, y, z
      }
      const auto series = std::make_shared<const PoseSeries>(ts, ps, qs);
      if (series->tEnd() < horizon - 1e-9) {
        std::cerr << "skip " << rid << ": target series ends at " << series->tEnd() << " s < " << horizon << " s\n";
        ++n_skipped;
        continue;
      }

      DynamicFollowParams Pr = P;
      Pr.translational_stiffness = num(rr, r_kt);
      Pr.rotational_stiffness = num(rr, r_kr);
      Pr.translational_damping = num(rr, r_dt);
      Pr.rotational_damping = num(rr, r_dr);

      auto robot = std::make_shared<RobotModel>(urdf_xml, joint_names, "fer_hand_tcp");
      const double f1 = num(st, s_f1), f2 = num(st, s_f2);
      if (robot->numPassiveJoints() == 2) robot->setPassiveJointPositions({f1, f2});
      else if (robot->numPassiveJoints() == 1) robot->setPassiveJointPositions({f1});  // mimic follower not a separate DOF
      else if (robot->numPassiveJoints() != 0) throw std::runtime_error("unexpected number of passive joints");
      DynamicFollowSimulator sim(robot, dyn, Pr);

      const double omega = num(rr, r_om) * M_PI / 180.0;  // omega_deg_s is in deg/s whatever --angle-unit is
      const CartesianSample goal0 = series->at(0.0);
      const haptic_dmp_learning::core::RotatablePose goal_at_end{goal0.position, goal0.orientation};
      const Eigen::Vector3d center(args.center[0], args.center[1], args.center[2]);
      const Eigen::Vector3d axis(args.axis[0], args.axis[1], args.axis[2]);

      const auto rit = ref_rows.find(rid);
      for (char variant : {'a', 'b', 'c'}) {
        const JV dq_init = variant == 'b' ? JV::Zero() : dq0;
        const DynamicFollowTarget target =
            variant == 'c' ? makeRotatingTarget(center, axis, omega, goal_at_end) : makePoseSeriesTarget(series);
        DynamicFollowObserver obs;
        const bool do_trace = variant == 'a' && !args.trace_run.empty() && args.trace_run == rid;
        if (do_trace) {
          trace.open(args.out + ".trace.csv");
          trace << std::setprecision(17) << "step,t_s";
          for (const char* nm : {"q", "dq", "tau", "acc"})
            for (int j = 1; j <= 7; ++j) trace << "," << nm << j;
          trace << "\n";
          obs = [&trace](const DynamicFollowStepInfo& s) {
            trace << s.step << "," << s.t_s;
            for (const JV* x : {&s.q, &s.v, &s.tau, &s.acc})
              for (int j = 0; j < 7; ++j) trace << "," << (*x)(j);
            trace << "\n";
          };
        }
        const DynamicFollowResult res = sim.simulate(q0, dq_init, q_start, target, horizon, do_trace ? &obs : nullptr);
        Row row;
        row.run_id = rid;
        row.variant = variant;
        row.diverged = res.diverged;
        row.t_div = res.diverged ? res.t_div_s : kNaN;
        row.margin_deg = res.min_margin_rad * kRad2Deg;
        row.margin_joint = res.min_margin_joint + 1;
        row.margin_t = res.min_margin_t_s;
        row.q15 = qAtOrNaN(res, 1.5);
        row.q30 = qAtOrNaN(res, 3.0);
        if (rit != ref_rows.end()) {
          const auto& fr = *rit->second;
          row.has_ref = true;
          const double to_deg = args.angle_unit == "deg" ? 1.0 : kRad2Deg;
          row.ref_margin_deg = num(fr, f_mm) * to_deg;
          row.ref_margin_joint = num(fr, f_mj);
          for (int j = 0; j < 7; ++j) {
            row.ref_q15(j) = num(fr, f_q15[j]) * ang;
            row.ref_q30(j) = num(fr, f_q30[j]) * ang;
          }
        }
        out.push_back(row);
      }
    }

    // ---- output ----
    std::ofstream of(args.out);
    if (!of.is_open()) throw std::runtime_error("cannot write " + args.out);
    of << std::setprecision(17);
    of << "run_id,variant,status,t_div_s,min_margin_deg,min_margin_joint,min_margin_t_s";
    for (int j = 1; j <= 7; ++j) of << ",q_h1p5_j" << j << "_rad";
    for (int j = 1; j <= 7; ++j) of << ",q_h3_j" << j << "_rad";
    of << ",ref_min_margin_deg,ref_min_margin_joint,d_margin_deg,max_abs_dq_h1p5_deg,max_abs_dq_h3_deg\n";
    double worst_margin = 0.0, worst_q30 = 0.0, worst_q15 = 0.0;
    int n_a = 0, n_bad = 0, n_joint_mismatch = 0;
    std::string worst_margin_run, worst_q30_run;
    for (const Row& r : out) {
      const double dm = r.has_ref ? r.margin_deg - r.ref_margin_deg : kNaN;
      const double d15 = r.has_ref ? (r.q15 - r.ref_q15).cwiseAbs().maxCoeff() * kRad2Deg : kNaN;
      const double d30 = r.has_ref ? (r.q30 - r.ref_q30).cwiseAbs().maxCoeff() * kRad2Deg : kNaN;
      of << r.run_id << "," << r.variant << "," << (r.diverged ? "diverged" : "ok") << "," << r.t_div << "," << r.margin_deg << ","
         << r.margin_joint << "," << r.margin_t;
      for (int j = 0; j < 7; ++j) of << "," << r.q15(j);
      for (int j = 0; j < 7; ++j) of << "," << r.q30(j);
      of << "," << r.ref_margin_deg << "," << r.ref_margin_joint << "," << dm << "," << d15 << "," << d30 << "\n";
      if (r.variant == 'a' && r.has_ref) {
        ++n_a;
        const bool ok = !r.diverged && std::isfinite(dm) && std::isfinite(d30) && std::abs(dm) <= args.tol_deg && d30 <= args.tol_deg;
        if (!ok) ++n_bad;
        if (std::isfinite(dm) && std::abs(dm) > std::abs(worst_margin)) { worst_margin = dm; worst_margin_run = r.run_id; }
        if (std::isfinite(d30) && d30 > worst_q30) { worst_q30 = d30; worst_q30_run = r.run_id; }
        if (std::isfinite(d15) && d15 > worst_q15) worst_q15 = d15;
        if (r.ref_margin_joint != r.margin_joint) ++n_joint_mismatch;
      }
    }
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "runs with reference (variant a): " << n_a << ", skipped inputs: " << n_skipped << "\n"
              << "worst |d margin| = " << std::abs(worst_margin) << " deg (" << worst_margin_run << ")\n"
              << "worst max_j |d q(+1.5 s)| = " << worst_q15 << " deg\n"
              << "worst max_j |d q(+3.0 s)| = " << worst_q30 << " deg (" << worst_q30_run << ")\n"
              << "limiting joint differs from the reference in " << n_joint_mismatch << " runs\n"
              << "runs outside tolerance " << args.tol_deg << " deg (or diverged): " << n_bad << "\n"
              << (n_a > 0 && n_bad == 0 ? "PARITY OK" : "PARITY FAILED (do not tune: locate the first divergence with --trace)") << "\n"
              << "written " << args.out << "\n";
    return n_a > 0 && n_bad == 0 ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 2;
  }
}
