// hold_end_check: offline verification of static grasp IK and hold-end IK (+3.0 s) with holdPoseIk.
// Reads inputs/runs_inertial.csv, evaluates grasp IK and hold-end IK, prints end_ok, q_hold_end, limit margin,
// w_presa, w_end, and maximum joint difference with measured q(+3.0 s) if available (or NON TROVATO).
// Sola lettura sui dati (no file modifications).

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
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/pose_rotation.hpp"
#include "haptic_dmp_learning/core/tcp_roll.hpp"
#include "satellite_grasp_planner/core/grasp_pose_eval.hpp"
#include "satellite_grasp_planner/core/grasp_s3_selection.hpp"
#include "satellite_grasp_planner/core/manipulability.hpp"
#include "satellite_grasp_planner/core/start_state.hpp"

using namespace satellite_grasp_planner::core;
using haptic_dmp_learning::core::CubeSatelliteModel;
using haptic_dmp_learning::core::GraspPointId;
using RobotModel = franka_cartesian_control::core::RobotModel;
using JV = RobotModel::JointVector;

namespace {

constexpr double kRad2Deg = 180.0 / M_PI;
constexpr double kDeg2Rad = M_PI / 180.0;

struct Csv {
  std::vector<std::string> header;
  std::vector<std::vector<std::string>> rows;
  int col(const std::string& name) const {
    for (std::size_t i = 0; i < header.size(); ++i) {
      if (header[i] == name) return static_cast<int>(i);
    }
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
  if (col < 0 || static_cast<std::size_t>(col) >= row.size() || row[static_cast<std::size_t>(col)].empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::stod(row[static_cast<std::size_t>(col)]);
}

std::string str(const std::vector<std::string>& row, int col) {
  if (col < 0 || static_cast<std::size_t>(col) >= row.size()) return "";
  return row[static_cast<std::size_t>(col)];
}

bool fileExists(const std::string& path) {
  std::ifstream f(path);
  return f.good();
}

std::string findFile(const std::vector<std::string>& candidates) {
  for (const auto& c : candidates) {
    if (fileExists(c)) return c;
  }
  return "";
}

Eigen::Vector3d parseVec3(const std::string& s) {
  std::stringstream ss(s);
  std::string item;
  std::vector<double> v;
  while (std::getline(ss, item, ',')) {
    v.push_back(std::stod(item));
  }
  if (v.size() != 3) throw std::invalid_argument("expected 3 comma-separated numbers: " + s);
  return Eigen::Vector3d(v[0], v[1], v[2]);
}

}  // namespace

int main(int argc, char** argv) {
  std::string runs_path = "";
  std::string meas_path = "";
  std::string urdf_path = "";
  double t_follow_s = 3.0;
  Eigen::Vector3d center(0.70, 0.0, 0.77);
  Eigen::Vector3d axis(0.0, 0.0, 1.0);
  double cube_side = 0.150;
  double standoff = 0.080;
  double collar_radius = 0.0725;
  double wall = 0.009;
  double grasp_radius = 0.068;
  double grasp_depth = 0.0125;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto needArg = [&](const std::string& opt) {
      if (i + 1 >= argc) throw std::invalid_argument(opt + " requires an argument");
      return argv[++i];
    };
    if (a == "--runs") {
      runs_path = needArg(a);
    } else if (a == "--meas") {
      meas_path = needArg(a);
    } else if (a == "--urdf") {
      urdf_path = needArg(a);
    } else if (a == "--t-follow") {
      t_follow_s = std::stod(needArg(a));
    } else if (a == "--center") {
      center = parseVec3(needArg(a));
    } else if (a == "--axis") {
      axis = parseVec3(needArg(a));
    } else if (a == "-h" || a == "--help") {
      std::cout << "Usage: hold_end_check [--runs FILE] [--meas FILE] [--urdf FILE] [--t-follow S]\n";
      return 0;
    }
  }

  if (runs_path.empty()) {
    runs_path = findFile({"inputs/runs_inertial.csv",
                          "_gate_out/endpoint_1028/inertial_1028/inputs/runs_inertial.csv",
                          "_gate_out/endpoint_1028/inertial_1028/out2/inputs/runs_inertial.csv"});
    if (runs_path.empty()) {
      std::cerr << "hold_end_check: cannot find runs_inertial.csv. Specify with --runs.\n";
      return 2;
    }
  }

  if (meas_path.empty()) {
    meas_path = findFile({"_gate_out/endpoint_1028/endpoint_inputs.csv",
                          "endpoint_inputs.csv"});
  }

  if (urdf_path.empty()) {
    urdf_path = findFile({"fer_rendered.urdf",
                          "src/franka_cartesian_control/models/fer_rendered.urdf",
                          "fer_flat_effort.urdf",
                          "src/franka_cartesian_control/models/fer_flat_effort.urdf"});
    if (urdf_path.empty()) {
      std::cerr << "hold_end_check: cannot find URDF file. Specify with --urdf.\n";
      return 2;
    }
  }

  std::cout << "hold_end_check: runs = " << runs_path << "\n";
  std::cout << "hold_end_check: urdf = " << urdf_path << "\n";
  std::cout << "hold_end_check: meas = " << (meas_path.empty() ? "NON TROVATO" : meas_path) << "\n";
  std::cout << "hold_end_check: t_follow = " << t_follow_s << " s\n\n";

  const auto robot_ptr = makeFerRobotModel(urdf_path, "hold_end_check: ");
  RobotModel& robot = *robot_ptr;

  CubeSatelliteModel::Params cube_params;
  cube_params.center_world = center;
  cube_params.axis_world = axis;
  cube_params.cube_side_m = cube_side;
  cube_params.standoff_m = standoff;
  cube_params.collar_radius_m = collar_radius;
  cube_params.wall_thickness_m = wall;
  cube_params.grasp_radius_m = grasp_radius;
  cube_params.grasp_depth_m = grasp_depth;
  cube_params.face_normal_body = Eigen::Vector3d::UnitX();
  const CubeSatelliteModel model(cube_params);

  const Csv runs_csv = readCsv(runs_path);
  const int col_id = runs_csv.need("run_id", runs_path);
  const int col_om = runs_csv.need("omega_deg_s", runs_path);
  const int col_k = runs_csv.need("k", runs_path);
  const int col_th = runs_csv.need("theta_star_deg", runs_path);
  const int col_flip = runs_csv.need("wrist_flip", runs_path);

  int col_qs[7];
  for (int j = 0; j < 7; ++j) {
    col_qs[j] = runs_csv.need("q_start_" + std::to_string(j + 1), runs_path);
  }

  // Check if measured q(+3.0 s) is in runs_csv
  int col_qmeas[7];
  bool meas_in_runs = true;
  for (int j = 0; j < 7; ++j) {
    const int c = runs_csv.col("q_end_meas_q" + std::to_string(j + 1));
    if (c < 0) {
      meas_in_runs = false;
      break;
    }
    col_qmeas[j] = c;
  }

  // Otherwise check external meas_csv
  std::map<std::string, JV> meas_map;
  if (!meas_in_runs && !meas_path.empty()) {
    try {
      const Csv mcsv = readCsv(meas_path);
      const int mid = mcsv.need("run_id", meas_path);
      int mq[7];
      for (int j = 0; j < 7; ++j) {
        mq[j] = mcsv.need("q_end_meas_q" + std::to_string(j + 1), meas_path);
      }
      for (const auto& r : mcsv.rows) {
        const std::string id = r[static_cast<std::size_t>(mid)];
        JV q;
        for (int j = 0; j < 7; ++j) {
          q(j) = num(r, mq[j]);  // already in degrees in endpoint_inputs.csv
        }
        meas_map[id] = q;
      }
    } catch (const std::exception& e) {
      std::cerr << "hold_end_check: error reading meas file: " << e.what() << "\n";
    }
  }

  std::cout << std::fixed << std::setprecision(4);
  std::cout << std::left
            << std::setw(28) << "run_id"
            << std::setw(8)  << "omega"
            << std::setw(4)  << "k"
            << std::setw(8)  << "th_deg"
            << std::setw(6)  << "flip"
            << std::setw(8)  << "end_ok"
            << std::setw(12) << "lim_mar_deg"
            << std::setw(8)  << "joint"
            << std::setw(10) << "w_grasp"
            << std::setw(10) << "w_end"
            << std::setw(18) << "max_diff_meas"
            << "\n";
  std::cout << std::string(118, '-') << "\n";

  int n_total = 0;
  int n_end_ok = 0;

  for (const auto& row : runs_csv.rows) {
    ++n_total;
    const std::string run_id = str(row, col_id);
    const double omega_deg_s = num(row, col_om);
    const double omega_rad_s = omega_deg_s * kDeg2Rad;
    const int k_int = static_cast<int>(num(row, col_k));
    const GraspPointId k_id = static_cast<GraspPointId>(k_int);
    const double theta_deg = num(row, col_th);
    const double theta_rad = theta_deg * kDeg2Rad;
    const std::string flip_str = str(row, col_flip);
    const bool wrist_flip = (flip_str == "true" || flip_str == "1" || flip_str == "True");

    JV q_start;
    for (int j = 0; j < 7; ++j) {
      q_start(j) = num(row, col_qs[j]);  // radians
    }

    // Grasp pose
    const auto grasp_pose = model.graspPoseAt(k_id, theta_rad);
    const Eigen::Vector3d grasp_pos = grasp_pose.position_world;
    const Eigen::Quaterniond grasp_ori = wrist_flip
        ? haptic_dmp_learning::core::applyRoll(grasp_pose.orientation_nominal_world, M_PI)
        : grasp_pose.orientation_nominal_world;

    // Static IK at grasp
    IkParams ik_params;
    const IkResult ik_grasp = solvePoseIk(robot, grasp_pos, grasp_ori, q_start, q_start, ik_params);
    double w_grasp = 0.0;
    if (ik_grasp.converged && ik_grasp.limits_ok) {
      robot.update(ik_grasp.q, JV::Zero());
      w_grasp = fullManipulability(robot.jacobian()).w_full;
    }

    const JV q_seed = ik_grasp.q;

    // Hold-end IK (+3.0 s)
    const HoldPoseIkResult hold_res = holdPoseIk(
        robot, grasp_pos, grasp_ori, center, axis, omega_rad_s, t_follow_s, q_seed, ik_params);

    if (hold_res.end_ok) {
      ++n_end_ok;
    }

    const JointMargins margins = jointMargins(robot, hold_res.q_hold_end);
    const double min_mar_deg = margins.min_margin_rad * kRad2Deg;
    const int worst_joint = margins.worst_joint + 1;

    // Measured comparison
    std::string meas_diff_str = "NON TROVATO";
    if (meas_in_runs) {
      JV q_meas;
      for (int j = 0; j < 7; ++j) q_meas(j) = num(row, col_qmeas[j]);
      const double max_diff = (hold_res.q_hold_end * kRad2Deg - q_meas).cwiseAbs().maxCoeff();
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(3) << max_diff << " deg";
      meas_diff_str = ss.str();
    } else {
      const auto it = meas_map.find(run_id);
      if (it != meas_map.end()) {
        const double max_diff = (hold_res.q_hold_end * kRad2Deg - it->second).cwiseAbs().maxCoeff();
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3) << max_diff << " deg";
        meas_diff_str = ss.str();
      }
    }

    std::cout << std::left
              << std::setw(28) << run_id
              << std::setw(8)  << omega_deg_s
              << std::setw(4)  << k_int
              << std::setw(8)  << theta_deg
              << std::setw(6)  << (wrist_flip ? "B" : "A")
              << std::setw(8)  << (hold_res.end_ok ? "true" : "FALSE")
              << std::setw(12) << min_mar_deg
              << std::setw(8)  << ("j" + std::to_string(worst_joint))
              << std::setw(10) << w_grasp
              << std::setw(10) << hold_res.w_end
              << std::setw(18) << meas_diff_str
              << "\n";
  }

  std::cout << std::string(118, '-') << "\n";
  std::cout << "hold_end_check: total rows = " << n_total << ", end_ok = " << n_end_ok
            << " (" << (n_total > 0 ? (100.0 * n_end_ok / n_total) : 0.0) << "%)\n";

  return 0;
}
