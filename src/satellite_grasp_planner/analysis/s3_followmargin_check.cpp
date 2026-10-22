// s3_followmargin_check: reproduces the G16 / step 0c predicted joint margins with core::followMargin().
//
// Acceptance criterion of followMargin(): every row with an expected value below must match it within 0.1 deg (the
// joint and the time are printed for the comparison with g16_results.csv; they are not enforced). Not run by the author:
// to be run offline from the workspace root (relative default paths, as _diag_1019/step0c_margins.cpp):
//   ros2 run satellite_grasp_planner s3_followmargin_check [--weights-dir DIR] [--urdf FILE]
//
// Scenario (identical to _diag_1019/step0c_margins.cpp and run_rollout_follow_all.cpp):
//   weights_demo_params.yaml = runs/20261015_095418_fit_demo_20261014T150851/weights_demo_params.yaml, URDF fer_flat_effort.urdf
//   satellite: centre (0.70, 0, 0.77), axis +z, theta_acq = -1.6727670737402633 rad, t_acq = 18.318 s
//   cube: side 0.150, standoff 0.08, collar_radius 0.0725, wall 0.009, grasp_radius 0.068, grasp_depth 0.0125, face normal body +x
//   q_start = {-9e-06, -0.786572, 2e-06, -2.378914, -6e-06, 3.141609, 0.785398} (seed); nullspace reference q_ref = readyPose()
//   target = graspPoseAt(k, theta).position_world (NO contact_to_end_offset), orientation nominal (branch A) or
//            applyRoll(nominal, pi) (branch B), psi = 0; follow 3.0 s; omega constant per row.
//
// Expected values (deg; joint 1-based fer_jointN; t_rel to AT_END):
//   G16 columns pred_min_margin_*_0c (_diag_1019/g16/g16_results.csv), step0c_margins.cpp (dt_follow = 0.01):
//     omega -5  k1 A theta -178 : 13.26  fer_joint7  t_rel +3.00
//     omega -5  k3 A theta  178 : 17.67  fer_joint6  t_rel -7.78
//     omega -5  k1 A theta -166 : 35.00  fer_joint6  t_rel -24.47
//     omega -5  k1 A theta -156 : 35.00  fer_joint6  t_rel -24.47
//   A8 (k3 A theta 171, omega -2): 20.84 fer_joint6 t_rel -8.39 is from run_rollout_follow_all.cpp with dt_follow = 0.1
//     (_diag_1019/pred_rollout.json, scaleA_A8: 20.8419). The same row with dt_follow = 0.01 has NO recorded value: it is
//     printed as informational (the difference between the two steps is exactly what this row shows).
//   omega -4  k0 A theta -163.2: 28.63. Reported by the author; NO file in _diag_1019/ records it. Assumed to come from the
//     step0c_margins.cpp row "k0 A -163.2" (omega -4, dt_follow = 0.01); joint and time unknown.
//
// Not reproduced here (not in the scenario above): anything about clearance or the selector.

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/tcp_roll.hpp"
#include "satellite_grasp_planner/core/follow_margin.hpp"
#include "satellite_grasp_planner/core/selection_inputs.hpp"
#include "satellite_grasp_planner/core/start_state.hpp"

using namespace satellite_grasp_planner::core;
using haptic_dmp_learning::core::CubeSatelliteModel;
using haptic_dmp_learning::core::GraspPointId;
using RobotModel = franka_cartesian_control::core::RobotModel;

namespace {

struct Row {
  const char* name;
  int k;
  int flip;  // 0 = branch A, 1 = branch B
  double theta_deg;
  double omega_deg_s;
  double dt_follow_s;
  double expected_deg;  // NaN: informational, not enforced
  int expected_joint;   // 1-based, 0 = unknown
  double expected_t_rel_s;
  const char* source;
};

}  // namespace

int main(int argc, char** argv) {
  std::string weights_dir = "runs/20261015_095418_fit_demo_20261014T150851";
  std::string urdf = "fer_flat_effort.urdf";
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string a = argv[i];
    if (a == "--weights-dir")
      weights_dir = argv[i + 1];
    else if (a == "--urdf")
      urdf = argv[i + 1];
    else {
      std::cerr << "unknown argument " << a << "\n";
      return 2;
    }
  }
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  constexpr double kTolDeg = 0.1;
  const std::vector<Row> rows = {
      {"w-5 k1A -178", 1, 0, -178.0, -5.0, 0.01, 13.26, 7, 3.00, "g16_results.csv pred_*_0c"},
      {"w-5 k3A  178", 3, 0, 178.0, -5.0, 0.01, 17.67, 6, -7.78, "g16_results.csv pred_*_0c"},
      {"w-5 k1A -166", 1, 0, -166.0, -5.0, 0.01, 35.00, 6, -24.47, "g16_results.csv pred_*_0c"},
      {"w-5 k1A -156", 1, 0, -156.0, -5.0, 0.01, 35.00, 6, -24.47, "g16_results.csv pred_*_0c"},
      {"A8 k3A 171 (dt_f 0.1)", 3, 0, 171.0, -2.0, 0.1, 20.84, 6, -8.39, "pred_rollout.json scaleA_A8 (run_rollout_follow_all)"},
      {"A8 k3A 171 (dt_f 0.01)", 3, 0, 171.0, -2.0, 0.01, kNaN, 0, 0.0, "informational (no recorded value)"},
      {"w-4 k0A -163.2", 0, 0, -163.2, -4.0, 0.01, 28.63, 0, 0.0, "reported by the author; step0c row assumed"},
  };

  SatelliteSnapshot snap;
  snap.center = Eigen::Vector3d(0.70, 0.0, 0.77);
  snap.axis = Eigen::Vector3d::UnitZ();
  snap.theta_rad = -1.6727670737402633;
  snap.t_s = 18.318;

  CubeSatelliteModel::Params cube;
  cube.center_world = snap.center;
  cube.axis_world = snap.axis;
  cube.face_normal_body = Eigen::Vector3d(1.0, 0.0, 0.0);
  cube.cube_side_m = 0.150;
  cube.standoff_m = 0.08;
  cube.collar_radius_m = 0.0725;
  cube.wall_thickness_m = 0.009;
  cube.grasp_radius_m = 0.068;
  cube.grasp_depth_m = 0.0125;

  const std::vector<double> q_start_vec = {-9e-06, -0.786572, 2e-06, -2.378914, -6e-06, 3.141609, 0.785398};
  const StartState start = makeStartStateFromOverride(q_start_vec, urdf);
  const SelectionInputs in = buildSelectionInputs(weights_dir + "/weights_demo_params.yaml", urdf, snap, cube, start);
  const CubeSatelliteModel model(in.cube_params);
  const RobotModel::JointVector q_seed = in.q0;
  const RobotModel::JointVector q_ref = RobotModel::readyPose();

  int n_fail = 0;
  std::cout << std::fixed << std::setprecision(2);
  std::cout << std::left << std::setw(26) << "row" << std::setw(11) << "margin_deg" << std::setw(11) << "expected" << std::setw(10)
            << "joint" << std::setw(10) << "t_rel_s" << std::setw(10) << "exp_t"
            << "result\n";
  for (const Row& r : rows) {
    const auto pose = model.graspPoseAt(static_cast<GraspPointId>(r.k), r.theta_deg * M_PI / 180.0);
    const Eigen::Quaterniond ori = r.flip == 0 ? pose.orientation_nominal_world
                                               : haptic_dmp_learning::core::applyRoll(pose.orientation_nominal_world, M_PI);
    FollowMarginParams fp;
    fp.follow_s = 3.0;
    fp.dt_approach_s = 0.005;
    fp.dt_follow_s = r.dt_follow_s;
    const FollowMarginResult fm = followMargin(in.prodmp_template, pose.position_world, ori, snap.center, snap.axis,
                                               r.omega_deg_s * M_PI / 180.0, in.robot, q_seed, q_ref, fp);
    const double deg = fm.min_margin_rad * 180.0 / M_PI;
    std::string verdict = "info";
    if (!std::isnan(r.expected_deg)) {
      const bool ok = std::abs(deg - r.expected_deg) <= kTolDeg;
      verdict = ok ? "PASS" : "FAIL";
      if (!ok) ++n_fail;
    }
    std::cout << std::left << std::setw(26) << r.name << std::setw(11) << deg << std::setw(11)
              << (std::isnan(r.expected_deg) ? std::string("-") : std::to_string(r.expected_deg).substr(0, 5)) << "fer_j"
              << std::setw(5) << fm.joint + 1 << std::setw(10) << fm.t_rel_s << std::setw(10)
              << (r.expected_joint ? std::to_string(r.expected_t_rel_s).substr(0, 6) : std::string("?")) << verdict
              << "  [exp joint " << (r.expected_joint ? std::to_string(r.expected_joint) : std::string("?")) << "; " << r.source
              << "]\n";
  }
  std::cout << (n_fail == 0 ? "ALL ENFORCED ROWS WITHIN " : "FAILED ROWS (tolerance ") << kTolDeg << " deg\n";
  return n_fail == 0 ? 0 : 1;
}
