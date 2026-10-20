// s3_clearance: clearance (signed distance gripper <-> rotating satellite, window [0, tau]) of ONE candidate
// (k, theta, flip) at a given satellite rate omega. Same scenario inputs as s3_landscape (--params, --center,
// --theta-snap-rad, --t-snap-s, --weights-dir/demo, --urdf, --q-start, --depth); the trajectory is the planner's
// (timedApproachTrajectory: the TCP path the executor publishes on /target_pose), the model is approachClearance()
// as used by the v2 constraint. Prints one line:
//   CLEARANCE k=.. theta=.. flip=.. omega=.. min_mm=.. coppia=.. t_rel_s=.. min_mm_senza_ultimi_2s=..
// --selftest: synthetic cases with known values (no URDF, no demo): exit 0 if they hold, 3 otherwise.
// Build/run: analysis/README.md (BUILD_ANALYSIS_TOOLS).

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "satellite_grasp_planner/core/approach_clearance.hpp"
#include "satellite_grasp_planner/core/grasp_pose_eval.hpp"
#include "satellite_grasp_planner/core/grasp_s3_selection.hpp"
#include "satellite_grasp_planner/core/kinematic_feasibility.hpp"
#include "satellite_grasp_planner/core/launch_plan.hpp"
#include "satellite_grasp_planner/core/selection_inputs.hpp"
#include "satellite_grasp_planner/core/start_state.hpp"

using namespace satellite_grasp_planner::core;
using haptic_dmp_learning::core::CubeSatelliteModel;
using haptic_dmp_learning::core::GraspPointId;

namespace {

constexpr double kPi = M_PI;
// Defaults = the 15/10/2026 run, as s3_landscape.cpp (kRunQStart, kRun15*).
const char* kRunQStart = "-0.000009,-0.786572,0.000002,-2.378914,-0.000006,3.141609,0.785398";
constexpr double kRun15ThetaSnapRad = -1.6727670737402633;
constexpr double kRun15TSnapS = 18.318;
const char* kDefaultParamsRel = "config/grasp_planner_example.yaml";

struct Args {
    std::string weights_dir = "runs/20261015_095418_fit_demo_20261014T150851";
    std::string params;
    bool params_explicit = false;
    std::string urdf;
    std::string hand_mesh;  // optional: STL path; default = the fer_hand collision mesh of the URDF
    std::string q_start = kRunQStart;
    double omega_deg_s = -2.0;
    double depth = 0.0125;
    std::string center = "0.70,0.0,0.77";
    double theta_snap_rad = kRun15ThetaSnapRad;
    double t_snap_s = kRun15TSnapS;
    int k = -1;
    double theta_deg = 0.0;
    bool have_theta = false;
    int flip = -1;
    bool selftest = false;
};

std::string trim(const std::string& s) {
    const auto a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    return s.substr(a, s.find_last_not_of(" \t\r\n") - a + 1);
}

std::vector<double> parseList(const std::string& s, std::size_t n, const char* what) {
    std::vector<double> v;
    std::stringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, ',')) v.push_back(std::stod(tok));
    if (v.size() != n) throw std::invalid_argument(std::string(what) + ": expected " + std::to_string(n) + " values");
    return v;
}

/// Flat `key: value` lines of the `grasp_planner_node:` block (same minimal reader as s3_landscape.cpp).
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

std::string resolveDefaultParams() {
    std::vector<std::string> cand;
    if (const char* ap = std::getenv("AMENT_PREFIX_PATH")) {
        std::stringstream ss(ap);
        std::string pre;
        while (std::getline(ss, pre, ':')) {
            if (!pre.empty()) cand.push_back(pre + "/share/satellite_grasp_planner/" + kDefaultParamsRel);
        }
    }
    cand.push_back(std::string(__FILE__).substr(0, std::string(__FILE__).rfind("/analysis/")) + "/" + kDefaultParamsRel);
    cand.push_back(kDefaultParamsRel);
    for (const auto& c : cand) {
        if (std::ifstream(c).good()) return c;
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
        if (k == "--weights-dir") a.weights_dir = val();
        else if (k == "--params") {
            a.params = val();
            a.params_explicit = true;
        }
        else if (k == "--urdf") a.urdf = val();
        else if (k == "--hand-mesh") a.hand_mesh = val();
        else if (k == "--q-start") a.q_start = val();
        else if (k == "--omega-deg-s") a.omega_deg_s = std::stod(val());
        else if (k == "--depth") a.depth = std::stod(val());
        else if (k == "--center") a.center = val();
        else if (k == "--theta-snap-rad") a.theta_snap_rad = std::stod(val());
        else if (k == "--t-snap-s") a.t_snap_s = std::stod(val());
        else if (k == "--k") a.k = std::stoi(val());
        else if (k == "--theta-deg") {
            a.theta_deg = std::stod(val());
            a.have_theta = true;
        }
        else if (k == "--flip") a.flip = std::stoi(val());
        else if (k == "--selftest") a.selftest = true;
        else throw std::invalid_argument("unknown argument " + k);
    }
    if (a.selftest) return a;
    if (a.k < 0 || a.k > 3) throw std::invalid_argument("--k 0..3 is required");
    if (!a.have_theta) throw std::invalid_argument("--theta-deg is required");
    if (a.flip != 0 && a.flip != 1) throw std::invalid_argument("--flip 0|1 is required");
    if (!std::isfinite(a.omega_deg_s)) throw std::invalid_argument("--omega-deg-s must be finite");
    if (a.params_explicit && a.params.empty()) throw std::invalid_argument("--params is empty");
    if (!a.params_explicit) a.params = resolveDefaultParams();
    if (a.urdf.empty()) {
        const char* home = std::getenv("HOME");
        a.urdf = std::string(home ? home : "/root") + "/thesis_ws/fer_flat_effort.urdf";
    }
    return a;
}

void printLine(int k, double theta_deg, int flip, double omega_deg_s, const ClearanceResult& r) {
    std::cout << std::fixed << std::setprecision(3) << "CLEARANCE k=" << k << " theta=" << theta_deg << " flip=" << flip
              << " omega=" << omega_deg_s << " min_mm=" << r.min_mm << " coppia=" << r.pair.str()
              << " t_rel_s=" << r.t_rel_s << " min_mm_senza_ultimi_2s=" << r.min_mm_no_tail << "\n";
}

// ---- --selftest: known values, synthetic hand (2 cm cube at the hand origin), real fingers and satellite ----

int selftest() {
    CubeSatelliteModel::Params cube;  // defaults: side 0.150, standoff 0.08, R 0.0725, wall 0.009, n = +x
    cube.center_world = Eigen::Vector3d::Zero();
    cube.axis_world = Eigen::Vector3d::UnitZ();
    const HandMesh hand = makeBoxHandMesh(Eigen::Vector3d(0.01, 0.01, 0.01));
    std::cout << describeHandMesh(hand) << " (synthetic)\n";
    const ApproachClearanceModel model(cube, hand);

    auto stationary = [](const Eigen::Vector3d& p, const Eigen::Quaterniond& q, double tau) {
        ApproachSample a, b;
        a.t_s = 0.0;
        b.t_s = tau;
        a.position = b.position = p;
        a.orientation = b.orientation = q;
        return std::vector<ApproachSample>{a, b};
    };
    bool ok = true;
    auto check = [&](const char* what, double got, double want, double tol) {
        const bool good = std::abs(got - want) <= tol;
        ok = ok && good;
        std::cout << (good ? "  ok   " : "  FAIL ") << what << ": got " << got << ", expected " << want << " +- " << tol << "\n";
    };
    // Hand z-axis along +x (R = Ry(90 deg)), fingers open 0.060: the finger tips' +x faces are at TCP.x - 0.1034 + 0.1129;
    // with TCP.x = -0.1845 they are 0.100 m from the cube's -x face (x = -0.075).
    const Eigen::Quaterniond along_x(Eigen::AngleAxisd(kPi / 2, Eigen::Vector3d::UnitY()));
    const auto r = approachClearance(model, stationary(Eigen::Vector3d(-0.1845, 0.0, 0.0), along_x, 5.0), 5.0, 0.0, 0.0,
                                     Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), ClearanceParams());
    printLine(0, 0.0, 0, 0.0, r);
    check("face to face, min_mm", r.min_mm, 100.0, 0.01);
    check("face to face, min_mm_senza_ultimi_2s", r.min_mm_no_tail, 100.0, 0.01);
    // Same hand 90 mm closer: 10 mm.
    const auto r2 = approachClearance(model, stationary(Eigen::Vector3d(-0.0945, 0.0, 0.0), along_x, 5.0), 5.0, 0.0, 0.0,
                                      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), ClearanceParams());
    check("closer by 90 mm, min_mm", r2.min_mm, 10.0, 0.01);
    std::cout << (ok ? "SELFTEST OK\n" : "SELFTEST FAIL\n");
    return ok ? 0 : 3;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args a = parseArgs(argc, argv);
        if (a.selftest) return selftest();

        // ---- planner parameters: defaults < --params YAML (only what this tool needs) ----
        S3Params s3;
        CubeSatelliteModel::Params cube;
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
            d("s1.angle_scale_rad", s3.angle_scale_rad);
            d("s1.ik.pos_tol_m", s3.ik.pos_tol_m);
            d("s1.ik.ori_tol_rad", s3.ik.ori_tol_rad);
            d("s1.ik.damping", s3.ik.damping);
            d("s1.ik.nullspace_gain", s3.ik.nullspace_gain);
            d("s1.ik.max_step_rad", s3.ik.max_step_rad);
            if (kv.count("s1.ik.max_iter")) s3.ik.max_iter = std::stoi(kv.at("s1.ik.max_iter"));
            d("s1.clearance_dt_s", s3.clearance_dt_s);
            d("s1.gripper_open_m", s3.gripper_open_m);
            d("cube_geometry.cube_side_m", cube.cube_side_m);
            d("cube_geometry.standoff_m", cube.standoff_m);
            d("cube_geometry.collar_radius_m", cube.collar_radius_m);
            d("cube_geometry.wall_thickness_m", cube.wall_thickness_m);
            cube.grasp_radius_m = std::stod(kv.at("cube_geometry.grasp_radius_m"));
            if (kv.count("cube_geometry.face_normal_body")) {
                const auto v = yamlList(kv.at("cube_geometry.face_normal_body"));
                if (v.size() == 3) cube.face_normal_body = Eigen::Vector3d(v[0], v[1], v[2]);
            }
        }
        validateClearanceParams(ClearanceParams{s3.clearance_dt_s, s3.gripper_open_m});
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
            start = makeStartStateFromOverride(parseList(a.q_start, 7, "--q-start"), a.urdf);
        }

        SelectionInputs in = buildSelectionInputs(a.weights_dir + "/weights_demo_params.yaml", a.urdf, snap, cube, start);

        // ---- the candidate, evaluated as the selector does (psi = 0, branch forced by --flip) ----
        const CubeSatelliteModel model(in.cube_params);
        RobotModel& robot = *in.robot;
        robot.update(in.q0, RobotModel::JointVector::Zero());
        DemoReference demo;
        demo.position = in.p0 + in.delta_g_demo;
        demo.orientation = robot.eeOrientation();
        demo.w_trans_demo = in.w_trans_demo;
        const double theta = a.theta_deg * kPi / 180.0;
        const PoseEvaluationDetail det = evaluateGraspPoseDetailed(model, static_cast<GraspPointId>(a.k), theta, 0.0, robot, in.q0,
                                                                   demo, s3.length_scale_m, s3.angle_scale_rad, s3.ik, false, a.flip);
        if (!det.eval.feasible_basic) {
            std::cerr << "warning: the IK of branch " << a.flip << " does not converge for this candidate (the clearance is "
                         "computed anyway: it only needs the commanded TCP path)\n";
        }
        robot.update(in.q0, RobotModel::JointVector::Zero());
        const Eigen::Vector3d p0 = robot.eePosition();
        const Eigen::Quaterniond quat0 = robot.eeOrientation();
        const Eigen::Vector3d goal = det.eval.pose.position_world + in.contact_to_end_offset;
        const std::vector<ApproachSample> traj =
            timedApproachTrajectory(in.prodmp_template, p0, quat0, goal, goalOrientation(det.eval), 0.005);
        const double tau = in.prodmp_template.tau();

        // ---- hand mesh (once), then the clearance ----
        const HandMesh hand_mesh = a.hand_mesh.empty() ? loadHandCollisionMesh(a.urdf) : loadHandMeshFile(a.hand_mesh);
        std::cout << describeHandMesh(hand_mesh) << "\n";
        const ApproachClearanceModel cmodel(in.cube_params, hand_mesh);
        const ClearanceResult r = approachClearance(cmodel, traj, tau, theta, snap.omega_rad_s, snap.center, snap.axis,
                                                    ClearanceParams{s3.clearance_dt_s, s3.gripper_open_m});
        std::cerr << std::fixed << std::setprecision(3) << "tau=" << tau << " s  dt=" << s3.clearance_dt_s << " s  gripper_open="
                  << s3.gripper_open_m << " m  goal=[" << goal.transpose()
                  << "]\nno-tail minimum: " << r.min_mm_no_tail << " mm " << r.pair_no_tail.str() << " at t_rel "
                  << r.t_rel_no_tail_s << " s\n";
        printLine(a.k, a.theta_deg, a.flip, a.omega_deg_s, r);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "s3_clearance: " << e.what() << "\n";
        return 2;
    }
}
