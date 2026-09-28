#include "satellite_grasp_planner/core/selection_inputs.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "haptic_dmp_learning/core/demo_params.hpp"
#include "haptic_dmp_learning/core/grasp_cost.hpp"
#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "satellite_grasp_planner/core/candidate_scan.hpp"
#include "satellite_grasp_planner/core/manipulability.hpp"

namespace satellite_grasp_planner {
namespace core {

using franka_cartesian_control::core::RobotModel;
namespace dp = haptic_dmp_learning::core::demo_params;

namespace {

std::string vecStr(const Eigen::Vector3d& v) {
    std::ostringstream o;
    o << std::setprecision(9) << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
    return o.str();
}

std::string jointsStr(const RobotModel::JointVector& q) {
    std::ostringstream o;
    o << std::setprecision(9) << "(";
    for (int i = 0; i < q.size(); ++i) o << (i ? ", " : "") << q(i);
    return o.str() + ")";
}

std::string num(double v) {
    std::ostringstream o;
    o << std::setprecision(9) << v;
    return o.str();
}

std::string dirOf(const std::string& path) {
    const std::size_t slash = path.find_last_of('/');
    return slash == std::string::npos ? std::string() : path.substr(0, slash + 1);
}

}  // namespace

SelectionInputs buildSelectionInputs(const std::string& demo_params_path, const std::string& urdf_path,
                                     const SatelliteSnapshot& snapshot,
                                     const haptic_dmp_learning::core::CubeSatelliteModel::Params& cube_geometry,
                                     const Eigen::Vector3d& robot_base_world) {
    // a) demo parameters + alignment with the weights file + template
    dp::DemoParams demo;
    std::string weights_path;
    try {
        demo = dp::read(demo_params_path);
        dp::verifyWeightsAlignment(demo_params_path, demo);
        weights_path = dirOf(demo_params_path) + demo.weights_file;
    } catch (const std::exception& e) {
        throw std::runtime_error("buildSelectionInputs: demo parameters file " + demo_params_path + ": " +
                                 e.what());
    }
    SelectionInputs in(haptic_dmp_learning::core::prodmp_io::loadProDmpFromYaml(weights_path));

    // b) robot, q0, p0 (from the robot kinematics)
    std::ifstream urdf(urdf_path);
    if (!urdf.is_open()) {
        throw std::runtime_error("buildSelectionInputs: cannot open URDF " + urdf_path);
    }
    std::stringstream urdf_text;
    urdf_text << urdf.rdbuf();
    std::vector<std::string> joint_names;
    for (int i = 1; i <= RobotModel::kNumJoints; ++i) joint_names.push_back("fer_joint" + std::to_string(i));
    in.robot = std::make_shared<RobotModel>(urdf_text.str(), joint_names, "fer_hand_tcp");

    const bool q0_from_demo = demo.start_joints_rad.has_value();
    if (q0_from_demo) {
        for (int i = 0; i < RobotModel::kNumJoints; ++i) in.q0(i) = (*demo.start_joints_rad)[static_cast<std::size_t>(i)];
    } else {
        in.q0 = RobotModel::readyPose();
    }
    const RobotModel::JointVector zero = RobotModel::JointVector::Zero();
    in.robot->update(in.q0, zero);
    in.p0 = in.robot->eePosition();

    // c) cube model at the measured satellite state
    in.cube_params = cube_geometry;
    in.cube_params.center_world = snapshot.center;
    in.cube_params.axis_world = snapshot.axis;
    const haptic_dmp_learning::core::CubeSatelliteModel model(in.cube_params);
    const double theta_center = windowCenterTheta(model, robot_base_world);

    // d) scan parameters
    ScanParams& scan = in.params.scan;
    scan.theta_center_rad = theta_center;
    scan.omega_rad_s = snapshot.omega_rad_s;
    scan.tau_contact_s = demo.t_contact_s.value_or(0.0);
    scan.p0 = in.p0;
    scan.delta_g_demo = demo.delta_g_demo_m;
    scan.contact_to_end_offset = demo.contact_to_end_offset_m;
    scan.e_v_ref = haptic_dmp_learning::core::meanSurfaceSpeedSquared(model, snapshot.omega_rad_s);
    scan.e_g_ref = demo.delta_g_demo_m.squaredNorm();
    if (!(demo.delta_g_demo_m.norm() >= 1e-9)) {
        throw std::invalid_argument("buildSelectionInputs: |delta_g_demo| < 1e-9 in " + demo_params_path);
    }
    if (scan.tau_contact_s > in.prodmp_template.tau()) {
        throw std::invalid_argument("buildSelectionInputs: tau_contact_s (" + num(scan.tau_contact_s) +
                                    ") exceeds the template tau (" + num(in.prodmp_template.tau()) + ") in " +
                                    demo_params_path);
    }

    // e) w_trans_demo
    double w_trans_demo;
    std::string w_source;
    if (demo.end_joints_rad.has_value()) {
        RobotModel::JointVector q_end;
        for (int i = 0; i < RobotModel::kNumJoints; ++i) q_end(i) = (*demo.end_joints_rad)[static_cast<std::size_t>(i)];
        in.robot->update(q_end, zero);
        w_trans_demo = wTransFromJacobian(in.robot->jacobian().topRows<3>());
        in.w_trans_demo_is_proxy = false;
        w_source = "da demo_params.demo.end_joints_rad, blocco traslazionale del Jacobiano";
    } else {
        w_trans_demo = referenceWTrans(in.prodmp_template, in.robot, in.q0, scan.delta_g_demo);
        in.w_trans_demo_is_proxy = true;
        w_source = "PROXY provvisorio: manipolabilita' a fine replica della demo (referenceWTrans); "
                   "end_joints_rad non disponibili";
    }
    in.robot->update(in.q0, zero);  // leave the robot at q0

    // f), g) selection parameters and flags
    in.params.w_trans_demo = w_trans_demo;
    in.contact_assumed_at_end = !demo.t_contact_s.has_value();

    // h) provenance
    auto& pv = in.provenance;
    pv.push_back(demo.t_contact_s.has_value()
                     ? "tau_contact_s = " + num(scan.tau_contact_s) + " (da demo_params.demo.t_contact_s)"
                     : "tau_contact_s = 0 (t_contact_s null: contatto assunto alla fine)");
    pv.push_back("delta_g_demo_m = " + vecStr(scan.delta_g_demo) + " (da demo_params.demo.delta_g_demo_m)");
    pv.push_back("contact_to_end_offset_m = " + vecStr(scan.contact_to_end_offset) +
                 (demo.t_contact_s.has_value() ? " (da demo_params.demo.contact_to_end_offset_m)"
                                                : " (nullo: t_contact_s null, contatto assunto alla fine)"));
    pv.push_back("q0 = " + jointsStr(in.q0) +
                 (q0_from_demo ? " (da demo_params.demo.start_joints_rad)"
                               : " (RobotModel::readyPose(): start_joints_rad null)"));
    pv.push_back("p0 = " + vecStr(in.p0) + " (cinematica diretta del robot a q0)");
    pv.push_back("w_trans_demo = " + num(w_trans_demo) + " (" + w_source + ")");
    pv.push_back("omega_rad_s = " + num(snapshot.omega_rad_s) + " (dalla snapshot del satellite)");
    pv.push_back("satellite center = " + vecStr(snapshot.center) + " (dalla snapshot)");
    pv.push_back("satellite axis = " + vecStr(snapshot.axis) + " (dalla snapshot)");
    pv.push_back("e_v_ref = " + num(scan.e_v_ref) + " (meanSurfaceSpeedSquared del modello con questo omega)");
    pv.push_back("e_g_ref = " + num(scan.e_g_ref) + " (|delta_g_demo|^2)");
    return in;
}

}  // namespace core
}  // namespace satellite_grasp_planner
