#include "satellite_grasp_planner/core/selection_inputs.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "haptic_dmp_learning/core/demo_params.hpp"
#include "haptic_dmp_learning/core/prodmp_io.hpp"
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
                                     const std::optional<StartState>& start) {
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
    in.weights_sha256 = demo.weights_sha256;  // == actual file hash: verifyWeightsAlignment threw otherwise

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

    const bool q0_from_start = start.has_value();
    const bool q0_from_demo = !q0_from_start && demo.start_joints_rad.has_value();
    if (q0_from_start) {
        // Same validation the node applies at startup, against the limits of THIS robot model.
        const std::vector<double> values(start->q.data(), start->q.data() + RobotModel::kNumJoints);
        validateStartJoints(values, in.robot->jointLowerLimits(), in.robot->jointUpperLimits());
        in.q0 = start->q;
        in.q0_source = start->source;
    } else if (q0_from_demo) {
        for (int i = 0; i < RobotModel::kNumJoints; ++i) in.q0(i) = (*demo.start_joints_rad)[static_cast<std::size_t>(i)];
        in.q0_source = "demo_params.demo.start_joints_rad";
    } else {
        in.q0 = RobotModel::readyPose();
        in.q0_source = "RobotModel::readyPose()";
    }
    const RobotModel::JointVector zero = RobotModel::JointVector::Zero();
    in.robot->update(in.q0, zero);
    in.p0 = in.robot->eePosition();

    // c) cube model at the measured satellite state
    in.cube_params = cube_geometry;
    in.cube_params.center_world = snapshot.center;
    in.cube_params.axis_world = snapshot.axis;
    in.snapshot = snapshot;

    // d) demo values
    in.omega_rad_s = snapshot.omega_rad_s;
    in.tau_contact_s = demo.t_contact_s.value_or(0.0);
    in.delta_g_demo = demo.delta_g_demo_m;
    in.contact_to_end_offset = demo.contact_to_end_offset_m;
    if (!(demo.delta_g_demo_m.norm() >= 1e-9)) {
        throw std::invalid_argument("buildSelectionInputs: |delta_g_demo| < 1e-9 in " + demo_params_path);
    }
    if (in.tau_contact_s > in.prodmp_template.tau()) {
        throw std::invalid_argument("buildSelectionInputs: tau_contact_s (" + num(in.tau_contact_s) +
                                    ") exceeds the template tau (" + num(in.prodmp_template.tau()) + ") in " +
                                    demo_params_path);
    }

    // e) w_trans_demo
    if (!demo.end_joints_rad.has_value()) {
        throw std::runtime_error(
            "buildSelectionInputs: " + demo_params_path +
            " has no demo.end_joints_rad, needed for w_trans_demo: redo the fit with the recorded joint states "
            "(the demo's joint_states CSV next to the demo CSV).");
    }
    RobotModel::JointVector q_end;
    for (int i = 0; i < RobotModel::kNumJoints; ++i) q_end(i) = (*demo.end_joints_rad)[static_cast<std::size_t>(i)];
    in.robot->update(q_end, zero);
    in.w_trans_demo = wTransFromJacobian(in.robot->jacobian().topRows<3>());
    in.robot->update(in.q0, zero);  // leave the robot at q0

    // f) flags
    in.contact_assumed_at_end = !demo.t_contact_s.has_value();

    // g) provenance
    auto& pv = in.provenance;
    pv.push_back(demo.t_contact_s.has_value()
                     ? "tau_contact_s = " + num(in.tau_contact_s) + " (da demo_params.demo.t_contact_s)"
                     : "tau_contact_s = 0 (t_contact_s null: contatto assunto alla fine)");
    pv.push_back("delta_g_demo_m = " + vecStr(in.delta_g_demo) + " (da demo_params.demo.delta_g_demo_m)");
    pv.push_back("contact_to_end_offset_m = " + vecStr(in.contact_to_end_offset) +
                 (demo.t_contact_s.has_value() ? " (da demo_params.demo.contact_to_end_offset_m)"
                                                : " (nullo: t_contact_s null, contatto assunto alla fine)"));
    pv.push_back("q0 = " + jointsStr(in.q0) +
                 (q0_from_start
                      ? " (da " + start->source +
                            (demo.start_joints_rad.has_value() ? "; demo_params.demo.start_joints_rad IGNORATO" : "") + ")"
                      : q0_from_demo ? " (da demo_params.demo.start_joints_rad)"
                                     : " (RobotModel::readyPose(): start_joints_rad null)"));
    pv.push_back("p0 = " + vecStr(in.p0) + " (cinematica diretta del robot a q0)");
    pv.push_back("w_trans_demo = " + num(in.w_trans_demo) +
                 " (da demo_params.demo.end_joints_rad, blocco traslazionale del Jacobiano)");
    pv.push_back("omega_rad_s = " + num(snapshot.omega_rad_s) + " (dalla snapshot del satellite)");
    pv.push_back("satellite center = " + vecStr(snapshot.center) + " (dalla snapshot)");
    pv.push_back("satellite axis = " + vecStr(snapshot.axis) + " (dalla snapshot)");
    return in;
}

std::string describeStartState(const SelectionInputs& in) {
    const StartDistance d = distanceFromReadyPose(*in.robot, in.q0);
    std::ostringstream o;
    o << std::setprecision(9);
    o << "start state: q0 source = " << in.q0_source << "\n"
      << "  q0 = " << jointsStr(in.q0) << "\n"
      << "  p0 = " << vecStr(in.p0) << "\n"
      << "  distance from readyPose: joints norm " << d.joint_norm_rad << " rad (" << d.joint_norm_rad * 180.0 / M_PI
      << " deg), max " << d.joint_max_rad << " rad (" << d.joint_max_rad * 180.0 / M_PI << " deg) on fer_joint"
      << d.joint_max_index << "; task space " << d.task_mm << " mm\n"
      << "  provenance:";
    for (const std::string& line : in.provenance) o << "\n    - " << line;
    return o.str();
}

}  // namespace core
}  // namespace satellite_grasp_planner
