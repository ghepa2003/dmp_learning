#include <iostream>
#include <iomanip>
#include <vector>
#include <memory>
#include <cmath>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "haptic_dmp_learning/core/satellite_intercept.hpp"
#include "satellite_grasp_planner/core/joint_path_simulator.hpp"
#include "satellite_grasp_planner/core/manipulability.hpp"

using namespace satellite_grasp_planner::core;
using namespace haptic_dmp_learning::core;
using namespace haptic_dmp_learning::core::satellite_intercept;
using RobotModel = franka_cartesian_control::core::RobotModel;

const std::vector<std::pair<double, double>> JOINT_LIMITS = {
    {-2.8973, 2.8973},   // q1: [-166.0, 166.0] deg
    {-1.7628, 1.7628},   // q2: [-101.0, 101.0] deg
    {-2.8973, 2.8973},   // q3: [-166.0, 166.0] deg
    {-3.0718, -0.0698},  // q4: [-176.0, -4.0] deg
    {-2.8973, 2.8973},   // q5: [-166.0, 166.0] deg
    {-0.0175, 3.7525},   // q6: [-1.0, 215.0] deg
    {-2.8973, 2.8973},   // q7: [-166.0, 166.0] deg
};

Eigen::Quaterniond slerpShortestPath(const Eigen::Quaterniond& q0, const Eigen::Quaterniond& q1, double s) {
    Eigen::Quaterniond target = q1;
    if (q0.coeffs().dot(target.coeffs()) < 0.0) {
        target.coeffs() = -target.coeffs();
    }
    return q0.slerp(s, target).normalized();
}

struct EvalResult {
    bool feasible = false;
    double max_joint_violation_rad = 0.0;
    double max_q6_violation_rad = 0.0;
    double pos_err_final_mm = 0.0;
    double w_trans_final = 0.0;
    RobotModel::JointVector q_final;
};

EvalResult evaluateCandidate(
    const std::shared_ptr<RobotModel>& robot_model,
    const ProDMP& prodmp_template,
    const RobotModel::JointVector& q0,
    const Eigen::Vector3d& p0,
    const Eigen::Quaterniond& quat0,
    const Eigen::Vector3d& p_goal,
    const Eigen::Quaterniond& quat_goal,
    double tau,
    double dt,
    const JointPathSimulator::Params& sim_params) {

    int n_steps = static_cast<int>(std::ceil(tau / dt));

    ProDMP prodmp = prodmp_template;
    prodmp.setRelativeGoal(false);
    prodmp.setInitialConditions(0.0, p0, Eigen::Vector3d::Zero());
    prodmp.setGoal(p_goal);

    std::vector<CartesianSample> trajectory;
    trajectory.reserve(n_steps);
    for (int i = 0; i < n_steps; ++i) {
        CartesianSample sample;
        sample.position = prodmp.step(dt);
        double s = std::min(1.0, static_cast<double>(i + 1) * dt / tau);
        sample.orientation = slerpShortestPath(quat0, quat_goal, s);
        trajectory.push_back(sample);
    }

    JointPathSimulator sim(robot_model, sim_params);
    auto steps = sim.simulate(trajectory, q0);

    EvalResult res;
    res.q_final = steps.back().q;
    res.w_trans_final = steps.back().w_trans;

    double max_viol = 0.0;
    double max_q6_viol = 0.0;
    for (const auto& step : steps) {
        for (int j = 0; j < 7; ++j) {
            double q_val = step.q(j);
            double min_l = JOINT_LIMITS[j].first;
            double max_l = JOINT_LIMITS[j].second;
            if (q_val < min_l) {
                double v = min_l - q_val;
                if (v > max_viol) max_viol = v;
                if (j == 5 && v > max_q6_viol) max_q6_viol = v;
            } else if (q_val > max_l) {
                double v = q_val - max_l;
                if (v > max_viol) max_viol = v;
                if (j == 5 && v > max_q6_viol) max_q6_viol = v;
            }
        }
    }
    res.max_joint_violation_rad = max_viol;
    res.max_q6_violation_rad = max_q6_viol;

    robot_model->update(res.q_final, RobotModel::JointVector::Zero());
    Eigen::Vector3d p_actual = robot_model->eePosition();
    res.pos_err_final_mm = (p_actual - p_goal).norm() * 1000.0;

    res.feasible = (res.max_joint_violation_rad < 1e-4) && (res.pos_err_final_mm < 2.0);
    return res;
}

void analyzePhase(
    double phase_deg,
    const std::shared_ptr<RobotModel>& robot_model,
    const ProDMP& prodmp_template,
    const RobotModel::JointVector& q0,
    const Eigen::Vector3d& p0,
    const Eigen::Quaterniond& quat0,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& axis,
    const Eigen::Vector3d& demo_disp,
    double tau,
    double dt,
    const JointPathSimulator::Params& sim_params) {

    double theta_rad = phase_deg * M_PI / 180.0;
    AnchoredGoal anchored = anchorAndRotate(p0, demo_disp, center, axis, theta_rad);
    Eigen::Vector3d p_goal = anchored.p_rotated_world;

    Eigen::AngleAxisd rot_sat(theta_rad, axis.normalized());
    Eigen::Matrix3d R_base = rot_sat.toRotationMatrix() * quat0.toRotationMatrix();

    std::cout << "\n====================================================================================\n";
    std::cout << "DETAILED ROLL PROFILE ANALYSIS FOR PHASE theta = " << phase_deg << " deg\n";
    std::cout << "p_goal: [" << p_goal.transpose() << "]\n";
    std::cout << "====================================================================================\n";
    std::cout << std::setw(6) << "psi" << " | "
              << std::setw(6) << "Feas?" << " | "
              << std::setw(7) << "Viol" << " | "
              << std::setw(6) << "q1" << " "
              << std::setw(6) << "q2" << " "
              << std::setw(6) << "q3" << " "
              << std::setw(6) << "q4" << " "
              << std::setw(6) << "q5" << " "
              << std::setw(6) << "q6" << " "
              << std::setw(6) << "q7" << " | "
              << std::setw(7) << "|q6-107|" << " | "
              << std::setw(7) << "w_trans" << "\n";
    std::cout << "------------------------------------------------------------------------------------\n";

    double q6_center_deg = 107.0;

    // Scan from -100 to +60 with 2 deg step
    for (int psi_int = -100; psi_int <= 60; psi_int += 2) {
        double psi_deg = static_cast<double>(psi_int);
        double psi_rad = psi_deg * M_PI / 180.0;

        Eigen::Matrix3d R_psi = R_base * Eigen::AngleAxisd(psi_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Quaterniond quat_goal_psi(R_psi);
        quat_goal_psi.normalize();

        EvalResult res = evaluateCandidate(
            robot_model, prodmp_template, q0, p0, quat0, p_goal, quat_goal_psi, tau, dt, sim_params);

        std::string s_feas = res.feasible ? "OK" : "FAIL";
        double q6_deg = res.q_final(5) * 180.0 / M_PI;
        double q6_err = std::abs(q6_deg - q6_center_deg);

        // Print only feasible or near feasible
        if (res.feasible || res.max_joint_violation_rad < 0.2) {
            std::cout << std::setw(5) << psi_deg << "° | "
                      << std::setw(6) << s_feas << " | "
                      << std::setw(7) << std::setprecision(3) << res.max_joint_violation_rad << " | "
                      << std::setw(6) << std::setprecision(1) << (res.q_final(0)*180.0/M_PI) << " "
                      << std::setw(6) << (res.q_final(1)*180.0/M_PI) << " "
                      << std::setw(6) << (res.q_final(2)*180.0/M_PI) << " "
                      << std::setw(6) << (res.q_final(3)*180.0/M_PI) << " "
                      << std::setw(6) << (res.q_final(4)*180.0/M_PI) << " "
                      << std::setw(6) << (res.q_final(5)*180.0/M_PI) << " "
                      << std::setw(6) << (res.q_final(6)*180.0/M_PI) << " | "
                      << std::setw(7) << std::setprecision(1) << q6_err << " | "
                      << std::setw(7) << std::setprecision(4) << res.w_trans_final << "\n";
        }
    }
}

int main() {
    std::string urdf_path = "/root/thesis_ws/fer_flat_effort.urdf";
    std::string weights_path = "/root/thesis_ws/runs/20260915_090358_fit_reach_task_baseline_prodmp/weights.yaml";

    std::ifstream f(urdf_path);
    if (!f.is_open()) {
        std::cerr << "Cannot open URDF: " << urdf_path << std::endl;
        return 1;
    }
    std::stringstream buffer;
    buffer << f.rdbuf();
    std::string urdf_content = buffer.str();

    std::vector<std::string> joint_names;
    for (int i = 1; i <= 7; ++i) joint_names.push_back("fer_joint" + std::to_string(i));

    auto robot_model = std::make_shared<RobotModel>(urdf_content, joint_names, "fer_hand_tcp");

    ProDMP prodmp_template = prodmp_io::loadProDmpFromYaml(weights_path);
    double tau = prodmp_template.tau();
    Eigen::Vector3d demo_disp = prodmp_template.demoDisplacement();

    RobotModel::JointVector q0;
    q0 << 0.0, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, 1.5707963267948966, 0.7853981633974483;

    robot_model->update(q0, RobotModel::JointVector::Zero());
    Eigen::Vector3d p0 = robot_model->eePosition();
    Eigen::Quaterniond quat0 = robot_model->eeOrientation();

    Eigen::Vector3d center(0.45, -0.05, 0.35);
    Eigen::Vector3d axis(1.0, 0.0, 0.0);

    double dt = 0.005;

    NullspaceBiasedResolution::Params ns_params;
    ns_params.nullspace_gain = 0.2236;
    ns_params.joint1_nullspace_gain_scale = 10.0;
    JointPathSimulator::Params sim_params;
    sim_params.dt = dt;
    sim_params.strategy = std::make_shared<NullspaceBiasedResolution>(ns_params);

    std::cout << std::fixed;
    analyzePhase(330.0, robot_model, prodmp_template, q0, p0, quat0, center, axis, demo_disp, tau, dt, sim_params);
    analyzePhase(345.0, robot_model, prodmp_template, q0, p0, quat0, center, axis, demo_disp, tau, dt, sim_params);

    return 0;
}
