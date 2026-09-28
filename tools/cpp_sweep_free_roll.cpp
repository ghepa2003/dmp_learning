#include <iostream>
#include <iomanip>
#include <vector>
#include <memory>
#include <cmath>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <omp.h>

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

// Joint limits for Franka Emika Panda
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
    double ori_err_final_deg = 0.0;
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

    // Check joint limits across all steps
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

    // FK at final config
    robot_model->update(res.q_final, RobotModel::JointVector::Zero());
    Eigen::Vector3d p_actual = robot_model->eePosition();
    Eigen::Quaterniond quat_actual = robot_model->eeOrientation();

    res.pos_err_final_mm = (p_actual - p_goal).norm() * 1000.0;

    Eigen::Quaterniond q_diff = quat_goal.conjugate() * quat_actual;
    if (q_diff.w() < 0.0) q_diff.coeffs() = -q_diff.coeffs();
    double angle_rad = 2.0 * std::atan2(q_diff.vec().norm(), std::abs(q_diff.w()));
    res.ori_err_final_deg = angle_rad * 180.0 / M_PI;

    res.feasible = (res.max_joint_violation_rad < 1e-4) && (res.pos_err_final_mm < 2.0);
    return res;
}

struct PhaseSummary {
    double phase_deg;
    Eigen::Vector3d approach_dir;
    EvalResult res_base;
    double best_psi_deg;
    EvalResult res_opt;
    double psi_min_feas;
    double psi_max_feas;
    bool has_feasible_roll;
};

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

    auto robot_model_master = std::make_shared<RobotModel>(urdf_content, joint_names, "fer_hand_tcp");

    // Load real ProDMP template
    ProDMP prodmp_template = prodmp_io::loadProDmpFromYaml(weights_path);
    double tau = prodmp_template.tau();
    Eigen::Vector3d demo_disp = prodmp_template.demoDisplacement();

    // Ready pose
    RobotModel::JointVector q0;
    q0 << 0.0, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, 1.5707963267948966, 0.7853981633974483;

    robot_model_master->update(q0, RobotModel::JointVector::Zero());
    Eigen::Vector3d p0 = robot_model_master->eePosition();
    Eigen::Quaterniond quat0 = robot_model_master->eeOrientation();

    // Satellite rotation center and axis: CORRECTED TO X-AXIS [1, 0, 0]
    Eigen::Vector3d center(0.45, -0.05, 0.35);
    Eigen::Vector3d axis(1.0, 0.0, 0.0); // CORRECTED: x-axis

    double dt = 0.005;

    std::vector<double> phases;
    for (int p = 0; p <= 360; p += 15) phases.push_back(static_cast<double>(p));

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "====================================================================================================================================\n";
    std::cout << "TASK 2: DETERMINISTIC 2-DOF GRASP + FREE ROLL OPTIMIZATION SWEEP\n";
    std::cout << "Satellite Rotation Axis: X = [1, 0, 0], Center = [" << center.transpose() << "]\n";
    std::cout << "Ready EE Pos: [" << p0.transpose() << "], Demo Disp: [" << demo_disp.transpose() << "]\n";
    std::cout << "====================================================================================================================================\n" << std::flush;

    std::vector<PhaseSummary> summaries(phases.size());

    #pragma omp parallel for schedule(dynamic)
    for (size_t i = 0; i < phases.size(); ++i) {
        double phase_deg = phases[i];
        auto local_robot_model = std::make_shared<RobotModel>(urdf_content, joint_names, "fer_hand_tcp");

        NullspaceBiasedResolution::Params ns_params;
        ns_params.nullspace_gain = 0.2236;
        ns_params.joint1_nullspace_gain_scale = 10.0;
        JointPathSimulator::Params sim_params;
        sim_params.dt = dt;
        sim_params.strategy = std::make_shared<NullspaceBiasedResolution>(ns_params);

        double theta_rad = phase_deg * M_PI / 180.0;
        AnchoredGoal anchored = anchorAndRotate(p0, demo_disp, center, axis, theta_rad);
        Eigen::Vector3d p_goal = anchored.p_rotated_world;

        // Base rigid rotation: R_sat(theta, x) * R_demo
        Eigen::AngleAxisd rot_sat(theta_rad, axis.normalized());
        Eigen::Matrix3d R_base = rot_sat.toRotationMatrix() * quat0.toRotationMatrix();
        Eigen::Vector3d approach_dir = R_base.col(2); // 3rd column: approach cylinder axis

        Eigen::Quaterniond quat_goal_base(R_base);
        quat_goal_base.normalize();

        // 1. Evaluate baseline (psi = 0 deg)
        EvalResult res_base = evaluateCandidate(
            local_robot_model, prodmp_template, q0, p0, quat0, p_goal, quat_goal_base, tau, dt, sim_params);

        // 2. Search over free roll psi in [-180, +180] deg (step 1 deg)
        double best_psi = 0.0;
        double min_viol = 1e9;
        EvalResult best_res;
        std::vector<double> feas_psis;

        for (int psi_int = -180; psi_int <= 180; psi_int += 1) {
            double psi_deg = static_cast<double>(psi_int);
            double psi_rad = psi_deg * M_PI / 180.0;

            // Rotate by psi around approach axis: R_goal = R_base * Rot_z(psi)
            Eigen::Matrix3d R_psi = R_base * Eigen::AngleAxisd(psi_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            Eigen::Quaterniond quat_goal_psi(R_psi);
            quat_goal_psi.normalize();

            EvalResult res_psi = evaluateCandidate(
                local_robot_model, prodmp_template, q0, p0, quat0, p_goal, quat_goal_psi, tau, dt, sim_params);

            if (res_psi.feasible) {
                feas_psis.push_back(psi_deg);
            }

            // Optimization metric: minimize max joint violation.
            // If feasible, prefer configuration where q6 is closest to center of range (107 deg = 1.8675 rad).
            if (res_psi.max_joint_violation_rad < min_viol) {
                min_viol = res_psi.max_joint_violation_rad;
                best_psi = psi_deg;
                best_res = res_psi;
            } else if (std::abs(res_psi.max_joint_violation_rad - min_viol) < 1e-6 && min_viol < 1e-4) {
                double q6_center = (JOINT_LIMITS[5].first + JOINT_LIMITS[5].second) / 2.0;
                if (std::abs(res_psi.q_final(5) - q6_center) < std::abs(best_res.q_final(5) - q6_center)) {
                    best_psi = psi_deg;
                    best_res = res_psi;
                }
            }
        }

        PhaseSummary ps;
        ps.phase_deg = phase_deg;
        ps.approach_dir = approach_dir;
        ps.res_base = res_base;
        ps.best_psi_deg = best_psi;
        ps.res_opt = best_res;
        ps.has_feasible_roll = !feas_psis.empty();
        ps.psi_min_feas = ps.has_feasible_roll ? feas_psis.front() : 0.0;
        ps.psi_max_feas = ps.has_feasible_roll ? feas_psis.back() : 0.0;

        summaries[i] = ps;
    }

    // Print progress in phase order
    for (const auto& ps : summaries) {
        std::string s_base = ps.res_base.feasible ? "OK" : "FAIL";
        std::string s_opt = ps.has_feasible_roll ? "OK" : "FAIL";

        std::cout << "θ=" << std::setw(3) << static_cast<int>(ps.phase_deg) << "° | "
                  << "AppDir=[" << std::setw(6) << ps.approach_dir.x() << "," << std::setw(6) << ps.approach_dir.y() << "," << std::setw(6) << ps.approach_dir.z() << "] | "
                  << "Base(ψ=0°): Viol=" << std::setw(6) << ps.res_base.max_joint_violation_rad << " q6=" << std::setw(6) << (ps.res_base.q_final(5)*180.0/M_PI) << "° [" << std::setw(4) << s_base << "] | "
                  << "Opt: ψ*=" << std::setw(4) << static_cast<int>(ps.best_psi_deg) << "° Viol=" << std::setw(6) << ps.res_opt.max_joint_violation_rad << " q6=" << std::setw(6) << (ps.res_opt.q_final(5)*180.0/M_PI) << "° [" << std::setw(4) << s_opt << "] | ";
        if (ps.has_feasible_roll) {
            std::cout << "Feas ψ span: [" << std::setw(4) << static_cast<int>(ps.psi_min_feas) << "°, " << std::setw(4) << static_cast<int>(ps.psi_max_feas) << "°] (width " << static_cast<int>(ps.psi_max_feas - ps.psi_min_feas) << "°)\n";
        } else {
            std::cout << "Feas ψ span: NONE (unresolvable by roll alone)\n";
        }
    }

    std::cout << "\n====================================================================================================================================\n";
    std::cout << "DETAILED CONFIGURATIONS AT OPTIMAL ROLL ψ* FOR EACH PHASE θ (AXIS = X)\n";
    std::cout << "====================================================================================================================================\n";
    std::cout << std::setw(6) << "θ" << " | "
              << std::setw(6) << "ψ*" << " | "
              << std::setw(7) << "q1(°)" << std::setw(7) << "q2(°)" << std::setw(7) << "q3(°)" << std::setw(7) << "q4(°)" << std::setw(7) << "q5(°)" << std::setw(7) << "q6(°)" << std::setw(7) << "q7(°)" << " | "
              << std::setw(8) << "w_trans" << " | "
              << std::setw(6) << "Feas?" << " | "
              << "Feasible ψ range\n";
    std::cout << "------------------------------------------------------------------------------------------------------------------------------------\n";

    for (const auto& ps : summaries) {
        std::string s_feas = ps.has_feasible_roll ? "OK" : "FAIL";
        std::cout << std::setw(5) << static_cast<int>(ps.phase_deg) << "° | "
                  << std::setw(5) << static_cast<int>(ps.best_psi_deg) << "° | "
                  << std::setw(6) << std::setprecision(1) << (ps.res_opt.q_final(0)*180.0/M_PI) << " "
                  << std::setw(6) << (ps.res_opt.q_final(1)*180.0/M_PI) << " "
                  << std::setw(6) << (ps.res_opt.q_final(2)*180.0/M_PI) << " "
                  << std::setw(6) << (ps.res_opt.q_final(3)*180.0/M_PI) << " "
                  << std::setw(6) << (ps.res_opt.q_final(4)*180.0/M_PI) << " "
                  << std::setw(6) << (ps.res_opt.q_final(5)*180.0/M_PI) << " "
                  << std::setw(6) << (ps.res_opt.q_final(6)*180.0/M_PI) << " | "
                  << std::setw(8) << std::setprecision(4) << ps.res_opt.w_trans_final << " | "
                  << std::setw(6) << s_feas << " | ";
        if (ps.has_feasible_roll) {
            std::cout << "[" << std::setw(4) << static_cast<int>(ps.psi_min_feas) << "°, " << std::setw(4) << static_cast<int>(ps.psi_max_feas) << "°] (width " << static_cast<int>(ps.psi_max_feas - ps.psi_min_feas) << "°)\n";
        } else {
            std::cout << "NONE\n";
        }
    }

    int n_total = summaries.size();
    int n_feas_base = 0;
    int n_feas_opt = 0;
    std::vector<double> psis_all_feas;
    std::vector<double> psis_infeas_yesterday;

    for (const auto& ps : summaries) {
        if (ps.res_base.feasible) n_feas_base++;
        if (ps.has_feasible_roll) {
            n_feas_opt++;
            psis_all_feas.push_back(ps.best_psi_deg);
            if (ps.phase_deg >= 195.0 && ps.phase_deg <= 300.0) {
                psis_infeas_yesterday.push_back(ps.best_psi_deg);
            }
        }
    }

    std::cout << "\n====================================================================================================================================\n";
    std::cout << "SUMMARY STATISTICS & ANSWERS TO QUESTIONS\n";
    std::cout << "====================================================================================================================================\n";
    std::cout << "1. Feasibility with Rigid Rotation (ψ=0°, satellite axis = X): " << n_feas_base << " / " << n_total << " (" << (n_feas_base*100.0/n_total) << "%)\n";
    std::cout << "2. Feasibility with FREE ROLL OPTIMIZATION (ψ*, satellite axis = X): " << n_feas_opt << " / " << n_total << " (" << (n_feas_opt*100.0/n_total) << "%)\n";
    std::cout << "3. Additional Phases Gained by 3rd DOF optimization: +" << (n_feas_opt - n_feas_base) << "\n";

    if (!psis_all_feas.empty()) {
        double min_all = *std::min_element(psis_all_feas.begin(), psis_all_feas.end());
        double max_all = *std::max_element(psis_all_feas.begin(), psis_all_feas.end());
        std::cout << "4. Total roll excursion across ALL 0-360° phases: [" << min_all << "°, " << max_all << "°], TOTAL EXCURSION = " << (max_all - min_all) << "°\n";
    }

    if (!psis_infeas_yesterday.empty()) {
        double min_y = *std::min_element(psis_infeas_yesterday.begin(), psis_infeas_yesterday.end());
        double max_y = *std::max_element(psis_infeas_yesterday.begin(), psis_infeas_yesterday.end());
        std::cout << "5. Roll excursion in previously failing range [195°, 300°]: [" << min_y << "°, " << max_y << "°], EXCURSION = " << (max_y - min_y) << "°\n";
    } else {
        std::cout << "5. Roll excursion in [195°, 300°]: NONE feasible in that range.\n";
    }

    return 0;
}
