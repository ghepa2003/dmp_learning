#include <iostream>
#include <iomanip>
#include <vector>
#include <memory>
#include <cmath>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <chrono>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "franka_cartesian_control/core/cartesian_error.hpp"
#include "franka_cartesian_control/core/velocity_ik_solver.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "haptic_dmp_learning/core/satellite_intercept.hpp"
#include "satellite_grasp_planner/core/joint_path_simulator.hpp"
#include "satellite_grasp_planner/core/manipulability.hpp"

using namespace satellite_grasp_planner::core;
using namespace haptic_dmp_learning::core;
using namespace haptic_dmp_learning::core::satellite_intercept;
using namespace franka_cartesian_control::core;
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

struct GoalIkResult {
    bool converged = false;
    bool feasible = false;
    double pos_err_mm = 0.0;
    double ori_err_deg = 0.0;
    double max_joint_violation_rad = 0.0;
    double max_q6_violation_rad = 0.0;
    RobotModel::JointVector q_solution;
    int iterations = 0;
};

// Iterative DLS IK on final goal pose alone (starting from q0)
GoalIkResult solveGoalOnlyIk(
    const std::shared_ptr<RobotModel>& robot_model,
    const RobotModel::JointVector& q0,
    const Eigen::Vector3d& p_goal,
    const Eigen::Quaterniond& quat_goal,
    const NullspaceBiasedResolution::Params& ns_params,
    int max_iters = 100,
    double tol_pos = 1e-4,     // 0.1 mm
    double tol_rot = 1e-3,     // ~0.05 deg
    double step_size = 0.5) {

    RobotModel::JointVector q = q0;
    VelocityIkSolver ik_solver(ns_params.ik_params);

    GoalIkResult res;
    for (int iter = 0; iter < max_iters; ++iter) {
        robot_model->update(q, RobotModel::JointVector::Zero());
        CartesianError err = computePoseError(
            robot_model->eePosition(), robot_model->eeOrientation(),
            p_goal, quat_goal);

        double pos_norm = err.linear.norm();
        double rot_norm = err.angular.norm();

        if (pos_norm < tol_pos && rot_norm < tol_rot) {
            res.converged = true;
            res.iterations = iter;
            break;
        }

        auto twist = ik_solver.desiredTwist(err);
        auto dq_primary = ik_solver.solve(robot_model->jacobian(), twist);

        const auto& J = robot_model->jacobian();
        Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
        JJt.diagonal().array() += ns_params.nullspace_damping * ns_params.nullspace_damping;
        Eigen::Matrix<double, 7, 6> J_pinv = J.transpose() * JJt.inverse();
        Eigen::Matrix<double, 7, 7> N = Eigen::Matrix<double, 7, 7>::Identity() - J_pinv * J;

        RobotModel::JointVector qe = q0 - q;
        qe(0) *= ns_params.joint1_nullspace_gain_scale;
        RobotModel::JointVector dq_ns = N * (ns_params.nullspace_gain * qe);

        RobotModel::JointVector dq_cmd = dq_primary + dq_ns;
        q += dq_cmd * step_size;
    }

    robot_model->update(q, RobotModel::JointVector::Zero());
    res.q_solution = q;
    res.pos_err_mm = (robot_model->eePosition() - p_goal).norm() * 1000.0;

    Eigen::Quaterniond q_diff = quat_goal.conjugate() * robot_model->eeOrientation();
    if (q_diff.w() < 0.0) q_diff.coeffs() = -q_diff.coeffs();
    double angle_rad = 2.0 * std::atan2(q_diff.vec().norm(), std::abs(q_diff.w()));
    res.ori_err_deg = angle_rad * 180.0 / M_PI;

    // Check joint limits
    double max_viol = 0.0;
    double max_q6_viol = 0.0;
    for (int j = 0; j < 7; ++j) {
        double val = q(j);
        double min_l = JOINT_LIMITS[j].first;
        double max_l = JOINT_LIMITS[j].second;
        if (val < min_l) {
            double v = min_l - val;
            if (v > max_viol) max_viol = v;
            if (j == 5 && v > max_q6_viol) max_q6_viol = v;
        } else if (val > max_l) {
            double v = val - max_l;
            if (v > max_viol) max_viol = v;
            if (j == 5 && v > max_q6_viol) max_q6_viol = v;
        }
    }
    res.max_joint_violation_rad = max_viol;
    res.max_q6_violation_rad = max_q6_viol;
    res.feasible = (res.converged || res.pos_err_mm < 1.0) && (res.max_joint_violation_rad < 1e-4);

    return res;
}

struct RolloutResult {
    bool feasible = false;
    double max_joint_violation_rad = 0.0;
    double max_q6_violation_rad = 0.0;
    double worst_violation_time_s = 0.0;
    int worst_violation_joint = -1;
    double pos_err_final_mm = 0.0;
    RobotModel::JointVector q_final;
    double q6_final_deg = 0.0;
    double q6_min_along_traj_deg = 1e9;
    double q6_max_along_traj_deg = -1e9;
};

RolloutResult simulateFullRollout(
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

    RolloutResult res;
    res.q_final = steps.back().q;
    res.q6_final_deg = res.q_final(5) * 180.0 / M_PI;

    double max_viol = 0.0;
    double max_q6_viol = 0.0;
    for (size_t i = 0; i < steps.size(); ++i) {
        double t = static_cast<double>(i) * dt;
        double q6_deg = steps[i].q(5) * 180.0 / M_PI;
        if (q6_deg < res.q6_min_along_traj_deg) res.q6_min_along_traj_deg = q6_deg;
        if (q6_deg > res.q6_max_along_traj_deg) res.q6_max_along_traj_deg = q6_deg;

        for (int j = 0; j < 7; ++j) {
            double val = steps[i].q(j);
            double min_l = JOINT_LIMITS[j].first;
            double max_l = JOINT_LIMITS[j].second;
            double v = 0.0;
            if (val < min_l) v = min_l - val;
            else if (val > max_l) v = val - max_l;

            if (v > max_viol) {
                max_viol = v;
                res.worst_violation_time_s = t;
                res.worst_violation_joint = j + 1;
            }
            if (j == 5 && v > max_q6_viol) max_q6_viol = v;
        }
    }

    res.max_joint_violation_rad = max_viol;
    res.max_q6_violation_rad = max_q6_viol;

    robot_model->update(res.q_final, RobotModel::JointVector::Zero());
    res.pos_err_final_mm = (robot_model->eePosition() - p_goal).norm() * 1000.0;
    res.feasible = (res.max_joint_violation_rad < 1e-4) && (res.pos_err_final_mm < 2.0);

    return res;
}

int main() {
    auto t_start = std::chrono::high_resolution_clock::now();

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

    std::vector<double> test_phases = {90.0, 180.0, 210.0, 270.0, 330.0};
    // Expected optimal rolls from full 25-phase rollout search:
    std::vector<double> expected_full_rolls = {+24.0, 0.0, +59.0, +21.0, -38.0};

    std::cout << "========================================================================================================================\n";
    std::cout << "VERIFICA RAPIDA: GOAL-ONLY IK SEARCH vs FULL-ROLLOUT SEARCH (5 FASI TEST)\n";
    std::cout << "========================================================================================================================\n\n" << std::flush;

    double q6_center_deg = 107.0;

    for (size_t idx = 0; idx < test_phases.size(); ++idx) {
        double phase_deg = test_phases[idx];
        double exp_roll = expected_full_rolls[idx];
        double theta_rad = phase_deg * M_PI / 180.0;

        AnchoredGoal anchored = anchorAndRotate(p0, demo_disp, center, axis, theta_rad);
        Eigen::Vector3d p_goal = anchored.p_rotated_world;

        Eigen::AngleAxisd rot_sat(theta_rad, axis.normalized());
        Eigen::Matrix3d R_base = rot_sat.toRotationMatrix() * quat0.toRotationMatrix();

        // 1. Goal-only IK search across psi in [-180, 180] deg (1 deg step)
        double best_psi_goal = 0.0;
        double min_q6_err = 1e9;
        GoalIkResult best_goal_res;
        std::vector<double> feas_goal_psis;

        for (int psi_int = -180; psi_int <= 180; psi_int += 1) {
            double psi_deg = static_cast<double>(psi_int);
            double psi_rad = psi_deg * M_PI / 180.0;

            Eigen::Matrix3d R_psi = R_base * Eigen::AngleAxisd(psi_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            Eigen::Quaterniond quat_goal_psi(R_psi);
            quat_goal_psi.normalize();

            GoalIkResult g_res = solveGoalOnlyIk(robot_model, q0, p_goal, quat_goal_psi, ns_params);

            if (g_res.feasible) {
                feas_goal_psis.push_back(psi_deg);
                double q6_deg = g_res.q_solution(5) * 180.0 / M_PI;
                double q6_err = std::abs(q6_deg - q6_center_deg);
                if (q6_err < min_q6_err) {
                    min_q6_err = q6_err;
                    best_psi_goal = psi_deg;
                    best_goal_res = g_res;
                }
            }
        }

        // 2. Full rollout verification using the roll chosen by Goal-Only IK
        Eigen::Matrix3d R_best_goal = R_base * Eigen::AngleAxisd(best_psi_goal * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Quaterniond quat_best_goal(R_best_goal);
        quat_best_goal.normalize();

        RolloutResult r_res_goal_choice = simulateFullRollout(
            robot_model, prodmp_template, q0, p0, quat0, p_goal, quat_best_goal, tau, dt, sim_params);

        // 3. Full rollout verification using the expected full rollout roll (for direct side-by-side comparison)
        Eigen::Matrix3d R_exp = R_base * Eigen::AngleAxisd(exp_roll * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Quaterniond quat_exp(R_exp);
        quat_exp.normalize();

        RolloutResult r_res_full_choice = simulateFullRollout(
            robot_model, prodmp_template, q0, p0, quat0, p_goal, quat_exp, tau, dt, sim_params);

        std::cout << ">>> FASE theta = " << std::setw(3) << static_cast<int>(phase_deg) << "° <<<\n";
        std::cout << "  - Roll Goal-Only IK:       psi* = " << std::setw(4) << static_cast<int>(best_psi_goal) << "° (Feasible range: ["
                  << (feas_goal_psis.empty() ? 0 : feas_goal_psis.front()) << "°, " << (feas_goal_psis.empty() ? 0 : feas_goal_psis.back()) << "°])\n";
        std::cout << "  - Roll Full-Rollout Sweep: psi* = " << std::setw(4) << static_cast<int>(exp_roll) << "°\n";
        std::cout << "  - Differenza Roll:         " << (best_psi_goal - exp_roll) << "°\n";
        
        std::cout << "  - VERIFICA ROLLOUT INTERO CON ROLL GOAL-ONLY (psi = " << static_cast<int>(best_psi_goal) << "°):\n";
        std::cout << "      Feasible lungo tutto il percorso? " << (r_res_goal_choice.feasible ? "SI (OK)" : "NO (VIOLAZIONE)") << "\n";
        std::cout << "      Violazione max giunti:           " << r_res_goal_choice.max_joint_violation_rad << " rad\n";
        std::cout << "      Range q6 durante la traiettoria: [" << std::fixed << std::setprecision(1) << r_res_goal_choice.q6_min_along_traj_deg << "°, " << r_res_goal_choice.q6_max_along_traj_deg << "°]\n";
        std::cout << "      q6 finale:                       " << r_res_goal_choice.q6_final_deg << "°\n";
        if (!r_res_goal_choice.feasible) {
            std::cout << "      --> Violazione al tempo t = " << r_res_goal_choice.worst_violation_time_s << " s sul giunto q" << r_res_goal_choice.worst_violation_joint << "\n";
        }
        std::cout << "------------------------------------------------------------------------------------------------------------------------\n" << std::flush;
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    double elapsed_s = std::chrono::duration<double>(t_end - t_start).count();
    std::cout << "\nTEMPO TOTALE DI ESECUZIONE: " << std::setprecision(2) << elapsed_s << " secondi.\n";

    return 0;
}
