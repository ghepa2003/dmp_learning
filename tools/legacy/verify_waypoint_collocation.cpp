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

struct TrajectoryProfile {
    std::vector<double> s_values;
    std::vector<double> time_s;
    std::vector<double> q6_deg;
    double s_q6_min = 0.0;
    double t_q6_min_s = 0.0;
    double q6_min_deg = 1e9;
    double s_q6_max = 0.0;
    double t_q6_max_s = 0.0;
    double q6_max_deg = -1e9;
    double max_joint_viol_rad = 0.0;
};

TrajectoryProfile computeFullProfile(
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

    TrajectoryProfile prof;
    prof.s_values.reserve(steps.size());
    prof.time_s.reserve(steps.size());
    prof.q6_deg.reserve(steps.size());

    for (size_t i = 0; i < steps.size(); ++i) {
        double t = static_cast<double>(i) * dt;
        double s = t / tau;
        double q6 = steps[i].q(5) * 180.0 / M_PI;

        prof.s_values.push_back(s);
        prof.time_s.push_back(t);
        prof.q6_deg.push_back(q6);

        if (q6 < prof.q6_min_deg) {
            prof.q6_min_deg = q6;
            prof.s_q6_min = s;
            prof.t_q6_min_s = t;
        }
        if (q6 > prof.q6_max_deg) {
            prof.q6_max_deg = q6;
            prof.s_q6_max = s;
            prof.t_q6_max_s = t;
        }

        for (int j = 0; j < 7; ++j) {
            double val = steps[i].q(j);
            double min_l = JOINT_LIMITS[j].first;
            double max_l = JOINT_LIMITS[j].second;
            double v = (val < min_l) ? (min_l - val) : ((val > max_l) ? (val - max_l) : 0.0);
            if (v > prof.max_joint_viol_rad) prof.max_joint_viol_rad = v;
        }
    }

    return prof;
}

struct CollocationResult {
    bool feasible = false;
    double max_joint_violation_rad = 0.0;
    double q6_final_deg = 0.0;
    RobotModel::JointVector q_final;
};

CollocationResult simulateKWaypoints(
    const std::shared_ptr<RobotModel>& robot_model,
    const ProDMP& prodmp_template,
    const RobotModel::JointVector& q0,
    const Eigen::Vector3d& p0,
    const Eigen::Quaterniond& quat0,
    const Eigen::Vector3d& p_goal,
    const Eigen::Quaterniond& quat_goal,
    double tau,
    int K,
    const JointPathSimulator::Params& sim_params) {

    double dt_colloc = tau / static_cast<double>(K);
    ProDMP prodmp = prodmp_template;
    prodmp.setRelativeGoal(false);
    prodmp.setInitialConditions(0.0, p0, Eigen::Vector3d::Zero());
    prodmp.setGoal(p_goal);

    std::vector<CartesianSample> trajectory;
    trajectory.reserve(K);

    double dt_fine = 0.005;
    int fine_steps_per_waypoint = static_cast<int>(std::round(dt_colloc / dt_fine));

    for (int k = 1; k <= K; ++k) {
        Eigen::Vector3d pos_k;
        for (int step = 0; step < fine_steps_per_waypoint; ++step) {
            pos_k = prodmp.step(dt_fine);
        }
        double s = static_cast<double>(k) / static_cast<double>(K);
        CartesianSample sample;
        sample.position = pos_k;
        sample.orientation = slerpShortestPath(quat0, quat_goal, s);
        trajectory.push_back(sample);
    }

    JointPathSimulator::Params colloc_params = sim_params;
    colloc_params.dt = dt_colloc;
    JointPathSimulator sim(robot_model, colloc_params);
    auto steps = sim.simulate(trajectory, q0);

    CollocationResult res;
    res.q_final = steps.back().q;
    res.q6_final_deg = res.q_final(5) * 180.0 / M_PI;

    double max_viol = 0.0;
    for (const auto& step : steps) {
        for (int j = 0; j < 7; ++j) {
            double val = step.q(j);
            double min_l = JOINT_LIMITS[j].first;
            double max_l = JOINT_LIMITS[j].second;
            double v = (val < min_l) ? (min_l - val) : ((val > max_l) ? (val - max_l) : 0.0);
            if (v > max_viol) max_viol = v;
        }
    }
    res.max_joint_violation_rad = max_viol;
    res.feasible = (res.max_joint_violation_rad < 1e-4);

    return res;
}

double findOptimalRollCollocation(
    const std::shared_ptr<RobotModel>& robot_model,
    const ProDMP& prodmp_template,
    const RobotModel::JointVector& q0,
    const Eigen::Vector3d& p0,
    const Eigen::Quaterniond& quat0,
    const Eigen::Vector3d& p_goal,
    const Eigen::Matrix3d& R_base,
    double tau,
    int K,
    const JointPathSimulator::Params& sim_params) {

    double best_psi = 0.0;
    double min_viol = 1e9;
    double best_q6_err = 1e9;
    double q6_center = 107.0;

    for (int psi_int = -180; psi_int <= 180; psi_int += 1) {
        double psi_deg = static_cast<double>(psi_int);
        double psi_rad = psi_deg * M_PI / 180.0;

        Eigen::Matrix3d R_psi = R_base * Eigen::AngleAxisd(psi_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Quaterniond quat_goal_psi(R_psi);
        quat_goal_psi.normalize();

        CollocationResult res = simulateKWaypoints(
            robot_model, prodmp_template, q0, p0, quat0, p_goal, quat_goal_psi, tau, K, sim_params);

        double q6_err = std::abs(res.q6_final_deg - q6_center);

        if (res.max_joint_violation_rad < min_viol - 1e-6) {
            min_viol = res.max_joint_violation_rad;
            best_q6_err = q6_err;
            best_psi = psi_deg;
        } else if (std::abs(res.max_joint_violation_rad - min_viol) < 1e-6) {
            if (q6_err < best_q6_err) {
                best_q6_err = q6_err;
                best_psi = psi_deg;
            }
        }
    }

    return best_psi;
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

    std::vector<double> test_phases = {90.0, 180.0, 210.0, 270.0, 330.0};
    std::vector<double> expected_full_rolls = {+24.0, 0.0, +59.0, +21.0, -38.0};

    std::cout << "========================================================================================================\n";
    std::cout << "PARTE 1: ANALISI DETTAGLIATA PUNTO CRITICO DI q6 LUNGO IL ROLLOUT COMPLETO\n";
    std::cout << "========================================================================================================\n";

    std::vector<double> k5_grid = {0.2, 0.4, 0.6, 0.8, 1.0};

    for (size_t idx = 0; idx < test_phases.size(); ++idx) {
        double phase_deg = test_phases[idx];
        double exp_roll = expected_full_rolls[idx];
        double theta_rad = phase_deg * M_PI / 180.0;

        AnchoredGoal anchored = anchorAndRotate(p0, demo_disp, center, axis, theta_rad);
        Eigen::Vector3d p_goal = anchored.p_rotated_world;

        Eigen::AngleAxisd rot_sat(theta_rad, axis.normalized());
        Eigen::Matrix3d R_base = rot_sat.toRotationMatrix() * quat0.toRotationMatrix();
        Eigen::Matrix3d R_exp = R_base * Eigen::AngleAxisd(exp_roll * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Quaterniond quat_exp(R_exp);
        quat_exp.normalize();

        TrajectoryProfile prof = computeFullProfile(
            robot_model, prodmp_template, q0, p0, quat0, p_goal, quat_exp, tau, dt, sim_params);

        // Find closest K=5 waypoint to s_q6_min
        double min_dist_s = 1e9;
        double closest_wp_s = 0.0;
        for (double wp_s : k5_grid) {
            double d = std::abs(wp_s - prof.s_q6_min);
            if (d < min_dist_s) {
                min_dist_s = d;
                closest_wp_s = wp_s;
            }
        }

        std::cout << "Fase theta = " << std::setw(3) << static_cast<int>(phase_deg) << "° (roll = " << std::setw(3) << static_cast<int>(exp_roll) << "°):\n";
        std::cout << "  - q6 Minimo: " << std::fixed << std::setprecision(1) << prof.q6_min_deg << "° all'istante t = " << std::setprecision(2) << prof.t_q6_min_s << " s (s = " << std::setprecision(3) << prof.s_q6_min << ")\n";
        std::cout << "  - Distanza dal waypoint K=5 piu' vicino (s = " << closest_wp_s << "): delta_s = " << min_dist_s << " (dt = " << (min_dist_s * tau) << " s)\n";
        std::cout << "  - q6 Massimo: " << prof.q6_max_deg << "° all'istante t = " << prof.t_q6_max_s << " s (s = " << prof.s_q6_max << ")\n";
        std::cout << "--------------------------------------------------------------------------------------------------------\n";
    }

    std::cout << "\n========================================================================================================\n";
    std::cout << "PARTE 2 & 3: CONFRONTO RICERCA ROLL OTTIMALE AL VARIARE DI K (K=5, K=10, K=20, K=50)\n";
    std::cout << "========================================================================================================\n";

    std::vector<int> K_values = {5, 10, 20, 50};

    std::cout << std::setw(7) << "theta" << " | "
              << std::setw(9) << "Roll_Full" << " | ";
    for (int K : K_values) {
        std::cout << "Roll(K=" << std::setw(2) << K << ") | ";
    }
    std::cout << "K_min_convergenza\n";
    std::cout << "--------------------------------------------------------------------------------------------------------\n";

    for (size_t idx = 0; idx < test_phases.size(); ++idx) {
        double phase_deg = test_phases[idx];
        double exp_roll = expected_full_rolls[idx];
        double theta_rad = phase_deg * M_PI / 180.0;

        AnchoredGoal anchored = anchorAndRotate(p0, demo_disp, center, axis, theta_rad);
        Eigen::Vector3d p_goal = anchored.p_rotated_world;

        Eigen::AngleAxisd rot_sat(theta_rad, axis.normalized());
        Eigen::Matrix3d R_base = rot_sat.toRotationMatrix() * quat0.toRotationMatrix();

        std::cout << std::setw(5) << static_cast<int>(phase_deg) << "° | "
                  << std::setw(7) << static_cast<int>(exp_roll) << "° | ";

        int k_conv = -1;
        for (int K : K_values) {
            double roll_k = findOptimalRollCollocation(
                robot_model, prodmp_template, q0, p0, quat0, p_goal, R_base, tau, K, sim_params);
            std::cout << std::setw(8) << static_cast<int>(roll_k) << "° | ";
            if (std::abs(roll_k - exp_roll) <= 1.0 && k_conv == -1) {
                k_conv = K;
            }
        }
        if (k_conv != -1) {
            std::cout << "K = " << k_conv << "\n";
        } else {
            std::cout << "K > 50\n";
        }
    }

    std::cout << "\n========================================================================================================\n";
    std::cout << "PARTE 4: MISURAZIONE TEMPO REALE DI ESECUZIONE (361 roll x 5 fasi)\n";
    std::cout << "========================================================================================================\n";

    for (int K : K_values) {
        auto t_start = std::chrono::high_resolution_clock::now();
        for (size_t idx = 0; idx < test_phases.size(); ++idx) {
            double phase_deg = test_phases[idx];
            double theta_rad = phase_deg * M_PI / 180.0;

            AnchoredGoal anchored = anchorAndRotate(p0, demo_disp, center, axis, theta_rad);
            Eigen::Vector3d p_goal = anchored.p_rotated_world;

            Eigen::AngleAxisd rot_sat(theta_rad, axis.normalized());
            Eigen::Matrix3d R_base = rot_sat.toRotationMatrix() * quat0.toRotationMatrix();

            findOptimalRollCollocation(
                robot_model, prodmp_template, q0, p0, quat0, p_goal, R_base, tau, K, sim_params);
        }
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        double ms_per_phase = elapsed_ms / test_phases.size();
        std::cout << "K = " << std::setw(2) << K << ": Tempo Totale (5 fasi) = " << std::setw(6) << std::setprecision(1) << elapsed_ms << " ms  -->  "
                  << std::setprecision(2) << ms_per_phase << " ms per fase (" << (ms_per_phase / 361.0) << " ms/candidato)\n";
    }

    return 0;
}
