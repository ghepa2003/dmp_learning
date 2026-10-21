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

struct PhaseCollocationData {
    std::vector<Eigen::Vector3d> positions;
    double dt_colloc;
};

// Pre-generate Cartesian positions once per phase
PhaseCollocationData precomputeCartesianWaypoints(
    const ProDMP& prodmp_template,
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p_goal,
    double tau,
    int K) {

    double dt_colloc = tau / static_cast<double>(K);
    ProDMP prodmp = prodmp_template;
    prodmp.setRelativeGoal(false);
    prodmp.setInitialConditions(0.0, p0, Eigen::Vector3d::Zero());
    prodmp.setGoal(p_goal);

    double dt_fine = 0.005;
    int fine_steps_per_waypoint = static_cast<int>(std::round(dt_colloc / dt_fine));

    PhaseCollocationData data;
    data.dt_colloc = dt_colloc;
    data.positions.reserve(K);

    for (int k = 1; k <= K; ++k) {
        Eigen::Vector3d pos_k;
        for (int step = 0; step < fine_steps_per_waypoint; ++step) {
            pos_k = prodmp.step(dt_fine);
        }
        data.positions.push_back(pos_k);
    }
    return data;
}

struct SweepResult {
    double best_psi_deg = 0.0;
    double min_joint_viol_rad = 1e9;
    double q6_final_deg = 0.0;
    bool feasible = false;
    double min_q6_along_traj_deg = 1e9;
    double max_q6_along_traj_deg = -1e9;
    std::vector<double> feas_psi_range;
};

SweepResult runFastRollSweep(
    const std::shared_ptr<RobotModel>& robot_model,
    const PhaseCollocationData& colloc_data,
    const RobotModel::JointVector& q0,
    const Eigen::Quaterniond& quat0,
    const Eigen::Matrix3d& R_base,
    const JointPathSimulator::Params& base_sim_params,
    int K) {

    JointPathSimulator::Params sim_params = base_sim_params;
    sim_params.dt = colloc_data.dt_colloc;
    JointPathSimulator sim(robot_model, sim_params);

    SweepResult sr;
    double best_q6_err = 1e9;
    double q6_center = 107.0;

    std::vector<CartesianSample> trajectory(K);

    for (int psi_int = -180; psi_int <= 180; psi_int += 1) {
        double psi_deg = static_cast<double>(psi_int);
        double psi_rad = psi_deg * M_PI / 180.0;

        Eigen::Matrix3d R_psi = R_base * Eigen::AngleAxisd(psi_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Quaterniond quat_goal_psi(R_psi);
        quat_goal_psi.normalize();

        for (int k = 0; k < K; ++k) {
            double s = static_cast<double>(k + 1) / static_cast<double>(K);
            trajectory[k].position = colloc_data.positions[k];
            trajectory[k].orientation = slerpShortestPath(quat0, quat_goal_psi, s);
        }

        auto steps = sim.simulate(trajectory, q0);

        double max_viol = 0.0;
        double q6_min = 1e9;
        double q6_max = -1e9;

        for (const auto& step : steps) {
            double q6_d = step.q(5) * 180.0 / M_PI;
            if (q6_d < q6_min) q6_min = q6_d;
            if (q6_d > q6_max) q6_max = q6_d;

            for (int j = 0; j < 7; ++j) {
                double val = step.q(j);
                double min_l = JOINT_LIMITS[j].first;
                double max_l = JOINT_LIMITS[j].second;
                double v = (val < min_l) ? (min_l - val) : ((val > max_l) ? (val - max_l) : 0.0);
                if (v > max_viol) max_viol = v;
            }
        }

        double q6_final = steps.back().q(5) * 180.0 / M_PI;
        double q6_err = std::abs(q6_final - q6_center);

        if (max_viol < 1e-4) {
            sr.feas_psi_range.push_back(psi_deg);
        }

        if (max_viol < sr.min_joint_viol_rad - 1e-6) {
            sr.min_joint_viol_rad = max_viol;
            best_q6_err = q6_err;
            sr.best_psi_deg = psi_deg;
            sr.q6_final_deg = q6_final;
            sr.min_q6_along_traj_deg = q6_min;
            sr.max_q6_along_traj_deg = q6_max;
        } else if (std::abs(max_viol - sr.min_joint_viol_rad) < 1e-6) {
            if (q6_err < best_q6_err) {
                best_q6_err = q6_err;
                sr.best_psi_deg = psi_deg;
                sr.q6_final_deg = q6_final;
                sr.min_q6_along_traj_deg = q6_min;
                sr.max_q6_along_traj_deg = q6_max;
            }
        }
    }

    sr.feasible = (sr.min_joint_viol_rad < 1e-4);
    return sr;
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

    NullspaceBiasedResolution::Params ns_params;
    ns_params.nullspace_gain = 0.2236;
    ns_params.joint1_nullspace_gain_scale = 10.0;
    JointPathSimulator::Params sim_params;
    sim_params.strategy = std::make_shared<NullspaceBiasedResolution>(ns_params);

    std::vector<double> test_phases = {90.0, 180.0, 210.0, 270.0, 330.0};
    std::vector<double> expected_full_rolls = {+24.0, 0.0, +59.0, +21.0, -38.0};

    std::cout << "========================================================================================================================\n";
    std::cout << "VERIFICA REALE: COLLOCATION K=15 vs FULL-ROLLOUT (361 ROLL x 5 FASI)\n";
    std::cout << "========================================================================================================================\n\n";

    // 1. Measure real execution for K=15
    int K = 15;
    auto t_start_k15 = std::chrono::high_resolution_clock::now();

    std::vector<SweepResult> results_k15(test_phases.size());
    for (size_t idx = 0; idx < test_phases.size(); ++idx) {
        double phase_deg = test_phases[idx];
        double theta_rad = phase_deg * M_PI / 180.0;

        AnchoredGoal anchored = anchorAndRotate(p0, demo_disp, center, axis, theta_rad);
        Eigen::Vector3d p_goal = anchored.p_rotated_world;

        Eigen::AngleAxisd rot_sat(theta_rad, axis.normalized());
        Eigen::Matrix3d R_base = rot_sat.toRotationMatrix() * quat0.toRotationMatrix();

        // Precompute positions once
        PhaseCollocationData data = precomputeCartesianWaypoints(prodmp_template, p0, p_goal, tau, K);

        // Run sweep
        results_k15[idx] = runFastRollSweep(robot_model, data, q0, quat0, R_base, sim_params, K);
    }
    auto t_end_k15 = std::chrono::high_resolution_clock::now();
    double elapsed_k15_ms = std::chrono::duration<double, std::milli>(t_end_k15 - t_start_k15).count();

    // 2. Measure real execution for K=20
    int K20 = 20;
    auto t_start_k20 = std::chrono::high_resolution_clock::now();
    std::vector<SweepResult> results_k20(test_phases.size());
    for (size_t idx = 0; idx < test_phases.size(); ++idx) {
        double phase_deg = test_phases[idx];
        double theta_rad = phase_deg * M_PI / 180.0;

        AnchoredGoal anchored = anchorAndRotate(p0, demo_disp, center, axis, theta_rad);
        Eigen::Vector3d p_goal = anchored.p_rotated_world;

        Eigen::AngleAxisd rot_sat(theta_rad, axis.normalized());
        Eigen::Matrix3d R_base = rot_sat.toRotationMatrix() * quat0.toRotationMatrix();

        PhaseCollocationData data = precomputeCartesianWaypoints(prodmp_template, p0, p_goal, tau, K20);
        results_k20[idx] = runFastRollSweep(robot_model, data, q0, quat0, R_base, sim_params, K20);
    }
    auto t_end_k20 = std::chrono::high_resolution_clock::now();
    double elapsed_k20_ms = std::chrono::duration<double, std::milli>(t_end_k20 - t_start_k20).count();

    // Print comparison table
    std::cout << std::setw(6) << "theta" << " | "
              << std::setw(11) << "Roll Full" << " | "
              << std::setw(11) << "Roll K=15" << " | "
              << std::setw(11) << "Roll K=20" << " | "
              << std::setw(11) << "Diff K=15" << " | "
              << std::setw(10) << "Feas K=15" << " | "
              << std::setw(18) << "q6 range along path" << "\n";
    std::cout << "------------------------------------------------------------------------------------------------------------------------\n";

    for (size_t idx = 0; idx < test_phases.size(); ++idx) {
        double phase_deg = test_phases[idx];
        double exp_roll = expected_full_rolls[idx];
        const auto& r15 = results_k15[idx];
        const auto& r20 = results_k20[idx];

        double diff15 = r15.best_psi_deg - exp_roll;
        std::string feas_str = r15.feasible ? "OK" : "VIOLATION";

        std::cout << std::setw(5) << static_cast<int>(phase_deg) << "° | "
                  << std::setw(10) << static_cast<int>(exp_roll) << "° | "
                  << std::setw(10) << static_cast<int>(r15.best_psi_deg) << "° | "
                  << std::setw(10) << static_cast<int>(r20.best_psi_deg) << "° | "
                  << std::setw(10) << static_cast<int>(diff15) << "° | "
                  << std::setw(10) << feas_str << " | ["
                  << std::fixed << std::setprecision(1) << r15.min_q6_along_traj_deg << "°, "
                  << r15.max_q6_along_traj_deg << "°]\n";
    }

    std::cout << "\n========================================================================================================================\n";
    std::cout << "TEMPI REALI DI CALCOLO MISURATI (361 candidati roll x 5 fasi = 1805 valutazioni):\n";
    std::cout << "========================================================================================================================\n";
    std::cout << "  - Configurazione K = 15 waypoint: " << std::fixed << std::setprecision(2) << elapsed_k15_ms << " ms totali ("
              << (elapsed_k15_ms / test_phases.size()) << " ms/fase, "
              << (elapsed_k15_ms / (test_phases.size() * 361.0)) << " ms/candidato)\n";
    std::cout << "  - Configurazione K = 20 waypoint: " << std::fixed << std::setprecision(2) << elapsed_k20_ms << " ms totali ("
              << (elapsed_k20_ms / test_phases.size()) << " ms/fase, "
              << (elapsed_k20_ms / (test_phases.size() * 361.0)) << " ms/candidato)\n";

    return 0;
}
