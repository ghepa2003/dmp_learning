#include <iostream>
#include <iomanip>
#include <vector>
#include <memory>
#include <cmath>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <chrono>
#include <omp.h>

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

struct TrajectoryPositionData {
    std::vector<Eigen::Vector3d> positions;
    std::vector<double> s_values;
    double dt;
    int n_steps;
};

// Precompute position trajectory at dt = 0.05 once per phase
TrajectoryPositionData precomputePositionTrajectory(
    const ProDMP& prodmp_template,
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p_goal,
    double tau,
    double dt) {

    int n_steps = static_cast<int>(std::ceil(tau / dt));
    ProDMP prodmp = prodmp_template;
    prodmp.setRelativeGoal(false);
    prodmp.setInitialConditions(0.0, p0, Eigen::Vector3d::Zero());
    prodmp.setGoal(p_goal);

    TrajectoryPositionData data;
    data.dt = dt;
    data.n_steps = n_steps;
    data.positions.reserve(n_steps);
    data.s_values.reserve(n_steps);

    for (int i = 0; i < n_steps; ++i) {
        Eigen::Vector3d p = prodmp.step(dt);
        double s = std::min(1.0, static_cast<double>(i + 1) * dt / tau);
        data.positions.push_back(p);
        data.s_values.push_back(s);
    }

    return data;
}

struct PhaseSweepResult {
    double phase_deg = 0.0;
    double best_psi_deg = 0.0;
    double min_joint_viol_rad = 1e9;
    double q6_final_deg = 0.0;
    double q6_min_along_traj_deg = 1e9;
    double q6_max_along_traj_deg = -1e9;
    bool feasible = false;
    std::vector<double> feas_psi_range;
};

PhaseSweepResult runPhaseSweepDt005(
    const std::string& urdf_content,
    const std::vector<std::string>& joint_names,
    const TrajectoryPositionData& pos_data,
    const RobotModel::JointVector& q0,
    const Eigen::Quaterniond& quat0,
    const Eigen::Matrix3d& R_base,
    double phase_deg) {

    PhaseSweepResult res;
    res.phase_deg = phase_deg;

    double q6_center = 107.0;
    double best_q6_err = 1e9;

    NullspaceBiasedResolution::Params ns_params;
    ns_params.nullspace_gain = 0.2236;
    ns_params.joint1_nullspace_gain_scale = 10.0;

    JointPathSimulator::Params sim_params;
    sim_params.dt = pos_data.dt;
    sim_params.strategy = std::make_shared<NullspaceBiasedResolution>(ns_params);

    // Evaluate 361 candidates
    for (int psi_int = -180; psi_int <= 180; psi_int += 1) {
        double psi_deg = static_cast<double>(psi_int);
        double psi_rad = psi_deg * M_PI / 180.0;

        Eigen::Matrix3d R_psi = R_base * Eigen::AngleAxisd(psi_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Quaterniond quat_goal_psi(R_psi);
        quat_goal_psi.normalize();

        std::vector<CartesianSample> trajectory(pos_data.n_steps);
        for (int i = 0; i < pos_data.n_steps; ++i) {
            trajectory[i].position = pos_data.positions[i];
            trajectory[i].orientation = slerpShortestPath(quat0, quat_goal_psi, pos_data.s_values[i]);
        }

        auto robot_model = std::make_shared<RobotModel>(urdf_content, joint_names, "fer_hand_tcp");
        JointPathSimulator sim(robot_model, sim_params);
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
            res.feas_psi_range.push_back(psi_deg);
        }

        if (max_viol < res.min_joint_viol_rad - 1e-6) {
            res.min_joint_viol_rad = max_viol;
            best_q6_err = q6_err;
            res.best_psi_deg = psi_deg;
            res.q6_final_deg = q6_final;
            res.q6_min_along_traj_deg = q6_min;
            res.q6_max_along_traj_deg = q6_max;
        } else if (std::abs(max_viol - res.min_joint_viol_rad) < 1e-6) {
            if (q6_err < best_q6_err) {
                best_q6_err = q6_err;
                res.best_psi_deg = psi_deg;
                res.q6_final_deg = q6_final;
                res.q6_min_along_traj_deg = q6_min;
                res.q6_max_along_traj_deg = q6_max;
            }
        }
    }

    res.feasible = (res.min_joint_viol_rad < 1e-4);
    return res;
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

    auto robot_model_master = std::make_shared<RobotModel>(urdf_content, joint_names, "fer_hand_tcp");

    ProDMP prodmp_template = prodmp_io::loadProDmpFromYaml(weights_path);
    double tau = prodmp_template.tau();
    Eigen::Vector3d demo_disp = prodmp_template.demoDisplacement();

    RobotModel::JointVector q0;
    q0 << 0.0, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, 1.5707963267948966, 0.7853981633974483;

    robot_model_master->update(q0, RobotModel::JointVector::Zero());
    Eigen::Vector3d p0 = robot_model_master->eePosition();
    Eigen::Quaterniond quat0 = robot_model_master->eeOrientation();

    Eigen::Vector3d center(0.45, -0.05, 0.35);
    Eigen::Vector3d axis(1.0, 0.0, 0.0);
    double dt_fast = 0.05; // 50 ms step -> 1220 steps

    std::vector<double> test_phases = {90.0, 180.0, 210.0, 270.0, 330.0};
    std::vector<double> expected_full_rolls = {+24.0, 0.0, +59.0, +21.0, -38.0};

    std::cout << "========================================================================================================================\n";
    std::cout << "VERIFICA REALE: SWEEP AD ALTA EFFICIENZA CON dt = 0.05 s (N = " << static_cast<int>(std::ceil(tau / dt_fast)) << " PASSI)\n";
    std::cout << "========================================================================================================================\n\n";

    // Measure timing
    auto t_start_all = std::chrono::high_resolution_clock::now();

    std::vector<PhaseSweepResult> results(test_phases.size());

    #pragma omp parallel for schedule(dynamic)
    for (size_t idx = 0; idx < test_phases.size(); ++idx) {
        double phase_deg = test_phases[idx];
        double theta_rad = phase_deg * M_PI / 180.0;

        AnchoredGoal anchored = anchorAndRotate(p0, demo_disp, center, axis, theta_rad);
        Eigen::Vector3d p_goal = anchored.p_rotated_world;

        Eigen::AngleAxisd rot_sat(theta_rad, axis.normalized());
        Eigen::Matrix3d R_base = rot_sat.toRotationMatrix() * quat0.toRotationMatrix();

        TrajectoryPositionData pos_data = precomputePositionTrajectory(prodmp_template, p0, p_goal, tau, dt_fast);

        results[idx] = runPhaseSweepDt005(urdf_content, joint_names, pos_data, q0, quat0, R_base, phase_deg);
    }

    auto t_end_all = std::chrono::high_resolution_clock::now();
    double elapsed_total_ms = std::chrono::duration<double, std::milli>(t_end_all - t_start_all).count();

    std::cout << std::setw(6) << "theta" << " | "
              << std::setw(15) << "Roll (dt=0.005s)" << " | "
              << std::setw(15) << "Roll (dt=0.05s)" << " | "
              << std::setw(11) << "Delta Roll" << " | "
              << std::setw(10) << "Feasible?" << " | "
              << std::setw(15) << "q6 final (deg)" << " | "
              << std::setw(22) << "q6 range along path" << "\n";
    std::cout << "------------------------------------------------------------------------------------------------------------------------\n";

    bool all_match = true;
    for (size_t idx = 0; idx < test_phases.size(); ++idx) {
        double phase_deg = test_phases[idx];
        double exp_roll = expected_full_rolls[idx];
        const auto& r = results[idx];

        double diff = r.best_psi_deg - exp_roll;
        bool match = (std::abs(diff) <= 1.0);
        if (!match) all_match = false;

        std::string feas_str = r.feasible ? "OK" : "FAIL";

        std::cout << std::setw(5) << static_cast<int>(phase_deg) << "° | "
                  << std::setw(14) << static_cast<int>(exp_roll) << "° | "
                  << std::setw(14) << static_cast<int>(r.best_psi_deg) << "° | "
                  << std::setw(10) << static_cast<int>(diff) << "° | "
                  << std::setw(10) << feas_str << " | "
                  << std::fixed << std::setprecision(1) << std::setw(14) << r.q6_final_deg << "° | ["
                  << std::setw(5) << r.q6_min_along_traj_deg << "°, "
                  << std::setw(5) << r.q6_max_along_traj_deg << "°]\n";
    }

    std::cout << "\n========================================================================================================================\n";
    std::cout << "TEMPO REALE MISURATO (361 roll x 5 fasi = 1805 traiettorie x 1220 passi = 2.202.100 passi IK totali):\n";
    std::cout << "========================================================================================================================\n";
    std::cout << "  - Tempo totale multi-thread (OpenMP): " << std::fixed << std::setprecision(2) << elapsed_total_ms << " ms ("
              << (elapsed_total_ms / 1000.0) << " secondi)\n";
    std::cout << "  - Tempo per singola fase (361 roll):   " << (elapsed_total_ms / test_phases.size()) << " ms\n";
    std::cout << "  - Tempo per singolo candidato roll:    " << (elapsed_total_ms / (test_phases.size() * 361.0)) << " ms\n";
    std::cout << "  - Tutte e 5 le fasi coincidono?       " << (all_match ? "SI (CONVERGENZA ESATTA +-1°)" : "NO") << "\n";

    return 0;
}
