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
    double q6_final_deg = 0.0;
    double q6_min_deg = 0.0;
    double q6_max_deg = 0.0;
    RobotModel::JointVector q_final;
};

struct PhaseSummary {
    double phase_deg = 0.0;
    Eigen::Vector3d r_hat;
    Eigen::Vector3d t_hat;
    EvalResult res_base;
    double best_psi_deg = 0.0;
    EvalResult res_opt;
    double psi_min_feas = 0.0;
    double psi_max_feas = 0.0;
    bool has_feasible_roll = false;
    int num_feasible_rolls = 0;
};

// Yesterday's results for comparison
struct YesterdayResult {
    double phase_deg;
    double roll_deg;
    bool feasible;
    double w_trans;
    double q6_deg;
};

const std::vector<YesterdayResult> YESTERDAY_RESULTS = {
    {0.0,   0.0,  true, 0.1068, 107.0},
    {15.0,  3.0,  true, 0.1066, 107.0},
    {30.0,  7.0,  true, 0.1061, 107.0},
    {45.0, 11.0,  true, 0.1051, 107.0},
    {60.0, 15.0,  true, 0.1037, 107.0},
    {75.0, 19.0,  true, 0.1017, 107.0},
    {90.0, 24.0,  true, 0.0988, 107.0},
    {105.0, 28.0, true, 0.0950, 107.0},
    {120.0, 32.0, true, 0.0901, 107.0},
    {135.0, 34.0, true, 0.0841, 107.0},
    {150.0, 32.0, true, 0.0772, 107.0},
    {165.0, 22.0, true, 0.0700, 107.0},
    {180.0,  0.0, true, 0.0634, 107.0},
    {195.0, 48.0, true, 0.0593,  94.6},
    {210.0, 59.0, true, 0.0573,  76.5},
    {225.0, 61.0, true, 0.0576,  64.4},
    {240.0, 56.0, true, 0.0599,  59.4},
    {255.0, 43.0, true, 0.0641,  63.7},
    {270.0, 21.0, true, 0.0700,  78.9},
    {285.0,-14.0, true, 0.0772, 104.9},
    {300.0,-41.0, true, 0.0847, 107.0},
    {315.0,-45.0, true, 0.0917, 107.0},
    {330.0,-38.0, true, 0.0975, 107.0},
    {345.0,-22.0, true, 0.1019, 107.0},
    {360.0,  0.0, true, 0.1068, 107.0}
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

    ProDMP prodmp_template = prodmp_io::loadProDmpFromYaml(weights_path);
    double tau = prodmp_template.tau();
    Eigen::Vector3d demo_disp = prodmp_template.demoDisplacement();

    RobotModel::JointVector q0;
    q0 << 0.0, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, 1.5707963267948966, 0.7853981633974483;

    robot_model_master->update(q0, RobotModel::JointVector::Zero());
    Eigen::Vector3d p0 = robot_model_master->eePosition();
    Eigen::Quaterniond quat0 = robot_model_master->eeOrientation();

    // Satellite geometry
    Eigen::Vector3d center(0.45, -0.05, 0.35);
    Eigen::Vector3d axis(1.0, 0.0, 0.0); // X-axis
    Eigen::Vector3d p_grasp0 = p0 + demo_disp; // [0.3928, -0.1150, 0.2988]

    double dt = 0.005;
    int n_steps = static_cast<int>(std::ceil(tau / dt));

    std::vector<double> phases;
    for (int p = 0; p <= 360; p += 15) phases.push_back(static_cast<double>(p));

    auto t_start = std::chrono::high_resolution_clock::now();
    std::vector<PhaseSummary> summaries(phases.size());

    NullspaceBiasedResolution::Params ns_params;
    ns_params.nullspace_gain = 0.2236;
    ns_params.joint1_nullspace_gain_scale = 10.0;
    JointPathSimulator::Params sim_params;
    sim_params.dt = dt;
    sim_params.strategy = std::make_shared<NullspaceBiasedResolution>(ns_params);

    #pragma omp parallel for schedule(dynamic)
    for (size_t i = 0; i < phases.size(); ++i) {
        double phase_deg = phases[i];
        double theta_rad = phase_deg * M_PI / 180.0;

        AnchoredGoal anchored = anchorAndRotate(p0, demo_disp, center, axis, theta_rad);
        Eigen::Vector3d p_goal = anchored.p_rotated_world;

        // Corrected Radial and Tangent directions at phase theta
        Eigen::AngleAxisd rot_sat(theta_rad, axis.normalized());
        Eigen::Vector3d r_vec = rot_sat * (p_grasp0 - center);
        Eigen::Vector3d r_hat = r_vec.normalized(); // Z_ee (Approach)
        
        // Tangent vector = axis x r_hat
        Eigen::Vector3d t_vec = axis.normalized().cross(r_hat);
        Eigen::Vector3d t_hat = t_vec.normalized(); // Y_ee (Handle/Cylinder axis)
        
        // X_ee = Y_ee x Z_ee
        Eigen::Vector3d x_hat = t_hat.cross(r_hat).normalized();

        // Base gripper rotation matrix: R_base = [x_hat, t_hat, r_hat]
        Eigen::Matrix3d R_base;
        R_base.col(0) = x_hat;
        R_base.col(1) = t_hat;
        R_base.col(2) = r_hat;

        // 1. Precompute position trajectory once per phase
        ProDMP prodmp = prodmp_template;
        prodmp.setRelativeGoal(false);
        prodmp.setInitialConditions(0.0, p0, Eigen::Vector3d::Zero());
        prodmp.setGoal(p_goal);

        std::vector<Eigen::Vector3d> positions(n_steps);
        std::vector<double> s_values(n_steps);
        for (int step_i = 0; step_i < n_steps; ++step_i) {
            positions[step_i] = prodmp.step(dt);
            s_values[step_i] = std::min(1.0, static_cast<double>(step_i + 1) * dt / tau);
        }

        // Thread-local RobotModel & Simulator
        auto local_robot_model = std::make_shared<RobotModel>(urdf_content, joint_names, "fer_hand_tcp");
        JointPathSimulator sim(local_robot_model, sim_params);

        std::vector<CartesianSample> trajectory(n_steps);

        // Helper lambda to simulate a candidate
        auto simulateCandidate = [&](const Eigen::Matrix3d& R_target) -> EvalResult {
            Eigen::Quaterniond quat_goal(R_target);
            quat_goal.normalize();

            for (int k = 0; k < n_steps; ++k) {
                trajectory[k].position = positions[k];
                trajectory[k].orientation = slerpShortestPath(quat0, quat_goal, s_values[k]);
            }

            auto steps = sim.simulate(trajectory, q0);
            EvalResult res;
            res.q_final = steps.back().q;
            res.w_trans_final = steps.back().w_trans;
            res.q6_final_deg = steps.back().q(5) * 180.0 / M_PI;

            double max_v = 0.0;
            double max_q6_v = 0.0;
            double q6_min = 1e9, q6_max = -1e9;

            for (const auto& st : steps) {
                double q6_d = st.q(5) * 180.0 / M_PI;
                if (q6_d < q6_min) q6_min = q6_d;
                if (q6_d > q6_max) q6_max = q6_d;

                for (int j = 0; j < 7; ++j) {
                    double val = st.q(j);
                    double min_l = JOINT_LIMITS[j].first;
                    double max_l = JOINT_LIMITS[j].second;
                    if (val < min_l) {
                        double v = min_l - val;
                        if (v > max_v) max_v = v;
                        if (j == 5 && v > max_q6_v) max_q6_v = v;
                    } else if (val > max_l) {
                        double v = val - max_l;
                        if (v > max_v) max_v = v;
                        if (j == 5 && v > max_q6_v) max_q6_v = v;
                    }
                }
            }
            res.max_joint_violation_rad = max_v;
            res.max_q6_violation_rad = max_q6_v;
            res.q6_min_deg = q6_min;
            res.q6_max_deg = q6_max;

            local_robot_model->update(res.q_final, RobotModel::JointVector::Zero());
            res.pos_err_final_mm = (local_robot_model->eePosition() - p_goal).norm() * 1000.0;

            Eigen::Quaterniond q_diff = quat_goal.conjugate() * local_robot_model->eeOrientation();
            if (q_diff.w() < 0.0) q_diff.coeffs() = -q_diff.coeffs();
            res.ori_err_final_deg = 2.0 * std::atan2(q_diff.vec().norm(), std::abs(q_diff.w())) * 180.0 / M_PI;

            res.feasible = (res.max_joint_violation_rad < 1e-4) && (res.pos_err_final_mm < 2.0);
            return res;
        };

        // 1. Evaluate baseline (psi = 0 deg)
        EvalResult res_base = simulateCandidate(R_base);

        // 2. Scan over psi in [-180, +180] deg around radial axis r_hat (Z_ee)
        double best_psi = 0.0;
        double min_viol = 1e9;
        double best_q6_err = 1e9;
        double q6_center = 107.0;
        EvalResult best_res;
        std::vector<double> feas_psis;

        for (int psi_int = -180; psi_int <= 180; psi_int += 1) {
            double psi_deg = static_cast<double>(psi_int);
            double psi_rad = psi_deg * M_PI / 180.0;

            // Rotate around Z_ee (r_hat) by psi: R_psi = R_base * Rot_z(psi)
            Eigen::Matrix3d R_psi = R_base * Eigen::AngleAxisd(psi_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();

            EvalResult res_psi = simulateCandidate(R_psi);

            if (res_psi.feasible) {
                feas_psis.push_back(psi_deg);
            }

            double q6_err = std::abs(res_psi.q6_final_deg - q6_center);

            if (res_psi.max_joint_violation_rad < min_viol - 1e-6) {
                min_viol = res_psi.max_joint_violation_rad;
                best_q6_err = q6_err;
                best_psi = psi_deg;
                best_res = res_psi;
            } else if (std::abs(res_psi.max_joint_violation_rad - min_viol) < 1e-6) {
                if (q6_err < best_q6_err) {
                    best_q6_err = q6_err;
                    best_psi = psi_deg;
                    best_res = res_psi;
                }
            }
        }

        PhaseSummary ps;
        ps.phase_deg = phase_deg;
        ps.r_hat = r_hat;
        ps.t_hat = t_hat;
        ps.res_base = res_base;
        ps.best_psi_deg = best_psi;
        ps.res_opt = best_res;
        ps.has_feasible_roll = !feas_psis.empty();
        ps.num_feasible_rolls = static_cast<int>(feas_psis.size());
        ps.psi_min_feas = ps.has_feasible_roll ? feas_psis.front() : 0.0;
        ps.psi_max_feas = ps.has_feasible_roll ? feas_psis.back() : 0.0;

        summaries[i] = ps;
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    double total_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "====================================================================================================================================\n";
    std::cout << "TASK 2: RICALCOLO FEASIBILITY SWEEP CON GEOMETRIA CORRETTA (RADIAL APPROACH, FREE ROLL ABOUT RADIAL AXIS)\n";
    std::cout << "Full Rollout dt = 0.005 s (N = 12195 passi), 25 fasi x 361 roll = 9025 rollout completi\n";
    std::cout << "Tempo totale calcolo: " << (total_ms / 1000.0) << " s (" << (total_ms / (1000.0 * 60.0)) << " min)\n";
    std::cout << "====================================================================================================================================\n";

    std::cout << std::setw(6) << "θ" << " | "
              << std::setw(30) << "Radial AppDir r_hat(θ)" << " | "
              << std::setw(6) << "ψ*" << " | "
              << std::setw(6) << "Feas?" << " | "
              << std::setw(8) << "w_trans" << " | "
              << std::setw(7) << "q6 fin" << " | "
              << std::setw(16) << "q6 range" << " | "
              << "Feasible ψ range (span)\n";
    std::cout << "------------------------------------------------------------------------------------------------------------------------------------\n";

    int n_feas_base = 0;
    int n_feas_opt = 0;

    for (const auto& ps : summaries) {
        if (ps.res_base.feasible) n_feas_base++;
        if (ps.has_feasible_roll) n_feas_opt++;

        std::string s_feas = ps.has_feasible_roll ? "OK" : "FAIL";
        std::ostringstream app_ss;
        app_ss << "[" << std::setw(6) << std::setprecision(3) << ps.r_hat.x() << ", "
               << std::setw(6) << ps.r_hat.y() << ", " << std::setw(6) << ps.r_hat.z() << "]";

        std::ostringstream q6_range_ss;
        q6_range_ss << "[" << std::setw(5) << std::setprecision(1) << ps.res_opt.q6_min_deg << "°, "
                    << std::setw(5) << ps.res_opt.q6_max_deg << "°]";

        std::cout << std::setw(5) << static_cast<int>(ps.phase_deg) << "° | "
                  << std::setw(30) << app_ss.str() << " | "
                  << std::setw(5) << static_cast<int>(ps.best_psi_deg) << "° | "
                  << std::setw(6) << s_feas << " | "
                  << std::setw(8) << std::setprecision(4) << ps.res_opt.w_trans_final << " | "
                  << std::setw(6) << std::setprecision(1) << ps.res_opt.q6_final_deg << "° | "
                  << std::setw(16) << q6_range_ss.str() << " | ";
        if (ps.has_feasible_roll) {
            std::cout << "[" << std::setw(4) << static_cast<int>(ps.psi_min_feas) << "°, "
                      << std::setw(4) << static_cast<int>(ps.psi_max_feas) << "°] (width "
                      << static_cast<int>(ps.psi_max_feas - ps.psi_min_feas) << "°, "
                      << ps.num_feasible_rolls << " roll)\n";
        } else {
            std::cout << "NONE\n";
        }
    }

    std::cout << "\n====================================================================================================================================\n";
    std::cout << "TASK 3: CONFRONTO SIDE-BY-SIDE TRA IERI (GEOMETRIA VECCHIA) E OGGI (GEOMETRIA CORRETTA)\n";
    std::cout << "====================================================================================================================================\n";
    std::cout << std::setw(6) << "θ" << " | "
              << std::setw(16) << "Ieri: Roll (ψ*)" << " | "
              << std::setw(10) << "Ieri Feas" << " | "
              << std::setw(12) << "Ieri w_trans" << " | "
              << std::setw(16) << "Oggi: Roll (ψ*)" << " | "
              << std::setw(10) << "Oggi Feas" << " | "
              << std::setw(12) << "Oggi w_trans" << " | "
              << std::setw(11) << "Delta Roll" << " | "
              << "Commento\n";
    std::cout << "------------------------------------------------------------------------------------------------------------------------------------\n";

    for (size_t i = 0; i < phases.size(); ++i) {
        const auto& y = YESTERDAY_RESULTS[i];
        const auto& t = summaries[i];

        double d_roll = t.best_psi_deg - y.roll_deg;
        std::string y_feas = y.feasible ? "OK" : "FAIL";
        std::string t_feas = t.has_feasible_roll ? "OK" : "FAIL";

        std::string comment = "";
        if (y.feasible && t.has_feasible_roll) {
            if (std::abs(d_roll) < 5.0) comment = "Roll analogo";
            else comment = "Roll ricalibrato";
        } else if (!y.feasible && t.has_feasible_roll) {
            comment = "NUOVO FEASIBLE!";
        } else if (y.feasible && !t.has_feasible_roll) {
            comment = "INVENTATO IERI (FALSO POSITIVO)!";
        }

        std::cout << std::setw(5) << static_cast<int>(y.phase_deg) << "° | "
                  << std::setw(15) << static_cast<int>(y.roll_deg) << "° | "
                  << std::setw(10) << y_feas << " | "
                  << std::setw(12) << std::setprecision(4) << y.w_trans << " | "
                  << std::setw(15) << static_cast<int>(t.best_psi_deg) << "° | "
                  << std::setw(10) << t_feas << " | "
                  << std::setw(12) << std::setprecision(4) << t.res_opt.w_trans_final << " | "
                  << std::setw(10) << static_cast<int>(d_roll) << "° | "
                  << comment << "\n";
    }

    std::cout << "\n====================================================================================================================================\n";
    std::cout << "STATISTICHE DI SINTESI TASK 3:\n";
    std::cout << "1. Fattibilità Geometria Corretta: " << n_feas_opt << " / " << summaries.size()
              << " (" << (n_feas_opt * 100.0 / summaries.size()) << "%)\n";
    std::cout << "2. Baseline senza roll libero (ψ=0°): " << n_feas_base << " / " << summaries.size()
              << " (" << (n_feas_base * 100.0 / summaries.size()) << "%)\n";
    std::cout << "====================================================================================================================================\n";

    return 0;
}
