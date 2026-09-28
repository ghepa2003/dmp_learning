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
    {-2.8973, 2.8973},   // q1
    {-1.7628, 1.7628},   // q2
    {-2.8973, 2.8973},   // q3
    {-3.0718, -0.0698},  // q4
    {-2.8973, 2.8973},   // q5
    {-0.0175, 3.7525},   // q6: [-1.0, 215.0] deg
    {-2.8973, 2.8973},   // q7
};

Eigen::Quaterniond slerpShortestPath(const Eigen::Quaterniond& q0, const Eigen::Quaterniond& q1, double s) {
    Eigen::Quaterniond target = q1;
    if (q0.coeffs().dot(target.coeffs()) < 0.0) target.coeffs() = -target.coeffs();
    return q0.slerp(s, target).normalized();
}

int main() {
    std::string urdf_path = "/root/thesis_ws/fer_flat_effort.urdf";
    std::string weights_path = "/root/thesis_ws/runs/20260915_090358_fit_reach_task_baseline_prodmp/weights.yaml";

    std::ifstream f(urdf_path);
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
    Eigen::Vector3d p_grasp0 = p0 + demo_disp;

    double dt = 0.005;
    int n_steps = static_cast<int>(std::ceil(tau / dt));

    std::vector<double> test_phases = {0.0, 90.0, 180.0, 210.0, 270.0, 330.0};

    NullspaceBiasedResolution::Params ns_params;
    ns_params.nullspace_gain = 0.2236;
    ns_params.joint1_nullspace_gain_scale = 10.0;
    JointPathSimulator::Params sim_params;
    sim_params.dt = dt;
    sim_params.strategy = std::make_shared<NullspaceBiasedResolution>(ns_params);

    std::cout << "Starting key-phases check with corrected radial geometry (dt=0.005s)...\n" << std::flush;

    for (double phase_deg : test_phases) {
        auto t_p0 = std::chrono::high_resolution_clock::now();
        double theta_rad = phase_deg * M_PI / 180.0;
        AnchoredGoal anchored = anchorAndRotate(p0, demo_disp, center, axis, theta_rad);
        Eigen::Vector3d p_goal = anchored.p_rotated_world;

        Eigen::AngleAxisd rot_sat(theta_rad, axis.normalized());
        Eigen::Vector3d r_vec = rot_sat * (p_grasp0 - center);
        Eigen::Vector3d r_hat = r_vec.normalized();
        Eigen::Vector3d t_vec = axis.normalized().cross(r_hat);
        Eigen::Vector3d t_hat = t_vec.normalized();
        Eigen::Vector3d x_hat = t_hat.cross(r_hat).normalized();

        Eigen::Matrix3d R_base;
        R_base.col(0) = x_hat;
        R_base.col(1) = t_hat;
        R_base.col(2) = r_hat;

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

        // Test roll scan with 5 deg step for quick validation
        double best_psi = 0.0;
        double min_viol = 1e9;
        double best_q6_err = 1e9;
        double best_w_trans = 0.0;
        double best_q6 = 0.0;
        int n_feas = 0;

        #pragma omp parallel
        {
            auto local_robot_model = std::make_shared<RobotModel>(urdf_content, joint_names, "fer_hand_tcp");
            JointPathSimulator sim(local_robot_model, sim_params);
            std::vector<CartesianSample> local_traj(n_steps);

            #pragma omp for schedule(dynamic)
            for (int psi_int = -180; psi_int <= 180; psi_int += 5) {
                double psi_deg = static_cast<double>(psi_int);
                double psi_rad = psi_deg * M_PI / 180.0;

                Eigen::Matrix3d R_psi = R_base * Eigen::AngleAxisd(psi_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                Eigen::Quaterniond quat_goal(R_psi);
                quat_goal.normalize();

                for (int k = 0; k < n_steps; ++k) {
                    local_traj[k].position = positions[k];
                    local_traj[k].orientation = slerpShortestPath(quat0, quat_goal, s_values[k]);
                }

                auto steps = sim.simulate(local_traj, q0);

                double max_v = 0.0;
                for (const auto& st : steps) {
                    for (int j = 0; j < 7; ++j) {
                        double val = st.q(j);
                        double min_l = JOINT_LIMITS[j].first;
                        double max_l = JOINT_LIMITS[j].second;
                        if (val < min_l) max_v = std::max(max_v, min_l - val);
                        else if (val > max_l) max_v = std::max(max_v, val - max_l);
                    }
                }

                double q6_fin = steps.back().q(5) * 180.0 / M_PI;
                double q6_err = std::abs(q6_fin - 107.0);
                double w_tr = steps.back().w_trans;
                bool feas = (max_v < 1e-4);

                #pragma omp critical
                {
                    if (feas) n_feas++;
                    if (max_v < min_viol - 1e-6) {
                        min_viol = max_v;
                        best_q6_err = q6_err;
                        best_psi = psi_deg;
                        best_w_trans = w_tr;
                        best_q6 = q6_fin;
                    } else if (std::abs(max_v - min_viol) < 1e-6) {
                        if (q6_err < best_q6_err) {
                            best_q6_err = q6_err;
                            best_psi = psi_deg;
                            best_w_trans = w_tr;
                            best_q6 = q6_fin;
                        }
                    }
                }
            }
        }

        auto t_p1 = std::chrono::high_resolution_clock::now();
        double dt_phase_ms = std::chrono::duration<double, std::milli>(t_p1 - t_p0).count();

        std::cout << "Phase " << std::setw(3) << static_cast<int>(phase_deg) << "°: "
                  << "Feas=" << (min_viol < 1e-4 ? "OK" : "FAIL") << " (Feas count=" << n_feas << "/73) | "
                  << "psi* = " << std::setw(4) << static_cast<int>(best_psi) << "° | "
                  << "w_trans = " << std::fixed << std::setprecision(4) << best_w_trans << " | "
                  << "q6_fin = " << std::fixed << std::setprecision(1) << best_q6 << "° | "
                  << "Time = " << (dt_phase_ms / 1000.0) << "s\n" << std::flush;
    }

    return 0;
}
