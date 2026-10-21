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
    double dt_fast = 0.05;
    int n_steps = static_cast<int>(std::ceil(tau / dt_fast));

    // Test 2 phases: 90° (expected 24°) and 210° (expected 59°)
    std::vector<double> test_phases = {90.0, 210.0};
    std::vector<double> expected_full_rolls = {+24.0, +59.0};

    NullspaceBiasedResolution::Params ns_params;
    ns_params.nullspace_gain = 0.2236;
    ns_params.joint1_nullspace_gain_scale = 10.0;
    JointPathSimulator::Params sim_params;
    sim_params.dt = dt_fast;
    sim_params.strategy = std::make_shared<NullspaceBiasedResolution>(ns_params);

    for (size_t idx = 0; idx < test_phases.size(); ++idx) {
        double phase_deg = test_phases[idx];
        double exp_roll = expected_full_rolls[idx];
        double theta_rad = phase_deg * M_PI / 180.0;

        AnchoredGoal anchored = anchorAndRotate(p0, demo_disp, center, axis, theta_rad);
        Eigen::Vector3d p_goal = anchored.p_rotated_world;

        Eigen::AngleAxisd rot_sat(theta_rad, axis.normalized());
        Eigen::Matrix3d R_base = rot_sat.toRotationMatrix() * quat0.toRotationMatrix();

        ProDMP prodmp = prodmp_template;
        prodmp.setRelativeGoal(false);
        prodmp.setInitialConditions(0.0, p0, Eigen::Vector3d::Zero());
        prodmp.setGoal(p_goal);

        std::vector<Eigen::Vector3d> positions(n_steps);
        std::vector<double> s_values(n_steps);
        for (int i = 0; i < n_steps; ++i) {
            positions[i] = prodmp.step(dt_fast);
            s_values[i] = std::min(1.0, static_cast<double>(i + 1) * dt_fast / tau);
        }

        double best_psi = 0.0;
        double min_viol = 1e9;
        double best_q6_err = 1e9;
        double q6_center = 107.0;

        std::vector<CartesianSample> trajectory(n_steps);

        #pragma omp parallel
        {
            auto local_robot_model = std::make_shared<RobotModel>(urdf_content, joint_names, "fer_hand_tcp");
            JointPathSimulator sim(local_robot_model, sim_params);
            std::vector<CartesianSample> local_traj(n_steps);

            #pragma omp for schedule(dynamic)
            for (int psi_int = -180; psi_int <= 180; psi_int += 1) {
                double psi_deg = static_cast<double>(psi_int);
                double psi_rad = psi_deg * M_PI / 180.0;

                Eigen::Matrix3d R_psi = R_base * Eigen::AngleAxisd(psi_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                Eigen::Quaterniond quat_goal_psi(R_psi);
                quat_goal_psi.normalize();

                for (int i = 0; i < n_steps; ++i) {
                    local_traj[i].position = positions[i];
                    local_traj[i].orientation = slerpShortestPath(quat0, quat_goal_psi, s_values[i]);
                }

                auto steps = sim.simulate(local_traj, q0);

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

                double q6_final = steps.back().q(5) * 180.0 / M_PI;
                double q6_err = std::abs(q6_final - q6_center);

                #pragma omp critical
                {
                    if (max_viol < min_viol - 1e-6) {
                        min_viol = max_viol;
                        best_q6_err = q6_err;
                        best_psi = psi_deg;
                    } else if (std::abs(max_viol - min_viol) < 1e-6) {
                        if (q6_err < best_q6_err) {
                            best_q6_err = q6_err;
                            best_psi = psi_deg;
                        }
                    }
                }
            }
        }

        std::cout << "Phase " << static_cast<int>(phase_deg) << "°: Roll(dt=0.005s) = "
                  << static_cast<int>(exp_roll) << "°, Roll(dt=0.05s) = " << static_cast<int>(best_psi)
                  << "° (Diff = " << static_cast<int>(best_psi - exp_roll) << "°)\n" << std::flush;
    }

    return 0;
}
