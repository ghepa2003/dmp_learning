#include "satellite_grasp_planner/core/kinematic_feasibility.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "haptic_dmp_learning/core/math_utils.hpp"
#include "satellite_grasp_planner/core/joint_path_simulator.hpp"

namespace satellite_grasp_planner {
namespace core {

FeasibilityResult checkKinematicFeasibility(
    const haptic_dmp_learning::core::ProDMP& prodmp_template,
    const Eigen::Vector3d& target_position,
    const Eigen::Quaterniond& target_orientation,
    const std::shared_ptr<RobotModel>& robot_model,
    const RobotModel::JointVector& q0,
    double dt,
    double max_joint_violation_tol_rad) {
    if (!robot_model) {
        throw std::invalid_argument("checkKinematicFeasibility: robot_model is null.");
    }
    if (!(dt > 0.0)) {
        throw std::invalid_argument("checkKinematicFeasibility: dt must be > 0.");
    }

    // Start pose = FK at q0.
    robot_model->update(q0, RobotModel::JointVector::Zero());
    const Eigen::Vector3d p0 = robot_model->eePosition();
    const Eigen::Quaterniond quat0 = robot_model->eeOrientation();

    const double tau = prodmp_template.tau();
    const int n_steps = static_cast<int>(std::ceil(tau / dt));

    haptic_dmp_learning::core::ProDMP prodmp = prodmp_template;  // step() mutates: work on a copy
    prodmp.setRelativeGoal(false);
    prodmp.setInitialConditions(0.0, p0, Eigen::Vector3d::Zero());
    prodmp.setGoal(target_position);

    std::vector<CartesianSample> trajectory;
    trajectory.reserve(static_cast<size_t>(n_steps));
    for (int i = 0; i < n_steps; ++i) {
        CartesianSample sample;
        sample.position = prodmp.step(dt);
        const double s = std::min(1.0, static_cast<double>(i + 1) * dt / tau);
        sample.orientation =
            haptic_dmp_learning::core::slerpShortestPath(quat0, target_orientation, s);
        trajectory.push_back(sample);
    }

    NullspaceBiasedResolution::Params ns_params;  // defaults = impedance-controller proxy, as in the tools
    JointPathSimulator::Params sim_params;
    sim_params.dt = dt;
    sim_params.strategy = std::make_shared<NullspaceBiasedResolution>(ns_params);
    const JointPathSimulator sim(robot_model, sim_params);
    const auto steps = sim.simulate(trajectory, q0);

    FeasibilityResult res;
    if (steps.empty()) {
        return res;  // tau <= 0: nothing simulated, infeasible
    }

    const RobotModel::JointVector& lo = robot_model->jointLowerLimits();
    const RobotModel::JointVector& hi = robot_model->jointUpperLimits();
    double max_viol = 0.0;
    for (std::size_t i = 0; i < steps.size(); ++i) {
        const auto& step = steps[i];
        for (int j = 0; j < RobotModel::kNumJoints; ++j) {
            const double below = lo(j) - step.q(j);
            const double above = step.q(j) - hi(j);
            if (std::max(below, above) > max_viol) {  // strictly: only real violations are recorded
                res.worst_joint = j;
                res.worst_step = static_cast<int>(i);
                res.worst_joint_value_rad = step.q(j);
                res.worst_joint_limit_rad = (below >= above) ? lo(j) : hi(j);
            }
            max_viol = std::max({max_viol, below, above});
        }
    }
    if (res.worst_step >= 0) {
        res.worst_step_fraction =
            steps.size() > 1 ? static_cast<double>(res.worst_step) / static_cast<double>(steps.size() - 1)
                             : 0.0;
    }
    res.max_joint_violation_rad = max_viol;
    res.q_final = steps.back().q;
    res.w_trans_final = steps.back().w_trans;

    robot_model->update(steps.back().q, RobotModel::JointVector::Zero());
    res.pos_err_final_mm = (robot_model->eePosition() - target_position).norm() * 1000.0;

    // Only the joint-limit criterion decides: pos_err_final_mm is diagnostic (it contains the
    // simulator's Kp tracking-lag artefact, see the header).
    res.feasible = res.max_joint_violation_rad < max_joint_violation_tol_rad;
    return res;
}

}  // namespace core
}  // namespace satellite_grasp_planner
