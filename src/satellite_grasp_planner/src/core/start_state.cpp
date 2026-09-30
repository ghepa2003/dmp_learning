#include "satellite_grasp_planner/core/start_state.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace satellite_grasp_planner {
namespace core {

using franka_cartesian_control::core::RobotModel;

void validateStartJoints(const std::vector<double>& values, const RobotModel::JointVector& lower,
                         const RobotModel::JointVector& upper) {
    const std::string name = kStartJointsParameterName;
    if (values.size() != static_cast<std::size_t>(RobotModel::kNumJoints)) {
        throw std::invalid_argument("parameter '" + name + "' must have exactly " +
                                    std::to_string(RobotModel::kNumJoints) + " elements (fer_joint1..7), got " +
                                    std::to_string(values.size()));
    }
    for (int i = 0; i < RobotModel::kNumJoints; ++i) {
        const double v = values[static_cast<std::size_t>(i)];
        if (!std::isfinite(v)) {
            throw std::invalid_argument("parameter '" + name + "': fer_joint" + std::to_string(i + 1) +
                                        " is not finite");
        }
        if (v < lower(i) || v > upper(i)) {
            std::ostringstream m;
            m << "parameter '" << name << "': fer_joint" << (i + 1) << " = " << v << " rad is outside the joint limits ["
              << lower(i) << ", " << upper(i) << "] rad";
            throw std::invalid_argument(m.str());
        }
    }
}

StartState makeStartStateFromOverride(const std::vector<double>& values, const std::string& urdf_path) {
    std::ifstream urdf(urdf_path);
    if (!urdf.is_open()) {
        throw std::runtime_error("makeStartStateFromOverride: cannot open URDF " + urdf_path);
    }
    std::stringstream text;
    text << urdf.rdbuf();
    std::vector<std::string> joint_names;
    for (int i = 1; i <= RobotModel::kNumJoints; ++i) joint_names.push_back("fer_joint" + std::to_string(i));
    const RobotModel robot(text.str(), joint_names, "fer_hand_tcp");
    validateStartJoints(values, robot.jointLowerLimits(), robot.jointUpperLimits());

    StartState s;
    for (int i = 0; i < RobotModel::kNumJoints; ++i) s.q(i) = values[static_cast<std::size_t>(i)];
    s.source = std::string(kStartJointsParameterName) + " parameter";
    return s;
}

StartDistance distanceFromReadyPose(RobotModel& robot, const RobotModel::JointVector& q0) {
    const RobotModel::JointVector zero = RobotModel::JointVector::Zero();
    const RobotModel::JointVector ready = RobotModel::readyPose();
    robot.update(ready, zero);
    const Eigen::Vector3d p_ready = robot.eePosition();
    robot.update(q0, zero);
    const Eigen::Vector3d p0 = robot.eePosition();

    StartDistance d;
    const RobotModel::JointVector diff = q0 - ready;
    d.joint_norm_rad = diff.norm();
    int idx = 0;
    d.joint_max_rad = diff.cwiseAbs().maxCoeff(&idx);
    d.joint_max_index = idx + 1;
    d.task_mm = (p0 - p_ready).norm() * 1000.0;
    return d;
}

}  // namespace core
}  // namespace satellite_grasp_planner
