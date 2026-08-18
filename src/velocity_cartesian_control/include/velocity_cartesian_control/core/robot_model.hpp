#pragma once

#include <string>
#include <vector>
#include <memory>
#include <Eigen/Dense>

namespace velocity_cartesian_control {
namespace core {

// Zero-ROS-dependency wrapper around a Pinocchio model, exposing exactly
// what a Cartesian impedance law needs: forward kinematics, Jacobian, and
// gravity vector for the Franka arm's 7 actuated joints. No effort/velocity
// interfaces, no controller_interface dependency - testable standalone
// against a plain URDF file, mirroring the core/ros split already used in
// haptic_dmp_learning.
class RobotModel {
public:
    static constexpr int kNumJoints = 7;

    // using stands for the Eigen types used in the interface, 
    // to avoid repeating the long Eigen::Matrix<double, ...> type everywhere.  
    using JointVector = Eigen::Matrix<double, kNumJoints, 1>;
    using Jacobian6x7 = Eigen::Matrix<double, 6, kNumJoints>;

    // Single entry point: always XML content, never a file path. Reading
    // from disk (when needed - e.g. standalone tests) is the caller's
    // responsibility, kept out of core/ on purpose (same principle already
    // used in haptic_dmp_learning: core/ never touches the filesystem,
    // only the io layer above it does).
    RobotModel(const std::string& urdf_xml_content,
               const std::vector<std::string>& joint_names,
               const std::string& ee_frame_name);


    // Destructor: defaulted, but declared here to ensure the unique_ptr<Impl>
    // is destructed in the .cpp where Impl is defined, avoiding an incomplete
    // type error in the header.
    ~RobotModel();

    // Recomputes forward kinematics, Jacobian, and gravity for the given
    // joint state. Must be called once per control cycle before any of the
    // getters below - they return values cached from the last update().
    void update(const JointVector& q, const JointVector& dq);

    // End-effector pose in the model's base frame, from the last update().
    const Eigen::Vector3d& eePosition() const { return ee_position_; }
    const Eigen::Quaterniond& eeOrientation() const { return ee_orientation_; }

    // 6x7 Jacobian (linear; angular), LOCAL_WORLD_ALIGNED convention -
    // matches franka::Model::zeroJacobian used by the SERL reference.
    const Jacobian6x7& jacobian() const { return jacobian_; }

    // Gravity torque vector g(q), 7x1 - NOTE: on Gazebo this is likely
    // redundant with gravity compensation already applied by the plugin
    // (verified in ign_system.cpp write()). Exposed here regardless so the
    // controller can decide whether to add it, gated by a parameter -
    // see open point about sim-vs-real gravity handling.
    const JointVector& gravity() const { return gravity_; }

    // Returns the joint names in the order expected by this model (the same
    // order as the input joint_names_ vector passed to the constructor).
    const std::vector<std::string>& jointNames() const { return joint_names_; }

private:
    std::vector<std::string> joint_names_;
    std::string ee_frame_name_;

    // Pinocchio model/data - concrete types deliberately not exposed in the
    // header, to avoid forcing any downstream users of this class to depend on
    // Pinocchio. The Impl struct is defined in the .cpp file, and the
    // unique_ptr<Impl> is destructed there too, so the Pinocchio types
    // are never seen by any downstream users of this header.
    struct Impl;
    std::unique_ptr<Impl> impl_;

    Eigen::Vector3d ee_position_;
    Eigen::Quaterniond ee_orientation_;
    Jacobian6x7 jacobian_;
    JointVector gravity_;
};

}  // namespace core
}  // namespace velocity_cartesian_control
