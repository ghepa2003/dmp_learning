#pragma once

#include <string>
#include <vector>
#include <memory>
#include <Eigen/Dense>

namespace franka_cartesian_control {
namespace core {

/**
 * @brief Standalone rigid-body kinematic and dynamic model of the Franka Emika Panda/FR3 robot.
 *
 * @details
 * Robotics Theory & Architecture:
 * - This class encapsulates rigid-body algorithms using the Pinocchio spatial algebra library:
 *   1. Forward Kinematics (FK): Calculates the Cartesian position p in R^3 and orientation quaternion q in SO(3)
 *      of the designated End-Effector (EE) frame with respect to the robot's base frame link0.
 *   2. Geometric Jacobian: Computes the 6x7 matrix J(q) mapping joint velocities to end-effector spatial twist:
 *      v_ee = J(q) * dq = [v_linear; omega_angular]^T
 *      The Jacobian is expressed in the LOCAL_WORLD_ALIGNED convention (linear and angular velocities aligned with the base frame).
 *   3. Dynamics - Gravity Vector: g(q) = dV(q)/dq, representing joint torques required to statically balance gravity.
 *   4. Dynamics - Coriolis/Centrifugal Effects: c(q, dq) = C(q, dq)*dq, computed via Recursive Newton-Euler Algorithm (RNEA).
 *
 * Design Decisions:
 * - PIMPL Pattern (Pointer to Implementation): Completely encapsulates all Pinocchio and Boost headers inside
 *   robot_model.cpp. Callers only depend on standard Eigen headers, preventing header pollution and binary incompatibilities.
 * - Zero ROS Dependencies: The core model is independent of ROS 2 or controller interfaces, enabling deterministic
 *   real-time execution and standalone unit testing directly from URDF strings.
 */
class RobotModel {
public:
    static constexpr int kNumJoints = 7;

    using JointVector = Eigen::Matrix<double, kNumJoints, 1>;
    using Jacobian6x7 = Eigen::Matrix<double, 6, kNumJoints>;
    using Matrix7d = Eigen::Matrix<double, kNumJoints, kNumJoints>;

    /**
     * @brief Constructs the robot model from URDF XML content.
     * @param urdf_xml_content String containing the complete URDF XML model description.
     * @param joint_names Vector of 7 joint names in the expected order (e.g. fer_joint1 to fer_joint7).
     * @param ee_frame_name Name of the end-effector link/frame in the URDF (e.g. fer_link8 or fer_hand_tcp).
     */
    RobotModel(const std::string& urdf_xml_content,
               const std::vector<std::string>& joint_names,
               const std::string& ee_frame_name);

    ~RobotModel();

    /**
     * @brief Computes forward kinematics, frame placements, Jacobian, gravity, and Coriolis torques.
     * Must be called once per control iteration before reading state getters.
     * @param q Current joint positions (radians, 7x1).
     * @param dq Current joint velocities (rad/s, 7x1).
     */
    void update(const JointVector& q, const JointVector& dq);

    /// @brief Returns cached end-effector position [x, y, z] in robot base frame (meters).
    const Eigen::Vector3d& eePosition() const { return ee_position_; }

    /// @brief Returns cached end-effector orientation quaternion in robot base frame.
    const Eigen::Quaterniond& eeOrientation() const { return ee_orientation_; }

    /// @brief Returns cached 6x7 geometric Jacobian in LOCAL_WORLD_ALIGNED frame: [J_linear (3x7); J_angular (3x7)].
    const Jacobian6x7& jacobian() const { return jacobian_; }

    /// @brief Returns cached joint gravity compensation torques g(q) (Nm, 7x1).
    const JointVector& gravity() const { return gravity_; }

    /// @brief Returns cached joint Coriolis and centrifugal torques c(q, dq) (Nm, 7x1).
    const JointVector& coriolis() const { return coriolis_; }

    /// @brief Returns cached 7x7 joint-space mass/inertia matrix M(q), computed via CRBA.
    const Matrix7d& massMatrix() const { return mass_matrix_; }

    /// @brief Returns the joint names in the active order.
    const std::vector<std::string>& jointNames() const { return joint_names_; }

private:
    std::vector<std::string> joint_names_;
    std::string ee_frame_name_;

    // PIMPL implementation struct containing Pinocchio Model, Data, and index mappings
    struct Impl;
    std::unique_ptr<Impl> impl_;

    Eigen::Vector3d ee_position_;
    Eigen::Quaterniond ee_orientation_;
    Jacobian6x7 jacobian_;
    JointVector gravity_;
    JointVector coriolis_;
    Matrix7d mass_matrix_;
};

}  // namespace core
}  // namespace franka_cartesian_control
