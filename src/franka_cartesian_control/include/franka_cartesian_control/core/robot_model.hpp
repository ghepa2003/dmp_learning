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
    using Matrix6d = Eigen::Matrix<double, 6, 6>;

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

    /**
     * @brief Reflected Cartesian (operational-space) inertia matrix Lambda(q), Khatib formulation.
     *
     * @details
     * Physical meaning:
     * @code
     *   Lambda(q) = ( J(q) * M(q)^-1 * J(q)^T )^-1        in R^{6x6}
     * @endcode
     * Lambda(q) is the apparent inertia the end-effector presents at its frame: it maps a
     * Cartesian acceleration of the EE to the equivalent Cartesian wrench for the free
     * (unconstrained) arm, f = Lambda(q) * a_ee. It is the quantity evaluated in the
     * Variable Impedance Matching (VIM) condition of Yoshida & Nakanishi: a desired
     * task-space impedance is only physically realizable if the reflected inertia along
     * the interaction direction stays within bounds.
     *
     * Numerics:
     * - Reuses the M(q) (CRBA) and J(q) (LOCAL_WORLD_ALIGNED, base frame fer_link0) cached
     *   by the last update(); nothing is recomputed here.
     * - M^-1 * J^T is obtained with an LDLT solve on the SPD mass matrix, the same
     *   M-inverse robustness approach used elsewhere in this package.
     * - The final 6x6 inversion uses a complete orthogonal decomposition (pseudo-inverse),
     *   which degrades gracefully to a least-squares result near kinematic singularities
     *   instead of diverging like a direct .inverse().
     *
     * @warning NOT real-time safe. Performs dense decompositions; must never be called from
     *          the 1 kHz update()/control path. Intended for low-rate monitoring nodes that
     *          check the VIM condition.
     *
     * @return 6x6 reflected Cartesian inertia Lambda(q), ordered [linear (3); angular (3)]
     *         consistently with jacobian().
     */
    Matrix6d reflectedCartesianInertia() const;

    /**
     * @brief Reflected mass seen by the end-effector along a given Cartesian direction.
     *
     * @details
     * Projects the translational 3x3 block of Lambda(q) onto a unit direction:
     * @code
     *   m_i = u^T * Lambda_tt(q) * u ,   u = direction / ||direction||
     * @endcode
     * where Lambda_tt is the upper-left 3x3 (translational) block of
     * reflectedCartesianInertia(). This scalar is the effective mass entering the 1-D VIM
     * criterion of Yoshida & Nakanishi along the contact/approach axis. The direction is
     * normalized internally; callers need not pass a unit vector.
     *
     * @warning NOT real-time safe. Wraps reflectedCartesianInertia(); use only from
     *          low-rate monitoring code, never from the 1 kHz control loop.
     *
     * @param direction Non-zero Cartesian direction in the robot base frame (fer_link0).
     * @return Reflected mass m_i along the normalized direction (kg). Returns 0 if
     *         @p direction has (near) zero norm.
     */
    double reflectedMassAlongDirection(const Eigen::Vector3d& direction) const;

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
