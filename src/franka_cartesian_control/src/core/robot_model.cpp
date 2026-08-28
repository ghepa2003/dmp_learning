#include "franka_cartesian_control/core/robot_model.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>

#include <stdexcept>
#include <algorithm>

namespace franka_cartesian_control {
namespace core {

/**
 * @brief Internal PIMPL implementation holding Pinocchio C++ objects.
 */
struct RobotModel::Impl {
    pinocchio::Model model;            ///< Static kinematic/dynamic tree parsed from URDF
    pinocchio::Data data;              ///< Dynamic workspace holding intermediate computations (FK, Jacobians, torques)
    pinocchio::FrameIndex ee_frame_id; ///< Integer frame identifier for the chosen end-effector link

    /**
     * Explicit index mapping arrays:
     * Pinocchio builds an internal joint tree and assigns indices to configuration (q) and velocity (v) coordinates.
     * We map each caller joint index (0 to 6) to Pinocchio's model.idx_qs[joint_id] and model.idx_vs[joint_id].
     * This guarantees correct ordering regardless of how URDF XML nodes are sequenced.
     */
    std::array<int, kNumJoints> q_index;
    std::array<int, kNumJoints> v_index;
};

RobotModel::RobotModel(const std::string& urdf_xml_content,
                        const std::vector<std::string>& joint_names,
                        const std::string& ee_frame_name)
    : joint_names_(joint_names), ee_frame_name_(ee_frame_name) {
    if (joint_names_.size() != static_cast<size_t>(kNumJoints)) {
        throw std::invalid_argument(
            "RobotModel: expected " + std::to_string(kNumJoints) +
            " joint names, got " + std::to_string(joint_names_.size()));
    }

    impl_ = std::make_unique<Impl>();

    // Parse URDF XML string directly into Pinocchio model structure
    pinocchio::urdf::buildModelFromXML(urdf_xml_content, impl_->model);
    impl_->data = pinocchio::Data(impl_->model);

    // Validate and locate the designated end-effector frame
    if (!impl_->model.existFrame(ee_frame_name)) {
        throw std::invalid_argument("RobotModel: end-effector frame not found in URDF: " + ee_frame_name);
    }
    impl_->ee_frame_id = impl_->model.getFrameId(ee_frame_name);

    // Map each joint name to its internal configuration and velocity coordinate offset
    for (int i = 0; i < kNumJoints; ++i) {
        const std::string& jn = joint_names_[static_cast<size_t>(i)];
        if (!impl_->model.existJointName(jn)) {
            throw std::invalid_argument("RobotModel: joint not found in URDF: " + jn);
        }
        pinocchio::JointIndex jid = impl_->model.getJointId(jn);
        impl_->q_index[i] = impl_->model.idx_qs[jid];
        impl_->v_index[i] = impl_->model.idx_vs[jid];
    }

    jacobian_.setZero();
    gravity_.setZero();
    coriolis_.setZero();
    mass_matrix_.setZero();
}

RobotModel::~RobotModel() = default;

void RobotModel::update(const JointVector& q, const JointVector& dq) {
    // 1. Pack the 7 actuated joint values into Pinocchio's full coordinate vectors
    Eigen::VectorXd q_full = Eigen::VectorXd::Zero(impl_->model.nq);
    Eigen::VectorXd v_full = Eigen::VectorXd::Zero(impl_->model.nv);
    for (int i = 0; i < kNumJoints; ++i) {
        q_full(impl_->q_index[static_cast<size_t>(i)]) = q(i);
        v_full(impl_->v_index[static_cast<size_t>(i)]) = dq(i);
    }

    // 2. Compute Forward Kinematics (FK) and Joint Jacobians for all joints
    pinocchio::computeJointJacobians(impl_->model, impl_->data, q_full);
    pinocchio::updateFramePlacements(impl_->model, impl_->data);

    // 3. Extract End-Effector Placement in Base Frame (SE(3))
    const auto& oMf = impl_->data.oMf[impl_->ee_frame_id];
    ee_position_ = oMf.translation();
    ee_orientation_ = Eigen::Quaterniond(oMf.rotation());

    // 4. Compute 6xN Geometric Frame Jacobian in LOCAL_WORLD_ALIGNED frame
    // This convention aligns linear and angular velocities with the robot base frame coordinates (link0),
    // matching standard Cartesian control laws v = J * dq.
    Jacobian6x7 J_full = Jacobian6x7::Zero();
    pinocchio::Data::Matrix6x J_pin(6, impl_->model.nv);
    J_pin.setZero();
    pinocchio::getFrameJacobian(impl_->model, impl_->data, impl_->ee_frame_id,
                                 pinocchio::LOCAL_WORLD_ALIGNED, J_pin);
    for (int i = 0; i < kNumJoints; ++i) {
        J_full.col(i) = J_pin.col(impl_->v_index[static_cast<size_t>(i)]);
    }
    jacobian_ = J_full;

    // 5. Compute Generalized Gravity Torques g(q) = dV/dq
    pinocchio::computeGeneralizedGravity(impl_->model, impl_->data, q_full);
    for (int i = 0; i < kNumJoints; ++i) {
        gravity_(i) = impl_->data.g(impl_->q_index[static_cast<size_t>(i)]);
    }

    // 6. Compute Coriolis Torques c(q, dq)
    // Pinocchio nonLinearEffects calculates NLE(q, dq) = C(q, dq)*dq + g(q) via RNEA.
    // Subtracting g(q) isolates the Coriolis and centrifugal joint torques.
    Eigen::VectorXd nle = pinocchio::nonLinearEffects(impl_->model, impl_->data, q_full, v_full);
    for (int i = 0; i < kNumJoints; ++i) {
        coriolis_(i) = nle(impl_->q_index[static_cast<size_t>(i)]) - gravity_(i);
    }

    // 7. Compute Joint-Space Mass Matrix M(q) via Composite Rigid Body Algorithm (CRBA).
    // CRBA fills only the upper triangular part of data.M by Pinocchio convention;
    // mirror it to get a full symmetric matrix before extracting our 7x7 submatrix.
    pinocchio::crba(impl_->model, impl_->data, q_full);
    impl_->data.M.triangularView<Eigen::StrictlyLower>() =
        impl_->data.M.transpose().triangularView<Eigen::StrictlyLower>();
    for (int i = 0; i < kNumJoints; ++i) {
        for (int j = 0; j < kNumJoints; ++j) {
            mass_matrix_(i, j) = impl_->data.M(impl_->v_index[static_cast<size_t>(i)],
                                                impl_->v_index[static_cast<size_t>(j)]);
        }
    }
}

// ---------------------------------------------------------------------------
// Reflected Cartesian inertia (Khatib operational-space inertia) for VIM checks.
// Low-rate diagnostic helpers only: NOT called from update() or the 1 kHz path.
// ---------------------------------------------------------------------------

RobotModel::Matrix6d RobotModel::reflectedCartesianInertia() const {
    // Reuse the quantities cached by the last update(): SPD joint-space mass matrix
    // M(q) (CRBA) and the 6x7 LOCAL_WORLD_ALIGNED frame Jacobian J(q). Nothing is
    // recomputed here.
    const Matrix7d& M = mass_matrix_;
    const Jacobian6x7& J = jacobian_;

    // M^-1 * J^T (7x6) via an LDLT solve on the SPD mass matrix, matching the
    // M-inverse robustness approach used elsewhere in this package.
    const Eigen::Matrix<double, kNumJoints, 6> Minv_Jt = M.ldlt().solve(J.transpose());

    // Cartesian mobility matrix Omega(q) = J * M^-1 * J^T (6x6, symmetric PSD).
    Matrix6d Omega = J * Minv_Jt;
    Omega = 0.5 * (Omega + Omega.transpose());  // symmetrize against round-off

    // Lambda(q) = Omega(q)^-1. Use a complete orthogonal decomposition instead of a
    // direct inverse so the result stays finite (least-squares pseudo-inverse) when
    // Omega loses rank near a kinematic singularity.
    return Omega.completeOrthogonalDecomposition().pseudoInverse();
}

double RobotModel::reflectedMassAlongDirection(const Eigen::Vector3d& direction) const {
    const double norm = direction.norm();
    if (norm < 1e-12) {
        return 0.0;
    }
    const Eigen::Vector3d u = direction / norm;
    const Matrix6d Lambda = reflectedCartesianInertia();
    const Eigen::Matrix3d Lambda_translational = Lambda.topLeftCorner<3, 3>();
    return (u.transpose() * Lambda_translational * u).value();
}

}  // namespace core
}  // namespace franka_cartesian_control