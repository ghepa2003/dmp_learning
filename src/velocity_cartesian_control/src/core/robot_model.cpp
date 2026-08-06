#include "velocity_cartesian_control/core/robot_model.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <stdexcept>
#include <algorithm>

namespace velocity_cartesian_control {
namespace core {

// Internal implementation details hidden behind the PIMPL idiom, to avoid
// exposing Pinocchio headers in the public RobotModel.hpp interface.
struct RobotModel::Impl {
    pinocchio::Model model;
    pinocchio::Data data;
    pinocchio::FrameIndex ee_frame_id;
    // Maps caller's joint order (joint_names_, e.g. fer_joint1..7) to
    // Pinocchio's internal q/v index for each joint - NOT assumed to be
    // identity, since Pinocchio may reorder joints internally relative to
    // URDF declaration order depending on the tree structure.
    std::array<int, kNumJoints> q_index;
    std::array<int, kNumJoints> v_index;
};

// Constructor: builds the Pinocchio model from URDF XML content, resolves the
// end-effector frame and the caller's joint order to Pinocchio's internal indices, and initializes the cached Jacobian and gravity vectors to zero.
RobotModel::RobotModel(const std::string& urdf_xml_content,
                        const std::vector<std::string>& joint_names,
                        const std::string& ee_frame_name)
    : joint_names_(joint_names), ee_frame_name_(ee_frame_name) {
    if (joint_names_.size() != static_cast<size_t>(kNumJoints)) {
        throw std::invalid_argument(
            "RobotModel: expected " + std::to_string(kNumJoints) +
            " joint names, got " + std::to_string(joint_names_.size()));
    }

    // Create the Pinocchio model and data structures, and build the model from the provided URDF XML content.
    impl_ = std::make_unique<Impl>();
    pinocchio::urdf::buildModelFromXML(urdf_xml_content, impl_->model);
    impl_->data = pinocchio::Data(impl_->model);

    if (!impl_->model.existFrame(ee_frame_name)) {
        throw std::invalid_argument("RobotModel: end-effector frame not found in URDF: " + ee_frame_name);
    }
    impl_->ee_frame_id = impl_->model.getFrameId(ee_frame_name);

    // Resolve caller joint order -> Pinocchio's internal indices, explicitly,
    // instead of assuming URDF declaration order == Pinocchio joint order.
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
}


RobotModel::~RobotModel() = default;

// Update the robot model's state based on the provided joint positions (q) and velocities (dq). 
// This function computes the forward kinematics, Jacobian, and gravity vector for the current joint state.
void RobotModel::update(const JointVector& q, const JointVector& dq) {
    // Assemble Pinocchio's full configuration/velocity vectors from the
    // caller's 7-vector, using the resolved indices - handles the case
    // where model.nq/model.nv > 7 (e.g. a free-flyer base, not expected
    // here but not assumed away either) and any internal joint reordering.
    Eigen::VectorXd q_full = Eigen::VectorXd::Zero(impl_->model.nq);
    Eigen::VectorXd v_full = Eigen::VectorXd::Zero(impl_->model.nv);
    for (int i = 0; i < kNumJoints; ++i) {
        q_full(impl_->q_index[static_cast<size_t>(i)]) = q(i);
        v_full(impl_->v_index[static_cast<size_t>(i)]) = dq(i);
    }

    pinocchio::computeJointJacobians(impl_->model, impl_->data, q_full);
    pinocchio::updateFramePlacements(impl_->model, impl_->data);

    const auto& oMf = impl_->data.oMf[impl_->ee_frame_id];
    ee_position_ = oMf.translation();
    ee_orientation_ = Eigen::Quaterniond(oMf.rotation());

    Jacobian6x7 J_full = Jacobian6x7::Zero();
    pinocchio::Data::Matrix6x J_pin(6, impl_->model.nv);
    J_pin.setZero();
    pinocchio::getFrameJacobian(impl_->model, impl_->data, impl_->ee_frame_id,
                                 pinocchio::LOCAL_WORLD_ALIGNED, J_pin);
    for (int i = 0; i < kNumJoints; ++i) {
        J_full.col(i) = J_pin.col(impl_->v_index[static_cast<size_t>(i)]);
    }
    jacobian_ = J_full;

    pinocchio::computeGeneralizedGravity(impl_->model, impl_->data, q_full);
    for (int i = 0; i < kNumJoints; ++i) {
        gravity_(i) = impl_->data.g(impl_->q_index[static_cast<size_t>(i)]);
    }
}

}  // namespace core
}  // namespace velocity_cartesian_control