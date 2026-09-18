#pragma once

#include <algorithm>
#include <memory>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include "franka_cartesian_control/core/robot_model.hpp"

namespace satellite_grasp_planner {
namespace core {

/**
 * @brief Translational manipulability index, product of the singular values
 *        of the translational (3xN) block of a geometric Jacobian.
 *
 * @details
 * 1:1 port of the formula used in
 * tools/gazebo_cartesian_eval/scripts/manipulability_check/compute_manipulability_goals.py
 * (the offline Pinocchio analysis already validated for the thesis):
 *
 * @code
 *   J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
 *   J_p = J[:3, :]
 *   w_t = sqrt(det(J_p @ J_p.T))
 * @endcode
 *
 * sqrt(det(Jt*Jt^T)) and "product of the singular values of Jt" are the same
 * quantity (det(Jt*Jt^T) = product of sigma_i^2), computed here via SVD
 * instead of det() for better numerical conditioning on a near-singular Jt.
 * Both the Python script and this function use the SAME URDF
 * (fer_flat_effort.urdf), the SAME frame (fer_link8, default) and the SAME
 * Jacobian convention (LOCAL_WORLD_ALIGNED, via franka_cartesian_control's
 * core::RobotModel), so the numbers match the ones already validated in the
 * thesis - see test_phase_selector.cpp for the cross-check against the
 * recorded Gazebo runs.
 *
 * Templated on the Jacobian block's Eigen expression type (rather than a
 * fixed Eigen::MatrixXd parameter) so it can be called directly on a
 * zero-copy `.topRows<3>()` block expression of an already-computed 6x7
 * Jacobian - this runs inside PhaseSelector's per-sample scoring loop, so
 * avoiding an extra dynamic-matrix allocation per call matters (see the
 * coarse-to-fine step-count budget documented in PhaseSelector).
 *
 * @param Jt Translational block of a geometric Jacobian (3 rows x N columns,
 *           N = number of joints; for the Panda, 3x7).
 * @return Product of the singular values of Jt (== sqrt(det(Jt*Jt^T))).
 *         0 at/near a translational singularity.
 */
template <typename Derived>
double wTransFromJacobian(const Eigen::MatrixBase<Derived>& Jt) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jt);
    const auto& sv = svd.singularValues();
    double w = 1.0;
    for (Eigen::Index i = 0; i < sv.size(); ++i) {
        w *= sv(i);
    }
    return std::max(0.0, w);
}

/**
 * @brief Convenience wrapper: w_trans(q) for a given joint configuration,
 *        via a shared franka_cartesian_control::core::RobotModel.
 *
 * @details
 * Calls RobotModel::update(q, 0) (which additionally computes gravity,
 * Coriolis and the mass matrix via RNEA/CRBA - unused here, but RobotModel
 * offers no cheaper "kinematics only" path today) and reads back the cached
 * Jacobian. When scoring MANY configurations along an already-simulated
 * joint path (as PhaseSelector does), prefer calling wTransFromJacobian()
 * directly on the Jacobian already computed by JointPathSimulator's own
 * RobotModel::update() call for the DLS step, instead of this method - it
 * avoids a second full Pinocchio pass (FK + Jacobian + RNEA + CRBA) per
 * sample, which matters given the coarse-to-fine step-count budget documented
 * in PhaseSelector.
 */
class Manipulability {
public:
    explicit Manipulability(std::shared_ptr<franka_cartesian_control::core::RobotModel> robot_model);

    double wTrans(const franka_cartesian_control::core::RobotModel::JointVector& q) const;

private:
    std::shared_ptr<franka_cartesian_control::core::RobotModel> robot_model_;
};

}  // namespace core
}  // namespace satellite_grasp_planner
