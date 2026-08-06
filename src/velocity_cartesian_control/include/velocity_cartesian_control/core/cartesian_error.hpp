#pragma once

#include <Eigen/Dense>

namespace velocity_cartesian_control {
namespace core {

struct CartesianError {
    Eigen::Vector3d linear;   // target_position - current_position
    Eigen::Vector3d angular;  // orientation error, log map convention (see below)
};

// Rotation vector (axis * angle) log map of a unit quaternion, same
// convention as haptic_dmp_learning::core::QuaternionDMP::logMap -
// reimplemented locally to avoid a cross-package dependency.
inline Eigen::Vector3d logMap(const Eigen::Quaterniond& q) {
    Eigen::Vector3d v(q.x(), q.y(), q.z());
    double vnorm = v.norm();
    if (vnorm < 1e-8) return Eigen::Vector3d::Zero();
    double w = std::max(-1.0, std::min(1.0, q.w()));
    double angle = std::acos(w);
    return angle * v / vnorm;
}

// Computes Cartesian pose error (target - current). Orientation error is
// 2*logMap(q_target * q_current^-1), matching the standard formulation used
// for the transformation-system forcing term elsewhere in this project.
inline CartesianError computePoseError(const Eigen::Vector3d& current_pos,
                                        const Eigen::Quaterniond& current_quat,
                                        const Eigen::Vector3d& target_pos,
                                        const Eigen::Quaterniond& target_quat) {
    CartesianError err;
    err.linear = target_pos - current_pos;

    // Normalize quaternions to avoid numerical drift issues
    Eigen::Quaterniond q_cur = current_quat.normalized();
    Eigen::Quaterniond q_tgt = target_quat.normalized();

    // Shortest-path fix: if the dot product is negative, the two
    // quaternions represent the same rotation but are on opposite
    // hemispheres of the double cover - flip one to avoid a spurious
    // near-180 degree error.
    if (q_cur.coeffs().dot(q_tgt.coeffs()) < 0.0) {
        q_tgt.coeffs() = -q_tgt.coeffs();
    }
    Eigen::Quaterniond q_err = q_tgt * q_cur.conjugate();
    err.angular = 2.0 * logMap(q_err);

    return err;
}

}  // namespace core
}  // namespace velocity_cartesian_control