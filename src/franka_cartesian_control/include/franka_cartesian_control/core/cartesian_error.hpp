#pragma once

#include <Eigen/Dense>

namespace franka_cartesian_control {
namespace core {

/**
 * @brief Represents task-space (Cartesian) pose error in 3D Euclidean space and SO(3).
 *
 * In robotic manipulation, task-space error is naturally split into:
 * 1. Translational error (R^3): Linear displacement vector between target and current end-effector positions.
 * 2. Rotational error (so(3)): 3D rotation vector (axis * angle) representing the geodesic rotational displacement.
 */
struct CartesianError {
    Eigen::Vector3d linear;   ///< Linear position error in base frame: e_p = p_target - p_current (meters)
    Eigen::Vector3d angular;  ///< Angular orientation error in base frame: e_o = 2 * logMap(q_target * q_current^-1) (radians)
};

/**
 * @brief Logarithmic map from the unit quaternion representation of SO(3) to the Lie algebra so(3) (R^3).
 *
 * @details
 * Mathematical background:
 * Any unit quaternion q = [w, v] = [cos(theta / 2), u * sin(theta / 2)] describes a 3D rotation by angle theta
 * around unit axis u. The logarithmic map extracts the rotation vector (axis * angle) r = theta * u:
 *
 *   theta = 2 * arccos(clamp(w, -1.0, 1.0))
 *   u = v / ||v||  (if ||v|| > epsilon)
 *   logMap(q) = (theta / 2) * u = (arccos(w) / sin(arccos(w))) * v
 *
 * Note: Here logMap(q) computes (theta / 2) * u. When computing the physical angular error / twist,
 * multiplying by 2 yields the full rotation vector theta * u.
 *
 * @param q Unit quaternion representing orientation.
 * @return Eigen::Vector3d Half-angle rotation vector (theta / 2) * u in R^3.
 */
inline Eigen::Vector3d logMap(const Eigen::Quaterniond& q) {
    Eigen::Vector3d v(q.x(), q.y(), q.z());
    double vnorm = v.norm();

    // Guard against singularity / division by zero near the identity rotation (theta -> 0).
    if (vnorm < 1e-8) {
        return Eigen::Vector3d::Zero();
    }

    // Clamp the scalar part to [-1, 1] to prevent NaN resulting from numerical inaccuracies in acos.
    double w = std::max(-1.0, std::min(1.0, q.w()));
    double angle = std::acos(w);  // angle = theta / 2

    // Return (theta / 2) * u
    return angle * (v / vnorm);
}

/**
 * @brief Computes Cartesian pose error (translation + orientation) between current and target poses.
 *
 * @details
 * 1. Linear Error:
 *    Computed via standard vector difference in the base frame:
 *    e_p = p_target - p_current
 *
 * 2. Angular Error (Geodesic on SO(3)):
 *    The relative rotation from current to target is:
 *    q_error = q_target * q_current^-1
 *
 *    Double Cover & Shortest Path:
 *    Unit quaternions form a double cover of SO(3), meaning q and -q represent the exact same physical rotation.
 *    If the quaternion dot product (q_current . q_target) < 0, they lie on opposite hemispheres of S^3.
 *    Directly computing q_target * q_current^-1 would yield a 360-theta degree rotation instead of theta.
 *    To ensure the shortest angular path, we flip the sign of q_target if (q_current . q_target) < 0.
 *
 *    Full rotation error vector:
 *    e_o = 2 * logMap(q_error) = theta * u
 *
 * @param current_pos End-effector position [x, y, z] in base frame.
 * @param current_quat End-effector orientation quaternion in base frame.
 * @param target_pos Desired end-effector position [x, y, z] in base frame.
 * @param target_quat Desired end-effector orientation quaternion in base frame.
 * @return CartesianError Struct containing 3D linear error and 3D angular error.
 */
inline CartesianError computePoseError(const Eigen::Vector3d& current_pos,
                                        const Eigen::Quaterniond& current_quat,
                                        const Eigen::Vector3d& target_pos,
                                        const Eigen::Quaterniond& target_quat) {
    CartesianError err;

    // Linear translation error (meters)
    err.linear = target_pos - current_pos;

    // Normalize quaternions defensively to avoid numerical drift accumulation
    Eigen::Quaterniond q_cur = current_quat.normalized();
    Eigen::Quaterniond q_tgt = target_quat.normalized();

    // Shortest-path hemisphere alignment: flip sign if in opposite hemisphere
    if (q_cur.coeffs().dot(q_tgt.coeffs()) < 0.0) {
        q_tgt.coeffs() = -q_tgt.coeffs();
    }

    // Relative orientation displacement: q_err = q_tgt * q_cur^-1
    Eigen::Quaterniond q_err = q_tgt * q_cur.conjugate();

    // Convert relative quaternion to angular error vector e_o in radians: e_o = 2 * (theta / 2) * u = theta * u
    err.angular = 2.0 * logMap(q_err);

    return err;
}

}  // namespace core
}  // namespace franka_cartesian_control