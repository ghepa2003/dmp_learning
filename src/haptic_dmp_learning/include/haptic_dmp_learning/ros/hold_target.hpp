#pragma once

/**
 * @file hold_target.hpp
 * @brief Pure helpers of the "hold target" that prodmp_gazebo_executor_node (grasp_goal_source=
 *        "grasp_command") publishes on target_pose_topic while it waits for the GraspCommand and for the
 *        trigger. Message types only (no rclcpp::Node), so it is unit-testable without rclcpp::init().
 *
 * WHY: cartesian_impedance_controller writes zero torque until its first target and, once it has one,
 * regulates to the LAST message forever (no watchdog). With no target the TCP drifts (~0.09 mm/s in
 * Gazebo); a fixed target equal to the pose read once from TF stops the drift. The pose is read ONCE
 * and republished unchanged: a target that followed the TF pose would chase the drifting TCP.
 */

#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/time.hpp>

namespace haptic_dmp_learning {
namespace ros_wrapper {
namespace hold_target {

/// Sentence every FATAL emitted while the hold is the controller's last target must contain.
inline constexpr const char* kHoldRemainsActiveNote =
    "hold target remains active in the controller (no watchdog)";

/// Upper bound of hold_target_rate_hz [Hz]: the controller runs at 1 kHz and reads the last message only.
inline constexpr double kMaxHoldRateHz = 200.0;

/// Tolerance on |q| - 1 for a quaternion coming from TF (a TF quaternion is normalized to ~1e-6).
inline constexpr double kQuaternionNormTol = 1e-3;

struct HoldPose {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

/// Builds the hold pose. Position and quaternion are kept as given (the quaternion is only
/// normalized, its sign is NOT changed).
/// @throws std::invalid_argument on a non-finite component or on |norm(q) - 1| > kQuaternionNormTol.
inline HoldPose makeHoldPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
    if (!position.allFinite()) {
        throw std::invalid_argument("hold pose: position has a non-finite component");
    }
    if (!orientation.coeffs().allFinite()) {
        throw std::invalid_argument("hold pose: quaternion has a non-finite component");
    }
    const double n = orientation.norm();
    if (!(std::fabs(n - 1.0) <= kQuaternionNormTol)) {
        throw std::invalid_argument("hold pose: quaternion norm " + std::to_string(n) + " is not 1 (tol " +
                                    std::to_string(kQuaternionNormTol) + ")");
    }
    HoldPose h;
    h.position = position;
    h.orientation = Eigen::Quaterniond(orientation.coeffs() / n);  // coeffs order x, y, z, w preserved
    return h;
}

/// The message the hold timer publishes: same fields, frame_id and stamp semantics as the rollout's
/// PoseStamped (position, then orientation w, x, y, z).
inline geometry_msgs::msg::PoseStamped toPoseStamped(const HoldPose& hold, const std::string& frame_id,
                                                     const rclcpp::Time& stamp) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.pose.position.x = hold.position.x();
    msg.pose.position.y = hold.position.y();
    msg.pose.position.z = hold.position.z();
    msg.pose.orientation.w = hold.orientation.w();
    msg.pose.orientation.x = hold.orientation.x();
    msg.pose.orientation.y = hold.orientation.y();
    msg.pose.orientation.z = hold.orientation.z();
    return msg;
}

/// Angle [deg] of the rotation between two orientations, invariant to q / -q.
/// @throws std::invalid_argument if either quaternion has zero or non-finite norm.
inline double quatAngleDeg(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
    const double na = a.norm(), nb = b.norm();
    if (!(std::isfinite(na) && na > 0.0 && std::isfinite(nb) && nb > 0.0)) {
        throw std::invalid_argument("quatAngleDeg: quaternion with zero or non-finite norm");
    }
    double d = std::fabs(a.coeffs().dot(b.coeffs())) / (na * nb);
    if (d > 1.0) d = 1.0;
    return 2.0 * std::acos(d) * 180.0 / M_PI;
}

/// Timer period [s] of hold_target_rate_hz.
/// @throws std::invalid_argument unless 0 < rate_hz <= kMaxHoldRateHz (and finite).
inline double holdPeriodS(double rate_hz) {
    if (!(std::isfinite(rate_hz) && rate_hz > 0.0 && rate_hz <= kMaxHoldRateHz)) {
        throw std::invalid_argument("parameter 'hold_target_rate_hz' must be finite and in (0, " +
                                    std::to_string(kMaxHoldRateHz) + "]");
    }
    return 1.0 / rate_hz;
}

/// @p message followed by kHoldRemainsActiveNote when @p hold_remains is true, unchanged otherwise.
inline std::string withHoldNote(const std::string& message, bool hold_remains) {
    return hold_remains ? message + " | " + kHoldRemainsActiveNote : message;
}

}  // namespace hold_target
}  // namespace ros_wrapper
}  // namespace haptic_dmp_learning
