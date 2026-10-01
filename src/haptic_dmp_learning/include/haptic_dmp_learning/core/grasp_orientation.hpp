#pragma once

/**
 * @file grasp_orientation.hpp
 * @brief Pure logic (Eigen + STL only, NO ROS) of the rollout orientation of prodmp_gazebo_executor_node when
 *        grasp_goal_source="grasp_command": parameter grasp_orientation_mode, validation of the goal quaternion
 *        of a GraspCommand, and the SLERP profile the planner validated.
 *
 * "planner_goal": q(t) = slerpShortestPath(q_start, q_goal, s(t)) with q_start = hold quaternion (read from TF),
 * q_goal = GraspCommand.goal_pose.orientation (psi is already applied by the planner: never re-applied here).
 *
 * s(t) is the SAME definition as the planner's kinematic feasibility rollout
 * (satellite_grasp_planner/src/core/kinematic_feasibility.cpp):
 *     s = std::min(1.0, t / tau)       with t = elapsed time of the rollout
 * a function of TIME (not of the ProDMP phase). In the planner sample i has t = (i + 1) * dt; in the executor the
 * sample published at a tick has t = elapsed (now - t_roll0; 0 on the first tick). Same SLERP function
 * (core::slerpShortestPath, math_utils.hpp), which the planner already includes: no second copy exists.
 */

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include "haptic_dmp_learning/core/math_utils.hpp"

namespace haptic_dmp_learning {
namespace core {
namespace grasp_orientation {

enum class Mode { kPlannerGoal, kDemoNative };

/// Tolerance on |q| - 1 of the message's goal quaternion.
inline constexpr double kGoalQuaternionNormTol = 1e-6;
/// Tolerance on |z_tcp . axis| (the approach axis of the goal must be orthogonal to the satellite rotation axis).
inline constexpr double kApproachAxisDotTol = 1e-3;

inline const char* modeName(Mode m) { return m == Mode::kPlannerGoal ? "planner_goal" : "demo_native"; }

/// Parses the MANDATORY parameter grasp_orientation_mode (no default).
/// @throws std::invalid_argument naming the parameter if it is missing or not one of the two values.
inline Mode parseMode(const std::optional<std::string>& value) {
    if (!value) {
        throw std::invalid_argument(
            "grasp_goal_source='grasp_command': mandatory parameter 'grasp_orientation_mode' was not provided "
            "(must be 'planner_goal' or 'demo_native')");
    }
    if (*value == "planner_goal") return Mode::kPlannerGoal;
    if (*value == "demo_native") return Mode::kDemoNative;
    throw std::invalid_argument("parameter 'grasp_orientation_mode' must be 'planner_goal' or 'demo_native', got '" +
                                *value + "'");
}

/// Angle [rad] of the rotation between two orientations, invariant to q / -q. Both must have non-zero finite norm.
inline double quatAngleRad(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
    const double na = a.norm(), nb = b.norm();
    if (!(std::isfinite(na) && na > 0.0 && std::isfinite(nb) && nb > 0.0)) {
        throw std::invalid_argument("quatAngleRad: quaternion with zero or non-finite norm");
    }
    double d = std::fabs(a.coeffs().dot(b.coeffs())) / (na * nb);
    if (d > 1.0) d = 1.0;
    return 2.0 * std::acos(d);
}

/// Coherence of the goal quaternion of a GraspCommand.
/// @throws std::runtime_error ("GraspCommand rejected: ...") if a component is not finite, |norm - 1| >
///         kGoalQuaternionNormTol, or |z_tcp . axis| >= kApproachAxisDotTol where z_tcp is the third column of
///         R(q) (the approach axis) and axis the satellite rotation axis (normalized here).
inline void checkGoalOrientation(const Eigen::Quaterniond& q, const Eigen::Vector3d& satellite_axis) {
    if (!q.coeffs().allFinite()) {
        throw std::runtime_error("GraspCommand rejected: goal_pose.orientation has a non-finite component");
    }
    const double n = q.norm();
    if (!(std::fabs(n - 1.0) <= kGoalQuaternionNormTol)) {
        std::ostringstream m;
        m << "GraspCommand rejected: goal_pose.orientation norm " << n << " is not 1 (tol " << kGoalQuaternionNormTol << ")";
        throw std::runtime_error(m.str());
    }
    if (!(satellite_axis.allFinite() && satellite_axis.norm() > 1e-9)) {
        throw std::invalid_argument("checkGoalOrientation: satellite axis is zero or not finite");
    }
    const Eigen::Vector3d z_tcp = q.normalized().toRotationMatrix().col(2);
    const double dot = z_tcp.dot(satellite_axis.normalized());
    if (!(std::fabs(dot) < kApproachAxisDotTol)) {
        std::ostringstream m;
        m << "GraspCommand rejected: the approach axis of goal_pose.orientation (z_tcp = [" << z_tcp.transpose()
          << "]) is not orthogonal to the satellite rotation axis: |z_tcp . axis| = " << std::fabs(dot) << " (limit "
          << kApproachAxisDotTol << ")";
        throw std::runtime_error(m.str());
    }
}

/// q1 with the sign chosen so that q0 . q1 >= 0 (the hemisphere slerpShortestPath uses).
inline Eigen::Quaterniond alignShortestPath(const Eigen::Quaterniond& q0, const Eigen::Quaterniond& q1) {
    Eigen::Quaterniond t = q1;
    if (q0.coeffs().dot(t.coeffs()) < 0.0) t.coeffs() = -t.coeffs();
    return t;
}

/// The planner's s(t): min(1, t / tau) (kinematic_feasibility.cpp), additionally floored at 0.
/// @throws std::invalid_argument if tau is not finite and > 0 or t is not finite.
inline double slerpFraction(double t_s, double tau_s) {
    if (!(std::isfinite(tau_s) && tau_s > 0.0) || !std::isfinite(t_s)) {
        throw std::invalid_argument("slerpFraction: tau must be finite and > 0, t finite");
    }
    return std::max(0.0, std::min(1.0, t_s / tau_s));
}

/// Commanded orientation at rollout time @p t_s: slerpShortestPath(q_start, q_goal, slerpFraction(t_s, tau)).
inline Eigen::Quaterniond slerpAt(const Eigen::Quaterniond& q_start, const Eigen::Quaterniond& q_goal, double t_s,
                                  double tau_s) {
    return slerpShortestPath(q_start, q_goal, slerpFraction(t_s, tau_s));
}

/// Constant angular velocity [rad/s], expressed in the frame of the quaternions (world), of the SLERP profile:
/// q(s) = q_start * exp(s * phi) with phi = log(q_start^-1 * q_goal) (shortest path), s = t / tau, hence
/// omega = R(q_start) * phi / tau (R(q_start) * phi is constant along the path). Zero if the two orientations
/// coincide. @p q_goal_aligned must be in the hemisphere of q_start (alignShortestPath()).
inline Eigen::Vector3d slerpAngularVelocityWorld(const Eigen::Quaterniond& q_start,
                                                 const Eigen::Quaterniond& q_goal_aligned, double tau_s) {
    if (!(std::isfinite(tau_s) && tau_s > 0.0)) {
        throw std::invalid_argument("slerpAngularVelocityWorld: tau must be finite and > 0");
    }
    const Eigen::Quaterniond qs = q_start.normalized();
    Eigen::Quaterniond d = (qs.conjugate() * q_goal_aligned.normalized()).normalized();
    if (d.w() < 0.0) d.coeffs() = -d.coeffs();
    const double vn = d.vec().norm();
    if (vn < 1e-12) return Eigen::Vector3d::Zero();
    const double angle = 2.0 * std::atan2(vn, d.w());
    const Eigen::Vector3d phi_body = angle * d.vec() / vn;
    return qs.toRotationMatrix() * phi_body / tau_s;
}

}  // namespace grasp_orientation
}  // namespace core
}  // namespace haptic_dmp_learning
