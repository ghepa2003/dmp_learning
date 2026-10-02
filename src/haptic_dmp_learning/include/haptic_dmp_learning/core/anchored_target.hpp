#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace haptic_dmp_learning {
namespace core {
namespace anchored_target {

// ---------------------------------------------------------------------------
// Anchored /target_pose: the master motion is re-expressed relative to its own
// first sample and applied on top of a fixed anchor pose.
//
//   p_pub = anchor_p + (p_corr - p_corr0)
//   q_pub = normalize(anchor_q (x) (q_corr0^-1 (x) q_corr))
//
// p_corr / q_corr are the master pose ALREADY corrected with R_z
// (frame_correction.hpp); p_corr0 / q_corr0 are those of the first sample.
// The rotation increment q_corr0^-1 (x) q_corr is expressed in the body frame,
// so a fixed left offset Q (q_corr = Q (x) q_raw) cancels out.
// ---------------------------------------------------------------------------

/// Tolerance on |q| = 1 for the anchor quaternion.
inline constexpr double kUnitNormTolerance = 1e-6;

/// Anchor position; throws std::invalid_argument naming `param_name` unless it has 3 entries.
inline Eigen::Vector3d parseAnchorPosition(const std::vector<double>& v,
                                           const std::string& param_name) {
    if (v.size() != 3) {
        throw std::invalid_argument(param_name + " must have exactly 3 elements (x, y, z), got " +
                                    std::to_string(v.size()));
    }
    return Eigen::Vector3d(v[0], v[1], v[2]);
}

/// Anchor orientation [x, y, z, w]; throws std::invalid_argument naming `param_name` unless it has
/// 4 finite entries with |q| = 1 +- kUnitNormTolerance. Returned as given (not renormalized).
inline Eigen::Quaterniond parseAnchorOrientation(const std::vector<double>& v,
                                                 const std::string& param_name) {
    if (v.size() != 4) {
        throw std::invalid_argument(param_name + " must have exactly 4 elements (x, y, z, w), got " +
                                    std::to_string(v.size()));
    }
    const double norm = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]);
    if (!std::isfinite(norm) || std::abs(norm - 1.0) > kUnitNormTolerance) {
        throw std::invalid_argument(param_name + " must be a unit quaternion (|q| = 1 +- 1e-6), got |q| = " +
                                    std::to_string(norm));
    }
    return Eigen::Quaterniond(v[3], v[0], v[1], v[2]);
}

/// Per-recording state: first-sample reference and previous output (for sign continuity).
struct State {
    bool has_reference = false;
    Eigen::Vector3d p_corr0 = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_corr0 = Eigen::Quaterniond::Identity();
    bool has_last = false;
    Eigen::Quaterniond q_pub_last = Eigen::Quaterniond::Identity();
};

struct Pose {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

/// One step. The first call on a fresh State stores the reference and returns exactly the anchor.
/// Throws std::invalid_argument if anchor_q or q_corr is not unit (|q| = 1 +- kUnitNormTolerance).
inline Pose computeAnchoredTarget(State& state,
                                  const Eigen::Vector3d& anchor_p,
                                  const Eigen::Quaterniond& anchor_q,
                                  const Eigen::Vector3d& p_corr,
                                  const Eigen::Quaterniond& q_corr) {
    if (std::abs(anchor_q.norm() - 1.0) > kUnitNormTolerance) {
        throw std::invalid_argument("anchor quaternion is not unit");
    }
    if (std::abs(q_corr.norm() - 1.0) > kUnitNormTolerance) {
        throw std::invalid_argument("master quaternion is not unit");
    }

    if (!state.has_reference) {
        state.has_reference = true;
        state.p_corr0 = p_corr;
        state.q_corr0 = q_corr;
    }

    Pose out;
    out.position = anchor_p + (p_corr - state.p_corr0);

    Eigen::Quaterniond q = anchor_q * (state.q_corr0.inverse() * q_corr);
    q.normalize();
    if (!state.has_last) {
        // First sample: relative rotation is the identity, the result is the anchor itself.
        q = anchor_q;
    } else if (state.q_pub_last.dot(q) < 0.0) {
        q.coeffs() = -q.coeffs();
    }
    state.q_pub_last = q;
    state.has_last = true;
    out.orientation = q;
    return out;
}

}  // namespace anchored_target
}  // namespace core
}  // namespace haptic_dmp_learning
