#pragma once

/**
 * @file grasp_cost.hpp
 * @brief Pure functions for the grasp-candidate cost (SI units: m, m/s, rad). No ProDMP, no
 *        RobotModel, no ROS: raw terms come from the caller, this module only builds the satellite
 *        surface velocity and combines normalized terms into a scalar to MINIMIZE.
 */

#include <stdexcept>

#include <Eigen/Dense>

#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/grasp_roll.hpp"

namespace haptic_dmp_learning {
namespace core {

/// Velocity of grasp point k on the spinning satellite: (omega * axis_unit) x (p_k(theta) - center).
/// omega is SIGNED: positive = counter-clockwise seen from +axis. The real case is clockwise, so
/// omega is negative there.
inline Eigen::Vector3d surfaceVelocity(const CubeSatelliteModel& satellite, GraspPointId k,
                                       double theta_rad, double omega_rad_s) {
    const Eigen::Vector3d r = satellite.graspPoseAt(k, theta_rad).position_world -
                              satellite.cubeCenterWorld();
    return (omega_rad_s * satellite.axisWorld()).cross(r);
}

/// Mean over the 4 grasp points of |v|^2. Independent of theta (rigid rotation about the axis
/// preserves distances from it), so it is evaluated at theta = 0.
inline double meanSurfaceSpeedSquared(const CubeSatelliteModel& satellite, double omega_rad_s) {
    double sum = 0.0;
    for (GraspPointId k : {GraspPointId::kP0, GraspPointId::kP90, GraspPointId::kP180,
                           GraspPointId::kP270}) {
        sum += surfaceVelocity(satellite, k, 0.0, omega_rad_s).squaredNorm();
    }
    return sum / 4.0;
}

/// ||v_ee - v_target||^2.
inline double velocityTerm(const Eigen::Vector3d& v_ee, const Eigen::Vector3d& v_target) {
    return (v_ee - v_target).squaredNorm();
}

/// ||delta_g - delta_g_demo||^2.
inline double goalTerm(const Eigen::Vector3d& delta_g, const Eigen::Vector3d& delta_g_demo) {
    return (delta_g - delta_g_demo).squaredNorm();
}

/// Cost weights. PROVISIONAL, to be recalibrated.
struct GraspCostWeights {
    double w_v = 1.0;
    double w_g = 0.15;
    double w_m = 0.25;
    double w_psi = 0.15;

    /// @throws std::invalid_argument if any weight is negative (or NaN).
    void validate() const {
        if (!(w_v >= 0.0 && w_g >= 0.0 && w_m >= 0.0 && w_psi >= 0.0)) {
            throw std::invalid_argument("GraspCostWeights: all weights must be >= 0.");
        }
    }
};

/// Normalization references, all strictly positive.
struct GraspCostReferences {
    double e_v_ref = 1.0;
    double e_g_ref = 1.0;
    double w_trans_demo = 1.0;

    /// @throws std::invalid_argument if any reference is <= 0 (or NaN).
    void validate() const {
        if (!(e_v_ref > 0.0 && e_g_ref > 0.0 && w_trans_demo > 0.0)) {
            throw std::invalid_argument("GraspCostReferences: all references must be > 0.");
        }
    }
};

/// Raw (unnormalized) cost terms of one candidate.
struct GraspCostTerms {
    double e_v = 0.0;
    double e_g = 0.0;
    double w_trans = 0.0;
    double psi_rad = 0.0;
};

struct GraspCostBreakdown {
    double e_v_hat = 0.0;
    double e_g_hat = 0.0;
    double w_hat = 0.0;
    double p_psi = 0.0;
    double total = 0.0;
};

/// total = w_v*e_v_hat + w_g*e_g_hat - w_m*w_hat + w_psi*p_psi, to be MINIMIZED. The MINUS on the
/// manipulability term is intentional (higher manipulability lowers the cost).
/// @throws std::invalid_argument on invalid references, weights or psi_tol_rad <= 0.
inline GraspCostBreakdown combineCost(const GraspCostTerms& t, const GraspCostReferences& ref,
                                      const GraspCostWeights& w, double psi_tol_rad) {
    ref.validate();
    w.validate();
    GraspCostBreakdown b;
    b.e_v_hat = t.e_v / ref.e_v_ref;
    b.e_g_hat = t.e_g / ref.e_g_ref;
    b.w_hat = t.w_trans / ref.w_trans_demo;
    b.p_psi = rollPenalty(t.psi_rad, psi_tol_rad);
    b.total = w.w_v * b.e_v_hat + w.w_g * b.e_g_hat - w.w_m * b.w_hat + w.w_psi * b.p_psi;
    return b;
}

}  // namespace core
}  // namespace haptic_dmp_learning
