#pragma once

/**
 * @file candidate_scan.hpp
 * @brief Offline scan of grasp candidates (grasp point k x satellite phase theta): for each one the
 *        ProDMP end-effector velocity at the contact instant is compared with the satellite
 *        surface velocity, plus the goal-displacement term. NO IK, NO RobotModel, NO collision.
 */

#include <vector>

#include <Eigen/Dense>

#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/grasp_cost.hpp"
#include "haptic_dmp_learning/core/math_utils.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "satellite_grasp_planner/core/launch_delay.hpp"  // launchDelay: moved out, re-included for existing users
#include "satellite_grasp_planner/core/window_center.hpp"  // windowCenterTheta: moved out, re-included for existing users

namespace satellite_grasp_planner {
namespace core {

struct ScanParams {
    double theta_center_rad = 0.0;
    double half_window_rad = M_PI / 2.0;
    double step_rad = 5.0 * M_PI / 180.0;
    double omega_rad_s = 0.0;       ///< signed, clockwise = negative
    double tau_contact_s = 0.0;     ///< contact time; <= 0 means "use the template's tau"
    Eigen::Vector3d p0 = Eigen::Vector3d::Zero();            ///< EE position at launch
    Eigen::Vector3d delta_g_demo = Eigen::Vector3d::Zero();  ///< demo displacement vector
    /// (end of demo) - (point at contact); goal_param = contact_point + this offset.
    Eigen::Vector3d contact_to_end_offset = Eigen::Vector3d::Zero();
    haptic_dmp_learning::core::GraspCostWeights weights;
    double e_v_ref = 1.0;  ///< e.g. meanSurfaceSpeedSquared(model, omega)
    double e_g_ref = 1.0;
};

struct CandidateRow {
    haptic_dmp_learning::core::GraspPointId k = haptic_dmp_learning::core::GraspPointId::kP0;
    double theta_rad = 0.0;
    Eigen::Vector3d contact_point = Eigen::Vector3d::Zero();
    Eigen::Vector3d goal_param = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_ee = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_target = Eigen::Vector3d::Zero();
    double e_v = 0.0;
    double e_g = 0.0;
    double e_v_hat = 0.0;
    double e_g_hat = 0.0;
    double partial_cost = 0.0;  ///< w_v*e_v_hat + w_g*e_g_hat (no manipulability / roll terms)
};

/// Scans theta in [center - half, center + half] (both ends included, step_rad) x the 4 grasp
/// points. The template is copied (never modified): the copy is re-anchored at p0 with a support
/// goal p0 + delta_g_demo, stepped with dt = 0.005 s until t >= tau_contact_s, and setGoal() is
/// never called again; each candidate only uses velocityForCandidateGoal().
/// @throws std::invalid_argument for step_rad <= 0, half_window_rad <= 0, tau_contact_s > template
///         tau, e_v_ref <= 0 or e_g_ref <= 0 (weights are validated too).
std::vector<CandidateRow> scanCandidates(
    const haptic_dmp_learning::core::CubeSatelliteModel& model,
    const haptic_dmp_learning::core::ProDMP& prodmp_template, const ScanParams& params);

/// Rows sorted by increasing partial_cost (stable).
std::vector<CandidateRow> sortedByPartialCost(std::vector<CandidateRow> rows);

/// For each grasp point (kP0..kP270 order) the n_per_point rows of lowest partial_cost, ascending
/// (fewer if the point has fewer rows). @throws std::invalid_argument if n_per_point <= 0.
std::vector<CandidateRow> bestPerPoint(const std::vector<CandidateRow>& rows, int n_per_point);

}  // namespace core
}  // namespace satellite_grasp_planner
