#pragma once

/**
 * @file launch_plan.hpp
 * @brief Pure functions to plan the launch time from a satellite measurement and to decide whether
 *        the grasp selection must be redone. No ProDMP, no RobotModel.
 */

#include <Eigen/Dense>

namespace satellite_grasp_planner {
namespace core {

/// Satellite state measured at instant t_s. Conventions: theta_rad is the phase in the convention
/// of CubeSatelliteModel (theta = 0 is the body in the configuration given by its Params);
/// omega_rad_s is SIGNED (counter-clockwise seen from +axis positive, clockwise negative, so theta
/// decreases in time when omega < 0); t_s is the instant of the measurement, on the same clock as
/// every other time in this header.
struct SatelliteSnapshot {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    double omega_rad_s = 0.0;
    double theta_rad = 0.0;
    double t_s = 0.0;
};

/// Phase at time t_s assuming constant omega: theta + omega*(t_s - snapshot.t_s), wrapped to (-pi, pi].
double phaseAt(const SatelliteSnapshot& snapshot, double t_s);

/// PROVISIONAL thresholds: they depend on the camera accuracy, which is not known.
struct ReselectionThresholds {
    double rel_omega_change = 0.05;
    double axis_angle_rad = 1.0 * 3.14159265358979323846 / 180.0;
    double center_shift_m = 0.01;
};

struct ReselectionCheck {
    bool needed = false;  ///< any of the three flags below
    bool omega_changed = false;
    bool axis_changed = false;
    bool center_changed = false;
    double rel_omega_change = 0.0;
    double axis_angle_rad = 0.0;
    double center_shift_m = 0.0;
};

/// Compares the satellite state at selection time with the one at launch time.
/// rel_omega_change = |omega_l - omega_s| / |omega_s|; axis_angle between the normalized axes;
/// center_shift = distance between centers. A sign change of omega always counts as omega_changed.
/// @throws std::invalid_argument if omega_s == 0 or an axis has norm < 1e-9.
ReselectionCheck needsReselection(const SatelliteSnapshot& at_selection,
                                  const SatelliteSnapshot& at_launch,
                                  const ReselectionThresholds& thresholds = ReselectionThresholds());

struct LaunchPlan {
    double delay_s = 0.0;
    double launch_time_s = 0.0;
    double contact_time_s = 0.0;
    double phase_at_contact_rad = 0.0;
};

/// Plans the launch so that contact happens at phase theta_star: theta_now = phaseAt(at_launch,
/// t_now), delay = launchDelay(theta_now, theta_star, omega, tau_contact, min_delay), launch_time =
/// t_now + delay, contact_time = launch_time + tau_contact, phase_at_contact = phaseAt(at_launch,
/// contact_time). min_delay_s is the communication delay D plus the residual computation time;
/// nothing is updated after the launch. @throws std::invalid_argument if omega == 0 or
/// tau_contact_s <= 0.
LaunchPlan planLaunch(const SatelliteSnapshot& at_launch, double t_now_s, double theta_star_rad,
                      double tau_contact_s, double min_delay_s = 0.0);

}  // namespace core
}  // namespace satellite_grasp_planner
