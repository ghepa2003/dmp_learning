#pragma once

/**
 * @file launch_plan.hpp
 * @brief Pure functions to plan the launch time from a satellite measurement. No ProDMP, no RobotModel.
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
