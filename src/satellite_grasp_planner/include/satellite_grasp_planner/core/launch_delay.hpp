#pragma once

/**
 * @file launch_delay.hpp
 * @brief Delay before launch so that the contact happens at a given satellite phase. Pure, no ROS.
 */

namespace satellite_grasp_planner {
namespace core {

/// Delay [s] before launch so that the contact happens at phase theta_star:
/// t = (theta_star - theta_now)/omega - tau_contact, omega SIGNED (clockwise rotation: omega < 0
/// and theta decreases in time), reduced modulo the period T = 2*pi/|omega| into
/// [min_delay_s, min_delay_s + T). @throws std::invalid_argument if omega == 0.
double launchDelay(double theta_now_rad, double theta_star_rad, double omega_rad_s,
                   double tau_contact_s, double min_delay_s = 0.0);

}  // namespace core
}  // namespace satellite_grasp_planner
