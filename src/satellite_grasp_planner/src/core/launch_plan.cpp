#include "satellite_grasp_planner/core/launch_plan.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "haptic_dmp_learning/core/math_utils.hpp"
#include "satellite_grasp_planner/core/candidate_scan.hpp"

namespace satellite_grasp_planner {
namespace core {

double phaseAt(const SatelliteSnapshot& snapshot, double t_s) {
    return haptic_dmp_learning::core::wrapPi(snapshot.theta_rad +
                                             snapshot.omega_rad_s * (t_s - snapshot.t_s));
}

ReselectionCheck needsReselection(const SatelliteSnapshot& at_selection,
                                  const SatelliteSnapshot& at_launch,
                                  const ReselectionThresholds& thresholds) {
    if (at_selection.omega_rad_s == 0.0) {
        throw std::invalid_argument("needsReselection: omega at selection is zero.");
    }
    if (!(at_selection.axis.norm() >= 1e-9) || !(at_launch.axis.norm() >= 1e-9)) {
        throw std::invalid_argument("needsReselection: an axis has norm < 1e-9.");
    }
    ReselectionCheck c;
    c.rel_omega_change = std::abs(at_launch.omega_rad_s - at_selection.omega_rad_s) /
                         std::abs(at_selection.omega_rad_s);
    const double cos_a =
        std::clamp(at_selection.axis.normalized().dot(at_launch.axis.normalized()), -1.0, 1.0);
    c.axis_angle_rad = std::acos(cos_a);
    c.center_shift_m = (at_launch.center - at_selection.center).norm();

    const bool sign_flip = (at_selection.omega_rad_s > 0.0) != (at_launch.omega_rad_s > 0.0);
    c.omega_changed = sign_flip || c.rel_omega_change > thresholds.rel_omega_change;
    c.axis_changed = c.axis_angle_rad > thresholds.axis_angle_rad;
    c.center_changed = c.center_shift_m > thresholds.center_shift_m;
    c.needed = c.omega_changed || c.axis_changed || c.center_changed;
    return c;
}

LaunchPlan planLaunch(const SatelliteSnapshot& at_launch, double t_now_s, double theta_star_rad,
                      double tau_contact_s, double min_delay_s) {
    if (at_launch.omega_rad_s == 0.0) throw std::invalid_argument("planLaunch: omega is zero.");
    if (!(tau_contact_s > 0.0)) throw std::invalid_argument("planLaunch: tau_contact_s must be > 0.");

    const double theta_now = phaseAt(at_launch, t_now_s);
    LaunchPlan plan;
    plan.delay_s = launchDelay(theta_now, theta_star_rad, at_launch.omega_rad_s, tau_contact_s, min_delay_s);
    plan.launch_time_s = t_now_s + plan.delay_s;
    plan.contact_time_s = plan.launch_time_s + tau_contact_s;
    plan.phase_at_contact_rad = phaseAt(at_launch, plan.contact_time_s);
    return plan;
}

}  // namespace core
}  // namespace satellite_grasp_planner
