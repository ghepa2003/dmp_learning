#include "satellite_grasp_planner/core/launch_plan.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "haptic_dmp_learning/core/math_utils.hpp"
#include "satellite_grasp_planner/core/launch_delay.hpp"

namespace satellite_grasp_planner {
namespace core {

double phaseAt(const SatelliteSnapshot& snapshot, double t_s) {
    return haptic_dmp_learning::core::wrapPi(snapshot.theta_rad +
                                             snapshot.omega_rad_s * (t_s - snapshot.t_s));
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
