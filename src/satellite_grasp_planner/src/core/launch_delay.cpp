#include "satellite_grasp_planner/core/launch_delay.hpp"

#include <cmath>
#include <stdexcept>

#include "haptic_dmp_learning/core/math_utils.hpp"

namespace satellite_grasp_planner {
namespace core {

double launchDelay(double theta_now_rad, double theta_star_rad, double omega_rad_s,
                   double tau_contact_s, double min_delay_s) {
    if (omega_rad_s == 0.0) throw std::invalid_argument("launchDelay: omega must be non-zero.");
    const double T = 2.0 * haptic_dmp_learning::core::kPi / std::abs(omega_rad_s);
    const double t = (theta_star_rad - theta_now_rad) / omega_rad_s - tau_contact_s;
    double r = std::fmod(t - min_delay_s, T);
    if (r < 0.0) r += T;
    // Round-off can leave r a hair below T (e.g. t - min_delay = -1e-15): that is the same instant
    // as 0, and the result must stay in [min_delay, min_delay + T).
    if (r >= T - 1e-9) r = 0.0;
    return min_delay_s + r;
}

}  // namespace core
}  // namespace satellite_grasp_planner
