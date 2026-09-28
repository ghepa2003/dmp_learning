// Pure tests for launch_plan.hpp (no ProDMP, no RobotModel).

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>

#include "haptic_dmp_learning/core/math_utils.hpp"
#include "satellite_grasp_planner/core/launch_plan.hpp"

using namespace satellite_grasp_planner::core;
using haptic_dmp_learning::core::degToRad;
using haptic_dmp_learning::core::wrapPi;

namespace {
SatelliteSnapshot snap(double omega_deg_s, double theta_deg = 0.0, double t_s = 0.0) {
    SatelliteSnapshot s;
    s.omega_rad_s = degToRad(omega_deg_s);
    s.theta_rad = degToRad(theta_deg);
    s.t_s = t_s;
    return s;
}
}  // namespace

TEST(LaunchPlanPhase, KnownValuesAndWrap) {
    const SatelliteSnapshot s = snap(-2.0);
    EXPECT_NEAR(phaseAt(s, 0.0), 0.0, 1e-12);
    EXPECT_NEAR(phaseAt(s, 10.0), degToRad(-20.0), 1e-12);
    EXPECT_NEAR(phaseAt(s, 45.0), degToRad(-90.0), 1e-12);
    EXPECT_NEAR(phaseAt(s, 200.0), degToRad(-40.0), 1e-12);  // -400 deg wraps to -40
}

// theta_now(t=10) = -20 deg; delay = (-90+20)/(-2) - 30 = 35 - 30 = 5 s.
TEST(LaunchPlanPlan, BaseCase) {
    const LaunchPlan p = planLaunch(snap(-2.0), 10.0, degToRad(-90.0), 30.0);
    EXPECT_NEAR(p.delay_s, 5.0, 1e-9);
    EXPECT_NEAR(p.launch_time_s, 15.0, 1e-9);
    EXPECT_NEAR(p.contact_time_s, 45.0, 1e-9);
    EXPECT_NEAR(p.phase_at_contact_rad, degToRad(-90.0), 1e-9);
}

// omega = -2.2 deg/s, phase -20 deg at t_s = 10, now = 10: delay = 70/2.2 - 30 = 1.8182 s;
// phase at contact = -20 - 2.2*(41.8182 - 10) = -90 deg.
TEST(LaunchPlanPlan, FasterRotation) {
    const LaunchPlan p = planLaunch(snap(-2.2, -20.0, 10.0), 10.0, degToRad(-90.0), 30.0);
    EXPECT_NEAR(p.delay_s, 1.8182, 1e-3);
    EXPECT_NEAR(p.phase_at_contact_rad, degToRad(-90.0), 1e-9);
}

// Hand computation: T = 180 s, t = 5 s, (5 - 10) mod 180 = 175, delay = 10 + 175 = 185 s;
// launch 10 + 185 = 195, contact 225, phase = -2*225 = -450 -> -90 deg.
TEST(LaunchPlanPlan, MinDelayWrapsOnePeriod) {
    const LaunchPlan p = planLaunch(snap(-2.0), 10.0, degToRad(-90.0), 30.0, 10.0);
    EXPECT_NEAR(p.delay_s, 185.0, 1e-9);
    EXPECT_NEAR(p.launch_time_s, 195.0, 1e-9);
    EXPECT_NEAR(p.contact_time_s, 225.0, 1e-9);
    EXPECT_NEAR(p.phase_at_contact_rad, degToRad(-90.0), 1e-9);
}

TEST(LaunchPlanPlan, ContactPhaseAlwaysMatchesTarget) {
    const double omegas[] = {-2.0, -2.2, 1.5, -0.7};
    const double stars[] = {-170.0, -90.0, 0.0, 60.0, 175.0};
    int n = 0;
    for (double om : omegas) {
        for (double st : stars) {
            const double t_now = 3.0 + 7.0 * n;
            const double tau = 5.0 + 4.0 * (n % 5);
            const double md = (n % 2) ? 0.0 : 2.5;
            const SatelliteSnapshot s = snap(om, 30.0, 1.0);
            const LaunchPlan p = planLaunch(s, t_now, degToRad(st), tau, md);
            EXPECT_NEAR(wrapPi(p.phase_at_contact_rad - degToRad(st)), 0.0, 1e-9)
                << "omega=" << om << " star=" << st;
            const double T = 2.0 * M_PI / std::abs(s.omega_rad_s);
            EXPECT_GE(p.delay_s, md);
            EXPECT_LT(p.delay_s, md + T);
            ++n;
        }
    }
    EXPECT_EQ(n, 20);
}

TEST(LaunchPlanReselection, Flags) {
    const SatelliteSnapshot base = snap(-2.0);
    EXPECT_FALSE(needsReselection(base, base).needed);

    SatelliteSnapshot l = snap(-2.2);
    ReselectionCheck c = needsReselection(base, l);
    EXPECT_TRUE(c.omega_changed);
    EXPECT_NEAR(c.rel_omega_change, 0.1, 1e-12);
    EXPECT_TRUE(c.needed);
    EXPECT_FALSE(c.axis_changed);
    EXPECT_FALSE(c.center_changed);

    l = base;
    l.axis = Eigen::AngleAxisd(degToRad(2.0), Eigen::Vector3d::UnitX()) * base.axis;
    c = needsReselection(base, l);
    EXPECT_TRUE(c.axis_changed);
    EXPECT_NEAR(c.axis_angle_rad, degToRad(2.0), 1e-12);
    EXPECT_TRUE(c.needed);

    l = base;
    l.center = Eigen::Vector3d(0.02, 0.0, 0.0);
    c = needsReselection(base, l);
    EXPECT_TRUE(c.center_changed);
    EXPECT_NEAR(c.center_shift_m, 0.02, 1e-15);
    EXPECT_TRUE(c.needed);

    c = needsReselection(base, snap(2.0));
    EXPECT_TRUE(c.omega_changed);  // sign change
    EXPECT_TRUE(c.needed);
}

TEST(LaunchPlanThrows, InvalidInputs) {
    EXPECT_THROW(needsReselection(snap(0.0), snap(-2.0)), std::invalid_argument);
    SatelliteSnapshot zero_axis = snap(-2.0);
    zero_axis.axis = Eigen::Vector3d::Zero();
    EXPECT_THROW(needsReselection(snap(-2.0), zero_axis), std::invalid_argument);
    EXPECT_THROW(needsReselection(zero_axis, snap(-2.0)), std::invalid_argument);
    EXPECT_THROW(planLaunch(snap(0.0), 0.0, 0.0, 30.0), std::invalid_argument);
    EXPECT_THROW(planLaunch(snap(-2.0), 0.0, 0.0, 0.0), std::invalid_argument);
    EXPECT_THROW(planLaunch(snap(-2.0), 0.0, 0.0, -1.0), std::invalid_argument);
}
