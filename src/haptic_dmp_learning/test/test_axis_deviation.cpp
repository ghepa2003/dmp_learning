// PhaseTracker::update(throw_on_off_axis) and AxisDeviationMonitor (pure logic, no ROS).
// See include/haptic_dmp_learning/core/satellite_intercept.hpp.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>

#include "haptic_dmp_learning/core/satellite_intercept.hpp"

using namespace haptic_dmp_learning::core::satellite_intercept;

namespace {

Eigen::Quaterniond quatAbout(const Eigen::Vector3d& axis, double theta_deg) {
    return Eigen::Quaterniond(Eigen::AngleAxisd(theta_deg * M_PI / 180.0, axis.normalized()));
}

}  // namespace

// ---------------------------------------------------------------------------------------
// PhaseTracker
// ---------------------------------------------------------------------------------------
TEST(PhaseTrackerThrowFlag, DefaultThrowsWhenOffAxis) {
    const Eigen::Vector3d axis(0.0, 0.0, 1.0);
    const Eigen::Quaterniond q = quatAbout(Eigen::Vector3d(1.0, 0.0, 0.0), 1.0) * quatAbout(axis, 10.0);
    PhaseTracker tr(axis, Eigen::Quaterniond::Identity());
    EXPECT_THROW(tr.update(q), std::runtime_error);
    PhaseTracker tr2(axis, Eigen::Quaterniond::Identity());
    EXPECT_THROW(tr2.update(q, true), std::runtime_error);
}

TEST(PhaseTrackerThrowFlag, NonThrowingReturnsTheSample) {
    const Eigen::Vector3d axis(0.0, 0.0, 1.0);
    const Eigen::Quaterniond q = quatAbout(Eigen::Vector3d(1.0, 0.0, 0.0), 1.0) * quatAbout(axis, 10.0);
    PhaseTracker tr(axis, Eigen::Quaterniond::Identity());
    PhaseSample s;
    ASSERT_NO_THROW(s = tr.update(q, false));
    EXPECT_NEAR(s.off_axis_deg, 1.0, 0.05);   // well above the 0.5 deg tolerance
    EXPECT_GT(s.off_axis_deg, 0.5);
    EXPECT_NEAR(s.theta_rad * 180.0 / M_PI, 10.0, 0.1);
    EXPECT_TRUE(tr.hasSample());
    EXPECT_DOUBLE_EQ(tr.theta(), s.theta_rad);
}

TEST(PhaseTrackerThrowFlag, NonThrowingKeepsUnwrappingAndOnAxisSamplesAreUnchanged) {
    const Eigen::Vector3d axis(0.0, 1.0, 0.0);
    PhaseTracker a(axis, Eigen::Quaterniond::Identity()), b(axis, Eigen::Quaterniond::Identity());
    for (double th : {10.0, 170.0, 190.0, 350.0}) {
        const PhaseSample sa = a.update(quatAbout(axis, th));
        const PhaseSample sb = b.update(quatAbout(axis, th), false);
        EXPECT_DOUBLE_EQ(sa.theta_rad, sb.theta_rad) << th;
        EXPECT_DOUBLE_EQ(sa.off_axis_deg, sb.off_axis_deg) << th;
    }
    EXPECT_NEAR(b.theta() * 180.0 / M_PI, 350.0, 1e-8);
}

// ---------------------------------------------------------------------------------------
// AxisDeviationMonitor
// ---------------------------------------------------------------------------------------
TEST(AxisDeviationMonitorTest, EmptyMonitor) {
    AxisDeviationMonitor m;
    EXPECT_EQ(m.samples(), 0u);
    EXPECT_DOUBLE_EQ(m.maxDeg(), 0.0);
    EXPECT_DOUBLE_EQ(m.firstAboveWarnT(), -1.0);
    EXPECT_DOUBLE_EQ(m.firstAboveTolT(), -1.0);
    EXPECT_EQ(m.countAboveWarn(), 0u);
    EXPECT_EQ(m.countAboveTol(), 0u);
    EXPECT_DOUBLE_EQ(m.warnDeg(), 0.25);
    EXPECT_DOUBLE_EQ(m.tolDeg(), 0.5);
}

TEST(AxisDeviationMonitorTest, MaxFirstInstantsAndCounts) {
    AxisDeviationMonitor m;
    m.add(1.0, 0.10);
    m.add(2.0, 0.20);
    m.add(3.0, 0.30);   // first above 0.25
    m.add(4.0, 0.60);   // first above 0.5
    m.add(5.0, 0.40);
    m.add(6.0, 0.90);   // new max
    m.add(7.0, 0.05);
    EXPECT_EQ(m.samples(), 7u);
    EXPECT_DOUBLE_EQ(m.maxDeg(), 0.90);
    EXPECT_DOUBLE_EQ(m.firstAboveWarnT(), 3.0);
    EXPECT_DOUBLE_EQ(m.firstAboveTolT(), 4.0);
    EXPECT_EQ(m.countAboveWarn(), 4u);   // 0.30, 0.60, 0.40, 0.90
    EXPECT_EQ(m.countAboveTol(), 2u);    // 0.60, 0.90
}

TEST(AxisDeviationMonitorTest, NoCrossingGivesMinusOne) {
    AxisDeviationMonitor m;
    m.add(1.0, 0.01);
    m.add(2.0, 0.24);
    EXPECT_EQ(m.samples(), 2u);
    EXPECT_DOUBLE_EQ(m.maxDeg(), 0.24);
    EXPECT_DOUBLE_EQ(m.firstAboveWarnT(), -1.0);
    EXPECT_DOUBLE_EQ(m.firstAboveTolT(), -1.0);
    EXPECT_EQ(m.countAboveWarn(), 0u);
    EXPECT_EQ(m.countAboveTol(), 0u);
}

TEST(AxisDeviationMonitorTest, OnlyWarnCrossed) {
    AxisDeviationMonitor m;
    m.add(10.0, 0.30);
    EXPECT_DOUBLE_EQ(m.firstAboveWarnT(), 10.0);
    EXPECT_DOUBLE_EQ(m.firstAboveTolT(), -1.0);
    EXPECT_EQ(m.countAboveWarn(), 1u);
    EXPECT_EQ(m.countAboveTol(), 0u);
}

TEST(AxisDeviationMonitorTest, ThresholdsAreStrict) {
    AxisDeviationMonitor m;
    m.add(1.0, 0.25);
    m.add(2.0, 0.5);
    EXPECT_EQ(m.countAboveWarn(), 1u);   // only the 0.5 sample is above 0.25
    EXPECT_EQ(m.countAboveTol(), 0u);
    EXPECT_DOUBLE_EQ(m.firstAboveWarnT(), 2.0);
}

TEST(AxisDeviationMonitorTest, NonFiniteSamplesAreIgnored) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    AxisDeviationMonitor m;
    m.add(1.0, nan);
    m.add(2.0, inf);
    m.add(3.0, -inf);
    m.add(nan, 0.9);   // non-finite time: not recorded either
    EXPECT_EQ(m.samples(), 0u);
    EXPECT_EQ(m.ignored(), 4u);
    EXPECT_DOUBLE_EQ(m.maxDeg(), 0.0);
    EXPECT_DOUBLE_EQ(m.firstAboveTolT(), -1.0);
    m.add(4.0, 0.3);
    EXPECT_EQ(m.samples(), 1u);
    EXPECT_EQ(m.ignored(), 4u);
    EXPECT_DOUBLE_EQ(m.maxDeg(), 0.3);
    EXPECT_DOUBLE_EQ(m.firstAboveWarnT(), 4.0);
}

TEST(AxisDeviationMonitorTest, CustomThresholdsAndSummary) {
    AxisDeviationMonitor m(0.1, 0.2);
    m.add(1.0, 0.15);
    m.add(2.0, 0.25);
    EXPECT_DOUBLE_EQ(m.firstAboveWarnT(), 1.0);
    EXPECT_DOUBLE_EQ(m.firstAboveTolT(), 2.0);
    const std::string s = m.summary();
    EXPECT_NE(s.find("max 0.25"), std::string::npos) << s;
    EXPECT_NE(s.find("2 samples"), std::string::npos) << s;
    EXPECT_EQ(AxisDeviationMonitor().summary(), "axis deviation: no samples");
}
