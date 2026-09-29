// Tests for the pure (no rclcpp::init needed) startup/runtime validation helpers used by
// grasp_planner_node - see grasp_planner_checks.hpp.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "satellite_grasp_planner/ros/grasp_planner_checks.hpp"

using namespace satellite_grasp_planner::ros_wrapper;

TEST(GraspPlannerChecksTest, RobotBaseWorldZeroPasses) {
    EXPECT_NO_THROW(checkRobotBaseWorldIsZero(Eigen::Vector3d::Zero(), 1e-6));
}

TEST(GraspPlannerChecksTest, RobotBaseWorldNonZeroThrows) {
    EXPECT_THROW(checkRobotBaseWorldIsZero(Eigen::Vector3d(0.01, 0.0, 0.0), 1e-6), std::invalid_argument);
}

TEST(GraspPlannerChecksTest, CubeAxisMatchesRotationAxisPasses) {
    // Different magnitudes, same direction - only the normalized axis matters.
    EXPECT_NO_THROW(checkCubeAxisMatchesRotationAxis(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 2)));
}

TEST(GraspPlannerChecksTest, CubeAxisMismatchThrows) {
    EXPECT_THROW(checkCubeAxisMatchesRotationAxis(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 1, 0)),
                std::invalid_argument);
}

TEST(GraspPlannerChecksTest, CubeCenterMatchesRotationCenterPasses) {
    EXPECT_NO_THROW(
        checkCubeCenterMatchesRotationCenter(Eigen::Vector3d(0.75, 0.0, 0.35), Eigen::Vector3d(0.75, 0.0, 0.35)));
}

TEST(GraspPlannerChecksTest, CubeCenterMismatchThrows) {
    EXPECT_THROW(
        checkCubeCenterMatchesRotationCenter(Eigen::Vector3d(0.75, 0.0, 0.35), Eigen::Vector3d(0.0, 0.0, 0.0)),
        std::invalid_argument);
}

TEST(GraspPlannerChecksTest, QRefIdentityOnAnyAxisPasses) {
    EXPECT_NO_THROW(checkQRefOnAxis(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0, 0, 1), 0.5));
    EXPECT_NO_THROW(checkQRefOnAxis(Eigen::Quaterniond::Identity(), Eigen::Vector3d(1, 0, 0), 0.5));
}

TEST(GraspPlannerChecksTest, QRefOnAxisPasses) {
    const Eigen::Quaterniond q(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
    EXPECT_NO_THROW(checkQRefOnAxis(q, Eigen::Vector3d(0, 0, 1), 0.5));
}

TEST(GraspPlannerChecksTest, QRefOffAxisThrows) {
    const Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()));
    EXPECT_THROW(checkQRefOnAxis(q, Eigen::Vector3d(0, 0, 1), 0.5), std::invalid_argument);
}

TEST(GraspPlannerChecksTest, FirstOdomSampleValidPasses) {
    EXPECT_NO_THROW(checkFirstOdomSample(Eigen::Vector3d(0, 0, 0), "world", Eigen::Vector3d(0, 0, 0)));
}

TEST(GraspPlannerChecksTest, FirstOdomSampleWrongFrameThrows) {
    EXPECT_THROW(checkFirstOdomSample(Eigen::Vector3d(0, 0, 0), "map", Eigen::Vector3d(0, 0, 0)),
                std::runtime_error);
}

TEST(GraspPlannerChecksTest, FirstOdomSampleTooFarThrows) {
    EXPECT_THROW(checkFirstOdomSample(Eigen::Vector3d(0.1, 0, 0), "world", Eigen::Vector3d(0, 0, 0)),
                std::runtime_error);
}

TEST(GraspPlannerChecksTest, OmegaConsistencyMatchPasses) {
    EXPECT_NO_THROW(
        checkOmegaConsistency(Eigen::Vector3d(0, 0, -0.0349), Eigen::Vector3d(0, 0, 1), -0.0349, 1e-3));
}

TEST(GraspPlannerChecksTest, OmegaConsistencyMismatchThrows) {
    EXPECT_THROW(
        checkOmegaConsistency(Eigen::Vector3d(0, 0, 0.0349), Eigen::Vector3d(0, 0, 1), -0.0349, 1e-3),
        std::runtime_error);
}

// ---- SelectionOverrides ----

TEST(GraspPlannerChecksTest, SelectionOverridesUnsetKeepDefaults) {
    satellite_grasp_planner::core::SelectionParams params;
    const satellite_grasp_planner::core::SelectionParams defaults;
    applySelectionOverrides(params, SelectionOverrides{});
    EXPECT_DOUBLE_EQ(params.scan.step_rad, defaults.scan.step_rad);
    EXPECT_DOUBLE_EQ(params.scan.step_rad, 5.0 * M_PI / 180.0);
    EXPECT_EQ(params.max_rows, 40);
    EXPECT_DOUBLE_EQ(params.w_hat_upper_bound, 2.5);
    EXPECT_DOUBLE_EQ(params.psi_tol_rad, 10.0 * M_PI / 180.0);
    EXPECT_DOUBLE_EQ(params.scan.half_window_rad, defaults.scan.half_window_rad);
}

TEST(GraspPlannerChecksTest, SelectionOverridesAreApplied) {
    satellite_grasp_planner::core::SelectionParams params;
    SelectionOverrides o;
    o.scan_step_deg = 2.0;
    o.max_rows = 12;
    o.w_hat_upper_bound = 3.0;
    o.psi_tol_deg = 5.0;
    applySelectionOverrides(params, o);
    EXPECT_DOUBLE_EQ(params.scan.step_rad, 2.0 * M_PI / 180.0);
    EXPECT_EQ(params.max_rows, 12);
    EXPECT_DOUBLE_EQ(params.w_hat_upper_bound, 3.0);
    EXPECT_DOUBLE_EQ(params.psi_tol_rad, 5.0 * M_PI / 180.0);
}

TEST(GraspPlannerChecksTest, SelectionOverridesRejectInvalidValuesNamingTheParameter) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const std::vector<std::pair<std::string, SelectionOverrides>> bad = [&] {
        std::vector<std::pair<std::string, SelectionOverrides>> v;
        for (const double x : {0.0, -1.0, nan, inf}) {
            SelectionOverrides a; a.scan_step_deg = x; v.push_back({"scan_step_deg", a});
            SelectionOverrides b; b.w_hat_upper_bound = x; v.push_back({"w_hat_upper_bound", b});
            SelectionOverrides c; c.psi_tol_deg = x; v.push_back({"psi_tol_deg", c});
        }
        for (const int r : {0, -5}) {
            SelectionOverrides d; d.max_rows = r; v.push_back({"max_rows", d});
        }
        return v;
    }();
    for (const auto& [name, o] : bad) {
        satellite_grasp_planner::core::SelectionParams params;
        const auto before = params.max_rows;
        try {
            applySelectionOverrides(params, o);
            ADD_FAILURE() << "no throw for " << name;
        } catch (const std::invalid_argument& e) {
            EXPECT_NE(std::string(e.what()).find(name), std::string::npos) << e.what();
        }
        EXPECT_EQ(params.max_rows, before);  // untouched on failure
        EXPECT_THROW(validateSelectionOverrides(o), std::invalid_argument);
    }
}

TEST(GraspPlannerChecksTest, DescribeSelectionParamsMarksDefaultsAndOverrides) {
    satellite_grasp_planner::core::SelectionParams params;
    SelectionOverrides o;
    o.max_rows = 12;
    applySelectionOverrides(params, o);
    const std::string d = describeSelectionParams(params, o);
    EXPECT_NE(d.find("max_rows=12(param)"), std::string::npos) << d;
    EXPECT_NE(d.find("scan_step_deg=5(default)"), std::string::npos) << d;
    EXPECT_NE(d.find("w_hat_upper_bound=2.5(default)"), std::string::npos) << d;
    EXPECT_NE(d.find("psi_tol_deg=10(default)"), std::string::npos) << d;
}
