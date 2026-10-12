// Pure tests for windowCenterTheta (no ROS). See core/window_center.hpp.

#include <gtest/gtest.h>

#include <stdexcept>

#include <Eigen/Dense>

#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/math_utils.hpp"
#include "satellite_grasp_planner/core/window_center.hpp"
#include "test_fixtures.hpp"

using satellite_grasp_planner::core::windowCenterTheta;
using haptic_dmp_learning::core::CubeSatelliteModel;
using haptic_dmp_learning::core::radToDeg;
using satellite_grasp_planner::test_fixtures::makeModelBParams;

TEST(WindowCenter, CenterThetaKnownCases) {
    {
        auto p = makeModelBParams();
        EXPECT_NEAR(windowCenterTheta(CubeSatelliteModel(p), Eigen::Vector3d::Zero()), 0.0, 1e-12);
    }
    {
        CubeSatelliteModel::Params p;
        p.center_world = Eigen::Vector3d(0.3928, -0.165, 0.1188);
        p.axis_world = Eigen::Vector3d(0.0, 1.0, 0.0);
        p.face_normal_body = Eigen::Vector3d(0.0, 0.0, 1.0);
        EXPECT_NEAR(radToDeg(windowCenterTheta(CubeSatelliteModel(p), Eigen::Vector3d::Zero())),
                    -106.83, 0.01);
    }
    // Base on the spin axis: undefined.
    CubeSatelliteModel::Params p;
    const CubeSatelliteModel m(p);
    EXPECT_THROW(windowCenterTheta(m, Eigen::Vector3d(0.0, 0.0, 5.0)), std::invalid_argument);
}
