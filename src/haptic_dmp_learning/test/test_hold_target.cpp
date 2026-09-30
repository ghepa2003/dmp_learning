// Pure helpers of the executor's hold target (message types only, no rclcpp::init()).

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include "haptic_dmp_learning/ros/hold_target.hpp"

namespace ht = haptic_dmp_learning::ros_wrapper::hold_target;

namespace {
const double kNaN = std::numeric_limits<double>::quiet_NaN();
}

TEST(HoldTarget, KeepsPositionAndQuaternionAsGiven) {
    // Z18-like: hand pointing down (x = 1 in x,y,z,w), tiny tilt.
    const Eigen::Quaterniond q(-0.000149, 0.999956, -0.000072, -0.009383);  // ctor order: w, x, y, z
    const Eigen::Vector3d p(0.306891, 0.0, 0.486882);
    const ht::HoldPose h = ht::makeHoldPose(p, Eigen::Quaterniond(q.coeffs() / q.norm()));
    EXPECT_EQ(h.position, p);
    // Normalized, and the sign is NOT flipped to w >= 0.
    EXPECT_NEAR(h.orientation.norm(), 1.0, 1e-15);
    EXPECT_LT(h.orientation.w(), 0.0);
    EXPECT_NEAR(h.orientation.x(), 0.999956 / q.norm(), 1e-12);
}

TEST(HoldTarget, IdentityIsPreserved) {
    const ht::HoldPose h = ht::makeHoldPose(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
    EXPECT_DOUBLE_EQ(h.orientation.w(), 1.0);
    EXPECT_DOUBLE_EQ(h.orientation.x(), 0.0);
}

TEST(HoldTarget, SlightlyOffNormIsNormalizedButFarOffNormThrows) {
    const Eigen::Quaterniond q(1.0004, 0.0, 0.0, 0.0);
    const ht::HoldPose h = ht::makeHoldPose(Eigen::Vector3d::Zero(), q);
    EXPECT_NEAR(h.orientation.norm(), 1.0, 1e-15);
    EXPECT_THROW(ht::makeHoldPose(Eigen::Vector3d::Zero(), Eigen::Quaterniond(1.1, 0.0, 0.0, 0.0)),
                 std::invalid_argument);
    EXPECT_THROW(ht::makeHoldPose(Eigen::Vector3d::Zero(), Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0)),
                 std::invalid_argument);
}

TEST(HoldTarget, NonFiniteComponentsThrow) {
    EXPECT_THROW(ht::makeHoldPose(Eigen::Vector3d(0.1, kNaN, 0.3), Eigen::Quaterniond::Identity()),
                 std::invalid_argument);
    EXPECT_THROW(ht::makeHoldPose(Eigen::Vector3d(0.1, 0.2, std::numeric_limits<double>::infinity()),
                                  Eigen::Quaterniond::Identity()),
                 std::invalid_argument);
    EXPECT_THROW(ht::makeHoldPose(Eigen::Vector3d::Zero(), Eigen::Quaterniond(1.0, kNaN, 0.0, 0.0)),
                 std::invalid_argument);
}

TEST(HoldTarget, MessageFieldsAndOrder) {
    const Eigen::Quaterniond q = Eigen::Quaterniond(0.5, -0.5, 0.5, -0.5).normalized();  // w, x, y, z
    const ht::HoldPose h = ht::makeHoldPose(Eigen::Vector3d(1.0, 2.0, 3.0), q);
    const rclcpp::Time stamp(12, 340000000, RCL_ROS_TIME);
    const auto msg = ht::toPoseStamped(h, "world_frame_x", stamp);
    EXPECT_EQ(msg.header.frame_id, "world_frame_x");
    EXPECT_EQ(rclcpp::Time(msg.header.stamp).nanoseconds(), stamp.nanoseconds());
    EXPECT_DOUBLE_EQ(msg.pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(msg.pose.position.y, 2.0);
    EXPECT_DOUBLE_EQ(msg.pose.position.z, 3.0);
    EXPECT_DOUBLE_EQ(msg.pose.orientation.w, 0.5);
    EXPECT_DOUBLE_EQ(msg.pose.orientation.x, -0.5);
    EXPECT_DOUBLE_EQ(msg.pose.orientation.y, 0.5);
    EXPECT_DOUBLE_EQ(msg.pose.orientation.z, -0.5);
}

TEST(HoldTarget, EveryTickCarriesTheSamePoseWithAFreshStamp) {
    const ht::HoldPose h = ht::makeHoldPose(Eigen::Vector3d(0.3, 0.0, 0.5), Eigen::Quaterniond::Identity());
    const auto a = ht::toPoseStamped(h, "f", rclcpp::Time(1, 0, RCL_ROS_TIME));
    const auto b = ht::toPoseStamped(h, "f", rclcpp::Time(1, 100000000, RCL_ROS_TIME));
    EXPECT_EQ(a.pose, b.pose);
    EXPECT_NE(rclcpp::Time(a.header.stamp).nanoseconds(), rclcpp::Time(b.header.stamp).nanoseconds());
}

TEST(HoldTarget, QuatAngleIsInvariantToSignAndMatchesKnownValues) {
    const Eigen::Quaterniond id = Eigen::Quaterniond::Identity();
    const Eigen::Quaterniond r90(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
    EXPECT_NEAR(ht::quatAngleDeg(id, id), 0.0, 1e-6);
    EXPECT_NEAR(ht::quatAngleDeg(id, r90), 90.0, 1e-9);
    EXPECT_NEAR(ht::quatAngleDeg(id, Eigen::Quaterniond(-r90.coeffs())), 90.0, 1e-9);
    EXPECT_NEAR(ht::quatAngleDeg(Eigen::Quaterniond(-id.coeffs()), id), 0.0, 1e-6);
    EXPECT_NEAR(ht::quatAngleDeg(id, Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()))), 180.0,
                1e-9);
    EXPECT_THROW(ht::quatAngleDeg(Eigen::Quaterniond(0, 0, 0, 0), id), std::invalid_argument);
}

TEST(HoldTarget, QuatAngleReproducesTheZ18Measurement) {
    // q0 of the production quaternion DMP, stored [w, x, y, z] in weights.yaml, and the Z18 hold quaternion
    // read as (x, y, z, w) = (0.999956, -0.000072, -0.009383, -0.000149): 92.75 deg apart.
    const Eigen::Quaterniond q0(0.72345797222799724, -0.6896759735248158, 0.0215395991731409, 0.022173399148810675);
    Eigen::Quaterniond hold(-0.000149, 0.999956, -0.000072, -0.009383);
    hold.normalize();
    EXPECT_NEAR(ht::quatAngleDeg(hold, q0), 92.7456, 5e-3);
}

TEST(HoldTarget, PeriodFromRate) {
    EXPECT_DOUBLE_EQ(ht::holdPeriodS(10.0), 0.1);
    EXPECT_DOUBLE_EQ(ht::holdPeriodS(200.0), 0.005);
    EXPECT_THROW(ht::holdPeriodS(0.0), std::invalid_argument);
    EXPECT_THROW(ht::holdPeriodS(-1.0), std::invalid_argument);
    EXPECT_THROW(ht::holdPeriodS(200.1), std::invalid_argument);
    EXPECT_THROW(ht::holdPeriodS(kNaN), std::invalid_argument);
    EXPECT_THROW(ht::holdPeriodS(std::numeric_limits<double>::infinity()), std::invalid_argument);
    try {
        ht::holdPeriodS(0.0);
    } catch (const std::invalid_argument& e) {
        EXPECT_NE(std::string(e.what()).find("hold_target_rate_hz"), std::string::npos);
    }
}

TEST(HoldTarget, FatalSentenceIsAppendedOnlyWhenTheHoldRemains) {
    EXPECT_EQ(ht::withHoldNote("boom", false), "boom");
    const std::string with = ht::withHoldNote("boom", true);
    EXPECT_EQ(with.rfind("boom", 0), 0u);
    EXPECT_NE(with.find("hold target remains active in the controller (no watchdog)"), std::string::npos);
}
