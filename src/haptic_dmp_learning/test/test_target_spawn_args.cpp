#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "haptic_dmp_learning/core/target_spawn_args.hpp"

using namespace haptic_dmp_learning::core::target_spawn;

namespace {

// Orchestrator defaults (demo_replay_sync_orchestrator_node.cpp, declare_parameter calls).
TargetSpawnParams defaults() {
  TargetSpawnParams p;
  p.name = "free_target_object";
  p.x = 0.6; p.y = 0.0; p.z = 0.5;
  p.size_x = p.size_y = p.size_z = 0.150;
  p.mass = 4.412;
  return p;
}

// The vector the orchestrator built before roll/pitch existed (literal, std::to_string format).
const std::vector<std::string> kLegacyDefaultArgs = {
    "ros2", "launch", "free_target_object", "free_target_object.launch.py",
    "name:=free_target_object",
    "x:=0.600000", "y:=0.000000", "z:=0.500000",
    "vx:=0.000000", "vy:=0.000000", "vz:=0.000000",
    "wx:=0.000000", "wy:=0.000000", "wz:=0.000000",
    "size_x:=0.150000", "size_y:=0.150000", "size_z:=0.150000",
    "mass:=4.412000",
    "with_collar:=true", "collar_collision:=true",
    "yaw:=0.000000"};

}  // namespace

TEST(TargetSpawnArgs, ZeroRollPitchGivesLegacyVector) {
  EXPECT_EQ(buildSpawnArgs(defaults()), kLegacyDefaultArgs);
}

TEST(TargetSpawnArgs, ZeroRollPitchWithRunParametersGivesLegacyVector) {
  auto p = defaults();
  p.x = 0.70; p.z = 0.77; p.wz = -0.034906585; p.yaw = -2.4435;
  // Order of the keys as in the original argv of the orchestrator (HEAD, demo_replay_sync_orchestrator_node.cpp:484-493).
  const std::vector<std::string> expected = {
      "ros2", "launch", "free_target_object", "free_target_object.launch.py",
      "name:=free_target_object",
      "x:=0.700000", "y:=0.000000", "z:=0.770000",
      "vx:=0.000000", "vy:=0.000000", "vz:=0.000000",
      "wx:=0.000000", "wy:=0.000000", "wz:=-0.034907",
      "size_x:=0.150000", "size_y:=0.150000", "size_z:=0.150000",
      "mass:=4.412000",
      "with_collar:=true", "collar_collision:=true",
      "yaw:=-2.443500"};
  EXPECT_EQ(buildSpawnArgs(p), expected);
}

TEST(TargetSpawnArgs, NegativeZeroIsTreatedAsZero) {
  auto p = defaults();
  p.roll = -0.0;
  p.pitch = -0.0;
  EXPECT_EQ(buildSpawnArgs(p), kLegacyDefaultArgs);
}

TEST(TargetSpawnArgs, NonZeroRollPitchInsertedBeforeYaw) {
  auto p = defaults();
  p.roll = 0.25;
  p.pitch = -0.125;
  p.yaw = 1.0;
  const auto a = buildSpawnArgs(p);
  ASSERT_EQ(a.size(), kLegacyDefaultArgs.size() + 2);
  EXPECT_EQ(a[a.size() - 3], "roll:=0.250000");
  EXPECT_EQ(a[a.size() - 2], "pitch:=-0.125000");
  EXPECT_EQ(a[a.size() - 1], "yaw:=1.000000");
  // Everything before roll is unchanged.
  for (std::size_t i = 0; i + 1 < kLegacyDefaultArgs.size(); ++i) EXPECT_EQ(a[i], kLegacyDefaultArgs[i]);
}

TEST(TargetSpawnArgs, OnlyOneOfRollPitch) {
  auto p = defaults();
  p.pitch = 0.3;
  const auto a = buildSpawnArgs(p);
  ASSERT_EQ(a.size(), kLegacyDefaultArgs.size() + 1);
  EXPECT_EQ(a[a.size() - 2], "pitch:=0.300000");
}

TEST(TargetSpawnArgs, RollPitchValidation) {
  EXPECT_TRUE(rollPitchValid(0.0, 0.0));
  EXPECT_TRUE(rollPitchValid(0.3, -0.3));
  EXPECT_FALSE(rollPitchValid(std::numeric_limits<double>::quiet_NaN(), 0.0));
  EXPECT_FALSE(rollPitchValid(0.0, std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(rollPitchValid(0.0, M_PI / 2.0));
  EXPECT_FALSE(rollPitchValid(0.0, -2.0));
}
