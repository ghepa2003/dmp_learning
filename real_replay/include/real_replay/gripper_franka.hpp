#pragma once
// FrankaGripper: franka::Gripper on the Hand's own connection (separate from the arm control connection). Only the
// real_replay executable links this (libfranka); tests use FakeGripper.
#include <memory>
#include <string>

#include "real_replay/gripper.hpp"

namespace real_replay {
namespace grip {

/// Connects (throws franka::Exception on failure), reads the state once and refuses if the Hand is not homed
/// (max_width < 0.05 m, like the standalone tools). No homing is ever done here.
std::unique_ptr<IGripper> makeFrankaGripper(const std::string& host);

}  // namespace grip
}  // namespace real_replay
