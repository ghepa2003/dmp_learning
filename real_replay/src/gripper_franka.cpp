#include "real_replay/gripper_franka.hpp"

#include <franka/exception.h>
#include <franka/gripper.h>

#include <stdexcept>

namespace real_replay {
namespace grip {

namespace {
class FrankaGripper final : public IGripper {
 public:
  explicit FrankaGripper(const std::string& host) : g_(host) {
    const franka::GripperState s = g_.readOnce();
    if (s.max_width < 0.05) throw std::runtime_error("gripper not homed (max_width < 0.05 m): homing is NOT done by the task layer");
  }
  bool move(double w, double v) override { return g_.move(w, v); }
  bool grasp(double w, double v, double f, double ei, double eo) override { return g_.grasp(w, v, f, ei, eo); }
  GripperState readState() override {
    const franka::GripperState s = g_.readOnce();
    GripperState r;
    r.valid = true;
    r.width = s.width;
    r.max_width = s.max_width;
    r.is_grasped = s.is_grasped;
    return r;
  }

 private:
  franka::Gripper g_;
};
}  // namespace

std::unique_ptr<IGripper> makeFrankaGripper(const std::string& host) { return std::make_unique<FrankaGripper>(host); }

}  // namespace grip
}  // namespace real_replay
