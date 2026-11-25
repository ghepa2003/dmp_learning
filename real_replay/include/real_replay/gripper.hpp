#pragma once

// Franka Hand interface of the task layer (stage 3). No libfranka here: FrankaGripper lives in gripper_franka.hpp and
// is linked only into the real_replay executable, so every test uses FakeGripper.
//
// All blocking calls (move(), grasp()) run in the GripperWorker thread, never in the 1 kHz loop and never in the task
// thread. The worker owns the IGripper object (single thread: no concurrent use of one franka::Gripper).
// Opening the hand does not depend on the arm: the gripper connection is separate from the robot control connection.

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace real_replay {
namespace grip {

constexpr double kMaxForceN = 10.0;     ///< hard limit: a larger force is REFUSED (not clamped)
constexpr double kMaxSpeedMs = 0.03;    ///< [m/s]
constexpr double kMaxEpsM = 0.01;       ///< [m] same limit as tools/gripper_grasp_test.cpp
constexpr double kMaxTotalOpenM = 0.08; ///< Franka Hand: 0.040 m per finger

/// hw_config.yaml `gripper:` section.
struct GripperParams {
  bool enabled = false;            ///< false = stage-2 behaviour (no gripper, APPROACH -> HOLD_SAFE)
  std::string host;                ///< "" = robot1.ip (the Hand is on the same address as the arm)
  double open_width_m = 0.080;     ///< TOTAL opening (0.040 per finger), before the approach and at RELEASE
  double speed_m_s = 0.02;         ///< (0, 0.03]
  /// Grasp force [N]. libfranka documents `force` as "Grasping force" WITHOUT saying whether it is per finger or the
  /// total: it is NOT assumed here. The value is passed to Gripper::grasp() as it is. Hard limit 10 N: larger = refused.
  double force_n = 5.0;
  double grasp_width_m = 0.009;    ///< commanded width = collar wall thickness (hw: assumed, not measured)
  /// Tolerances of is_grasped (NOT the width reached): grasped if width in [grasp_width - eps_in, grasp_width + eps_out].
  double eps_in_m = 0.010, eps_out_m = 0.010;
  double squeeze_s = 2.0;          ///< SQUEEZE: keep the grasp at force_n for this long
  double open_timeout_s = 10.0;    ///< initial opening / release: no confirmation within this = fault
  double poll_period_s = 0.1;      ///< state polling of the idle worker
};

/// @throws std::runtime_error naming the key. Force > 10 N and speed > 0.03 m/s are refused.
void validateGripperParams(const GripperParams& g);

struct GripperState {
  bool valid = false;
  double width = 0.0, max_width = 0.0;
  bool is_grasped = false;
};

class IGripper {
 public:
  virtual ~IGripper() = default;
  /// Blocking. @return false if the Hand reports failure; may throw (network, command error).
  virtual bool move(double width_m, double speed_m_s) = 0;
  /// Blocking (this is why the worker thread exists). @return false if the grasp failed.
  virtual bool grasp(double width_m, double speed_m_s, double force_n, double eps_in_m, double eps_out_m) = 0;
  virtual GripperState readState() = 0;
};

/// Scriptable gripper for tests. Thread-safe. Every call is recorded with its CLOCK_MONOTONIC time.
class FakeGripper final : public IGripper {
 public:
  struct Call {
    std::string what;  ///< "move" | "grasp"
    double width = 0.0, t_start_s = 0.0, t_end_s = 0.0;
  };
  // script (set before the run)
  bool move_ok = true, grasp_ok = true;
  bool throw_on_move = false, throw_on_grasp = false;
  double move_delay_s = 0.02, grasp_delay_s = 0.05;
  double width_after_grasp = 0.009;   ///< state reported after a successful grasp
  double max_width = 0.080;
  bool drop_object = false;            ///< is_grasped becomes false (object lost) once set

  bool move(double width_m, double speed_m_s) override;
  bool grasp(double width_m, double speed_m_s, double force_n, double eps_in_m, double eps_out_m) override;
  GripperState readState() override;
  std::vector<Call> calls() const;
  /// args of the last grasp()
  double last_force = 0.0, last_speed = 0.0, last_eps_in = 0.0, last_eps_out = 0.0;

 private:
  mutable std::mutex m_;
  std::vector<Call> calls_;
  double width_ = 0.080;
  bool grasped_ = false;
};

/// Runs the gripper commands in a dedicated thread. Command ids are assigned by the caller (the machine), strictly
/// increasing; a command replaces a pending one that has not started (an opening wins over a pending grasp).
class GripperWorker {
 public:
  enum class Kind { kOpen, kGrasp };
  struct Status {
    std::uint32_t done_id = 0;  ///< id of the last COMPLETED command (any outcome)
    bool done_ok = false;
    std::uint32_t busy_id = 0;  ///< id being executed (0 = idle)
    bool is_grasped = false;
    double width = 0.0;
    bool state_valid = false;
    std::string error;          ///< last error text ("" = none)
  };

  GripperWorker(std::unique_ptr<IGripper> g, const GripperParams& p);
  ~GripperWorker();
  void start();
  void request(Kind k, std::uint32_t id);
  Status status() const;

 private:
  void loop();
  std::unique_ptr<IGripper> g_;
  GripperParams p_;
  mutable std::mutex m_;
  std::condition_variable cv_;
  bool quit_ = false, pending_ = false;
  Kind pending_kind_ = Kind::kOpen;
  std::uint32_t pending_id_ = 0;
  Status st_;
  std::thread th_;
};

}  // namespace grip
}  // namespace real_replay
