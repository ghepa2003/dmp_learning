#pragma once

// Runtime of the task layer (stage 2): the target source that runs inside the 1 kHz loop, the data shared with the
// task thread (lock-free), the UDP threads (satellite state in, heartbeat/START/STOP out) and the task thread that
// drives the pure TaskMachine (task_logic.hpp).
//
// Rules of the 1 kHz callback: TaskSource::sample() and recordCycle() never allocate, never call the OS for
// network I/O, never block: they read/write std::atomic and sat::LatestValue (seqlock) only. Everything else
// (sockets, YAML, validation, stdin, printing) runs in the threads below.

#include <atomic>
#include <csignal>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "real_replay/gripper.hpp"
#include "real_replay/prodmp_source.hpp"
#include "real_replay/satellite_udp.hpp"
#include "real_replay/target_source.hpp"
#include "real_replay/task_logic.hpp"

namespace real_replay {
namespace task {

struct PoseSnap {
  double p[3];
  double q[4];  ///< w, x, y, z
  double qj[7];
  double t_mono_s;
};

/// Shared between the 1 kHz callback (writer of the loop side) and the task thread (writer of the command side).
struct TaskShared {
  std::atomic<int> source_cmd{static_cast<int>(SourceCmd::kHold)};
  std::atomic<bool> end_requested{false};
  std::atomic<bool> activation_ready{false};
  sat::LatestValue<PoseSnap> activation;
  sat::LatestValue<PoseSnap> measured;
  std::atomic<bool> approach_started{false};
  std::atomic<bool> approach_done{false};
  std::atomic<bool> loop_finished{false};
  std::atomic<int> loop_reason{0};
  std::atomic<bool> jitter_fault{false};
  std::atomic<std::uint64_t> loop_cycles{0};  ///< incremented by EVERY 1 kHz callback: gates the heartbeat
  std::atomic<bool> guard_valid{false};       ///< published by the callback (robot mode only)
  std::atomic<double> e_pos_m{0.0}, f_ext_n{0.0};
  std::atomic<int> task_state{static_cast<int>(TaskState::kWaitCmd)};
  sat::LatestValue<SatSample> sat_latest;  ///< latest ACCEPTED satellite sample
};

/// Target source of --mode task. HOLD: activation pose. APPROACH: the goal-mode ProDmpSource, started at the
/// activation pose in the first cycle that sees the command. FOLLOW (stage 3): the end pose of the approach rotated
/// rigidly with the satellite about its axis through its centre (see below). FREEZE: last target, zero twist (sticky).
///
/// FOLLOW: dtheta = wrapPi(theta_sat_est - theta_end), where theta_end is the satellite phase estimated in the first
/// follow cycle and theta_sat_est = sample.theta + rate * age (the sample is `age` old; rate = the sample's omega_cmd,
/// which follows the stop ramp). If the latest sample is older than kFollowFreshAgeS the estimate runs open loop at the
/// nominal omega from the last fresh estimate (omega*(t - t_end) if there never was a fresh one).
///   p = c + R(axis, dtheta) (p_end - c),   q = R(axis, dtheta) q_end,   v = w x (p - c),   w = rate * axis.
/// c and axis come from the satellite datagram (preflight-checked against hw_config in satellite_node).
class TaskSource final : public ITargetSource {
 public:
  using ClockFn = double (*)();  ///< CLOCK_MONOTONIC [s]; default = the system clock (tests inject a fake)
  static constexpr double kFollowFreshAgeS = 0.5;
  TaskSource(std::unique_ptr<ProDmpSource> approach, TaskShared& shared, double nominal_s, ClockFn clock = nullptr, double omega_nominal = 0.0);
  void start(const Pose& activation) override;
  Pose sample(double dt) override;
  double progress() const override { return approach_started_ ? approach_->progress() : 0.0; }
  bool done() const override { return shared_.end_requested.load(std::memory_order_acquire); }
  double nominalDurationS() const override { return nominal_s_; }
  bool twist(Eigen::Vector3d& v, Eigen::Vector3d& w) const override;

 private:
  std::unique_ptr<ProDmpSource> approach_;
  TaskShared& shared_;
  double nominal_s_;
  SourceCmd mode_ = SourceCmd::kHold;
  void startFollow();
  void stepFollow();

  ClockFn clock_;
  double omega_nominal_;
  bool approach_started_ = false;
  bool approach_done_ = false;
  bool follow_started_ = false;
  Pose act_, last_;
  // follow state (1 kHz callback only)
  Eigen::Vector3d f_c_ = Eigen::Vector3d::Zero(), f_axis_ = Eigen::Vector3d::UnitZ(), f_p_end_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond f_q_end_ = Eigen::Quaterniond::Identity();
  double f_theta_end_ = 0.0, f_theta_est_ = 0.0, f_t_est_ = 0.0, f_rate_ = 0.0;
  Eigen::Vector3d f_v_ = Eigen::Vector3d::Zero(), f_w_ = Eigen::Vector3d::Zero();

 public:
  /// Follow figures for the tests / logs.
  double followDtheta() const { return f_dtheta_; }
  bool followStale() const { return f_stale_; }

 private:
  double f_dtheta_ = 0.0;
  bool f_stale_ = false;
};

/// Per-cycle task record (written by the callback wrapper into preallocated storage).
struct TaskCycleRec {
  double t_mono_s;
  double task_state;
  double source_cmd;
  double theta_deg;
  double sample_age_ms;
  double e_pos_m;
  double e_rot_rad;
};

class TaskCycleLog {
 public:
  explicit TaskCycleLog(std::size_t capacity) { rec_.reserve(capacity); }
  /// Called by the callback wrapper after loop.step(): no allocation (stops recording when full).
  void record(const TaskShared& sh, double t_mono_s, const double e_pos[3], const double e_rot[3]) noexcept;
  bool writeCsv(const std::string& path, std::string* err) const;
  std::size_t size() const { return rec_.size(); }

 private:
  std::vector<TaskCycleRec> rec_;
};

/// Inputs that do not change during the run, for VALIDATE.
struct StaticValidation {
  double tau = 0.0;
  std::string weights_sha256;
  bool path_checked = false, path_ok = false;
  std::string path_detail;
  double stop_at_fraction = 1.0;
};

/// Where the machine's one-shot actions go. The ORDER of executeActions() is the contract (tested with a recording
/// sink): START, then STOP, then the gripper. On a fault the STOP to the satellite is therefore always sent before
/// the hand is commanded to open; the opening does not wait for the satellite to stop.
class IActionSink {
 public:
  virtual ~IActionSink() = default;
  virtual void sendStart() = 0;
  virtual void sendStop(bool hard) = 0;
  virtual void gripper(GripperCmd c, std::uint32_t id) = 0;
};
void executeActions(const MachineActions& a, IActionSink& sink);

class TaskRuntime : private IActionSink {
 public:
  TaskRuntime(const HwTaskConfig& hw, TaskMachine& machine, TaskShared& shared, const StaticValidation& sv, bool yes,
              const volatile std::sig_atomic_t* abort_flag);
  ~TaskRuntime();
  /// Gripper worker (optional, owned by the caller, must outlive the runtime). Without it gripper actions are ignored.
  void setGripper(grip::GripperWorker* g) { grip_ = g; }
  /// Opens the sockets (throws std::runtime_error) and starts the threads.
  void start();
  /// Lets the task thread see the end of the loop, makes sure STOP was sent if START was, joins the threads.
  void finish();
  std::uint64_t rejected(SampleVerdict v) const { return rej_[static_cast<int>(v)].load(); }
  std::uint64_t accepted() const { return rej_[0].load(); }

 private:
  void rxLoop();
  void hbLoop();
  void taskLoop();
  void confirmLoop();
  void send(std::uint16_t type);
  void sendStart() override;
  void sendStop(bool hard) override;
  void gripper(GripperCmd c, std::uint32_t id) override;
  void runValidation(const MachineInputs& in);

  HwTaskConfig hw_;
  TaskMachine& m_;
  TaskShared& sh_;
  StaticValidation sv_;
  bool yes_;
  const volatile std::sig_atomic_t* abort_;
  grip::GripperWorker* grip_ = nullptr;
  int rx_fd_ = -1, tx_fd_ = -1;
  std::atomic<bool> quit_{false};
  std::atomic<bool> hb_enabled_{false};
  std::atomic<int> confirm_answer_{0};  ///< 0 none, 1 yes, 2 no
  std::atomic<bool> confirm_asked_{false};
  std::atomic<std::uint32_t> tx_seq_{0};
  std::atomic<std::uint64_t> rej_[5];
  std::thread rx_, hb_, task_, confirm_;
};

}  // namespace task
}  // namespace real_replay
