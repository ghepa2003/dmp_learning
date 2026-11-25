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
  std::atomic<int> task_state{static_cast<int>(TaskState::kWaitCmd)};
  sat::LatestValue<SatSample> sat_latest;  ///< latest ACCEPTED satellite sample
};

/// Target source of --mode task. HOLD: activation pose. APPROACH: the goal-mode ProDmpSource, started at the
/// activation pose in the first cycle that sees the command. FREEZE: last target, zero twist (sticky).
class TaskSource final : public ITargetSource {
 public:
  TaskSource(std::unique_ptr<ProDmpSource> approach, TaskShared& shared, double nominal_s);
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
  bool approach_started_ = false;
  bool approach_done_ = false;
  Pose act_, last_;
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

class TaskRuntime {
 public:
  TaskRuntime(const HwTaskConfig& hw, TaskMachine& machine, TaskShared& shared, const StaticValidation& sv, bool yes,
              const volatile std::sig_atomic_t* abort_flag);
  ~TaskRuntime();
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
  void runValidation(const MachineInputs& in);

  HwTaskConfig hw_;
  TaskMachine& m_;
  TaskShared& sh_;
  StaticValidation sv_;
  bool yes_;
  const volatile std::sig_atomic_t* abort_;
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
