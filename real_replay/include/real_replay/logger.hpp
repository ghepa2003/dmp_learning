#pragma once

// Preallocated in-memory log. The control loop only fills slots (no allocation, no I/O, no
// threads); the CSV is written once, after the control loop has ended.

#include <cstddef>
#include <string>
#include <vector>

namespace real_replay {

/// One row per control cycle. All members are double so the struct is a flat array of doubles.
struct LogRecord {
  double t;             ///< trajectory time = sum of period [s]
  double period;        ///< period reported by libfranka for this cycle [s]
  double cb_us;         ///< duration of this cycle's callback [us]
  double success_rate;  ///< control_command_success_rate
  double gain_alpha;    ///< K scale of the ramp
  double phase;         ///< LoopPhase as number
  double flags;         ///< bit0 rate-limited, bit1 clamped, bit2 stopping
  double q[7];
  double dq[7];
  double tau_J[7];      ///< measured joint torques
  double tau_cmd[7];    ///< torque sent (after safety layer)
  double p_des[3];      ///< source target position (not the latched stop target)
  double q_des[4];      ///< source target quaternion w,x,y,z
  double p_meas[3];     ///< FK position from measured q
  double q_meas[4];     ///< FK quaternion w,x,y,z
  double e_pos[3];      ///< position error actually used by the law
  double e_rot[3];      ///< orientation error (rotation vector) actually used by the law
};

class Logger {
 public:
  static constexpr std::size_t kNumFields = sizeof(LogRecord) / sizeof(double);

  /// Allocates and zero-fills (touches every page) `capacity` records.
  explicit Logger(std::size_t capacity);

  /// Next free slot, or nullptr when full.
  LogRecord* next() noexcept { return size_ < buf_.size() ? &buf_[size_++] : nullptr; }
  bool full() const noexcept { return size_ >= buf_.size(); }
  std::size_t size() const noexcept { return size_; }
  std::size_t capacity() const noexcept { return buf_.size(); }
  const LogRecord& at(std::size_t i) const noexcept { return buf_[i]; }

  /// True when every logged value is finite.
  bool allFinite() const noexcept;

  /// Writes header + one row per logged cycle. Not for use inside the control loop.
  bool writeCsv(const std::string& path, std::string* error) const;

 private:
  std::vector<LogRecord> buf_;
  std::size_t size_ = 0;
};

}  // namespace real_replay
