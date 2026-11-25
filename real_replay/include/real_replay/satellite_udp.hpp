#pragma once

// UDP datagrams of satellite_node (protocol version 3) and a lock-free "latest value" mailbox.
// Layout documented in hw/README.md; the Python side (hw/hw.py) uses the same struct formats (version 3; layout unchanged since v2):
//   state   '<IHHIIQ5d3d3dII'  (120 bytes)
//   command '<IHHIIQ'           (24 bytes)
// Byte order: little-endian (host order, checked below). Angles in rad, angular speeds in rad/s, lengths in m.

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace real_replay {
namespace sat {

static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "the datagram layout is little-endian (host order)");

constexpr std::uint32_t kStateMagic = 0x53544153u;    // bytes 'S','A','T','S'
constexpr std::uint32_t kCommandMagic = 0x43544153u;  // bytes 'S','A','T','C'
// v2: `flags` (was `reserved`) with kFlagOmegaValid.
// v3: CommandType::kStopRamp (normal stop, ramp robot2.control.stop_ramp_s); kStop is now the HARD stop (ramp
//     robot2.control.stop_hard_ramp_s, used for faults). Layouts unchanged. A v2 peer is refused on both sides (version
//     check), so a node that would ignore kStopRamp can never be paired with a task layer that sends it.
constexpr std::uint16_t kProtocolVersion = 3;

enum class SatState : std::uint32_t {
  kInit = 0,       ///< preflight, not controlling yet
  kWaitStart = 1,  ///< controlling, holding q0, waiting for START
  kRunning = 2,    ///< nominal profile (start ramp, constant speed, nominal stop ramp at the window end)
  kStopping = 3,   ///< early stop ramp (command STOP, fault, Ctrl-C)
  kHold = 4,       ///< at rest, final hold
  kDone = 5,       ///< control ended (last datagrams before exit)
};

/// Fault / event bits (state datagram `faults`). kPhaseMismatch is a WARNING only (no stop).
enum FaultBits : std::uint32_t {
  kFaultWindow = 1u << 0,         ///< q7 or theta outside the window: stop ramp
  kFaultHeartbeat = 1u << 1,      ///< no heartbeat for T_hb_s after the first one
  kFaultTracking = 1u << 2,       ///< joint tracking error
  kFaultSafety = 1u << 3,         ///< SafetyGuard (period, budget, success rate, joint limit/speed)
  kFaultPhaseMismatch = 1u << 4,  ///< |theta_flange - theta_q7| > phase_mismatch_warn_deg (warning)
  kFaultStopCommand = 1u << 5,    ///< STOP received (normal end requested by the task layer)
  kFaultSignal = 1u << 6,         ///< SIGINT / SIGTERM
  kFaultNonFinite = 1u << 7,      ///< non-finite state or torque
  kFaultWaitTimeout = 1u << 8,    ///< no START within max_wait_start_s
  kFaultLogFull = 1u << 9,
};

/// Status bits (state datagram `flags`, NOT latched: valid for this sample only).
enum FlagBits : std::uint32_t {
  /// omega_meas usable: RUNNING, the whole regression window lies in the constant-speed phase
  /// (ramp_s + omega_window_s <= t_run <= end of the constant phase) and the regression is not NaN.
  /// False in WAIT_START, during the start ramp, the nominal stop ramp, STOPPING, HOLD, DONE.
  /// Consumers must not use omega_meas for any decision while this bit is false.
  kFlagOmegaValid = 1u << 0,
};

#pragma pack(push, 1)
struct StatePacket {
  std::uint32_t magic = kStateMagic;
  std::uint16_t version = kProtocolVersion;
  std::uint16_t size = 0;           ///< sizeof(StatePacket), filled by makeState
  std::uint32_t seq = 0;            ///< incremented by the publisher for every datagram
  std::uint32_t state = 0;          ///< SatState
  std::uint64_t t_mono_ns = 0;      ///< CLOCK_MONOTONIC of the control cycle the sample comes from
  double theta = 0.0;               ///< phase from the flange, branch [-270, 90) deg [rad]
  double omega_meas = 0.0;          ///< LS slope of theta over omega_window_s [rad/s]; NaN while the window is incomplete
  double theta_q7 = 0.0;            ///< phase from q7 [rad]
  double q7 = 0.0;                  ///< [rad]
  double omega_cmd = 0.0;           ///< commanded phase speed of this cycle (profile) [rad/s]
  double center[3] = {0.0, 0.0, 0.0};  ///< satellite centre, frame robot 1 [m]
  double axis[3] = {0.0, 0.0, 1.0};    ///< rotation axis (upwards), frame robot 1, unit
  std::uint32_t faults = 0;         ///< FaultBits (latched)
  std::uint32_t flags = 0;          ///< FlagBits (this sample)
};

/// kStop = hard stop (fault): short ramp. kStopRamp = normal stop (RELEASE / end of run): the nominal ramp.
enum class CommandType : std::uint16_t { kHeartbeat = 1, kStart = 2, kStop = 3, kStopRamp = 4 };

struct CommandPacket {
  std::uint32_t magic = kCommandMagic;
  std::uint16_t version = kProtocolVersion;
  std::uint16_t type = 0;       ///< CommandType
  std::uint32_t seq = 0;
  std::uint32_t reserved = 0;
  std::uint64_t t_mono_ns = 0;  ///< sender's CLOCK_MONOTONIC (informative)
};
#pragma pack(pop)

static_assert(sizeof(StatePacket) == 120, "StatePacket layout changed: bump kProtocolVersion and update hw/README.md + hw.py");
static_assert(offsetof(StatePacket, t_mono_ns) == 16 && offsetof(StatePacket, theta) == 24 && offsetof(StatePacket, center) == 64 &&
                  offsetof(StatePacket, axis) == 88 && offsetof(StatePacket, faults) == 112 && offsetof(StatePacket, flags) == 116,
              "StatePacket field offsets changed");
static_assert(sizeof(CommandPacket) == 24, "CommandPacket layout changed");
static_assert(std::is_trivially_copyable<StatePacket>::value && std::is_trivially_copyable<CommandPacket>::value, "POD");

constexpr std::size_t kStatePacketSize = sizeof(StatePacket);
constexpr std::size_t kCommandPacketSize = sizeof(CommandPacket);

inline void packState(const StatePacket& p, std::uint8_t* buf) noexcept { std::memcpy(buf, &p, sizeof p); }

/// false on wrong size, magic, version or size field.
inline bool unpackState(const std::uint8_t* buf, std::size_t len, StatePacket& out) noexcept {
  if (len != kStatePacketSize) return false;
  StatePacket p;
  std::memcpy(&p, buf, sizeof p);
  if (p.magic != kStateMagic || p.version != kProtocolVersion || p.size != kStatePacketSize) return false;
  out = p;
  return true;
}

inline void packCommand(const CommandPacket& p, std::uint8_t* buf) noexcept { std::memcpy(buf, &p, sizeof p); }

inline bool unpackCommand(const std::uint8_t* buf, std::size_t len, CommandPacket& out) noexcept {
  if (len != kCommandPacketSize) return false;
  CommandPacket p;
  std::memcpy(&p, buf, sizeof p);
  if (p.magic != kCommandMagic || p.version != kProtocolVersion) return false;
  if (p.type < static_cast<std::uint16_t>(CommandType::kHeartbeat) || p.type > static_cast<std::uint16_t>(CommandType::kStopRamp)) return false;
  out = p;
  return true;
}

/// Single-writer / multi-reader seqlock. Every word is a std::atomic (no data race); store() and load() never
/// allocate, never block, never call the OS. load() returns false if no value was stored yet or if it could not get
/// a consistent copy in a few attempts (the reader just retries at its next period).
template <class T>
class LatestValue {
  static_assert(std::is_trivially_copyable<T>::value, "LatestValue needs a trivially copyable type");
  static constexpr std::size_t kWords = (sizeof(T) + 7) / 8;

 public:
  void store(const T& v) noexcept {
    std::uint64_t buf[kWords] = {};
    std::memcpy(buf, &v, sizeof(T));
    const std::uint64_t s = seq_.load(std::memory_order_relaxed);
    seq_.store(s + 1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    for (std::size_t i = 0; i < kWords; ++i) w_[i].store(buf[i], std::memory_order_relaxed);
    seq_.store(s + 2, std::memory_order_release);
  }

  bool load(T& out) const noexcept {
    for (int attempt = 0; attempt < 64; ++attempt) {
      const std::uint64_t s1 = seq_.load(std::memory_order_acquire);
      if (s1 == 0) return false;
      if (s1 & 1u) continue;
      std::uint64_t buf[kWords];
      for (std::size_t i = 0; i < kWords; ++i) buf[i] = w_[i].load(std::memory_order_relaxed);
      std::atomic_thread_fence(std::memory_order_acquire);
      if (seq_.load(std::memory_order_relaxed) == s1) {
        std::memcpy(&out, buf, sizeof(T));
        return true;
      }
    }
    return false;
  }

  std::uint64_t version() const noexcept { return seq_.load(std::memory_order_acquire) / 2; }

 private:
  std::atomic<std::uint64_t> seq_{0};
  std::array<std::atomic<std::uint64_t>, kWords> w_{};
};

}  // namespace sat
}  // namespace real_replay
