#include "real_replay/task_runtime.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>

#include "real_replay/satellite_phase.hpp"

namespace real_replay {
namespace task {

namespace {

double monoNow() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<double>(ts.tv_sec) + 1e-9 * static_cast<double>(ts.tv_nsec);
}

std::uint64_t monoNs() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ull + static_cast<std::uint64_t>(ts.tv_nsec);
}

constexpr double kDeg = sat::kDeg;

}  // namespace

// ---- TaskSource -------------------------------------------------------------------------------------------------
const char* toString(FollowStartKind k) {
  switch (k) {
    case FollowStartKind::kNone: return "not started";
    case FollowStartKind::kFresh: return "fresh read";
    case FollowStartKind::kKeptCopy: return "seqlock read failed (contention): last valid copy kept by the 1 kHz loop";
  }
  return "?";
}

const char* toString(FollowFreezeReason r) {
  switch (r) {
    case FollowFreezeReason::kNone: return "none";
    case FollowFreezeReason::kNoSample: return "no satellite sample ever stored";
    case FollowFreezeReason::kReadContention: return "seqlock read failing (contention) beyond follow_read_hold_s, no valid copy kept";
    case FollowFreezeReason::kInvalidSample: return "satellite sample not finite / axis invalid";
    case FollowFreezeReason::kStaleSample: return "satellite sample older than kFollowFreshAgeS";
  }
  return "?";
}

TaskSource::TaskSource(std::unique_ptr<ProDmpSource> approach, TaskShared& shared, double nominal_s, ClockFn clock, double omega_nominal,
                       double read_hold_s)
    : approach_(std::move(approach)),
      shared_(shared),
      nominal_s_(nominal_s),
      clock_(clock ? clock : &monoNow),
      omega_nominal_(omega_nominal),
      read_hold_s_(read_hold_s) {
  if (!approach_) throw std::invalid_argument("TaskSource: no approach source");
  if (!(read_hold_s_ > 0.0) || !std::isfinite(read_hold_s_)) throw std::invalid_argument("TaskSource: read_hold_s must be > 0");
}

void TaskSource::cacheSat() {
  SatSample s;
  if (shared_.sat_latest.tryLoad(s) == sat::LoadStatus::kOk) {  // bounded, no syscall
    rt_last_ = s;
    rt_have_ = true;
  }
}

void TaskSource::freezeFollow(FollowFreezeReason r, double now) {
  mode_ = SourceCmd::kFreeze;  // no usable phase: never move; the machine faults on the same data
  shared_.follow_wait_s.store(std::isfinite(f_wait_since_) ? now - f_wait_since_ : 0.0, std::memory_order_relaxed);
  shared_.follow_freeze_reason.store(static_cast<int>(r), std::memory_order_release);
}

void TaskSource::start(const Pose& activation) {
  act_ = activation;
  last_ = activation;
  mode_ = SourceCmd::kHold;
}

Pose TaskSource::sample(double dt) {
  const auto cmd = static_cast<SourceCmd>(shared_.source_cmd.load(std::memory_order_acquire));
  if (cmd == SourceCmd::kFreeze) mode_ = SourceCmd::kFreeze;  // sticky
  if (cmd == SourceCmd::kFollow && mode_ == SourceCmd::kApproach && approach_done_ && !follow_started_) startFollow();
  if (mode_ == SourceCmd::kHold && cmd == SourceCmd::kApproach && !approach_started_) {
    // The goal was set (setGoal) by the task thread BEFORE the release store of kApproach: safe to use here.
    // Anchored at the activation pose: the orientation read ONCE in WAIT_CMD.
    approach_->start(act_);
    last_ = approach_->sample(0.0);
    approach_started_ = true;
    mode_ = SourceCmd::kApproach;
    shared_.approach_started.store(true, std::memory_order_release);
    return last_;
  }
  switch (mode_) {
    case SourceCmd::kHold:
      last_ = act_;
      break;
    case SourceCmd::kApproach:
      cacheSat();  // keeps a valid copy for the first follow cycle (its own read may fail on contention)
      last_ = approach_->sample(dt);
      if (!approach_done_ && approach_->done()) {
        approach_done_ = true;
        shared_.approach_done.store(true, std::memory_order_release);
      }
      break;
    case SourceCmd::kFreeze:
      break;  // last target, zero twist
    case SourceCmd::kFollow:
      stepFollow();
      break;
  }
  return last_;
}

void TaskSource::startFollow() {
  SatSample s;
  const double now = clock_();
  FollowStartKind kind = FollowStartKind::kFresh;
  const sat::LoadStatus st = shared_.sat_latest.tryLoad(s);
  if (st == sat::LoadStatus::kOk) {
    rt_last_ = s;
    rt_have_ = true;
  } else if (st == sat::LoadStatus::kEmpty) {
    freezeFollow(FollowFreezeReason::kNoSample, now);
    return;
  } else {  // kContended: the rx thread is mid-store. NOT "no sample".
    shared_.follow_read_failures.fetch_add(1, std::memory_order_relaxed);
    if (rt_have_) {
      s = rt_last_;  // last valid copy (read in an earlier 1 kHz cycle); its age is checked below as any sample
      kind = FollowStartKind::kKeptCopy;
    } else {
      // nothing kept yet: hold the end of the approach for this cycle and retry at the next one, within read_hold_s_
      if (!std::isfinite(f_wait_since_)) f_wait_since_ = now;
      if (now - f_wait_since_ > read_hold_s_) freezeFollow(FollowFreezeReason::kReadContention, now);
      return;  // mode_ stays kApproach (goal pose, zero twist); follow_started_ stays false
    }
  }
  const bool ok = std::isfinite(s.theta) && std::isfinite(s.center[0]) && std::isfinite(s.center[1]) && std::isfinite(s.center[2]) &&
                  std::isfinite(s.axis[0]) && std::isfinite(s.axis[1]) && std::isfinite(s.axis[2]);
  const Eigen::Vector3d ax(s.axis[0], s.axis[1], s.axis[2]);
  if (!ok || !(ax.norm() > 0.5)) {
    freezeFollow(FollowFreezeReason::kInvalidSample, now);
    return;
  }
  if (now - static_cast<double>(s.t_mono_ns) * 1e-9 > kFollowFreshAgeS) {
    freezeFollow(FollowFreezeReason::kStaleSample, now);
    return;
  }
  shared_.follow_wait_s.store(std::isfinite(f_wait_since_) ? now - f_wait_since_ : 0.0, std::memory_order_relaxed);
  shared_.follow_start_kind.store(static_cast<int>(kind), std::memory_order_release);
  f_c_ = Eigen::Vector3d(s.center[0], s.center[1], s.center[2]);
  f_axis_ = ax.normalized();
  f_p_end_ = last_.p;  // the exact end of the approach (goal)
  f_q_end_ = last_.q;
  const double age = now - static_cast<double>(s.t_mono_ns) * 1e-9;
  f_rate_ = std::isfinite(s.omega_cmd) ? s.omega_cmd : omega_nominal_;
  f_theta_est_ = s.theta + f_rate_ * age;
  f_theta_end_ = f_theta_est_;
  f_t_est_ = now;
  follow_started_ = true;
  mode_ = SourceCmd::kFollow;
}

void TaskSource::stepFollow() {
  SatSample s;
  const double now = clock_();
  f_stale_ = true;
  if (shared_.sat_latest.load(s) && std::isfinite(s.theta)) {
    const double age = now - static_cast<double>(s.t_mono_ns) * 1e-9;
    if (age >= -1e-3 && age <= kFollowFreshAgeS) {
      f_stale_ = false;
      f_rate_ = std::isfinite(s.omega_cmd) ? s.omega_cmd : omega_nominal_;
      f_theta_est_ = s.theta + f_rate_ * age;
      f_t_est_ = now;
    }
  }
  if (f_stale_) {  // open loop from the last fresh estimate, nominal omega
    f_rate_ = omega_nominal_;
    f_theta_est_ += omega_nominal_ * (now - f_t_est_);
    f_t_est_ = now;
  }
  f_dtheta_ = sat::wrapPi(f_theta_est_ - f_theta_end_);
  const Eigen::Quaterniond rot(Eigen::AngleAxisd(f_dtheta_, f_axis_));
  last_.p = f_c_ + rot * (f_p_end_ - f_c_);
  last_.q = (rot * f_q_end_).normalized();
  f_w_ = f_rate_ * f_axis_;
  f_v_ = f_w_.cross(last_.p - f_c_);
}

bool TaskSource::twist(Eigen::Vector3d& v, Eigen::Vector3d& w) const {
  if (mode_ == SourceCmd::kApproach && !approach_done_) return approach_->twist(v, w);
  if (mode_ == SourceCmd::kFollow) {  // rigid rotation with the satellite (follows the stop ramp through omega_cmd)
    v = f_v_;
    w = f_w_;
    return true;
  }
  v.setZero();
  w.setZero();
  return true;
}

// ---- Cycle log --------------------------------------------------------------------------------------------------
void TaskCycleLog::record(const TaskShared& sh, double t_mono_s, const double e_pos[3], const double e_rot[3]) noexcept {
  if (rec_.size() >= rec_.capacity()) return;
  TaskCycleRec r{};
  r.t_mono_s = t_mono_s;
  r.task_state = static_cast<double>(sh.task_state.load(std::memory_order_relaxed));
  r.source_cmd = static_cast<double>(sh.source_cmd.load(std::memory_order_relaxed));
  SatSample s;
  if (sh.sat_latest.load(s)) {
    r.theta_deg = s.theta / kDeg;
    r.sample_age_ms = (t_mono_s - static_cast<double>(s.t_mono_ns) * 1e-9) * 1e3;
  } else {
    r.theta_deg = r.sample_age_ms = std::nan("");
  }
  r.e_pos_m = std::sqrt(e_pos[0] * e_pos[0] + e_pos[1] * e_pos[1] + e_pos[2] * e_pos[2]);
  r.e_rot_rad = std::sqrt(e_rot[0] * e_rot[0] + e_rot[1] * e_rot[1] + e_rot[2] * e_rot[2]);
  rec_.push_back(r);  // capacity reserved: no allocation
}

bool TaskCycleLog::writeCsv(const std::string& path, std::string* err) const {
  std::FILE* f = std::fopen(path.c_str(), "wx");
  if (!f) {
    if (err) *err = "cannot create '" + path + "'";
    return false;
  }
  std::fprintf(f, "t_mono_s,task_state,source_cmd,theta_deg,sample_age_ms,e_pos_m,e_rot_rad\n");
  for (const TaskCycleRec& r : rec_) {
    std::fprintf(f, "%.6f,%.0f,%.0f,%.6f,%.3f,%.6e,%.6e\n", r.t_mono_s, r.task_state, r.source_cmd, r.theta_deg, r.sample_age_ms, r.e_pos_m,
                 r.e_rot_rad);
  }
  std::fclose(f);
  return true;
}

// ---- Action order -----------------------------------------------------------------------------------------------
void executeActions(const MachineActions& a, IActionSink& sink) {
  if (a.send_start) sink.sendStart();
  if (a.send_stop) sink.sendStop(a.stop_hard);  // always BEFORE the gripper (fault: satellite first, hand right after)
  if (a.gripper != GripperCmd::kNone) sink.gripper(a.gripper, a.gripper_id);
}

// ---- Runtime ----------------------------------------------------------------------------------------------------
TaskRuntime::TaskRuntime(const HwTaskConfig& hw, TaskMachine& machine, TaskShared& shared, const StaticValidation& sv, bool yes,
                         const volatile std::sig_atomic_t* abort_flag)
    : hw_(hw), m_(machine), sh_(shared), sv_(sv), yes_(yes), abort_(abort_flag) {
  for (auto& r : rej_) r.store(0);
}

TaskRuntime::~TaskRuntime() {
  quit_.store(true);
  for (std::thread* t : {&rx_, &hb_, &task_, &confirm_}) {
    if (t->joinable()) t->join();
  }
  if (rx_fd_ >= 0) ::close(rx_fd_);
  if (tx_fd_ >= 0) ::close(tx_fd_);
}

void TaskRuntime::start() {
  auto addr = [&](int port) {
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(static_cast<std::uint16_t>(port));
    if (inet_pton(AF_INET, hw_.udp_host.c_str(), &a.sin_addr) != 1) throw std::runtime_error("udp.host is not IPv4: " + hw_.udp_host);
    return a;
  };
  rx_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  tx_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (rx_fd_ < 0 || tx_fd_ < 0) throw std::runtime_error("socket() failed");
  const sockaddr_in ra = addr(hw_.udp_state_port);
  if (::bind(rx_fd_, reinterpret_cast<const sockaddr*>(&ra), sizeof ra) != 0) {
    throw std::runtime_error("cannot bind the satellite state port " + std::to_string(hw_.udp_state_port) + ": " + std::strerror(errno));
  }
  timeval tv{};
  tv.tv_usec = 50000;
  setsockopt(rx_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
  const sockaddr_in ta = addr(hw_.udp_command_port);
  if (::connect(tx_fd_, reinterpret_cast<const sockaddr*>(&ta), sizeof ta) != 0) {
    throw std::runtime_error(std::string("connect() of the command socket failed: ") + std::strerror(errno));
  }
  rx_ = std::thread([this] { rxLoop(); });
  hb_ = std::thread([this] { hbLoop(); });
  task_ = std::thread([this] { taskLoop(); });
  confirm_ = std::thread([this] { confirmLoop(); });
}

void TaskRuntime::finish() {
  sh_.loop_finished.store(true);
  // let the task thread process the end of the loop (fault or DONE, STOP) before stopping it
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  quit_.store(true);
  for (std::thread* t : {&task_, &rx_, &hb_, &confirm_}) {
    if (t->joinable()) t->join();
  }
}

void TaskRuntime::send(std::uint16_t type) {
  sat::CommandPacket c;
  c.type = type;
  c.seq = ++tx_seq_;
  c.t_mono_ns = monoNs();
  std::uint8_t buf[sat::kCommandPacketSize];
  sat::packCommand(c, buf);
  (void)::send(tx_fd_, buf, sizeof buf, MSG_DONTWAIT);  // ECONNREFUSED (no satellite) ignored: the watchdogs act
}

void TaskRuntime::sendStart() { send(static_cast<std::uint16_t>(sat::CommandType::kStart)); }
void TaskRuntime::sendStop(bool hard) {
  send(static_cast<std::uint16_t>(hard ? sat::CommandType::kStop : sat::CommandType::kStopRamp));
}
void TaskRuntime::gripper(GripperCmd c, std::uint32_t id) {
  if (!grip_) return;
  grip_->request(c == GripperCmd::kOpen ? grip::GripperWorker::Kind::kOpen : grip::GripperWorker::Kind::kGrasp, id);
}

SatSnapshotReader::Result SatSnapshotReader::read(const sat::LatestValue<SatSample>& src, SatSample& out) {
  Result r;
  SatSample s;
  sat::LoadStatus st = src.tryLoad(s);
  if (st == sat::LoadStatus::kContended) {
    // One yield, then one last try: if the rx thread was preempted mid-store on this core it can finish. The task
    // thread is not real-time and runs every 2 ms, so a syscall here is harmless (never done in the 1 kHz callback).
    std::this_thread::yield();
    st = src.tryLoad(s);
    if (st == sat::LoadStatus::kOk) ++recovered_;
  }
  if (st == sat::LoadStatus::kOk) {
    last_ = s;
    have_ = true;
    streak_ = 0;
  } else if (st == sat::LoadStatus::kContended) {
    ++failures_;
    ++streak_;
    r.load_failed = true;
  }
  // kEmpty: nothing was ever stored -> have_ stays as it is (false)
  r.have = have_;
  if (have_) out = last_;
  return r;
}

void TaskRuntime::rxLoop() {
  SatFilter filter(hw_.p.T_age_s);
  std::uint8_t buf[512];
  while (!quit_.load()) {
    const ssize_t n = ::recv(rx_fd_, buf, sizeof buf, 0);
    if (n < 0) continue;
    const double now = monoNow();
    sat::StatePacket p;
    if (!sat::unpackState(buf, static_cast<std::size_t>(n), p)) {
      ++rej_[static_cast<int>(SampleVerdict::kNonFinite)];  // malformed datagram: counted as a bad sample
      continue;
    }
    SatSample s;
    s.seq = p.seq;
    s.state = p.state;
    s.faults = p.faults;
    s.flags = p.flags;
    s.t_mono_ns = p.t_mono_ns;
    s.theta = p.theta;
    s.omega_meas = p.omega_meas;
    s.omega_cmd = p.omega_cmd;
    for (int k = 0; k < 3; ++k) {
      s.center[k] = p.center[k];
      s.axis[k] = p.axis[k];
    }
    s.omega_valid = (p.flags & sat::kFlagOmegaValid) != 0;  // omega_meas is used ONLY with this bit
    s.rx_s = now;
    const SampleVerdict v = filter.accept(s, now);
    ++rej_[static_cast<int>(v)];
    if (v == SampleVerdict::kOk) sh_.sat_latest.store(s);
  }
}

void TaskRuntime::hbLoop() {
  const auto period = std::chrono::nanoseconds(static_cast<long long>(1e9 / hw_.hb_send_hz));
  auto next = std::chrono::steady_clock::now();
  HeartbeatGate gate(hw_.hb_stall_s);
  while (!quit_.load()) {
    next += period;
    std::this_thread::sleep_until(next);
    // NOT an independent heartbeat: it needs the machine's enable AND a 1 kHz loop that is still cycling
    if (gate.shouldSend(hb_enabled_.load(), sh_.loop_cycles.load(std::memory_order_relaxed), monoNow())) {
      send(static_cast<std::uint16_t>(sat::CommandType::kHeartbeat));
    }
  }
}

void TaskRuntime::confirmLoop() {
  while (!quit_.load() && !confirm_asked_.load()) std::this_thread::sleep_for(std::chrono::milliseconds(20));
  std::string line;
  while (!quit_.load() && confirm_answer_.load() == 0) {
    pollfd pf{STDIN_FILENO, POLLIN, 0};
    const int r = ::poll(&pf, 1, 100);
    if (r <= 0) continue;
    char c = 0;
    const ssize_t n = ::read(STDIN_FILENO, &c, 1);
    if (n <= 0) {
      confirm_answer_.store(2);  // EOF: no confirmation
      break;
    }
    if (c == '\n') {
      const bool yes = line == "s" || line == "si" || line == "y" || line == "yes";
      confirm_answer_.store(yes ? 1 : 2);
      break;
    }
    line += c;
  }
}

void TaskRuntime::runValidation(const MachineInputs& in) {
  PoseSnap act{};
  sh_.activation.load(act);
  ValidationContext x;
  x.now_unix_s = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  x.tau = sv_.tau;
  x.weights_sha256 = sv_.weights_sha256;
  x.tcp_p = Eigen::Vector3d(act.p[0], act.p[1], act.p[2]);
  x.tcp_q = Eigen::Quaterniond(act.q[0], act.q[1], act.q[2], act.q[3]);
  x.path_checked = sv_.path_checked;
  x.path_ok = sv_.path_ok;
  x.path_detail = sv_.path_detail;
  x.stop_at_fraction = sv_.stop_at_fraction;
  const std::vector<Check> checks = validateCommand(m_.command(), hw_.p, x);
  const Feasibility fe = feasibility(m_.command(), hw_.p, sv_.tau, in.sat.theta);
  std::printf("[task] VALIDATE: theta0 (satellite) %.3f deg, theta_trig %.4f deg, T_c %.4f s, admitted theta0 [%.3f, %.3f] deg, "
              "trigger timeout %.1f s\n",
              in.sat.theta / kDeg, fe.theta_trig / kDeg, fe.T_c, fe.theta0_lo / kDeg, fe.theta0_hi / kDeg, fe.trigger_timeout_s);
  for (const Check& c : checks) std::printf("  [%s] %s: %s\n", c.ok ? "PASS" : (c.blocking ? "FAIL" : "WARN"), c.name.c_str(), c.detail.c_str());
  for (const Check& c : fe.checks) std::printf("  [%s] %s: %s\n", c.ok ? "PASS" : (c.blocking ? "FAIL" : "WARN"), c.name.c_str(), c.detail.c_str());
  std::printf("  portata robot 1: NON verificata (punto di estensione, stadio 3)\n");
  std::fflush(stdout);
  m_.setValidation(checks, fe);
}

void TaskRuntime::taskLoop() {
  std::size_t printed = 0;
  bool final_pass = false;
  double t_final = 0.0;
  SatSnapshotReader sat_reader;
  double t_prev = kNaN, period_max_1s = 0.0, t_health = kNaN;
  std::uint64_t fail_at_last_health = 0;
  bool follow_logged = false;
  for (;;) {
    const bool q = quit_.load();
    MachineInputs in;
    in.now_s = monoNow();
    if (std::isfinite(t_prev)) {
      const double dt = in.now_s - t_prev;
      period_max_1s = std::max(period_max_1s, dt);
      health_.max_task_period_s = std::max(health_.max_task_period_s, dt);
    }
    t_prev = in.now_s;
    in.activation_ready = sh_.activation_ready.load(std::memory_order_acquire);
    PoseSnap ms{};
    if (sh_.measured.load(ms)) {
      in.tcp_p = Eigen::Vector3d(ms.p[0], ms.p[1], ms.p[2]);
      in.tcp_q = Eigen::Quaterniond(ms.q[0], ms.q[1], ms.q[2], ms.q[3]);
    }
    {
      const SatSnapshotReader::Result sr = sat_reader.read(sh_.sat_latest, in.sat);
      in.sat_have = sr.have;
      in.sat_load_failed = sr.load_failed;
      in.sat_load_fail_streak = sat_reader.failStreak();
      health_.load_failures = sat_reader.loadFailures();
      health_.load_recovered = sat_reader.retriesRecovered();
      health_.max_fail_streak = std::max(health_.max_fail_streak, sat_reader.failStreak());
      if (in.sat_have) health_.max_rx_age_s = std::max(health_.max_rx_age_s, in.now_s - in.sat.rx_s);
    }
    in.rejected_seq = rej_[static_cast<int>(SampleVerdict::kSeq)].load();
    in.rejected_clock = rej_[static_cast<int>(SampleVerdict::kClockBack)].load();
    in.rejected_nonfinite = rej_[static_cast<int>(SampleVerdict::kNonFinite)].load();
    in.rejected_age = rej_[static_cast<int>(SampleVerdict::kAge)].load();
    in.approach_started = sh_.approach_started.load(std::memory_order_acquire);
    in.approach_done = sh_.approach_done.load(std::memory_order_acquire);
    in.loop_finished = sh_.loop_finished.load();
    if (sh_.loop_text_ready.load(std::memory_order_acquire)) in.loop_stop_text = sh_.loop_text;
    if (in.loop_finished) {
      in.loop_reason = in.loop_stop_text.empty() ? std::string("loop finished") : in.loop_stop_text;
      if (sh_.exc_text[0] != '\0') in.loop_reason += std::string("; ") + sh_.exc_text;
    }
    in.jitter_fault = sh_.jitter_fault.load();
    in.abort = abort_ != nullptr && *abort_ != 0;
    in.guard_valid = sh_.guard_valid.load(std::memory_order_relaxed);
    in.e_pos_m = sh_.e_pos_m.load(std::memory_order_relaxed);
    in.f_ext_n = sh_.f_ext_n.load(std::memory_order_relaxed);
    if (grip_) {
      const grip::GripperWorker::Status g = grip_->status();
      in.grip_done_id = g.done_id;
      in.grip_done_ok = g.done_ok;
      in.grip_is_grasped = g.is_grasped;
      in.grip_state_valid = g.state_valid;
      in.grip_width = g.state_valid ? g.width : kNaN;
    }

    if (m_.needsValidation() && in.activation_ready && in.sat_have && in.now_s - in.sat.rx_s < hw_.p.sat_timeout_s) runValidation(in);
    if (m_.needsConfirmation()) {
      if (yes_) {
        m_.setConfirmed(true);
      } else if (!confirm_asked_.load()) {
        std::printf("\n[task] VALIDATE OK. Confermi l'avvio della rotazione del satellite e dell'avvicinamento del robot 1? [s/N] ");
        std::fflush(stdout);
        confirm_asked_.store(true);
      } else if (confirm_answer_.load() != 0) {
        m_.setConfirmed(confirm_answer_.load() == 1);
      }
    }
    if (!follow_logged) {  // FOLLOW start / freeze decided by the 1 kHz loop: logged once, with the reason
      const int fr = sh_.follow_freeze_reason.load(std::memory_order_acquire);
      const int fk = sh_.follow_start_kind.load(std::memory_order_acquire);
      if (fr != 0 || fk != 0) {
        char buf[320];
        std::snprintf(buf, sizeof buf, "%s; waited %.1f ms, failed seqlock reads %llu",
                      fr != 0 ? toString(static_cast<FollowFreezeReason>(fr)) : toString(static_cast<FollowStartKind>(fk)),
                      sh_.follow_wait_s.load(std::memory_order_relaxed) * 1e3,
                      static_cast<unsigned long long>(sh_.follow_read_failures.load(std::memory_order_relaxed)));
        m_.note(in.now_s, fr != 0 ? "FOLLOW target FROZEN" : "FOLLOW started", buf);
        follow_logged = true;
      }
    }
    if (!std::isfinite(t_health)) t_health = in.now_s;
    if (in.now_s - t_health >= 1.0) {  // 1 Hz satellite-link health line (events CSV + stdout)
      char buf[320];
      std::snprintf(buf, sizeof buf,
                    "rx_age %s s, accepted %llu, rej seq %llu clock %llu age %llu nonfinite %llu, load_fail %llu (+%llu in 1 s, "
                    "recovered by yield %llu, streak %llu), task period max %.1f ms",
                    in.sat_have ? std::to_string(in.now_s - in.sat.rx_s).c_str() : "none",
                    static_cast<unsigned long long>(rej_[0].load()), static_cast<unsigned long long>(in.rejected_seq),
                    static_cast<unsigned long long>(in.rejected_clock), static_cast<unsigned long long>(in.rejected_age),
                    static_cast<unsigned long long>(in.rejected_nonfinite),
                    static_cast<unsigned long long>(sat_reader.loadFailures()),
                    static_cast<unsigned long long>(sat_reader.loadFailures() - fail_at_last_health),
                    static_cast<unsigned long long>(sat_reader.retriesRecovered()),
                    static_cast<unsigned long long>(sat_reader.failStreak()), period_max_1s * 1e3);
      m_.note(in.now_s, "sat_link", buf);
      fail_at_last_health = sat_reader.loadFailures();
      period_max_1s = 0.0;
      t_health = in.now_s;
    }
    const MachineActions a = m_.update(in);
    sh_.task_state.store(static_cast<int>(m_.state()), std::memory_order_relaxed);
    executeActions(a, *this);
    hb_enabled_.store(a.hb_enable);
    sh_.source_cmd.store(static_cast<int>(a.source), std::memory_order_release);
    if (a.end_loop) sh_.end_requested.store(true, std::memory_order_release);

    const auto& ev = m_.events();
    for (; printed < ev.size(); ++printed) {
      std::printf("[task %.3f] %s%s%s\n", ev[printed].t, ev[printed].name.c_str(), ev[printed].detail.empty() ? "" : ": ",
                  ev[printed].detail.c_str());
    }
    std::fflush(stdout);
    if (final_pass) {
      // the hand must not stay closed on the object: keep serving the machine (bounded) until it has reopened
      if (!(grip_ && m_.gripperHolding() && monoNow() < t_final + hw_.p.open_timeout_s + 1.0)) break;
    } else if (q) {
      final_pass = true;  // one more pass after quit with loop_finished = true
      t_final = monoNow();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  hb_enabled_.store(false);
  if (m_.startSent() && !m_.stopSent()) {
    send(static_cast<std::uint16_t>(sat::CommandType::kStop));
    std::printf("[task] exit with START sent and no STOP: STOP sent now\n");
  }
}

}  // namespace task
}  // namespace real_replay
