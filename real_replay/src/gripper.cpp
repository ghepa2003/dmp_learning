#include "real_replay/gripper.hpp"

#include <time.h>

#include <chrono>
#include <cmath>
#include <stdexcept>

namespace real_replay {
namespace grip {

namespace {
double monoNow() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<double>(ts.tv_sec) + 1e-9 * static_cast<double>(ts.tv_nsec);
}
void sleepS(double s) { std::this_thread::sleep_for(std::chrono::duration<double>(s)); }
}  // namespace

void validateGripperParams(const GripperParams& g) {
  auto bad = [](const char* m) { throw std::runtime_error(std::string("gripper: ") + m); };
  if (!(std::isfinite(g.open_width_m) && g.open_width_m > 0.0 && g.open_width_m <= kMaxTotalOpenM + 1e-9))
    bad("open_width_m must be in (0, 0.080] (total opening, 0.040 per finger)");
  if (!(std::isfinite(g.speed_m_s) && g.speed_m_s > 0.0 && g.speed_m_s <= kMaxSpeedMs)) bad("speed_m_s must be in (0, 0.03] m/s: larger values are refused");
  if (!(std::isfinite(g.force_n) && g.force_n > 0.0 && g.force_n <= kMaxForceN)) bad("force_n must be in (0, 10] N: larger values are refused");
  if (!(std::isfinite(g.grasp_width_m) && g.grasp_width_m >= 0.001 && g.grasp_width_m < g.open_width_m)) bad("grasp_width_m must be in [0.001, open_width_m)");
  if (!(std::isfinite(g.eps_in_m) && g.eps_in_m >= 0.0 && g.eps_in_m <= kMaxEpsM)) bad("eps_in_m must be in [0, 0.01]");
  if (!(std::isfinite(g.eps_out_m) && g.eps_out_m >= 0.0 && g.eps_out_m <= kMaxEpsM)) bad("eps_out_m must be in [0, 0.01]");
  if (!(std::isfinite(g.squeeze_s) && g.squeeze_s >= 0.0)) bad("squeeze_s must be >= 0");
  if (!(std::isfinite(g.open_timeout_s) && g.open_timeout_s > 0.0)) bad("open_timeout_s must be > 0");
  if (!(std::isfinite(g.poll_period_s) && g.poll_period_s >= 0.01)) bad("poll_period_s must be >= 0.01");
}

// ---- FakeGripper ------------------------------------------------------------------------------------------------
bool FakeGripper::move(double width_m, double) {
  const double t0 = monoNow();
  sleepS(move_delay_s);
  if (throw_on_move) throw std::runtime_error("fake gripper: move() exception");
  std::lock_guard<std::mutex> l(m_);
  if (move_ok) {
    width_ = width_m;
    grasped_ = false;
  }
  calls_.push_back({"move", width_m, t0, monoNow()});
  return move_ok;
}

bool FakeGripper::grasp(double width_m, double speed, double force, double ein, double eout) {
  const double t0 = monoNow();
  sleepS(grasp_delay_s);
  if (throw_on_grasp) throw std::runtime_error("fake gripper: grasp() exception");
  std::lock_guard<std::mutex> l(m_);
  last_force = force;
  last_speed = speed;
  last_eps_in = ein;
  last_eps_out = eout;
  if (grasp_ok) {
    width_ = width_after_grasp;
    grasped_ = true;
  }
  calls_.push_back({"grasp", width_m, t0, monoNow()});
  return grasp_ok;
}

GripperState FakeGripper::readState() {
  std::lock_guard<std::mutex> l(m_);
  GripperState s;
  s.valid = true;
  s.width = width_;
  s.max_width = max_width;
  s.is_grasped = grasped_ && !drop_object;
  return s;
}

std::vector<FakeGripper::Call> FakeGripper::calls() const {
  std::lock_guard<std::mutex> l(m_);
  return calls_;
}

// ---- GripperWorker ----------------------------------------------------------------------------------------------
GripperWorker::GripperWorker(std::unique_ptr<IGripper> g, const GripperParams& p) : g_(std::move(g)), p_(p) {
  if (!g_) throw std::invalid_argument("GripperWorker: no gripper");
}

GripperWorker::~GripperWorker() {
  {
    std::lock_guard<std::mutex> l(m_);
    quit_ = true;
  }
  cv_.notify_all();
  if (th_.joinable()) th_.join();
}

void GripperWorker::start() { th_ = std::thread([this] { loop(); }); }

void GripperWorker::request(Kind k, std::uint32_t id) {
  std::lock_guard<std::mutex> l(m_);
  // a pending (not started) grasp is replaced by anything newer; an opening is never replaced by a grasp
  if (pending_ && pending_kind_ == Kind::kOpen && k == Kind::kGrasp) return;
  pending_ = true;
  pending_kind_ = k;
  pending_id_ = id;
  cv_.notify_all();
}

GripperWorker::Status GripperWorker::status() const {
  std::lock_guard<std::mutex> l(m_);
  return st_;
}

void GripperWorker::loop() {
  std::unique_lock<std::mutex> l(m_);
  while (!quit_) {
    if (!pending_) {
      l.unlock();
      GripperState gs;
      std::string err;
      try {
        gs = g_->readState();
      } catch (const std::exception& e) {
        err = e.what();
      }
      l.lock();
      if (!err.empty()) {
        st_.state_valid = false;
        st_.error = err;
      } else {
        st_.state_valid = gs.valid;
        st_.is_grasped = gs.is_grasped;
        st_.width = gs.width;
      }
      if (!pending_ && !quit_) cv_.wait_for(l, std::chrono::duration<double>(p_.poll_period_s));
      continue;
    }
    const Kind k = pending_kind_;
    const std::uint32_t id = pending_id_;
    pending_ = false;
    st_.busy_id = id;
    l.unlock();
    bool ok = false;
    std::string err;
    try {
      ok = (k == Kind::kOpen) ? g_->move(p_.open_width_m, p_.speed_m_s)
                              : g_->grasp(p_.grasp_width_m, p_.speed_m_s, p_.force_n, p_.eps_in_m, p_.eps_out_m);
    } catch (const std::exception& e) {
      err = e.what();
    }
    GripperState gs;
    try {
      gs = g_->readState();
    } catch (const std::exception&) {
    }
    l.lock();
    st_.busy_id = 0;
    st_.done_id = id;
    st_.done_ok = ok && err.empty();
    st_.error = err.empty() && !ok ? (k == Kind::kOpen ? "move() returned false" : "grasp() returned false") : err;
    if (gs.valid) {
      st_.state_valid = true;
      st_.is_grasped = gs.is_grasped;
      st_.width = gs.width;
    }
  }
}

}  // namespace grip
}  // namespace real_replay
