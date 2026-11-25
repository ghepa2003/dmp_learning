// Stage 3 (A+B): gripper interface, grasp/follow/release state machine, action order. No robot, no sockets.
//   test_task_grasp <hw_config.yaml> <grasp_command_v2.yaml>
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include "real_replay/gripper.hpp"
#include "real_replay/satellite_phase.hpp"
#include "real_replay/satellite_udp.hpp"
#include "real_replay/task_logic.hpp"
#include "real_replay/task_runtime.hpp"

using namespace real_replay;
using namespace real_replay::task;

static int g_fail = 0;
static void check(bool ok, const std::string& what) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}
constexpr double D = sat::kDeg;
static TaskParams g_params;
static GraspCommand g_cmd;

// ---- gripper params -----------------------------------------------------------------------------------------------
static void testParams() {
  std::printf("gripper params\n");
  grip::GripperParams g;
  g.enabled = true;
  auto refused = [&](const std::function<void(grip::GripperParams&)>& mod) {
    grip::GripperParams x = g;
    mod(x);
    try {
      grip::validateGripperParams(x);
    } catch (const std::runtime_error&) {
      return true;
    }
    return false;
  };
  check(!refused([](grip::GripperParams&) {}), "defaults (open 0.080 total, 0.02 m/s, 5 N) accepted");
  check(refused([](grip::GripperParams& x) { x.force_n = 10.5; }), "force 10.5 N refused (hard limit 10 N)");
  check(!refused([](grip::GripperParams& x) { x.force_n = 10.0; }), "force 10 N accepted");
  check(refused([](grip::GripperParams& x) { x.speed_m_s = 0.031; }), "speed 0.031 m/s refused (max 0.03)");
  check(refused([](grip::GripperParams& x) { x.open_width_m = 0.09; }), "open width 0.09 m refused (Hand: 0.080 total)");
  check(refused([](grip::GripperParams& x) { x.eps_in_m = 0.02; }), "eps_in 0.02 refused (max 0.01)");
  check(refused([](grip::GripperParams& x) { x.force_n = 0.0; }), "force 0 refused");
}

// ---- worker + FakeGripper (real thread) --------------------------------------------------------------------------
static void waitDone(grip::GripperWorker& w, std::uint32_t id, double max_s = 3.0) {
  for (double t = 0; t < max_s; t += 0.005) {
    if (w.status().done_id >= id) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}
static void testWorker() {
  std::printf("GripperWorker + FakeGripper (dedicated thread)\n");
  grip::GripperParams gp;
  gp.enabled = true;
  gp.poll_period_s = 0.01;
  {
    auto fg = std::make_unique<grip::FakeGripper>();
    grip::FakeGripper* raw = fg.get();
    grip::GripperWorker w(std::move(fg), gp);
    w.start();
    w.request(grip::GripperWorker::Kind::kOpen, 1);
    waitDone(w, 1);
    grip::GripperWorker::Status s = w.status();
    check(s.done_id == 1 && s.done_ok && !s.is_grasped && std::fabs(s.width - 0.080) < 1e-9, "open: done_ok, width 0.080");
    w.request(grip::GripperWorker::Kind::kGrasp, 2);
    waitDone(w, 2);
    s = w.status();
    check(s.done_id == 2 && s.done_ok && s.is_grasped, "grasp: done_ok, is_grasped");
    check(std::fabs(raw->last_force - 5.0) < 1e-12 && std::fabs(raw->last_speed - 0.02) < 1e-12 && std::fabs(raw->last_eps_in - 0.01) < 1e-12,
          "grasp() called with the configured force 5 N, speed 0.02 m/s, eps 0.01 m");
    auto calls = raw->calls();
    check(calls.size() == 2 && calls[0].what == "move" && std::fabs(calls[0].width - 0.080) < 1e-12 && calls[1].what == "grasp" &&
              std::fabs(calls[1].width - 0.009) < 1e-12,
          "calls: move(0.080) then grasp(0.009)");
  }
  {
    auto fg = std::make_unique<grip::FakeGripper>();
    fg->grasp_ok = false;
    grip::GripperWorker w(std::move(fg), gp);
    w.start();
    w.request(grip::GripperWorker::Kind::kGrasp, 1);
    waitDone(w, 1);
    check(w.status().done_id == 1 && !w.status().done_ok, "grasp() false -> done_ok false");
  }
  {
    auto fg = std::make_unique<grip::FakeGripper>();
    fg->throw_on_grasp = true;
    grip::GripperWorker w(std::move(fg), gp);
    w.start();
    w.request(grip::GripperWorker::Kind::kGrasp, 1);
    waitDone(w, 1);
    check(w.status().done_id == 1 && !w.status().done_ok && !w.status().error.empty(), "exception in grasp() -> done_ok false, error text kept");
    w.request(grip::GripperWorker::Kind::kOpen, 2);
    waitDone(w, 2);
    check(w.status().done_id == 2 && w.status().done_ok, "worker still alive after the exception: opening works");
  }
  {  // an opening is never replaced by a later grasp request that arrives while it is pending
    auto fg = std::make_unique<grip::FakeGripper>();
    fg->grasp_delay_s = 0.3;
    grip::GripperWorker w(std::move(fg), gp);
    w.start();
    w.request(grip::GripperWorker::Kind::kGrasp, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));  // grasp(1) running (blocking)
    w.request(grip::GripperWorker::Kind::kOpen, 2);              // fault: open requested while grasp() blocks
    w.request(grip::GripperWorker::Kind::kGrasp, 3);             // must NOT replace the pending opening
    waitDone(w, 2, 3.0);
    check(w.status().done_id == 2 && w.status().done_ok, "open requested during a blocking grasp() runs right after it, not replaced by a later grasp");
  }
}

// ---- action order ------------------------------------------------------------------------------------------------
struct RecSink : IActionSink {
  std::vector<std::string> log;
  void sendStart() override { log.push_back("START"); }
  void sendStop(bool hard) override { log.push_back(hard ? "STOP" : "STOP_RAMP"); }
  void gripper(GripperCmd c, std::uint32_t id) override { log.push_back(std::string(c == GripperCmd::kOpen ? "OPEN" : "GRASP") + std::to_string(id)); }
};
static void testOrder() {
  std::printf("executeActions order\n");
  RecSink s;
  MachineActions a;
  a.gripper = GripperCmd::kOpen;
  a.gripper_id = 7;
  a.send_stop = true;
  a.stop_hard = true;  // fault: hard STOP
  executeActions(a, s);
  check(s.log.size() == 2 && s.log[0] == "STOP" && s.log[1] == "OPEN7", "hard STOP is executed BEFORE the gripper opening");
  RecSink s3;
  MachineActions n;
  n.send_stop = true;  // normal: STOP_RAMP
  executeActions(n, s3);
  check(s3.log.size() == 1 && s3.log[0] == "STOP_RAMP", "a normal stop is the STOP_RAMP command");
  RecSink s2;
  MachineActions b;
  b.send_start = true;
  b.gripper = GripperCmd::kOpen;
  b.gripper_id = 1;
  executeActions(b, s2);
  check(s2.log.size() == 2 && s2.log[0] == "START" && s2.log[1] == "OPEN1", "START before the initial opening");
}

// ---- machine harness ---------------------------------------------------------------------------------------------
struct Harness {
  TaskParams p;
  TaskMachine m;
  SatFilter filter;
  double t = 0.0, tau = 24.474, omega;
  double sat_theta = -10.0 * D;
  std::uint32_t seq = 0, sat_state = 1, sat_faults = 0;
  double t_run0 = -1.0, t_stop_sent = -1.0, t_stopped = -1.0, t_open_cmd_after_stop = -1.0;
  bool have = false, produce = true;
  SatSample last;
  std::uint64_t rej[5] = {0, 0, 0, 0, 0};
  bool approach_started = false, approach_done = false, abort = false, loop_finished = false;
  double t_approach = -1.0;
  int n_stop = 0;
  bool end_loop = false;
  SourceCmd source = SourceCmd::kHold;
  // gripper model
  double open_delay = 0.3, grasp_delay = 1.0;
  bool open_ok = true, grasp_ok = true, grasp_never_returns = false, object_lost_after = false;
  std::uint32_t g_done_id = 0;
  bool g_done_ok = false, g_grasped = false;
  struct Pend { std::uint32_t id; GripperCmd c; double t_done; };
  std::vector<Pend> pend;
  struct Cmd { std::uint32_t id; GripperCmd c; double t; bool same_update_as_stop; };
  std::vector<Cmd> cmds;
  double omega_meas_override = std::nan("");
  bool guard_on = false;
  double e_pos = 0.0, f_ext = 0.0;
  std::string loop_stop_text, loop_reason_text = "test";

  Harness(const TaskParams& pp, const GraspCommand& c) : p(pp), m(pp, 24.474, sat::kFaultPhaseMismatch), filter(pp.T_age_s), omega(c.omega) {
    m.submitCommand(c);
  }

  void satStep() {
    if (sat_state == 2) {
      const double tr = t - t_run0, T1 = p.sat_ramp_s, x = std::min(1.0, tr / T1);
      const double d = tr < T1 ? T1 * (x * x * x - 0.5 * x * x * x * x) : T1 / 2.0 + (tr - T1);
      sat_theta = -10.0 * D + omega * d;
      last_theta_run = sat_theta;
    } else if (sat_state == 3) {  // stop ramp: linear deceleration over sat_stop_ramp_s
      const double ts = t - t_stop_sent;
      if (ts >= p.sat_stop_ramp_s) {
        sat_state = 4;
        t_stopped = t;
      } else {
        sat_theta = last_theta_run + omega * (ts - 0.5 * ts * ts / p.sat_stop_ramp_s);
      }
    }
    if (!produce) return;
    SatSample s;
    s.seq = ++seq;
    s.t_mono_ns = static_cast<std::uint64_t>(t * 1e9);
    s.theta = sat::unwrapPhase(sat_theta);
    s.state = sat_state;
    s.faults = sat_faults | (sat_state >= 3 && stop_cmd ? sat::kFaultStopCommand : 0u);
    const bool valid = sat_state == 2 && (t - t_run0) >= p.sat_ramp_s + p.sat_omega_window_s;
    s.omega_valid = valid;
    s.omega_meas = valid ? (std::isfinite(omega_meas_override) ? omega_meas_override : omega) : std::nan("");
    s.rx_s = t;
    const SampleVerdict v = filter.accept(s, t);
    ++rej[static_cast<int>(v)];
    if (v == SampleVerdict::kOk) {
      last = s;
      have = true;
    }
  }
  double last_theta_run = -10.0 * D;
  bool stop_cmd = false;

  void tick() {
    t += 0.001;
    if (static_cast<long>(std::llround(t * 1000.0)) % 10 == 0) satStep();
    for (std::size_t i = 0; i < pend.size();) {
      if (t >= pend[i].t_done) {
        g_done_id = pend[i].id;
        if (pend[i].c == GripperCmd::kOpen) {
          g_done_ok = open_ok;
          if (open_ok) g_grasped = false;
        } else {
          g_done_ok = grasp_ok;
          g_grasped = grasp_ok;
        }
        pend.erase(pend.begin() + static_cast<long>(i));
      } else {
        ++i;
      }
    }
    if (m.needsValidation() && have) {
      ValidationContext x;
      x.now_unix_s = m.command().stamp_unix_s + 1.0;
      x.tau = tau;
      x.weights_sha256 = m.command().weights_sha256;
      x.tcp_p = m.command().start_p;
      x.tcp_q = m.command().start_q;
      x.path_checked = x.path_ok = true;
      m.setValidation(validateCommand(m.command(), p, x), feasibility(m.command(), p, tau, last.theta));
    }
    if (m.needsConfirmation()) m.setConfirmed(true);
    MachineInputs in;
    in.now_s = t;
    in.activation_ready = true;
    in.tcp_p = m.command().start_p;
    in.tcp_q = m.command().start_q;
    in.sat_have = have;
    in.sat = last;
    in.rejected_seq = rej[1];
    in.rejected_clock = rej[2];
    in.rejected_age = rej[3];
    in.rejected_nonfinite = rej[4];
    in.approach_started = approach_started;
    in.approach_done = approach_done;
    in.loop_finished = loop_finished;
    in.loop_reason = loop_reason_text;
    in.loop_stop_text = loop_stop_text;
    in.abort = abort;
    in.grip_done_id = g_done_id;
    in.grip_done_ok = g_done_ok;
    in.grip_is_grasped = g_grasped && !(object_lost_after && t_grasp_ok_seen > 0 && t - t_grasp_ok_seen > 0.5);
    in.grip_state_valid = true;
    in.grip_width = g_grasped ? 0.009 : 0.080;
    in.guard_valid = guard_on;
    in.e_pos_m = e_pos;
    in.f_ext_n = f_ext;
    const MachineActions a = m.update(in);
    if (m.state() == TaskState::kSqueeze && t_grasp_ok_seen < 0) t_grasp_ok_seen = t;
    if (a.send_start && sat_state == 1) {
      sat_state = 2;
      t_run0 = t;
    }
    if (a.send_stop) {
      ++n_stop;
      stop_cmd = true;
      t_stop_sent = t;
      if (sat_state == 2) {
        sat_state = 3;
      } else if (sat_state == 1) {
        sat_state = 4;
        t_stopped = t;
      }
    }
    if (a.gripper != GripperCmd::kNone) {
      cmds.push_back({a.gripper_id, a.gripper, t, a.send_stop});
      if (a.gripper == GripperCmd::kOpen && t_stop_sent > 0 && t_open_cmd_after_stop < 0) t_open_cmd_after_stop = t;
      const double delay = a.gripper == GripperCmd::kOpen ? open_delay : (grasp_never_returns ? 1e9 : grasp_delay);
      pend.push_back({a.gripper_id, a.gripper, t + delay});
    }
    source = a.source;
    end_loop = a.end_loop;
    if (source == SourceCmd::kApproach && !approach_started) {
      approach_started = true;
      t_approach = t;
    }
    if (approach_started && !approach_done && source == SourceCmd::kApproach && t - t_approach >= tau) approach_done = true;
  }
  double t_grasp_ok_seen = -1.0;

  void runUntil(const std::function<bool()>& cond, double max_s) {
    const double t_end = t + max_s;
    while (t < t_end && !cond()) tick();
  }
  void runFor(double s) {
    const double t_end = t + s;
    while (t < t_end) tick();
  }
  bool in(TaskState s) const { return m.state() == s; }
  int countCmd(GripperCmd c) const {
    int n = 0;
    for (const Cmd& x : cmds) n += x.c == c;
    return n;
  }
};

static TaskParams grasping() {
  TaskParams p = g_params;
  p.grasp_enabled = true;
  p.squeeze_s = 2.0;
  p.open_timeout_s = 10.0;
  p.fault_linger_s = 0.0;
  return p;
}
static GraspCommand nominalCmd() {
  GraspCommand c = g_cmd;
  c.theta_star = -90.0 * D;
  c.omega = -2.0 * D;
  c.contact_time_s = 24.474;
  return c;
}

static void testMachine() {
  std::printf("grasp machine: nominal run (VALIDATE .. RELEASE .. DONE)\n");
  {
    Harness h(grasping(), nominalCmd());
    std::vector<TaskState> seen;
    TaskState prev = h.m.state();
    while (h.t < 200.0 && !h.in(TaskState::kDone)) {
      h.tick();
      if (h.m.state() != prev) {
        prev = h.m.state();
        seen.push_back(prev);
      }
    }
    const std::vector<TaskState> want = {TaskState::kValidate,   TaskState::kWaitTrigger, TaskState::kApproach, TaskState::kCloseFollow,
                                         TaskState::kSqueeze,    TaskState::kHold,        TaskState::kRelease,  TaskState::kHoldSafe,
                                         TaskState::kDone};
    check(seen == want, "sequence VALIDATE, WAIT_TRIGGER, APPROACH, CLOSE_FOLLOW, SQUEEZE, HOLD, RELEASE, HOLD_SAFE, DONE");
    check(h.m.exitCode() == 0 && h.end_loop && h.n_stop == 1, "exit 0, one STOP, controlled stop of the loop");
    check(h.cmds.size() == 3 && h.cmds[0].c == GripperCmd::kOpen && h.cmds[1].c == GripperCmd::kGrasp && h.cmds[2].c == GripperCmd::kOpen,
          "gripper commands: OPEN (before the approach), GRASP (at AT_END), OPEN (release)");
    check(h.cmds[0].t < h.t_approach, "initial opening commanded before the approach starts");
    check(h.t_open_cmd_after_stop >= h.t_stopped - 1e-9 && h.t_stopped > h.t_stop_sent,
          "release: the hand opens only AFTER the satellite reports STOPPED (" + std::to_string(h.t_open_cmd_after_stop - h.t_stop_sent) +
              " s after the STOP, ramp " + std::to_string(h.p.sat_stop_ramp_s) + " s)");
    const RunFigures& f = h.m.figures();
    check(std::fabs((f.t_release - f.t_grasp_ok) - h.p.T_follow_s) < 0.02, "HOLD ends T_follow_s after the grasp succeeded (worst case = plan condition D)");
    check(f.grasp_width_m > 0.0 && f.grasp_width_m < 0.02, "grasp width recorded");
  }
  std::printf("grasp machine: faults\n");
  {  // grasp() fails
    Harness h(grasping(), nominalCmd());
    h.grasp_ok = false;
    h.runUntil([&] { return h.in(TaskState::kFaultGrasp); }, 200.0);
    h.runFor(1.0);
    check(h.in(TaskState::kFaultGrasp) && h.m.exitCode() == 8 && h.n_stop == 1, "grasp() false -> FAULT_GRASP, exit 8, STOP sent");
    check(h.countCmd(GripperCmd::kOpen) >= 2 && h.cmds.back().c == GripperCmd::kOpen && h.cmds.back().same_update_as_stop,
          "the opening is commanded in the SAME update as the STOP (STOP first in the execution order, no wait for the satellite)");
    check(!h.end_loop, "process stays alive in FAULT_GRASP");
  }
  {  // timeout T_close_max
    Harness h(grasping(), nominalCmd());
    h.grasp_never_returns = true;
    h.runUntil([&] { return h.in(TaskState::kFaultGrasp); }, 200.0);
    const double dt = h.t - h.m.figures().t_grasp_cmd;
    check(h.in(TaskState::kFaultGrasp) && std::fabs(dt - h.p.T_close_max_s) < 0.05 && h.m.faultReason().find("close timeout") != std::string::npos,
          "no grasp within T_close_max_s -> FAULT_GRASP after " + std::to_string(dt) + " s: " + h.m.faultReason());
    check(h.countCmd(GripperCmd::kOpen) == 2 && h.n_stop == 1, "STOP + opening commanded");
  }
  {  // fault in HOLD (omega out of band): STOP first, hand opens right after, without waiting for STOPPED
    Harness h(grasping(), nominalCmd());
    h.runUntil([&] { return h.in(TaskState::kHold); }, 200.0);
    h.omega_meas_override = h.omega + 0.5 * D;
    h.runFor(0.05);
    check(h.in(TaskState::kFaultGrasp) && h.n_stop == 1, "omega out of band in HOLD -> FAULT_GRASP, STOP sent");
    check(h.cmds.back().c == GripperCmd::kOpen && h.cmds.back().same_update_as_stop && h.t_stopped < 0.0 && h.sat_state == 3,
          "opening commanded in the same update as the STOP while the satellite is still decelerating (no wait for STOPPED)");
  }
  {  // grasp lost during SQUEEZE
    Harness h(grasping(), nominalCmd());
    h.object_lost_after = true;
    h.runUntil([&] { return h.in(TaskState::kFaultGrasp); }, 200.0);
    check(h.in(TaskState::kFaultGrasp) && h.m.faultReason().find("grasp lost") != std::string::npos, "is_grasped false in SQUEEZE -> FAULT_GRASP: " + h.m.faultReason());
  }
  {  // fault in APPROACH: motion fault, the hand stays open (never commanded again)
    Harness h(grasping(), nominalCmd());
    h.runUntil([&] { return h.in(TaskState::kApproach); }, 200.0);
    h.runFor(1.0);
    h.omega_meas_override = h.omega + 0.5 * D;
    h.runFor(0.05);
    check(h.in(TaskState::kFaultMotion) && h.m.exitCode() == 6 && h.countCmd(GripperCmd::kOpen) == 1 && h.countCmd(GripperCmd::kGrasp) == 0,
          "fault in APPROACH -> FAULT_MOTION (exit 6): no grasp, no further gripper command (hand stays open)");
  }
  {  // the exact reason of the loop stop goes to the events (once), and into the fault text; satellite fault bits are named
    Harness h(grasping(), nominalCmd());
    h.runUntil([&] { return h.in(TaskState::kApproach); }, 200.0);
    h.runFor(1.0);
    h.loop_stop_text = "t=56.577 s cycle 56370: joint_speed: joint 5: |dq| = 1.008 rad/s > 1.000 rad/s for 3 consecutive cycles";
    h.runFor(0.05);
    h.loop_reason_text = h.loop_stop_text + "; libfranka/control exception: joint_velocity_violation";
    h.loop_finished = true;
    h.runFor(0.05);
    int n_ev = 0;
    std::string detail;
    for (const Event& e : h.m.events()) {
      if (e.name == "control loop stop") {
        ++n_ev;
        detail = e.detail;
      }
    }
    check(n_ev == 1 && detail.find("joint 5") != std::string::npos && detail.find("1.008") != std::string::npos,
          "event 'control loop stop' written once with joint and value: " + detail);
    check(h.in(TaskState::kFaultMotion) && h.m.faultReason().find("joint_speed") != std::string::npos &&
              h.m.faultReason().find("joint_velocity_violation") != std::string::npos,
          "fault text carries the guard and the libfranka exception: " + h.m.faultReason());
  }
  {  // satellite fault word 0x8: named bit, not the old misleading "bit 1 = our heartbeat lost"
    Harness h(grasping(), nominalCmd());
    h.runUntil([&] { return h.in(TaskState::kApproach); }, 200.0);
    h.sat_faults = sat::kFaultSafety;
    h.runFor(0.2);
    check(h.in(TaskState::kFaultMotion) && h.m.faultReason().find("0x8") != std::string::npos &&
              h.m.faultReason().find("bit3 SafetyGuard") != std::string::npos && h.m.faultReason().find("heartbeat lost") == std::string::npos,
          "satellite faults 0x8 -> names bit3 (SafetyGuard of robot 2): " + h.m.faultReason());
  }
  {  // fault before the approach
    Harness h(grasping(), nominalCmd());
    h.runUntil([&] { return h.in(TaskState::kWaitTrigger); }, 50.0);
    h.runFor(1.0);
    h.produce = false;
    h.runFor(2.5);
    check(h.in(TaskState::kFaultNoMotion) && h.m.exitCode() == 5 && h.countCmd(GripperCmd::kGrasp) == 0 && h.countCmd(GripperCmd::kOpen) == 1,
          "satellite lost in WAIT_TRIGGER -> FAULT_NOMOTION (exit 5), hand stays open");
  }
  {  // initial opening fails
    Harness h(grasping(), nominalCmd());
    h.open_ok = false;
    h.runUntil([&] { return h.in(TaskState::kFaultNoMotion); }, 100.0);
    check(h.in(TaskState::kFaultNoMotion) && h.m.faultReason().find("opening failed") != std::string::npos && h.n_stop == 1,
          "initial opening fails -> FAULT_NOMOTION before any motion: " + h.m.faultReason());
  }
  {  // Ctrl-C in HOLD
    Harness h(grasping(), nominalCmd());
    h.runUntil([&] { return h.in(TaskState::kHold); }, 200.0);
    h.abort = true;
    h.runFor(0.05);
    check(h.in(TaskState::kFaultGrasp) && h.cmds.back().c == GripperCmd::kOpen && h.cmds.back().same_update_as_stop, "Ctrl-C in HOLD -> STOP then opening");
  }
  std::printf("grasp machine: guard after the grasp (K*e_pos, F_ext)\n");
  {
    TaskParams p = grasping();
    p.guard_enabled = true;
    Harness h(p, nominalCmd());
    h.guard_on = true;
    h.e_pos = 0.012;  // 24 N-eq: would trip, but only AFTER the grasp starts (APPROACH is not guarded)
    h.runUntil([&] { return h.in(TaskState::kApproach); }, 200.0);
    h.runFor(0.5);
    check(h.in(TaskState::kApproach), "guard not applied during the approach (e_pos 12 mm ignored)");
    h.e_pos = 0.0;
    h.runUntil([&] { return h.in(TaskState::kHold); }, 200.0);
    h.e_pos = 0.005;  // 10 N-eq: warn (>= 8), no fault (< 16)
    h.runFor(0.5);
    bool warned = false;
    for (const Event& e : h.m.events()) warned = warned || e.name == "guard WARN";
    check(h.in(TaskState::kHold) && warned, "5 mm (10 N-eq): WARN event, no fault");
    h.e_pos = 0.012;
    h.runFor(0.02);
    h.e_pos = 0.0;
    h.runFor(0.1);
    check(h.in(TaskState::kHold), "a 20 ms spike above the fault threshold is debounced (50 ms)");
    h.e_pos = 0.009;  // 18 N-eq >= 16, sustained
    h.runFor(0.2);
    check(h.in(TaskState::kFaultGrasp) && h.m.faultReason().find("guard") != std::string::npos && h.n_stop == 1,
          "sustained 9 mm (18 N-eq) -> FAULT_GRASP: " + h.m.faultReason());
    check(h.cmds.back().c == GripperCmd::kOpen && h.cmds.back().same_update_as_stop, "guard fault: STOP then opening in the same update");
  }
  {
    TaskParams p = grasping();
    p.guard_enabled = true;
    Harness h(p, nominalCmd());
    h.guard_on = true;
    h.runUntil([&] { return h.in(TaskState::kSqueeze); }, 200.0);
    h.f_ext = 25.0;  // >= 20 N
    h.runFor(0.2);
    check(h.in(TaskState::kFaultGrasp) && h.m.faultReason().find("F_ext") != std::string::npos, "|F_ext| 25 N -> FAULT_GRASP: " + h.m.faultReason());
    check(h.m.faultReason().find("|F_ext| = 25.0 N") != std::string::npos, "the fault reason reports the F_ext value");
  }
  {
    TaskParams p = grasping();
    p.guard_enabled = true;
    Harness h(p, nominalCmd());
    h.guard_on = false;  // dry-run: fixed state, e_pos meaningless
    h.e_pos = 1.0;
    h.runUntil([&] { return h.in(TaskState::kDone); }, 200.0);
    check(h.in(TaskState::kDone), "guard_valid false (dry-run): inputs ignored, run completes");
  }
  {  // a fault AFTER the normal stop ramp was commanded sends a HARD stop on top of it
    Harness h(grasping(), nominalCmd());
    h.runUntil([&] { return h.in(TaskState::kRelease); }, 200.0);
    h.runFor(0.3);
    h.abort = true;
    h.runFor(0.05);
    check(h.in(TaskState::kFaultGrasp) && h.n_stop == 2, "fault during RELEASE: STOP_RAMP already sent, then a hard STOP is added (n_stop 2)");
  }
  {  // stage-2 behaviour preserved when the gripper is disabled
    TaskParams p = g_params;
    p.grasp_enabled = false;
    Harness h(p, nominalCmd());
    h.runUntil([&] { return h.in(TaskState::kDone); }, 120.0);
    check(h.in(TaskState::kDone) && h.cmds.empty(), "gripper disabled: APPROACH -> HOLD_SAFE -> DONE, no gripper command");
  }
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s <hw_config.yaml> <grasp_command_v2.yaml>\n", argv[0]);
    return 2;
  }
  try {
    const HwTaskConfig hw = loadHwTaskConfig(argv[1]);
    g_params = hw.p;
    g_cmd = loadGraspCommand(argv[2]);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 2;
  }
  testParams();
  testWorker();
  testOrder();
  testMachine();
  std::printf("%s (%d failure(s))\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
