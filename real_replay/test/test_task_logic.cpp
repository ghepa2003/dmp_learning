// Unit tests of the stage-2 task layer logic (include/real_replay/task_logic.hpp). No robot, no sockets.
//   test_task_logic <hw_config.yaml> <grasp_command_v2.yaml>
// 1) trigger on a synthetic phase theta(t) = theta0 + omega t (omega < 0 and > 0, crossing of +-180, phase jumps)
// 2) input filter (seq, clock, age, NaN)
// 3) feasibility A-E (same numbers as hw.py plan)
// 4) state machine: nominal run to DONE, and fault injection (lost / old / NaN / non-increasing packets, duplicate
//    START and command, heartbeat lost on the satellite, omega out of band, jitter, loop ended, trigger timeout,
//    infeasible command, abort in every state): final state, STOP sent, process "alive" (no end_loop unless Ctrl-C
//    or fault_linger_s).
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>

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
static std::string f(const char* fmt, double a = 0, double b = 0, double c = 0) {
  char buf[300];
  std::snprintf(buf, sizeof buf, fmt, a, b, c);
  return buf;
}
constexpr double D = sat::kDeg;

// ---- 1) trigger ---------------------------------------------------------------------------------------------------
static void testTrigger() {
  std::printf("trigger on a synthetic phase\n");
  for (double w_deg : {-2.0, 2.0, -3.0, 3.0}) {
    const double w = w_deg * D;
    const double theta_trig = -41.0 * D;
    PhaseTrigger tr(theta_trig, w, 90.0 * D);
    const double theta0 = w < 0 ? -10.0 * D : -80.0 * D;
    int fired = 0;
    double th_fire = 0.0;
    for (int k = 0; k < 4000; ++k) {  // 40 s at 100 Hz
      const double th = sat::unwrapPhase(theta0 + w * k * 0.01);
      if (tr.update(th).fired) {
        ++fired;
        th_fire = th;
      }
    }
    const double err = (th_fire - theta_trig) / D;
    check(fired == 1 && std::fabs(err) <= std::fabs(w_deg) * 0.01 + 1e-9,
          f("omega %+.0f deg/s: exactly one firing, theta at firing - theta_trig = %+.4f deg (<= |omega|*10 ms)", w_deg, err));
  }
  {  // crossing of the +-180 jump of wrapPi: theta_trig = wrapPi(-181.5) = +178.5 deg
    const double w = -2.0 * D;
    PhaseTrigger tr(sat::wrapPi(-181.5 * D), w, 90.0 * D);
    int fired = 0;
    double th_fire = 0;
    for (int k = 0; k < 2000; ++k) {
      const double th = sat::unwrapPhase(-170.0 * D + w * k * 0.01);
      if (tr.update(th).fired) {
        ++fired;
        th_fire = th;
      }
    }
    check(fired == 1 && std::fabs(th_fire / D + 181.5) <= 0.03, f("theta_trig behind the +-180 jump (branch -181.5): one firing at %.3f deg", th_fire / D));
  }
  {  // phase jump: theta jumps across theta_trig + 180 (|d| ~ 180): never fires
    PhaseTrigger tr(-41.0 * D, -2.0 * D, 90.0 * D);
    bool spurious = false;
    const double seq[] = {-41.0 + 179.0, -41.0 - 179.0, -41.0 + 175.0, -41.0 - 170.0, -41.0 + 100.0, -41.0 - 95.0};
    for (double th : seq) spurious |= tr.update(sat::unwrapPhase(th * D)).fired;
    check(!spurious, "phase jumps through theta_trig + 180 deg (|d| >= 90): no spurious firing");
    PhaseTrigger t2(-41.0 * D, -2.0 * D, 90.0 * D);
    t2.update(-30.0 * D);                          // before the threshold (omega < 0: theta decreasing)
    const bool jump_fire = t2.update(-150.0 * D).fired;  // jump of -120 deg: d = -109 > 90 in magnitude
    check(!jump_fire, "jump from -30 to -150 deg (d -109 deg): no firing");
    PhaseTrigger t3(-41.0 * D, -2.0 * D, 90.0 * D);
    t3.update(-40.0 * D);
    check(!t3.update(-39.0 * D).fired, "wrong direction (theta increasing with omega < 0): no firing");
  }
}

// ---- 2) filter ----------------------------------------------------------------------------------------------------
static SatSample mk(std::uint32_t seq, double t, double theta) {
  SatSample s;
  s.seq = seq;
  s.t_mono_ns = static_cast<std::uint64_t>(t * 1e9);
  s.theta = theta;
  s.state = 1;
  return s;
}

static void testFilter() {
  std::printf("input filter\n");
  SatFilter fl(0.1);
  check(fl.accept(mk(1, 10.0, -0.1), 10.005) == SampleVerdict::kOk, "fresh sample: ok");
  check(fl.accept(mk(1, 10.01, -0.1), 10.015) == SampleVerdict::kSeq, "same seq: refused");
  check(fl.accept(mk(0, 10.02, -0.1), 10.025) == SampleVerdict::kSeq, "seq going back: refused");
  check(fl.accept(mk(3, 9.0, -0.1), 9.005) == SampleVerdict::kClockBack, "t_mono going back: refused (clock backwards)");
  check(fl.accept(mk(4, 10.0, -0.1), 10.2) == SampleVerdict::kAge, "age 200 ms >= T_age 100 ms: refused");
  check(fl.accept(mk(5, 10.3, -0.1), 10.29) == SampleVerdict::kAge, "negative age (stamp in the future): refused");
  check(fl.accept(mk(6, 10.4, std::nan("")), 10.401) == SampleVerdict::kNonFinite, "NaN theta: refused");
  check(fl.accept(mk(7, 10.5, 2.0), 10.501) == SampleVerdict::kNonFinite, "theta off the branch [-270, 90): refused");
  SatSample s = mk(8, 10.6, -0.1);
  s.omega_valid = true;
  s.omega_meas = std::nan("");
  check(fl.accept(s, 10.601) == SampleVerdict::kNonFinite, "omega_valid with NaN omega: refused");
  s.seq = 9;
  s.omega_valid = false;
  check(fl.accept(s, 10.601) == SampleVerdict::kOk, "omega NaN without omega_valid: ok (omega not used)");
}

// ---- 3) feasibility -----------------------------------------------------------------------------------------------
static TaskParams g_params;
static GraspCommand g_cmd;

static void testFeasibility() {
  std::printf("feasibility A-E (hw_config + example command, contact_delay_s %.3f)\n", g_params.contact_delay_s);
  GraspCommand c = g_cmd;
  c.theta_star = -83.9 * D;
  c.omega = -2.0 * D;
  const Feasibility fe = feasibility(c, g_params, 21.466, -10.0 * D);
  check(std::fabs(fe.theta_trig / D - (-83.9 + 2.0 * 21.466)) < 1e-9, f("theta_trig %.4f deg (= -83.9 + 2*21.466)", fe.theta_trig / D));
  check(fe.ok && std::fabs(fe.theta0_lo / D - (fe.theta_trig / D + 16.0)) < 1e-9 && std::fabs(fe.theta0_hi / D + 5.0) < 1e-9,
        f("nominal OK, admitted theta0 [%.3f, %.3f] deg (A = 16 deg)", fe.theta0_lo / D, fe.theta0_hi / D));
  check(std::fabs(fe.theta_end / D + 109.9) < 1e-9, f("theta_end %.3f deg", fe.theta_end / D));
  const Feasibility fa = feasibility(c, g_params, 21.466, -30.0 * D);
  check(!fa.ok && !fa.checks[0].ok, "theta0 too close to the trigger: A fails");
  const Feasibility fc = feasibility(c, g_params, 21.466, 2.0 * D);
  check(!fc.ok && !fc.checks[2].ok, "theta0 outside the window: C fails");
  GraspCommand z = c;
  z.omega = 0.0;
  check(!feasibility(z, g_params, 21.466, -10.0 * D).ok, "omega = 0: not feasible");
  GraspCommand e = c;
  e.omega = -2.5 * D;
  const Feasibility fe2 = feasibility(e, g_params, 21.466, -5.5 * D);
  check(!fe2.checks[4].ok && !fe2.checks[4].blocking, "|omega| 2.5 outside {2,3}: E is a warning only");
}

// ---- 4) machine ---------------------------------------------------------------------------------------------------
/// Synthetic satellite + loop around the machine, 1 ms ticks of the task thread, 100 Hz satellite samples.
/// LatestValue whose writer can be frozen mid-store (seq left odd), as if the rx thread were preempted inside store():
/// every read then fails with kContended until release().
struct StuckValue : sat::LatestValue<SatSample> {
  bool held = false;
  void holdOpen() {
    if (!held) seq_.fetch_add(1);
    held = true;
  }
  void release() {
    if (held) seq_.fetch_add(1);
    held = false;
  }
};

struct Harness {
  TaskParams p;
  double tau = 24.474;
  TaskMachine m;
  SatFilter filter;
  double t = 100.0;
  double sat_theta = -10.0 * D;
  double omega;
  std::uint32_t seq = 0;
  std::uint32_t sat_state = 1;  // WAIT_START
  std::uint32_t sat_faults = 0;
  double t_run0 = -1.0;
  bool have = false;
  SatSample last;
  std::uint64_t rej[5] = {0, 0, 0, 0, 0};
  bool approach_started = false, approach_done = false;
  double t_approach = -1.0;
  bool loop_finished = false, jitter = false, abort = false;
  int n_start = 0, n_stop = 0;
  bool end_loop = false;
  SourceCmd source = SourceCmd::kHold;
  bool produce = true;
  std::function<void(SatSample&)> corrupt;  // fault injection on the next datagram
  double age_offset = 0.0;                    // datagrams stamped this much in the past
  bool age_ramp = false;                      // age_offset grows by 1 ms per datagram (delivery delay)
  std::string file_sha;                       // sha256 of the weights file (empty = the command's)
  bool via_seqlock = false;                   // inputs read like TaskRuntime: accepted -> LatestValue -> SatSnapshotReader
  StuckValue lv;
  SatSnapshotReader reader;

  Harness(const TaskParams& pp, const GraspCommand& c, bool submit = true)
      : p(pp), m(pp, 24.474, sat::kFaultPhaseMismatch), filter(pp.T_age_s), omega(c.omega) {
    if (submit) m.submitCommand(c);
  }

  void satStep() {
    if (sat_state == 2) {  // RUNNING: smoothstep start ramp, then constant omega
      const double tr = t - t_run0;
      const double T1 = p.sat_ramp_s;
      const double x = std::min(1.0, tr / T1);
      const double d = tr < T1 ? T1 * (x * x * x - 0.5 * x * x * x * x) : T1 / 2.0 + (tr - T1);
      sat_theta = -10.0 * D + omega * d;
    }
    if (!produce) return;
    if (age_ramp) age_offset += 0.001;
    SatSample s;
    s.seq = ++seq;
    s.t_mono_ns = static_cast<std::uint64_t>((t - age_offset) * 1e9);
    s.theta = sat::unwrapPhase(sat_theta);
    s.state = sat_state;
    s.faults = sat_faults;
    const bool valid = sat_state == 2 && (t - t_run0) >= p.sat_ramp_s + p.sat_omega_window_s;
    s.omega_valid = valid;
    s.omega_meas = valid ? omega : std::nan("");
    s.rx_s = t;
    if (corrupt) {
      corrupt(s);
      corrupt = nullptr;
    }
    const SampleVerdict v = filter.accept(s, t);
    ++rej[static_cast<int>(v)];
    if (v == SampleVerdict::kOk) {
      last = s;
      have = true;
      if (via_seqlock && !lv.held) lv.store(s);  // a frozen writer publishes nothing
    }
  }

  void tick() {
    t += 0.001;
    if (static_cast<long>(std::llround(t * 1000.0)) % 10 == 0) satStep();
    if (m.needsValidation() && have) {
      ValidationContext x;
      x.now_unix_s = m.command().stamp_unix_s + 1.0;
      x.tau = tau;
      x.weights_sha256 = file_sha.empty() ? m.command().weights_sha256 : file_sha;
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
    if (via_seqlock) {
      const SatSnapshotReader::Result r = reader.read(lv, in.sat);
      in.sat_have = r.have;
      in.sat_load_failed = r.load_failed;
      in.sat_load_fail_streak = reader.failStreak();
    } else {
      in.sat_have = have;
      in.sat = last;
    }
    in.rejected_seq = rej[1];
    in.rejected_clock = rej[2];
    in.rejected_age = rej[3];
    in.rejected_nonfinite = rej[4];
    in.approach_started = approach_started;
    in.approach_done = approach_done;
    in.loop_finished = loop_finished;
    in.loop_reason = "test";
    in.jitter_fault = jitter;
    in.abort = abort;
    const MachineActions a = m.update(in);
    if (a.send_start) {
      ++n_start;
      if (sat_state == 1) {
        sat_state = 2;
        t_run0 = t;
      }
    }
    if (a.send_stop) {
      ++n_stop;
      if (sat_state == 2) sat_state = 3;
    }
    source = a.source;
    end_loop = a.end_loop;
    // the 1 kHz loop: TaskSource starts the approach at the first cycle that sees kApproach
    if (source == SourceCmd::kApproach && !approach_started) {
      approach_started = true;
      t_approach = t;
    }
    if (approach_started && !approach_done && source == SourceCmd::kApproach && t - t_approach >= tau) approach_done = true;
  }

  void runUntil(const std::function<bool()>& cond, double max_s) {
    const double t_end = t + max_s;
    while (t < t_end && !cond()) tick();
  }
  void runFor(double s) {
    const double t_end = t + s;
    while (t < t_end) tick();
  }
  bool in(TaskState s) const { return m.state() == s; }
};

static GraspCommand nominalCmd() {
  GraspCommand c = g_cmd;
  c.theta_star = -90.0 * D;
  c.omega = -2.0 * D;
  c.contact_time_s = 24.474;
  return c;
}

static void expectFault(const char* name, Harness& h, TaskState expected) {
  const bool ok = h.in(expected) && h.n_stop == 1 && !h.end_loop && h.source == SourceCmd::kFreeze;
  check(ok, std::string(name) + ": " + toString(h.m.state()) + " (expected " + toString(expected) + "), STOP sent " + std::to_string(h.n_stop) +
                ", alive (no end_loop), target frozen; reason: " + h.m.faultReason());
}

static void testMachine() {
  std::printf("state machine: nominal\n");
  {
    Harness h(g_params, nominalCmd());
    h.runUntil([&] { return h.in(TaskState::kDone); }, 120.0);
    const RunFigures& fg = h.m.figures();
    check(h.in(TaskState::kDone) && h.n_start == 1 && h.n_stop == 1 && h.end_loop && h.m.exitCode() == 0,
          "nominal: VALIDATE -> START -> WAIT_TRIGGER -> APPROACH -> HOLD_SAFE -> DONE, one START, one STOP, end_loop, exit 0");
    const double e_trig = (fg.theta_trig - fg.theta_at_trigger) / D;
    check(std::fabs(e_trig) <= 0.02 + 1e-9, f("trigger error %.4f deg (<= |omega|*10 ms)", e_trig));
    const double e_ph = std::remainder(fg.theta_at_approach_end - fg.theta_star, 2 * M_PI) / D;
    check(std::fabs(e_ph) <= 0.05, f("phase error at the end of the approach %.4f deg (synthetic, contact_delay 0)", e_ph));
    check(std::isfinite(fg.t_omega_valid) && fg.t_omega_valid - fg.t_start_sent >= h.p.sat_ramp_s + h.p.sat_omega_window_s - 0.011,
          f("trigger armed only with omega_valid (%.2f s after START)", fg.t_omega_valid - fg.t_start_sent));
    bool stubs_never = true;
    for (const Event& e : h.m.events()) {
      for (TaskState s : {TaskState::kCloseFollow, TaskState::kSqueeze, TaskState::kRelease, TaskState::kFaultGrasp}) {
        if (e.name == std::string("state ") + toString(s)) stubs_never = false;
      }
    }
    check(stubs_never, "stage-3 stub states never entered");
    check(transitionAllowed(TaskState::kApproach, TaskState::kCloseFollow) && transitionAllowed(TaskState::kRelease, TaskState::kHoldSafe) &&
              transitionAllowed(TaskState::kSqueeze, TaskState::kFaultGrasp) && !transitionAllowed(TaskState::kWaitTrigger, TaskState::kHoldSafe),
          "stage-3 transitions declared (APPROACH->CLOSE_FOLLOW->SQUEEZE->RELEASE->HOLD_SAFE, ->FAULT_GRASP)");
  }

  std::printf("satellite watchdog: seqlock read failures (writer frozen mid-store)\n");
  auto toTrigger0 = [](Harness& h) { h.runUntil([&] { return h.in(TaskState::kWaitTrigger); }, 10.0); };
  auto hasReason = [](const Harness& h, const char* key) { return h.m.faultReason().find(key) != std::string::npos; };
  {  // (a) short freezes, many of them: the kept copy stays fresh -> no fault
    Harness h(g_params, nominalCmd());
    h.via_seqlock = true;
    toTrigger0(h);
    for (int k = 0; k < 40; ++k) {  // 40 x (freeze 50 ms, run 50 ms) = 4 s, far beyond sat_timeout_s in total
      h.lv.holdOpen();
      h.runFor(0.05);
      h.lv.release();
      h.runFor(0.05);
    }
    check(h.in(TaskState::kWaitTrigger) && h.reader.loadFailures() > 1000,
          "(a) repeated 50 ms read failures over 4 s: still WAIT_TRIGGER, no fault (" + std::to_string(h.reader.loadFailures()) +
              " failed reads, state " + toString(h.m.state()) + ")");
  }
  {  // (a') one freeze shorter than the timeout
    Harness h(g_params, nominalCmd());
    h.via_seqlock = true;
    toTrigger0(h);
    h.lv.holdOpen();
    h.runFor(0.8 * g_params.sat_timeout_s);
    check(h.in(TaskState::kWaitTrigger) && h.reader.failStreak() > 0,
          f("(a') reads failing for %.2f s (< sat_timeout_s): no fault, kept copy used", 0.8 * g_params.sat_timeout_s));
    // (b) the same freeze continues: the kept copy gets older than sat_timeout_s -> fault, message 'rx age'
    h.runFor(0.4 * g_params.sat_timeout_s + 0.05);
    expectFault("(b) reads failing beyond sat_timeout_s: kept copy too old", h, TaskState::kFaultNoMotion);
    check(hasReason(h, "rx age") && hasReason(h, "seqlock contention"), "  ... reason names 'rx age' and the read failures: " + h.m.faultReason());
  }
  {  // (c1) never a sample: no datagram at all
    Harness h(g_params, nominalCmd());
    h.via_seqlock = true;
    h.produce = false;
    h.runFor(g_params.sat_timeout_s + 0.5);
    check(h.in(TaskState::kFaultNoMotion) && h.n_start == 0 && hasReason(h, "never received"),
          std::string("(c1) no datagram ever: ") + toString(h.m.state()) + ", no START; reason: " + h.m.faultReason());
  }
  {  // (c2) never a sample: the very first store is frozen (seq 0 -> 1) -> kContended without any copy
    Harness h(g_params, nominalCmd());
    h.via_seqlock = true;
    h.lv.holdOpen();
    h.runFor(g_params.sat_timeout_s + 0.5);
    check(h.in(TaskState::kFaultNoMotion) && h.n_start == 0 && hasReason(h, "never received") && h.reader.loadFailures() > 0,
          std::string("(c2) first store frozen: ") + toString(h.m.state()) + "; reason: " + h.m.faultReason());
  }
  {  // the plain timeout still works through the reader: satellite silent > sat_timeout_s
    Harness h(g_params, nominalCmd());
    h.via_seqlock = true;
    toTrigger0(h);
    h.produce = false;
    h.runFor(g_params.sat_timeout_s + 0.5);
    expectFault("satellite silent through the reader", h, TaskState::kFaultNoMotion);
    check(hasReason(h, "rx age") && !hasReason(h, "seqlock"), "  ... reason 'rx age' without read failures: " + h.m.faultReason());
  }
  {  // a nominal run through the reader reaches DONE
    Harness h(g_params, nominalCmd());
    h.via_seqlock = true;
    h.runUntil([&] { return h.in(TaskState::kDone); }, 120.0);
    check(h.in(TaskState::kDone) && h.reader.loadFailures() == 0, "nominal run through LatestValue + SatSnapshotReader: DONE, 0 read failures");
  }
  {  // LatestValue::tryLoad statuses
    StuckValue v;
    SatSample o;
    const bool empty = v.tryLoad(o) == sat::LoadStatus::kEmpty;
    SatSample x;
    x.seq = 7;
    v.store(x);
    const bool ok = v.tryLoad(o) == sat::LoadStatus::kOk && o.seq == 7;
    v.holdOpen();
    const bool cont = v.tryLoad(o) == sat::LoadStatus::kContended && !v.load(o);
    v.release();
    check(empty && ok && cont && v.load(o), "LatestValue::tryLoad: kEmpty before any store, kOk after, kContended with the writer frozen");
  }

  std::printf("state machine: fault injection\n");
  auto toTrigger = [](Harness& h) { h.runUntil([&] { return h.in(TaskState::kWaitTrigger); }, 10.0); };
  auto toArmed = [](Harness& h) { h.runUntil([&] { return std::isfinite(h.m.figures().t_omega_valid); }, 20.0); };
  auto toApproach = [](Harness& h) { h.runUntil([&] { return h.in(TaskState::kApproach); }, 60.0); };
  {
    Harness h(g_params, nominalCmd());
    toTrigger(h);
    h.produce = false;
    h.runFor(2.5);
    expectFault("packets lost > 2 s in WAIT_TRIGGER", h, TaskState::kFaultNoMotion);
  }
  {
    Harness h(g_params, nominalCmd());
    toApproach(h);
    h.produce = false;
    h.runFor(0.3);
    expectFault("packets lost in APPROACH (sample age > T_age_s)", h, TaskState::kFaultMotion);
  }
  {
    Harness h(g_params, nominalCmd());
    toTrigger(h);
    h.age_ramp = true;  // growing delivery delay: stamps still increasing, age grows past T_age -> refused
    h.runFor(3.5);
    expectFault("old datagrams (age >= T_age, stamps monotonic): refused, then 'no valid datagram'", h, TaskState::kFaultNoMotion);
    check(h.rej[3] > 100 && h.rej[2] == 0, "  ... refused as 'age' (" + std::to_string(h.rej[3]) + "), never as clock-backwards");
  }
  {
    Harness h(g_params, nominalCmd());
    toTrigger(h);
    h.runFor(0.1);
    h.corrupt = [](SatSample& s) { s.theta = std::nan(""); };
    h.runFor(0.05);
    expectFault("NaN theta", h, TaskState::kFaultNoMotion);
  }
  {
    Harness h(g_params, nominalCmd());
    toApproach(h);
    h.corrupt = [&](SatSample& s) { s.seq = h.seq - 5; };
    h.runFor(0.05);
    expectFault("seq not increasing during the approach", h, TaskState::kFaultMotion);
  }
  {
    Harness h(g_params, nominalCmd());
    toTrigger(h);
    h.runFor(0.1);
    h.corrupt = [](SatSample& s) { s.t_mono_ns -= 50000000ull; };
    h.runFor(0.05);
    expectFault("clock going backwards (t_mono decreasing)", h, TaskState::kFaultNoMotion);
  }
  {
    Harness h(g_params, nominalCmd());
    toTrigger(h);
    h.m.requestStartAgain(h.t);
    h.m.submitCommand(nominalCmd());
    h.runUntil([&] { return h.in(TaskState::kDone); }, 120.0);
    bool dup_start = false, dup_cmd = false;
    for (const Event& e : h.m.events()) {
      dup_start |= e.name == "duplicate_start";
      dup_cmd |= e.name == "duplicate_command";
    }
    check(h.n_start == 1 && dup_start && dup_cmd && h.in(TaskState::kDone),
          "duplicate START and duplicate GraspCommand: ignored and logged, run completes, START sent once");
  }
  {
    Harness h(g_params, nominalCmd());
    toArmed(h);
    h.sat_state = 3;
    h.sat_faults = sat::kFaultHeartbeat;
    h.runFor(0.05);
    expectFault("satellite lost our heartbeat (faults bit, STOPPING)", h, TaskState::kFaultNoMotion);
  }
  {
    Harness h(g_params, nominalCmd());
    toTrigger(h);
    h.runUntil([&] { return (h.t - h.t_run0) > h.p.sat_ramp_s + h.p.sat_omega_window_s - 0.05; }, 20.0);
    h.omega = -2.3 * D;  // satellite faster than the command: omega_meas out of band
    h.runFor(0.1);
    expectFault("omega_meas out of band (2.3 vs 2.0 deg/s)", h, TaskState::kFaultNoMotion);
  }
  {
    Harness h(g_params, nominalCmd());
    toApproach(h);
    h.jitter = true;
    h.runFor(0.01);
    expectFault("cycle jitter during the approach", h, TaskState::kFaultMotion);
  }
  {
    Harness h(g_params, nominalCmd());
    toApproach(h);
    h.runFor(1.0);
    h.loop_finished = true;  // e.g. tracking guard -> controlled stop of the loop
    h.runFor(0.01);
    expectFault("control loop ended during the approach (tracking guard)", h, TaskState::kFaultMotion);
  }
  {
    TaskParams p = g_params;
    p.trigger_timeout_margin_s = -30.0;  // force a timeout before the crossing
    Harness h(p, nominalCmd());
    h.runUntil([&] { return h.m.state() == TaskState::kFaultNoMotion; }, 60.0);
    expectFault("trigger timeout", h, TaskState::kFaultNoMotion);
  }
  {
    GraspCommand c = nominalCmd();
    c.theta_star = -60.0 * D;  // theta_trig = -11: theta0 = -10 too close (A fails)
    Harness h(g_params, c);
    h.runFor(1.0);
    check(h.in(TaskState::kFaultNoMotion) && h.n_start == 0 && h.n_stop == 1 && !h.end_loop,
          "infeasible command: FAULT_NOMOTION in VALIDATE, NO START, STOP sent, alive; " + h.m.faultReason());
  }
  {
    Harness h(g_params, nominalCmd());
    h.file_sha = "0123456789abcdef";
    h.runFor(1.0);
    check(h.in(TaskState::kFaultNoMotion) && h.n_start == 0 && h.n_stop == 1, "sha256 mismatch: FAULT_NOMOTION in VALIDATE, no START");
  }

  std::printf("state machine: abort (Ctrl-C) in every state\n");
  struct AbortCase {
    const char* name;
    std::function<void(Harness&)> go;
    TaskState expected;
    int starts;
  };
  const AbortCase cases[] = {
      {"WAIT_CMD", [](Harness&) {}, TaskState::kFaultNoMotion, 0},
      {"VALIDATE", [](Harness& h) { h.runUntil([&] { return h.in(TaskState::kValidate); }, 1.0); }, TaskState::kFaultNoMotion, 0},
      {"WAIT_TRIGGER", [&](Harness& h) { toTrigger(h); h.runFor(1.0); }, TaskState::kFaultNoMotion, 1},
      {"APPROACH", [&](Harness& h) { toApproach(h); h.runFor(2.0); }, TaskState::kFaultMotion, 1},
      {"HOLD_SAFE", [](Harness& h) { h.runUntil([&] { return h.in(TaskState::kHoldSafe); }, 120.0); h.runFor(0.5); }, TaskState::kFaultMotion, 1},
  };
  for (const AbortCase& ac : cases) {
    Harness h(g_params, nominalCmd(), std::string(ac.name) != "WAIT_CMD");
    ac.go(h);
    const TaskState before = h.m.state();
    h.abort = true;
    h.runFor(0.01);
    check(h.in(ac.expected) && h.n_stop == 1 && h.n_start == ac.starts && h.end_loop && h.source == SourceCmd::kFreeze,
          std::string("abort in ") + toString(before) + " -> " + toString(h.m.state()) + ", STOP sent, controlled stop requested");
  }
  {
    Harness h(g_params, nominalCmd());
    toTrigger(h);
    h.produce = false;
    h.runFor(2.5);
    h.runFor(10.0);
    const bool alive = h.in(TaskState::kFaultNoMotion) && !h.end_loop;
    h.abort = true;
    h.runFor(0.01);
    check(alive && h.end_loop && h.n_stop == 1, "abort in FAULT_NOMOTION: stayed alive 10 s, then Ctrl-C -> controlled stop (STOP not repeated)");
  }
  {
    TaskParams p = g_params;
    p.fault_linger_s = 1.0;
    Harness h(p, nominalCmd());
    toTrigger(h);
    h.produce = false;
    h.runFor(2.5);
    const bool before = !h.end_loop;
    h.runFor(1.1);
    check(before && h.end_loop, "fault_linger_s = 1: alive in the fault state, controlled stop after 1 s");
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
    g_params.fault_linger_s = 0.0;
    g_cmd = loadGraspCommand(argv[2]);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 2;
  }
  std::printf("command %s: theta* %.3f deg, omega %.3f deg/s (degrees converted once in loadGraspCommand)\n", g_cmd.path.c_str(),
              g_cmd.theta_star / D, g_cmd.omega / D);
  testTrigger();
  testFilter();
  testFeasibility();
  testMachine();
  std::printf("%s (%d failure(s))\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
