// Stage 3 (E, F, G): heartbeat gate (the heartbeat stops with the control loop), post-grasp guard, optional start quaternion,
// config of the gripper / guard sections.
//   test_task_safety <hw_config.yaml> <grasp_command_v2.yaml>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

#include "real_replay/satellite_phase.hpp"
#include "real_replay/task_logic.hpp"

using namespace real_replay;
using namespace real_replay::task;

static int g_fail = 0;
static void check(bool ok, const std::string& what) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}
constexpr double D = sat::kDeg;

static void testHeartbeatGate() {
  std::printf("heartbeat gate: the heartbeat follows the control loop, not a free-running timer\n");
  HeartbeatGate g(0.1);
  std::uint64_t cyc = 0;
  double t = 0.0;
  int sent_alive = 0;
  for (int i = 0; i < 1000; ++i) {  // 1 s, loop cycling at 1 kHz, heartbeat decisions at 20 Hz
    t += 0.001;
    ++cyc;
    if (i % 50 == 0 && g.shouldSend(true, cyc, t)) ++sent_alive;
  }
  check(sent_alive == 20, "loop cycling: heartbeats sent (" + std::to_string(sent_alive) + " of 20 decisions)");
  // the loop stops (hang, exception, reflex): the counter freezes
  int sent_after = 0;
  double t_last_sent = -1.0;
  for (int i = 0; i < 400; ++i) {  // 0.4 s more, 20 Hz decisions, counter NOT incremented
    t += 0.001;
    if (i % 50 == 0 && g.shouldSend(true, cyc, t)) {
      ++sent_after;
      t_last_sent = t;
    }
  }
  check(sent_after <= 3 && t_last_sent - (t - 0.4) <= 0.1 + 0.0015  /* +1 ms: first decision after the freeze */,
        "loop frozen: heartbeat stops within hb_stall_s = 0.1 s (" + std::to_string(sent_after) + " more heartbeats, last " +
            std::to_string(t_last_sent - (t - 0.4)) + " s after the freeze)");
  check(!g.shouldSend(true, cyc, t + 1.0), "loop frozen for 1.4 s: no heartbeat");
  check(!g.shouldSend(false, cyc + 1, t + 2.0), "machine disabled (fault): no heartbeat even with a cycling loop");
  check(g.shouldSend(true, cyc + 2, t + 2.01), "loop resumed and machine enabled: heartbeat again");
  // the satellite side (T_hb 0.5 s -> ramp) is covered end to end by hw/test (satellite_e2e test 2 and task_e2e case 7)
}

static void testHwConfig(const char* path) {
  std::printf("hw_config sections\n");
  const HwTaskConfig hw = loadHwTaskConfig(path);
  check(!hw.grip.enabled, "gripper.enabled is false in the shipped hw_config (stage-2 behaviour unless the operator enables it)");
  check(std::fabs(hw.grip.open_width_m - 0.080) < 1e-12 && std::fabs(hw.grip.speed_m_s - 0.02) < 1e-12 && std::fabs(hw.grip.force_n - 5.0) < 1e-12,
        "gripper defaults: open 0.080 m total, 0.02 m/s, 5 N");
  check(hw.grip.host == hw.robot1_ip && !hw.grip.host.empty(), "gripper host defaults to robot1.ip (" + hw.grip.host + ")");
  check(hw.p.guard_enabled && std::fabs(hw.p.guard_feq_fault_n - 16.0) < 1e-12 && std::fabs(hw.p.guard_fext_fault_n - 20.0) < 1e-12,
        "task.guard proposed values loaded (K*e_pos 8/16 N, F_ext 10/20 N)");
  check(std::fabs(hw.hb_stall_s - 0.1) < 1e-12, "task.hb_stall_s = 0.1");
  check(std::fabs(hw.p.sat_stop_ramp_s - 2.0) < 1e-12, "robot2.control.stop_ramp_s = 2 s read for the STOPPED timeout");
}

static void testStartQuaternion(const char* cmd_path) {
  std::printf("start_pose.quaternion_wxyz optional\n");
  const std::string tmp = "/tmp/test_task_safety_cmd_noq.yaml";
  {
    std::ofstream o(tmp);
    // keep the YAML valid: start_pose is a flow/mapping; rewrite as a block with position only
    o << "format: grasp_command_v2\nstatus: SELECTED\nframe_id: world\nstamp_unix_s: 1764000000.0\ntheta_star_deg: -83.9\ncontact_time_s: 21.466\n"
         "omega_deg_s: -2.0\nweights_sha256: \"abc\"\nstart_pose:\n  position_m: [0.000002, 0.432494, 0.772246]\n"
         "goal_pose:\n  position_m: [0.0, 0.5125, 0.7722]\n  quaternion_wxyz: [0.494585, -0.505353, -0.505353, -0.494593]\nk: 0\nwrist_flip: false\n";
  }
  const GraspCommand c = loadGraspCommand(tmp);
  check(!c.start_q_present && c.start_p.allFinite(), "loader accepts a start_pose without quaternion (start_q_present = false)");
  const GraspCommand c0 = loadGraspCommand(cmd_path);
  check(c0.start_q_present, "the shipped command has the quaternion (start_q_present = true)");
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s <hw_config.yaml> <grasp_command_v2.yaml>\n", argv[0]);
    return 2;
  }
  try {
    testHeartbeatGate();
    testHwConfig(argv[1]);
    testStartQuaternion(argv[2]);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "exception: %s\n", e.what());
    return 2;
  }
  std::printf("%s (%d failure(s))\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
