// Unit tests of include/real_replay/satellite_phase.hpp and of the hw_config loader. No robot.
//   test_satellite_phase <hw_config.yaml>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>

#include "real_replay/hw_config.hpp"
#include "real_replay/satellite_phase.hpp"

using namespace real_replay::sat;

static int g_fail = 0;
static void check(bool ok, const std::string& what) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}
static bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

static void testUnwrap() {
  std::printf("unwrap on the branch [-270, +90) deg\n");
  const double e = 1e-9;
  check(near(unwrapPhase(-179.0 * kDeg), -179.0 * kDeg, e), "-179 -> -179");
  check(near(unwrapPhase(181.0 * kDeg), -179.0 * kDeg, e), "+181 -> -179 (crossing +-180 is continuous on the branch)");
  check(near(unwrapPhase(179.0 * kDeg), -181.0 * kDeg, e), "+179 -> -181 (wrapPi would give +179)");
  check(near(unwrapPhase(-180.0 * kDeg), -180.0 * kDeg, e), "-180 (window edge) stays -180");
  check(near(unwrapPhase(180.0 * kDeg), -180.0 * kDeg, e), "+180 -> -180 (window edge)");
  check(near(unwrapPhase(0.0), 0.0, e), "0 (window edge) stays 0");
  check(near(unwrapPhase(90.0 * kDeg), -270.0 * kDeg, e), "+90 -> -270 (upper end excluded)");
  check(near(unwrapPhase(-270.0 * kDeg), -270.0 * kDeg, e), "-270 included");
  check(near(unwrapPhase(89.999 * kDeg), 89.999 * kDeg, 1e-9), "89.999 stays");
  check(near(unwrapPhase(-630.0 * kDeg), 90.0 * kDeg - 360.0 * kDeg, e), "-630 -> -270");
  // a sweep across the whole window with wrapPi-style inputs must be continuous on the branch
  bool cont = true;
  double prev = unwrapPhase(wrapPi(5.0 * kDeg));
  for (double th = 5.0; th >= -185.0; th -= 0.1) {
    const double u = unwrapPhase(wrapPi(th * kDeg));
    if (std::fabs(u - prev) > 0.11 * kDeg) cont = false;
    prev = u;
  }
  check(cont, "sweep +5 -> -185 deg through the +-180 jump of wrapPi: continuous after unwrap");
  check(near(wrapPi(-180.0 * kDeg), 180.0 * kDeg, e) && near(wrapPi(190.0 * kDeg), -170.0 * kDeg, e), "wrapPi in (-pi, pi]");
}

static void testFkAndPhase() {
  std::printf("DH FK and phase on the validated pose\n");
  const Q7 qv = {-90.0 * kDeg, -35.456 * kDeg, 0.0, -93.701 * kDeg, 0.0, 58.246 * kDeg, -135.0 * kDeg};
  const Eigen::Isometry3d F = pandaFlangeFk(qv);
  const Eigen::Vector3d p_exp(0.0, -0.255, 0.8035);
  check((F.translation() - p_exp).norm() < 1e-3, "validated pose: flange (0, -0.255, 0.8035) within 1 mm, err " +
                                                     std::to_string((F.translation() - p_exp).norm() * 1e3) + " mm");
  check(F.linear().col(2).dot(-Eigen::Vector3d::UnitZ()) > 0.99999, "flange z points down");
  PhaseParams pp;
  pp.base2_in_base1 = BaseTransform::fromYaw({0.0, 0.975, 0.0}, 0.0);
  const SatGeom g = computeSatGeom(pp, qv);
  check((g.center - Eigen::Vector3d(0.0, 0.720, 0.700)).norm() < 1e-3, "centre (0, 0.720, 0.700) in frame 1 within 1 mm");
  check((g.axis_up - Eigen::Vector3d::UnitZ()).norm() < 1e-4, "axis_up = +z");
  check(near(g.theta_flange, 0.0, 1e-6), "theta_flange = 0 at q7 = -135 deg");
  double worst = 0.0;
  for (double q7 = -135.0; q7 <= 45.0; q7 += 5.0) {
    Q7 q = qv;
    q[6] = q7 * kDeg;
    const SatGeom gg = computeSatGeom(pp, q);
    worst = std::max(worst, std::fabs(angleDiff(gg.theta_flange, gg.theta_q7)));
  }
  check(worst < 0.5 * kDeg, "theta_flange == theta_q7 (-135 - q7) over the whole window, worst " + std::to_string(worst / kDeg) + " deg");
  // yaw of base 2 shifts the flange phase by the yaw, not the q7 phase (a mismatch the node flags)
  PhaseParams py = pp;
  py.base2_in_base1 = BaseTransform::fromYaw({0.0, 0.975, 0.0}, 2.0 * kDeg);
  const SatGeom gy = computeSatGeom(py, qv);
  check(near(angleDiff(gy.theta_flange, gy.theta_q7), 2.0 * kDeg, 1e-9), "base yaw 2 deg -> theta_flange - theta_q7 = 2 deg");
  const Eigen::Vector3d ax = axisFromTilt(0.0, 0.0);
  check((ax - Eigen::Vector3d::UnitZ()).norm() < 1e-12, "axisFromTilt(0, 0) = +z");
  const Eigen::Vector3d ax2 = axisFromTilt(10.0 * kDeg, 0.0);
  check(near(ax2[1], -std::sin(10.0 * kDeg), 1e-12) && near(ax2[2], std::cos(10.0 * kDeg), 1e-12), "axisFromTilt(10, 0) = Rx(10) ez");
}

static void testTravel() {
  std::printf("travel clamp on the window\n");
  const double lo = -135.0 * kDeg, hi = 45.0 * kDeg, m = 5.0 * kDeg, minr = 4.0 * kDeg;
  TravelPlan t = planTravelInWindow(-125.0 * kDeg, +1, lo, hi, m, 0.0, minr);
  check(t.ok && near(t.travel, 165.0 * kDeg, 1e-12) && near(t.q7_end, 40.0 * kDeg, 1e-12), "sign + from -125: travel 165 deg, end 40 deg (edge - margin)");
  t = planTravelInWindow(-125.0 * kDeg, -1, lo, hi, m, 0.0, 6.0 * kDeg);
  check(!t.ok && near(t.travel, 5.0 * kDeg, 1e-12), "sign - from -125: only 5 deg available (< 6 deg of ramps) -> refused");
  t = planTravelInWindow(-125.0 * kDeg, +1, lo, hi, m, 30.0 * kDeg, minr);
  check(t.ok && t.capped && near(t.travel, 30.0 * kDeg, 1e-12), "max_travel 30 deg caps the travel");
  t = planTravelInWindow(-135.0 * kDeg, +1, lo, hi, m, 0.0, minr);
  check(!t.ok, "q7_0 on the window edge (validated pose) -> refused (outside the window with margin)");
  t = planTravelInWindow(50.0 * kDeg, -1, lo, hi, m, 0.0, minr);
  check(!t.ok, "q7_0 outside the window -> refused");
  t = planTravelInWindow(0.0, +1, lo, hi, m, 1000.0 * kDeg, minr);
  check(t.ok && t.travel < 360.0 * kDeg && near(t.q7_end, hi - m, 1e-12), "huge max_travel: never beyond the window (never 360 deg)");
  check(insideWindow(-5.0 * kDeg, -180.0 * kDeg, 0.0, m) && !insideWindow(-4.9 * kDeg, -180.0 * kDeg, 0.0, m) &&
            !insideWindow(-180.0 * kDeg, -180.0 * kDeg, 0.0, m),
        "insideWindow with margin at both edges");
}

static void testOmega() {
  std::printf("omega regression\n");
  const double w = -2.0 * kDeg, dt = 0.01;
  OmegaRegressor r(5.0, dt);
  bool nan_before = true;
  double last = 0.0;
  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0.0, 0.05 * kDeg);
  for (int i = 0; i <= 1000; ++i) {
    const double t = i * dt;
    const double v = r.push(t, 0.3 + w * t + noise(rng));
    if (t < 5.0 - 1.5 * dt - 1e-9 && !std::isnan(v)) nan_before = false;
    last = v;
    if (i == 498) check(std::isnan(v), "t = 4.98 s (span < window - 1.5 sample periods): NaN");
    if (i == 500) check(!std::isnan(v), "t = 5.00 s (window complete): finite");
  }
  check(nan_before, "NaN for every push before the window is full");
  {  // pushes jittered off the grid (as from the 1 kHz loop): once full, the window must stay valid (no NaN)
    OmegaRegressor rj(5.0, 0.01);
    std::uniform_real_distribution<double> late(0.0, 0.0011);
    bool any_nan = false;
    for (int i = 0; i <= 2000; ++i) {
      const double t = i * 0.01 + late(rng);
      const double v = rj.push(t, w * t);
      if (t > 5.02 && std::isnan(v)) any_nan = true;
    }
    check(!any_nan, "jittered pushes (up to 1.1 ms late): no NaN once the window is full");
  }
  check(near(last, w, 0.01 * std::fabs(w)), "noisy ramp (0.05 deg rms): slope within 1 % of -2 deg/s, got " + std::to_string(last / kDeg));
  OmegaRegressor r2(5.0, 0.001);
  double v2 = 0.0;
  for (int i = 0; i <= 6000; ++i) v2 = r2.push(100.0 + i * 0.001, 1.0 + 3.0 * kDeg * (100.0 + i * 0.001));
  check(near(v2, 3.0 * kDeg, 1e-9), "exact ramp at 1 kHz, t offset 100 s: slope exact to 1e-9");
  check(std::isnan(r2.push(std::nan(""), 0.0)), "non-finite input -> NaN");
  r2.reset();
  check(std::isnan(r2.push(200.0, 0.0)), "after reset: NaN");
}

static void testOmegaValid() {
  std::printf("omega_valid\n");
  const double ramp = 2.0, win = 5.0, cend = 17.0, w = -0.0349;
  check(!omegaValid(false, 10.0, ramp, win, cend, w), "not running (WAIT_START / STOPPING / HOLD): false");
  check(!omegaValid(true, 1.0, ramp, win, cend, w), "start ramp: false");
  check(!omegaValid(true, 6.999, ramp, win, cend, w), "window still overlaps the ramp (t_run < ramp + window): false");
  check(omegaValid(true, 7.0, ramp, win, cend, w), "t_run = ramp + window: true");
  check(!omegaValid(true, 10.0, ramp, win, cend, std::nan("")), "regression NaN: false");
  check(!omegaValid(true, 17.001, ramp, win, cend, w), "nominal stop ramp: false");
}

static void testMassConfirmation() {
  std::printf("payload confirmation\n");
  using real_replay::massConfirmationOk;
  check(massConfirmationOk("0.4", 0.4) && massConfirmationOk(" 0.400 \n", 0.4) && massConfirmationOk("0.401", 0.4), "0.4 / 0.400 / 0.401 accepted");
  check(!massConfirmationOk("0.402", 0.4) && !massConfirmationOk("0.398", 0.4), "2 g off refused");
  check(!massConfirmationOk("", 0.4) && !massConfirmationOk("si", 0.4) && !massConfirmationOk("0.4kg", 0.4), "empty / text / unit suffix refused");
}

static void testHeartbeat() {
  std::printf("heartbeat timeout\n");
  HeartbeatMonitor h(0.5);
  check(!h.update(0.0, 0) && !h.update(10.0, 0) && !h.armed(), "never armed without a heartbeat (no timeout)");
  check(!h.update(10.0, 1) && h.armed(), "first heartbeat arms");
  check(!h.update(10.5, 1), "age 0.5 s == T_hb: not yet");
  check(!h.update(10.4, 2) && !h.update(10.85, 2), "new heartbeat at 10.4 resets the age");
  check(h.update(10.91, 2), "age 0.51 s > T_hb: timeout");
  check(h.update(11.0, 3), "latched: a late heartbeat does not clear the timeout");
}

static void testConfig(const char* path) {
  std::printf("hw_config loader: %s\n", path);
  try {
    const real_replay::HwConfig c = real_replay::loadHwConfig(path);
    check(near(c.scene.center_m[1], 0.720, 1e-12) && near(c.scene.omega_rad_s, -2.0 * kDeg, 1e-12), "scene values");
    check(c.robot2.ip == "192.168.0.202" && c.udp.publish_port == 5601 && c.udp.command_port == 5602, "robot2 ip and ports");
    check(near(c.robot2.q7_lo_rad, -135.0 * kDeg, 1e-12) && near(c.robot2.q7_hi_rad, 45.0 * kDeg, 1e-12), "q7 window");
    check(near(c.mockup.mass_kg, 0.4, 1e-12), "mockup 0.4 kg");
  } catch (const std::exception& e) {
    check(false, std::string("load: ") + e.what());
  }
}

int main(int argc, char** argv) {
  testUnwrap();
  testFkAndPhase();
  testTravel();
  testOmega();
  testHeartbeat();
  testOmegaValid();
  testMassConfirmation();
  if (argc > 1) testConfig(argv[1]);
  std::printf("%s (%d failure(s))\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
