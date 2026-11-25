// Unit tests of the J7 reference generator (include/real_replay/j7_profile.hpp). No libfranka, no config.
//   test_j7_profile
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <stdexcept>
#include <string>

#include "real_replay/j7_profile.hpp"

using namespace real_replay;

static int g_fail = 0;
static void check(bool ok, const std::string& what) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}
static std::string f(const char* fmt, double a, double b = 0.0, double c = 0.0) {
  char buf[200];
  std::snprintf(buf, sizeof buf, fmt, a, b, c);
  return buf;
}

constexpr double kDeg = M_PI / 180.0;
constexpr double kDt = 1e-3;

struct Case {
  double v_deg, travel_deg, t1, t2, hold;
};

static void runCase(const Case& c) {
  std::printf("case: v=%.2f deg/s travel=%.1f deg ramp=%.2f stop=%.2f hold=%.2f\n", c.v_deg, c.travel_deg, c.t1, c.t2, c.hold);
  J7ProfileParams p;
  p.speed_rad_s = c.v_deg * kDeg;
  p.travel_rad = c.travel_deg * kDeg;
  p.ramp_s = c.t1;
  p.stop_ramp_s = c.t2;
  p.hold_s = c.hold;
  const J7Profile prof(p);
  const double T = prof.motionDurationS();

  // total rotation exactly travel, final velocity zero
  const J7Ref end = prof.sample(T);
  check(std::fabs(end.d - p.travel_rad) < 1e-9, f("total rotation == travel (|err| %.3e rad)", std::fabs(end.d - p.travel_rad)));
  check(end.v == 0.0, "final velocity is zero");
  const J7Ref late = prof.sample(T + 100.0);
  check(late.d == end.d && late.v == 0.0, "after the motion: position and velocity stay constant");
  check(prof.sample(0.0).d == 0.0 && prof.sample(0.0).v == 0.0, "initial position and velocity are zero");
  check(prof.sample(-1.0).d == 0.0, "negative time is clamped");
  check(std::fabs(prof.totalDurationS() - (T + c.hold)) < 1e-12, "totalDuration = motion + hold");

  // numerical integration of the velocity reproduces the position; continuity of both; monotonicity
  double d_int = 0.0, max_dd = 0.0, max_dv = 0.0, max_pos_err = 0.0, max_acc = 0.0, max_dacc = 0.0;
  bool monotone = true, vmax_ok = true;
  J7Ref prev = prof.sample(0.0);
  double prev_acc = 0.0;
  for (double t = kDt; t <= T + 1e-12; t += kDt) {
    const J7Ref cur = prof.sample(t);
    d_int += 0.5 * (prev.v + cur.v) * kDt;
    max_pos_err = std::max(max_pos_err, std::fabs(d_int - cur.d));
    max_dd = std::max(max_dd, std::fabs(cur.d - prev.d));
    max_dv = std::max(max_dv, std::fabs(cur.v - prev.v));
    const double acc = (cur.v - prev.v) / kDt;
    max_acc = std::max(max_acc, std::fabs(acc));
    if (t > 2.0 * kDt) max_dacc = std::max(max_dacc, std::fabs(acc - prev_acc));
    prev_acc = acc;
    monotone = monotone && cur.d >= prev.d - 1e-15 && cur.v >= -1e-15;
    vmax_ok = vmax_ok && cur.v <= p.speed_rad_s * (1.0 + 1e-12);
    prev = cur;
  }
  check(max_pos_err < 1e-6, f("d(t) == integral of v(t) (max err %.3e rad, trapezoid at 1 kHz)", max_pos_err));
  check(max_dd <= p.speed_rad_s * kDt * (1.0 + 1e-9), f("position continuous (max step %.3e rad per ms)", max_dd));
  // velocity step per ms bounded by v * max(s') * dt / T = v*1.5*dt/T
  const double dv_bound = 1.5 * p.speed_rad_s * kDt / std::min(c.t1, c.t2) * (1.0 + 1e-6);
  check(max_dv <= dv_bound, f("velocity continuous (max step %.3e rad/s per ms, bound %.3e)", max_dv, dv_bound));
  // acceleration step per ms bounded by v * max|s''| * dt / T^2 = v*6*dt/T^2
  const double Tm = std::min(c.t1, c.t2);
  const double dacc_bound = 6.0 * p.speed_rad_s * kDt / (Tm * Tm) * (1.0 + 1e-6) + 1e-12;
  check(max_dacc <= dacc_bound, f("acceleration continuous (max step %.3e rad/s^2 per ms, bound %.3e)", max_dacc, dacc_bound));
  check(monotone && vmax_ok, "velocity in [0, v], position non-decreasing");

  // cruise phase
  const double mid = 0.5 * (prof.constantStartS() + prof.constantEndS());
  check(std::fabs(prof.sample(mid).v - p.speed_rad_s) < 1e-15, "velocity == v in the constant phase");

  // symmetry of the two ramps (same duration): v_start(x) == v_stop(1-x) and distances equal
  if (std::fabs(c.t1 - c.t2) < 1e-12) {
    double worst = 0.0;
    for (int i = 0; i <= 100; ++i) {
      const double x = i / 100.0;
      const double vs = prof.sample(x * c.t1).v;
      const double ve = prof.sample(prof.constantEndS() + (1.0 - x) * c.t2).v;
      worst = std::max(worst, std::fabs(vs - ve));
    }
    check(worst < 1e-12, f("ramps symmetric: v_start(x) == v_stop(1-x) (max dev %.3e)", worst));
  }
  const double d_start = prof.sample(prof.constantStartS()).d;
  const double d_stop = prof.sample(T).d - prof.sample(prof.constantEndS()).d;
  check(std::fabs(d_start - 0.5 * p.speed_rad_s * c.t1) < 1e-12, "start ramp covers v*T1/2");
  check(std::fabs(d_stop - 0.5 * p.speed_rad_s * c.t2) < 1e-12, "stop ramp covers v*T2/2");
}

int main() {
  for (const Case& c : {Case{2.0, 30.0, 2.0, 2.0, 1.0}, Case{5.0, 90.0, 2.0, 2.0, 1.0}, Case{1.0, 10.0, 1.0, 3.0, 0.0},
                        Case{2.0, 5.0, 2.0, 2.0, 0.5} /* Tc = 0.5 s, short cruise */}) {
    runCase(c);
  }

  // sign: the generator is sign-free (d >= 0); the sign enters the reference q7_0 + sign*d
  {
    std::printf("sign\n");
    J7ProfileParams p;
    p.speed_rad_s = 2.0 * kDeg;
    p.travel_rad = 30.0 * kDeg;
    const J7Profile prof(p);
    const J7Ref r = prof.sample(5.0);
    const double q0 = 0.3;
    check(r.d > 0.0 && r.v > 0.0, "d and v are positive in the motion");
    check((q0 + 1 * r.d) > q0 && (q0 - 1 * r.d) < q0, "sign +1 increases q7, sign -1 decreases it");
    check(std::fabs((q0 + 1 * prof.sample(prof.motionDurationS()).d) - (q0 + 30.0 * kDeg)) < 1e-9, "q_end = q0 + sign*travel (+)");
    check(std::fabs((q0 - 1 * prof.sample(prof.motionDurationS()).d) - (q0 - 30.0 * kDeg)) < 1e-9, "q_end = q0 + sign*travel (-)");
  }

  // exact: T_c with ramps 2+2 and v = 2 deg/s, travel 30 deg: (30 - 4)/2 = 13 s
  {
    J7ProfileParams p;
    p.speed_rad_s = 2.0 * kDeg;
    p.travel_rad = 30.0 * kDeg;
    const J7Profile prof(p);
    check(std::fabs(prof.constantS() - 13.0) < 1e-9, "Tc = 13 s for v=2 deg/s, travel 30 deg, ramps 2+2");
    check(std::fabs(prof.motionDurationS() - 17.0) < 1e-9, "motion duration = 17 s");
    check(std::fabs(prof.totalDurationS() - 18.0) < 1e-9, "total duration with 1 s hold = 18 s");
  }

  // early stop
  {
    std::printf("early stop\n");
    J7ProfileParams p;
    p.speed_rad_s = 5.0 * kDeg;
    p.travel_rad = 60.0 * kDeg;
    p.ramp_s = 2.0;
    p.stop_ramp_s = 2.0;
    const J7Profile prof(p);
    for (double ts : {0.5, 1.0, 1.999, 2.0, 6.0, 9.5}) {
      const J7Ref a = prof.sample(ts);
      const J7Ref s0 = stopFrom(a.d, a.v, p.stop_ramp_s, 0.0);
      check(std::fabs(s0.d - a.d) < 1e-15 && std::fabs(s0.v - a.v) < 1e-15, f("stopFrom continuous at the switch (t=%.3f)", ts));
      J7Ref pr = s0;
      bool mono = true;
      double max_dd = 0.0;
      for (double tau = kDt; tau <= p.stop_ramp_s + 1e-12; tau += kDt) {
        const J7Ref c = stopFrom(a.d, a.v, p.stop_ramp_s, tau);
        mono = mono && c.v <= pr.v + 1e-15 && c.v >= 0.0 && c.d >= pr.d - 1e-15;
        max_dd = std::max(max_dd, std::fabs(c.d - pr.d));
        pr = c;
      }
      const J7Ref e = stopFrom(a.d, a.v, p.stop_ramp_s, p.stop_ramp_s);
      check(e.v == 0.0 && std::fabs(e.d - (a.d + 0.5 * a.v * p.stop_ramp_s)) < 1e-12, "early stop ends at zero velocity, d0 + v0*T2/2");
      check(mono, "velocity decreases monotonically to zero");
      check(e.d <= p.travel_rad + 1e-12, "early stop never goes beyond the planned travel");
      (void)max_dd;
    }
  }

  // validation
  {
    std::printf("validation\n");
    auto throws = [](J7ProfileParams p) {
      try {
        J7Profile x(p);
      } catch (const std::invalid_argument&) {
        return true;
      }
      return false;
    };
    J7ProfileParams ok;
    ok.speed_rad_s = 0.03;
    ok.travel_rad = 0.5;
    check(!throws(ok), "valid parameters accepted");
    J7ProfileParams p = ok;
    p.speed_rad_s = 0.0;
    check(throws(p), "speed 0 refused");
    p = ok;
    p.speed_rad_s = std::nan("");
    check(throws(p), "NaN speed refused");
    p = ok;
    p.travel_rad = -0.1;
    check(throws(p), "negative travel refused");
    p = ok;
    p.ramp_s = 0.0;
    check(throws(p), "ramp 0 refused");
    p = ok;
    p.stop_ramp_s = -1.0;
    check(throws(p), "negative stop ramp refused");
    p = ok;
    p.hold_s = -0.1;
    check(throws(p), "negative hold refused");
    p = ok;
    p.travel_rad = 0.5 * p.speed_rad_s * (p.ramp_s + p.stop_ramp_s) * 0.99;
    check(throws(p), "travel shorter than the two ramps refused");
    p.travel_rad = 0.5 * p.speed_rad_s * (p.ramp_s + p.stop_ramp_s);
    check(!throws(p), "travel exactly equal to the two ramps accepted (Tc = 0)");
  }

  // travel check against limits
  {
    std::printf("travel check\n");
    const double lo = -2.8973, hi = 2.8973, m = 10.0 * kDeg;
    J7TravelCheck c = checkJ7Travel(0.785, +1, 30.0 * kDeg, lo, hi, m);
    check(c.ok, "+30 deg from 0.785 rad inside the interval");
    check(std::fabs(c.max_pos - (hi - m - 0.785)) < 1e-12 && std::fabs(c.max_neg - (0.785 - (lo + m))) < 1e-12, "max rotation in both directions");
    check(!checkJ7Travel(2.5, +1, 30.0 * kDeg, lo, hi, m).ok, "+30 deg from 2.5 rad exceeds hi - margin");
    check(checkJ7Travel(2.5, -1, 30.0 * kDeg, lo, hi, m).ok, "-30 deg from 2.5 rad is fine");
    check(!checkJ7Travel(-2.5, -1, 30.0 * kDeg, lo, hi, m).ok, "-30 deg from -2.5 rad exceeds lo + margin");
    check(!checkJ7Travel(2.8, -1, 1.0 * kDeg, lo, hi, m).ok, "start inside the margin is refused");
    const double reach = hi - m - 0.0;
    check(checkJ7Travel(0.0, +1, reach, lo, hi, m).ok && !checkJ7Travel(0.0, +1, reach + 1e-6, lo, hi, m).ok, "boundary: exactly at hi - margin accepted, beyond refused");
    check(!checkJ7Travel(std::nan(""), +1, 0.1, lo, hi, m).ok, "NaN start refused");
  }

  // sign (command) and phase_map (physical map) are independent: omega_meas = phase_map * dq7
  {
    std::printf("sign x phase_map\n");
    J7ProfileParams p;
    p.speed_rad_s = 2.0 * kDeg;
    p.travel_rad = 30.0 * kDeg;
    const J7Profile prof(p);
    const double q0 = 0.4, theta0 = 0.25;
    for (int sign : {+1, -1}) {
      for (int pm : {+1, -1}) {
        SlopeFilter sf(0.2, 400);
        double om = 0.0, th0 = 0.0, th_end = 0.0, q7_end = 0.0;
        const double t_probe = prof.constantStartS() + 3.0;
        double om_probe = 0.0, dq7_probe = 0.0;
        for (double t = 0.0; t <= prof.motionDurationS() + 1e-9; t += kDt) {
          const J7Ref r = prof.sample(t);
          const double q7 = q0 + sign * r.d;
          const double th = phaseTheta(theta0, pm, q7, q0);
          om = sf.push(t, th);
          if (t == 0.0) th0 = th;
          if (std::fabs(t - t_probe) < 0.5 * kDt) {
            om_probe = om;
            dq7_probe = sign * r.v;
          }
          th_end = th;
          q7_end = q7;
        }
        const std::string tag = f("sign %+.0f, ", sign) + f("phase_map %+.0f", pm);
        check(std::fabs(th0 - theta0) < 1e-15, tag + ": theta(0) == theta0");
        check(std::fabs(om_probe - pm * dq7_probe) < 1e-9, tag + ": omega_meas == phase_map * dq7 (" + f("%.6f rad/s", om_probe) + ")");
        check(std::fabs(om_probe - pm * sign * p.speed_rad_s) < 1e-9, tag + ": omega_meas == phase_map*sign*v in the constant phase");
        check(std::fabs((th_end - theta0) - pm * (q7_end - q0)) < 1e-12, tag + ": total theta change == phase_map * (q7_end - q7_0)");
        check(std::fabs(q7_end - (q0 + sign * p.travel_rad)) < 1e-9, tag + ": q7_end == q7_0 + sign*travel");
      }
    }
    // project convention: default phase_map -1 with sign +1 gives omega < 0 (clockwise seen from above)
    check(phaseTheta(0.0, -1, 1.0, 0.5) < 0.0 && phaseTheta(0.0, +1, 1.0, 0.5) > 0.0, "phaseTheta: phase_map -1 lowers theta when q7 grows");
  }

  // slope filter
  {
    std::printf("slope filter\n");
    SlopeFilter sf(0.2, 400);
    double w = 0.0;
    for (int i = 0; i <= 1000; ++i) {
      const double t = i * kDt;
      w = sf.push(t, 3.0 * t + 1.0);  // linear: slope 3
      if (t >= 0.25) {
        if (std::fabs(w - 3.0) > 1e-9) {
          check(false, f("slope of a ramp is exact at t=%.3f (got %.6f)", t, w));
          break;
        }
      }
    }
    check(std::fabs(w - 3.0) < 1e-9, "slope of a linear signal == 3");
    // step in slope: after one window the estimate follows
    SlopeFilter sg(0.2, 400);
    double x = 0.0;
    for (int i = 0; i <= 600; ++i) {
      const double t = i * kDt;
      x += (t < 0.3 ? 1.0 : 2.0) * kDt;
      w = sg.push(t, x);
    }
    check(std::fabs(w - 2.0) < 1e-6, "after the window the slope follows a speed change");
    SlopeFilter sh(0.2, 400);
    check(sh.push(0.0, 5.0) == 0.0, "first sample: slope 0");
    check(std::fabs(sh.push(0.001, 5.002) - 2.0) < 1e-9, "short history: slope over the available span");
    // circular buffer wrap with a small capacity: the filter must not misbehave
    SlopeFilter sw(0.2, 8);
    for (int i = 0; i <= 2000; ++i) w = sw.push(i * kDt, 4.0 * i * kDt);
    check(std::isfinite(w) && std::fabs(w - 4.0) < 1e-6, "buffer smaller than the window: still finite and correct slope over the stored span");
  }

  if (g_fail == 0) {
    std::printf("ALL PASSED\n");
  } else {
    std::printf("%d FAILED\n", g_fail);
  }
  return g_fail == 0 ? 0 : 1;
}
