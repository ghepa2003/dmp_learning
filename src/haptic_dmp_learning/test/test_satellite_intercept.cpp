// Pure-logic tests for satellite_rotation_mode="continuous" (no ROS): shared goal math,
// phase from a quaternion, phase-consistency check, trigger crossing, rollout time bases and
// start-up validation. See include/haptic_dmp_learning/core/satellite_intercept.hpp.

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "haptic_dmp_learning/core/satellite_intercept.hpp"

using namespace haptic_dmp_learning::core::satellite_intercept;

namespace {

double deg(double d) { return d * M_PI / 180.0; }

Eigen::Quaterniond quatAbout(const Eigen::Vector3d& axis, double theta_deg) {
    return Eigen::Quaterniond(Eigen::AngleAxisd(deg(theta_deg), axis.normalized()));
}

/// Wraps degrees to (-180, 180].
double wrapDeg(double d) {
    double w = std::fmod(d + 180.0, 360.0);
    if (w <= 0.0) w += 360.0;
    return w - 180.0;
}

}  // namespace

// ---------------------------------------------------------------------------------------
// Goal math
// ---------------------------------------------------------------------------------------
TEST(SatelliteInterceptGoal, ThetaZeroGivesDemoGoal) {
    const Eigen::Vector3d ee(0.3048, -0.0001, 0.4801), demo(0.0843163, -0.0487211, -0.00828399);
    const Eigen::Vector3d center(0.45, -0.05, 0.35), axis(0.0, 2.0, 0.0);  // not normalized on purpose
    const AnchoredGoal g = anchorAndRotate(ee, demo, center, axis, 0.0);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(g.goal_relative(i), demo(i), 1e-12) << i;
        EXPECT_NEAR(g.p_rotated_world(i), g.p_demo_world(i), 1e-12) << i;
        EXPECT_NEAR(g.p_demo_world(i), ee(i) + demo(i), 1e-15) << i;
    }
}

TEST(SatelliteInterceptGoal, MatchesInlineFormulaOfTheFrozenBranch) {
    // Oracle: the formula that was inlined in the frozen branch of startTimer(), kept verbatim here.
    auto oracle = [](const Eigen::Vector3d& ee_now, const Eigen::Vector3d& demo_grasp_goal,
                     const Eigen::Vector3d& center, const Eigen::Vector3d& axis, double phase_deg) {
        const Eigen::Vector3d p_grasp_demo_world = ee_now + demo_grasp_goal;
        const double theta_rad = phase_deg * M_PI / 180.0;
        const Eigen::AngleAxisd rot(theta_rad, axis.normalized());
        return Eigen::Vector3d(center + rot * (p_grasp_demo_world - center));
    };
    const Eigen::Vector3d ee(0.3048, -0.0001, 0.4801), demo(0.0843163, -0.0487211, -0.00828399);
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> configs = {
        {Eigen::Vector3d(0.45, -0.05, 0.35), Eigen::Vector3d(0.0, 1.0, 0.0)},
        {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0)},
        {Eigen::Vector3d(0.6, 0.1, 0.5), Eigen::Vector3d(1.0, 2.0, 3.0)}};
    for (const auto& cfg : configs) {
        for (int phase = 0; phase < 360; phase += 30) {   // 12 phases, includes 30/90/180/270
            const Eigen::Vector3d expect = oracle(ee, demo, cfg.first, cfg.second, phase);
            const AnchoredGoal g = anchorAndRotate(ee, demo, cfg.first, cfg.second, phase * M_PI / 180.0);
            for (int i = 0; i < 3; ++i) {
                EXPECT_NEAR(g.p_rotated_world(i), expect(i), 1e-9) << "phase " << phase << " comp " << i;
                EXPECT_NEAR(g.goal_relative(i), expect(i) - ee(i), 1e-9) << "phase " << phase << " comp " << i;
            }
        }
    }
}

TEST(SatelliteInterceptGoal, RotateAboutCenterRightHandedAndDistanceFromAxis) {
    // +90 deg about +z maps x -> y (right-handed)
    const Eigen::Vector3d p = rotateAboutCenter(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), deg(90.0),
                                                Eigen::Vector3d(1.0, 0.0, 0.0));
    EXPECT_NEAR(p.x(), 0.0, 1e-12);
    EXPECT_NEAR(p.y(), 1.0, 1e-12);
    EXPECT_NEAR(distanceFromAxis(Eigen::Vector3d(3.0, 4.0, 7.0), Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, 5)), 5.0, 1e-12);
}

// ---------------------------------------------------------------------------------------
// Phase from a quaternion
// ---------------------------------------------------------------------------------------
TEST(SatelliteInterceptPhase, FirstSampleIsWrappedSignedAngleWithNonTrivialAxisAndReference) {
    const Eigen::Vector3d axis(1.0, 2.0, 3.0);
    const Eigen::Quaterniond q_ref = quatAbout(Eigen::Vector3d(0.3, -0.5, 0.8), 37.0);
    for (double th : {10.0, 170.0, 190.0, 350.0, 370.0, -10.0}) {
        PhaseTracker tr(axis, q_ref);
        const PhaseSample s = tr.update(quatAbout(axis, th) * q_ref);   // R_rel = R_body R_ref^T = R(theta)
        EXPECT_NEAR(radToDeg(s.theta_rad), wrapDeg(th), 1e-9) << th;
        EXPECT_LT(s.off_axis_deg, 1e-9) << th;
        // R_rel really is R(theta) about the axis
        const Eigen::Matrix3d R = Eigen::AngleAxisd(deg(th), axis.normalized()).toRotationMatrix();
        EXPECT_LT((s.R_rel - R).norm(), 1e-12) << th;
    }
}

TEST(SatelliteInterceptPhase, UnwrapIncreasingAndDecreasingSequences) {
    const Eigen::Vector3d axis(0.0, 1.0, 0.0);
    const Eigen::Quaterniond q_ref = quatAbout(Eigen::Vector3d(1.0, 0.0, 0.0), 20.0);
    for (double step : {15.0, 150.0, -15.0, -150.0}) {
        PhaseTracker tr(axis, q_ref);
        double truth = 0.0;
        for (int i = 0; i < 25; ++i, truth += step) {
            const PhaseSample s = tr.update(quatAbout(axis, truth) * q_ref);
            EXPECT_NEAR(radToDeg(s.theta_rad), truth, 1e-8) << "step " << step << " i " << i;
        }
    }
    // Sequence that crosses the 170/190 boundary explicitly
    PhaseTracker tr(axis, Eigen::Quaterniond::Identity());
    double prev = 0.0;
    for (double th : {10.0, 170.0, 190.0, 350.0, 370.0}) {
        const double u = radToDeg(tr.update(quatAbout(axis, th)).theta_rad);
        EXPECT_NEAR(u, th, 1e-8);
        EXPECT_GT(u, prev);
        prev = u;
    }
}

TEST(SatelliteInterceptPhase, SignIsRightHandedAboutTheAxis) {
    const Eigen::Vector3d axis(0.0, 0.0, 1.0);
    PhaseTracker a(axis, Eigen::Quaterniond::Identity()), b(axis, Eigen::Quaterniond::Identity());
    EXPECT_GT(a.update(quatAbout(axis, 25.0)).theta_rad, 0.0);
    EXPECT_LT(b.update(quatAbout(-axis, 25.0)).theta_rad, 0.0);
}

TEST(SatelliteInterceptPhase, RejectsRotationOutOfAxis) {
    const Eigen::Vector3d axis(0.0, 0.0, 1.0);
    const Eigen::Vector3d perp(1.0, 0.0, 0.0);
    {
        PhaseTracker tr(axis, Eigen::Quaterniond::Identity());
        EXPECT_THROW(tr.update(quatAbout(perp, 1.0) * quatAbout(axis, 10.0)), std::runtime_error);
    }
    {
        PhaseTracker tr(axis, Eigen::Quaterniond::Identity());
        PhaseSample s;
        EXPECT_NO_THROW(s = tr.update(quatAbout(perp, 0.3) * quatAbout(axis, 10.0)));
        EXPECT_NEAR(s.off_axis_deg, 0.3, 0.05);
    }
    {   // rotation entirely about a different axis
        PhaseTracker tr(axis, Eigen::Quaterniond::Identity());
        EXPECT_THROW(tr.update(quatAbout(perp, 30.0)), std::runtime_error);
    }
    EXPECT_THROW(PhaseTracker(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()), std::invalid_argument);
}

// ---------------------------------------------------------------------------------------
// Trigger
// ---------------------------------------------------------------------------------------
TEST(SatelliteInterceptTrigger, ThetaTrigIsModulo360) {
    EXPECT_NEAR(radToDeg(InterceptTrigger(deg(10.0), deg(2.0), 60.0).thetaTrigRad()), 250.0, 1e-9);   // 10-120
    EXPECT_NEAR(radToDeg(InterceptTrigger(deg(90.0), deg(2.0), 60.0).thetaTrigRad()), 330.0, 1e-9);   // 90-120
    EXPECT_NEAR(radToDeg(InterceptTrigger(deg(90.0), deg(-2.0), 60.0).thetaTrigRad()), 210.0, 1e-9);  // 90+120
    EXPECT_NEAR(radToDeg(InterceptTrigger(deg(0.0), deg(2.0), 90.0).thetaTrigRad()), 180.0, 1e-9);
    EXPECT_THROW(InterceptTrigger(0.0, 0.0, 60.0), std::invalid_argument);
}

TEST(SatelliteInterceptTrigger, CrossingPositiveOmega) {
    InterceptTrigger t(deg(90.0), deg(2.0), 60.0);   // theta_trig = 330 deg
    int fired_at = -1, i = 0;
    for (double th : {328.5, 329.5, 330.5, 331.5}) {
        const auto r = t.update(deg(th));
        EXPECT_FALSE(r.first_sample_past);
        if (r.fired && fired_at < 0) fired_at = i;
        ++i;
    }
    EXPECT_EQ(fired_at, 2);   // first sample with e >= 0
}

TEST(SatelliteInterceptTrigger, CrossingNegativeOmega) {
    InterceptTrigger t(deg(90.0), deg(-2.0), 60.0);  // theta_trig = 210 deg, phase decreasing
    int fired_at = -1, i = 0;
    for (double th : {211.5, 210.5, 209.5, 208.5}) {
        const auto r = t.update(deg(th));
        if (r.fired && fired_at < 0) fired_at = i;
        ++i;
    }
    EXPECT_EQ(fired_at, 2);
}

TEST(SatelliteInterceptTrigger, WrapAroundAtPlusMinus180) {
    // theta_trig = 180 deg. Unwrapped phase and the same phase expressed in (-180, 180] both trigger.
    {
        InterceptTrigger t(deg(0.0), deg(2.0), 90.0);
        int fired_at = -1, i = 0;
        for (double th : {178.5, 179.5, 180.5, 181.5}) {
            if (t.update(deg(th)).fired && fired_at < 0) fired_at = i;
            ++i;
        }
        EXPECT_EQ(fired_at, 2);
    }
    {
        InterceptTrigger t(deg(0.0), deg(2.0), 90.0);
        int fired_at = -1, i = 0;
        for (double th : {178.5, 179.5, -179.5, -178.5}) {   // 180.5 and 181.5 written as wrapped angles
            if (t.update(deg(th)).fired && fired_at < 0) fired_at = i;
            ++i;
        }
        EXPECT_EQ(fired_at, 2);
    }
    {   // must NOT fire on the e = +180 -> -180 discontinuity (theta_trig - 180 = 150 deg here): the first
        // sample (100 deg, e = +130) is already past, so the trigger is unarmed and waits for the next lap
        InterceptTrigger t(deg(90.0), deg(2.0), 60.0);   // theta_trig 330
        bool fired = false;
        for (double th = 100.0; th <= 200.0; th += 1.0) fired |= t.update(deg(th)).fired;
        EXPECT_FALSE(fired);
    }
}

TEST(SatelliteInterceptTrigger, FirstSampleAlreadyPastWaitsForTheNextLap) {
    InterceptTrigger t(deg(90.0), deg(2.0), 60.0);   // theta_trig = 330 deg
    int fired_at = -1;
    int i = 0;
    for (double th = 331.5; th < 700.0; th += 1.0, ++i) {
        const auto r = t.update(deg(th));
        if (i == 0) EXPECT_TRUE(r.first_sample_past);
        else EXPECT_FALSE(r.first_sample_past);
        if (r.fired && fired_at < 0) fired_at = i;
    }
    EXPECT_EQ(fired_at, 359);   // theta = 690.5 deg: e = +0.5 deg after the lap
    // expected wait from that first sample: 329 deg... to theta_trig at 2 deg/s -> next lap
    InterceptTrigger t2(deg(90.0), deg(2.0), 60.0);
    t2.update(deg(331.0));
    EXPECT_NEAR(t2.expectedWaitS(deg(331.0)), 359.0 / 2.0, 1e-9);
    InterceptTrigger t3(deg(90.0), deg(2.0), 60.0);
    EXPECT_NEAR(t3.expectedWaitS(deg(300.0)), 15.0, 1e-9);
}

TEST(SatelliteInterceptTrigger, TimeoutIsOneLapPlusTotalPlusMargin) {
    EXPECT_NEAR(InterceptTrigger::timeoutS(2.0, 61.0, 30.0), 180.0 + 61.0 + 30.0, 1e-12);
    EXPECT_NEAR(InterceptTrigger::timeoutS(-2.0, 61.0, 30.0), 180.0 + 61.0 + 30.0, 1e-12);
}

// ---------------------------------------------------------------------------------------
// Phase-vs-time consistency
// ---------------------------------------------------------------------------------------
TEST(SatelliteInterceptConsistency, CoherentOmegaPasses) {
    const double omega = deg(10.0);
    PhaseConsistencyChecker c(omega, 5.0, 1.0);
    bool validated_before_window = false;
    for (int i = 0; i <= 200; ++i) {
        const double t = 100.0 + 0.05 * i;   // 10 s of samples
        EXPECT_NO_THROW(c.update(t, deg(20.0) + omega * (t - 100.0)));
        if (i < 100) validated_before_window |= c.validated();
    }
    EXPECT_FALSE(validated_before_window);   // no verdict before a full window
    EXPECT_TRUE(c.validated());
    EXPECT_NEAR(c.lastErrorDeg(), 0.0, 1e-9);
}

TEST(SatelliteInterceptConsistency, OmegaWrongBy5PercentIsRejected) {
    const double omega_true = deg(10.0 * 1.05), omega_cfg = deg(10.0);   // 5% too fast: 2.5 deg over 5 s > 1 deg
    PhaseConsistencyChecker c(omega_cfg, 5.0, 1.0);
    bool threw = false;
    try {
        for (int i = 0; i <= 200; ++i) {
            const double t = 0.05 * i;
            c.update(t, omega_true * t);
        }
    } catch (const std::runtime_error& e) {
        threw = true;
        EXPECT_NE(std::string(e.what()).find("omega"), std::string::npos);
    }
    EXPECT_TRUE(threw);
}

TEST(SatelliteInterceptConsistency, NonMonotonicStampThrowsAndArgsValidated) {
    PhaseConsistencyChecker c(deg(2.0), 5.0, 1.0);
    c.update(10.0, 0.0);
    EXPECT_THROW(c.update(9.9, 0.0), std::runtime_error);
    EXPECT_THROW(PhaseConsistencyChecker(deg(2.0), 0.0, 1.0), std::invalid_argument);
    EXPECT_THROW(PhaseConsistencyChecker(deg(2.0), 5.0, 0.0), std::invalid_argument);
}

// ---------------------------------------------------------------------------------------
// Time bases
// ---------------------------------------------------------------------------------------
namespace {
struct TickRun {
    int end_tick = -1;
    double duration = 0.0;
    std::uint64_t skipped = 0;
    double mean_period = 0.0;
};
TickRun runMeasured(const std::vector<double>& nows, double dt, double tau) {
    MeasuredTimeBase tb(dt, tau);
    TickRun r;
    int k = 0;
    for (double now : nows) {
        const auto tick = tb.update(now);
        if (tick.at_end) {
            r.end_tick = k;
            r.duration = tick.elapsed;
            break;
        }
        ++k;
    }
    r.skipped = tb.skippedTicks();
    r.mean_period = tb.meanPeriod();
    return r;
}
}  // namespace

TEST(SatelliteInterceptTimeBase, ElapsedFollowsTheInjectedClockWithJitterAndSkips) {
    const double dt = 0.005, tau = 1.0;
    std::vector<double> nows;
    double t = 250.0;   // arbitrary sim-time origin
    for (int k = 0; k < 400; ++k) {
        nows.push_back(t);
        t += 0.004 + 0.0015 * std::sin(0.7 * k);   // jitter around 4 ms
        if (k == 100) t += 0.010;                    // one 10 ms hole (> 1.5 dt): a skipped tick
    }
    MeasuredTimeBase tb(dt, tau);
    int end_tick = -1;
    for (std::size_t k = 0; k < nows.size(); ++k) {
        const auto tick = tb.update(nows[k]);
        EXPECT_NEAR(tick.elapsed, nows[k] - nows[0], 1e-12);
        if (k == 0) {
            EXPECT_TRUE(tick.first);
            EXPECT_EQ(tick.dt_step, 0.0);
        }
        const bool expect_end = (nows[k] - nows[0]) >= tau;
        EXPECT_EQ(tick.at_end, expect_end) << k;
        if (tick.at_end) { end_tick = static_cast<int>(k); break; }
    }
    ASSERT_GE(end_tick, 0);
    EXPECT_LT(nows[end_tick - 1] - nows[0], tau);         // at_end is the FIRST tick with elapsed >= tau
    EXPECT_GE(tb.skippedTicks(), 1u);
    EXPECT_NEAR(tb.t0(), 250.0, 1e-12);
    EXPECT_EQ(tb.tickCount(), static_cast<std::uint64_t>(end_tick + 1));
    EXPECT_GT(tb.maxPeriod(), 0.010);
}

TEST(SatelliteInterceptTimeBase, DurationStaysTauForTickPeriodsDifferentFromDt) {
    const double dt = 0.005, tau = 60.972930046;
    for (double period : {0.00375, 0.006}) {
        std::vector<double> nows;
        for (int k = 0; k < 20000; ++k) nows.push_back(1000.0 + period * k);
        const TickRun r = runMeasured(nows, dt, tau);
        ASSERT_GE(r.end_tick, 0) << period;
        EXPECT_GE(r.duration, tau);
        EXPECT_LT(r.duration - tau, period + 1e-9) << period;   // overshoot below one tick
        EXPECT_NEAR(r.mean_period, period, 1e-9);
        EXPECT_EQ(r.skipped, 0u);                               // neither 3.75 ms nor 6 ms exceeds 1.5*dt
    }
    // The fixed-step increment (frozen) would instead last ticks*period: the effect this mode removes.
    const double period = 0.00375;
    double elapsed = 0.0;
    int ticks = 0;
    while (!fixedStepAtEnd(elapsed, dt, tau)) { elapsed += dt; ++ticks; }
    EXPECT_NEAR(ticks * period, 45.7, 0.1);   // 12194 ticks * 3.75 ms, far from tau = 60.97 s
}

TEST(SatelliteInterceptTimeBase, FrozenFixedStepIsUnchanged) {
    // Non-regression of the frozen branch: elapsed += dt per non-final tick, at_end when (elapsed+dt) >= tau.
    const double dt = 0.005;
    {   // small case computed by hand: tau = 0.0125 -> ticks at elapsed 0, 0.005, 0.010; at_end at 0.010
        double elapsed = 0.0;
        int non_end = 0;
        while (!fixedStepAtEnd(elapsed, dt, 0.0125)) { elapsed += dt; ++non_end; }
        EXPECT_EQ(non_end, 2);
        EXPECT_NEAR(elapsed, 0.010, 1e-15);
    }
    {   // the real tau of the baseline: 12194 non-final ticks (12194.586 ticks of dt), then the at_end tick
        const double tau = 60.972930046;
        double elapsed = 0.0;
        int non_end = 0;
        while (!fixedStepAtEnd(elapsed, dt, tau)) { elapsed += dt; ++non_end; }
        EXPECT_EQ(non_end, 12194);
        EXPECT_NEAR(elapsed, 12194 * dt, 1e-6);
    }
    EXPECT_TRUE(fixedStepAtEnd(0.9951, 0.005, 1.0));   // (elapsed + dt) >= tau is at_end
    EXPECT_FALSE(fixedStepAtEnd(0.99, 0.005, 1.0));
}

TEST(SatelliteInterceptTimeBase, FrozenStepDtIsZeroOnlyOnTheFirstTick) {
    // frozenStepDt() is what prodmp_gazebo_executor_node::stepCallback() feeds into
    // ProDMP::step()/QuaternionDMP::step() for the frozen/baseline branch: the first
    // tick must be s=0 exact (step_dt=0.0, matching MeasuredTimeBase's ticks_==0
    // case), every tick after uses the nominal dt.
    const double dt = 0.005;
    EXPECT_EQ(frozenStepDt(/*is_first_tick=*/true, dt), 0.0);
    EXPECT_EQ(frozenStepDt(/*is_first_tick=*/false, dt), dt);

    // Simulate a short rollout: elapsed_ starts at 0.0 and accumulates step_dt each
    // tick (0.0 on the first, dt after) - same invariant the node relies on.
    bool first_tick_pending = true;
    double elapsed = 0.0;
    std::vector<double> step_dts;
    for (int i = 0; i < 5; ++i) {
        const double step_dt = frozenStepDt(first_tick_pending, dt);
        first_tick_pending = false;
        step_dts.push_back(step_dt);
        elapsed += step_dt;
    }
    EXPECT_EQ(step_dts, (std::vector<double>{0.0, dt, dt, dt, dt}));
    EXPECT_NEAR(elapsed, 4 * dt, 1e-15);   // 5 ticks, only 4 of them advanced the phase
}

TEST(SatelliteInterceptTimeBase, ClockGoingBackwardsAndBadArgsThrow) {
    MeasuredTimeBase tb(0.005, 1.0);
    tb.update(10.0);
    EXPECT_THROW(tb.update(9.999), std::runtime_error);
    EXPECT_THROW(MeasuredTimeBase(0.0, 1.0), std::invalid_argument);
    EXPECT_THROW(MeasuredTimeBase(0.005, 0.0), std::invalid_argument);
}

// ---------------------------------------------------------------------------------------
// Start-up validation
// ---------------------------------------------------------------------------------------
namespace {
ContinuousInputs validInputs(bool measured = true) {
    ContinuousInputs in;
    in.rotation_enabled = true;
    in.use_sim_time = true;
    in.target_odom_required = false;
    in.axis_explicit = in.center_explicit = in.omega_explicit = true;
    in.axis = Eigen::Vector3d(0.0, 2.0, 0.0);
    in.center = Eigen::Vector3d(0.45, -0.05, 0.35);
    in.omega_deg_s = 2.0;
    in.intercept_phase_deg = 90.0;
    in.contact_time_s = 60.972930046 + 0.005;
    in.phase_source = measured ? "measured" : "model";
    if (measured) {
        in.odom_topic = "/free_target_object/odometry";
        in.q_ref = std::vector<double>{0.0, 0.0, 0.0, 1.0};
    } else {
        in.model_phase0_deg = 15.0;
    }
    in.contact_time_tolerance_s = 0.1;
    in.allow_contact_time_mismatch = false;
    in.phase_consistency_window_s = 5.0;
    in.phase_consistency_tol_deg = 1.0;
    in.trigger_timeout_margin_s = 30.0;
    in.rollout_time_tol_s = 0.05;
    in.tau = 60.972930046;
    in.dt = 0.005;
    return in;
}

/// Runs validateContinuous and returns the exception message ("" if it did not throw).
std::string errorOf(const ContinuousInputs& in) {
    try {
        validateContinuous(in);
    } catch (const std::invalid_argument& e) {
        return e.what();
    }
    return "";
}
}  // namespace

TEST(SatelliteInterceptConfig, ValidInputsAreAcceptedAndNormalized) {
    const ContinuousConfig c = validateContinuous(validInputs(true));
    EXPECT_NEAR(c.axis_unit.norm(), 1.0, 1e-15);
    EXPECT_NEAR(c.axis_unit.y(), 1.0, 1e-15);
    EXPECT_TRUE(c.measured);
    EXPECT_NEAR(c.omega_rad_s, deg(2.0), 1e-15);
    EXPECT_NEAR(c.theta_int_rad, deg(90.0), 1e-15);
    EXPECT_NEAR(c.expected_contact_time_s, 60.972930046 + 0.005, 1e-12);
    EXPECT_FALSE(c.contact_time_mismatch);
    EXPECT_NEAR(c.trigger_timeout_s, 180.0 + c.contact_time_s + 30.0, 1e-9);
    // q_ref given as (x, y, z, w): a 90 deg rotation about z has w = cos(45)
    ContinuousInputs in = validInputs(true);
    in.q_ref = std::vector<double>{0.0, 0.0, std::sin(deg(45.0)), std::cos(deg(45.0))};
    const ContinuousConfig c2 = validateContinuous(in);
    EXPECT_NEAR(c2.q_ref.w(), std::cos(deg(45.0)), 1e-15);
    EXPECT_NEAR(c2.q_ref.z(), std::sin(deg(45.0)), 1e-15);
    const ContinuousConfig cm = validateContinuous(validInputs(false));
    EXPECT_FALSE(cm.measured);
    EXPECT_NEAR(cm.theta0_model_rad, deg(15.0), 1e-15);
}

TEST(SatelliteInterceptConfig, MissingOrInvalidParametersNameTheOffender) {
    struct Case {
        std::string expect;
        ContinuousInputs in;
    };
    std::vector<Case> cases;
    auto add = [&cases](const std::string& expect, ContinuousInputs in) { cases.push_back({expect, in}); };
    ContinuousInputs b = validInputs(true);
    { auto in = b; in.rotation_enabled = false; add("satellite_rotation_enabled", in); }
    { auto in = b; in.use_sim_time = false; add("use_sim_time", in); }
    { auto in = b; in.target_odom_required = true; add("target_odom_required", in); }
    { auto in = b; in.axis_explicit = false; add("satellite_rotation_axis", in); }
    { auto in = b; in.center_explicit = false; add("satellite_rotation_center", in); }
    { auto in = b; in.omega_explicit = false; add("satellite_rotation_angular_velocity_deg_s", in); }
    { auto in = b; in.axis = Eigen::Vector3d(0.0, 1e-12, 0.0); add("satellite_rotation_axis", in); }
    { auto in = b; in.omega_deg_s = 0.0; add("satellite_rotation_angular_velocity_deg_s", in); }
    { auto in = b; in.intercept_phase_deg.reset(); add("satellite_intercept_phase_deg", in); }
    { auto in = b; in.contact_time_s.reset(); add("satellite_contact_time_s", in); }
    { auto in = b; in.contact_time_s = -1.0; add("satellite_contact_time_s", in); }
    { auto in = b; in.phase_source.reset(); add("satellite_phase_source", in); }
    { auto in = b; in.phase_source = "bogus"; add("satellite_phase_source", in); }
    { auto in = b; in.odom_topic.reset(); add("satellite_odom_topic", in); }
    { auto in = b; in.odom_topic = ""; add("satellite_odom_topic", in); }
    { auto in = b; in.q_ref.reset(); add("satellite_q_ref", in); }
    { auto in = b; in.q_ref = std::vector<double>{0.0, 0.0, 1.0}; add("satellite_q_ref", in); }
    { auto in = b; in.q_ref = std::vector<double>{0.0, 0.0, 0.0, 1.001}; add("satellite_q_ref", in); }
    { auto in = validInputs(false); in.model_phase0_deg.reset(); add("satellite_model_phase0_deg", in); }
    { auto in = b; in.phase_consistency_window_s = 0.0; add("phase_consistency_window_s", in); }
    { auto in = b; in.phase_consistency_tol_deg = 0.0; add("phase_consistency_tol_deg", in); }
    { auto in = b; in.trigger_timeout_margin_s = -1.0; add("trigger_timeout_margin_s", in); }
    { auto in = b; in.rollout_time_tol_s = 0.0; add("rollout_time_tol_s", in); }
    { auto in = b; in.contact_time_tolerance_s = 0.0; add("contact_time_tolerance_s", in); }
    for (const auto& c : cases) {
        const std::string msg = errorOf(c.in);
        ASSERT_FALSE(msg.empty()) << "expected an error for " << c.expect;
        EXPECT_NE(msg.find(c.expect), std::string::npos) << "'" << msg << "' does not name " << c.expect;
    }
}

TEST(SatelliteInterceptConfig, ModelModeDoesNotNeedOdomOrQref) {
    ContinuousInputs in = validInputs(false);   // no odom_topic, no q_ref
    EXPECT_NO_THROW(validateContinuous(in));
}

TEST(SatelliteInterceptConfig, ContactTimeMismatchIsAnErrorUnlessAllowedExplicitly) {
    ContinuousInputs in = validInputs(true);
    in.contact_time_s = 62.0;   // expected 60.9779 +/- 0.1
    const std::string msg = errorOf(in);
    ASSERT_FALSE(msg.empty());
    EXPECT_NE(msg.find("satellite_contact_time_s"), std::string::npos);
    EXPECT_NE(msg.find("allow_contact_time_mismatch"), std::string::npos);
    in.allow_contact_time_mismatch = true;
    const ContinuousConfig c = validateContinuous(in);
    EXPECT_TRUE(c.contact_time_mismatch);
    EXPECT_TRUE(c.contact_time_mismatch_allowed);
    // inside the tolerance: fine, not flagged
    in = validInputs(true);
    in.contact_time_s = 60.972930046 + 0.005 + 0.09;
    EXPECT_FALSE(validateContinuous(in).contact_time_mismatch);
}
