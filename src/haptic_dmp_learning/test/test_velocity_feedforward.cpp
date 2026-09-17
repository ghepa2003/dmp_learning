#include <gtest/gtest.h>
#include "haptic_dmp_learning/core/dmp.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/quaternion_dmp.hpp"
#include <vector>
#include <cmath>

using namespace haptic_dmp_learning::core;

// Sanity check for the velocity_ik_solver feedforward work: DMP::velocity(),
// ProDMP::velocity() and QuaternionDMP::omega() are claimed to be native
// analytic state (already integrated inside step(), see core/dmp.cpp,
// core/prodmp.cpp, core/quaternion_dmp.cpp), NOT a finite-difference
// reconstruction. This is the cheapest possible check of that claim: roll
// out each model, record position/orientation at every tick, and compare the
// analytic velocity()/omega() returned by step() against a central
// finite-difference derivative computed independently from the recorded
// position/orientation samples. Run this BEFORE any Gazebo campaign - a
// mismatch here means the feedforward is wired to the wrong quantity (wrong
// frame, wrong scaling, stale state) and no Gazebo run will explain why.

namespace {

std::vector<Sample> makeMinimumJerkDemo(const Eigen::Vector3d& p_start,
                                         const Eigen::Vector3d& p_goal, int n, double dt) {
    std::vector<Sample> demo(n);
    for (int i = 0; i < n; ++i) {
        double s = static_cast<double>(i) / (n - 1);
        double poly = s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);  // minimum-jerk profile
        demo[i].t = i * dt;
        demo[i].position = p_start + poly * (p_goal - p_start);
        // Orientation sweeps a fixed axis by a fixed angle with the same profile,
        // so QuaternionDMP has a non-trivial demonstrated rotation to learn.
        Eigen::Vector3d axis(0.0, 0.0, 1.0);
        demo[i].orientation = Eigen::Quaterniond(Eigen::AngleAxisd(poly * 0.8, axis));
    }
    return demo;
}

}  // namespace

TEST(VelocityFeedforwardSanityCheck, DmpVelocityMatchesFiniteDifference) {
    const int N_demo = 100;
    const double dt_demo = 0.02;
    auto demo = makeMinimumJerkDemo(Eigen::Vector3d(0.0, 0.0, 0.0),
                                     Eigen::Vector3d(0.3, -0.2, 0.4), N_demo, dt_demo);

    DMP dmp(25, 1.0, 25.0, 6.25, false);
    dmp.learnFromDemonstration(demo);
    dmp.reset();

    // Fine rollout step, independent of the (coarser) demo sampling.
    const double dt = 0.002;
    const int n_steps = static_cast<int>(dmp.tau() / dt);

    std::vector<Eigen::Vector3d> pos(n_steps + 1);
    std::vector<Eigen::Vector3d> vel_analytic(n_steps + 1);
    pos[0] = dmp.position();
    vel_analytic[0] = dmp.velocity();
    for (int k = 1; k <= n_steps; ++k) {
        pos[k] = dmp.step(dt);
        vel_analytic[k] = dmp.velocity();
    }

    // Central finite difference on the recorded position samples, independent
    // of dmp.velocity() - compare only at interior points (needs k-1 and k+1).
    double max_abs_err = 0.0;
    for (int k = 1; k < n_steps; ++k) {
        Eigen::Vector3d vel_fd = (pos[k + 1] - pos[k - 1]) / (2.0 * dt);
        double err = (vel_fd - vel_analytic[k]).norm();
        max_abs_err = std::max(max_abs_err, err);
    }

    // O(dt^2) trapezoid/central-difference error on a smooth trajectory at
    // dt=0.002s should be tiny; a wrong quantity (wrong scaling/frame/stale
    // state) would show up as an error orders of magnitude larger than this.
    EXPECT_LT(max_abs_err, 1e-3) << "DMP::velocity() diverges from the finite-difference "
                                     "derivative of DMP::position() by more than 1 mm/s - "
                                     "check core::DMP::velocity() scaling (z_ / tau_).";
}

TEST(VelocityFeedforwardSanityCheck, ProDmpVelocityMatchesFiniteDifference) {
    const int N_demo = 100;
    const double dt_demo = 0.02;
    auto demo = makeMinimumJerkDemo(Eigen::Vector3d(0.0, 0.0, 0.0),
                                     Eigen::Vector3d(0.3, -0.2, 0.4), N_demo, dt_demo);

    ProDMP prodmp(20, 25.0, 4.6, 1e-9);
    prodmp.learnFromDemonstration(demo);

    const double dt = 0.002;
    const int n_steps = static_cast<int>(prodmp.tau() / dt);

    std::vector<Eigen::Vector3d> pos(n_steps + 1);
    std::vector<Eigen::Vector3d> vel_analytic(n_steps + 1);
    pos[0] = prodmp.position();
    vel_analytic[0] = prodmp.velocity();
    for (int k = 1; k <= n_steps; ++k) {
        pos[k] = prodmp.step(dt);
        vel_analytic[k] = prodmp.velocity();
    }

    double max_abs_err = 0.0;
    for (int k = 1; k < n_steps; ++k) {
        Eigen::Vector3d vel_fd = (pos[k + 1] - pos[k - 1]) / (2.0 * dt);
        double err = (vel_fd - vel_analytic[k]).norm();
        max_abs_err = std::max(max_abs_err, err);
    }

    EXPECT_LT(max_abs_err, 1e-3) << "ProDMP::velocity() diverges from the finite-difference "
                                     "derivative of ProDMP::position() by more than 1 mm/s - "
                                     "check core::ProDMP::velocity() (vel_ from step()).";
}

TEST(VelocityFeedforwardSanityCheck, QuaternionDmpOmegaMatchesFiniteDifference) {
    const int N_demo = 100;
    const double dt_demo = 0.02;
    auto demo = makeMinimumJerkDemo(Eigen::Vector3d(0.0, 0.0, 0.0),
                                     Eigen::Vector3d(0.3, -0.2, 0.4), N_demo, dt_demo);

    QuaternionDMP qdmp(20, 4.6, 25.0, 6.25);
    qdmp.learnFromDemonstration(demo);
    qdmp.reset();

    const double dt = 0.002;
    const int n_steps = static_cast<int>(qdmp.tau() / dt);

    std::vector<Eigen::Quaterniond> quat(n_steps + 1);
    std::vector<Eigen::Vector3d> omega_analytic(n_steps + 1);
    quat[0] = qdmp.orientation();
    omega_analytic[0] = qdmp.omega();
    for (int k = 1; k <= n_steps; ++k) {
        quat[k] = qdmp.step(dt);
        omega_analytic[k] = qdmp.omega();
    }

    // Central finite-difference angular velocity from the recorded quaternion
    // samples alone, independent of qdmp.omega(): omega ~= 2 * logMap(q_{k+1} *
    // q_{k-1}^-1) / (2*dt), same logMap() convention used internally (and by
    // franka_cartesian_control::core::logMap for the pose error).
    double max_abs_err = 0.0;
    for (int k = 1; k < n_steps; ++k) {
        Eigen::Quaterniond dq = quat[k + 1] * quat[k - 1].conjugate();
        Eigen::Vector3d omega_fd = 2.0 * QuaternionDMP::logMap(dq) / (2.0 * dt);
        double err = (omega_fd - omega_analytic[k]).norm();
        max_abs_err = std::max(max_abs_err, err);
    }

    EXPECT_LT(max_abs_err, 1e-2) << "QuaternionDMP::omega() diverges from the finite-difference "
                                     "angular velocity of QuaternionDMP::orientation() by more "
                                     "than 0.01 rad/s - check core::QuaternionDMP::omega() "
                                     "scaling (eta_ / tau_) and frame convention.";
}
