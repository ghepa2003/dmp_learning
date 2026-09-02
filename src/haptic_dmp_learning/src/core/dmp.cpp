#include "haptic_dmp_learning/core/dmp.hpp"
#include "haptic_dmp_learning/core/filter_utils.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace haptic_dmp_learning {
namespace core {

DMP::DMP(int n_basis, double alpha_x, double alpha_z, double beta_z, bool second_order_canonical)
    : n_basis_(n_basis),
      alpha_x_(alpha_x),
      alpha_z_(alpha_z),
      beta_z_(beta_z),
      second_order_canonical_(second_order_canonical),
      tau_(1.0),
      y0_(Eigen::Vector3d::Zero()),
      goal_(Eigen::Vector3d::Zero()),
      x_(1.0),
      v_(0.0),
      y_(Eigen::Vector3d::Zero()),
      z_(Eigen::Vector3d::Zero()),
      z0_(Eigen::Vector3d::Zero()),
      learned_(false),
      dG_(Eigen::Vector3d::Zero()),
      A_(Eigen::Vector3d::Zero()),
      scale_(Eigen::Vector3d::Ones()) {
    scale_reliable_.fill(true);
    for (auto& w : weights_) {
        w = Eigen::VectorXd::Zero(n_basis_);
    }
    initBasisFunctions();
}

void DMP::initBasisFunctions() {
    // 1. Distribute kernel centers c_i: equispaced in normalized time t_norm in [0, 1],
    // then mapped to phase space via the analytic decay law of the canonical system.
    Eigen::VectorXd t_norm = Eigen::VectorXd::LinSpaced(n_basis_, 0.0, 1.0);
    centers_.resize(n_basis_);

    for (int i = 0; i < n_basis_; ++i) {
        if (second_order_canonical_) {
            double a = alpha_z_ / 2.0; 
            // Closed-form critically damped 2nd-order impulse response: x(t) = (1 + a*t) * exp(-a*t)
            centers_(i) = (1.0 + a * t_norm(i)) * std::exp(-a * t_norm(i));
        } else {
            // First-order exponential decay: x(t) = exp(-alpha_x * t)
            centers_(i) = std::exp(-alpha_x_ * t_norm(i));
        }
    }

    // 2. Compute Gaussian kernel bandwidths h_i:
    // Chosen so neighboring basis functions overlap at ~55% of their peak height:
    //   h_i = 1 / ( (c_{i+1} - c_i) * 0.55 )^2
    widths_ = Eigen::VectorXd::Zero(n_basis_);
    for (int i = 0; i < n_basis_; ++i) {
        if (i < n_basis_ - 1) {
            double d = (centers_(i + 1) - centers_(i)) * 0.55;
            widths_(i) = 1.0 / (d * d);
        } else {
            widths_(i) = widths_(i - 1);
        }
    }
}

double DMP::basisFunction(int i, double x) const {
    // Evaluates Gaussian Radial Basis Function: psi_i(x) = exp( -h_i * (x - c_i)^2 )
    double d = x - centers_(i);
    return std::exp(-widths_(i) * d * d);
}

void DMP::learnFromDemonstration(const std::vector<Sample>& demo_in) {
    // 0. Defence in depth: drop samples with a non-increasing timestamp before
    // any finite differencing. A repeated t (dt=0) would otherwise be clamped
    // to 1e-6 s below and produce a huge spurious velocity/acceleration spike
    // that the regression fits into oversized weights. The recorder is meant to
    // never emit such rows; if it does, diag_.dropped_non_monotonic_samples
    // makes it visible in the learning logs instead of only in anomalous weights.
    int dropped_samples = 0;
    const std::vector<Sample> demo =
        filter_utils::dropNonIncreasingTimeSamples(demo_in, &dropped_samples);
    diag_.dropped_non_monotonic_samples = dropped_samples;

    // 1. Validation of demonstration length and temporal monotonicity
    if (demo.size() < 5) {
        throw std::runtime_error(
            "DMP::learnFromDemonstration: demonstration too short (need at least 5 samples).");
    }

    const size_t N = demo.size();
    tau_ = demo.back().t - demo.front().t;
    if (tau_ <= 0.0) {
        throw std::runtime_error(
            "DMP::learnFromDemonstration: non-increasing timestamps in demonstration.");
    }

    y0_ = demo.front().position;
    goal_ = demo.back().position;

    // 2. Compute phase trajectory x(t) analytically at each sample time
    std::vector<double> x_t(N);
    for (size_t k = 0; k < N; ++k) {
        double t_rel = demo[k].t - demo.front().t;
        double t_norm = t_rel / tau_;
        if (second_order_canonical_) {
            double a = alpha_z_ / 2.0;
            x_t[k] = (1.0 + a * t_norm) * std::exp(-a * t_norm);
        } else {
            x_t[k] = std::exp(-alpha_x_ * t_norm);
        }
    }

    // 3. Extract time and position trajectories
    std::vector<double> t_all(N);
    std::vector<Eigen::Vector3d> pos_all(N);
    for (size_t k = 0; k < N; ++k) {
        t_all[k] = demo[k].t;
        pos_all[k] = demo[k].position;
    }

    // 4. Pre-filtering: optionally smooth position before numerical differentiation
    std::vector<Eigen::Vector3d> pos_for_diff = use_velocity_filter_
        ? filter_utils::movingAverageSmooth(pos_all, t_all, filter_window_sec_1_)
        : pos_all;

    // 5. Compute velocities via central finite differences: dy/dt ~ (y[k+1] - y[k-1]) / (t[k+1] - t[k-1])
    std::vector<Eigen::Vector3d> vel(N), acc(N);
    for (size_t k = 0; k < N; ++k) {
        size_t km1 = (k == 0) ? 0 : k - 1;
        size_t kp1 = (k == N - 1) ? N - 1 : k + 1;
        double dt = t_all[kp1] - t_all[km1];
        if (dt <= 0.0) dt = 1e-6;
        vel[k] = (pos_for_diff[kp1] - pos_for_diff[km1]) / dt;
    }

    // 6. Post-filtering of velocity signal
    if (use_velocity_filter_) {
        vel = filter_utils::movingAverageSmooth(vel, t_all, filter_window_sec_2_);
    }

    // 7. Store boundary diagnostics and initial velocity offset z0
    diag_.initial_vel_norm = vel.front().norm();
    diag_.final_vel_norm = vel.back().norm();
    diag_.initial_z_norm = (tau_ * vel.front()).norm();
    diag_.final_z_norm = (tau_ * vel.back()).norm();
    z0_ = tau_ * vel.front();

    // 8. Compute accelerations via central finite differences on smoothed velocity
    for (size_t k = 0; k < N; ++k) {
        size_t km1 = (k == 0) ? 0 : k - 1;
        size_t kp1 = (k == N - 1) ? N - 1 : k + 1;
        double dt = t_all[kp1] - t_all[km1];
        if (dt <= 0.0) dt = 1e-6;
        acc[k] = (vel[kp1] - vel[km1]) / dt;
    }

    // 9. Compute target forcing signal f_target(t) by inverting the transformation system:
    //    f_target,d(t) = tau^2 * d^2y/dt^2 - alpha_z * (beta_z * (goal - y) - tau * dy/dt)
    std::vector<Eigen::Vector3d> f_target(N);
    for (size_t k = 0; k < N; ++k) {
        for (int d = 0; d < 3; ++d) {
            f_target[k](d) = tau_ * tau_ * acc[k](d) -
                              alpha_z_ * (beta_z_ * (goal_(d) - demo[k].position(d)) - tau_ * vel[k](d));
        }
    }

    // 10. Fit kernel weights w_{d,i}
    if (use_ridge_regression_) {
        // Global Ridge Regression (L2 regularized):
        // Design matrix Phi (N x M): Phi(k, i) = (psi_i(x_k) / sum_j psi_j(x_k)) * x_k
        // Normal equation: w_d = (Phi^T Phi + lambda * I)^-1 * Phi^T * f_d
        Eigen::MatrixXd Phi(N, n_basis_);
        Eigen::VectorXd psi_row(n_basis_);
        for (size_t k = 0; k < N; ++k) {
            double psi_sum = 0.0;
            for (int i = 0; i < n_basis_; ++i) {
                psi_row(i) = basisFunction(i, x_t[k]);
                psi_sum += psi_row(i);
            }
            if (psi_sum < 1e-8) psi_sum = 1e-8;
            for (int i = 0; i < n_basis_; ++i) {
                Phi(static_cast<int>(k), i) = (psi_row(i) / psi_sum) * x_t[k];
            }
        }
        Eigen::MatrixXd Gram = Phi.transpose() * Phi;
        Gram.diagonal().array() += ridge_lambda_;
        Eigen::LDLT<Eigen::MatrixXd> solver(Gram);

        for (int d = 0; d < 3; ++d) {
            Eigen::VectorXd f_d(N);
            for (size_t k = 0; k < N; ++k) f_d(static_cast<int>(k)) = f_target[k](d);
            weights_[d] = solver.solve(Phi.transpose() * f_d);
        }
    } else {
        // Standard Locally Weighted Regression (LWR):
        // w_{d,i} = sum_k (psi_i(x_k) * x_k * f_target[k](d)) / sum_k (psi_i(x_k) * x_k^2)
        for (int d = 0; d < 3; ++d) {
            Eigen::VectorXd num = Eigen::VectorXd::Zero(n_basis_);
            Eigen::VectorXd den = Eigen::VectorXd::Zero(n_basis_);

            for (size_t k = 0; k < N; ++k) {
                double s = x_t[k];
                for (int i = 0; i < n_basis_; ++i) {
                    double psi = basisFunction(i, x_t[k]);
                    num(i) += psi * s * f_target[k](d);
                    den(i) += psi * s * s;
                }
            }
            for (int i = 0; i < n_basis_; ++i) {
                weights_[d](i) = (den(i) > 1e-8) ? num(i) / den(i) : 0.0;
            }
        }
    }

    // 11. Record original displacement dG = goal - y0 and bounding amplitude A per dimension
    dG_ = goal_ - y0_;
    scale_ = Eigen::Vector3d::Ones();  
    scale_reliable_.fill(true);

    for (int d = 0; d < 3; ++d) {
        double min_v = demo.front().position(d), max_v = min_v;
        for (const auto& s : demo) {
            min_v = std::min(min_v, s.position(d));
            max_v = std::max(max_v, s.position(d));
        }
        A_(d) = max_v - min_v;
    }

    learned_ = true;
}

void DMP::reset() {
    x_ = 1.0;
    y_ = y0_;
    z_ = z0_;
    v_ = 0.0;
}

void DMP::setGoal(const Eigen::Vector3d& goal) {
    constexpr double kAmplitudeRatioThreshold = 2.0;  
    constexpr double kMinDG = 1e-6;  

    for (int d = 0; d < 3; ++d) {
        double new_dG = goal(d) - y0_(d);
        // Guard against division by zero if demonstrated motion had zero net displacement along axis d
        if (std::abs(dG_(d)) < kMinDG) {
            scale_reliable_[d] = false;
            scale_(d) = 1.0; 
            continue;
        }

        // Verify that the demonstrated motion did not exhibit large reversals relative to net displacement
        double ratio = A_(d) / std::abs(dG_(d));
        if (ratio > kAmplitudeRatioThreshold) {
            scale_reliable_[d] = false;
            scale_(d) = 1.0;
        } else {
            scale_(d) = new_dG / dG_(d);
            scale_reliable_[d] = true;
        }
    }
    goal_ = goal;
}

void DMP::setLearnedParameters(double tau, const Eigen::Vector3d& y0, const Eigen::Vector3d& goal,
                                const Eigen::Vector3d& dG, const Eigen::Vector3d& A,
                                const Eigen::VectorXd& centers, const Eigen::VectorXd& widths,
                                const std::array<Eigen::VectorXd, 3>& weights, const Eigen::Vector3d& z0) {
    tau_ = tau;
    y0_ = y0;
    goal_ = goal;
    dG_ = dG;
    A_ = A;
    z0_ = z0;
    scale_ = Eigen::Vector3d::Ones();
    scale_reliable_.fill(true);
    centers_ = centers;
    widths_ = widths;
    weights_ = weights;
    n_basis_ = static_cast<int>(centers.size());
    learned_ = true;
}

Eigen::Vector3d DMP::step(double dt, const Eigen::Vector3d& ct, double cc) {
    // 1. Integrate canonical phase clock forward
    if (second_order_canonical_) {
        double dv = (alpha_z_ * (beta_z_ * (0.0 - x_) - v_) + cc) / tau_;
        double dx = v_ / tau_;
        v_ += dv * dt;
        x_ += dx * dt;
    } else {
        double dx = (-alpha_x_ * x_ + cc) / tau_;
        x_ += dx * dt;
    }
    if (x_ < 0.0) x_ = 0.0;

    // 2. Evaluate normalized Gaussian basis function kernels at current phase x_
    Eigen::VectorXd psi(n_basis_);
    double psi_sum = 0.0;
    for (int i = 0; i < n_basis_; ++i) {
        psi(i) = basisFunction(i, x_);
        psi_sum += psi(i);
    }
    if (psi_sum < 1e-8) psi_sum = 1e-8;

    // 3. Compute non-linear forcing function f(x):
    //    f_d(x) = ( sum_i psi_i(x) * w_{d,i} / sum_i psi_i(x) ) * x * s_d
    Eigen::Vector3d f = Eigen::Vector3d::Zero();
    for (int d = 0; d < 3; ++d) {
        double weighted = 0.0;
        for (int i = 0; i < n_basis_; ++i) {
            weighted += weights_[d](i) * psi(i);
        }
        f(d) = (weighted / psi_sum) * x_ * scale_(d);
    }

    // 4. Integrate transformation system:
    //    dz/dt = ( alpha_z * (beta_z * (goal - y) - z) + f + ct ) / tau
    //    dy/dt = z / tau
    Eigen::Vector3d dz = (alpha_z_ * (beta_z_ * (goal_ - y_) - z_) + f + ct) / tau_;
    Eigen::Vector3d dy = z_ / tau_;

    z_ += dz * dt;
    y_ += dy * dt;

    return y_;
}

}  // namespace core
}  // namespace haptic_dmp_learning
