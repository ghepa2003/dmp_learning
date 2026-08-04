#include "haptic_dmp_learning/core/dmp.hpp"
#include <cmath>
#include <stdexcept>
#include <iostream>
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
    // Equispaced centers in the normalized time [0,1], then mapped to phase space via the canonical system decay
    // - standard approach (Ijspeert).

    Eigen::VectorXd t_norm = Eigen::VectorXd::LinSpaced(n_basis_, 0.0, 1.0);
    centers_.resize(n_basis_);
    // Compute the centers in phase space according to the canonical system dynamics.
    for (int i = 0; i < n_basis_; ++i) {
        if (second_order_canonical_) {
            double a = alpha_z_ / 2.0; 
            // Closed-form solution of the second-order canonical system with critical damping: x(t) = (1 + a * t) * exp(-a * t)
            centers_(i) = (1.0 + a * t_norm(i)) * std::exp(-a * t_norm(i));
        } else {
            // First-order canonical system: x(t) = exp(-alpha_x * t)
            centers_(i) = std::exp(-alpha_x_ * t_norm(i));
        }
    }
    widths_ = Eigen::VectorXd::Zero(n_basis_);
    for (int i = 0; i < n_basis_; ++i) {
        if (i < n_basis_ - 1) {
            double d = (centers_(i + 1) - centers_(i)) * 0.55;  // overlap factor from the reference implementation (used by Schaal/Ijspeert)
            widths_(i) = 1.0 / (d * d);
        } else {
            widths_(i) = widths_(i - 1);
        }
    }
}

double DMP::basisFunction(int i, double x) const {
    // Direct implmentation of the Gaussian basis function: psi_i(x) = exp(-h_i * (x - c_i)^2)
    double d = x - centers_(i);
    return std::exp(-widths_(i) * d * d);
}

std::vector<Eigen::Vector3d> DMP::movingAverageSmooth(const std::vector<Eigen::Vector3d>& signal,
                                                        const std::vector<double>& t, double window_sec) const {
    const size_t N = signal.size();
    if (window_sec <= 0.0 || N < 2) return signal;
    double dt_est = (t.back() - t.front()) / static_cast<double>(N - 1);
    if (dt_est <= 0.0) return signal;
    int window_samples = std::max(1, static_cast<int>(std::round(window_sec / dt_est)));
    int half = window_samples / 2;
    std::vector<Eigen::Vector3d> out(N);
    for (size_t k = 0; k < N; ++k) {
        int lo = std::max(0, static_cast<int>(k) - half);
        int hi = std::min(static_cast<int>(N) - 1, static_cast<int>(k) + half);
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        int count = 0;
        for (int j = lo; j <= hi; ++j) { sum += signal[j]; ++count; }
        out[k] = sum / static_cast<double>(count);
    }
    return out;
}


void DMP::learnFromDemonstration(const std::vector<Sample>& demo) {

    // Exception for dealing with too-short demonstrations or non-increasing timestamps.
    if (demo.size() < 5) {
        throw std::runtime_error(
            "DMP::learnFromDemonstration: demonstration too short (need at least 5 samples).");
    }

    // 
    const size_t N = demo.size();
    tau_ = demo.back().t - demo.front().t;
    if (tau_ <= 0.0) {
        throw std::runtime_error(
            "DMP::learnFromDemonstration: non-increasing timestamps in demonstration.");
    }

    y0_ = demo.front().position;
    goal_ = demo.back().position;

    // Canonical system phase x(t) is a simple exponential decay: no need to
    // integrate it forward, just evaluate it analytically at each sample time.
    // x(t) = exp(-alpha_x / tau * t_rel), where t_rel = t - t0
    std::vector<double> x_t(N);
    for (size_t k = 0; k < N; ++k) {
    double t_rel = demo[k].t - demo.front().t;
    double t_norm = t_rel / tau_;
    if (second_order_canonical_) {
        double a = alpha_z_ / 2.0;
        x_t[k] = (1.0 + a * t_norm) * std::exp(-a * t_norm);  // closed-form solution of the second-order canonical system with critical damping
    } else {
        x_t[k] = std::exp(-alpha_x_ * t_norm);
    }
}

    // Estimate velocity and acceleration via central finite differences.
    // Se il filtro e' attivo, la media mobile viene applicata PRIMA di
    // ciascuno dei due stadi di derivazione (non solo sul segnale grezzo in
    // ingresso) -- uno smoothing singolo migliorerebbe solo la velocita',
    // lasciando l'accelerazione esposta al rumore introdotto dalla prima
    // derivazione stessa.
    std::vector<double> t_all(N);
    std::vector<Eigen::Vector3d> pos_all(N);
    for (size_t k = 0; k < N; ++k) {
        t_all[k] = demo[k].t;
        pos_all[k] = demo[k].position;
    }
    std::vector<Eigen::Vector3d> pos_for_diff = use_velocity_filter_
        ? movingAverageSmooth(pos_all, t_all, filter_window_sec_1_)
        : pos_all;

    std::vector<Eigen::Vector3d> vel(N), acc(N);
    for (size_t k = 0; k < N; ++k) {
        size_t km1 = (k == 0) ? 0 : k - 1;
        size_t kp1 = (k == N - 1) ? N - 1 : k + 1;
        double dt = t_all[kp1] - t_all[km1];
        if (dt <= 0.0) dt = 1e-6;
        vel[k] = (pos_for_diff[kp1] - pos_for_diff[km1]) / dt;
    }

    if (use_velocity_filter_) {
        vel = movingAverageSmooth(vel, t_all, filter_window_sec_2_);
    }

    std::cerr << "[DMP diag] |vel(0)| = " << vel.front().norm()
              << " m/s, |vel(N-1)| = " << vel.back().norm() << " m/s ("
              << "|z(0)| = " << (tau_ * vel.front()).norm()
              << ", |z(N-1)| = " << (tau_ * vel.back()).norm() << " scaled)\n";

    z0_ = tau_ * vel.front();

    for (size_t k = 0; k < N; ++k) {
        size_t km1 = (k == 0) ? 0 : k - 1;
        size_t kp1 = (k == N - 1) ? N - 1 : k + 1;
        double dt = t_all[kp1] - t_all[km1];
        if (dt <= 0.0) dt = 1e-6;
        acc[k] = (vel[kp1] - vel[km1]) / dt;
    }

    // Precompute the per-sample forcing-term target for all three dimensions
    // (shared between the independent-LWR and ridge paths below).
    std::vector<Eigen::Vector3d> f_target(N);
    for (size_t k = 0; k < N; ++k) {
        for (int d = 0; d < 3; ++d) {
            f_target[k](d) = tau_ * tau_ * acc[k](d) -
                              alpha_z_ * (beta_z_ * (goal_(d) - demo[k].position(d)) - tau_ * vel[k](d));
        }
    }

    if (use_ridge_regression_) {
        // Joint ridge regression per dimension: Phi(k,i) = psi_i(x_k) * x_k,
        // w_d = (Phi^T Phi + lambda I)^-1 Phi^T f_d. A differenza della LWR
        // indipendente sotto, questa tiene conto dei termini incrociati tra
        // basi sovrapposte (il Gram off-diagonal non e' assunto nullo).
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
        // Locally weighted regression, independently per dimension and per basis
        // function - standard closed-form DMP weight fitting (original behavior).
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

    // Compute the observed amplitude of the forcing term per dimension, for later scaling checks.
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
    // Check if the new goal is too far from the original goal, in which case we might not want to scale the forcing term.
    constexpr double kAmplitudeRatioThreshold = 2.0;  
    constexpr double kMinDG = 1e-6;  
    for (int d = 0; d < 3; ++d) {
        double new_dG = goal(d) - y0_(d);
        if (std::abs(dG_(d)) < kMinDG) {
            scale_reliable_[d] = false;
            // If the original movement amplitude is too small, we cannot reliably scale the forcing term. Set scale to 1.0 and mark it as unreliable.
            scale_(d) = 1.0; 
            continue;
        }
        // Check the ratio of the learned amplitude to the new amplitude. If it's too large, mark the scale as unreliable.
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
    // Set the learned parameters directly, bypassing the learning step. This is used when loading a DMP from saved parameters.                                
    tau_ = tau; y0_ = y0; goal_ = goal;
    dG_ = dG; A_ = A;
    z0_ = z0;
    scale_ = Eigen::Vector3d::Ones();
    scale_reliable_.fill(true);
    centers_ = centers; widths_ = widths; weights_ = weights;
    n_basis_ = static_cast<int>(centers.size());
    learned_ = true;
}

Eigen::Vector3d DMP::step(double dt, const Eigen::Vector3d& ct, double cc) {
    // Canonical system: tau * dx = -alpha_x * x
    // Integrate forward with simple Euler step. x_ is clamped to [0,1].
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

    // Forcing term (normalized weighted sum of Gaussian kernels), per dimension
    Eigen::VectorXd psi(n_basis_);
    double psi_sum = 0.0;
    for (int i = 0; i < n_basis_; ++i) {
        psi(i) = basisFunction(i, x_);
        psi_sum += psi(i);
    }

    // Guard against the case where all basis functions are effectively zero (e.g., x_ is very small and all kernels are far away).
    if (psi_sum < 1e-8) psi_sum = 1e-8;

    // Compute the forcing term f(x) for each dimension, scaled by the learned weights and the current phase x_.
    Eigen::Vector3d f = Eigen::Vector3d::Zero();
    for (int d = 0; d < 3; ++d) {
        double weighted = 0.0;
        for (int i = 0; i < n_basis_; ++i) {
            weighted += weights_[d](i) * psi(i);
        }
        f(d) = (weighted / psi_sum) * x_ * scale_(d);
    }

    // Transformation system: tau*dz = alpha_z(beta_z(g-y)-z) + f ; tau*dy = z
    Eigen::Vector3d dz = (alpha_z_ * (beta_z_ * (goal_ - y_) - z_) + f + ct) / tau_;
    Eigen::Vector3d dy = z_ / tau_;

    z_ += dz * dt;
    y_ += dy * dt;

    return y_;
}

}  // namespace core
}  // namespace haptic_dmp_learning
