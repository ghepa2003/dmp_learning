#include "haptic_dmp_learning/core/dmp.hpp"
#include <cmath>
#include <stdexcept>

namespace haptic_dmp_learning {
namespace core {

DMP::DMP(int n_basis, double alpha_x, double alpha_z, double beta_z)
    : n_basis_(n_basis),
      alpha_x_(alpha_x),
      alpha_z_(alpha_z),
      beta_z_(beta_z),
      tau_(1.0),
      y0_(Eigen::Vector3d::Zero()),
      goal_(Eigen::Vector3d::Zero()),
      x_(1.0),
      y_(Eigen::Vector3d::Zero()),
      z_(Eigen::Vector3d::Zero()),
      learned_(false) {
    for (auto& w : weights_) {
        w = Eigen::VectorXd::Zero(n_basis_);
    }
    initBasisFunctions();
}

void DMP::initBasisFunctions() {
    // Centers equally spaced in phase space x in (0, 1] (simplified variant
    // of the classic Ijspeert placement, which spaces them in time and then
    // maps through the canonical system - equivalent in spirit, simpler to
    // implement, and fine for a first working version).
    centers_ = Eigen::VectorXd::LinSpaced(n_basis_, 1.0, 1e-3);
    widths_ = Eigen::VectorXd::Zero(n_basis_);
    for (int i = 0; i < n_basis_; ++i) {
        if (i < n_basis_ - 1) {
            double d = centers_(i + 1) - centers_(i);
            widths_(i) = 1.0 / (d * d);
        } else {
            widths_(i) = widths_(i - 1);
        }
    }
}

double DMP::basisFunction(int i, double x) const {
    double d = x - centers_(i);
    return std::exp(-widths_(i) * d * d);
}

void DMP::learnFromDemonstration(const std::vector<Sample>& demo) {
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

    // Canonical system phase x(t) is a simple exponential decay: no need to
    // integrate it forward, just evaluate it analytically at each sample time.
    std::vector<double> x_t(N);
    for (size_t k = 0; k < N; ++k) {
        double t_rel = demo[k].t - demo.front().t;
        x_t[k] = std::exp(-alpha_x_ / tau_ * t_rel);
    }

    // Estimate velocity and acceleration via central finite differences.
    // NOTE: haptic device data is typically noisy. If the learned DMP looks
    // jittery when replayed, the first thing to try is low-pass filtering
    // position (or velocity) before this step, rather than differentiating
    // the raw samples directly.
    std::vector<Eigen::Vector3d> vel(N), acc(N);
    for (size_t k = 0; k < N; ++k) {
        size_t km1 = (k == 0) ? 0 : k - 1;
        size_t kp1 = (k == N - 1) ? N - 1 : k + 1;
        double dt = demo[kp1].t - demo[km1].t;
        if (dt <= 0.0) dt = 1e-6;
        vel[k] = (demo[kp1].position - demo[km1].position) / dt;
    }
    for (size_t k = 0; k < N; ++k) {
        size_t km1 = (k == 0) ? 0 : k - 1;
        size_t kp1 = (k == N - 1) ? N - 1 : k + 1;
        double dt = demo[kp1].t - demo[km1].t;
        if (dt <= 0.0) dt = 1e-6;
        acc[k] = (vel[kp1] - vel[km1]) / dt;
    }

    // Locally weighted regression, independently per dimension and per basis
    // function - standard closed-form DMP weight fitting.
    for (int d = 0; d < 3; ++d) {
        Eigen::VectorXd num = Eigen::VectorXd::Zero(n_basis_);
        Eigen::VectorXd den = Eigen::VectorXd::Zero(n_basis_);

        double scale = goal_(d) - y0_(d);
        // Guard against the ill-conditioning case documented in the thesis
        // (goal ~= start): fall back to a small nonzero scale to avoid
        // dividing by ~0. Goal generalization along this dimension will not
        // rescale correctly afterwards - accepted limitation for now.
        if (std::abs(scale) < 1e-4) {
            scale = (scale >= 0.0) ? 1e-4 : -1e-4;
        }

        for (size_t k = 0; k < N; ++k) {
            double f_target = tau_ * tau_ * acc[k](d) -
                               alpha_z_ * (beta_z_ * (goal_(d) - demo[k].position(d)) - tau_ * vel[k](d));
            double s = x_t[k] * scale;  // forcing-term regressor: x * (g - y0)

            for (int i = 0; i < n_basis_; ++i) {
                double psi = basisFunction(i, x_t[k]);
                num(i) += psi * s * f_target;
                den(i) += psi * s * s;
            }
        }

        for (int i = 0; i < n_basis_; ++i) {
            weights_[d](i) = (den(i) > 1e-8) ? num(i) / den(i) : 0.0;
        }
    }

    learned_ = true;
}

void DMP::reset() {
    x_ = 1.0;
    y_ = y0_;
    z_ = Eigen::Vector3d::Zero();
}

void DMP::setGoal(const Eigen::Vector3d& goal) {
    goal_ = goal;
}

void DMP::setLearnedParameters(double tau, const Eigen::Vector3d& y0, const Eigen::Vector3d& goal,
                                const Eigen::VectorXd& centers, const Eigen::VectorXd& widths,
                                const std::array<Eigen::VectorXd, 3>& weights) {
    tau_ = tau;
    y0_ = y0;
    goal_ = goal;
    centers_ = centers;
    widths_ = widths;
    weights_ = weights;
    n_basis_ = static_cast<int>(centers.size());
    learned_ = true;
}

Eigen::Vector3d DMP::step(double dt) {
    // Canonical system: tau * dx = -alpha_x * x
    double dx = -alpha_x_ / tau_ * x_;
    x_ += dx * dt;
    if (x_ < 0.0) x_ = 0.0;

    // Forcing term (normalized weighted sum of Gaussian kernels), per dimension
    Eigen::VectorXd psi(n_basis_);
    double psi_sum = 0.0;
    for (int i = 0; i < n_basis_; ++i) {
        psi(i) = basisFunction(i, x_);
        psi_sum += psi(i);
    }
    if (psi_sum < 1e-8) psi_sum = 1e-8;

    Eigen::Vector3d f = Eigen::Vector3d::Zero();
    for (int d = 0; d < 3; ++d) {
        double weighted = 0.0;
        for (int i = 0; i < n_basis_; ++i) {
            weighted += weights_[d](i) * psi(i);
        }
        f(d) = (weighted / psi_sum) * x_ * (goal_(d) - y0_(d));
    }

    // Transformation system: tau*dz = alpha_z(beta_z(g-y)-z) + f ; tau*dy = z
    Eigen::Vector3d dz = (alpha_z_ * (beta_z_ * (goal_ - y_) - z_) + f) / tau_;
    Eigen::Vector3d dy = z_ / tau_;

    z_ += dz * dt;
    y_ += dy * dt;

    return y_;
}

}  // namespace core
}  // namespace haptic_dmp_learning
