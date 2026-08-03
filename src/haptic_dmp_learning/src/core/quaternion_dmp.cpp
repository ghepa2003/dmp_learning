#include "haptic_dmp_learning/core/quaternion_dmp.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>

namespace haptic_dmp_learning {
namespace core {

QuaternionDMP::QuaternionDMP(int n_basis, double alpha_x, double alpha_z, double beta_z)
    : n_basis_(n_basis), alpha_x_(alpha_x), alpha_z_(alpha_z), beta_z_(beta_z),
      tau_(1.0),
      q0_(Eigen::Quaterniond::Identity()),
      goal_(Eigen::Quaterniond::Identity()),
      x_(1.0),
      q_(Eigen::Quaterniond::Identity()),
      eta_(Eigen::Vector3d::Zero()),
      learned_(false) {
    for (auto& w : weights_) w = Eigen::VectorXd::Zero(n_basis_);
    initBasisFunctions();
}

// The basis functions are Gaussian kernels in the phase space, with centers and widths determined by the canonical system's decay.
// Same as the positional DMP, but here we use them for the forcing term in the quaternion DMP.
void QuaternionDMP::initBasisFunctions() {
    Eigen::VectorXd t_norm = Eigen::VectorXd::LinSpaced(n_basis_, 0.0, 1.0);
    centers_.resize(n_basis_);
    for (int i = 0; i < n_basis_; ++i) centers_(i) = std::exp(-alpha_x_ * t_norm(i));

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

double QuaternionDMP::basisFunction(int i, double x) const {
    double d = x - centers_(i);
    return std::exp(-widths_(i) * d * d);
}

// The log map from SO(3) to R^3, mapping a unit quaternion to its corresponding rotation vector (axis-angle representation).
Eigen::Vector3d QuaternionDMP::logMap(const Eigen::Quaterniond& q) {
    Eigen::Vector3d v(q.x(), q.y(), q.z());
    double vnorm = v.norm();
    if (vnorm < 1e-8) return Eigen::Vector3d::Zero();
    double w = std::max(-1.0, std::min(1.0, q.w()));  // clamp for safe acos
    double angle = std::acos(w);
    return angle * v / vnorm;
}

// The exp map from R^3 to SO(3), mapping a rotation vector back to a unit quaternion.
Eigen::Quaterniond QuaternionDMP::expMap(const Eigen::Vector3d& r) {
    double theta = r.norm();
    if (theta < 1e-8) return Eigen::Quaterniond::Identity();
    Eigen::Vector3d axis = r / theta;
    double s = std::sin(theta);
    return Eigen::Quaterniond(std::cos(theta), axis.x() * s, axis.y() * s, axis.z() * s);
}

// The learnFromDemonstration method computes the weights for the forcing term based on the provided demonstration samples.
void QuaternionDMP::learnFromDemonstration(const std::vector<Sample>& demo) {
    if (demo.size() < 5) {
        throw std::runtime_error("QuaternionDMP::learnFromDemonstration: demonstration too short.");
    }
    const size_t N = demo.size();
    tau_ = demo.back().t - demo.front().t;
    if (tau_ <= 0.0) {
        throw std::runtime_error("QuaternionDMP::learnFromDemonstration: non-increasing timestamps.");
    }
    q0_ = demo.front().orientation.normalized();
    goal_ = demo.back().orientation.normalized();

    std::vector<double> x_t(N);
    for (size_t k = 0; k < N; ++k) {
        double t_rel = demo[k].t - demo.front().t;
        x_t[k] = std::exp(-alpha_x_ / tau_ * t_rel);
    }

    // Compute angular velocity (eta) and its derivative (eta_dot) using central finite differences.
    std::vector<Eigen::Vector3d> eta(N);
    for (size_t k = 0; k < N; ++k) {
        size_t km1 = (k == 0) ? 0 : k - 1;
        size_t kp1 = (k == N - 1) ? N - 1 : k + 1;
        double dt = demo[kp1].t - demo[km1].t;
        if (dt <= 0.0) dt = 1e-6;
        Eigen::Quaterniond qk1 = demo[kp1].orientation.normalized();
        Eigen::Quaterniond qk0 = demo[km1].orientation.normalized();
        Eigen::Quaterniond dq = qk1 * qk0.conjugate();
        Eigen::Vector3d omega = 2.0 * logMap(dq) / dt;
        eta[k] = tau_ * omega;
    }

    eta0_ = eta.front();

    std::cerr << "[QuaternionDMP diag] |eta(0)| = " << eta.front().norm()
              << " rad/s (scaled by tau), |eta(N-1)| = " << eta.back().norm()
              << " rad/s (scaled by tau)\n";

    // Compute the derivative of eta (angular acceleration) using central finite differences.
    std::vector<Eigen::Vector3d> eta_dot(N);
    for (size_t k = 0; k < N; ++k) {
        size_t km1 = (k == 0) ? 0 : k - 1;
        size_t kp1 = (k == N - 1) ? N - 1 : k + 1;
        double dt = demo[kp1].t - demo[km1].t;
        if (dt <= 0.0) dt = 1e-6;
        eta_dot[k] = (eta[kp1] - eta[km1]) / dt;
    }

    // Precompute per-sample forcing-term target for all three dimensions
    // (shared between the independent-LWR and ridge paths below).
    std::vector<Eigen::Vector3d> f_target(N);
    for (size_t k = 0; k < N; ++k) {
        Eigen::Quaterniond qk = demo[k].orientation.normalized();
        Eigen::Vector3d log_err = logMap(goal_ * qk.conjugate());
        for (int d = 0; d < 3; ++d) {
            f_target[k](d) = tau_ * eta_dot[k](d) - alpha_z_ * (2.0 * beta_z_ * log_err(d) - eta[k](d));
        }
    }

    if (use_ridge_regression_) {
        // Joint ridge regression per dimension (stessa logica di
        // DMP::learnFromDemonstration): Phi(k,i) = psi_i(x_k) * x_k,
        // w_d = (Phi^T Phi + lambda I)^-1 Phi^T f_d.
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
        // function - original behavior.
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
    learned_ = true;
}

void QuaternionDMP::reset() {
    x_ = 1.0;
    q_ = q0_;
    eta_ = eta0_;
}

void QuaternionDMP::setGoal(const Eigen::Quaterniond& goal) {
    goal_ = goal.normalized();
}

void QuaternionDMP::setLearnedParameters(double tau, const Eigen::Quaterniond& q0, const Eigen::Quaterniond& goal,
                                          const Eigen::VectorXd& centers, const Eigen::VectorXd& widths,
                                          const std::array<Eigen::VectorXd, 3>& weights, const Eigen::Vector3d& eta0) {
    tau_ = tau;
    q0_ = q0.normalized();
    goal_ = goal.normalized();
    centers_ = centers;
    widths_ = widths;
    weights_ = weights;
    eta0_ = eta0;
    n_basis_ = static_cast<int>(centers.size());
    learned_ = true;
}

// The step function integrates the DMP forward in time by dt, updating the internal state and returning the current orientation.
Eigen::Quaterniond QuaternionDMP::step(double dt) {
    double dx = -alpha_x_ / tau_ * x_;
    x_ += dx * dt;
    if (x_ < 0.0) x_ = 0.0;

    // Compute the forcing term f(x) using the learned weights and the current phase x_.
    Eigen::VectorXd psi(n_basis_);
    double psi_sum = 0.0;
    for (int i = 0; i < n_basis_; ++i) {
        psi(i) = basisFunction(i, x_);
        psi_sum += psi(i);
    }
    if (psi_sum < 1e-8) psi_sum = 1e-8;

    // Compute the forcing term f(x) for each dimension, scaled by the learned weights and the current phase x_.
    Eigen::Vector3d f = Eigen::Vector3d::Zero();
    for (int d = 0; d < 3; ++d) {
        double weighted = 0.0;
        for (int i = 0; i < n_basis_; ++i) weighted += weights_[d](i) * psi(i);
        f(d) = (weighted / psi_sum) * x_;
    }

    // Transformation system: tau * d(eta)/dt = alpha_z * (2 * beta_z * log(goal * q^{-1}) - eta) + f
    Eigen::Vector3d log_err = logMap(goal_ * q_.conjugate());
    Eigen::Vector3d eta_dot = (alpha_z_ * (2.0 * beta_z_ * log_err - eta_) + f) / tau_;
    eta_ += eta_dot * dt;

    // Integration with exponential map: q(t+dt) = exp_q((dt/2tau)*eta) * q(t)
    Eigen::Quaterniond dq = expMap((dt / (2.0 * tau_)) * eta_);
    q_ = dq * q_;
    q_.normalize();  // Normalization (used by Fabisch/Böckmann)

    return q_;
}

}  // namespace core
}  // namespace haptic_dmp_learning