#include "haptic_dmp_learning/core/quaternion_dmp.hpp"
#include "haptic_dmp_learning/core/filter_utils.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm>
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

void QuaternionDMP::initBasisFunctions() {
    // 1. Distribute kernel centers c_i across canonical phase decay: c_i = exp(-alpha_x * t_norm_i)
    Eigen::VectorXd t_norm = Eigen::VectorXd::LinSpaced(n_basis_, 0.0, 1.0);
    centers_.resize(n_basis_);
    for (int i = 0; i < n_basis_; ++i) centers_(i) = std::exp(-alpha_x_ * t_norm(i));

    // 2. Bandwidths h_i with 55% Gaussian kernel overlap
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

Eigen::Vector3d QuaternionDMP::logMap(const Eigen::Quaterniond& q) {
    // Extracts half-angle rotation vector (theta / 2) * u from unit quaternion q
    Eigen::Vector3d v(q.x(), q.y(), q.z());
    double vnorm = v.norm();
    if (vnorm < 1e-8) return Eigen::Vector3d::Zero();
    double w = std::max(-1.0, std::min(1.0, q.w()));  // Clamp against numerical overshoot in acos
    double angle = std::acos(w);
    return angle * v / vnorm;
}

Eigen::Quaterniond QuaternionDMP::expMap(const Eigen::Vector3d& r) {
    // Maps rotation vector r = (theta / 2) * u back to unit quaternion on S^3
    double theta = r.norm();
    if (theta < 1e-8) return Eigen::Quaterniond::Identity();
    Eigen::Vector3d axis = r / theta;
    double s = std::sin(theta);
    return Eigen::Quaterniond(std::cos(theta), axis.x() * s, axis.y() * s, axis.z() * s);
}

std::vector<Eigen::Vector3d> QuaternionDMP::unwrapRotationVector(const std::vector<Sample>& demo) const {
    // Unwraps discrete orientation trajectory into continuous cumulative rotation vector in R^3
    // Prevents spurious +/- 2*pi phase wrap-around discontinuities during filtering
    const size_t N = demo.size();
    std::vector<Eigen::Vector3d> r(N, Eigen::Vector3d::Zero());
    for (size_t k = 1; k < N; ++k) {
        Eigen::Quaterniond qk = demo[k].orientation.normalized();
        Eigen::Quaterniond qkm1 = demo[k - 1].orientation.normalized();
        Eigen::Quaterniond dq = qk * qkm1.conjugate();
        Eigen::Vector3d incr = 2.0 * logMap(dq);
        r[k] = r[k - 1] + incr;
    }
    return r;
}

void QuaternionDMP::learnFromDemonstration(const std::vector<Sample>& demo_in) {
    // 0. Defence in depth: drop samples with a non-increasing timestamp before
    // any finite differencing (see DMP::learnFromDemonstration and
    // filter_utils::dropNonIncreasingTimeSamples). dt=0 rows would otherwise be
    // clamped to 1e-6 s below and blow up the angular velocity estimate.
    int dropped_samples = 0;
    const std::vector<Sample> demo =
        filter_utils::dropNonIncreasingTimeSamples(demo_in, &dropped_samples);
    diag_.dropped_non_monotonic_samples = dropped_samples;

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

    // 1. Analytic evaluation of canonical phase decay
    std::vector<double> x_t(N);
    for (size_t k = 0; k < N; ++k) {
        double t_rel = demo[k].t - demo.front().t;
        x_t[k] = std::exp(-alpha_x_ / tau_ * t_rel);
    }

    // 2. Compute angular velocity eta and its derivative eta_dot
    std::vector<Eigen::Vector3d> eta(N), eta_dot(N);

    if (use_velocity_filter_) {
        std::vector<double> t_all(N);
        for (size_t k = 0; k < N; ++k) t_all[k] = demo[k].t;

        std::vector<Eigen::Vector3d> r = unwrapRotationVector(demo);
        r = filter_utils::movingAverageSmooth(r, t_all, filter_window_sec_1_);

        for (size_t k = 0; k < N; ++k) {
            size_t km1 = (k == 0) ? 0 : k - 1;
            size_t kp1 = (k == N - 1) ? N - 1 : k + 1;
            double dt = t_all[kp1] - t_all[km1];
            if (dt <= 0.0) dt = 1e-6;
            eta[k] = tau_ * (r[kp1] - r[km1]) / dt;
        }
        eta = filter_utils::movingAverageSmooth(eta, t_all, filter_window_sec_2_);

        for (size_t k = 0; k < N; ++k) {
            size_t km1 = (k == 0) ? 0 : k - 1;
            size_t kp1 = (k == N - 1) ? N - 1 : k + 1;
            double dt = t_all[kp1] - t_all[km1];
            if (dt <= 0.0) dt = 1e-6;
            eta_dot[k] = (eta[kp1] - eta[km1]) / dt;
        }
    } else {
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
        for (size_t k = 0; k < N; ++k) {
            size_t km1 = (k == 0) ? 0 : k - 1;
            size_t kp1 = (k == N - 1) ? N - 1 : k + 1;
            double dt = demo[kp1].t - demo[km1].t;
            if (dt <= 0.0) dt = 1e-6;
            eta_dot[k] = (eta[kp1] - eta[km1]) / dt;
        }
    }

    eta0_ = eta.front();
    diag_.initial_eta_norm = eta.front().norm();
    diag_.final_eta_norm = eta.back().norm();

    // 3. Compute target angular forcing function:
    //    f_target = tau * deta/dt - alpha_z * (2 * beta_z * logMap(goal * q^-1) - eta)
    std::vector<Eigen::Vector3d> f_target(N);
    for (size_t k = 0; k < N; ++k) {
        Eigen::Quaterniond qk = demo[k].orientation.normalized();
        Eigen::Vector3d log_err = logMap(goal_ * qk.conjugate());
        for (int d = 0; d < 3; ++d) {
            f_target[k](d) = tau_ * eta_dot[k](d) - alpha_z_ * (2.0 * beta_z_ * log_err(d) - eta[k](d));
        }
    }

    // 4. Fit 3D angular weights w_i
    if (use_ridge_regression_) {
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

Eigen::Quaterniond QuaternionDMP::step(double dt) {
    // 1. Decay canonical phase clock
    double dx = -alpha_x_ / tau_ * x_;
    x_ += dx * dt;
    if (x_ < 0.0) x_ = 0.0;

    // 2. Evaluate Gaussian basis functions
    Eigen::VectorXd psi(n_basis_);
    double psi_sum = 0.0;
    for (int i = 0; i < n_basis_; ++i) {
        psi(i) = basisFunction(i, x_);
        psi_sum += psi(i);
    }
    if (psi_sum < 1e-8) psi_sum = 1e-8;

    // 3. Compute angular forcing term f(x)
    Eigen::Vector3d f = Eigen::Vector3d::Zero();
    for (int d = 0; d < 3; ++d) {
        double weighted = 0.0;
        for (int i = 0; i < n_basis_; ++i) weighted += weights_[d](i) * psi(i);
        f(d) = (weighted / psi_sum) * x_;
    }

    // 4. Transformation system on Lie algebra: tau * deta/dt = alpha_z * (2 * beta_z * log(goal * q^-1) - eta) + f
    Eigen::Vector3d log_err = logMap(goal_ * q_.conjugate());
    Eigen::Vector3d eta_dot = (alpha_z_ * (2.0 * beta_z_ * log_err - eta_) + f) / tau_;
    eta_ += eta_dot * dt;

    // 5. Exponential map geodesic integration on S^3:
    //    dq = expMap( (dt / (2 * tau)) * eta )
    //    q(t + dt) = (dq * q(t)).normalized()
    Eigen::Quaterniond dq = expMap((dt / (2.0 * tau_)) * eta_);
    q_ = dq * q_;
    q_.normalize();

    return q_;
}

}  // namespace core
}  // namespace haptic_dmp_learning