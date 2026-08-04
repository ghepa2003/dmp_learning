#pragma once
#include <array>
#include <vector>
#include <Eigen/Geometry>
#include "haptic_dmp_learning/core/types.hpp"

namespace haptic_dmp_learning {
namespace core {

// Quaternion DMP (Ude et al. 2014): state (q, eta) instead of (y, z), error 
// via logarithmic/exponential map instead of Euclidean subtraction. 
// Shares the same canonical system (same tau, same alpha_x) as the positional DMP learned 
// from the same demo - see tau() and the consistency check in the wrapper.

class QuaternionDMP {
public:
    explicit QuaternionDMP(int n_basis = 20, double alpha_x = 4.6, double alpha_z = 25.0, double beta_z = 6.25);

    void learnFromDemonstration(const std::vector<Sample>& demo);
    void reset();
    void setGoal(const Eigen::Quaterniond& goal);
    const Eigen::Quaterniond& goal() const { return goal_; }
    const Eigen::Vector3d& eta0() const { return eta0_; }

    // Optionally enable ridge regression (L2 regularization) for the weight fitting.
    void setRidgeRegression(bool enabled, double lambda = 1e-6) {
        use_ridge_regression_ = enabled;
        ridge_lambda_ = lambda;
    }
    bool ridgeRegressionEnabled() const { return use_ridge_regression_; }

    // Optionally enable a velocity filter on the input demonstration before fitting the weights.
    void setVelocityFilter(bool enabled, double window_sec_1 = 0.05, double window_sec_2 = 0.05) {
        use_velocity_filter_ = enabled;
        filter_window_sec_1_ = window_sec_1;
        filter_window_sec_2_ = window_sec_2;
    }
    bool velocityFilterEnabled() const { return use_velocity_filter_; }

    // Step integration; it returns the normalized current orientation
    Eigen::Quaterniond step(double dt);

    bool isLearned() const { return learned_; }


    double tau() const { return tau_; }  // for controlling the coherence with the positional DMP

    int nBasis() const { return n_basis_; }
    double alphaX() const { return alpha_x_; }
    double alphaZ() const { return alpha_z_; }
    double betaZ() const { return beta_z_; }
    const Eigen::Quaterniond& q0() const { return q0_; }
    const Eigen::VectorXd& centers() const { return centers_; }
    const Eigen::VectorXd& widths() const { return widths_; }
    const std::array<Eigen::VectorXd, 3>& weights() const { return weights_; }

    void setLearnedParameters(double tau, const Eigen::Quaterniond& q0, const Eigen::Quaterniond& goal,
                               const Eigen::VectorXd& centers, const Eigen::VectorXd& widths,
                               const std::array<Eigen::VectorXd, 3>& weights, const Eigen::Vector3d& eta0);

    static Eigen::Vector3d logMap(const Eigen::Quaterniond& q);
    static Eigen::Quaterniond expMap(const Eigen::Vector3d& r);

private:
    void initBasisFunctions();
    double basisFunction(int i, double x) const;
    // Costruisce una traiettoria CONTINUA in R^3 integrando cumulativamente
    // gli incrementi di rotazione locali tra campioni consecutivi -- a
    // differenza di logMap(q(t)*q0^-1), resta valida anche se l'asse di
    // rotazione cambia nel tempo (percorso non vincolato a un'unica
    // geodetica), perche' ogni incremento e' piccolo per costruzione.
    std::vector<Eigen::Vector3d> unwrapRotationVector(const std::vector<Sample>& demo) const;
    std::vector<Eigen::Vector3d> movingAverageSmooth(const std::vector<Eigen::Vector3d>& signal,
                                                       const std::vector<double>& t, double window_sec) const;

    int n_basis_;
    double alpha_x_, alpha_z_, beta_z_;
    double tau_;
    Eigen::Quaterniond q0_, goal_;
    Eigen::VectorXd centers_, widths_;
    std::array<Eigen::VectorXd, 3> weights_;

    double x_;
    Eigen::Quaterniond q_;
    Eigen::Vector3d eta_;
    Eigen::Vector3d eta0_;

    bool learned_;
    bool use_ridge_regression_ = false;
    double ridge_lambda_ = 1e-6;
    bool use_velocity_filter_ = false;
    double filter_window_sec_1_ = 0.05;
    double filter_window_sec_2_ = 0.05;
};

}  // namespace core
}  // namespace haptic_dmp_learning