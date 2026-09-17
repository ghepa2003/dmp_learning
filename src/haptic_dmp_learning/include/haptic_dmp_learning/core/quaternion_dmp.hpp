#pragma once

#include <array>
#include <vector>
#include <Eigen/Geometry>
#include "haptic_dmp_learning/core/types.hpp"

namespace haptic_dmp_learning {
namespace core {

/**
 * @brief Discrete Dynamic Movement Primitive for 3D Orientation Trajectories in SO(3) (Ude et al. 2014).
 *
 * @details
 * Mathematical Theory & Riemannian Geometry on Unit Quaternions:
 * Unlike 3D positions in Euclidean space R^3, orientations live on the non-Euclidean Riemannian manifold SO(3),
 * represented by unit quaternions S^3 = { q in H : ||q|| = 1 }.
 * Standard vector subtraction (g - y) cannot be applied to quaternions.
 *
 * 1. Logarithmic & Exponential Maps (Lie Group SO(3) <-> Lie Algebra so(3) ~ R^3):
 *    - Log Map: Extracts rotation vector r = (theta / 2) * u from quaternion q:
 *        logMap(q) = (theta / 2) * u
 *      Orientation error from q to attractor goal g:
 *        e_o = 2 * logMap(g * q^-1)
 *    - Exp Map: Converts rotation vector r into incremental unit quaternion:
 *        expMap(r) = [cos(||r||), sin(||r||) * (r / ||r||)]
 *
 * 2. Quaternion Transformation System:
 *    Let eta in R^3 be the scaled angular velocity vector (eta = tau * omega):
 *      tau * deta/dt = alpha_z * (2 * beta_z * logMap(goal * q^-1) - eta) + f(x)
 *    Manifold integration step on S^3:
 *      dq = expMap( (dt / (2 * tau)) * eta )
 *      q(t + dt) = (dq * q(t)).normalized()
 *
 * 3. Non-Linear Forcing Function f(x):
 *    Modulates angular trajectory shape using Gaussian kernels across phase x in (0, 1]:
 *      f(x) = ( sum_{i=1}^M psi_i(x) * w_i / sum_{i=1}^M psi_i(x) ) * x
 *    where w_i in R^3 are 3-dimensional learned angular shape weights.
 */
class QuaternionDMP {
public:
    /**
     * @brief Constructs a Quaternion DMP.
     * @param n_basis Number of Gaussian basis functions (default: 20).
     * @param alpha_x Canonical decay rate (default: 4.6).
     * @param alpha_z Transformation stiffness gain (default: 25.0).
     * @param beta_z Transformation damping gain (default: 6.25, critical damping).
     */
    explicit QuaternionDMP(int n_basis = 20, double alpha_x = 4.6, double alpha_z = 25.0, double beta_z = 6.25);

    /**
     * @brief Fits orientation weights from recorded demonstration samples.
     * @param demo Vector of recorded Sample objects containing orientations.
     */
    void learnFromDemonstration(const std::vector<Sample>& demo);

    /**
     * @brief Resets internal rollout state to initial conditions (x=1, q=q0, eta=eta0).
     */
    void reset();

    /**
     * @brief Overrides the target orientation attractor goal in SO(3).
     * @param goal Desired final unit quaternion.
     */
    void setGoal(const Eigen::Quaterniond& goal);

    /// @brief Returns the active target goal quaternion.
    const Eigen::Quaterniond& goal() const { return goal_; }

    /// @brief Returns the initial angular velocity offset eta0.
    const Eigen::Vector3d& eta0() const { return eta0_; }

    /// @brief Enables or disables global Ridge Regression (L2 regularization).
    void setRidgeRegression(bool enabled, double lambda = 1e-6) {
        use_ridge_regression_ = enabled;
        ridge_lambda_ = lambda;
    }

    bool ridgeRegressionEnabled() const { return use_ridge_regression_; }
    double ridgeLambda() const { return ridge_lambda_; }

    /// @brief Enables pre-filtering of demonstration rotation trajectory.
    void setVelocityFilter(bool enabled, double window_sec_1 = 0.05, double window_sec_2 = 0.05) {
        use_velocity_filter_ = enabled;
        filter_window_sec_1_ = window_sec_1;
        filter_window_sec_2_ = window_sec_2;
    }

    bool velocityFilterEnabled() const { return use_velocity_filter_; }
    double filterWindowSec1() const { return filter_window_sec_1_; }
    double filterWindowSec2() const { return filter_window_sec_2_; }

    /**
     * @brief Integrates the Quaternion DMP forward by time step dt on S^3.
     * @param dt Integration time step in seconds.
     * @return Eigen::Quaterniond Updated unit quaternion orientation q(t + dt).
     */
    Eigen::Quaterniond step(double dt);

    /// @brief Returns the current integrated unit quaternion orientation q.
    const Eigen::Quaterniond& orientation() const { return q_; }

    /**
     * @brief Returns the current instantaneous angular velocity omega = eta / tau,
     *        real time scale, expressed in the same fixed (base/world) frame as
     *        orientation()/goal() - eta_ is scaled angular velocity (eta = tau *
     *        omega), already integrated as native state on every step() call
     *        (core/quaternion_dmp.cpp); this is a plain unscaling, not a
     *        numerical derivative.
     */
    Eigen::Vector3d omega() const { return eta_ / tau_; }

    /// @brief Returns true if weights have been successfully learned or loaded.
    bool isLearned() const { return learned_; }

    /// @brief Returns the demonstration duration tau.
    double tau() const { return tau_; } 

    // --- Accessors for Serialization & I/O ---
    int nBasis() const { return n_basis_; }
    double alphaX() const { return alpha_x_; }
    double alphaZ() const { return alpha_z_; }
    double betaZ() const { return beta_z_; }
    const Eigen::Quaterniond& q0() const { return q0_; }
    const Eigen::VectorXd& centers() const { return centers_; }
    const Eigen::VectorXd& widths() const { return widths_; }
    const std::array<Eigen::VectorXd, 3>& weights() const { return weights_; }

    struct Diagnostics {
        double initial_eta_norm = 0.0;
        double final_eta_norm = 0.0;
        int dropped_non_monotonic_samples = 0;  ///< samples removed for non-increasing timestamp
        int quat_sign_flips_corrected = 0;      ///< quaternion double-cover sign transitions corrected
    };
    const Diagnostics& diagnostics() const { return diag_; }

    /**
     * @brief Directly sets pre-trained parameters (used by YAML loader).
     */
    void setLearnedParameters(double tau, const Eigen::Quaterniond& q0, const Eigen::Quaterniond& goal,
                               const Eigen::VectorXd& centers, const Eigen::VectorXd& widths,
                               const std::array<Eigen::VectorXd, 3>& weights, const Eigen::Vector3d& eta0);

    /**
     * @brief Logarithmic map from unit quaternion to Lie algebra so(3) rotation vector.
     */
    static Eigen::Vector3d logMap(const Eigen::Quaterniond& q);

    /**
     * @brief Exponential map from Lie algebra so(3) rotation vector to unit quaternion.
     */
    static Eigen::Quaterniond expMap(const Eigen::Vector3d& r);

private:
    void initBasisFunctions();
    double basisFunction(int i, double x) const;

    std::vector<Eigen::Vector3d> unwrapRotationVector(const std::vector<Sample>& demo) const;

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
    Diagnostics diag_;
};

}  // namespace core
}  // namespace haptic_dmp_learning