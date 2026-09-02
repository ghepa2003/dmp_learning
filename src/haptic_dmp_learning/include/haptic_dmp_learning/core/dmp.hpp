#pragma once

#include <array>
#include <vector>
#include <Eigen/Dense>
#include "haptic_dmp_learning/core/types.hpp"

namespace haptic_dmp_learning {
namespace core {

/**
 * @brief Discrete Dynamic Movement Primitive (DMP) for 3D Cartesian Position Trajectories.
 *
 * @details
 * Mathematical Theory & Formulation (Ijspeert et al. / Schaal):
 * A DMP models a goal-directed point-to-point motion as a set of non-linear differential equations:
 *
 * 1. Canonical System (Phase Clock):
 *    Substitutes explicit time dependency with a phase variable x in (0, 1] that monotonically decays:
 *      tau * dx/dt = -alpha_x * x,   with initial condition x(0) = 1.0
 *    where tau is the movement duration (seconds) and alpha_x is the decay rate.
 *    Alternatively, a critically damped second-order canonical system can be used:
 *      tau * dv/dt = alpha_z * (beta_z * (0 - x) - v)
 *      tau * dx/dt = v
 *
 * 2. Transformation System (Damped Spring-Attractor with Forcing Function):
 *    For each Cartesian dimension d in {x, y, z}:
 *      tau * dz_d/dt = alpha_z * (beta_z * (goal_d - y_d) - z_d) + f_d(x) + C_t,d
 *      tau * dy_d/dt = z_d + C_c,d
 *    where:
 *      y_d is the current position coordinate.
 *      z_d is the scaled velocity coordinate (z = tau * dy/dt).
 *      alpha_z, beta_z are stiffness and damping gains (critically damped when beta_z = alpha_z / 4).
 *      goal_d is the final attractor position.
 *      C_t, C_c are optional external coupling terms (e.g. haptic feedback or obstacle avoidance).
 *
 * 3. Non-Linear Forcing Term (Shape Modulation):
 *    f_d(x) modulates the baseline spring dynamics to reproduce arbitrary demonstrated trajectory shapes:
 *      f_d(x) = ( sum_{i=1}^M psi_i(x) * w_{d,i} / sum_{i=1}^M psi_i(x) ) * x * s_d
 *    where:
 *      psi_i(x) = exp(-h_i * (x - c_i)^2) are normalized Gaussian Radial Basis Functions (RBF).
 *      c_i are kernel centers distributed across the phase space.
 *      h_i are kernel bandwidths/widths ensuring smooth Gaussian overlap.
 *      w_{d,i} are learnable shape weights fitted from demonstration.
 *      s_d is the spatial scaling factor for goal adaptation: s_d = (goal_new,d - y0_d) / (goal_orig,d - y0_d).
 *
 * 4. Learning via Regression:
 *    Given demonstrated positions y_demo(t), velocities dy_demo(t), and accelerations d^2y_demo(t):
 *    The target forcing signal is computed by inverting the transformation system:
 *      f_target,d(t) = tau^2 * d^2y_demo,d(t) - alpha_z * (beta_z * (goal_d - y_demo,d(t)) - tau * dy_demo,d(t))
 *    Weights are fitted via either:
 *    - Locally Weighted Regression (LWR): Closed-form weighted least squares per kernel.
 *    - Ridge Regression: Global linear regression with L2 regularization penalty (lambda * ||w||^2).
 */
class DMP {
public:
    /**
     * @brief Constructs a 3D Cartesian DMP.
     * @param n_basis Number of Gaussian basis functions per dimension (default: 20).
     * @param alpha_x Canonical decay rate (default: 4.6 => x(tau) ~ 0.01).
     * @param alpha_z Transformation system stiffness gain (default: 25.0).
     * @param beta_z Transformation system damping gain (default: 6.25, critical damping).
     * @param second_order_canonical Whether to use a 2nd-order canonical system.
     */
    explicit DMP(int n_basis = 20, double alpha_x = 4.6, double alpha_z = 25.0, double beta_z = 6.25,
                 bool second_order_canonical = false);

    /**
     * @brief Fits DMP weights from a demonstrated sequence of time-stamped pose samples.
     * Extracts tau = t_end - t_0, y0 = demo[0].position, goal = demo[last].position.
     * @param demo Vector of recorded Sample objects.
     */
    void learnFromDemonstration(const std::vector<Sample>& demo);

    /**
     * @brief Resets internal rollout state to start conditions (x=1, y=y0, z=z0, v=0).
     */
    void reset();

    /**
     * @brief Overrides the target goal position (spatial generalization).
     * Automatically verifies scaling stability and updates scale factors s_d.
     * @param goal New 3D target attractor position.
     */
    void setGoal(const Eigen::Vector3d& goal);

    /// @brief Returns the active goal position.
    const Eigen::Vector3d& goal() const { return goal_; }

    /// @brief Manually overrides the spatial scaling factor for a given dimension.
    void setScale(int dim, double s) { scale_(dim) = s; }

    /// @brief Returns the 3D spatial scaling vector [s_x, s_y, s_z].
    const Eigen::Vector3d& scale() const { return scale_; }

    /// @brief Returns true if automatic goal-based spatial rescaling is numerically well-conditioned.
    bool isScaleReliable(int dim) const { return scale_reliable_[dim]; }

    /// @brief Enables or disables global Ridge Regression (L2 regularization) during weight learning.
    void setRidgeRegression(bool enabled, double lambda = 1e-6) {
        use_ridge_regression_ = enabled;
        ridge_lambda_ = lambda;
    }

    bool ridgeRegressionEnabled() const { return use_ridge_regression_; }
    double ridgeLambda() const { return ridge_lambda_; }

    /// @brief Enables pre-filtering of demonstration position and velocity before numerical differentiation.
    void setVelocityFilter(bool enabled, double window_sec_1 = 0.05, double window_sec_2 = 0.05) {
        use_velocity_filter_ = enabled;
        filter_window_sec_1_ = window_sec_1;
        filter_window_sec_2_ = window_sec_2;
    }

    bool velocityFilterEnabled() const { return use_velocity_filter_; }
    double filterWindowSec1() const { return filter_window_sec_1_; }
    double filterWindowSec2() const { return filter_window_sec_2_; }

    /// @brief Configures 2nd-order canonical system dynamics and recomputes basis kernel centers.
    void setSecondOrderCanonical(bool enabled) {
        second_order_canonical_ = enabled;
        initBasisFunctions();
    }

    bool secondOrderCanonical() const { return second_order_canonical_; }

    /**
     * @brief Integrates the DMP forward by time step dt (Euler integration).
     * @param dt Integration time step in seconds (e.g. 0.001 s for 1 kHz loop).
     * @param ct Optional task-space coupling term added to transformation acceleration.
     * @param cc Optional canonical coupling term added to phase rate.
     * @return Eigen::Vector3d Integrated 3D position y(t + dt).
     */
    Eigen::Vector3d step(double dt, const Eigen::Vector3d& ct = Eigen::Vector3d::Zero(), double cc = 0.0);

    /// @brief Returns the current integrated 3D position y.
    const Eigen::Vector3d& position() const { return y_; }

    /// @brief Returns the current phase state x in [0, 1].
    double phase() const { return x_; }

    /// @brief Returns true if weights have been successfully learned or loaded.
    bool isLearned() const { return learned_; }

    // --- Accessors for Serialization & I/O ---
    int nBasis() const { return n_basis_; }
    double alphaX() const { return alpha_x_; }
    double alphaZ() const { return alpha_z_; }
    double betaZ() const { return beta_z_; }
    double tau() const { return tau_; }
    const Eigen::Vector3d& z0() const { return z0_; }
    const Eigen::Vector3d& y0() const { return y0_; }
    const Eigen::Vector3d& dG() const { return dG_; }
    const Eigen::Vector3d& A() const { return A_; }
    const Eigen::VectorXd& centers() const { return centers_; }
    const Eigen::VectorXd& widths() const { return widths_; }
    const std::array<bool, 3>& scaleReliable() const { return scale_reliable_; }
    const std::array<Eigen::VectorXd, 3>& weights() const { return weights_; }

    struct Diagnostics {
        double initial_vel_norm = 0.0;
        double final_vel_norm = 0.0;
        double initial_z_norm = 0.0;
        double final_z_norm = 0.0;
        int dropped_non_monotonic_samples = 0;  ///< samples removed for non-increasing timestamp
    };
    const Diagnostics& diagnostics() const { return diag_; }

    /**
     * @brief Directly sets pre-trained parameters (used by YAML loader).
     */
    void setLearnedParameters(double tau, const Eigen::Vector3d& y0, const Eigen::Vector3d& goal,
                              const Eigen::Vector3d& dG, const Eigen::Vector3d& A,
                              const Eigen::VectorXd& centers, const Eigen::VectorXd& widths,
                              const std::array<Eigen::VectorXd, 3>& weights, const Eigen::Vector3d& z0);

private:
    void initBasisFunctions();
    double basisFunction(int i, double x) const;

    int n_basis_;
    double alpha_x_;
    double alpha_z_;
    double beta_z_;
    bool second_order_canonical_;
    bool use_ridge_regression_ = false;
    double ridge_lambda_ = 1e-6;
    bool use_velocity_filter_ = false;
    double filter_window_sec_1_ = 0.05;
    double filter_window_sec_2_ = 0.05;

    double tau_;
    Eigen::Vector3d y0_;
    Eigen::Vector3d goal_;

    Eigen::VectorXd centers_;                ///< Kernel centers c_i in phase space (0, 1]
    Eigen::VectorXd widths_;                 ///< Kernel bandwidths h_i
    std::array<Eigen::VectorXd, 3> weights_; ///< Fitted weights per dimension (x, y, z)

    // Integration state
    double x_;
    double v_;
    Eigen::Vector3d y_;
    Eigen::Vector3d z_;
    Eigen::Vector3d z0_;

    bool learned_;

    Eigen::Vector3d dG_;                     ///< Original demonstrated displacement (goal - y0)
    Eigen::Vector3d A_;                      ///< Observed bounding amplitude of demonstration
    Eigen::Vector3d scale_;                  ///< Goal adaptation scaling factors
    std::array<bool, 3> scale_reliable_;
    Diagnostics diag_;
};

}  // namespace core
}  // namespace haptic_dmp_learning
