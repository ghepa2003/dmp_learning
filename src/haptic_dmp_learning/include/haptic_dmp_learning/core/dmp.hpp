#pragma once

#include <array>
#include <vector>
#include <Eigen/Dense>
#include "haptic_dmp_learning/core/types.hpp"

namespace haptic_dmp_learning {
namespace core {

// Standard discrete Dynamic Movement Primitive (Ijspeert et al.), covering
// all 3 Cartesian dimensions together: they share the same canonical system
// (same phase x, same tau), only the forcing-term weights differ per dimension.

class DMP {
public:
    // n_basis: number of Gaussian basis functions per dimension
    // alpha_x: canonical system decay rate (default 4.6 => x(T)/x(0) ~ 1%)
    // alpha_z, beta_z: transformation system gains
    //   (critically damped when beta_z = alpha_z / 4)
    // second_order_canonical: if true, the canonical system is second-order
    explicit DMP(int n_basis = 20, double alpha_x = 4.6, double alpha_z = 25.0, double beta_z = 6.25,
             bool second_order_canonical = false);


    // Fits the weights from a single recorded demonstration via locally
    // weighted regression (LWR). 
    //
    // Sets internally: tau (= demo duration),
    // y0 (first sample), goal (last sample).
    void learnFromDemonstration(const std::vector<Sample>& demo);

    // Resets the internal integration state (x=1, y=y0, z=0) to
    // replay/execute the learned DMP from the start via step().
    void reset();

    // Optionally override the goal before execution (spatial generalization).
    void setGoal(const Eigen::Vector3d& goal);

    // Returns the current goal (last sample of the demonstration, or manually set).
    const Eigen::Vector3d& goal() const { return goal_; }

    // Manually set the scale factor for one dimension (bypass of the automatic computation)
    // To be used when isScaleReliable() comes back false
    void setScale(int dim, double s) { scale_(dim) = s; }

    // Returns the current scale factor for each dimension (computed from the original movement amplitude and the current goal).
    const Eigen::Vector3d& scale() const { return scale_; }

    // False if, for that dimension, the goal based rescaling risks to be numerically
    // unstable (high movement amplitude wrt the original g-y0)
    bool isScaleReliable(int dim) const { return scale_reliable_[dim]; }

    // Optionally enable ridge regression (L2 regularization) for the weight fitting.
    void setRidgeRegression(bool enabled, double lambda = 1e-6) {
        use_ridge_regression_ = enabled;
        ridge_lambda_ = lambda;
    }

    // Returns true if ridge regression is enabled for the weight fitting, false otherwise.
    bool ridgeRegressionEnabled() const { return use_ridge_regression_; }
    double ridgeLambda() const { return ridge_lambda_; }

    // Optionally enable a velocity filter on the input demonstration before fitting the weights.
    void setVelocityFilter(bool enabled, double window_sec_1 = 0.05, double window_sec_2 = 0.05) {
        use_velocity_filter_ = enabled;
        filter_window_sec_1_ = window_sec_1;
        filter_window_sec_2_ = window_sec_2;
    }

    // Returns true if a velocity filter is enabled for the input demonstration, false otherwise.
    bool velocityFilterEnabled() const { return use_velocity_filter_; }
    double filterWindowSec1() const { return filter_window_sec_1_; }
    double filterWindowSec2() const { return filter_window_sec_2_; }

    // Set whether to use a second-order canonical system (recomputes basis functions).
    void setSecondOrderCanonical(bool enabled) {
        second_order_canonical_ = enabled;
        initBasisFunctions();
    }

    // Returns true if the DMP is using a second-order canonical system, false otherwise.
    bool secondOrderCanonical() const { return second_order_canonical_; }

    // Integrates one step of duration dt (seconds). Returns the new position.
    // Call reset() once before the first call.
    Eigen::Vector3d step(double dt, const Eigen::Vector3d& ct = Eigen::Vector3d::Zero(), double cc = 0.0);

    // Returns the current phase (x) of the canonical system, in [0,1].
    double phase() const { return x_; }

    // Returns true if the DMP has been learned from a demonstration (weights have been fitted).
    bool isLearned() const { return learned_; }

    // --- accessors needed for serialization (see dmp_io.hpp) ---
    int nBasis() const { return n_basis_; }
    double alphaX() const { return alpha_x_; }
    double alphaZ() const { return alpha_z_; }
    double betaZ() const { return beta_z_; }
    double tau() const { return tau_; }
    const Eigen::Vector3d& z0() const { return z0_; }
    const Eigen::Vector3d& y0() const { return y0_; }
    // dG()[d] is the original movement amplitude (goal - y0) for dimension d, captured during the fit
    const Eigen::Vector3d& dG() const { return dG_; }
    // A()[d] is the observed amplitude of the forcing term for dimension d, captured during the fit
    const Eigen::Vector3d& A() const { return A_; }
    const Eigen::VectorXd& centers() const { return centers_; }
    const Eigen::VectorXd& widths() const { return widths_; }
    // scaleReliable()[d] is true if the automatic rescaling of the forcing term is considered reliable for dimension d
    const std::array<bool, 3>& scaleReliable() const { return scale_reliable_; }
    // weights()[d] is the weight vector (size n_basis) for dimension d (0=x,1=y,2=z)
    const std::array<Eigen::VectorXd, 3>& weights() const { return weights_; }

    // Reconstructs a DMP from previously saved parameters (e.g. loaded from
    // YAML). Used by dmp_io when loading.
    void setLearnedParameters(double tau, const Eigen::Vector3d& y0, const Eigen::Vector3d& goal,
                              const Eigen::Vector3d& dG, const Eigen::Vector3d& A,
                              const Eigen::VectorXd& centers, const Eigen::VectorXd& widths,
                              const std::array<Eigen::VectorXd, 3>& weights, const Eigen::Vector3d& z0);
private:

    // Initializes the Gaussian basis functions (centers and widths) based on the number of basis functions and the canonical system dynamics.
    void initBasisFunctions();

    // Computes the value of the i-th Gaussian basis function at phase x.
    double basisFunction(int i, double x) const;

    // Filter the input demonstration to reduce noise in the velocity and acceleration estimates.
    std::vector<Eigen::Vector3d> movingAverageSmooth(const std::vector<Eigen::Vector3d>& signal,
                                                       const std::vector<double>& t, double window_sec) const;

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

    Eigen::VectorXd centers_;  // c_i, in phase space (0,1]
    Eigen::VectorXd widths_;   // h_i
    std::array<Eigen::VectorXd, 3> weights_;  // w_i per dimension

    // integration state (valid after reset())
    double x_;
    double v_;
    Eigen::Vector3d y_;
    Eigen::Vector3d z_;
    Eigen::Vector3d z0_;

    bool learned_;

    Eigen::Vector3d dG_;               // (goal - y0) original, captured during the fit
    Eigen::Vector3d A_;                // observed amplitude of the forcing term, captured during the fit
    Eigen::Vector3d scale_;            // scale factor applied in step(), default 1
    std::array<bool, 3> scale_reliable_;  // false if automatic rescaling has been blocked for that dimension
};

}  // namespace core
}  // namespace haptic_dmp_learning
