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
    explicit DMP(int n_basis = 20, double alpha_x = 4.6, double alpha_z = 25.0, double beta_z = 6.25);

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
    const Eigen::Vector3d& goal() const { return goal_; }

    // Integrates one step of duration dt (seconds). Returns the new position.
    // Call reset() once before the first call.
    Eigen::Vector3d step(double dt);

    double phase() const { return x_; }
    bool isLearned() const { return learned_; }

    // --- accessors needed for serialization (see dmp_io.hpp) ---
    int nBasis() const { return n_basis_; }
    double alphaX() const { return alpha_x_; }
    double alphaZ() const { return alpha_z_; }
    double betaZ() const { return beta_z_; }
    double tau() const { return tau_; }
    const Eigen::Vector3d& y0() const { return y0_; }
    const Eigen::VectorXd& centers() const { return centers_; }
    const Eigen::VectorXd& widths() const { return widths_; }
    // weights()[d] is the weight vector (size n_basis) for dimension d (0=x,1=y,2=z)
    const std::array<Eigen::VectorXd, 3>& weights() const { return weights_; }

    // Reconstructs a DMP from previously saved parameters (e.g. loaded from
    // YAML). Used by dmp_io when loading.
    void setLearnedParameters(double tau, const Eigen::Vector3d& y0, const Eigen::Vector3d& goal,
                               const Eigen::VectorXd& centers, const Eigen::VectorXd& widths,
                               const std::array<Eigen::VectorXd, 3>& weights);

private:
    void initBasisFunctions();
    double basisFunction(int i, double x) const;

    int n_basis_;
    double alpha_x_;
    double alpha_z_;
    double beta_z_;

    double tau_;
    Eigen::Vector3d y0_;
    Eigen::Vector3d goal_;

    Eigen::VectorXd centers_;  // c_i, in phase space (0,1]
    Eigen::VectorXd widths_;   // h_i
    std::array<Eigen::VectorXd, 3> weights_;  // w_i per dimension

    // integration state (valid after reset())
    double x_;
    Eigen::Vector3d y_;
    Eigen::Vector3d z_;

    bool learned_;
};

}  // namespace core
}  // namespace haptic_dmp_learning
