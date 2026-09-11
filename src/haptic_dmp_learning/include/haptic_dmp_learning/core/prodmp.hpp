#pragma once

#include <array>
#include <vector>
#include <Eigen/Dense>
#include "haptic_dmp_learning/core/types.hpp"

namespace haptic_dmp_learning {
namespace core {

/**
 * @brief Integral-form Probabilistic Dynamic Movement Primitive (ProDMP) for 3D
 *        Cartesian position trajectories - deterministic / "level 1" variant.
 *
 * @details
 * This is an independent, self-contained C++ port of a subset of the MP_PyTorch
 * ProDMP formulation (ALRhub; Li et al. 2023, "ProDMP: A Unified Perspective on
 * Dynamic and Probabilistic Movement Primitives"). It intentionally lives next
 * to the classic core::DMP (Ijspeert/Schaal) as a *second, fully functional*
 * implementation so the two formulations can be A/B compared on the same task.
 * core::DMP is not deprecated by this class and is never touched by it.
 *
 * ---------------------------------------------------------------------------
 * MATHEMATICAL DERIVATION (mirrors the reference Python, adapted to a single
 * real-time sequential rollout instead of batched autograd evaluation)
 * ---------------------------------------------------------------------------
 * The DMP transformation system uses the same critically damped gains as
 * core::DMP (beta_z = alpha_z / 4). Written in scaled time s = t / tau, and
 * noting that the forcing term f(s) depends only on the phase (never on y or
 * its derivative), the transformation system is a *linear, constant-coefficient*
 * second-order ODE in y:
 *
 *     y''(s) + alpha * y'(s) + (alpha^2 / 4) * y(s) = (alpha^2 / 4) * g + f(s)
 *
 * Because it is linear, the solution is obtained by SUPERPOSITION instead of
 * being integrated step-by-step as core::DMP::step() does today:
 *
 *  1. HOMOGENEOUS part (depends only on init_pos, init_vel at init_time):
 *       y_h(s) = c1 * y1(s) + c2 * y2(s)
 *       y1(s) = exp(-alpha/2 * s),   y2(s) = s * y1(s)      [double root -alpha/2]
 *     c1, c2 solve the 2x2 linear system imposing y(s0) = init_pos,
 *     y'(s0) = init_vel. The Wronskian W = y1*y2' - y2*y1' = y1^2 = exp(-alpha*s)
 *     is strictly positive, hence the system is never singular.
 *
 *  2. PARTICULAR part from the SHAPE term f(s) = sum_i w_i * x(s) * psi_i(x(s)),
 *     with x(s) the canonical phase and psi_i the SAME normalized RBFs used by
 *     core::DMP. By variation of parameters the response to each unit RBF weight
 *     integrates in closed form:
 *       p1_i(s) = integral_0^s [ y2(s') * x(s') * psi_i(s') / W(s') ] ds'
 *               = integral_0^s [ s' * exp(alpha/2 * s') * x(s') * psi_i(s') ] ds'
 *       p2_i(s) = integral_0^s [ y1(s') * x(s') * psi_i(s') / W(s') ] ds'
 *               = integral_0^s [      exp(alpha/2 * s') * x(s') * psi_i(s') ] ds'
 *       pos_basis_w_i(s) = y2(s) * p2_i(s) - y1(s) * p1_i(s)
 *
 *  3. PARTICULAR part from the GOAL, treated as a STEP input (not as a sum of
 *     RBFs) - its step response integrates analytically, no quadrature needed:
 *       q1(s) = (0.5*alpha*s - 1) * exp(0.5*alpha*s) + 1
 *       q2(s) = 0.5*alpha * (exp(0.5*alpha*s) - 1)
 *       pos_basis_g(s) = y2(s) * q2(s) - y1(s) * q1(s)
 *
 *  Result:
 *     y(s) = y_h(s) + sum_i w_i * pos_basis_w_i(s) + g * pos_basis_g(s)
 *
 *  This expression is LINEAR both in the shape weights w_i AND in the goal g:
 *  the goal is structurally "just one more weight" with its own closed-form
 *  basis (hence num_basis_g = num_basis + 1 in the reference code). For this
 *  reason, UNLIKE core::DMP::setGoal(), setGoal() here needs NO numerical
 *  guard (no kMinDG, no amplitude-ratio > 2.0 check): there is no spatial
 *  scaling factor s_d to protect, only a direct assignment of one linear
 *  parameter, which does not alter the shape weights w_i in any way.
 *
 *  Learning (learnFromDemonstration) therefore collapses to a SINGLE joint
 *  least-squares solve for (w_1..w_n, g), instead of core::DMP's dynamics
 *  inversion followed by a per-axis LWR/ridge fit.
 *
 * ---------------------------------------------------------------------------
 * INTEGRATION STRATEGY: incremental accumulation, not grid + interpolation
 * ---------------------------------------------------------------------------
 * The reference Python pre-computes p1/p2 on a fixed grid and interpolates,
 * which is required there to evaluate at arbitrary/batched times during
 * autograd training. Here the rollout is ALWAYS sequential and monotone
 * (t = 0, dt, 2*dt, ... never out of order, never repeated), so p1_i(s) and
 * p2_i(s) are built by INCREMENTAL trapezoidal quadrature, one tick at a time:
 *
 *     dp1_now = s * exp(alpha*s/2) * x(s) * psi_i(s)
 *     p1_accum_i += 0.5 * (dp1_prev_i + dp1_now) * (s - s_prev)
 *     dp1_prev_i  = dp1_now                       (analogous for dp2 / p2)
 *
 * Advantages: (a) no interpolation error - evaluated exactly at the real
 * sampling instants; (b) local error O(dt^2) of the trapezoid, more accurate
 * than the explicit Euler O(dt) used today in core::DMP::step(); (c) O(1) work
 * per tick instead of O(N) per query.
 *
 * STATE NOTE: p1_accum_ / p2_accum_ / dp1_prev_ / dp2_prev_ (one entry per RBF)
 * are PERSISTENT state between ticks, unlike the stateless Python version. They
 * are reset EXPLICITLY on every new rollout via resetIntegrationState(), which
 * is called both from the constructor and from setInitialConditions(), so it
 * cannot be forgotten in one path and not the other (same fail-safe principle
 * as core::DMP::reset()).
 */
class ProDMP {
public:
    /**
     * @brief Constructs a 3D Cartesian ProDMP (integral form).
     * @param num_basis Number of Gaussian RBF shape kernels (>= 2). The goal
     *                  contributes one additional closed-form basis, so the
     *                  linear parameter vector has num_basis + 1 entries per axis.
     * @param alpha  Transformation-system gain alpha_z (default: 25.0). Damping
     *               is critical: beta_z = alpha / 4, as in core::DMP.
     * @param alpha_x Canonical phase decay rate (default: 4.6 => x(1) ~ 0.01).
     * @param ridge_lambda Ridge term added to the normal-equations diagonal
     *                     during learnFromDemonstration() (default: 1e-9,
     *                     identical to the previous hardcoded constant, so
     *                     every existing call site keeps its exact behaviour).
     */
    explicit ProDMP(int num_basis, double alpha = 25.0, double alpha_x = 4.6,
                    double ridge_lambda = 1e-9);

    /**
     * @brief Fits shape weights and goal from a SINGLE demonstration (not batched).
     *
     * Extracts init_time / init_pos / init_vel from the first samples (finite
     * difference over the first interval, as learn_mp_params_from_trajs does in
     * the reference prodmp.py), then solves one joint linear least-squares
     * problem for (w_1..w_n, g) per Cartesian axis (Eigen LDLT on the normal
     * equations with a tiny ridge term - same solver family core::DMP uses for
     * its ridge fit). tau is taken as demo.back().t - demo.front().t.
     *
     * On return the object is ready to roll out: setInitialConditions() has been
     * called internally with the extracted initial conditions.
     *
     * @param demo Time-stamped Cartesian pose samples (>= 5, strictly increasing t).
     * @param tau_override If > 0, forces tau_ to this value instead of deriving
     *                     it from demo.back().t - demo.front().t. The regression
     *                     design matrix is then built at phases
     *                     s_k = (t_k - t0) / tau_override (so a training set that
     *                     covers only part of the movement is fitted at its TRUE
     *                     fraction of the full phase, not stretched to fill
     *                     [0, 1]). Default 0.0 = derive tau from the demo span,
     *                     which reproduces the previous behaviour bit-for-bit -
     *                     every existing caller is unaffected. Intended for
     *                     offline hold-out / extrapolation studies only.
     */
    void learnFromDemonstration(const std::vector<Sample>& demo, double tau_override = 0.0);

    /**
     * @brief Sets the initial conditions for a runtime rollout and rewinds the
     *        integrator (calls resetIntegrationState() internally).
     * @param init_time Absolute start time (stored for bookkeeping; the rollout
     *                  internally uses scaled time s starting at 0).
     * @param init_pos  Initial Cartesian position y(s0).
     * @param init_vel  Initial Cartesian velocity dy/dt(s0) (real time scale).
     */
    void setInitialConditions(double init_time, const Eigen::Vector3d& init_pos,
                              const Eigen::Vector3d& init_vel);

    /**
     * @brief Overrides ONLY the goal parameter (spatial generalization).
     *
     * No numerical guard is applied here (no kMinDG, no amplitude-ratio check):
     * in the integral form the goal is one linear parameter with its own
     * closed-form basis pos_basis_g(s); reassigning it rescales nothing and
     * leaves every shape weight w_i untouched. See the class-level derivation.
     *
     * @param new_goal Desired absolute 3D attractor position. If relativeGoal()
     *                 is true it is stored relative to the current init_pos.
     */
    void setGoal(const Eigen::Vector3d& new_goal);

    /// @brief True if the goal contribution is expressed relative to init_pos.
    bool relativeGoal() const { return relative_goal_; }

    /**
     * @brief Switches the goal representation between absolute and relative to
     *        init_pos, keeping the current absolute goal invariant.
     */
    void setRelativeGoal(bool relative);

    /**
     * @brief Advances the internal state by dt and returns the current position.
     *
     * Accumulates p1_i / p2_i for every RBF via the incremental trapezoid, then
     * evaluates the closed-form superposition. Same logical signature as
     * core::DMP::step(dt) for drop-in use by a future twin executor node.
     *
     * @param dt Time step in seconds (fixed-rate real-time loop).
     * @return Current 3D position y(t + dt).
     */
    Eigen::Vector3d step(double dt);

    /// @brief Movement duration tau (seconds).
    double tau() const { return tau_; }

    /// @brief Current absolute goal position.
    const Eigen::Vector3d& goal() const { return goal_; }

    /// @brief Current integrated position (equals init_pos before the first step).
    const Eigen::Vector3d& position() const { return pos_; }

    /// @brief Current integrated velocity, real time scale (equals init_vel before the first step).
    const Eigen::Vector3d& velocity() const { return vel_; }

    /// @brief True once weights + goal have been learned or loaded.
    bool isLearned() const { return learned_; }

    /**
     * @brief Sets a symmetric moving-average smoothing window (seconds) applied
     *        to the demonstration's POSITION trajectory before
     *        learnFromDemonstration() extracts init_pos/init_vel and builds the
     *        regression design matrix. Reuses filter_utils::movingAverageSmooth
     *        verbatim (same function core::DMP uses for its velocity filter).
     *
     * @param window_sec Window width in seconds. <= 0.0 (the default) disables
     *                   the filter entirely, reproducing today's behaviour
     *                   bit-for-bit - no smoothing is ever applied unless this
     *                   is called with a positive value.
     */
    void setPositionFilterWindow(double window_sec) { position_filter_window_sec_ = window_sec; }

    /// @brief Current position-filter window (seconds); <= 0 means disabled.
    double positionFilterWindow() const { return position_filter_window_sec_; }

    /// @brief Ridge term used on the normal-equations diagonal during learning.
    double ridgeLambda() const { return ridge_lambda_; }

    /**
     * @brief Opt-in: compute the 2-norm condition number of the column-scaled
     *        design matrix H during the next learnFromDemonstration() and store
     *        it in diagnostics().design_matrix_condition_number.
     *
     * Off by default because it needs a full JacobiSVD of an (N x P) matrix,
     * which is far more expensive than the fit itself. Intended for offline
     * conditioning studies only; has no effect on the fitted parameters.
     */
    void setComputeConditionNumber(bool enable) { compute_condition_number_ = enable; }
    bool computeConditionNumber() const { return compute_condition_number_; }

    /**
     * @brief Opt-in: during the next learnFromDemonstration(), do NOT estimate
     *        the goal parameter jointly with the shape weights. Instead fix it
     *        in closed form to the demonstration's final observed position
     *        (y_demo(tau)), subtract that known contribution from the regression
     *        target, and fit the shape weights on the residual only.
     *
     * Rationale: in the classic DMP (Ijspeert/Schaal; Li et al. 2023, Eq. 11)
     * the goal g is a KNOWN constant taken as the trajectory endpoint, and
     * convergence to it is guaranteed by the critically-damped transformation
     * dynamics. ProDMP's integral form instead puts g as one more unknown in
     * the SAME uniformly-weighted least-squares system as every shape weight,
     * so the endpoint accuracy is only whatever the averaged regression yields
     * (measured ~1.19 mm on trajC vs ~0.08 mm for the classic DMP). This flag
     * restores the classic behaviour for the goal term while keeping ProDMP's
     * linear shape parameterisation.
     *
     * Default false = unchanged joint estimation (byte-identical fit).
     * setGoal()/setRelativeGoal()/the retarget path are untouched: goal_param_
     * stays freely settable AFTER the fit; only how it is computed DURING the
     * fit changes.
     */
    void setFixGoalToDemoEndpoint(bool enable) { fix_goal_to_demo_endpoint_ = enable; }
    bool fixGoalToDemoEndpoint() const { return fix_goal_to_demo_endpoint_; }

    // --- Accessors for serialization & I/O ---
    int numBasis() const { return num_basis_; }
    double alpha() const { return alpha_; }
    double alphaX() const { return alpha_x_; }
    const Eigen::Vector3d& initPos() const { return init_pos_; }
    const Eigen::Vector3d& initVel() const { return init_vel_; }
    double initTime() const { return init_time_; }
    /// @brief The raw goal linear parameter (relative to init_pos when relativeGoal()).
    const Eigen::Vector3d& goalParam() const { return goal_param_; }
    const Eigen::VectorXd& centers() const { return centers_; }
    const Eigen::VectorXd& widths() const { return widths_; }
    const std::array<Eigen::VectorXd, 3>& weights() const { return weights_; }

    struct Diagnostics {
        double initial_vel_norm = 0.0;      ///< || init_vel || used at the last setInitialConditions
        double learn_residual_rms = 0.0;    ///< RMS of the least-squares residual over all axes/samples
        int dropped_non_monotonic_samples = 0;  ///< samples removed for a non-increasing timestamp
        /// 2-norm cond(H) of the COLUMN-SCALED design matrix at the last fit.
        /// 0.0 unless setComputeConditionNumber(true) was called beforehand.
        double design_matrix_condition_number = 0.0;
    };
    const Diagnostics& diagnostics() const { return diag_; }

    /**
     * @brief Directly injects pre-trained parameters (used by the YAML loader).
     *
     * Mirrors the calling style of core::DMP::setLearnedParameters() for
     * consistency, but this is a DISTINCT function on a DISTINCT class - the
     * classic DMP loader is untouched.
     *
     * @param tau          Movement duration.
     * @param init_pos     Stored initial position.
     * @param init_vel     Stored initial velocity (real time scale).
     * @param goal_param   Goal linear parameter (relative to init_pos iff relative_goal).
     * @param centers      RBF centers in phase space.
     * @param widths       RBF bandwidths.
     * @param weights      Per-axis shape weights (size num_basis each).
     * @param relative_goal Whether goal_param is relative to init_pos.
     */
    void setLearnedParameters(double tau, const Eigen::Vector3d& init_pos,
                              const Eigen::Vector3d& init_vel, const Eigen::Vector3d& goal_param,
                              const Eigen::VectorXd& centers, const Eigen::VectorXd& widths,
                              const std::array<Eigen::VectorXd, 3>& weights, bool relative_goal);

private:
    /// @brief Distributes RBF centers/widths in phase space (identical scheme to core::DMP).
    void initBasisFunctions();

    /// @brief Fills @p psi with the normalized RBF activations at canonical phase @p x.
    void evalBasis(double x, Eigen::VectorXd& psi) const;

    /// @brief Canonical phase x(s) = exp(-alpha_x * s).
    double canonicalPhase(double s) const;

    /// @brief Zeroes the persistent trapezoid accumulators and re-seeds them at s = 0.
    void resetIntegrationState();

    /// @brief Recomputes the cached absolute goal_ from goal_param_ / init_pos_.
    void refreshGoalCache();

    int num_basis_;
    double alpha_;      ///< Transformation-system gain alpha_z
    double alpha_x_;    ///< Canonical phase decay rate
    double ridge_lambda_;  ///< Ridge term for the learning normal equations

    double position_filter_window_sec_ = 0.0;  ///< <= 0 = disabled (default)
    bool compute_condition_number_ = false;     ///< opt-in SVD cond(H) diagnostic
    bool fix_goal_to_demo_endpoint_ = false;    ///< opt-in: goal = y_demo(tau), not regressed

    double tau_;
    bool relative_goal_;
    bool learned_;

    Eigen::VectorXd centers_;                ///< RBF centers c_i in phase space (0, 1]
    Eigen::VectorXd widths_;                 ///< RBF bandwidths h_i
    std::array<Eigen::VectorXd, 3> weights_; ///< Shape weights per axis (x, y, z), size num_basis_

    Eigen::Vector3d goal_param_;             ///< Goal linear parameter (one more "weight")
    Eigen::Vector3d goal_;                   ///< Cached absolute goal for goal()

    double init_time_;
    Eigen::Vector3d init_pos_;
    Eigen::Vector3d init_vel_;

    // Homogeneous-solution Wronskian at the rollout start (s = 0). Analytically
    // W(0) = y1(0)^2 = 1; recomputed and guarded in setInitialConditions().
    double wronskian_det_;

    // --- Persistent incremental-integration state (one entry per RBF) ---
    double s_;                    ///< Current scaled time s = (t - init_time) / tau
    Eigen::VectorXd p1_accum_;    ///< Accumulated integral p1_i(s)
    Eigen::VectorXd p2_accum_;    ///< Accumulated integral p2_i(s)
    Eigen::VectorXd dp1_prev_;    ///< Previous-tick integrand dp1_i (trapezoid left node)
    Eigen::VectorXd dp2_prev_;    ///< Previous-tick integrand dp2_i (trapezoid left node)

    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;

    Diagnostics diag_;
};

}  // namespace core
}  // namespace haptic_dmp_learning
