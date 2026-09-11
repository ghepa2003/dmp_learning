#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/filter_utils.hpp"
#include <Eigen/SVD>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace haptic_dmp_learning {
namespace core {

namespace {
// Below this the homogeneous 2x2 system is treated as singular. W(0) = 1
// analytically, so hitting this means alpha is corrupted or has underflowed.
constexpr double kMinWronskian = 1e-12;
}  // namespace

ProDMP::ProDMP(int num_basis, double alpha, double alpha_x, double ridge_lambda)
    : num_basis_(num_basis),
      alpha_(alpha),
      alpha_x_(alpha_x),
      ridge_lambda_(ridge_lambda),
      tau_(1.0),
      relative_goal_(false),
      learned_(false),
      goal_param_(Eigen::Vector3d::Zero()),
      goal_(Eigen::Vector3d::Zero()),
      init_time_(0.0),
      init_pos_(Eigen::Vector3d::Zero()),
      init_vel_(Eigen::Vector3d::Zero()),
      wronskian_det_(1.0),
      s_(0.0) {
    if (num_basis_ < 2) {
        throw std::invalid_argument("ProDMP: need at least 2 basis functions.");
    }
    for (auto& w : weights_) {
        w = Eigen::VectorXd::Zero(num_basis_);
    }
    p1_accum_ = Eigen::VectorXd::Zero(num_basis_);
    p2_accum_ = Eigen::VectorXd::Zero(num_basis_);
    dp1_prev_ = Eigen::VectorXd::Zero(num_basis_);
    dp2_prev_ = Eigen::VectorXd::Zero(num_basis_);
    initBasisFunctions();
    resetIntegrationState();
}

void ProDMP::initBasisFunctions() {
    // Identical layout to core::DMP (first-order canonical decay): centers
    // equispaced in normalized time then mapped through x(t) = exp(-alpha_x * t),
    // bandwidths set so neighbouring kernels overlap at ~55% of their peak.
    Eigen::VectorXd t_norm = Eigen::VectorXd::LinSpaced(num_basis_, 0.0, 1.0);
    centers_.resize(num_basis_);
    for (int i = 0; i < num_basis_; ++i) {
        centers_(i) = std::exp(-alpha_x_ * t_norm(i));
    }

    widths_ = Eigen::VectorXd::Zero(num_basis_);
    for (int i = 0; i < num_basis_; ++i) {
        if (i < num_basis_ - 1) {
            double d = (centers_(i + 1) - centers_(i)) * 0.55;
            widths_(i) = 1.0 / (d * d);
        } else {
            widths_(i) = widths_(i - 1);
        }
    }
}

double ProDMP::canonicalPhase(double s) const {
    return std::exp(-alpha_x_ * s);
}

void ProDMP::evalBasis(double x, Eigen::VectorXd& psi) const {
    // Normalized Gaussian RBFs: psi_i(x) = exp(-h_i (x - c_i)^2) / sum_j exp(...)
    double sum = 0.0;
    for (int i = 0; i < num_basis_; ++i) {
        double d = x - centers_(i);
        psi(i) = std::exp(-widths_(i) * d * d);
        sum += psi(i);
    }
    if (sum < 1e-8) sum = 1e-8;
    psi /= sum;
}

void ProDMP::refreshGoalCache() {
    goal_ = relative_goal_ ? (goal_param_ + init_pos_) : goal_param_;
}

void ProDMP::resetIntegrationState() {
    // Persistent trapezoid state is wiped on every new rollout so a stale
    // integral from a previous run cannot bleed into this one.
    s_ = 0.0;
    p1_accum_.setZero();
    p2_accum_.setZero();

    // Seed the trapezoid's left node with the integrand value at s = 0:
    //   dp1_i(0) = 0 * exp(0) * x(0) * psi_i  = 0
    //   dp2_i(0) =     exp(0) * x(0) * psi_i  = x(0) * psi_i
    const double x0 = canonicalPhase(0.0);
    Eigen::VectorXd psi0(num_basis_);
    evalBasis(x0, psi0);
    for (int i = 0; i < num_basis_; ++i) {
        dp1_prev_(i) = 0.0;
        dp2_prev_(i) = x0 * psi0(i);
    }

    pos_ = init_pos_;
    vel_ = init_vel_;
}

void ProDMP::setInitialConditions(double init_time, const Eigen::Vector3d& init_pos,
                                  const Eigen::Vector3d& init_vel) {
    // Homogeneous general-solution values at the rollout start s = 0:
    //   y1(0) = 1,  y2(0) = 0,  y1'(0) = -alpha/2,  y2'(0) = 1
    const double y1_0 = 1.0;
    const double y2_0 = 0.0;
    const double dy1_0 = -0.5 * alpha_ * y1_0;
    const double dy2_0 = y1_0 - 0.5 * alpha_ * y2_0;

    // Wronskian of the homogeneous system. Analytically W(s) = y1(s)^2 =
    // exp(-alpha*s), strictly positive for every finite s only while alpha is a
    // finite positive gain; at s = 0 it then evaluates to exactly 1 and the 2x2
    // system for (c1, c2) is well-posed. The degenerate case is theoretical:
    // alpha <= 0 collapses the critically damped double root (r = -alpha/2), so
    // y1 and y2 = s*y1 stop being a decaying attractor basis, and a non-finite
    // alpha turns det into NaN. Either way dividing the homogeneous
    // coefficients below by this value would produce silent garbage, so we
    // recompute det, require a sane alpha, and refuse loudly otherwise.
    const double det = y1_0 * dy2_0 - y2_0 * dy1_0;
    if (!std::isfinite(det) || std::abs(det) < kMinWronskian || !(alpha_ > 0.0)) {
        throw std::runtime_error(
            "ProDMP::setInitialConditions: degenerate homogeneous system - alpha must be a "
            "finite positive gain (critically damped attractor requires alpha > 0).");
    }

    init_time_ = init_time;
    init_pos_ = init_pos;
    init_vel_ = init_vel;
    wronskian_det_ = det;
    diag_.initial_vel_norm = init_vel.norm();

    resetIntegrationState();
    refreshGoalCache();
}

void ProDMP::setGoal(const Eigen::Vector3d& new_goal) {
    // Deliberately no kMinDG / amplitude-ratio guard (contrast core::DMP::setGoal):
    // the goal is a plain linear parameter with its own closed-form basis, and
    // reassigning it neither rescales nor perturbs the shape weights w_i.
    goal_param_ = relative_goal_ ? (new_goal - init_pos_) : new_goal;
    refreshGoalCache();
}

void ProDMP::setRelativeGoal(bool relative) {
    if (relative == relative_goal_) return;
    // Keep the absolute goal fixed across the representation switch.
    const Eigen::Vector3d goal_abs = goal_;
    relative_goal_ = relative;
    goal_param_ = relative_goal_ ? (goal_abs - init_pos_) : goal_abs;
    refreshGoalCache();
}

void ProDMP::learnFromDemonstration(const std::vector<Sample>& demo_in, double tau_override) {
    // Defence in depth, same rationale as core::DMP: a repeated/backwards
    // timestamp would corrupt the first-interval velocity estimate and the
    // scaled-time grid. Drop such rows and make the count visible.
    int dropped_samples = 0;
    const std::vector<Sample> demo =
        filter_utils::dropNonIncreasingTimeSamples(demo_in, &dropped_samples);
    diag_.dropped_non_monotonic_samples = dropped_samples;

    if (demo.size() < 5) {
        throw std::runtime_error(
            "ProDMP::learnFromDemonstration: demonstration too short (need at least 5 samples).");
    }

    const int N = static_cast<int>(demo.size());
    const double t0 = demo.front().t;
    const double demo_span = demo.back().t - t0;
    if (demo_span <= 0.0) {
        throw std::runtime_error(
            "ProDMP::learnFromDemonstration: non-increasing timestamps in demonstration.");
    }
    // tau_override <= 0.0 (the default) reproduces the previous behaviour
    // exactly: tau_ = span of the passed demo. A positive override is used only
    // by offline hold-out studies that fit on a leading fraction of a movement
    // but want the phase grid s_k = (t_k - t0)/tau_ anchored to the FULL
    // movement duration, so the training rows sit at their true partial phase
    // instead of being stretched to cover [0, 1].
    tau_ = (tau_override > 0.0) ? tau_override : demo_span;
    if (tau_ <= 0.0) {
        throw std::runtime_error(
            "ProDMP::learnFromDemonstration: tau_override must be positive.");
    }

    // Optional position pre-filter (disabled by default: window_sec <= 0.0
    // reproduces today's behaviour bit-for-bit, movingAverageSmooth() returns
    // the input unchanged). When enabled, the smoothed trajectory feeds BOTH
    // the init_pos/init_vel finite difference AND the design matrix B below -
    // same reused filter_utils::movingAverageSmooth core::DMP applies to its
    // own velocity estimate.
    std::vector<double> t_all(N);
    std::vector<Eigen::Vector3d> pos_all(N);
    for (int k = 0; k < N; ++k) {
        t_all[k] = demo[k].t;
        pos_all[k] = demo[k].position;
    }
    const std::vector<Eigen::Vector3d> pos_for_fit =
        (position_filter_window_sec_ > 0.0)
            ? filter_utils::movingAverageSmooth(pos_all, t_all, position_filter_window_sec_)
            : pos_all;

    // Initial conditions from the first samples (finite difference over the
    // first interval), matching learn_mp_params_from_trajs in prodmp.py.
    const double init_time = t0;
    const Eigen::Vector3d init_pos = pos_for_fit.front();
    const double dt0 = demo[1].t - demo[0].t;
    const Eigen::Vector3d init_vel = (pos_for_fit[1] - pos_for_fit[0]) / dt0;
    const Eigen::Vector3d init_vel_scaled = init_vel * tau_;  // d y / d s = tau * d y / d t

    // Homogeneous Wronskian at s = 0 (W(0) = y1(0)^2 = 1); computed here so the
    // design matrix does not depend on any previously stored rollout state.
    const double dy1_0 = -0.5 * alpha_;
    const double dy2_0 = 1.0;
    const double det_ic = 1.0 * dy2_0 - 0.0 * dy1_0;

    const int P = num_basis_ + 1;  // shape weights + goal (num_basis_g)

    // Build the design matrix H (N x P). Row k holds the closed-form basis
    // values at scaled time s_k; column P-1 is the goal step-response basis.
    // The homogeneous initial-condition correction terms (xi_3 / xi_4 in the
    // reference) vanish here because every basis is evaluated from s = 0, where
    // pos_basis_*(0) = 0, so H carries the pure particular solution.
    Eigen::MatrixXd H(N, P);
    Eigen::MatrixXd B(N, 3);

    Eigen::VectorXd p1(Eigen::VectorXd::Zero(num_basis_));
    Eigen::VectorXd p2(Eigen::VectorXd::Zero(num_basis_));
    Eigen::VectorXd dp1_prev(num_basis_);
    Eigen::VectorXd dp2_prev(num_basis_);
    Eigen::VectorXd psi(num_basis_);
    {
        const double x0 = canonicalPhase(0.0);
        evalBasis(x0, psi);
        for (int i = 0; i < num_basis_; ++i) {
            dp1_prev(i) = 0.0;
            dp2_prev(i) = x0 * psi(i);
        }
    }

    double s_prev = 0.0;
    for (int k = 0; k < N; ++k) {
        const double s = (demo[k].t - t0) / tau_;
        const double ds = s - s_prev;

        const double x = canonicalPhase(s);
        evalBasis(x, psi);
        const double e = std::exp(0.5 * alpha_ * s);

        // Incremental trapezoid for p1_i(s), p2_i(s).
        for (int i = 0; i < num_basis_; ++i) {
            const double dp1 = s * e * x * psi(i);
            const double dp2 = e * x * psi(i);
            p1(i) += 0.5 * (dp1_prev(i) + dp1) * ds;
            p2(i) += 0.5 * (dp2_prev(i) + dp2) * ds;
            dp1_prev(i) = dp1;
            dp2_prev(i) = dp2;
        }

        // General-solution values at s.
        const double y1 = std::exp(-0.5 * alpha_ * s);
        const double y2 = s * y1;

        // Shape bases: pos_basis_w_i(s) = y2 * p2_i - y1 * p1_i.
        for (int i = 0; i < num_basis_; ++i) {
            H(k, i) = y2 * p2(i) - y1 * p1(i);
        }

        // Goal step-response basis: pos_basis_g(s) = y2 * q2 - y1 * q1.
        const double q1 = (0.5 * alpha_ * s - 1.0) * e + 1.0;
        const double q2 = 0.5 * alpha_ * (e - 1.0);
        const double pos_g = y2 * q2 - y1 * q1;
        H(k, P - 1) = pos_g;

        // Homogeneous coefficients at s (init conditions imposed at s = 0):
        //   xi_1(s) = (y2'(0) y1(s) - y1'(0) y2(s)) / W ,  xi_2(s) = y2(s)
        const double xi1 = (dy2_0 * y1 - dy1_0 * y2) / det_ic;
        const double xi2 = y2;

        for (int d = 0; d < 3; ++d) {
            double target = pos_for_fit[k](d) - (init_pos(d) * xi1 + init_vel_scaled(d) * xi2);
            if (relative_goal_) {
                // The goal parameter then represents (goal - init_pos); peel the
                // init_pos * pos_basis_g(s) contribution off the target so the
                // solve recovers that relative quantity.
                target -= init_pos(d) * pos_g;
            }
            B(k, d) = target;
        }

        s_prev = s;
    }

    // Column-scale preconditioning with a RELATIVE FLOOR, mirroring
    // auto_compute_basis_scale_factors() in the reference prodmp_basis.py but
    // refusing to "promote" quasi-degenerate columns to unit scale.
    //
    // Two earlier attempts both failed for opposite reasons (see git history /
    // task notes for the numeric evidence):
    //  - ridge summed AFTER unconditional per-column scaling (H.col(i) *=
    //    1/max|H.col(i)|, THEN ridge on H_scaled^T H_scaled) equalizes every
    //    column to unit peak - including columns the demo barely activates
    //    (e.g. tail RBFs on a long, slow demo). In physical units that is
    //    equivalent to a PER-COLUMN ridge of ridge_lambda * max|H.col(i)|^2,
    //    which for a near-degenerate column (max|H.col(i)| ~ 1e-7) collapses
    //    to ~ridge_lambda * 1e-14 - de-regularizes it almost completely, so
    //    the solver fits noise there and the final rescale amplifies that
    //    noise back into huge physical weights.
    //  - ridge summed BEFORE scaling, scaling applied as a symmetric
    //    congruence S*A*S on the already-ridged normal matrix, is provably
    //    equivalent (exact arithmetic) to solving the unscaled system
    //    directly - a pure numerical-conditioning change with NO effect on
    //    the fitted solution, so it reproduces the original (pre-any-fix)
    //    pathological fit on data with a genuinely wide, real information
    //    spread across columns (e.g. trajC/n_basis=200).
    //
    // The actual data (see colmax_diag tool, trajC/n_basis=200) show no clean
    // two-cluster split among the shape-weight columns: max|H.col(i)| forms a
    // SMOOTH geometric progression from ~4e-8 up to ~1.4e-4 relative to the
    // largest column, with the goal column alone standing ~4 orders of
    // magnitude above that whole progression at ~1.0. There is no scale-free
    // way to tell "under-activated tail RBF" from "genuinely informative but
    // naturally small column" from magnitude alone, so kScaleFloorRatio is a
    // real threshold choice, not a fixed physical boundary. Swept empirically
    // in decade and half-decade steps on demo_raw_trajC.csv (num_basis=200,
    // ridge_lambda=1e-6, filter=0.20s) against test/test_prodmp.cpp's default
    // ProDMP(20)/ridge=1e-9/no-filter scenarios (all 7 must keep passing):
    // every floor <= 3e-6 breaks 2 of the 7 tests (SetGoalPreservesShapeWeights
    // convergence check, RelativeGoalGeneralization both checks) while trajC's
    // rmse_overall_mm bottoms out near 0.21mm; every floor >= 4e-6 keeps all 7
    // tests passing, with trajC's rmse_overall_mm degrading smoothly back
    // toward the unscaled 2.998mm baseline as the floor grows (4e-6: 0.665mm,
    // 5e-6: 0.713mm, 1e-4: 2.991mm, 1e-3: 2.998mm - see task verification for
    // the full table). 5e-6 is one sweep step above the 3e-6 -> 4e-6 failure
    // cliff (not the cliff value itself, for margin against a slightly
    // different demo/config shifting the boundary) and was cross-checked on
    // demo_raw_trajA.csv - an INDEPENDENT trajectory not used to pick this
    // value - where it still cuts rmse_overall_mm from 2.621mm to 1.057mm. A
    // column whose peak magnitude falls below this fraction of the overall
    // largest column peak is left at its natural (small) scale, so
    // ridge_lambda_ dominates and suppresses it exactly as it did before any
    // column-scale preconditioning existed; columns above the floor are
    // equalized to unit peak so the (now comparable) ridge_lambda_ regularizes
    // them evenly instead of favoring whichever happened to have the largest
    // natural magnitude.
    constexpr double kScaleFloorRatio = 5e-6;

    // Closed-form goal option (Li et al. 2023, Eq. 11: g is a KNOWN constant =
    // trajectory endpoint in the classic DMP, not a regressed quantity). When
    // fix_goal_to_demo_endpoint_ is set we peel the goal-column contribution
    // out of the target using the demonstration's final RAW position (the same
    // y_demo(tau) the verification harnesses use for final_position_error) and
    // fit ONLY the shape weights on the residual. The physical goal column of H
    // is captured here BEFORE the column scaling below, because that peel is
    // done in physical units. Off by default -> identical joint estimation.
    const Eigen::VectorXd goal_col_phys = H.col(P - 1);
    const Eigen::Vector3d demo_endpoint = demo.back().position;

    const double overall_max_abs = H.cwiseAbs().maxCoeff();
    Eigen::VectorXd scale(P);
    for (int i = 0; i < P; ++i) {
        const double col_max_abs = H.col(i).cwiseAbs().maxCoeff();
        const bool below_floor =
            (overall_max_abs < 1e-300) || (col_max_abs < kScaleFloorRatio * overall_max_abs);
        // Guard/floor: a column whose peak is (numerically) zero, or too small
        // relative to the largest column to be trusted as genuinely
        // informative, is left unscaled - ridge_lambda_ dominates it at its
        // natural small magnitude, same protection as pre-scaling code.
        scale(i) = below_floor ? 1.0 : (1.0 / col_max_abs);
        H.col(i) *= scale(i);
    }

    // In fixed-goal mode the goal column is dropped from the regression, so the
    // system the LDLT actually factorizes is the first num_basis_ columns only.
    const int P_fit = fix_goal_to_demo_endpoint_ ? num_basis_ : P;

    // Opt-in conditioning diagnostic: 2-norm cond(H) of the column-scaled
    // design matrix (sigma_max / sigma_min) that is actually solved. Never on
    // the default path - the JacobiSVD of an (N x P) matrix costs far more than
    // the whole fit - only when an offline study explicitly asked for it.
    diag_.design_matrix_condition_number = 0.0;
    if (compute_condition_number_) {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(H.leftCols(P_fit));
        const Eigen::VectorXd sv = svd.singularValues();
        const double sv_min = sv(sv.size() - 1);
        diag_.design_matrix_condition_number =
            (sv_min > 0.0) ? (sv(0) / sv_min) : std::numeric_limits<double>::infinity();
    }

    // Least-squares solve per axis on the normal equations, shared Gram
    // factorization (LDLT) - same solver family as core::DMP's ridge fit.
    // H is column-scaled (above-floor columns only). A is (P_fit x P_fit):
    // full joint system by default, shape-only when the goal is fixed.
    Eigen::MatrixXd A = H.leftCols(P_fit).transpose() * H.leftCols(P_fit);
    A.diagonal().array() += ridge_lambda_;
    Eigen::LDLT<Eigen::MatrixXd> solver(A);

    // When the goal is fixed, goal_param_(d) is that known endpoint value
    // (expressed relative to init_pos when relative_goal_ is set, to match the
    // convention goal() = relative_goal_ ? goal_param_ + init_pos_ : goal_param_),
    // and its already-known contribution is subtracted from the target so the
    // shape weights fit only the residual.
    Eigen::Vector3d goal_fixed(Eigen::Vector3d::Zero());
    if (fix_goal_to_demo_endpoint_) {
        for (int d = 0; d < 3; ++d) {
            goal_fixed(d) = relative_goal_ ? (demo_endpoint(d) - init_pos(d)) : demo_endpoint(d);
        }
    }

    double sq_res = 0.0;
    for (int d = 0; d < 3; ++d) {
        Eigen::VectorXd rhs_target = B.col(d);
        if (fix_goal_to_demo_endpoint_) {
            rhs_target -= goal_col_phys * goal_fixed(d);  // physical-units peel
        }

        // params is in the SCALED basis here; H_fit * params - rhs_target is the
        // same residual as H_original_fit * params_physical - rhs_target, since
        // H_scaled = H_original * diag(scale). Compute the residual before
        // rescaling. In fixed-goal mode this residual already reflects the full
        // model (shape + known goal) vs the ORIGINAL B, so learn_residual_rms
        // stays directly comparable to the default path.
        Eigen::VectorXd params =
            solver.solve(H.leftCols(P_fit).transpose() * rhs_target);
        sq_res += (H.leftCols(P_fit) * params - rhs_target).squaredNorm();

        // Rescale the recovered parameters back to physical units.
        params = params.cwiseProduct(scale.head(P_fit));
        weights_[d] = params.head(num_basis_);
        goal_param_(d) = fix_goal_to_demo_endpoint_ ? goal_fixed(d) : params(num_basis_);
    }
    diag_.learn_residual_rms = std::sqrt(sq_res / (3.0 * N));

    // Ready the object for an immediate rollout from the demonstrated start.
    setInitialConditions(init_time, init_pos, init_vel);
    learned_ = true;
}

Eigen::Vector3d ProDMP::step(double dt) {
    if (!learned_) {
        throw std::runtime_error(
            "ProDMP::step: parameters not learned or loaded. Call learnFromDemonstration() "
            "or setLearnedParameters() first.");
    }

    const double s_prev = s_;
    s_ += dt / tau_;
    const double ds = s_ - s_prev;

    const double x = canonicalPhase(s_);
    Eigen::VectorXd psi(num_basis_);
    evalBasis(x, psi);
    const double e = std::exp(0.5 * alpha_ * s_);

    // Incremental trapezoidal accumulation of the per-basis forcing integrals:
    //   p1_i(s) = integral_0^s s' e^{alpha s'/2} x(s') psi_i(s') ds'
    //   p2_i(s) = integral_0^s    e^{alpha s'/2} x(s') psi_i(s') ds'
    for (int i = 0; i < num_basis_; ++i) {
        const double dp1 = s_ * e * x * psi(i);
        const double dp2 = e * x * psi(i);
        p1_accum_(i) += 0.5 * (dp1_prev_(i) + dp1) * ds;
        p2_accum_(i) += 0.5 * (dp2_prev_(i) + dp2) * ds;
        dp1_prev_(i) = dp1;
        dp2_prev_(i) = dp2;
    }

    // General-solution values and their s-derivatives at s_.
    const double y1 = std::exp(-0.5 * alpha_ * s_);
    const double y2 = s_ * y1;
    const double dy1 = -0.5 * alpha_ * y1;
    const double dy2 = y1 - 0.5 * alpha_ * y2;  // = y1 * (1 - alpha/2 * s)

    // Homogeneous coefficients: initial conditions imposed at s = 0.
    //   xi_1(s) = (y2'(0) y1(s) - y1'(0) y2(s)) / W        [value 1, slope 0 at s=0]
    //   xi_2(s) = y2(s)                                     [value 0, slope 1 at s=0]
    const double dy1_0 = -0.5 * alpha_;
    const double dy2_0 = 1.0;
    const double xi1 = (dy2_0 * y1 - dy1_0 * y2) / wronskian_det_;
    const double xi2 = y2;
    const double dxi1 = (dy2_0 * dy1 - dy1_0 * dy2) / wronskian_det_;
    const double dxi2 = dy2;

    // Goal step-response basis (closed form, no quadrature).
    const double q1 = (0.5 * alpha_ * s_ - 1.0) * e + 1.0;
    const double q2 = 0.5 * alpha_ * (e - 1.0);
    const double pos_g = y2 * q2 - y1 * q1;
    const double vel_g = dy2 * q2 - dy1 * q1;

    const Eigen::Vector3d init_vel_scaled = init_vel_ * tau_;

    // y(s)      = init_pos * xi1 + init_vel_scaled * xi2 + sum_i w_i * pos_basis_w_i + g * pos_basis_g
    // dy/ds (s) = init_pos * dxi1 + init_vel_scaled * dxi2 + sum_i w_i * vel_basis_w_i + g * vel_basis_g
    Eigen::Vector3d pos = init_pos_ * xi1 + init_vel_scaled * xi2;
    Eigen::Vector3d vel_s = init_pos_ * dxi1 + init_vel_scaled * dxi2;

    for (int i = 0; i < num_basis_; ++i) {
        const double pos_w = y2 * p2_accum_(i) - y1 * p1_accum_(i);
        const double vel_w = dy2 * p2_accum_(i) - dy1 * p1_accum_(i);
        for (int d = 0; d < 3; ++d) {
            pos(d) += weights_[d](i) * pos_w;
            vel_s(d) += weights_[d](i) * vel_w;
        }
    }

    pos += goal_param_ * pos_g;
    vel_s += goal_param_ * vel_g;
    if (relative_goal_) {
        // Goal parameter holds (goal - init_pos); add the init_pos share back.
        pos += init_pos_ * pos_g;
        vel_s += init_pos_ * vel_g;
    }

    pos_ = pos;
    vel_ = vel_s / tau_;  // back to real time scale
    return pos_;
}

void ProDMP::setLearnedParameters(double tau, const Eigen::Vector3d& init_pos,
                                  const Eigen::Vector3d& init_vel, const Eigen::Vector3d& goal_param,
                                  const Eigen::VectorXd& centers, const Eigen::VectorXd& widths,
                                  const std::array<Eigen::VectorXd, 3>& weights, bool relative_goal) {
    tau_ = tau;
    centers_ = centers;
    widths_ = widths;
    weights_ = weights;
    num_basis_ = static_cast<int>(centers.size());
    relative_goal_ = relative_goal;
    goal_param_ = goal_param;

    p1_accum_ = Eigen::VectorXd::Zero(num_basis_);
    p2_accum_ = Eigen::VectorXd::Zero(num_basis_);
    dp1_prev_ = Eigen::VectorXd::Zero(num_basis_);
    dp2_prev_ = Eigen::VectorXd::Zero(num_basis_);

    setInitialConditions(0.0, init_pos, init_vel);
    learned_ = true;
}

}  // namespace core
}  // namespace haptic_dmp_learning
