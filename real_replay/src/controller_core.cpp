#include "real_replay/controller_core.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace real_replay {
namespace {

Array7 toArray(const core::RobotModel::JointVector& v) {
  Array7 a{};
  for (int i = 0; i < 7; ++i) a[static_cast<std::size_t>(i)] = v(i);
  return a;
}

core::CartesianImpedanceSolver::Params makeParams(const GainsConfig& g) {
  core::CartesianImpedanceSolver::Params p;
  p.translational_stiffness = g.translational_stiffness;
  p.rotational_stiffness = g.rotational_stiffness;
  // Damping is NOT done by the solver: it is added per axis by axisDampingTorque(). With
  // Dt = Dr = 0 the solver's own damping term vanishes. gains.translational/rotational_damping
  // are intentionally unused.
  p.translational_damping = 0.0;
  p.rotational_damping = 0.0;
  p.nullspace_stiffness = g.nullspace_stiffness;
  p.joint1_nullspace_stiffness = g.joint1_nullspace_stiffness;
  p.nullspace_pinv_damping = g.nullspace_pinv_damping;
  p.delta_tau_max = g.delta_tau_max;
  // The robot compensates gravity itself in FCI torque mode (libfranka control_types.h:43:
  // "torque commands without gravity and friction").
  p.compensate_gravity_internally = false;
  return p;
}

std::size_t logCapacity(const Config& c) {
  // One record per cycle at nominal 1 kHz, for the full run plus the controlled stop, with margin.
  const double cycles = (c.run.max_duration_s + c.safety.stop_duration_s) / 1e-3 + 1000.0;
  return static_cast<std::size_t>(std::ceil(cycles));
}

}  // namespace

double delayedSpectralRadius(double a, double b, int delay) {
  // p(z) = z^delay (z-1)^2 + a (z-1) + b, monic of degree n = delay + 2.
  const int n = delay + 2;
  Eigen::VectorXd c = Eigen::VectorXd::Zero(n + 1);  // c[k] = coefficient of z^k
  // z^delay (z^2 - 2z + 1)
  c[delay + 2] += 1.0;
  c[delay + 1] += -2.0;
  c[delay] += 1.0;
  c[1] += a;
  c[0] += b - a;
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(n, n);
  for (int j = 0; j < n; ++j) C(0, j) = -c[n - 1 - j];
  for (int i = 1; i < n; ++i) C(i, i - 1) = 1.0;
  const Eigen::EigenSolver<Eigen::MatrixXd> es(C, /*computeEigenvectors=*/false);
  return es.eigenvalues().cwiseAbs().maxCoeff();
}

double delayedSpectralRadius(double a, double b) { return delayedSpectralRadius(a, b, 1); }

double gainMargin(double a, double b, int delay) {
  constexpr double kUnstable = 1.0 - 1e-12;  // "stable" means max|z| < kUnstable (rounding guard)
  auto stable = [&](double g) { return delayedSpectralRadius(g * a, g * b, delay) < kUnstable; };
  if (!stable(1.0)) return 0.0;
  double lo = 1.0, hi = 1.0;
  while (true) {
    hi = lo * 1.05;
    if (!stable(hi)) break;
    lo = hi;
    if (lo > 1e6) return std::numeric_limits<double>::infinity();
  }
  for (int it = 0; it < 100; ++it) {
    const double mid = 0.5 * (lo + hi);
    (stable(mid) ? lo : hi) = mid;
  }
  return lo;
}

double gainMarginClosedForm1(double a, double b) {
  if (!(a > 2.0 * b)) return std::numeric_limits<double>::quiet_NaN();
  return (a - 2.0 * b) / ((a - b) * (a - b));
}

DampingDesign designDamping(const GainsConfig& gains, const DampingConfig& damping, double stop_scale,
                            double dt, const Array6& lambda_act, const Array6& lambda_min,
                            const Array6& lambda_max, bool path_known) {
  DampingDesign r;
  r.path_known = path_known;
  r.lambda_act = lambda_act;
  r.lambda_max = lambda_max;
  for (std::size_t i = 0; i < 6; ++i) {
    const double act = lambda_act[i];
    const double mn = std::min(lambda_min[i], act);  // the cap must use the smaller of the two
    if (!(std::isfinite(act) && act > 0.0 && std::isfinite(mn) && mn > 0.0)) {
      throw std::invalid_argument("designDamping: Lambda must be finite and > 0 on every axis");
    }
    r.lambda_min[i] = mn;
    const bool trans = i < 3;
    r.k[i] = trans ? gains.translational_stiffness : gains.rotational_stiffness;
    const double zeta_target = trans ? damping.zeta_trans : damping.zeta_rot;
    const double d_zeta = 2.0 * zeta_target * std::sqrt(r.k[i] * act);
    const double d_cap = damping.max_damping_per_sample * mn / dt;
    r.d[i] = std::min(d_zeta, d_cap);
    r.capped[i] = d_cap < d_zeta;
    r.d_stop[i] = std::min(stop_scale * r.d[i], d_cap);
  }
  if (!damping.override_d.empty()) {
    r.overridden = true;
    for (std::size_t i = 0; i < 6; ++i) {
      r.d[i] = damping.override_d[i];
      r.d_stop[i] = stop_scale * damping.override_d[i];
      r.capped[i] = false;
    }
  }
  for (std::size_t i = 0; i < 6; ++i) {
    r.zeta[i] = r.d[i] / (2.0 * std::sqrt(r.k[i] * r.lambda_act[i]));
    r.a[i] = r.d[i] * dt / r.lambda_min[i];
    r.zeta_lmax[i] = r.d[i] / (2.0 * std::sqrt(r.k[i] * r.lambda_max[i]));
    r.b[i] = r.k[i] * dt * dt / r.lambda_min[i];
    r.rho[i] = delayedSpectralRadius(r.a[i], r.b[i]);
    r.g1[i] = gainMargin(r.a[i], r.b[i], 1);
    r.g2[i] = gainMargin(r.a[i], r.b[i], 2);
  }
  return r;
}

core::RobotModel::JointVector axisDampingTorque(const core::RobotModel::Jacobian6x7& J,
                                                const core::RobotModel::JointVector& dq,
                                                const Vector6& D) noexcept {
  const Vector6 twist = J * dq;
  const Vector6 force = -(D.cwiseProduct(twist));
  return J.transpose() * force;
}

ReplayLoop::ReplayLoop(const Config& cfg, const std::string& urdf_xml,
                       std::unique_ptr<ITargetSource> source)
    : cfg_(cfg),
      model_(urdf_xml, cfg.robot.joint_names, cfg.robot.ee_frame),
      solver_(makeParams(cfg.gains)),
      base_params_(makeParams(cfg.gains)),
      guard_(cfg.safety, toArray(model_.jointLowerLimits()), toArray(model_.jointUpperLimits())),
      source_(std::move(source)),
      logger_(logCapacity(cfg)) {}

double ReplayLoop::gainAlpha(double t) const noexcept {
  const double a0 = cfg_.gains.gain_ramp_initial;
  const double ramp = cfg_.gains.gain_ramp_s;
  if (!(ramp > 0.0)) return 1.0;
  const double x = std::min(1.0, std::max(0.0, t / ramp));
  const double s = x * x * (3.0 - 2.0 * x);  // smoothstep
  return a0 + (1.0 - a0) * s;
}

void ReplayLoop::enterStopping(StopReason r, const Eigen::Vector3d& p,
                               const Eigen::Quaterniond& q) noexcept {
  if (phase_ != LoopPhase::kRunning) return;
  stop_reason_ = r;
  phase_ = LoopPhase::kStopping;
  stop_target_.p = p;
  stop_target_.q = q;
  stop_elapsed_ = 0.0;
}

void ReplayLoop::prepareDamping(const Array7& q, const Array6* lambda_min, const Array6* lambda_max) {
  JointVec qv;
  for (int i = 0; i < 7; ++i) qv(i) = q[static_cast<std::size_t>(i)];
  model_.update(qv, JointVec::Zero());
  const core::RobotModel::Matrix6d L = model_.reflectedCartesianInertia();  // not real-time: called outside the loop
  Array6 act{};
  for (int i = 0; i < 6; ++i) act[static_cast<std::size_t>(i)] = L(i, i);
  design_ = designDamping(cfg_.gains, cfg_.damping, cfg_.safety.stop_damping_scale, kNominalCycleS, act,
                          lambda_min ? *lambda_min : act, lambda_max ? *lambda_max : act, lambda_min != nullptr);
  for (int i = 0; i < 6; ++i) {
    d_nom_(i) = design_.d[static_cast<std::size_t>(i)];
    d_stop_(i) = design_.d_stop[static_cast<std::size_t>(i)];
  }
  q_design_ = q;
  damping_ready_ = true;
}

void ReplayLoop::setLastCallbackUs(double us) noexcept {
  last_cb_us_ = us;
  if (last_rec_) last_rec_->cb_us = us;
}

void ReplayLoop::markException(const char* what) noexcept {
  stop_reason_ = StopReason::kException;
  phase_ = LoopPhase::kFinished;
  try {
    exception_msg_ = what ? what : "";
  } catch (...) {
  }
}

void ReplayLoop::step(const LoopInput& in, LoopOutput& out) {
  out.tau.fill(0.0);
  out.finished = false;
  last_rec_ = nullptr;

  if (phase_ == LoopPhase::kFinished) {
    out.finished = true;
    return;
  }

  JointVec q;
  JointVec dq;
  for (int i = 0; i < 7; ++i) {
    q(i) = in.q[static_cast<std::size_t>(i)];
    dq(i) = in.dq[static_cast<std::size_t>(i)];
  }
  if (!q.allFinite() || !dq.allFinite()) {
    if (stop_reason_ == StopReason::kNone) stop_reason_ = StopReason::kNonFiniteState;
    phase_ = LoopPhase::kFinished;
    out.finished = true;  // zero torque
    return;
  }

  // Fallback only: main() calls prepareDamping() before the loop starts.
  if (phase_ == LoopPhase::kWaitFirst && !damping_ready_) prepareDamping(in.q, nullptr, nullptr);

  model_.update(q, dq);
  const Eigen::Vector3d p_meas = model_.eePosition();
  const Eigen::Quaterniond q_meas = model_.eeOrientation();

  // ---- activation (first cycle) / time bookkeeping ------------------------------------------
  if (phase_ == LoopPhase::kWaitFirst) {
    double drift = 0.0;
    for (std::size_t i = 0; i < 7; ++i) drift = std::max(drift, std::fabs(in.q[i] - q_design_[i]));
    if (drift > cfg_.damping.max_q_drift_rad) {
      // The robot moved between the damping design and the start of control: the design is stale.
      stop_reason_ = StopReason::kActivationDrift;
      phase_ = LoopPhase::kFinished;
      out.finished = true;  // zero torque
      return;
    }
    tau_solver_prev_.setZero();
    t_ = 0.0;
    cycle_ = 0;
    activation_.p = p_meas;
    activation_.q = q_meas;
    solver_.setNullspaceTarget(q);  // without this the nullspace would pull towards q = 0
    guard_.reset();
    source_->start(activation_);
    target_ = source_->sample(0.0);
    first_target_ = target_;
    have_first_target_ = true;
    pending_dt_ = 0.0;
    since_update_ = 0;
    phase_ = LoopPhase::kRunning;
  } else {
    t_ += in.period_s;
    ++cycle_;
    if (phase_ == LoopPhase::kRunning) {
      pending_dt_ += in.period_s;
      if (++since_update_ >= cfg_.trajectory.target_update_divisor) {
        target_ = source_->sample(pending_dt_);
        pending_dt_ = 0.0;
        since_update_ = 0;
      }
    } else {
      stop_elapsed_ += in.period_s;
    }
  }

  // ---- error w.r.t. the active target -------------------------------------------------------
  const Pose* tgt = (phase_ == LoopPhase::kStopping) ? &stop_target_ : &target_;
  core::CartesianError err = core::computePoseError(p_meas, q_meas, tgt->p, tgt->q);

  // ---- supervision (only while running) -----------------------------------------------------
  if (phase_ == LoopPhase::kRunning) {
    StopReason r = StopReason::kNone;
    if (ext_stop_ != nullptr && *ext_stop_ != 0) {
      r = StopReason::kUserSignal;
    } else if (t_ >= cfg_.run.max_duration_s) {
      r = StopReason::kMaxDuration;
    } else if (logger_.full()) {
      r = StopReason::kLogFull;
    }
    if (r == StopReason::kNone) r = guard_.checkCycle(in.period_s, last_cb_us_, in.success_rate, cycle_);
    if (r == StopReason::kNone) r = guard_.checkState(in.q, in.dq, err.linear.norm(), err.angular.norm());
    // The source decides when it is over (replay: stop_at_fraction + hold_after_end_s live there).
    if (r == StopReason::kNone && source_->done()) r = StopReason::kCompleted;
    if (r != StopReason::kNone) {
      enterStopping(r, p_meas, q_meas);
      err = core::computePoseError(p_meas, q_meas, stop_target_.p, stop_target_.q);
    }
  }

  // ---- control law --------------------------------------------------------------------------
  const double alpha = gainAlpha(t_);
  core::CartesianImpedanceSolver::Params p = base_params_;
  p.translational_stiffness *= alpha;
  p.rotational_stiffness *= alpha;
  p.nullspace_stiffness *= alpha;
  solver_.setParams(p);

  // Solver (stiffness + nullspace + Coriolis, its own slew limit w.r.t. ITS previous output) plus
  // the per-axis damping term; the sum then goes through the safety layer.
  const JointVec tau_solver = solver_.computeTorque(model_, err, q, dq, tau_solver_prev_);
  const JointVec tau_damp =
      axisDampingTorque(model_.jacobian(), dq, phase_ == LoopPhase::kStopping ? d_stop_ : d_nom_);
  const JointVec tau_d = tau_solver + tau_damp;

  if (!tau_d.allFinite()) {
    if (stop_reason_ == StopReason::kNone || stop_reason_ == StopReason::kCompleted) {
      stop_reason_ = StopReason::kNonFiniteTorque;
    }
    phase_ = LoopPhase::kFinished;
    out.finished = true;  // zero torque
    return;
  }

  tau_solver_prev_ = tau_solver;
  const Array7 tau_arr = toArray(tau_d);
  guard_.limitTorque(tau_arr, out.tau);
  if (guard_.lastRateLimited()) ++n_rate_limited_;
  if (guard_.lastClamped()) ++n_clamped_;

  // ---- log ----------------------------------------------------------------------------------
  const LoopPhase phase_for_log = phase_;
  if (LogRecord* r = logger_.next()) {
    last_rec_ = r;
    r->t = t_;
    r->period = in.period_s;
    r->cb_us = 0.0;
    r->success_rate = in.success_rate;
    r->gain_alpha = alpha;
    r->phase = static_cast<double>(static_cast<int>(phase_for_log));
    r->flags = (guard_.lastRateLimited() ? 1.0 : 0.0) + (guard_.lastClamped() ? 2.0 : 0.0) +
               (phase_for_log == LoopPhase::kStopping ? 4.0 : 0.0);
    for (int i = 0; i < 7; ++i) {
      const std::size_t k = static_cast<std::size_t>(i);
      r->q[i] = in.q[k];
      r->dq[i] = in.dq[k];
      r->tau_J[i] = in.tau_J[k];
      r->tau_cmd[i] = out.tau[k];
    }
    for (int i = 0; i < 3; ++i) {
      r->p_des[i] = target_.p(i);
      r->p_meas[i] = p_meas(i);
      r->e_pos[i] = err.linear(i);
      r->e_rot[i] = err.angular(i);
    }
    r->q_des[0] = target_.q.w();
    r->q_des[1] = target_.q.x();
    r->q_des[2] = target_.q.y();
    r->q_des[3] = target_.q.z();
    r->q_meas[0] = q_meas.w();
    r->q_meas[1] = q_meas.x();
    r->q_meas[2] = q_meas.y();
    r->q_meas[3] = q_meas.z();
  }

  // ---- end of the controlled stop -----------------------------------------------------------
  if (phase_ == LoopPhase::kStopping && stop_elapsed_ >= cfg_.safety.stop_duration_s) {
    phase_ = LoopPhase::kFinished;
    out.finished = true;  // the torque computed above is still sent this cycle
  }
}

}  // namespace real_replay
