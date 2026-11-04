#include "real_replay/target_source.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace real_replay {

// ---------------------------------------------------------------------------------------------
// HoldSource
// ---------------------------------------------------------------------------------------------

HoldSource::HoldSource(double duration_s) : duration_(duration_s) {
  if (!(std::isfinite(duration_s) && duration_s > 0.0)) {
    throw std::invalid_argument("HoldSource: duration must be finite and > 0");
  }
}

void HoldSource::start(const Pose& activation) {
  pose_ = activation;
  elapsed_ = 0.0;
}

Pose HoldSource::sample(double dt) {
  elapsed_ += dt;
  return pose_;
}

double HoldSource::progress() const { return std::min(1.0, elapsed_ / duration_); }

// ---------------------------------------------------------------------------------------------
// StepSource
// ---------------------------------------------------------------------------------------------

namespace {
double smoothstep(double x) {
  x = std::min(1.0, std::max(0.0, x));
  return x * x * (3.0 - 2.0 * x);
}
double smoothstepDerivative(double x) {  // d/dx, zero outside (0, 1)
  if (x <= 0.0 || x >= 1.0) return 0.0;
  return 6.0 * x * (1.0 - x);
}
}  // namespace

StepSource::StepSource(const StepConfig& cfg, bool rotational) : cfg_(cfg), rotational_(rotational) {
  const Array3& ax = rotational ? cfg.rot_axis : cfg.axis;
  axis_ = Eigen::Vector3d(ax[0], ax[1], ax[2]);
  if (!(axis_.norm() > 1e-9)) throw std::invalid_argument("StepSource: axis must be non-zero");
  axis_.normalize();
  if (rotational && !(std::isfinite(cfg.rot_angle_rad) && cfg.rot_angle_rad > 0.0)) {
    throw std::invalid_argument("StepSource: rot_angle_rad must be finite and > 0");
  }
  if (!(std::isfinite(cfg.amplitude_m) && cfg.amplitude_m > 0.0 && cfg.move_duration_s > 0.0 &&
        cfg.dwell_s >= 0.0)) {
    throw std::invalid_argument("StepSource: invalid amplitude/durations");
  }
  if (cfg.return_to_start && !(cfg.return_duration_s > 0.0)) {
    throw std::invalid_argument("StepSource: return_duration_s must be > 0");
  }
  total_ = cfg.move_duration_s + cfg.dwell_s + (cfg.return_to_start ? cfg.return_duration_s : 0.0);
}

void StepSource::profile(double t, double& f, double& df) const {
  const double t1 = cfg_.move_duration_s;
  const double t2 = t1 + cfg_.dwell_s;
  if (t < t1) {
    f = smoothstep(t / t1);
    df = smoothstepDerivative(t / t1) / t1;
  } else if (t < t2 || !cfg_.return_to_start) {
    f = 1.0;
    df = 0.0;
  } else {
    const double x = (t - t2) / cfg_.return_duration_s;
    f = 1.0 - smoothstep(x);
    df = -smoothstepDerivative(x) / cfg_.return_duration_s;
  }
}

void StepSource::start(const Pose& activation) {
  act_ = activation;
  elapsed_ = 0.0;
}

Pose StepSource::sample(double dt) {
  elapsed_ += dt;
  double f = 0.0;
  double df = 0.0;
  profile(elapsed_, f, df);
  Pose out = act_;
  if (rotational_) {
    // Base-frame (left) rotation of the activation orientation; position untouched.
    const Eigen::Quaterniond dq(Eigen::AngleAxisd(cfg_.rot_angle_rad * f, axis_));
    out.q = (dq * act_.q).normalized();
  } else {
    out.p = act_.p + axis_ * (cfg_.amplitude_m * f);
  }
  return out;
}

double StepSource::progress() const { return std::min(1.0, elapsed_ / total_); }

bool StepSource::twist(Eigen::Vector3d& v, Eigen::Vector3d& w) const {
  double f = 0.0;
  double df = 0.0;
  profile(elapsed_, f, df);
  if (rotational_) {
    v.setZero();
    w = axis_ * (cfg_.rot_angle_rad * df);
  } else {
    v = axis_ * (cfg_.amplitude_m * df);
    w.setZero();
  }
  return true;
}

}  // namespace real_replay
