#include "real_replay/prodmp_source.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "real_replay/sha256.hpp"

namespace real_replay {

namespace hcore = haptic_dmp_learning::core;

ProDmpSource::ProDmpSource(const std::string& weights_path, const std::string& expected_sha256,
                           double stop_at_fraction, double hold_after_end_s)
    : prodmp_(20),  // placeholder, overwritten by the loader (same as the executor)
      qdmp_(20, 4.6, 25.0, 6.25),
      end_fraction_(stop_at_fraction),
      hold_after_end_s_(hold_after_end_s) {
  if (!(stop_at_fraction > 0.0 && stop_at_fraction <= 1.0)) {
    throw std::invalid_argument("ProDmpSource: stop_at_fraction must be in (0, 1]");
  }
  if (!(std::isfinite(hold_after_end_s) && hold_after_end_s >= 0.0)) {
    throw std::invalid_argument("ProDmpSource: hold_after_end_s must be finite and >= 0");
  }

  sha256_ = sha256FileHex(weights_path);
  if (!expected_sha256.empty() && expected_sha256 != sha256_) {
    throw std::runtime_error("ProDmpSource: SHA-256 of '" + weights_path + "' is " + sha256_ +
                             ", expected " + expected_sha256);
  }

  hcore::QuaternionDMP loaded_qdmp(20, 4.6, 25.0, 6.25);
  bool has_orientation = false;
  prodmp_ = hcore::prodmp_io::loadProDmpFromYaml(weights_path, loaded_qdmp, has_orientation);
  if (!has_orientation) {
    throw std::runtime_error("ProDmpSource: '" + weights_path +
                             "' has no inline quaternion_dmp section (orientation_weights_yaml_path "
                             "is deliberately not supported here)");
  }
  qdmp_ = loaded_qdmp;

  tau_ = prodmp_.tau();
  if (!(std::isfinite(tau_) && tau_ > 0.0)) throw std::runtime_error("ProDmpSource: invalid tau");
  if (std::fabs(tau_ - qdmp_.tau()) > 1e-6) {
    throw std::runtime_error("ProDmpSource: position tau and orientation tau differ");
  }
  end_time_ = end_fraction_ * tau_;
}

void ProDmpSource::start(const Pose& activation) {
  // Rewind exactly as the executor does before a rollout (executor.cpp:664-665).
  prodmp_.setInitialConditions(0.0, prodmp_.initPos(), prodmp_.initVel());
  qdmp_.reset();

  elapsed_ = 0.0;
  hold_elapsed_ = 0.0;
  frozen_ = false;

  p_raw0_ = prodmp_.step(0.0);
  q_raw0_ = qdmp_.step(0.0).normalized();
  p_off_ = activation.p - p_raw0_;
  q_off_ = activation.q * q_raw0_.conjugate();
  started_ = true;

  last_.p = p_off_ + p_raw0_;
  last_.q = (q_off_ * q_raw0_).normalized();
}

Pose ProDmpSource::sample(double dt) {
  if (!started_) throw std::logic_error("ProDmpSource::sample before start()");
  if (frozen_) {
    hold_elapsed_ += dt;
    return last_;
  }

  double step_dt = dt;
  bool reached = false;
  if (elapsed_ + step_dt >= end_time_) {
    step_dt = end_time_ - elapsed_;  // land exactly on the end of the trajectory
    reached = true;
  }

  Eigen::Vector3d p_raw = prodmp_.position();
  Eigen::Quaterniond q_raw = qdmp_.orientation();
  if (step_dt > 0.0 || elapsed_ == 0.0) {
    p_raw = prodmp_.step(step_dt);
    q_raw = qdmp_.step(step_dt);
  }
  elapsed_ += step_dt;

  if (reached) {
    if (end_fraction_ >= 1.0) {
      p_raw = prodmp_.goal();
      q_raw = qdmp_.goal();
    }
    frozen_ = true;
    hold_elapsed_ = dt - step_dt;  // the part of dt past the end already counts as hold time
  }

  last_.p = p_off_ + p_raw;
  last_.q = (q_off_ * q_raw).normalized();
  return last_;
}

double ProDmpSource::progress() const { return std::min(1.0, elapsed_ / tau_); }

bool ProDmpSource::twist(Eigen::Vector3d& v, Eigen::Vector3d& w) const {
  if (frozen_ || !started_) {
    v.setZero();
    w.setZero();
    return true;
  }
  // Linear velocity passes through the anchoring unchanged (pure translation); the angular one is
  // rotated by q_off (pure left rotation), see the twist method documented at ros_utils.hpp:221-235.
  v = prodmp_.velocity();
  w = q_off_ * qdmp_.omega();
  return true;
}

}  // namespace real_replay
