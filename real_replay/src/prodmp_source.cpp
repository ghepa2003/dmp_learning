#include "real_replay/prodmp_source.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "haptic_dmp_learning/core/grasp_orientation.hpp"
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

void ProDmpSource::setGoal(const Eigen::Vector3d& p_goal, const Eigen::Quaterniond& q_goal) {
  if (!p_goal.allFinite() || !q_goal.coeffs().allFinite() || !(q_goal.norm() > 1e-9)) {
    throw std::invalid_argument("ProDmpSource::setGoal: non-finite goal or zero quaternion");
  }
  if (started_) throw std::logic_error("ProDmpSource::setGoal after start()");
  p_goal_ = p_goal;
  q_goal_ = q_goal.normalized();
  goal_set_ = true;
}

void ProDmpSource::start(const Pose& activation) {
  elapsed_ = 0.0;
  hold_elapsed_ = 0.0;
  frozen_ = false;

  if (goal_set_) {
    // Executor chain, grasp_command mode (prodmp_gazebo_executor_node.cpp:1523-1541).
    namespace go = haptic_dmp_learning::core::grasp_orientation;
    prodmp_.setRelativeGoal(true);
    prodmp_.setInitialConditions(/*init_time=*/0.0, activation.p, Eigen::Vector3d::Zero());
    prodmp_.setGoal(p_goal_);
    qdmp_.reset();  // not used for the target in goal mode (as the executor with planner_goal)
    q_start_ = activation.q.normalized();
    q_goal_aligned_ = go::alignShortestPath(q_start_, q_goal_);
    w_slerp_ = go::slerpAngularVelocityWorld(q_start_, q_goal_aligned_, tau_);
    p_raw0_ = prodmp_.step(0.0);
    q_raw0_ = q_start_;
    // Anchored by the initial condition: the rigid offsets must be the identity.
    if ((p_raw0_ - activation.p).norm() > 1e-9) {
      throw std::logic_error("ProDmpSource goal mode: step(0) != activation position");
    }
    p_off_.setZero();
    q_off_.setIdentity();
    started_ = true;
    last_.p = activation.p;
    last_.q = q_start_;
    return;
  }

  // Rewind exactly as the executor does before a rollout (executor.cpp:664-665).
  prodmp_.setInitialConditions(0.0, prodmp_.initPos(), prodmp_.initVel());
  qdmp_.reset();

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

  if (goal_set_) {
    namespace go = haptic_dmp_learning::core::grasp_orientation;
    double step_dt = dt;
    bool reached = false;
    if (elapsed_ + step_dt >= end_time_) {
      step_dt = end_time_ - elapsed_;
      reached = true;
    }
    Eigen::Vector3d p = prodmp_.position();
    if (step_dt > 0.0 || elapsed_ == 0.0) p = prodmp_.step(step_dt);
    elapsed_ += step_dt;
    Eigen::Quaterniond q = go::slerpAt(q_start_, q_goal_aligned_, elapsed_, tau_);
    if (reached) {
      if (end_fraction_ >= 1.0) {  // exactly the commanded goal, no rounding of goal_param + init_pos
        p = p_goal_;
        q = q_goal_aligned_;
      }
      frozen_ = true;
      hold_elapsed_ = dt - step_dt;
    }
    last_.p = p;
    last_.q = q;
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
  if (goal_set_) {
    v = prodmp_.velocity();
    w = w_slerp_;
    return true;
  }
  // Linear velocity passes through the anchoring unchanged (pure translation); the angular one is
  // rotated by q_off (pure left rotation), see the twist method documented at ros_utils.hpp:221-235.
  v = prodmp_.velocity();
  w = q_off_ * qdmp_.omega();
  return true;
}

}  // namespace real_replay
