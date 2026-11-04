#pragma once

// Target generators. A source returns targets already expressed in the robot base frame, anchored
// to the activation pose handed to start(). Implemented here: HoldSource, StepSource. The ProDMP
// replay lives in prodmp_source.hpp.

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "real_replay/config.hpp"

namespace real_replay {

struct Pose {
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};

class ITargetSource {
 public:
  virtual ~ITargetSource() = default;

  /// Called once, in the first control cycle, with the pose of the control frame (FK of the
  /// measured q). Resets the source's internal time to zero.
  virtual void start(const Pose& activation) = 0;

  /// Advances the source by `dt` seconds of trajectory time (the sum of the periods reported by
  /// libfranka, not a cycle count) and returns the target. The very first call after start()
  /// uses dt = 0 and must return the activation pose.
  virtual Pose sample(double dt) = 0;

  /// Fraction of the trajectory already played, in [0, 1]. Informational.
  virtual double progress() const = 0;

  /// True when the source is over (including any final hold); the loop then performs the
  /// controlled stop.
  virtual bool done() const = 0;

  /// Nominal length in trajectory seconds, including any final hold, excluding the controlled stop.
  virtual double nominalDurationS() const = 0;

  /// Feedforward twist of the LAST sample in the base frame (used only by --check-joint-limits).
  /// Returns false when the source has none.
  virtual bool twist(Eigen::Vector3d& v, Eigen::Vector3d& w) const {
    v.setZero();
    w.setZero();
    return false;
  }
};

/// Holds the activation pose for `duration_s` seconds of trajectory time.
class HoldSource final : public ITargetSource {
 public:
  explicit HoldSource(double duration_s);

  void start(const Pose& activation) override;
  Pose sample(double dt) override;
  double progress() const override;
  bool done() const override { return elapsed_ >= duration_; }
  double nominalDurationS() const override { return duration_; }
  bool twist(Eigen::Vector3d& v, Eigen::Vector3d& w) const override {
    v.setZero();
    w.setZero();
    return true;
  }

 private:
  double duration_;
  double elapsed_ = 0.0;
  Pose pose_;
};

/// Small Cartesian translation along a base-frame axis, smoothstep profile, orientation held:
///   [0, T_move]                 0 -> A
///   [T_move, T_move + T_dwell]  A
///   then, if return_to_start:   A -> 0 over T_return
/// Always starts from the activation pose. If return_to_start is false the source ends at A.
class StepSource final : public ITargetSource {
 public:
  /// rotational=false: translation along cfg.axis by cfg.amplitude_m (orientation held).
  /// rotational=true : rotation about cfg.rot_axis (BASE frame) by cfg.rot_angle_rad, position held:
  ///                   q_des(t) = Exp(theta(t) * axis) * q_act.
  explicit StepSource(const StepConfig& cfg, bool rotational = false);

  void start(const Pose& activation) override;
  Pose sample(double dt) override;
  double progress() const override;
  bool done() const override { return elapsed_ >= total_; }
  double nominalDurationS() const override { return total_; }
  bool twist(Eigen::Vector3d& v, Eigen::Vector3d& w) const override;

  /// Displacement fraction f(t) in [0, 1] and its time derivative, at trajectory time t.
  void profile(double t, double& f, double& df) const;

 private:
  StepConfig cfg_;
  bool rotational_ = false;
  Eigen::Vector3d axis_;
  double total_;
  double elapsed_ = 0.0;
  Pose act_;
};

}  // namespace real_replay
