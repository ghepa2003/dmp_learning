#pragma once

// ProDMP (position) + QuaternionDMP (orientation) replay, anchored to the activation pose.
//
// Same maths as the simulation chain for target_odom_required:=false
// (prodmp_gazebo_executor_node.cpp:664-665 for the rollout, ros_utils.hpp FrameAligner::align()
// lines 195-196 and 211-212 for the anchoring), done here without ROS:
//
//   p_raw0, q_raw0 = step(0.0) of the freshly reset models
//   p_off = p_act - p_raw0                 q_off = q_act * q_raw0^-1
//   target(t) = ( p_off + p_raw(t),  normalize(q_off * q_raw(t)) )
//
// Orientation anchoring (decided, do not change): q_off * q_raw(t) = q_act * q_raw0^-1 * q_raw(t),
// i.e. the orientation increment is applied in the BODY frame, exactly as FrameAligner / the Gazebo
// chain. Writing it as q_act * (q_raw0^-1 * q_raw(t)) is the SAME expression, not an alternative.
// The genuinely different variant q_raw(t) * q_raw0^-1 * q_act (increment applied in the world
// frame) must NOT be used.
//
// Time is the sum of the dt handed to sample() (the periods reported by libfranka), never a cycle
// count. At fraction stop_at_fraction of tau the target stays frozen on the last value for
// hold_after_end_s, then done() becomes true. At fraction 1 the final sample is clamped to the
// attractor (goal position / goal quaternion), as the executor does (executor.cpp:872-882).
//
// GOAL MODE (setGoal() before start(), task layer): same call chain as the simulation executor in
// grasp_command mode (prodmp_gazebo_executor_node.cpp startContinuousRollout(), lines 1523-1541):
//   prodmp.setRelativeGoal(true); prodmp.setInitialConditions(0, p_act, 0); prodmp.setGoal(p_goal);
//   orientation = SLERP (grasp_orientation_mode = planner_goal):
//     q(t) = slerpAt(q_act, alignShortestPath(q_act, q_goal), t, tau),  s = min(1, t/tau)
//     w    = slerpAngularVelocityWorld(q_act, q_goal_aligned, tau)  (constant)
// The ProDMP is anchored by its own initial condition at the activation pose, so the rigid offsets of
// the default mode are the identity here (p_off = 0, q_off = I: checked, not assumed). At t = tau the
// target is EXACTLY (p_goal, q_goal_aligned) with zero twist. Without setGoal() nothing changes.

#include <string>

#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/quaternion_dmp.hpp"
#include "real_replay/target_source.hpp"

namespace real_replay {

class ProDmpSource final : public ITargetSource {
 public:
  /// Loads the weights (orientation must be inline in the quaternion_dmp section), computes their
  /// SHA-256 and, if `expected_sha256` is not empty, throws on mismatch. Throws on any load error.
  ProDmpSource(const std::string& weights_path, const std::string& expected_sha256,
               double stop_at_fraction, double hold_after_end_s);

  /// Goal mode: must be called BEFORE start(). Position in the robot base frame, quaternion normalised here.
  /// Throws std::invalid_argument on non-finite input or a zero quaternion.
  void setGoal(const Eigen::Vector3d& p_goal, const Eigen::Quaterniond& q_goal);
  bool hasGoal() const { return goal_set_; }

  void start(const Pose& activation) override;
  Pose sample(double dt) override;
  double progress() const override;
  bool done() const override { return frozen_ && hold_elapsed_ >= hold_after_end_s_; }
  double nominalDurationS() const override { return end_time_ + hold_after_end_s_; }
  bool twist(Eigen::Vector3d& v, Eigen::Vector3d& w) const override;

  const std::string& sha256() const { return sha256_; }
  double tau() const { return tau_; }
  double endTime() const { return end_time_; }
  const Eigen::Vector3d& rawPosition0() const { return p_raw0_; }
  const Eigen::Quaterniond& rawOrientation0() const { return q_raw0_; }
  const Eigen::Vector3d& positionOffset() const { return p_off_; }
  const Eigen::Quaterniond& orientationOffset() const { return q_off_; }
  /// Goal mode: the goal quaternion after alignShortestPath() with the activation quaternion (valid after start()).
  const Eigen::Quaterniond& alignedGoalOrientation() const { return q_goal_aligned_; }
  /// Trajectory time consumed so far [s].
  double elapsed() const { return elapsed_; }

 private:
  haptic_dmp_learning::core::ProDMP prodmp_;
  haptic_dmp_learning::core::QuaternionDMP qdmp_;
  std::string sha256_;
  double tau_ = 0.0;
  double end_fraction_ = 1.0;
  double end_time_ = 0.0;
  double hold_after_end_s_ = 0.0;

  bool started_ = false;
  bool frozen_ = false;
  double elapsed_ = 0.0;       ///< trajectory time consumed by the models [s]
  double hold_elapsed_ = 0.0;  ///< time spent frozen [s]

  Eigen::Vector3d p_raw0_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_raw0_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p_off_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_off_ = Eigen::Quaterniond::Identity();
  Pose last_;

  // goal mode
  bool goal_set_ = false;
  Eigen::Vector3d p_goal_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_goal_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond q_start_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond q_goal_aligned_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d w_slerp_ = Eigen::Vector3d::Zero();
};

}  // namespace real_replay
