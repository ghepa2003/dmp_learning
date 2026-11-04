#pragma once

/**
 * @file grasp_launch_result.hpp
 * @brief Output of the grasp-selection + launch-planning entry point (planGraspAndLaunchS1()): one struct
 *        that never exposes a goal without an explicit "selected" status (no silent zero goal).
 */

#include <optional>

#include <Eigen/Dense>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "satellite_grasp_planner/core/launch_plan.hpp"

namespace satellite_grasp_planner {
namespace core {

struct GraspLaunchResult {
  enum class Status {
    kSelected,             ///< a candidate passed the constraints and the post-hoc checks; the optionals below are set.
    kNoFeasibleCandidate,  ///< no candidate satisfies the a-priori constraints (S3Result::kNoCandidate).
    kPosthocFailed,        ///< constraint-passing candidates exist, all failed the post-hoc checks.
  };
  Status status = Status::kNoFeasibleCandidate;

  /// Set only when status == kSelected.
  std::optional<Eigen::Vector3d> goal_position;
  std::optional<Eigen::Quaterniond> goal_orientation;
  std::optional<haptic_dmp_learning::core::GraspPointId> k;
  std::optional<double> theta_star_rad;

  /// Of the selected candidate (zero / false when status != kSelected).
  bool wrist_flip = false;  ///< goal_orientation is the nominal one rolled by pi about z_tcp
  double psi_rad = 0.0;     ///< roll about z_tcp already included in goal_orientation (not in GraspCommand)
  double cost = 0.0;        ///< S3Candidate::cost of the chosen candidate: the J of its rollout (d_mm + kappa_w*(-ln w_rollout) + kappa_t*wait), to be MINIMIZED
  double d_pos_m = 0.0;
  double w_hat = 0.0;
  double min_joint_margin_rad = 0.0;  ///< min joint-limit margin of the IK solution
  double posthoc_dv_m_s = 0.0;        ///< |v_target - v_ee| at contact; NaN if the velocity check is off (not measured)

  /// Always populated, whatever the status.
  int evaluations = 0;     ///< evaluateGraspPose calls
  double elapsed_s = 0.0;  ///< wall-clock time of the selection

  /// Present iff status == kSelected. PROVISIONAL: re-run planLaunch() on a freshly re-read
  /// SatelliteSnapshot right before the actual launch; this one is only a preview computed at
  /// selection time.
  std::optional<LaunchPlan> launch;

  /// The tau actually passed to planLaunch(); 0.0 when status != kSelected (no launch was planned).
  double tau_launch_s = 0.0;

  /// Additive hold-end outputs (populated only when s1.use_hold_end is true and status == kSelected).
  bool hold_end_done = false;
  bool end_ok = false;
  std::optional<franka_cartesian_control::core::RobotModel::JointVector> q_hold_end;
  std::optional<Eigen::Vector3d> hold_position;
  std::optional<Eigen::Quaterniond> hold_orientation;
  double w_end = 0.0;
  double w_eff = 0.0;

  /// Per-stage wall-clock times of the selection [ms], always populated whatever the status (0 for a stage that did not run).
  /// Diagnostic only. In the lazy flow the stages are interleaved per finalist: each value is the SUM of that stage over all the
  /// finalists examined. Copied from S3Result (scan, clearance, dynamic, posthoc) or summed over S3Result::v2_finalists (rollout =
  /// V1, hold_end = V1').
  struct StageTimesMs {
    double scan = 0.0;       ///< candidate scan + static evaluation + J_static sort
    double rollout = 0.0;    ///< V1 kinematic rollouts
    double clearance = 0.0;  ///< V2 clearance
    double hold_end = 0.0;   ///< V1' hold-end IK
    double dynamic = 0.0;    ///< L2 (0 without use_dynamic_follow)
    double posthoc = 0.0;    ///< post-hoc velocity stage
    int finalists = 0;       ///< finalists in S3Result::v2_finalists
  };
  StageTimesMs stage_ms;
};

}  // namespace core
}  // namespace satellite_grasp_planner
