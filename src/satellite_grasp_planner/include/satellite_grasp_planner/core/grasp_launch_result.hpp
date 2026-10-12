#pragma once

/**
 * @file grasp_launch_result.hpp
 * @brief Output of the grasp-selection + launch-planning entry point (planGraspAndLaunchS1()): one struct
 *        that never exposes a goal without an explicit "selected" status (no silent zero goal).
 */

#include <optional>

#include <Eigen/Dense>

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
    bool wrist_flip = false;           ///< goal_orientation is the nominal one rolled by pi about z_tcp
    double cost = 0.0;                 ///< d + epsilon_wait_per_s * wait_s
    double d_pos_m = 0.0;
    double w_hat = 0.0;
    double min_joint_margin_rad = 0.0; ///< min joint-limit margin of the IK solution
    double posthoc_dv_m_s = 0.0;       ///< |v_target - v_ee| at contact (0 if the velocity check is off)

    /// Always populated, whatever the status.
    int evaluations = 0;               ///< evaluateGraspPose calls
    double elapsed_s = 0.0;            ///< wall-clock time of the selection

    /// Present iff status == kSelected. PROVISIONAL: re-run planLaunch() on a freshly re-read
    /// SatelliteSnapshot right before the actual launch; this one is only a preview computed at
    /// selection time.
    std::optional<LaunchPlan> launch;

    /// The tau actually passed to planLaunch(); 0.0 when status != kSelected (no launch was planned).
    double tau_launch_s = 0.0;
};

}  // namespace core
}  // namespace satellite_grasp_planner
