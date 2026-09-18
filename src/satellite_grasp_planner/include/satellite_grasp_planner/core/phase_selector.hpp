#pragma once

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "haptic_dmp_learning/core/prodmp.hpp"
#include "franka_cartesian_control/core/robot_model.hpp"
#include "satellite_grasp_planner/core/joint_path_simulator.hpp"

namespace satellite_grasp_planner {
namespace core {

using franka_cartesian_control::core::RobotModel;

/**
 * @brief Kinematic model of the target satellite: pure rotation about a
 *        fixed axis through @c center, at constant angular velocity
 *        @c omega (rad/s, >= 0), at phase @c phase0 (rad) when t = 0.
 *        All vectors are in the SAME frame as JointPathSimulator/RobotModel
 *        (the robot base frame) - no TF/frame handling here, that belongs to
 *        the future ROS wrapper node.
 */
struct SatelliteRotationModel {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();  ///< must be unit-norm
    double phase0 = 0.0;
    double omega = 0.0;  ///< 0 = static target (degenerate case: phase is irrelevant)

    /// @brief World/base-frame position of a point rigidly attached to the
    /// satellite (given in the satellite's own body frame) at time t.
    Eigen::Vector3d transformPointAt(double t, const Eigen::Vector3d& point_body) const;

    /// @brief World/base-frame orientation of a frame rigidly attached to the
    /// satellite (given in the satellite's own body frame) at time t.
    Eigen::Quaterniond transformOrientationAt(double t, const Eigen::Quaterniond& quat_body) const;

    /// @brief Rotation period (s); 0 for the degenerate omega == 0 case.
    double period() const;
};

/**
 * @brief Level 1: picks the approach start time t_start that maximizes
 *        translational manipulability along the ProDMP approach path toward
 *        a target grasp point that is rigidly rotating with the satellite.
 *
 * @details
 * ALGORITHM
 * For a candidate t_start, the predicted grasp pose at ARRIVAL
 * (t_start + tau, tau = the ProDMP's own rollout duration) is computed from
 * the satellite rotation model, and used as a FIXED goal for that candidate
 * approach (Level 1 does not re-track the moving target mid-approach - that
 * refinement is Level 2, out of scope here). A copy of the fitted ProDMP
 * template is retargeted to that goal and rolled out tick by tick; the
 * orientation trajectory (ProDMP models position only) is a constant-rate
 * SLERP from ee_init_orientation to the candidate goal orientation over the
 * same tau, which is exact for a fixed axis/angle rotation and is a
 * deliberately simple Level-1 placeholder - CHOMP-style refinement of the
 * whole trajectory (not just its endpoint) is Level 2. No velocity
 * feedforward is passed to JointPathSimulator (linear/angular = 0): the
 * already-validated Gazebo runs this score is cross-checked against
 * (test_phase_selector.cpp) were themselves recorded with
 * feedforward_enabled_=false (the CartesianVelocityController default,
 * Kp-only tracking) - matching that control law here keeps the offline
 * score consistent with the real baseline it is validated against.
 * JointPathSimulator then
 * then simulates the joint path q(t) the real DLS controller would follow,
 * and the candidate is scored by the LOW (default: 10th) percentile of
 * w_trans(q(t)) along that path - a percentile rather than the bare minimum,
 * for robustness to single noisy/transient samples.
 *
 * COST / COARSE-TO-FINE SEARCH
 * Each candidate costs one full JointPathSimulator::simulate() over the
 * whole rollout (tau/dt Euler+DLS+Pinocchio steps, e.g. ~12200 steps at
 * dt=0.005s for a 61s rollout) - a dense uniform scan of 100+ t_start
 * candidates is O(1-5 minutes), not negligible. select() therefore runs a
 * COARSE pass (Params::coarse_candidates samples spread over one full
 * rotation period) followed by a FINE pass (Params::fine_candidates samples,
 * restricted to a Params::fine_window_fraction-wide window centered on the
 * coarse best) instead of a single dense scan. A wall-clock budget
 * (Params::max_wall_time_sec) bounds the total cost; if it is hit before the
 * fine pass completes, select() stops early, logs loudly (stderr) and
 * returns the best candidate found so far with Result::budget_exceeded=true
 * - never a silent partial result.
 *
 * FAIL-LOUD THRESHOLD
 * If the best score found is below Params::score_threshold, select() logs
 * loudly and sets Result::below_threshold=true: a signal that Level 1 phase
 * selection alone is not enough for this target/grasp geometry (e.g. every
 * phase of the rotation passes through a bad approach direction) and Level 2
 * trajectory refinement - or a different grasp point - is needed. This is
 * never silently swallowed into a "best effort" result.
 */
class PhaseSelector {
public:
    struct Params {
        // --- Candidate search: coarse-to-fine, see class docs ---
        int coarse_candidates = 24;          ///< t_start samples over one full rotation period, coarse pass
        int fine_candidates = 24;            ///< t_start samples in the refinement window, fine pass
        double fine_window_fraction = 0.15;  ///< fraction of the rotation period spanned by the fine pass

        // --- Scoring ---
        double score_percentile = 10.0;  ///< low percentile (0-100) of w_trans along a candidate path used as its score
        double score_threshold = 0.05;   ///< fail-loud floor: best score below this => Level 1 alone is insufficient
        /// @brief Relative score margin threshold between best and runner-up candidates.
        /// If (best_score - runner_up_score) / best_score < indecision_margin_fraction, the distinction
        /// between the top candidates falls within the known kinematic proxy branch uncertainty.
        /// Calibrated against the measured WORST CASE of postural bifurcation (Goal 2: +23.9%) with
        /// a safety margin (default = 0.25 / 25%).
        double indecision_margin_fraction = 0.25;

        /// @brief Expected intrinsic uncertainty fraction on the predicted manipulability score
        /// @brief Expected intrinsic uncertainty fraction on the predicted manipulability score
        /// of the winning candidate due to multi-basin dynamic postural bifurcations.
        /// Calibrated on the worst-case observed divergence across impedance control replicates (Goal 2: +23.9% ~ 0.24).
        double known_branch_uncertainty = 0.24;

        /// @brief Maximum reachable Euclidean distance from robot base frame origin (m).
        /// Candidates whose predicted arrival goal exceeds this distance are pruned immediately
        /// as infeasible without running the expensive joint-path simulation.
        /// Default 0.80m gives a safety margin below the physical Panda limit (~0.855m).
        double max_reach_m = 0.80;

        // --- Budget ---
        double max_wall_time_sec = 120.0;  ///< abort (fail loud, keep best-so-far) if the scan exceeds this

        // --- Underlying joint-path simulation (dt, DLS lambda, ...) ---
        JointPathSimulator::Params sim_params;
    };

    PhaseSelector(std::shared_ptr<RobotModel> robot_model,
                  const haptic_dmp_learning::core::ProDMP& prodmp_template);
    PhaseSelector(std::shared_ptr<RobotModel> robot_model,
                  const haptic_dmp_learning::core::ProDMP& prodmp_template,
                  const Params& params);

    struct Result {
        double t_start = 0.0;
        double score = 0.0;
        double best_score_lower_bound = 0.0;  ///< score * (1.0 - known_branch_uncertainty)
        double runner_up_score = 0.0;
        double score_margin = 0.0;     ///< (score - runner_up_score) / score
        Eigen::Vector3d goal_position = Eigen::Vector3d::Zero();
        Eigen::Quaterniond goal_orientation = Eigen::Quaterniond::Identity();
        int candidates_evaluated = 0;
        int candidates_infeasible = 0;   ///< count of candidates pruned early due to reachability limit (dist > max_reach_m)
        bool budget_exceeded = false;    ///< true if max_wall_time_sec was hit before the scan finished
        bool below_threshold = false;    ///< true if score < Params::score_threshold or all candidates infeasible
        bool indecisive_margin = false;  ///< true if score_margin < Params::indecision_margin_fraction
        std::string message;             ///< human-readable explanation, set whenever a flag above is true
    };

    /**
     * @param rotation Satellite rotation model.
     * @param grasp_pos_body / grasp_quat_body Grasp point pose in the
     *        satellite's body frame.
     * @param q0 Robot joint configuration the approach would start from.
     * @param ee_init_position / ee_init_orientation EE pose at rollout start
     *        (== FK(q0); passed explicitly to avoid a redundant RobotModel
     *        call per candidate, and because in general the caller already
     *        has it from the last robot_model_->update()).
     */
    Result select(const SatelliteRotationModel& rotation, const Eigen::Vector3d& grasp_pos_body,
                  const Eigen::Quaterniond& grasp_quat_body, const RobotModel::JointVector& q0,
                  const Eigen::Vector3d& ee_init_position,
                  const Eigen::Quaterniond& ee_init_orientation) const;

private:
    struct Candidate {
        double t_start = 0.0;
        double score = 0.0;
        bool is_feasible = true;
        Eigen::Vector3d goal_position = Eigen::Vector3d::Zero();
        Eigen::Quaterniond goal_orientation = Eigen::Quaterniond::Identity();
    };

    Candidate scoreCandidate(double t_start, const SatelliteRotationModel& rotation,
                              const Eigen::Vector3d& grasp_pos_body,
                              const Eigen::Quaterniond& grasp_quat_body,
                              const RobotModel::JointVector& q0,
                              const Eigen::Vector3d& ee_init_position,
                              const Eigen::Quaterniond& ee_init_orientation) const;

    std::shared_ptr<RobotModel> robot_model_;
    haptic_dmp_learning::core::ProDMP prodmp_template_;
    JointPathSimulator simulator_;
    Params params_;
};

}  // namespace core
}  // namespace satellite_grasp_planner
