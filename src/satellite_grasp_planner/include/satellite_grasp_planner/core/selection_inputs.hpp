#pragma once

/**
 * @file selection_inputs.hpp
 * @brief Assembles everything the S1 selection (planGraspAndLaunchS1) needs - ProDMP template, robot, start
 *        state, demo data - from the demo-parameters file written at fit time and from the current satellite
 *        measurement. No rollout, no search: only reading, validation and cheap kinematics.
 */

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "satellite_grasp_planner/core/launch_plan.hpp"
#include "satellite_grasp_planner/core/start_state.hpp"

namespace satellite_grasp_planner {
namespace core {

struct SelectionInputs {
    /// ProDMP has no default constructor, hence this one (the other members have defaults).
    explicit SelectionInputs(haptic_dmp_learning::core::ProDMP tmpl) : prodmp_template(std::move(tmpl)) {}

    haptic_dmp_learning::core::CubeSatelliteModel::Params cube_params;
    haptic_dmp_learning::core::ProDMP prodmp_template;  ///< as loaded: no setGoal, no re-anchoring
    std::shared_ptr<franka_cartesian_control::core::RobotModel> robot;  ///< left at q0 (zero velocity)
    franka_cartesian_control::core::RobotModel::JointVector q0 =
        franka_cartesian_control::core::RobotModel::JointVector::Zero();  ///< start joints
    Eigen::Vector3d p0 = Eigen::Vector3d::Zero();  ///< EE position at q0, from the robot kinematics
    /// Where q0 comes from: "start_joints_override_rad parameter", "demo_params.demo.start_joints_rad" or
    /// "RobotModel::readyPose()". Empty in a SelectionInputs built by hand.
    std::string q0_source;
    Eigen::Vector3d delta_g_demo = Eigen::Vector3d::Zero();           ///< demo displacement (end - start)
    Eigen::Vector3d contact_to_end_offset = Eigen::Vector3d::Zero();  ///< goal minus contact point, from the demo
    double tau_contact_s = 0.0;  ///< demo contact time; 0 = contact assumed at the end (the template's tau)
    double omega_rad_s = 0.0;    ///< signed, = snapshot.omega_rad_s
    double w_trans_demo = 0.0;   ///< translational manipulability at the demo's end_joints_rad
    SatelliteSnapshot snapshot;  ///< the satellite measurement the inputs were built for
    std::vector<std::string> provenance;  ///< one line per value: value and where it comes from
    bool contact_assumed_at_end = false;  ///< true if demo_params had no contact time
    /// SHA-256 of the weights file the template was loaded from. Equals demo_params.weights_sha256, and
    /// buildSelectionInputs throws if that differs from the file's actual hash, so it is the hash of the
    /// file really read.
    std::string weights_sha256;
};

/**
 * Builds the selection inputs.
 *  a) demo_params::read + verifyWeightsAlignment on @p demo_params_path (errors are rethrown as
 *     std::runtime_error whose message names that file); the template is loaded from the weights file
 *     resolved relative to the folder of @p demo_params_path.
 *  b) q0 = @p start->q if @p start is given (validated against the URDF joint limits; it wins over the demo
 *     file's start_joints_rad, which is then reported as ignored in the provenance), else the demo file's
 *     start_joints_rad if present, else RobotModel::readyPose(); p0 = EE position at q0 (NEVER
 *     start_position_demo_frame_m: that is in the demo frame, not the robot's).
 *  c) cube_params = @p cube_geometry with center_world = snapshot.center and axis_world = snapshot.axis.
 *  d) omega = snapshot.omega_rad_s, tau_contact_s = t_contact_s.value_or(0) (0 = the template's tau, contact
 *     assumed at the end), delta_g_demo and contact_to_end_offset from the demo.
 *     @throws std::invalid_argument if |delta_g| < 1e-9 or tau_contact_s exceeds the template tau.
 *  e) w_trans_demo = translational manipulability at end_joints_rad (product of the singular values of the
 *     3x7 translational Jacobian block, as tools/compute_w_trans_demo.py). @throws std::runtime_error if the
 *     demo_params has no end_joints_rad: the fit must be redone with the recorded joint states.
 *     The robot is put back at q0 afterwards.
 *  f) provenance: one line per value used (tau_contact, delta_g, contact_to_end_offset, q0, p0,
 *     w_trans_demo, omega, satellite center and axis).
 */
SelectionInputs buildSelectionInputs(const std::string& demo_params_path, const std::string& urdf_path,
                                     const SatelliteSnapshot& snapshot,
                                     const haptic_dmp_learning::core::CubeSatelliteModel::Params& cube_geometry,
                                     const std::optional<StartState>& start = std::nullopt);

/// Multi-line log text for the start of a selection: source of q0, q0, p0, distance from readyPose (joints:
/// norm and max, task space: mm) and the complete provenance. Leaves in.robot updated at in.q0.
std::string describeStartState(const SelectionInputs& in);

}  // namespace core
}  // namespace satellite_grasp_planner
