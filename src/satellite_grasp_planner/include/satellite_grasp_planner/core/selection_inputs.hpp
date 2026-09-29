#pragma once

/**
 * @file selection_inputs.hpp
 * @brief Assembles everything selectGrasp() needs (template, robot, scan/selection parameters) from the
 *        demo-parameters file written at fit time and from the current satellite measurement. No
 *        rollout, no search: only reading, validation and cheap kinematics.
 */

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "satellite_grasp_planner/core/grasp_selection.hpp"
#include "satellite_grasp_planner/core/launch_plan.hpp"

namespace satellite_grasp_planner {
namespace core {

struct SelectionInputs {
    /// ProDMP has no default constructor, hence this one (the other members have defaults).
    explicit SelectionInputs(haptic_dmp_learning::core::ProDMP tmpl) : prodmp_template(std::move(tmpl)) {}

    SelectionParams params;
    haptic_dmp_learning::core::CubeSatelliteModel::Params cube_params;
    haptic_dmp_learning::core::ProDMP prodmp_template;  ///< as loaded: no setGoal, no re-anchoring
    std::shared_ptr<franka_cartesian_control::core::RobotModel> robot;  ///< left at q0 (zero velocity)
    franka_cartesian_control::core::RobotModel::JointVector q0 =
        franka_cartesian_control::core::RobotModel::JointVector::Zero();
    Eigen::Vector3d p0 = Eigen::Vector3d::Zero();  ///< EE position at q0, from the robot kinematics
    std::vector<std::string> provenance;           ///< one line per value: value and where it comes from
    bool w_trans_demo_is_proxy = false;            ///< true if w_trans_demo comes from referenceWTrans()
    bool contact_assumed_at_end = false;           ///< true if demo_params had no contact time
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
 *  b) q0 = start_joints_rad if present, else RobotModel::readyPose(); p0 = EE position at q0 (NEVER
 *     start_position_demo_frame_m: that is in the demo frame, not the robot's).
 *  c) cube_params = @p cube_geometry with center_world = snapshot.center and axis_world = snapshot.axis;
 *     theta_center = windowCenterTheta(model, robot_base_world).
 *  d) ScanParams: default window and step, omega = snapshot.omega_rad_s, tau_contact_s =
 *     t_contact_s.value_or(0) (0 = the template's tau, contact assumed at the end), delta_g and
 *     contact_to_end_offset from the demo, default weights, e_v_ref = meanSurfaceSpeedSquared,
 *     e_g_ref = |delta_g|^2. @throws std::invalid_argument if |delta_g| < 1e-9 or tau_contact_s exceeds
 *     the template tau.
 *  e) w_trans_demo = translational manipulability at end_joints_rad if present (product of the
 *     singular values of the 3x7 translational Jacobian block, as tools/compute_w_trans_demo.py);
 *     otherwise the PROVISIONAL proxy referenceWTrans() and w_trans_demo_is_proxy = true.
 *     The robot is put back at q0 afterwards.
 *  h) provenance: one line per value used (tau_contact, delta_g, contact_to_end_offset, q0, p0,
 *     w_trans_demo, omega, satellite center and axis, e_v_ref, e_g_ref).
 */
SelectionInputs buildSelectionInputs(const std::string& demo_params_path, const std::string& urdf_path,
                                     const SatelliteSnapshot& snapshot,
                                     const haptic_dmp_learning::core::CubeSatelliteModel::Params& cube_geometry,
                                     const Eigen::Vector3d& robot_base_world);

}  // namespace core
}  // namespace satellite_grasp_planner
