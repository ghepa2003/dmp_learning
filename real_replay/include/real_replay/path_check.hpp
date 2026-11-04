#pragma once

// Optional offline check of the joint path along a target source, using the existing, ROS-free
// satellite_grasp_planner::core::JointPathSimulator (explicit-Euler integration of a
// nullspace-biased damped-least-squares resolution: the planner's proxy of the impedance
// controller). It is a KINEMATIC approximation: it does not model the dynamic response of the
// real torque loop.

#include <string>

#include "real_replay/config.hpp"
#include "real_replay/target_source.hpp"

namespace real_replay {

struct PathCheckResult {
  bool ok = false;             ///< every joint stays at least `margin` away from its limits
  std::size_t samples = 0;
  double duration_s = 0.0;
  double margin_rad = 0.0;     ///< margin used
  double min_w_trans = 0.0;    ///< minimum translational manipulability along the path
  Array7 min_margin_rad{};     ///< per joint: smallest distance to the nearest limit
  Array7 q_end{};              ///< configuration after the last step
  Array6 lambda_act{};         ///< diag of the reflected Cartesian inertia at the first sample [x y z rx ry rz]
  Array6 lambda_min{};         ///< per-axis minimum of diag(Lambda) along the simulated joint path
  Array6 lambda_max{};         ///< per-axis maximum
  int worst_joint = -1;        ///< joint with the smallest margin (0-based)
};

/// Consumes `src` (it is started and sampled to the end): pass a separate instance from the one
/// used by the control loop. `q_act` is the activation configuration.
PathCheckResult checkJointPath(const Config& cfg, const std::string& urdf_xml, ITargetSource& src,
                               const Array7& q_act);

}  // namespace real_replay
