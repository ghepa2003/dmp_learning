#pragma once

// Pure (robot-free) preflight helpers, unit-tested with synthetic data.

#include <string>
#include <vector>

#include "real_replay/config.hpp"

namespace real_replay {

/// Gravity torque of franka::Model (current Desk setting) against the measured joint torques tau_J,
/// robot at rest. Checks the robot/Desk side, independently of the URDF.
struct MeasuredGravityResult {
  enum class Verdict { kPass, kFail, kWarnMoving };
  Verdict verdict = Verdict::kPass;
  double max_abs_diff = 0.0;  ///< max_i |g_franka_i - tau_J_i| [Nm]
  int worst_joint = 0;        ///< 0-based
  double max_abs_dq = 0.0;    ///< [rad/s]
  bool moving = false;        ///< max|dq| above the still threshold: the comparison is not valid
};

MeasuredGravityResult checkGravityVsMeasured(const Array7& g_franka, const Array7& tau_j, const Array7& dq,
                                             double tol_nm, double still_max_dq_rad_s);

/// franka::Model against Pinocchio on the URDF: informative (WARN only).
struct UrdfGravityResult {
  bool warn = false;
  double max_abs_diff = 0.0;
  int worst_joint = 0;
};

UrdfGravityResult checkGravityVsUrdf(const Array7& g_franka, const Array7& g_urdf, double warn_nm);

/// Returns `urdf_xml` with the mass of the named links multiplied by `factor` and their inertia
/// tensor (ixx..izz) multiplied by the same factor (same shape, density scaled). Throws if a link
/// or its <mass>/<inertia> element is missing.
std::string scaleLinkMasses(const std::string& urdf_xml, const std::vector<std::string>& links, double factor);

}  // namespace real_replay
