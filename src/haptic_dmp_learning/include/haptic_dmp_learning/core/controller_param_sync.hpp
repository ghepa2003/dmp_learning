#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Dense>

namespace haptic_dmp_learning {
namespace core {
namespace controller_param_sync {

/// Name of the cartesian_impedance_controller ROS 2 parameter (double[3], NaN sentinel = "not
/// set") that FrameAligner uses to compute its alignment offset from an EXPECTED initial raw
/// position instead of deducing it from the first /target_pose sample the controller's
/// single-slot RealtimeBuffer happens to read - see ros_utils.hpp::FrameAligner in
/// franka_cartesian_control for the full rationale (200 Hz publisher / 1 kHz RT consumer
/// mismatch, ~4.5 mm spurious offset observed). Single source of truth for the node side so the
/// string literal is not duplicated at each call site.
inline constexpr const char* kInitialAlignmentPositionOverrideParamName =
    "initial_alignment_position_override";

/// Name of the cartesian_impedance_controller ROS 2 parameter (bool, default false) that forces
/// FrameAligner::setIdentityAlignment() - a pure pass-through, bypassing BOTH the first-raw_pos
/// deduction and kInitialAlignmentPositionOverrideParamName. For branches where world and the
/// robot base frame are known to coincide exactly (anchorAndRotate() already publishes in
/// absolute world coordinates), even the override above has a small residual (~1.89 mm observed)
/// from sampling activation_ee_position_ and ee_now at two different times - identity removes it
/// entirely. Set TOGETHER with kInitialAlignmentPositionOverrideParamName (not instead of it): if
/// this parameter is ever left unset by mistake, the controller falls back to the override-based
/// offset (Fix 2, ~1.89 mm residual) instead of the original first-raw_pos race (~5.70 mm) - see
/// FrameAligner::setIdentityAlignment() in ros_utils.hpp for the full precedence rule.
inline constexpr const char* kSkipInitialAlignmentParamName = "skip_initial_alignment";

/// Pure logic (no ROS) of what the node sends as the override value: same ee_now already used to
/// anchor prodmp_.setInitialConditions(), just repackaged as the flat double[3] the parameter
/// expects.
inline std::vector<double> toAlignmentOverrideValue(const Eigen::Vector3d& ee_now) {
    return {ee_now.x(), ee_now.y(), ee_now.z()};
}

/// True iff `value` is a value the controller would actually apply as an override (exactly 3
/// components, none NaN) - mirrors the controller-side all-or-nothing check in
/// CartesianImpedanceController::applyInitialAlignmentPositionOverride(). Used by the node as a
/// defensive fail-loud guard before sending: ee_now comes from a TF lookup that should never be
/// NaN, but silently forwarding a bogus value that the controller would treat as "not set" would
/// defeat this whole fix without any error - better to fail loud here.
inline bool isValidAlignmentOverrideValue(const std::vector<double>& value) {
    if (value.size() != 3) return false;
    for (double v : value) {
        if (std::isnan(v)) return false;
    }
    return true;
}

}  // namespace controller_param_sync
}  // namespace core
}  // namespace haptic_dmp_learning
