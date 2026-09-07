#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace haptic_dmp_learning {
namespace core {
namespace frame_correction {

// ---------------------------------------------------------------------------
// Fixed Geomagic -> Franka base-frame correction.
//
// The Geomagic Touch base frame (omni_base) is mounted rotated by +90 deg
// about its own z axis relative to the Franka base frame (fer_link0), purely
// because of how the two devices are placed on the bench. Every raw master
// pose the driver emits is therefore expressed in omni_base and must be
// re-expressed in fer_link0 before ANY downstream processing (the quaternion
// sign-continuity fix, the DMP sample buffer, the live /target_pose mirror,
// and further down FrameAligner in franka_cartesian_control).
//
// This is a pure change of coordinates by a fixed rotation R_z(+90 deg), so it
// applies identically to a position vector (rotation product) and to an
// orientation quaternion (LEFT composition, not conjugation):
//     p_franka = R_z(+90 deg) * p_geomagic
//     q_franka = q_offset (x) q_geomagic
// Because the rotation is linear it commutes with the subtraction FrameAligner
// performs to obtain position deltas, so rotating the absolute raw sample (as
// the consumer nodes do) is equivalent to rotating the delta. FrameAligner
// itself needs no change.
//
// HARDWARE VERIFICATION STATUS - read before touching the angle/axis below:
//   * x axis: VERIFIED experimentally. Moving the Geomagic along its own x
//     (lateral) axis moves the robot along its own x (frontal) axis, i.e. the
//     Geomagic x axis must map onto the Franka y axis. R_z(+90 deg) does
//     exactly this: [1,0,0] -> [0,1,0].
//   * y and z axes: ASSUMED, NOT yet verified on hardware. They follow only
//     from the assumption that the mounting offset is a *pure* +90 deg
//     rotation about z:
//         Geomagic y -> Franka -x   ([0,1,0] -> [-1,0,0])
//         Geomagic z -> Franka  z   ([0,0,1] -> [0,0,1], unchanged)
//     If a hardware check on y/z contradicts this, THIS FILE is the single
//     place to fix it: adjust kFrameOffsetAngleRad and/or kFrameOffsetAxis
//     below and nothing else in the pipeline changes.
// ---------------------------------------------------------------------------

/// Mounting offset: +90 degrees about the Geomagic base z axis. Single source
/// of truth for the whole correction (see the verification note above).
inline constexpr double kFrameOffsetAngleRad = M_PI / 2.0;

/// Rotation axis of the mounting offset, in the Geomagic base frame.
inline const Eigen::Vector3d& frameOffsetAxis() {
    static const Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    return axis;
}

/// Quaternion of the fixed omni_base -> fer_link0 rotation R_z(+90 deg).
/// Derived once from kFrameOffsetAngleRad / frameOffsetAxis(); every other
/// function in this file is expressed through it.
inline const Eigen::Quaterniond& frameOffsetQuaternion() {
    static const Eigen::Quaterniond q_offset(
        Eigen::AngleAxisd(kFrameOffsetAngleRad, frameOffsetAxis()));
    return q_offset;
}

/// Re-express a position given in the Geomagic base frame (omni_base) in the
/// Franka base frame (fer_link0).
inline Eigen::Vector3d rotatePosition(const Eigen::Vector3d& p_geomagic) {
    return frameOffsetQuaternion() * p_geomagic;
}

/// Re-express an orientation given in the Geomagic base frame (omni_base) in
/// the Franka base frame (fer_link0). Left composition q_offset (x) q_geomagic
/// (not q_offset (x) q (x) q_offset^-1): this restates the frame the
/// orientation is written in, it does not rotate a vector by it.
inline Eigen::Quaterniond rotateOrientation(const Eigen::Quaterniond& q_geomagic) {
    return frameOffsetQuaternion() * q_geomagic;
}

}  // namespace frame_correction
}  // namespace core
}  // namespace haptic_dmp_learning
