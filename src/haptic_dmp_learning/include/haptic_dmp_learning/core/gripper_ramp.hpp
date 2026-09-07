#pragma once

namespace haptic_dmp_learning {
namespace core {
namespace gripper_ramp {

/**
 * @brief Linearly interpolate a gripper position command along a timed ramp.
 *
 * Shared by demo_replay_sync_orchestrator_node (SPACE-triggered close during
 * live teleoperation) and dmp_gazebo_executor_node (CSV-triggered close during
 * autonomous DMP replay), so the ramp shape is defined in exactly one place.
 *
 * A single step command drives the fingers into the object at full speed and
 * bounces / ejects it; ramping the position setpoint over @p ramp_duration_sec
 * keeps the closing motion slow.
 *
 * Behaviour:
 *   - elapsed_since_start <= 0                -> open_position   (no backward
 *                                               extrapolation past the start)
 *   - elapsed_since_start == ramp_duration_sec-> closed_position (exact)
 *   - 0 < elapsed < ramp_duration_sec         -> linear blend
 *   - elapsed_since_start >= ramp_duration_sec-> closed_position (clamped,
 *                                               never overshot)
 *   - ramp_duration_sec <= 0                  -> closed_position for any
 *                                               elapsed > 0 (degenerate ramp)
 *
 * @note Direction-agnostic: the maths never assumes closed_position <
 *       open_position. It behaves identically whether "closed" is the smaller
 *       value (here 0.0 vs 0.06 open) or the larger one - both call sites rely
 *       on this.
 *
 * @param elapsed_since_start Seconds elapsed since the ramp began.
 * @param ramp_duration_sec   Total ramp length in seconds.
 * @param open_position       Command value at the start of the ramp.
 * @param closed_position     Command value at and after the end of the ramp.
 * @return Interpolated command, always between open_position and closed_position.
 */
inline double interpolateGripperPosition(double elapsed_since_start,
                                         double ramp_duration_sec,
                                         double open_position,
                                         double closed_position) {
    if (elapsed_since_start <= 0.0) {
        return open_position;
    }
    if (!(ramp_duration_sec > 0.0) || elapsed_since_start >= ramp_duration_sec) {
        return closed_position;
    }
    const double frac = elapsed_since_start / ramp_duration_sec;  // in (0, 1)
    return open_position + frac * (closed_position - open_position);
}

}  // namespace gripper_ramp
}  // namespace core
}  // namespace haptic_dmp_learning
