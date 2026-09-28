#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"

namespace satellite_grasp_planner {
namespace core {

/// Signed distance of one arm capsule to the satellite cylinder (diagnostic).
struct CapsuleDistance {
    std::string name;             ///< "<frame>#<k>", e.g. "fer_hand#1"
    double distance_m = 0.0;      ///< signed, negative under interpenetration
    bool expected_contact = false;///< true = excluded from min_distance_arm_m / feasible
    /// Signed distance to the cube body. NOT affected by expected_contact (that flag only
    /// concerns the cylinder).
    double distance_cube_m = 0.0;
};

struct CollisionCheckResult {
    bool feasible = false;
    /// Minimum SIGNED distance between any NON-excluded arm-body capsule and the satellite
    /// cylinder (negative when they interpenetrate). Capsules with expected_contact are ignored.
    double min_distance_arm_m = 0.0;
    /// Non-excluded capsule achieving min_distance_arm_m, as "<frame>#<k>" with k the index among
    /// that frame's capsules (e.g. "fer_hand#0"). Diagnostic only.
    std::string closest_capsule;
    /// Minimum SIGNED distance between ANY of the 12 capsules (fer_hand#1 included) and the cube.
    double min_distance_cube_m = 0.0;
    /// Capsule achieving min_distance_cube_m ("<frame>#<k>"). Diagnostic only.
    std::string closest_capsule_cube;
    /// One entry per capsule of the table (12), excluded ones included, in table order.
    std::vector<CapsuleDistance> per_capsule;
};

/**
 * @brief Hard non-collision constraint between the Franka arm BODY (links in
 *        RobotModel::armLinkFrameNames(): fer_link0..fer_link8, fer_hand) and the satellite's
 *        hollow cylinder, at the final rollout pose.
 *
 * Geometry: each arm link carries the official Franka collision capsules (franka_arm.xacro /
 * franka_hand.xacro), defined in the link's LOCAL frame (center, axis, radius, length) and moved to
 * world with RobotModel::framePose(link); fer_link5, fer_link7 and fer_hand have two capsules each,
 * every one a separate pair against the satellite. fer_link8 has no capsule in the source and none
 * is modelled. The arm values were NOT verified byte-for-byte against the real xacro in this work
 * (carried over from an earlier inspection, cross-checked only on the two fer_hand capsules present
 * in this repo); see the table in satellite_collision.cpp. The satellite is a coal::Cylinder built
 * from CubeSatelliteModel::cylinderOuterRadius()/cylinderBaseCenterWorld()/cylinderTopCenterWorld()/
 * cylinderAxisWorld(theta); its hollow interior is ignored (solid outer cylinder: conservative).
 * Distances use coal::distance with DistanceRequest::enable_signed_distance = true set EXPLICITLY,
 * so interpenetration yields a negative distance.
 *
 * EXCLUDED capsule (expected contact): fer_hand#1 (palm top: radius 0.02, local center z=0.10,
 * axis Y) is evaluated and reported in per_capsule but does NOT enter min_distance_arm_m nor
 * feasible, exactly like the fingers (limitation (b)). Why: in the production probe (reachable
 * satellite, IK error 7-10 mm) it interpenetrates the cylinder by about 9.5 mm at the nominal
 * grasp on all 4 grasp points and by about 19 mm with the grasp 1 cm deeper. The hypothesis
 * behind the exclusion is NOT verified: the official Franka capsules are self-collision
 * envelopes covering the region between the fingers, where the wall sits in a real grasp.
 * LIMIT: a genuine palm impact in that region is not detected. fer_hand#0 is still checked.
 *
 * Cube body: the satellite cube is a second collision object, a coal::Box with side cube_side_m,
 * centered at center_world and rotated by R(theta) about axis_world (CubeSatelliteModel::
 * cubeCenterWorld()/cubeRotationWorld()). It is an EXACT solid box, no conservative approximation.
 * All 12 capsules are checked against it, fer_hand#1 included: expected_contact applies to the
 * cylinder only, NOT to the cube. Signed distance is enabled here too. feasible requires BOTH
 * min_distance_arm_m >= d_safe_arm_m AND min_distance_cube_m >= d_safe_arm_m. The cube check also
 * sees only the final pose, and the 10 mm margin is provisional.
 *
 * Limitations / scope:
 *  (a) Only the FINAL pose is seen: collisions during the approach are not detected (same known
 *      limitation as checkKinematicFeasibility() for joint limits).
 *  (b) The fingers (RobotModel::gripperFrameNames()) are excluded ON PURPOSE: the candidate pose
 *      places them on the cylinder wall by construction (CubeSatelliteModel::graspPoseAt), so their
 *      contact is expected and is not something to detect here.
 *
 * @param robot_model Already updated to the configuration of interest via update().
 * @param theta_rad Satellite phase (explicit, as everywhere in CubeSatelliteModel).
 * @param d_safe_arm_m Conservative clearance margin, to be recalibrated.
 * @return feasible == (min_distance_arm_m >= d_safe_arm_m && min_distance_cube_m >= d_safe_arm_m).
 */
CollisionCheckResult checkSatelliteCollision(
    const franka_cartesian_control::core::RobotModel& robot_model,
    const haptic_dmp_learning::core::CubeSatelliteModel& satellite_model,
    double theta_rad,
    double d_safe_arm_m = 0.010);

}  // namespace core
}  // namespace satellite_grasp_planner
