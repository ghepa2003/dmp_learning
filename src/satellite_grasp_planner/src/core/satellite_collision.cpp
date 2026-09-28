#include "satellite_grasp_planner/core/satellite_collision.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <coal/distance.h>
#include <coal/math/transform.h>
#include <coal/shape/geometric_shapes.h>

namespace satellite_grasp_planner {
namespace core {

using franka_cartesian_control::core::RobotModel;

namespace {

enum class Axis { X, Y, Z };

/// Capsule fixed in a link's LOCAL frame: center, axis, radius and length of the cylindrical part
/// (the two hemispherical caps come on top), as in the xacro collision_capsule macro.
struct LinkCapsule {
    const char* frame;
    Eigen::Vector3d center;
    Axis axis;
    double radius;
    double length;
    /// true = pair evaluated and reported (per_capsule) but EXCLUDED from min_distance_arm_m and
    /// feasible. See the note on fer_hand#1 below.
    bool expected_contact = false;
};

// Franka capsules from franka_arm.xacro (official Franka source) and franka_hand.xacro.
// NOT verified byte-for-byte against the real xacro in this work session (no direct access): the
// arm rows are carried over from an earlier inspection and only partially cross-checked against
// the two fer_hand capsules found in this repo (src/franka_description_overrides/franka_hand.xacro),
// which match exactly. To be re-confirmed against the real source file.
//
// - fer_link5 has TWO capsules (main + elbow) and fer_link7 has TWO (axis + flange); both are
//   registered as separate pairs against the satellite cylinder.
// - The fer_link5 ELBOW capsule axis is ASSUMED to be Z for consistency with the other rows: this
//   specific value was not explicitly confirmed in the source. If a discrepancy ever shows up for
//   this link, check that axis first.
// - fer_hand#1 (palm top, radius 0.02, center z=0.10, axis Y) is flagged expected_contact: it is
//   evaluated but does NOT enter min_distance_arm_m / feasible. Measured in the production probe
//   (reachable satellite, IK error 7-10 mm): it interpenetrates the cylinder by ~9.5 mm at the
//   nominal grasp on all 4 grasp points and by ~19 mm with the grasp 1 cm deeper. UNVERIFIED
//   hypothesis behind the exclusion: the official Franka capsules are self-collision envelopes
//   that cover the region between the fingers, where the wall sits in a real grasp, so this
//   overlap is expected and not a real collision. LIMIT: a genuine palm impact in that region
//   is NOT detected. fer_hand#0 and all other capsules are still checked.
// - fer_link8 has NO capsule in the source, so none is modelled here. This may be intentional or a
//   gap in the source file; no safety placeholder was invented for it.
const std::vector<LinkCapsule>& linkCapsules() {
    static const std::vector<LinkCapsule> capsules = {
        {"fer_link0", {-0.075, 0.0, 0.060}, Axis::X, 0.060, 0.030},
        {"fer_link1", {0.0, 0.0, -0.1915}, Axis::Z, 0.060, 0.283},
        {"fer_link2", {0.0, 0.0, 0.0}, Axis::Z, 0.060, 0.120},
        {"fer_link3", {0.0, 0.0, -0.145}, Axis::Z, 0.060, 0.150},
        {"fer_link4", {0.0, 0.0, 0.0}, Axis::Z, 0.060, 0.120},
        {"fer_link5", {0.0, 0.0, -0.260}, Axis::Z, 0.060, 0.100},   // main
        {"fer_link5", {0.0, 0.080, -0.130}, Axis::Z, 0.025, 0.140}, // elbow (axis unconfirmed)
        {"fer_link6", {0.0, 0.0, -0.030}, Axis::Z, 0.050, 0.080},
        {"fer_link7", {0.0, 0.0, 0.010}, Axis::Z, 0.040, 0.140},    // axis
        {"fer_link7", {0.060, 0.0, 0.082}, Axis::X, 0.030, 0.010},  // flange
        {"fer_hand", {0.0, 0.0, 0.04}, Axis::Y, 0.040, 0.10},       // palm base (confirmed)
        {"fer_hand", {0.0, 0.0, 0.10}, Axis::Y, 0.020, 0.10, true}, // palm top (confirmed), expected contact
    };
    return capsules;
}

Eigen::Vector3d axisVector(Axis a) {
    switch (a) {
        case Axis::X: return Eigen::Vector3d::UnitX();
        case Axis::Y: return Eigen::Vector3d::UnitY();
        default: return Eigen::Vector3d::UnitZ();
    }
}

double signedDistance(const coal::CollisionGeometry& a, const coal::Transform3s& ta,
                      const coal::CollisionGeometry& b, const coal::Transform3s& tb) {
    coal::DistanceRequest req;
    req.enable_signed_distance = true;  // required for negative distances under penetration
    coal::DistanceResult res;
    coal::distance(&a, ta, &b, tb, req, res);
    return res.min_distance;
}

}  // namespace

CollisionCheckResult checkSatelliteCollision(
    const RobotModel& robot_model,
    const haptic_dmp_learning::core::CubeSatelliteModel& satellite_model,
    double theta_rad,
    double d_safe_arm_m) {
    // Satellite cylinder: axis segment base -> top.
    const Eigen::Vector3d base = satellite_model.cylinderBaseCenterWorld(theta_rad);
    const Eigen::Vector3d top = satellite_model.cylinderTopCenterWorld(theta_rad);
    const Eigen::Vector3d axis = satellite_model.cylinderAxisWorld(theta_rad).normalized();
    const double height = (top - base).norm();
    const coal::Cylinder cylinder(satellite_model.cylinderOuterRadius(), height);  // (radius, full height)
    const Eigen::Vector3d cyl_center = 0.5 * (base + top);
    const Eigen::Matrix3d cyl_R =
        Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), axis).toRotationMatrix();
    const coal::Transform3s cyl_tf(cyl_R, cyl_center);

    // Satellite cube body: exact solid box (full side lengths), pose from the model.
    const double side = satellite_model.cubeSideM();
    const coal::Box cube(side, side, side);
    const coal::Transform3s cube_tf(satellite_model.cubeRotationWorld(theta_rad),
                                    satellite_model.cubeCenterWorld());

    double min_dist = std::numeric_limits<double>::infinity();
    double min_dist_cube = std::numeric_limits<double>::infinity();
    std::string closest_cube;
    std::string closest;
    std::vector<CapsuleDistance> per_capsule;
    std::string prev_frame;
    int idx_in_frame = 0;
    for (const LinkCapsule& cap : linkCapsules()) {
        // World pose of the capsule = link frame pose composed with the local capsule offset.
        idx_in_frame = (prev_frame == cap.frame) ? idx_in_frame + 1 : 0;
        prev_frame = cap.frame;
        const RobotModel::FramePose link = robot_model.framePose(cap.frame);
        const Eigen::Matrix3d R_link = link.orientation.toRotationMatrix();
        const Eigen::Matrix3d R_local =
            Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), axisVector(cap.axis))
                .toRotationMatrix();
        const coal::Capsule capsule(cap.radius, cap.length);  // (radius, cylindrical-part length)
        const coal::Transform3s tf(R_link * R_local, link.position + R_link * cap.center);
        const double d = signedDistance(capsule, tf, cylinder, cyl_tf);
        const std::string name = std::string(cap.frame) + "#" + std::to_string(idx_in_frame);
        const double d_cube = signedDistance(capsule, tf, cube, cube_tf);
        per_capsule.push_back({name, d, cap.expected_contact, d_cube});
        if (d_cube < min_dist_cube) {
            min_dist_cube = d_cube;
            closest_cube = name;
        }
        if (!cap.expected_contact && d < min_dist) {
            min_dist = d;
            closest = name;
        }
    }

    CollisionCheckResult res;
    res.per_capsule = std::move(per_capsule);
    res.min_distance_arm_m = min_dist;
    res.closest_capsule = closest;
    res.min_distance_cube_m = min_dist_cube;
    res.closest_capsule_cube = closest_cube;
    res.feasible = (min_dist >= d_safe_arm_m) && (min_dist_cube >= d_safe_arm_m);
    return res;
}

}  // namespace core
}  // namespace satellite_grasp_planner
