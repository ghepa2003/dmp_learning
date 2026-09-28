#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace haptic_dmp_learning {
namespace core {

enum class GraspPointId { kP0 = 0, kP90, kP180, kP270 };  // phi_k = 0/90/180/270 deg

struct GraspTargetPose {
    Eigen::Vector3d position_world;
    Eigen::Quaterniond orientation_nominal_world;  // reference psi = 0
    Eigen::Vector3d approach_axis_world;           // = z_tcp; also the roll axis for psi
};

/**
 * @brief Pure geometry of the cube-shaped target satellite (replaces the ring model).
 *
 * The cube spins about axis_world through center_world by theta. A circular collar
 * (radius collar_radius_m) sits at standoff_m in front of a lateral face; the four grasp
 * points lie on it at phi_k = k*90 deg. Collar frame: u_body = n x axis (horizontal),
 * v_body = axis (vertical), r_k = cos(phi_k) u + sin(phi_k) v.
 *
 * No temporal state: theta_rad is an explicit argument of every query, and all queries
 * are pure functions of (k, theta_rad, params_).
 */
class CubeSatelliteModel {
public:
    struct Params {
        Eigen::Vector3d center_world = Eigen::Vector3d::Zero();
        Eigen::Vector3d axis_world = Eigen::Vector3d::UnitZ();       // normalized at construction
        double cube_side_m = 0.20;
        double standoff_m = 0.08;         // along the normal, beyond the face
        double collar_radius_m = 0.05;    // Phi10cm / 2
        Eigen::Vector3d face_normal_body = Eigen::Vector3d::UnitX();  // must be lateral
        double wall_thickness_m = 0.015;  // hollow-cylinder wall, typical 1-2 cm, to be calibrated
    };

    /// Normalizes axis_world / face_normal_body. Throws std::invalid_argument if either has
    /// ~zero norm, if the face is not lateral (|axis . n| > kLateralTolerance), or if
    /// wall_thickness_m <= 0 or >= collar_radius_m.
    explicit CubeSatelliteModel(Params params);

    GraspTargetPose graspPoseAt(GraspPointId k, double theta_rad) const;

    // Public: needed by the collision constraint in Step B.
    Eigen::Vector3d faceNormalWorld(double theta_rad) const;                       // R(theta) * n
    Eigen::Vector3d radialDirectionWorld(GraspPointId k, double theta_rad) const;  // R(theta) * r_k

    // Hollow-cylinder geometry (pure geometry, no collision logic). The cylinder is radial:
    // its axis is the face normal and it spans from the cube face to standoff_m.
    double cylinderOuterRadius() const;                                  // collar_radius_m
    double cylinderInnerRadius() const;                                  // collar_radius_m - wall
    Eigen::Vector3d cylinderAxisWorld(double theta_rad) const;           // = faceNormalWorld
    Eigen::Vector3d cylinderBaseCenterWorld(double theta_rad) const;     // on the cube face
    Eigen::Vector3d cylinderTopCenterWorld(double theta_rad) const;      // base + standoff * axis

    // Cube body pose (pure geometry): centered at center_world, rotated by R(theta) about axis_world.
    Eigen::Vector3d cubeCenterWorld() const;
    /// Cube body orientation in world: R(theta) * [n, axis, n x axis] (columns), the same frame
    /// graspPoseAt uses. NOT R(theta) alone (that would assume a world-aligned body frame).
    Eigen::Matrix3d cubeRotationWorld(double theta_rad) const;
    double cubeSideM() const;
    Eigen::Vector3d axisWorld() const;  // normalized spin axis (theta-independent)
    Eigen::Vector3d faceNormalBody() const;  // normalized face normal at theta = 0                                   // cube_side_m

    /// Tolerance on |axis . n| for "face is lateral". 1e-6 (~1 microrad) is far above double
    /// round-off on unit vectors (~1e-16), so exactly-perpendicular inputs always pass, yet
    /// tight enough that the frame u/v/n stays orthonormal to well below any geometric
    /// effect on the millimetre-scale grasp poses. A tilted face is a modelling error, not
    /// something to absorb silently.
    static constexpr double kLateralTolerance = 1e-6;

private:
    Eigen::Vector3d radialDirectionBody(GraspPointId k) const;
    Eigen::Matrix3d rotation(double theta_rad) const;

    Params params_;
};

}  // namespace core
}  // namespace haptic_dmp_learning
