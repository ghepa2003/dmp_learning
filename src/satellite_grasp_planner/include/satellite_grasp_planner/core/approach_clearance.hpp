#pragma once

/**
 * @file approach_clearance.hpp
 * @brief Signed distance between the gripper (hand + 2 fingers) and the rotating satellite (cube + collar) along the
 *        COMMANDED Cartesian approach trajectory of the TCP. Used by the v2 selection as a constraint
 *        (S3Params::use_clearance); see docs/S3_SPEC.md, section "vincolo di clearance".
 *
 * Model (reproduces _diag_1019/interference/predict_distance.py, predictor T, which predicted the distance measured in
 * Gazebo with mean error 0.35 mm and max 1.46 mm on 17 runs without contact; NOT the planner's joint rollout, which
 * is off by 5 mm on average):
 *   - TCP pose (p, q) from the trajectory; hand frame = TCP frame (fer_hand_tcp_joint has rpy 0) with origin
 *     p - R (0, 0, kHandTcpOffsetM).
 *   - hand: the fer_hand collision mesh (URDF) AS IS, built once into a coal::Convex. The mesh is verified closed and convex at
 *     load time (validateHandMesh); coal 3.0.3 of ROS Humble is built without Qhull, so no hull is ever computed.
 *   - fingers: 4 boxes each (franka_hand.xacro), finger frame = hand frame + (0, +/-gripper_open_m, kFingerOriginZM).
 *   - satellite: cube (cube_side_m) + collar of kCollarSegments boxes, rotated about the axis through the center by
 *     theta(t) = theta_star - omega * (tau - t)  (omega signed, t in [0, tau] from the start of the approach).
 *   - signed distance with coal (negative = penetration); the minimum over the window [0, tau], sampled every dt_s.
 * Constants come from CubeSatelliteModel::Params (the planner's cube_geometry), not from copies.
 *
 * Eigen + STL in this header; coal is confined to approach_clearance.cpp (pimpl).
 */

#include <array>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "haptic_dmp_learning/core/cube_satellite_model.hpp"

namespace satellite_grasp_planner {
namespace core {

/// fer_hand -> fer_hand_tcp along the hand z [m]: franka_hand.xacro tcp_xyz default "0 0 0.1034", fer_flat_effort.urdf:252.
inline constexpr double kHandTcpOffsetM = 0.1034;
/// Finger frame origin along the hand z [m]: franka_hand.xacro finger_joint1/2 origin "0 .. 0.0584".
inline constexpr double kFingerOriginZM = 0.0584;
/// Finger joint range [m] (franka_hand.xacro, prismatic limit lower 0 upper 0.08).
inline constexpr double kFingerOpenMaxM = 0.08;
/// Number of collar boxes and tangential overlap factor: free_target_object.sdf.xacro (collar_segments default 24,
/// collar_overlap 0.05; box = standoff x wall x 2 R tan(pi/N) (1 + overlap)).
inline constexpr int kCollarSegments = 24;
inline constexpr double kCollarOverlap = 0.05;
/// The constraint ignores the last kClearanceTailExcludeS seconds before contact (the hand is meant to reach the collar).
inline constexpr double kClearanceTailExcludeS = 2.0;

/// One sample of the commanded TCP trajectory: time from the start of the approach (the TRIGGER), pose of the TCP.
struct ApproachSample {
  double t_s = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

struct ClearanceParams {
  double dt_s = 0.05;             ///< sampling of the window [0, tau]; > 0
  double gripper_open_m = 0.060;  ///< finger joint position (both fingers), in [0, kFingerOpenMaxM]
};

/// @throws std::invalid_argument naming the field.
void validateClearanceParams(const ClearanceParams& p);

struct ClearancePair {
  std::string robot;      ///< "mano" | "dito 1" (left, +y) | "dito 2" (right, -y)
  std::string satellite;  ///< "cubo" | "collare"
  std::string str() const { return robot.empty() ? std::string("n/v") : robot + "-" + satellite; }
};

struct ClearanceResult {
  /// Minimum over the whole window [0, tau] [mm], with its pair and its time relative to contact (t - tau, <= 0).
  double min_mm = 0.0;
  ClearancePair pair;
  double t_rel_s = 0.0;
  /// Same over the window [0, tau - kClearanceTailExcludeS] ("senza ultimi 2 s"): THIS is the value the constraint uses.
  double min_mm_no_tail = 0.0;
  ClearancePair pair_no_tail;
  double t_rel_no_tail_s = 0.0;
};

/// Triangle mesh of the hand collision geometry in the hand frame (unique vertices, triangles as index triples).
struct HandMesh {
  std::vector<Eigen::Vector3d> points;
  std::vector<std::array<std::size_t, 3>> triangles;
  /// Max convexity violation [m] measured by validateHandMesh (reported by describeHandMesh); < 0 = not measured yet.
  double convexity_violation_m = -1.0;
};

/// Tolerance of the convexity check of validateHandMesh [m]. ASSUMED 1.0 mm: the real fer_hand mesh (hand.stl) measures
/// 0.74 mm (19/10/2026); GJK works on the vertices, so the effect on the distance is expected to be negligible against the
/// 2 mm clearance margin (S3_SPEC.md 10.6). Used only by validateHandMesh.
inline constexpr double kHandMeshConvexityTolM = 1.0e-3;

/// Fail-loud checks of a hand mesh, then outward (counter-clockwise seen from outside) orientation of every triangle:
///  (a) indices in range, >= 4 points, >= 4 triangles, finite points;
///  (b) no degenerate triangle (|(b-a)x(c-a)| < 1e-12 m^2);
///  (b') each triangle with n.(a - centroid of the points) < 0 gets b and c swapped;
///  (c) convexity: max over triangles and points of n_hat.(p - a) must be <= convexity_tol_m; the measured maximum is stored
///      in mesh.convexity_violation_m (also when it exceeds the tolerance).
/// @throws std::invalid_argument ("... is not convex (max violation X mm): the clearance model needs a convex mesh", ...).
void validateHandMesh(HandMesh& mesh, double convexity_tol_m = kHandMeshConvexityTolM);

/// One line "HAND_MESH vertices=N triangles=M bbox_min=(x,y,z) bbox_max=(x,y,z) max_convexity_violation_mm=V" (bbox in m;
/// V is "n/a" if the mesh was never validated).
std::string describeHandMesh(const HandMesh& mesh);

/// Axis-aligned box [-h, h] as a closed mesh: 8 points, 12 triangles, outward orientation (synthetic hand for tests/selftest).
HandMesh makeBoxHandMesh(const Eigen::Vector3d& half_extents);

/// Hand convex mesh + collar/finger geometry. Built once; not copyable. Thread-compatible (const use only).
class ApproachClearanceModel {
 public:
  /// @param cube  cube_side_m, standoff_m, collar_radius_m, wall_thickness_m, face_normal_body are used (center_world
  ///              and axis_world are NOT: they come with every query, from the snapshot).
  /// @param hand  collision mesh of fer_hand in the hand frame; validateHandMesh() is run on a copy (convexity, orientation).
  /// @throws std::invalid_argument for invalid geometry or an invalid / non-convex mesh.
  ApproachClearanceModel(const haptic_dmp_learning::core::CubeSatelliteModel::Params& cube, const HandMesh& hand);
  ~ApproachClearanceModel();
  ApproachClearanceModel(const ApproachClearanceModel&) = delete;
  ApproachClearanceModel& operator=(const ApproachClearanceModel&) = delete;

  struct Impl;
  const Impl& impl() const { return *impl_; }

 private:
  std::unique_ptr<Impl> impl_;
};

/**
 * Pure function: clearance of the commanded approach.
 * @param tcp_trajectory TCP samples, strictly increasing t_s, first t_s <= 0, last t_s >= tau (position linearly and
 *        orientation by shortest-path SLERP interpolated at the sampling times).
 * @param tau_s  duration of the approach (window [0, tau]); must exceed kClearanceTailExcludeS.
 * @param theta_star_rad / omega_rad_s  phase of the satellite at contact and its signed rate:
 *        theta(t) = theta_star - omega (tau - t).
 * @param center_world / axis_world  center and spin axis of the satellite (snapshot).
 * Sampling times: k * dt_s for k * dt_s < tau, plus t = tau. t_rel = t - tau.
 * Window of min_mm_no_tail: t_rel <= -kClearanceTailExcludeS (+1e-4 s, as the Python reference).
 * @throws std::invalid_argument for invalid inputs (no fallback).
 */
ClearanceResult approachClearance(const ApproachClearanceModel& model, const std::vector<ApproachSample>& tcp_trajectory,
                                  double tau_s, double theta_star_rad, double omega_rad_s,
                                  const Eigen::Vector3d& center_world, const Eigen::Vector3d& axis_world,
                                  const ClearanceParams& params);

/// What the selection hands to the clearance evaluator for one finalist.
struct ClearanceQuery {
  const std::vector<ApproachSample>* trajectory = nullptr;
  double tau_s = 0.0;
  double theta_star_rad = 0.0;
  double omega_rad_s = 0.0;
  Eigen::Vector3d center_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d axis_world = Eigen::Vector3d::UnitZ();
  int k = 0;
  bool wrist_flip = false;
};
using ClearanceEvaluator = std::function<ClearanceResult(const ClearanceQuery&)>;

/// Evaluator calling approachClearance(*model, ...). @p model must outlive the returned function.
ClearanceEvaluator makeClearanceEvaluator(const ApproachClearanceModel& model, const ClearanceParams& params);

// ---- hand collision mesh from the URDF (startup only) ----

/// URI of the <collision><geometry><mesh filename="..."> of the link "fer_hand".
/// @throws std::runtime_error if the link/collision/mesh is not found, or the collision has an <origin> or a mesh
///         scale (the loader would silently ignore it).
std::string handCollisionMeshUri(const std::string& urdf_xml);

/// "package://pkg/rest" -> "<prefix>/share/pkg/rest" for the first prefix of @p ament_prefix_path (colon separated)
/// where the file exists. @throws std::runtime_error if the URI is not package:// or nothing is found.
std::string resolvePackageUri(const std::string& uri, const std::string& ament_prefix_path);

/// Mesh of a binary (size == 84 + 50 n, n = little-endian uint32 at bytes 80-83) or ASCII STL. Identical vertices (exact
/// value) are merged. Not validated (see validateHandMesh). @throws std::runtime_error (also for a truncated binary file).
HandMesh readStlMesh(const std::string& path);

/// readStlMesh + validateHandMesh of an STL file.
HandMesh loadHandMeshFile(const std::string& path);

/// handCollisionMeshUri + resolvePackageUri(getenv("AMENT_PREFIX_PATH")) + loadHandMeshFile for the URDF at @p urdf_path.
/// @throws std::runtime_error / std::invalid_argument.
HandMesh loadHandCollisionMesh(const std::string& urdf_path);

}  // namespace core
}  // namespace satellite_grasp_planner
