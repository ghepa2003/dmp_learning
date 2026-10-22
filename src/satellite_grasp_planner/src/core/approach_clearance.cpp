#include "satellite_grasp_planner/core/approach_clearance.hpp"

#include <algorithm>
#include <array>
#include <map>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>

#include <coal/distance.h>
#include <coal/math/transform.h>
#include <coal/data_types.h>
#include <coal/shape/convex.h>
#include <coal/shape/geometric_shapes.h>

#include "haptic_dmp_learning/core/math_utils.hpp"

namespace satellite_grasp_planner {
namespace core {

using haptic_dmp_learning::core::CubeSatelliteModel;

namespace {

constexpr double kPi = haptic_dmp_learning::core::kPi;
constexpr double kInf = std::numeric_limits<double>::infinity();

/// Collision boxes of one finger in the finger frame, franka_hand.xacro: leftfinger boxes at lines 51, 58, 65, 72 (right: 90, 97,
/// 104, 111; the same with y -> -y and the diagonal box rotated by -pi/6). sign = +1 left, -1 right.
struct FingerBoxSpec {
  double sx, sy, sz;  ///< full side lengths [m]
  double ox, oy, oz;  ///< origin in the finger frame, left finger [m]
  double roll;        ///< rotation about x, left finger [rad]
};
constexpr FingerBoxSpec kFingerBoxes[4] = {
    {22e-3, 15e-3, 20e-3, 0.0, 18.5e-3, 11e-3, 0.0},            // screw mount
    {22e-3, 8.8e-3, 3.8e-3, 0.0, 6.8e-3, 2.2e-3, 0.0},          // carriage sledge
    {17.5e-3, 7e-3, 23.5e-3, 0.0, 15.9e-3, 28.35e-3, kPi / 6},  // diagonal finger
    {17.5e-3, 15.2e-3, 18.5e-3, 0.0, 7.58e-3, 45.25e-3, 0.0},   // rubber tip
};

struct Shape {
  std::shared_ptr<coal::CollisionGeometry> geom;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();  ///< bounding-sphere center in the shape frame
  double radius = 0.0;                               ///< bounding-sphere radius
};

struct FingerBox {
  Shape shape;
  Eigen::Vector3d p;  ///< origin in the finger frame
  Eigen::Matrix3d R;  ///< orientation in the finger frame
};

double signedDistanceM(const coal::CollisionGeometry& a, const coal::Transform3s& ta, const coal::CollisionGeometry& b,
                       const coal::Transform3s& tb) {
  coal::DistanceRequest req;
  req.enable_signed_distance = true;  // negative distances under penetration
  coal::DistanceResult res;
  coal::distance(&a, ta, &b, tb, req, res);
  return res.min_distance;
}

Shape makeBox(double sx, double sy, double sz) {
  Shape s;
  s.geom = std::make_shared<coal::Box>(sx, sy, sz);
  s.radius = 0.5 * std::sqrt(sx * sx + sy * sy + sz * sz);
  return s;
}

}  // namespace

namespace {

/// The ONLY place that touches coal's mesh API (coal 3.0.3, no Qhull): Convex<Triangle> straight from vertices and
/// triangles (counter-clockwise from outside, guaranteed by validateHandMesh). The constructor does not use Qhull.
std::shared_ptr<coal::ConvexBase> makeCoalConvex(const HandMesh& mesh) {
  auto pts = std::make_shared<std::vector<coal::Vec3s>>(mesh.points.begin(), mesh.points.end());
  auto tris = std::make_shared<std::vector<coal::Triangle>>();
  tris->reserve(mesh.triangles.size());
  for (const auto& t : mesh.triangles) tris->emplace_back(t[0], t[1], t[2]);
  const unsigned n_pts = static_cast<unsigned>(pts->size());
  const unsigned n_tris = static_cast<unsigned>(tris->size());
  return std::make_shared<coal::Convex<coal::Triangle>>(pts, n_pts, tris, n_tris);
}

}  // namespace

void validateHandMesh(HandMesh& mesh, double convexity_tol_m) {
  if (!(std::isfinite(convexity_tol_m) && convexity_tol_m >= 0.0)) {
    throw std::invalid_argument("hand mesh: convexity tolerance must be finite and >= 0.");
  }
  const std::size_t np = mesh.points.size();
  if (np < 4 || mesh.triangles.size() < 4) throw std::invalid_argument("hand mesh: needs >= 4 points and >= 4 triangles.");
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const Eigen::Vector3d& p : mesh.points) {
    if (!p.allFinite()) throw std::invalid_argument("hand mesh: non-finite point.");
    centroid += p;
  }
  centroid /= static_cast<double>(np);
  for (auto& t : mesh.triangles) {
    if (t[0] >= np || t[1] >= np || t[2] >= np) throw std::invalid_argument("hand mesh: triangle index out of range.");
    const Eigen::Vector3d& a = mesh.points[t[0]];
    const Eigen::Vector3d n = (mesh.points[t[1]] - a).cross(mesh.points[t[2]] - a);
    if (!(n.norm() >= 1e-12)) throw std::invalid_argument("hand mesh: degenerate triangle (|cross product| < 1e-12 m^2).");
    if (n.dot(a - centroid) < 0.0) std::swap(t[1], t[2]);
  }
  double worst = 0.0;
  for (const auto& t : mesh.triangles) {
    const Eigen::Vector3d& a = mesh.points[t[0]];
    const Eigen::Vector3d n = (mesh.points[t[1]] - a).cross(mesh.points[t[2]] - a).normalized();
    for (const Eigen::Vector3d& p : mesh.points) worst = std::max(worst, n.dot(p - a));
  }
  mesh.convexity_violation_m = worst;
  if (worst > convexity_tol_m) {
    std::ostringstream m;
    m << "fer_hand collision mesh is not convex (max violation " << worst * 1000.0
      << " mm): the clearance model needs a convex mesh";
    throw std::invalid_argument(m.str());
  }
}

std::string describeHandMesh(const HandMesh& mesh) {
  Eigen::Vector3d lo = Eigen::Vector3d::Constant(kInf), hi = Eigen::Vector3d::Constant(-kInf);
  for (const Eigen::Vector3d& p : mesh.points) {
    lo = lo.cwiseMin(p);
    hi = hi.cwiseMax(p);
  }
  std::ostringstream o;
  o.precision(6);
  o << "HAND_MESH vertices=" << mesh.points.size() << " triangles=" << mesh.triangles.size() << " bbox_min=(" << lo.x()
    << "," << lo.y() << "," << lo.z() << ") bbox_max=(" << hi.x() << "," << hi.y() << "," << hi.z() << ")";
  if (mesh.convexity_violation_m >= 0.0) {
    o << " max_convexity_violation_mm=" << mesh.convexity_violation_m * 1000.0;
  } else {
    o << " max_convexity_violation_mm=n/a";
  }
  return o.str();
}

HandMesh makeBoxHandMesh(const Eigen::Vector3d& h) {
  HandMesh m;
  for (int i = 0; i < 8; ++i) m.points.emplace_back((i & 1 ? h.x() : -h.x()), (i & 2 ? h.y() : -h.y()), (i & 4 ? h.z() : -h.z()));
  // vertex index = x_bit + 2 y_bit + 4 z_bit; two triangles per face, outward (counter-clockwise from outside)
  const std::size_t f[12][3] = {{0, 2, 1}, {1, 2, 3}, {4, 5, 6}, {5, 7, 6}, {0, 1, 4}, {1, 5, 4}, {2, 6, 3}, {3, 6, 7}, {0, 4, 2}, {2, 4, 6}, {1, 3, 5}, {3, 7, 5}};
  for (const auto& t : f) m.triangles.push_back({t[0], t[1], t[2]});
  return m;
}

struct ApproachClearanceModel::Impl {
  CubeSatelliteModel::Params cube;  ///< constants only (center / axis come with the query)
  Shape hand;
  Shape cube_box;
  Shape collar_box;
  std::vector<FingerBox> finger[2];  ///< [0] left (+y, "dito 1"), [1] right (-y, "dito 2")
  double collar_xc = 0.0;            ///< distance of the box centers from the cube center along the face normal
  double collar_rm = 0.0;            ///< radius of the box centers (mid-wall)
};

void validateClearanceParams(const ClearanceParams& p) {
  if (!(std::isfinite(p.dt_s) && p.dt_s > 0.0)) {
    throw std::invalid_argument("s1.clearance_dt_s must be finite and > 0.");
  }
  if (!(std::isfinite(p.gripper_open_m) && p.gripper_open_m >= 0.0 && p.gripper_open_m <= kFingerOpenMaxM)) {
    throw std::invalid_argument("s1.gripper_open_m must be finite and in [0, 0.08] m (finger joint range).");
  }
}

ApproachClearanceModel::ApproachClearanceModel(const CubeSatelliteModel::Params& cube, const HandMesh& hand_mesh)
    : impl_(std::make_unique<Impl>()) {
  Impl& m = *impl_;
  m.cube = cube;
  if (!(std::isfinite(cube.cube_side_m) && cube.cube_side_m > 0.0 && std::isfinite(cube.standoff_m) &&
        cube.standoff_m > 0.0 && std::isfinite(cube.collar_radius_m) && std::isfinite(cube.wall_thickness_m) &&
        cube.wall_thickness_m > 0.0 && cube.wall_thickness_m < cube.collar_radius_m)) {
    throw std::invalid_argument("ApproachClearanceModel: invalid cube_geometry (side, standoff, collar radius, wall).");
  }
  if (!(cube.face_normal_body.norm() > 1e-9)) {
    throw std::invalid_argument("ApproachClearanceModel: face_normal_body has ~zero norm.");
  }

  // ---- hand: the collision mesh as is (checked convex and outward-oriented), no hull computation ----
  HandMesh mesh = hand_mesh;
  validateHandMesh(mesh);
  m.hand.geom = makeCoalConvex(mesh);
  double r = 0.0;
  for (const Eigen::Vector3d& p : mesh.points) r = std::max(r, p.norm());
  m.hand.center = Eigen::Vector3d::Zero();
  m.hand.radius = r;

  // ---- satellite shapes ----
  m.cube_box = makeBox(cube.cube_side_m, cube.cube_side_m, cube.cube_side_m);
  const double tangential = 2.0 * cube.collar_radius_m * std::tan(kPi / kCollarSegments) * (1.0 + kCollarOverlap);
  m.collar_box = makeBox(cube.standoff_m, cube.wall_thickness_m, tangential);
  m.collar_xc = 0.5 * cube.cube_side_m + 0.5 * cube.standoff_m;
  m.collar_rm = 0.5 * (cube.collar_radius_m + (cube.collar_radius_m - cube.wall_thickness_m));

  // ---- fingers ----
  for (int side = 0; side < 2; ++side) {
    const double sg = side == 0 ? 1.0 : -1.0;
    for (const FingerBoxSpec& b : kFingerBoxes) {
      FingerBox fb;
      fb.shape = makeBox(b.sx, b.sy, b.sz);
      fb.p = Eigen::Vector3d(b.ox, sg * b.oy, b.oz);
      fb.R = Eigen::AngleAxisd(sg * b.roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
      m.finger[side].push_back(fb);
    }
  }
}

ApproachClearanceModel::~ApproachClearanceModel() = default;

namespace {

/// Position (linear) and orientation (shortest-path SLERP) of the trajectory at time t, t within [front, back].
void interpolate(const std::vector<ApproachSample>& traj, double t, Eigen::Vector3d& p, Eigen::Matrix3d& R) {
  auto it = std::upper_bound(traj.begin(), traj.end(), t,
                             [](double v, const ApproachSample& s) { return v < s.t_s; });
  if (it == traj.begin()) {
    p = traj.front().position;
    R = traj.front().orientation.normalized().toRotationMatrix();
    return;
  }
  if (it == traj.end()) {
    p = traj.back().position;
    R = traj.back().orientation.normalized().toRotationMatrix();
    return;
  }
  const ApproachSample& b = *it;
  const ApproachSample& a = *(it - 1);
  const double s = (t - a.t_s) / (b.t_s - a.t_s);
  p = (1.0 - s) * a.position + s * b.position;
  R = haptic_dmp_learning::core::slerpShortestPath(a.orientation.normalized(), b.orientation.normalized(), s)
          .toRotationMatrix();
}

struct RobotShape {
  const Shape* shape;
  coal::Transform3s tf;
  Eigen::Vector3d center_world;
  int part;  ///< 0 mano, 1 dito 1, 2 dito 2
};

struct SatShape {
  const Shape* shape;
  coal::Transform3s tf;
  Eigen::Vector3d center_world;
  int part;  ///< 0 cubo, 1 collare
};

const char* robotPartName(int part) { return part == 0 ? "mano" : (part == 1 ? "dito 1" : "dito 2"); }
const char* satPartName(int part) { return part == 0 ? "cubo" : "collare"; }

}  // namespace

ClearanceResult approachClearance(const ApproachClearanceModel& model, const std::vector<ApproachSample>& traj,
                                  double tau_s, double theta_star_rad, double omega_rad_s,
                                  const Eigen::Vector3d& center_world, const Eigen::Vector3d& axis_world,
                                  const ClearanceParams& params) {
  validateClearanceParams(params);
  if (!(std::isfinite(tau_s) && tau_s > kClearanceTailExcludeS)) {
    throw std::invalid_argument("approachClearance: tau must be finite and > " + std::to_string(kClearanceTailExcludeS) +
                                " s (the constraint ignores the last " + std::to_string(kClearanceTailExcludeS) + " s).");
  }
  if (!(std::isfinite(theta_star_rad) && std::isfinite(omega_rad_s))) {
    throw std::invalid_argument("approachClearance: theta_star and omega must be finite.");
  }
  if (!(center_world.allFinite() && axis_world.allFinite() && axis_world.norm() > 1e-9)) {
    throw std::invalid_argument("approachClearance: invalid satellite center / axis.");
  }
  if (traj.size() < 2) throw std::invalid_argument("approachClearance: the trajectory needs >= 2 samples.");
  for (std::size_t i = 0; i < traj.size(); ++i) {
    if (!(std::isfinite(traj[i].t_s) && traj[i].position.allFinite() && traj[i].orientation.coeffs().allFinite() &&
          traj[i].orientation.norm() > 1e-9)) {
      throw std::invalid_argument("approachClearance: non-finite sample or zero quaternion in the trajectory.");
    }
    if (i > 0 && !(traj[i].t_s > traj[i - 1].t_s)) {
      throw std::invalid_argument("approachClearance: trajectory times must be strictly increasing.");
    }
  }
  if (!(traj.front().t_s <= 1e-12 && traj.back().t_s >= tau_s - 1e-9)) {
    throw std::invalid_argument("approachClearance: the trajectory does not cover [0, tau].");
  }

  const ApproachClearanceModel::Impl& m = model.impl();
  const Eigen::Vector3d axis = axis_world.normalized();
  const Eigen::Vector3d n_body = m.cube.face_normal_body.normalized();
  if (!(n_body.cross(axis).norm() > 1e-6)) {
    throw std::invalid_argument("approachClearance: face_normal_body is parallel to the satellite axis.");
  }
  const Eigen::Vector3d u = n_body.cross(axis).normalized();  // collar frame of CubeSatelliteModel: u = n x axis, v = axis

  // Cube box orientation at theta = 0: the proper rotation [n, a x n, a] (det = n . ((a x n) x a) = n . n = 1 for n . a = 0),
  // the same as satelliteBaseOrientation() (ros/grasp_planner_checks.cpp; not linkable from the core). It is the identity,
  // component by component, for n = +x, a = +z: then R_cube is a copy of R_s and the nominal numeric path is unchanged.
  if (!(std::abs(n_body.dot(axis)) <= CubeSatelliteModel::kLateralTolerance)) {
    throw std::invalid_argument("approachClearance: face_normal_body is not lateral (|n . axis| > 1e-6).");
  }
  Eigen::Matrix3d R_base;
  R_base.col(0) = n_body;
  R_base.col(1) = axis.cross(n_body).normalized();
  R_base.col(2) = axis;
  const bool base_is_identity = (R_base == Eigen::Matrix3d::Identity());  // exact comparison (-0.0 == 0.0)

  // Collar boxes in the satellite body frame (theta = 0), relative to the center. Box axes: x along n (length),
  // y radial (thickness), z tangential - as free_target_object.sdf.xacro (rotation phi about +n).
  std::vector<Eigen::Vector3d> col_p(kCollarSegments);
  std::vector<Eigen::Matrix3d> col_R(kCollarSegments);
  for (int i = 0; i < kCollarSegments; ++i) {
    const double phi = 2.0 * kPi * i / kCollarSegments;
    const Eigen::Vector3d r = std::cos(phi) * u + std::sin(phi) * axis;
    col_p[static_cast<std::size_t>(i)] = m.collar_xc * n_body + m.collar_rm * r;
    Eigen::Matrix3d R;
    R.col(0) = n_body;
    R.col(1) = r;
    R.col(2) = n_body.cross(r);
    col_R[static_cast<std::size_t>(i)] = R;
  }

  // Sampling times: k * dt for k * dt < tau, then tau.
  std::vector<double> times;
  for (int k = 0;; ++k) {
    const double t = k * params.dt_s;
    if (!(t < tau_s - 1e-9)) break;
    times.push_back(t);
  }
  times.push_back(tau_s);

  ClearanceResult out;
  out.min_mm = kInf;
  out.min_mm_no_tail = kInf;
  const double tail_limit = tau_s - kClearanceTailExcludeS + 1e-4;

  std::vector<RobotShape> robot;
  std::vector<SatShape> sat;
  robot.reserve(9);
  sat.reserve(1 + kCollarSegments);

  for (const double t : times) {
    Eigen::Vector3d p_tcp;
    Eigen::Matrix3d R_h;
    interpolate(traj, t, p_tcp, R_h);
    const bool in_no_tail = t <= tail_limit;
    const Eigen::Vector3d p_hand = p_tcp - R_h * Eigen::Vector3d(0.0, 0.0, kHandTcpOffsetM);

    robot.clear();
    robot.push_back({&m.hand, coal::Transform3s(R_h, p_hand), p_hand, 0});
    for (int side = 0; side < 2; ++side) {
      const double sg = side == 0 ? 1.0 : -1.0;
      const Eigen::Vector3d p_f = p_hand + R_h * Eigen::Vector3d(0.0, sg * params.gripper_open_m, kFingerOriginZM);
      for (const FingerBox& fb : m.finger[side]) {
        const Eigen::Vector3d pw = p_f + R_h * fb.p;
        robot.push_back({&fb.shape, coal::Transform3s(R_h * fb.R, pw), pw, side + 1});
      }
    }

    const double theta = theta_star_rad - omega_rad_s * (tau_s - t);
    const Eigen::Matrix3d R_s = Eigen::AngleAxisd(theta, axis).toRotationMatrix();
    sat.clear();
    Eigen::Matrix3d R_cube = R_s;
    if (!base_is_identity) R_cube = R_s * R_base;
    sat.push_back({&m.cube_box, coal::Transform3s(R_cube, center_world), center_world, 0});
    for (int i = 0; i < kCollarSegments; ++i) {
      const Eigen::Vector3d pw = center_world + R_s * col_p[static_cast<std::size_t>(i)];
      sat.push_back({&m.collar_box, coal::Transform3s(R_s * col_R[static_cast<std::size_t>(i)], pw), pw, 1});
    }

    // Pairs in the order of the Python reference: robot part by part, cube then collar. A pair is skipped only if
    // its bounding spheres are disjoint and farther than both running minima (it cannot become a new minimum).
    for (const RobotShape& rs : robot) {
      for (const SatShape& ss : sat) {
        const double lb_mm = ((rs.center_world - ss.center_world).norm() - rs.shape->radius - ss.shape->radius) * 1000.0;
        const double best_mm = in_no_tail ? std::max(out.min_mm, out.min_mm_no_tail) : out.min_mm;
        if (lb_mm >= 0.0 && lb_mm > best_mm) continue;
        const double d_mm = signedDistanceM(*rs.shape->geom, rs.tf, *ss.shape->geom, ss.tf) * 1000.0;
        if (d_mm < out.min_mm) {
          out.min_mm = d_mm;
          out.pair = {robotPartName(rs.part), satPartName(ss.part)};
          out.t_rel_s = t - tau_s;
        }
        if (in_no_tail && d_mm < out.min_mm_no_tail) {
          out.min_mm_no_tail = d_mm;
          out.pair_no_tail = {robotPartName(rs.part), satPartName(ss.part)};
          out.t_rel_no_tail_s = t - tau_s;
        }
      }
    }
  }
  return out;
}

ClearanceEvaluator makeClearanceEvaluator(const ApproachClearanceModel& model, const ClearanceParams& params) {
  validateClearanceParams(params);
  const ApproachClearanceModel* mp = &model;
  return [mp, params](const ClearanceQuery& q) {
    if (!q.trajectory) throw std::invalid_argument("ClearanceQuery: null trajectory.");
    return approachClearance(*mp, *q.trajectory, q.tau_s, q.theta_star_rad, q.omega_rad_s, q.center_world,
                             q.axis_world, params);
  };
}

// ---------------------------------------------------------------------------------------------------------------------
// Hand collision mesh from the URDF
// ---------------------------------------------------------------------------------------------------------------------

std::string handCollisionMeshUri(const std::string& xml) {
  // Find <link ... name="fer_hand" ...> (exact name: "fer_hand_tcp" must not match).
  std::size_t pos = 0;
  std::size_t link_begin = std::string::npos;
  while ((pos = xml.find("<link", pos)) != std::string::npos) {
    const std::size_t tag_end = xml.find('>', pos);
    if (tag_end == std::string::npos) break;
    const std::string tag = xml.substr(pos, tag_end - pos);
    if (tag.find("name=\"fer_hand\"") != std::string::npos) {
      link_begin = pos;
      if (!tag.empty() && tag.back() == '/') {
        throw std::runtime_error("URDF: link fer_hand is empty (no collision mesh).");
      }
      break;
    }
    pos = tag_end;
  }
  if (link_begin == std::string::npos) throw std::runtime_error("URDF: link \"fer_hand\" not found.");
  const std::size_t link_end = xml.find("</link>", link_begin);
  if (link_end == std::string::npos) throw std::runtime_error("URDF: link fer_hand is not closed.");
  const std::string link = xml.substr(link_begin, link_end - link_begin);

  const std::size_t c0 = link.find("<collision");
  if (c0 == std::string::npos) throw std::runtime_error("URDF: link fer_hand has no <collision>.");
  const std::size_t c1 = link.find("</collision>", c0);
  if (c1 == std::string::npos) throw std::runtime_error("URDF: fer_hand <collision> is not closed.");
  const std::string col = link.substr(c0, c1 - c0);
  if (col.find("<origin") != std::string::npos) {
    throw std::runtime_error("URDF: fer_hand <collision> has an <origin>: not supported (the mesh would be misplaced).");
  }
  if (col.find("scale=") != std::string::npos) {
    throw std::runtime_error("URDF: fer_hand collision mesh has a scale: not supported.");
  }
  const std::string key = "filename=\"";
  const std::size_t f0 = col.find(key);
  if (f0 == std::string::npos) throw std::runtime_error("URDF: fer_hand collision has no mesh filename.");
  const std::size_t f1 = col.find('"', f0 + key.size());
  if (f1 == std::string::npos) throw std::runtime_error("URDF: unterminated mesh filename.");
  return col.substr(f0 + key.size(), f1 - f0 - key.size());
}

std::string resolvePackageUri(const std::string& uri, const std::string& ament_prefix_path) {
  const std::string scheme = "package://";
  if (uri.compare(0, scheme.size(), scheme) != 0) {
    throw std::runtime_error("mesh URI '" + uri + "' is not package://");
  }
  const std::string rest = uri.substr(scheme.size());  // pkg/relative/path
  const std::size_t slash = rest.find('/');
  if (slash == std::string::npos) throw std::runtime_error("malformed package URI '" + uri + "'");
  const std::string pkg = rest.substr(0, slash), rel = rest.substr(slash + 1);
  std::stringstream ss(ament_prefix_path);
  std::string prefix, tried;
  while (std::getline(ss, prefix, ':')) {
    if (prefix.empty()) continue;
    const std::string cand = prefix + "/share/" + pkg + "/" + rel;
    if (std::ifstream(cand, std::ios::binary).good()) return cand;
    tried += "\n  " + cand;
  }
  throw std::runtime_error("cannot resolve " + uri + " through AMENT_PREFIX_PATH; tried:" + tried);
}

HandMesh readStlMesh(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open STL " + path);
  const std::string bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  std::vector<std::array<double, 9>> facets;  // 3 vertices per facet
  bool binary = false;
  if (bytes.size() >= 84) {
    std::uint32_t n = 0;
    std::memcpy(&n, bytes.data() + 80, sizeof n);  // little endian (x86 / ARM Linux)
    if (bytes.size() == 84 + static_cast<std::size_t>(n) * 50) {
      binary = true;
      for (std::uint32_t i = 0; i < n; ++i) {
        const char* facet = bytes.data() + 84 + static_cast<std::size_t>(i) * 50;
        float xyz[9];
        std::memcpy(xyz, facet + 12, sizeof xyz);  // skip the normal
        std::array<double, 9> t;
        for (int k = 0; k < 9; ++k) t[static_cast<std::size_t>(k)] = xyz[k];
        facets.push_back(t);
      }
    }
  }
  if (!binary) {  // ASCII STL
    std::istringstream is(bytes);
    std::string tok;
    std::vector<double> v;
    while (is >> tok) {
      if (tok == "vertex") {
        double x, y, z;
        if (!(is >> x >> y >> z)) throw std::runtime_error("malformed ASCII STL " + path);
        v.push_back(x);
        v.push_back(y);
        v.push_back(z);
      }
    }
    if (v.empty() || v.size() % 9 != 0) {
      throw std::runtime_error("STL " + path +
                               ": not a valid ASCII STL and its size does not match a binary STL (84 + 50 n): "
                               "truncated file?");
    }
    for (std::size_t i = 0; i < v.size(); i += 9) {
      std::array<double, 9> t;
      for (std::size_t k = 0; k < 9; ++k) t[k] = v[i + k];
      facets.push_back(t);
    }
  }
  if (facets.empty()) throw std::runtime_error("STL " + path + " has no facets");
  // Merge identical vertices (exact value: float32 vertices shared between facets are bit-identical).
  HandMesh mesh;
  std::map<std::array<double, 3>, std::size_t> index;
  for (const auto& t : facets) {
    std::array<std::size_t, 3> tri{};
    for (std::size_t k = 0; k < 3; ++k) {
      const std::array<double, 3> key{t[3 * k], t[3 * k + 1], t[3 * k + 2]};
      const auto it = index.find(key);
      if (it != index.end()) {
        tri[k] = it->second;
      } else {
        tri[k] = mesh.points.size();
        index.emplace(key, tri[k]);
        mesh.points.emplace_back(key[0], key[1], key[2]);
      }
    }
    mesh.triangles.push_back(tri);
  }
  return mesh;
}

HandMesh loadHandMeshFile(const std::string& path) {
  HandMesh mesh = readStlMesh(path);
  validateHandMesh(mesh);
  return mesh;
}

HandMesh loadHandCollisionMesh(const std::string& urdf_path) {
  std::ifstream f(urdf_path);
  if (!f) throw std::runtime_error("cannot open URDF " + urdf_path);
  std::stringstream buf;
  buf << f.rdbuf();
  const std::string uri = handCollisionMeshUri(buf.str());  // throws on <origin> / scale
  const char* ap = std::getenv("AMENT_PREFIX_PATH");
  if (!ap) throw std::runtime_error("AMENT_PREFIX_PATH is not set: cannot resolve " + uri);
  return loadHandMeshFile(resolvePackageUri(uri, ap));
}

}  // namespace core
}  // namespace satellite_grasp_planner
