// Tests for the approach clearance (core/approach_clearance.hpp) and for the lazy clearance filter
// (s3ClearanceRound). The geometry tests use a synthetic hand hull (a 2 cm cube at the hand origin), so they need
// neither the URDF nor the Franka meshes; the fingers and the satellite are the real boxes.
//
// Geometry of the known cases (satellite at the origin, axis z, face normal x, theta* = 0; cube 0.15, collar on +x):
//  - far hand: nothing within a metre.
//  - "face to face": hand z-axis along +x (R = Ry(90 deg): hand x -> -z, hand y -> y, hand z -> +x), fingers open 0.060.
//    The finger-tip boxes (hand z up to 0.0584 + 0.04525 + 0.00925 = 0.1129, y in [0.060, 0.0752]) are the most +x parts
//    of the gripper; with TCP.x = -0.1845 their +x faces are at x = -0.1845 - 0.1034 + 0.1129 = -0.175, i.e. 0.100 m from
//    the cube's -x face (x = -0.075), which they face within the face extent: signed distance = 100 mm.
//  - overlap: R = I, left tip box centre (0, 0.060 + 0.00758, 0.0584 + 0.04525) placed on the centre of the collar box
//    phi = 0, (0.115, -0.068, 0): penetration.
//  - rotation: same hand as face-to-face with TCP.x = -0.0945 (tip faces at x = -0.085, 10 mm from the cube's -x face);
//    the cube turns by theta(t) = -omega (tau - t) about z, its -x face tilts towards the finger at y = -0.0676.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "satellite_grasp_planner/core/approach_clearance.hpp"
#include "satellite_grasp_planner/core/grasp_s3_selection.hpp"

using namespace satellite_grasp_planner::core;
using haptic_dmp_learning::core::CubeSatelliteModel;

namespace {

constexpr double kDeg = M_PI / 180.0;

CubeSatelliteModel::Params cubeParams() {
  CubeSatelliteModel::Params p;  // defaults: side 0.150, standoff 0.08, R 0.0725, wall 0.009, n = +x
  p.center_world = Eigen::Vector3d::Zero();
  p.axis_world = Eigen::Vector3d::UnitZ();
  return p;
}

/// Synthetic hand: 2 cm box, 8 points / 12 triangles, with three triangles flipped (mixed orientation) to exercise the
/// automatic outward orientation of validateHandMesh.
HandMesh syntheticHand(double half) {
  HandMesh m = makeBoxHandMesh(Eigen::Vector3d(half, half, half));
  for (const std::size_t i : {1u, 4u, 7u}) std::swap(m.triangles[i][1], m.triangles[i][2]);
  return m;
}

ApproachClearanceModel makeModel() { return ApproachClearanceModel(cubeParams(), syntheticHand(0.01)); }

/// Stationary TCP pose over [0, tau] (two samples).
std::vector<ApproachSample> stationary(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, double tau) {
  ApproachSample a, b;
  a.t_s = 0.0;
  b.t_s = tau;
  a.position = b.position = p;
  a.orientation = b.orientation = q;
  return {a, b};
}

ClearanceResult run(const ApproachClearanceModel& m, const std::vector<ApproachSample>& traj, double tau, double omega) {
  return approachClearance(m, traj, tau, /*theta_star=*/0.0, omega, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(),
                           ClearanceParams());
}

const Eigen::Quaterniond kHandAlongPlusX(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()));

}  // namespace

TEST(ApproachClearanceTest, FarHandIsLargeAndPositive) {
  const ApproachClearanceModel m = makeModel();
  const auto r = run(m, stationary(Eigen::Vector3d(2.0, 0.0, 0.0), Eigen::Quaterniond::Identity(), 5.0), 5.0, 0.0);
  EXPECT_GT(r.min_mm, 1000.0);
  EXPECT_GT(r.min_mm_no_tail, 1000.0);
  EXPECT_FALSE(r.pair.robot.empty());
  EXPECT_FALSE(r.pair_no_tail.satellite.empty());
}

TEST(ApproachClearanceTest, FaceToFaceKnownValue) {
  const ApproachClearanceModel m = makeModel();
  const auto r = run(m, stationary(Eigen::Vector3d(-0.1845, 0.0, 0.0), kHandAlongPlusX, 5.0), 5.0, 0.0);
  EXPECT_NEAR(r.min_mm_no_tail, 100.0, 0.01);
  EXPECT_NEAR(r.min_mm, 100.0, 0.01);
  EXPECT_EQ(r.pair_no_tail.satellite, "cubo");
  EXPECT_NE(r.pair_no_tail.robot, "mano");
}

TEST(ApproachClearanceTest, FingerBoxOverlappingCollarBoxIsNegative) {
  const ApproachClearanceModel m = makeModel();
  const Eigen::Vector3d hand_origin(0.115, -0.068 - 0.06758, -0.10365);
  const Eigen::Vector3d tcp = hand_origin + Eigen::Vector3d(0.0, 0.0, kHandTcpOffsetM);
  const auto r = run(m, stationary(tcp, Eigen::Quaterniond::Identity(), 5.0), 5.0, 0.0);
  EXPECT_LT(r.min_mm_no_tail, 0.0);
  EXPECT_EQ(r.pair_no_tail.robot, "dito 1");
  EXPECT_EQ(r.pair_no_tail.satellite, "collare");
}

TEST(ApproachClearanceTest, ClearanceDecreasesWithOmegaForARotatingCollarAgainstAStillFinger) {
  // Monotonic in omega only while the shapes stay SEPARATED: a signed distance below zero is the penetration depth
  // (minimum translation), which is not monotonic in the rotation angle once the cube corner has entered the finger box
  // (the escape direction switches). Here theta(0) = -10 omega tops out at -4 deg, the corner stays >= ~4.9 mm from the
  // finger tip (x of the corner = -0.075 (cos a + sin a) against the tip face at -0.085).
  const ApproachClearanceModel m = makeModel();
  const auto traj = stationary(Eigen::Vector3d(-0.0945, 0.0, 0.0), kHandAlongPlusX, 10.0);
  double previous = std::numeric_limits<double>::infinity();
  for (const double omega_deg_s : {0.0, 0.1, 0.2, 0.3, 0.4}) {
    const double c = run(m, traj, 10.0, omega_deg_s * kDeg).min_mm_no_tail;
    if (omega_deg_s == 0.0) {
      EXPECT_NEAR(c, 10.0, 0.05);
    }
    EXPECT_GT(c, 0.0) << "omega " << omega_deg_s << " deg/s: the test geometry must stay separated";
    EXPECT_LT(c, previous) << "omega " << omega_deg_s << " deg/s";
    previous = c;
  }
}

TEST(ApproachClearanceTest, NoTailWindowExcludesTheLastTwoSeconds) {
  // The cube turns towards the finger and reaches the minimum at the END of the window: only min_mm sees it.
  // Tip face at x = TCP.x + 0.0095 = -0.105 (30 mm from the cube face at theta = 0): the corner of the cube face
  // (x = -0.075 (cos a + sin a), y inside the tip box for a <= 10 deg) stays separated, so the distance decreases
  // monotonically with a = |theta| and the closest instant is the last sample of each window.
  const ApproachClearanceModel m = makeModel();
  const auto traj = stationary(Eigen::Vector3d(-0.1145, 0.0, 0.0), kHandAlongPlusX, 10.0);
  // theta(t) = theta* - omega (tau - t) with theta* = -10 deg and omega = -1 deg/s: theta = -t deg (0 at t = 0,
  // -10 deg at t = tau, the closest); at t = tau - 2 s it is -8 deg.
  const auto r = approachClearance(m, traj, 10.0, -10.0 * kDeg, -1.0 * kDeg, Eigen::Vector3d::Zero(),
                                   Eigen::Vector3d::UnitZ(), ClearanceParams());
  EXPECT_GT(r.min_mm, 0.0);
  EXPECT_LT(r.min_mm, r.min_mm_no_tail);
  EXPECT_NEAR(r.t_rel_s, 0.0, 1e-9);                              // t_rel = t - tau: 0 at contact
  EXPECT_NEAR(r.t_rel_no_tail_s, -kClearanceTailExcludeS, 1e-6);  // last sample of [0, tau - 2 s]
}

TEST(ApproachClearanceTest, InvalidInputsThrow) {
  const ApproachClearanceModel m = makeModel();
  const auto good = stationary(Eigen::Vector3d(2.0, 0.0, 0.0), Eigen::Quaterniond::Identity(), 5.0);
  const Eigen::Vector3d c = Eigen::Vector3d::Zero(), a = Eigen::Vector3d::UnitZ();
  EXPECT_NO_THROW(approachClearance(m, good, 5.0, 0.0, 0.0, c, a, ClearanceParams()));
  EXPECT_THROW(approachClearance(m, good, 2.0, 0.0, 0.0, c, a, ClearanceParams()), std::invalid_argument);  // tau <= 2 s
  EXPECT_THROW(approachClearance(m, good, 6.0, 0.0, 0.0, c, a, ClearanceParams()), std::invalid_argument);  // does not cover tau
  EXPECT_THROW(approachClearance(m, good, 5.0, std::nan(""), 0.0, c, a, ClearanceParams()), std::invalid_argument);
  EXPECT_THROW(approachClearance(m, {good[0]}, 5.0, 0.0, 0.0, c, a, ClearanceParams()), std::invalid_argument);
  ClearanceParams bad;
  bad.dt_s = 0.0;
  EXPECT_THROW(approachClearance(m, good, 5.0, 0.0, 0.0, c, a, bad), std::invalid_argument);
  bad = ClearanceParams();
  bad.gripper_open_m = 0.09;
  EXPECT_THROW(validateClearanceParams(bad), std::invalid_argument);
  bad.gripper_open_m = -0.001;
  EXPECT_THROW(validateClearanceParams(bad), std::invalid_argument);
  EXPECT_THROW(ApproachClearanceModel(cubeParams(), HandMesh()), std::invalid_argument);
}

// ---- lazy filter (pure) ----

namespace {
struct RoundFixture {
  std::vector<double> j, value;
  std::vector<char> alive, done;
  std::vector<double> cache;
  std::vector<int> calls;
  RoundFixture(std::vector<double> j_, std::vector<double> v) : j(std::move(j_)), value(std::move(v)) {
    alive.assign(j.size(), 1);
    done.assign(j.size(), 0);
    cache.assign(j.size(), 0.0);
    calls.assign(j.size(), 0);
  }
  bool round(double thr, double band) {
    return s3ClearanceRound(
        j, alive, done, cache,
        [this](std::size_t i) {
          ++calls[i];
          return value[i];
        },
        thr, band);
  }
};
}  // namespace

TEST(ClearanceRoundTest, SkipsViolatorsFixesJStarAndEvaluatesOnlyTheBand) {
  RoundFixture f({1.0, 2.0, 3.0, 10.0}, {0.5, 5.0, 1.0, 9.0});
  ASSERT_TRUE(f.round(2.0, 3.0));  // J*: idx1 (J 2); band J <= 5: idx2 too; idx3 (J 10) is never evaluated
  EXPECT_EQ(f.alive, (std::vector<char>{0, 1, 0, 1}));
  EXPECT_EQ(f.calls, (std::vector<int>{1, 1, 1, 0}));
  EXPECT_EQ(f.done, (std::vector<char>{1, 1, 1, 0}));
}

TEST(ClearanceRoundTest, ScansInJOrderNotInIndexOrder) {
  RoundFixture f({5.0, 1.0, 3.0}, {9.0, 9.0, 9.0});
  ASSERT_TRUE(f.round(2.0, 0.0));  // all admissible: only the smallest J is evaluated (band 0, J* = 1)
  EXPECT_EQ(f.calls, (std::vector<int>{0, 1, 0}));
}

TEST(ClearanceRoundTest, NoAdmissibleFinalistKillsEverythingAndReportsFalse) {
  RoundFixture f({1.0, 2.0}, {1.0, 1.99});
  EXPECT_FALSE(f.round(2.0, 3.0));
  EXPECT_EQ(f.alive, (std::vector<char>{0, 0}));
  EXPECT_EQ(f.calls, (std::vector<int>{1, 1}));
}

TEST(ClearanceRoundTest, NanAndNonFiniteJAreHandled) {
  RoundFixture f({1.0, std::numeric_limits<double>::infinity(), 2.0}, {std::nan(""), 9.0, 9.0});
  ASSERT_TRUE(f.round(2.0, 1.0));  // NaN clearance = violation; the infinite-J entry is never evaluated
  EXPECT_EQ(f.alive, (std::vector<char>{0, 1, 1}));
  EXPECT_EQ(f.calls[1], 0);
}

TEST(ClearanceRoundTest, EvaluationsAreCachedAcrossRounds) {
  RoundFixture f({1.0, 2.0, 30.0}, {9.0, 9.0, 9.0});
  ASSERT_TRUE(f.round(2.0, 3.0));
  f.alive[0] = 0;  // e.g. the post-hoc velocity check discarded the winner
  ASSERT_TRUE(f.round(2.0, 3.0));
  EXPECT_EQ(f.calls[1], 1);  // already evaluated
  f.alive[1] = 0;
  ASSERT_TRUE(f.round(2.0, 3.0));  // now the J = 30 finalist enters
  EXPECT_EQ(f.calls[2], 1);
  EXPECT_THROW(s3ClearanceRound(
                   f.j, f.alive, f.done, f.cache, [](std::size_t) { return 0.0; }, -1.0, 1.0),
               std::invalid_argument);
  std::vector<char> shorter(2, 1);
  EXPECT_THROW(s3ClearanceRound(
                   f.j, shorter, f.done, f.cache, [](std::size_t) { return 0.0; }, 1.0, 1.0),
               std::invalid_argument);
}

// ---- parameters ----

TEST(ClearanceParamsTest, DefaultsAreValid) {
  S3Params p;
  EXPECT_FALSE(p.use_clearance);
  const double q[7] = {0.0, -0.785398163397, 0.0, -2.356194490192, 0.0, 1.570796326795, 0.785398163397};
  p.rollout_q_ref.assign(q, q + 7);
  EXPECT_NO_THROW(validateS3Params(p));
  p.use_clearance = true;
  EXPECT_NO_THROW(validateS3Params(p));
  auto bad = [&](auto mutate) {
    S3Params b = p;
    mutate(b);
    EXPECT_THROW(validateS3Params(b), std::invalid_argument);
  };
  bad([](S3Params& b) { b.min_clearance_mm = -0.1; });
  bad([](S3Params& b) { b.min_clearance_mm = std::nan(""); });
  bad([](S3Params& b) { b.clearance_dt_s = 0.0; });
  bad([](S3Params& b) { b.clearance_dt_s = std::numeric_limits<double>::infinity(); });
  bad([](S3Params& b) { b.gripper_open_m = 0.081; });
  bad([](S3Params& b) { b.gripper_open_m = -0.01; });
}

// ---- URDF / STL helpers ----

TEST(HandMeshTest, UriFromUrdfAndPackageResolution) {
  const std::string urdf =
      "<robot><link name=\"fer_hand_tcp\"/><link name=\"fer_hand\"><visual><geometry><mesh filename=\"package://p/v.dae\"/>"
      "</geometry></visual><collision name=\"c\"><geometry><mesh filename=\"package://franka_description/meshes/hand.stl\"/>"
      "</geometry></collision></link></robot>";
  EXPECT_EQ(handCollisionMeshUri(urdf), "package://franka_description/meshes/hand.stl");
  EXPECT_THROW(handCollisionMeshUri("<robot><link name=\"fer_hand_tcp\"/></robot>"), std::runtime_error);
  std::string with_origin = urdf;
  with_origin.replace(with_origin.find("<collision name=\"c\">"), 20, "<collision name=\"c\"><origin xyz=\"0 0 1\"/>");
  EXPECT_THROW(handCollisionMeshUri(with_origin), std::runtime_error);

  const std::string root = testing::TempDir() + "clearance_pkg_test";
  const std::string dir = root + "/share/franka_description/meshes";
  ASSERT_EQ(std::system(("mkdir -p " + dir).c_str()), 0);
  { std::ofstream(dir + "/hand.stl") << "x"; }
  EXPECT_EQ(resolvePackageUri("package://franka_description/meshes/hand.stl", "/nonexistent:" + root),
            root + "/share/franka_description/meshes/hand.stl");
  EXPECT_THROW(resolvePackageUri("package://franka_description/meshes/hand.stl", "/nonexistent"), std::runtime_error);
  EXPECT_THROW(resolvePackageUri("file:///x.stl", root), std::runtime_error);
}

namespace {

/// Unit-ish cube (half 0.5) as STL. Binary: 80-byte header, uint32 n, n x (normal, 3 vertices, uint16).
void writeBinaryStl(const std::string& path, const HandMesh& m, std::size_t drop_bytes = 0) {
  std::string b(80, ' ');
  const std::uint32_t n = static_cast<std::uint32_t>(m.triangles.size());
  b.append(reinterpret_cast<const char*>(&n), 4);
  for (const auto& t : m.triangles) {
    const float zero[3] = {0.f, 0.f, 0.f};
    b.append(reinterpret_cast<const char*>(zero), 12);
    for (const std::size_t k : t) {
      const float v[3] = {static_cast<float>(m.points[k].x()), static_cast<float>(m.points[k].y()),
                          static_cast<float>(m.points[k].z())};
      b.append(reinterpret_cast<const char*>(v), 12);
    }
    b.append(2, '\0');
  }
  b.resize(b.size() - drop_bytes);
  std::ofstream(path, std::ios::binary) << b;
}

void writeAsciiStl(const std::string& path, const HandMesh& m) {
  std::ofstream f(path);
  f << "solid cube\n";
  for (const auto& t : m.triangles) {
    f << "facet normal 0 0 0\nouter loop\n";
    for (const std::size_t k : t) f << "vertex " << m.points[k].x() << " " << m.points[k].y() << " " << m.points[k].z() << "\n";
    f << "endloop\nendfacet\n";
  }
  f << "endsolid cube\n";
}

}  // namespace

TEST(HandMeshTest, BinaryStlCubeHasEightUniqueVerticesAndTwelveTriangles) {
  const std::string path = testing::TempDir() + "clearance_cube_bin.stl";
  writeBinaryStl(path, makeBoxHandMesh(Eigen::Vector3d(0.5, 0.5, 0.5)));
  const HandMesh m = loadHandMeshFile(path);
  EXPECT_EQ(m.points.size(), 8u);
  EXPECT_EQ(m.triangles.size(), 12u);
  EXPECT_NE(describeHandMesh(m).find("vertices=8 triangles=12"), std::string::npos);
}

TEST(HandMeshTest, AsciiStlCubeHasEightUniqueVerticesAndTwelveTriangles) {
  const std::string path = testing::TempDir() + "clearance_cube_ascii.stl";
  writeAsciiStl(path, makeBoxHandMesh(Eigen::Vector3d(0.5, 0.5, 0.5)));
  const HandMesh m = readStlMesh(path);
  EXPECT_EQ(m.points.size(), 8u);
  EXPECT_EQ(m.triangles.size(), 12u);
  EXPECT_NO_THROW(loadHandMeshFile(path));
}

TEST(HandMeshTest, TruncatedBinaryStlThrows) {
  const std::string path = testing::TempDir() + "clearance_cube_trunc.stl";
  writeBinaryStl(path, makeBoxHandMesh(Eigen::Vector3d(0.5, 0.5, 0.5)), /*drop_bytes=*/30);
  EXPECT_THROW(readStlMesh(path), std::runtime_error);
}

TEST(HandMeshTest, MixedOrientationIsMadeOutward) {
  HandMesh m = syntheticHand(0.5);
  validateHandMesh(m);
  for (const auto& t : m.triangles) {
    const Eigen::Vector3d n = (m.points[t[1]] - m.points[t[0]]).cross(m.points[t[2]] - m.points[t[0]]);
    EXPECT_GT(n.dot(m.points[t[0]]), 0.0);  // box centred at the origin
  }
}

namespace {
/// Box with half extents 0.05 whose corner 7 (+,+,+) is pushed out along +x by @p d: the triangle {1,3,5} (plane x = h)
/// then has that point at exactly n.(p - a) = d, and no other triangle sees more, so the convexity violation is d.
HandMesh boxWithCornerPushedOut(double d) {
  HandMesh m = makeBoxHandMesh(Eigen::Vector3d(0.05, 0.05, 0.05));
  m.points[7].x() += d;
  return m;
}
}  // namespace

TEST(HandMeshTest, NonConvexMeshThrows) {
  HandMesh m = boxWithCornerPushedOut(5e-3);  // 5 mm, well above the 1.0 mm tolerance
  try {
    validateHandMesh(m);
    ADD_FAILURE() << "expected an exception";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("not convex"), std::string::npos) << e.what();
    EXPECT_NE(std::string(e.what()).find("max violation 5"), std::string::npos) << e.what();
  }
}

TEST(HandMeshTest, ViolationOfTheRealMeshSizeDoesNotThrow) {
  HandMesh m = boxWithCornerPushedOut(0.74e-3);  // the measured violation of the real hand.stl
  EXPECT_NO_THROW(validateHandMesh(m));
  EXPECT_NEAR(m.convexity_violation_m, 0.74e-3, 1e-9);
  EXPECT_NE(describeHandMesh(m).find("max_convexity_violation_mm=0.74"), std::string::npos) << describeHandMesh(m);
}

TEST(HandMeshTest, ViolationAboveTheToleranceThrows) {
  HandMesh m = boxWithCornerPushedOut(1.5e-3);
  EXPECT_THROW(validateHandMesh(m), std::invalid_argument);
  EXPECT_NEAR(m.convexity_violation_m, 1.5e-3, 1e-9);  // measured and stored even when it throws
  HandMesh ok = boxWithCornerPushedOut(1.5e-3);
  EXPECT_NO_THROW(validateHandMesh(ok, 2.0e-3));  // the tolerance is a parameter
}

TEST(HandMeshTest, DescribeReportsNotMeasuredForAnUnvalidatedMesh) {
  const HandMesh m = makeBoxHandMesh(Eigen::Vector3d(0.01, 0.01, 0.01));
  EXPECT_NE(describeHandMesh(m).find("max_convexity_violation_mm=n/a"), std::string::npos);
}

TEST(HandMeshTest, DegenerateTriangleThrows) {
  HandMesh m = makeBoxHandMesh(Eigen::Vector3d(0.5, 0.5, 0.5));
  m.triangles[3] = {0, 0, 1};
  try {
    validateHandMesh(m);
    ADD_FAILURE() << "expected an exception";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("degenerate"), std::string::npos) << e.what();
  }
}

// Rigid invariance: the clearance depends only on the relative geometry. Applying the same rigid motion x -> R x + shift to
// the hand trajectory (positions and orientations), the satellite centre, the spin axis and the collar normal gives the
// same distances, pair and time (the hand frames and the satellite shapes move together).
TEST(ApproachClearanceTest, RigidMotionOfTheWholeSceneLeavesTheClearanceUnchanged) {
  constexpr double tau = 8.0, theta_star = 0.4, omega = -5.0 * kDeg;
  const Eigen::Quaterniond qa = kHandAlongPlusX * Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond qb = kHandAlongPlusX * Eigen::Quaterniond(Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitZ()));
  ApproachSample a, b;
  a.t_s = 0.0;
  b.t_s = tau;
  a.position = Eigen::Vector3d(-0.60, 0.05, 0.03);
  b.position = Eigen::Vector3d(-0.19, 0.013, 0.021);
  a.orientation = qa;
  b.orientation = qb;
  const ApproachClearanceModel m0(cubeParams(), syntheticHand(0.01));
  const ClearanceResult r0 = approachClearance(m0, {a, b}, tau, theta_star, omega, Eigen::Vector3d::Zero(),
                                               Eigen::Vector3d::UnitZ(), ClearanceParams());
  ASSERT_TRUE(std::isfinite(r0.min_mm));

  const Eigen::Vector3d shift(0.31, -0.22, 0.47);
  std::vector<double> min_mm_values;
  int orientation_sensitive = 0;  // tilts for which ignoring the rotation of the satellite frame changes the result
  int tilt_index = -1;
  for (const auto& tilt : {std::pair<double, double>{0.0, 0.0}, {5.0, 0.0}, {20.0, 33.0}, {35.0, -120.0}, {80.0, 250.0}}) {
    ++tilt_index;
    SCOPED_TRACE(testing::Message() << "tilt index " << tilt_index << " alpha " << tilt.first << " deg beta " << tilt.second << " deg");
    const double al = tilt.first * kDeg, be = tilt.second * kDeg;
    const Eigen::Matrix3d R =
        (Eigen::AngleAxisd(al, Eigen::Vector3d(-std::sin(be), std::cos(be), 0.0)) * Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    const Eigen::Quaterniond qR(R);
    CubeSatelliteModel::Params p = cubeParams();
    p.axis_world = R * Eigen::Vector3d::UnitZ();
    p.face_normal_body = R * Eigen::Vector3d::UnitX();
    const ApproachClearanceModel m1(p, syntheticHand(0.01));
    ApproachSample a2 = a, b2 = b;
    a2.position = R * a.position + shift;
    b2.position = R * b.position + shift;
    a2.orientation = (qR * qa).normalized();
    b2.orientation = (qR * qb).normalized();
    const ClearanceResult r1 = approachClearance(m1, {a2, b2}, tau, theta_star, omega, shift, p.axis_world, ClearanceParams());
    // non-vacuity: a finite, positive, bounded clearance (a degenerate scene would pass any invariance check)
    for (const double v : {r1.min_mm, r1.min_mm_no_tail}) {
      EXPECT_TRUE(std::isfinite(v));
      EXPECT_GT(v, 0.0);
      EXPECT_LT(v, 1000.0);
    }
    min_mm_values.push_back(r1.min_mm);
    // The same moved hand against the NOMINAL satellite frame (n = +x, a = +z): if this equals r0 the rotation would not
    // be tested at all.
    const ClearanceResult r_nominal_frame =
        approachClearance(m0, {a2, b2}, tau, theta_star, omega, shift, Eigen::Vector3d::UnitZ(), ClearanceParams());
    if (std::abs(r_nominal_frame.min_mm - r0.min_mm) > 1e-2) ++orientation_sensitive;
    EXPECT_NEAR(r1.min_mm, r0.min_mm, 1e-3) << "alpha " << tilt.first << " beta " << tilt.second;
    EXPECT_NEAR(r1.min_mm_no_tail, r0.min_mm_no_tail, 1e-3);
    EXPECT_NEAR(r1.t_rel_s, r0.t_rel_s, 1e-9);
    EXPECT_NEAR(r1.t_rel_no_tail_s, r0.t_rel_no_tail_s, 1e-9);
    EXPECT_EQ(r1.pair.str(), r0.pair.str());
    EXPECT_EQ(r1.pair_no_tail.str(), r0.pair_no_tail.str());
  }
  const auto mm = std::minmax_element(min_mm_values.begin(), min_mm_values.end());
  std::cout << "[RigidMotion] min_mm over the tilts: min " << *mm.first << " max " << *mm.second << " (r0 " << r0.min_mm
            << "), orientation-sensitive tilts " << orientation_sensitive << "/" << min_mm_values.size() << std::endl;
  EXPECT_GE(orientation_sensitive, 2);
}

// Known face of the cube, nominal scene and the same scene rotated: face to face at 10 mm (see the header: hand z along +x,
// fingers open 0.060, TCP.x = -0.0945, tip faces at x = -0.085, cube -x face at x = -0.075), spin omega = 0. The cube box
// must turn with the satellite frame (n, a x n, a): with a world-aligned cube box the tilted scenes would give another value.
TEST(ApproachClearanceTest, KnownCubeFaceDistanceIsTheSameInARotatedScene) {
  constexpr double tau = 5.0;
  const Eigen::Vector3d tcp(-0.0945, 0.0, 0.0);
  const ApproachClearanceModel m0 = makeModel();
  const ClearanceResult r0 = run(m0, stationary(tcp, kHandAlongPlusX, tau), tau, 0.0);
  EXPECT_NEAR(r0.min_mm, 10.0, 0.01);
  EXPECT_NEAR(r0.min_mm_no_tail, 10.0, 0.01);
  EXPECT_EQ(r0.pair.satellite, "cubo");
  EXPECT_EQ(r0.pair_no_tail.satellite, "cubo");

  const Eigen::Vector3d shift(0.31, -0.22, 0.47);
  const std::vector<Eigen::Matrix3d> rotations = {
      Eigen::AngleAxisd(35.0 * kDeg, Eigen::Vector3d::UnitY()).toRotationMatrix(),
      Eigen::AngleAxisd(60.0 * kDeg, Eigen::Vector3d(1.0, 1.0, 1.0).normalized()).toRotationMatrix(),
      (Eigen::AngleAxisd(80.0 * kDeg, Eigen::Vector3d(-0.5, 0.8, 0.0).normalized()) * Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitX()))
          .toRotationMatrix()};
  int index = -1;
  for (const Eigen::Matrix3d& R : rotations) {
    ++index;
    SCOPED_TRACE(testing::Message() << "rotation index " << index);
    CubeSatelliteModel::Params p = cubeParams();
    p.axis_world = R * Eigen::Vector3d::UnitZ();
    p.face_normal_body = R * Eigen::Vector3d::UnitX();
    p.center_world = shift;
    const ApproachClearanceModel m1(p, syntheticHand(0.01));
    const std::vector<ApproachSample> traj = stationary(R * tcp + shift, Eigen::Quaterniond(R) * kHandAlongPlusX, tau);
    const ClearanceResult r1 = approachClearance(m1, traj, tau, 0.0, 0.0, shift, p.axis_world, ClearanceParams());
    EXPECT_NEAR(r1.min_mm, 10.0, 0.01);
    EXPECT_NEAR(r1.min_mm_no_tail, r0.min_mm_no_tail, 1e-3);
    // Only the satellite body is compared, not the finger index: the two fingers are symmetric about the hand x-z plane
    // and both at 10 mm from the face, so which one is reported as the closest is decided by rounding.
    EXPECT_EQ(r1.pair.satellite, "cubo");
    EXPECT_EQ(r1.pair_no_tail.satellite, "cubo");
  }
}
