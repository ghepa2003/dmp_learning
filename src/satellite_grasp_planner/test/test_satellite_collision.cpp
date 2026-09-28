// Tests for checkSatelliteCollision(). Like test_kinematic_feasibility.cpp these need Pinocchio, Coal
// and $HOME/thesis_ws/fer_flat_effort.urdf.

#include <gtest/gtest.h>

#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <utility>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <coal/distance.h>
#include <coal/math/transform.h>
#include <coal/shape/geometric_shapes.h>

#include "probe_gate.hpp"
#include "test_fixtures.hpp"
#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/math_utils.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "haptic_dmp_learning/core/types.hpp"
#include "satellite_grasp_planner/core/joint_path_simulator.hpp"
#include "satellite_grasp_planner/core/satellite_collision.hpp"

using satellite_grasp_planner::core::checkSatelliteCollision;
using haptic_dmp_learning::core::CubeSatelliteModel;
using RobotModel = franka_cartesian_control::core::RobotModel;
using satellite_grasp_planner::test_fixtures::makeModelBParams;
using satellite_grasp_planner::test_fixtures::resetRobotToReady;

namespace {

std::unique_ptr<RobotModel> loadPandaRobotModel() {
    const char* home = std::getenv("HOME");
    const std::string urdf_path =
        std::string(home ? home : "/root") + "/thesis_ws/fer_flat_effort.urdf";
    std::ifstream f(urdf_path);
    if (!f.is_open()) {
        ADD_FAILURE() << "Could not open URDF (required test fixture): " << urdf_path;
        return nullptr;
    }
    std::stringstream buffer;
    buffer << f.rdbuf();
    std::vector<std::string> joint_names;
    for (int i = 1; i <= 7; ++i) joint_names.push_back("fer_joint" + std::to_string(i));
    return std::make_unique<RobotModel>(buffer.str(), joint_names, "fer_hand_tcp");
}

}  // namespace

TEST(SatelliteCollisionTest, FarSatelliteIsFeasibleWithLargePositiveDistance) {
    auto model = loadPandaRobotModel();
    ASSERT_NE(model, nullptr);
    resetRobotToReady(model);

    CubeSatelliteModel::Params p;
    p.center_world = Eigen::Vector3d(3.0, 3.0, 1.0);  // metres away from the arm
    const CubeSatelliteModel satellite(p);

    const auto res = checkSatelliteCollision(*model, satellite, 0.0);
    EXPECT_TRUE(res.feasible);
    EXPECT_GT(res.min_distance_arm_m, 1.0);

    EXPECT_GT(res.min_distance_cube_m, 1.0);
    EXPECT_FALSE(res.closest_capsule_cube.empty());

    EXPECT_EQ(res.per_capsule.size(), 12u);
    for (const auto& c : res.per_capsule) {
        EXPECT_EQ(c.expected_contact, c.name == "fer_hand#1") << c.name;
    }
}

// Deterministic penetration: the cylinder is placed so that the fer_link4 frame origin lies on
// its axis at mid-height (theta = 0: n = +X, axis = +Z, cylinder spans [side/2, side/2 + standoff]
// along +X from center). The fer_link4 capsule is centered exactly at the frame origin (local offset
// 0, radius 0.06), so its center is at zero distance from the cylinder axis and inside it: the
// penetration does not depend on the capsule orientation or on the link frame's rotation.
TEST(SatelliteCollisionTest, ArmLinkInsideCylinderIsInfeasibleWithNegativeDistance) {
    auto model = loadPandaRobotModel();
    ASSERT_NE(model, nullptr);
    resetRobotToReady(model);
    const Eigen::Vector3d link4 = model->framePose("fer_link4").position;

    CubeSatelliteModel::Params p;
    p.center_world = link4 - (p.cube_side_m / 2.0 + p.standoff_m / 2.0) * Eigen::Vector3d::UnitX();
    const CubeSatelliteModel satellite(p);

    const auto res = checkSatelliteCollision(*model, satellite, 0.0);
    EXPECT_FALSE(res.feasible);
    EXPECT_LT(res.min_distance_arm_m, 0.0);
}

// Cube body: fer_link4's capsule is centered exactly on the frame origin, so a cube centered there
// contains it (signed distance < 0, independent of link orientation).
TEST(SatelliteCollisionTest, CubeCenteredOnLink4IsInfeasibleWithNegativeCubeDistance) {
    auto model = loadPandaRobotModel();
    ASSERT_NE(model, nullptr);
    resetRobotToReady(model);
    CubeSatelliteModel::Params p;
    p.center_world = model->framePose("fer_link4").position;
    const CubeSatelliteModel satellite(p);

    const auto res = checkSatelliteCollision(*model, satellite, 0.0);
    EXPECT_LT(res.min_distance_cube_m, 0.0);
    EXPECT_FALSE(res.feasible);
}

// The cube is symmetric under a 90 deg rotation about its own axis: same arm pose, same distance.
TEST(SatelliteCollisionTest, CubeDistanceInvariantToQuarterTurnAboutAxis) {
    auto model = loadPandaRobotModel();
    ASSERT_NE(model, nullptr);
    resetRobotToReady(model);
    CubeSatelliteModel::Params p;
    p.center_world = model->framePose("fer_link4").position + Eigen::Vector3d(0.25, 0.05, 0.0);
    const CubeSatelliteModel satellite(p);

    const auto r0 = checkSatelliteCollision(*model, satellite, 0.0);
    const auto r90 = checkSatelliteCollision(*model, satellite, M_PI / 2.0);
    EXPECT_TRUE(std::isfinite(r0.min_distance_cube_m));
    EXPECT_NEAR(r0.min_distance_cube_m, r90.min_distance_cube_m, 1e-9);
}

// The cube box frame must be the graspPoseAt body frame: the cylinder base center lies ON the
// face (signed box distance 0) and the top center is exactly standoff_m outside it. Uses the same
// cubeCenterWorld()/cubeRotationWorld() that checkSatelliteCollision passes to coal::Box.
// Params are those of test_cube_satellite_model's generalParams() (tilted axis, off-origin center).
TEST(SatelliteCollisionTest, CubeBoxFrameMatchesCylinderEndpoints) {
    CubeSatelliteModel::Params p;
    p.center_world = Eigen::Vector3d(0.45, -0.05, 0.35);
    p.axis_world = Eigen::Vector3d(1.0, 2.0, 3.0);
    p.face_normal_body = Eigen::Vector3d(2.0, -1.0, 0.0);
    const CubeSatelliteModel m(p);

    auto boxSignedDistance = [&](const Eigen::Vector3d& world_pt, double theta) {
        const Eigen::Vector3d local =
            m.cubeRotationWorld(theta).transpose() * (world_pt - m.cubeCenterWorld());
        const Eigen::Vector3d q = local.cwiseAbs() - Eigen::Vector3d::Constant(m.cubeSideM() / 2.0);
        return q.cwiseMax(0.0).norm() + std::min(q.maxCoeff(), 0.0);
    };
    for (double deg : {0.0, 60.0, -100.0}) {
        const double theta = deg * M_PI / 180.0;
        EXPECT_NEAR(boxSignedDistance(m.cylinderBaseCenterWorld(theta), theta), 0.0, 1e-9)
            << "theta=" << deg;
        EXPECT_NEAR(boxSignedDistance(m.cylinderTopCenterWorld(theta), theta), p.standoff_m, 1e-9)
            << "theta=" << deg;
    }
}

TEST(SatelliteCollisionTest, MarginDecidesFeasibility) {
    auto model = loadPandaRobotModel();
    ASSERT_NE(model, nullptr);
    resetRobotToReady(model);
    CubeSatelliteModel::Params p;
    p.center_world = Eigen::Vector3d(3.0, 3.0, 1.0);
    const CubeSatelliteModel satellite(p);

    const auto res = checkSatelliteCollision(*model, satellite, 0.0);
    // feasible now needs BOTH distances >= margin, so the binding one is the smaller of the two
    // (here the cube, which is nearer to the arm than the cylinder sticking out of its far face).
    const double d_min = std::min(res.min_distance_arm_m, res.min_distance_cube_m);
    EXPECT_TRUE(checkSatelliteCollision(*model, satellite, 0.0, d_min - 1e-9).feasible);
    EXPECT_FALSE(checkSatelliteCollision(*model, satellite, 0.0, d_min + 1e-3).feasible);
}

// EMPIRICAL probe, deliberately without an expected outcome. It reaches a real grasp pose from
// CubeSatelliteModel::graspPoseAt() with the same mechanism as checkKinematicFeasibility()
// (ProDMP position rollout + shortest-path SLERP + JointPathSimulator, NullspaceBiasedResolution),
// then runs checkSatelliteCollision() on the final configuration and REPORTS the result. Purpose:
// see whether the hand capsules make a nominal grasp pose look like a collision (tcp on the top rim,
// hand capsule reaching ~1.7 cm past the tcp plane). Read the printed line / recorded properties.
// Only sanity is asserted (finite numbers, non-empty capsule name).
TEST(SatelliteCollisionProbe, ReportsCollisionDistanceAtRealGraspPose) {
    SKIP_UNLESS_PROBES_ENABLED();
    auto model_owner = loadPandaRobotModel();
    ASSERT_NE(model_owner, nullptr);
    auto model = std::shared_ptr<RobotModel>(std::move(model_owner));

    // Production scenario: top face (n = +Z), spin axis +Y, so the approach axis z_tcp = -Z points
    // down onto the collar rim.
    CubeSatelliteModel::Params p;
    p.center_world = Eigen::Vector3d(0.3928, -0.1650, 0.1188);
    p.face_normal_body = Eigen::Vector3d(0.0, 0.0, 1.0);
    p.axis_world = Eigen::Vector3d(0.0, 1.0, 0.0);
    const CubeSatelliteModel satellite(p);

    const std::vector<std::pair<const char*, haptic_dmp_learning::core::GraspPointId>> points = {
        {"kP0", haptic_dmp_learning::core::GraspPointId::kP0},
        {"kP90", haptic_dmp_learning::core::GraspPointId::kP90},
        {"kP180", haptic_dmp_learning::core::GraspPointId::kP180},
        {"kP270", haptic_dmp_learning::core::GraspPointId::kP270}};
    const double theta = 0.0;
    const double dt = 0.005;

    // Production ProDMP template (n_basis=80, tau ~61 s). Start/goal are overridden per rollout.
    // Path: $GRASP_PROBE_WEIGHTS, else the n80 fit run below (identical to runs/20260915_090358_*).
    SKIP_UNLESS_PRODUCTION_WEIGHTS(weights_path);
    const haptic_dmp_learning::core::ProDMP tmpl =
        haptic_dmp_learning::core::prodmp_io::loadProDmpFromYaml(weights_path);
    std::cout << "[probe3] template " << weights_path << " tau=" << tmpl.tau() << std::endl;

    // Expected grasp points (theta = 0) as a guard that the scenario is the intended one.
    const std::vector<Eigen::Vector3d> expected = {
        {0.3428, -0.165, 0.2988}, {0.3928, -0.115, 0.2988},
        {0.4428, -0.165, 0.2988}, {0.3928, -0.215, 0.2988}};
    for (std::size_t k = 0; k < points.size(); ++k) {
        const Eigen::Vector3d got = satellite.graspPoseAt(points[k].second, theta).position_world;
        ASSERT_LT((got - expected[k]).norm(), 1e-3)
            << points[k].first << " grasp point " << got.transpose() << " != expected "
            << expected[k].transpose();
    }

    const RobotModel::JointVector q0 = RobotModel::readyPose();
    model->update(q0, RobotModel::JointVector::Zero());
    const Eigen::Vector3d p0 = model->eePosition();
    const Eigen::Quaterniond quat0 = model->eeOrientation();

    // Same reach mechanism as before, factored so a shifted grasp pose can be reached too.
    auto reach = [&](const Eigen::Vector3d& goal, const Eigen::Quaterniond& q_goal) {
        haptic_dmp_learning::core::ProDMP prodmp = tmpl;
        prodmp.setRelativeGoal(false);
        prodmp.setInitialConditions(0.0, p0, Eigen::Vector3d::Zero());
        prodmp.setGoal(goal);
        const int n_steps = static_cast<int>(std::ceil(prodmp.tau() / dt));
        std::vector<satellite_grasp_planner::core::CartesianSample> traj;
        for (int i = 0; i < n_steps; ++i) {
            satellite_grasp_planner::core::CartesianSample cs;
            cs.position = prodmp.step(dt);
            const double s = std::min(1.0, (i + 1) * dt / prodmp.tau());
            cs.orientation = haptic_dmp_learning::core::slerpShortestPath(quat0, q_goal, s);
            traj.push_back(cs);
        }
        satellite_grasp_planner::core::JointPathSimulator::Params sp;
        sp.dt = dt;
        sp.strategy = std::make_shared<satellite_grasp_planner::core::NullspaceBiasedResolution>();
        const satellite_grasp_planner::core::JointPathSimulator sim(model, sp);
        const auto steps = sim.simulate(traj, q0);
        EXPECT_FALSE(steps.empty());
        if (!steps.empty()) model->update(steps.back().q, RobotModel::JointVector::Zero());
    };

    // fer_hand#1 = "palm top" capsule of satellite_collision.cpp (local center (0,0,0.10), axis Y,
    // radius 0.02, length 0.10). Duplicated here because the production table is file-local.
    auto handDistance = [&](const coal::CollisionGeometry& shape, const coal::Transform3s& tf) {
        const auto link = model->framePose("fer_hand");
        const Eigen::Matrix3d R = link.orientation.toRotationMatrix();
        const Eigen::Matrix3d Rl = Eigen::Quaterniond::FromTwoVectors(
            Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitY()).toRotationMatrix();
        const coal::Capsule cap(0.020, 0.10);
        const coal::Transform3s ctf(R * Rl, link.position + R * Eigen::Vector3d(0.0, 0.0, 0.10));
        coal::DistanceRequest req;
        req.enable_signed_distance = true;
        coal::DistanceResult r;
        coal::distance(&cap, ctf, &shape, tf, req, r);
        return r.min_distance;
    };

    const double wall = p.wall_thickness_m;
    const Eigen::Vector3d n_w = satellite.faceNormalWorld(theta);
    const Eigen::Vector3d base_w = satellite.cylinderBaseCenterWorld(theta);

    for (const auto& pt : points) {
        const auto target = satellite.graspPoseAt(pt.second, theta);
        reach(target.position_world, target.orientation_nominal_world);
        const double pos_err_mm = (model->eePosition() - target.position_world).norm() * 1000.0;

        // (1) solid cylinder, (2) thin wall box, on the nominal final configuration.
        const Eigen::Vector3d r_w = satellite.radialDirectionWorld(pt.second, theta);
        const Eigen::Vector3d t_w = n_w.cross(r_w);
        Eigen::Matrix3d Rb;
        Rb.col(0) = r_w;
        Rb.col(1) = t_w;
        Rb.col(2) = n_w;
        const coal::Box wall_box(wall, 0.06, p.standoff_m);
        const coal::Transform3s box_tf(
            Rb, base_w + (p.standoff_m / 2.0) * n_w + (p.collar_radius_m - wall / 2.0) * r_w);
        const coal::Cylinder solid(satellite.cylinderOuterRadius(), p.standoff_m);
        const coal::Transform3s solid_tf(
            Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), n_w).toRotationMatrix(),
            base_w + (p.standoff_m / 2.0) * n_w);
        const double d1 = handDistance(solid, solid_tf);
        const double d2 = handDistance(wall_box, box_tf);
        const auto res = checkSatelliteCollision(*model, satellite, theta);

        // (3) same thin box, grasp repeated with TCP at mid-wall radius and 1 cm past the free edge
        // (deeper towards the cube = -n).
        const Eigen::Vector3d goal3 =
            target.position_world - (wall / 2.0) * r_w - 0.01 * n_w;
        reach(goal3, target.orientation_nominal_world);
        const double pos_err3_mm = (model->eePosition() - goal3).norm() * 1000.0;
        const double d3 = handDistance(wall_box, box_tf);
        const bool invalid_nominal = pos_err_mm > 15.0;
        const bool invalid = pos_err_mm > 15.0 || pos_err3_mm > 15.0;
        std::cout << (invalid ? "[probe3-INVALID] " : "[probe3] ") << pt.first
                  << ": (1) solid=" << d1 << " (checkSatelliteCollision: " << res.min_distance_arm_m
                  << " @ " << res.closest_capsule << ")  (2) thin_box_nominal=" << d2
                  << "  (3) thin_box_deep_midwall=" << d3 << "  | IK err mm: nominal=" << pos_err_mm
                  << " deep=" << pos_err3_mm
                  << (invalid ? "  (IK error > 15 mm: distances NOT to be interpreted)" : "")
                  << std::endl;

        // Per-capsule breakdown from `res`, which was computed on the NOMINAL configuration
        // (before the deep-grasp reach above), so it is unaffected by the model having moved since.
        std::ostringstream p4;
        p4 << (invalid_nominal ? "[probe4-INVALID] " : "[probe4] ") << pt.first << ":";
        for (const auto& c : res.per_capsule) {
            p4 << " " << c.name << "=" << c.distance_m * 1000.0 << "mm"
               << (c.expected_contact ? "(excluded)" : "");
        }
        p4 << " | min_distance_arm_m=" << res.min_distance_arm_m << " closest_capsule="
           << res.closest_capsule << " feasible(d_safe=0.010)=" << res.feasible
           << " | IK err mm nominal=" << pos_err_mm;
        std::cout << p4.str() << std::endl;

        std::cout << (invalid_nominal ? "[probe5-INVALID] " : "[probe5] ") << pt.first
                  << ": min_distance_cube_m=" << res.min_distance_cube_m
                  << " closest_capsule_cube=" << res.closest_capsule_cube
                  << " | IK err mm nominal=" << pos_err_mm << std::endl;

        std::ostringstream msg;
        msg << "[probe] " << pt.first << " theta=" << theta << ": min_distance_arm_m="
            << res.min_distance_arm_m << " closest_capsule=" << res.closest_capsule
            << " feasible(d_safe=0.010)=" << res.feasible << " | IK final pos_err_mm=" << pos_err_mm
            << " (a large IK error means the configuration did not reach the pose)";
        std::cout << msg.str() << std::endl;
        RecordProperty(std::string("probe_") + pt.first, msg.str());

        EXPECT_TRUE(std::isfinite(res.min_distance_arm_m)) << msg.str();
        EXPECT_FALSE(res.closest_capsule.empty()) << msg.str();
    }
}

// EMPIRICAL probe, NO asserts on values. Real configuration (vertical spin axis, lateral face
// towards the robot base); the satellite POSITION (0.75,0,0.35) is a convenience one: the probe does
// no IK, so it does not depend on reachability. For a window of satellite phases it
// prints the ProDMP velocity at s>=1 obtained with velocityForCandidateGoal(g) for each grasp
// point, next to the velocity v_target = (omega*a) x (g - center) of a point rigidly spinning
// with the satellite, and the squared mismatch. The ProDMP is brought to s>=1 with a fixed
// support goal (p0 + demoDisplacement()) and setGoal() is never called afterwards.
TEST(SatelliteVelocityProbe, ReportsProDmpVelocityVersusSatelliteSurfaceVelocity) {
    SKIP_UNLESS_PROBES_ENABLED();
    auto model_owner = loadPandaRobotModel();
    ASSERT_NE(model_owner, nullptr);
    auto model = std::shared_ptr<RobotModel>(std::move(model_owner));

    auto p = makeModelBParams();
    const CubeSatelliteModel satellite(p);

    SKIP_UNLESS_PRODUCTION_WEIGHTS(weights_path);
    haptic_dmp_learning::core::ProDMP prodmp =
        haptic_dmp_learning::core::prodmp_io::loadProDmpFromYaml(weights_path);

    const RobotModel::JointVector q0 = RobotModel::readyPose();
    model->update(q0, RobotModel::JointVector::Zero());
    const Eigen::Vector3d p0 = model->eePosition();

    // 1. theta_c from the fer_link0 position.
    const Eigen::Vector3d a = p.axis_world.normalized();
    const Eigen::Vector3d n = p.face_normal_body.normalized();
    const Eigen::Vector3d u =
        (model->framePose("fer_link0").position - p.center_world).normalized();
    const Eigen::Vector3d u_perp = u - u.dot(a) * a;
    const double theta_c = std::atan2(a.dot(n.cross(u_perp)), n.dot(u_perp));
    const double rad2deg = 180.0 / M_PI;
    std::cout << "[probe6-window] theta_c=" << theta_c * rad2deg << " deg, window=["
              << theta_c * rad2deg - 90.0 << ", " << theta_c * rad2deg + 90.0 << "] deg"
              << std::endl;

    // 4. Bring the ProDMP to s >= 1 with a fixed support goal.
    const Eigen::Vector3d demo_disp = prodmp.demoDisplacement();  // before re-anchoring
    prodmp.setRelativeGoal(false);
    prodmp.setInitialConditions(0.0, p0, Eigen::Vector3d::Zero());
    const Eigen::Vector3d support_goal = p0 + demo_disp;
    prodmp.setGoal(support_goal);
    const double dt = 0.005;
    const int n_steps = static_cast<int>(std::ceil(prodmp.tau() / dt));
    for (int i = 0; i < n_steps; ++i) prodmp.step(dt);

    // 5. Checks after stepping.
    const Eigen::Vector3d vel_mm = prodmp.velocity() * 1000.0;
    std::cout << "[probe6-check] |velocity()|=" << vel_mm.norm() << " mm/s velocity()=("
              << vel_mm.x() << ", " << vel_mm.y() << ", " << vel_mm.z() << ") mm/s" << std::endl;
    std::cout << "[probe6-check] |position()-support_goal|="
              << (prodmp.position() - support_goal).norm() * 1000.0 << " mm" << std::endl;

    const std::vector<std::pair<const char*, haptic_dmp_learning::core::GraspPointId>> points = {
        {"kP0", haptic_dmp_learning::core::GraspPointId::kP0},
        {"kP90", haptic_dmp_learning::core::GraspPointId::kP90},
        {"kP180", haptic_dmp_learning::core::GraspPointId::kP180},
        {"kP270", haptic_dmp_learning::core::GraspPointId::kP270}};
    const double omega = 2.0 * M_PI / 180.0;  // 2 deg/s
    // 2./3. Samples theta_c + {-90,-45,0,45,90} deg.
    for (double off : {-90.0, -45.0, 0.0, 45.0, 90.0}) {
        const double theta = theta_c + off / rad2deg;
        for (const auto& pt : points) {
            const Eigen::Vector3d g = satellite.graspPoseAt(pt.second, theta).position_world;
            const Eigen::Vector3d v_ee = prodmp.velocityForCandidateGoal(g);
            const Eigen::Vector3d v_target = (omega * a).cross(g - p.center_world);
            const Eigen::Vector3d v_target_neg = -v_target;  // omega = -2 deg/s
            // Cosine between the (rotated) face normal and the satellite -> fer_link0 direction.
            const double cos_nu = satellite.faceNormalWorld(theta).dot(u);
            auto mm = [](const Eigen::Vector3d& v) {
                std::ostringstream o;
                o << "(" << v.x() * 1000.0 << ", " << v.y() * 1000.0 << ", " << v.z() * 1000.0 << ")";
                return o.str();
            };
            std::cout << "[probe6] theta_c" << (off >= 0 ? "+" : "") << off << " " << pt.first
                      << " theta=" << theta * rad2deg << " deg: v_ee=" << mm(v_ee)
                      << " mm/s v_target(+2deg/s)=" << mm(v_target)
                      << " mm/s v_ee.v_target=" << v_ee.dot(v_target) * 1e6
                      << " (mm/s)^2 e_v(+2)=" << (v_ee - v_target).squaredNorm() * 1e6
                      << " e_v(-2)=" << (v_ee - v_target_neg).squaredNorm() * 1e6
                      << " (mm/s)^2 |g-p0|=" << (g - p0).norm() * 1000.0
                      << " mm n.u=" << cos_nu << std::endl;
        }
    }
}
