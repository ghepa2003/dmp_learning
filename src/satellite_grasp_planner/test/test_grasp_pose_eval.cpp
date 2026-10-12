// Tests for grasp_pose_eval: static pose IK, joint margins, w_trans, SE(3) distance, grasp-pose
// orientation convention (values worked out by hand) and per-candidate evaluation.
// Need Pinocchio and $HOME/thesis_ws/fer_flat_effort.urdf (tests skip if it is missing).

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "test_fixtures.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "satellite_grasp_planner/core/grasp_pose_eval.hpp"
#include "satellite_grasp_planner/core/manipulability.hpp"

using namespace satellite_grasp_planner::core;
using haptic_dmp_learning::core::CubeSatelliteModel;
using haptic_dmp_learning::core::GraspPointId;
using RobotModel = franka_cartesian_control::core::RobotModel;
namespace tf = satellite_grasp_planner::test_fixtures;

#define REQUIRE_URDF_OR_SKIP()                                                                  \
    do {                                                                                        \
        if (!std::ifstream(tf::urdfPath()).good()) {                                            \
            GTEST_SKIP() << "URDF fixture not found: " << tf::urdfPath();                       \
        }                                                                                       \
    } while (0)

namespace {

RobotModel::JointVector readyQ6Pi() {
    RobotModel::JointVector q;
    q << 0.0, -0.7854, 0.0, -2.3562, 0.0, 3.1416, 0.7854;
    return q;
}

/// Real satellite parameters, centre (0.75, 0, 0.77), axis +z, face normal +x, no rotation at theta = 0.
CubeSatelliteModel makeRealModel(const Eigen::Vector3d& center = Eigen::Vector3d(0.75, 0.0, 0.77)) {
    CubeSatelliteModel::Params p;
    p.center_world = center;
    p.axis_world = Eigen::Vector3d::UnitZ();
    p.face_normal_body = Eigen::Vector3d::UnitX();
    p.cube_side_m = 0.150;
    p.standoff_m = 0.08;
    p.collar_radius_m = 0.0725;
    p.wall_thickness_m = 0.009;
    p.grasp_radius_m = 0.068;
    return CubeSatelliteModel(p);
}

Eigen::Vector3d rot90z(const Eigen::Vector3d& v) { return Eigen::Vector3d(-v.y(), v.x(), v.z()); }

}  // namespace

TEST(GraspPoseEvalTest, IkReachesFkGeneratedPose) {
    REQUIRE_URDF_OR_SKIP();
    auto robot = tf::loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    const RobotModel::JointVector q_seed = readyQ6Pi();
    RobotModel::JointVector q_true = q_seed;
    q_true(0) += 0.1;
    q_true(2) -= 0.1;
    q_true(4) += 0.1;
    q_true(6) -= 0.1;
    robot->update(q_true, RobotModel::JointVector::Zero());
    const Eigen::Vector3d p = robot->eePosition();
    const Eigen::Quaterniond o = robot->eeOrientation();

    const IkParams params;
    const IkResult r = solvePoseIk(*robot, p, o, q_seed, q_seed, params);
    EXPECT_TRUE(r.converged);
    EXPECT_TRUE(r.limits_ok);
    EXPECT_LT(r.iterations, params.max_iter);
    EXPECT_LT(r.pos_err_m, params.pos_tol_m);
    EXPECT_LT(r.ori_err_rad, params.ori_tol_rad);
    // Independent check of the reported errors through FK on the returned q.
    robot->update(r.q, RobotModel::JointVector::Zero());
    EXPECT_LT((robot->eePosition() - p).norm(), 2.0 * params.pos_tol_m);
}

TEST(GraspPoseEvalTest, UnreachablePoseDoesNotConverge) {
    REQUIRE_URDF_OR_SKIP();
    auto robot = tf::loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    const RobotModel::JointVector q_seed = readyQ6Pi();
    const IkResult r = solvePoseIk(*robot, Eigen::Vector3d(2.0, 0.0, 0.5), Eigen::Quaterniond::Identity(),
                                   q_seed, q_seed, IkParams());
    EXPECT_FALSE(r.converged);
    EXPECT_TRUE(r.q.allFinite());
    EXPECT_TRUE(std::isfinite(r.pos_err_m));
    EXPECT_TRUE(std::isfinite(r.ori_err_rad));
    EXPECT_GT(r.pos_err_m, 0.5);  // reach is < 1.3 m, the target is 2 m away
}

TEST(GraspPoseEvalTest, ResultNeverOutsideLimits) {
    REQUIRE_URDF_OR_SKIP();
    auto robot = tf::loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    const RobotModel::JointVector lo = robot->jointLowerLimits();
    const RobotModel::JointVector hi = robot->jointUpperLimits();
    const RobotModel::JointVector q_seed = readyQ6Pi();

    // Targets taken from FK at configurations OUTSIDE the limits, plus an unreachable one.
    RobotModel::JointVector q_bad1 = q_seed;
    q_bad1(1) = hi(1) + 0.5;
    RobotModel::JointVector q_bad2 = q_seed;
    q_bad2(3) = hi(3) + 0.5;
    q_bad2(5) = hi(5) + 0.3;
    RobotModel::JointVector q_bad3 = q_seed;
    q_bad3(4) = lo(4) - 0.6;
    for (const auto& qb : {q_bad1, q_bad2, q_bad3}) {
        robot->update(qb, RobotModel::JointVector::Zero());
        const IkResult r = solvePoseIk(*robot, robot->eePosition(), robot->eeOrientation(), q_seed, q_seed,
                                       IkParams());
        EXPECT_TRUE((r.q.array() >= lo.array()).all());
        EXPECT_TRUE((r.q.array() <= hi.array()).all());
        EXPECT_TRUE(r.limits_ok);
        if (r.converged) {
            EXPECT_TRUE(r.limits_ok);
        }
    }
    // A seed outside the limits is clamped, never returned as is.
    RobotModel::JointVector q_out = q_seed;
    q_out(1) = hi(1) + 1.0;
    const IkResult r2 = solvePoseIk(*robot, Eigen::Vector3d(2.0, 0.0, 0.5), Eigen::Quaterniond::Identity(),
                                    q_out, q_seed, IkParams());
    EXPECT_TRUE((r2.q.array() <= hi.array()).all());
    EXPECT_TRUE((r2.q.array() >= lo.array()).all());
}

TEST(GraspPoseEvalTest, JointMargins) {
    REQUIRE_URDF_OR_SKIP();
    auto robot = tf::loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    const RobotModel::JointVector lo = robot->jointLowerLimits();
    const RobotModel::JointVector hi = robot->jointUpperLimits();
    const RobotModel::JointVector mid = 0.5 * (lo + hi);

    const JointMargins m0 = jointMargins(*robot, mid);
    for (int j = 0; j < 7; ++j) EXPECT_NEAR(m0.margin_rad(j), 0.5 * (hi(j) - lo(j)), 1e-12);
    const RobotModel::JointVector half = 0.5 * (hi - lo);
    int expected_worst = 0;
    const double expected_min = half.minCoeff(&expected_worst);
    EXPECT_NEAR(m0.min_margin_rad, expected_min, 1e-12);
    EXPECT_EQ(m0.worst_joint, expected_worst);

    RobotModel::JointVector q = mid;
    q(2) = hi(2) - 0.1;  // joint 3 has a 5.79 rad range: its 0.1 margin is the smallest
    const JointMargins m1 = jointMargins(*robot, q);
    EXPECT_NEAR(m1.margin_rad(2), 0.1, 1e-12);
    EXPECT_NEAR(m1.min_margin_rad, 0.1, 1e-12);
    EXPECT_EQ(m1.worst_joint, 2);

    q = mid;
    q(5) = lo(5) + 0.1;  // lower side
    const JointMargins m2 = jointMargins(*robot, q);
    EXPECT_NEAR(m2.min_margin_rad, 0.1, 1e-12);
    EXPECT_EQ(m2.worst_joint, 5);
}

TEST(GraspPoseEvalTest, WTransAtMatchesManipulability) {
    REQUIRE_URDF_OR_SKIP();
    auto robot = tf::loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    const Manipulability manip(robot);
    RobotModel::JointVector q = readyQ6Pi();
    q(0) += 0.2;
    q(3) += 0.15;
    EXPECT_NEAR(wTransAt(*robot, q), manip.wTrans(q), 1e-12);
    EXPECT_NEAR(wTransAt(*robot, RobotModel::readyPose()), manip.wTrans(RobotModel::readyPose()), 1e-12);
}

TEST(GraspPoseEvalTest, Se3Distance) {
    const Eigen::Vector3d p(0.3, -0.2, 0.5);
    const Eigen::Quaterniond q(Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized()));
    const Se3Distance same = se3Distance(p, q, p, q, 0.1, 0.5);
    EXPECT_NEAR(same.d, 0.0, 1e-12);
    EXPECT_NEAR(same.pos_m, 0.0, 1e-15);
    EXPECT_NEAR(same.ori_rad, 0.0, 1e-12);

    Eigen::Quaterniond qn = q;
    qn.coeffs() = -qn.coeffs();
    const Se3Distance sign = se3Distance(p, q, p, qn, 0.1, 0.5);
    EXPECT_NEAR(sign.ori_rad, 0.0, 1e-12);
    EXPECT_NEAR(sign.d, 0.0, 1e-12);

    // By hand: 0.1 m and 0.5 rad with scales (0.2 m, 1.0 rad) -> sqrt(0.5^2 + 0.5^2).
    const Eigen::Quaterniond q2(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
    const Se3Distance h = se3Distance(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                                      Eigen::Vector3d(0.1, 0.0, 0.0), q2, 0.2, 1.0);
    EXPECT_NEAR(h.pos_m, 0.1, 1e-12);
    EXPECT_NEAR(h.ori_rad, 0.5, 1e-12);
    EXPECT_NEAR(h.d, std::sqrt(0.5), 1e-12);

    EXPECT_THROW(se3Distance(p, q, p, q, 0.0, 1.0), std::invalid_argument);
    EXPECT_THROW(se3Distance(p, q, p, q, 1.0, -1.0), std::invalid_argument);
}

// Hand-verified orientation convention (cube_satellite_model.cpp:55-75), real parameters, theta = 0:
// approach z_tcp = -n = (-1,0,0) for every k, fingers y_tcp = -r_k (radial), x_tcp = y_tcp x z_tcp.
TEST(GraspPoseEvalTest, GraspPoseConventionByHand) {
    struct Row {
        GraspPointId k;
        Eigen::Vector3d pos, z, y, x;
    };
    const Row rows[4] = {
        {GraspPointId::kP0, {0.905, -0.068, 0.770}, {-1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
        {GraspPointId::kP90, {0.905, 0.000, 0.838}, {-1, 0, 0}, {0, 0, -1}, {0, 1, 0}},
        {GraspPointId::kP180, {0.905, 0.068, 0.770}, {-1, 0, 0}, {0, -1, 0}, {0, 0, -1}},
        {GraspPointId::kP270, {0.905, 0.000, 0.702}, {-1, 0, 0}, {0, 0, 1}, {0, -1, 0}},
    };
    const Eigen::Vector3d center(0.75, 0.0, 0.77);
    const CubeSatelliteModel model = makeRealModel(center);
    const double tol = 1e-9;
    for (const Row& r : rows) {
        for (int th = 0; th < 2; ++th) {
            const double theta = th == 0 ? 0.0 : M_PI / 2.0;
            const auto pose = model.graspPoseAt(r.k, theta);
            const Eigen::Matrix3d R = pose.orientation_nominal_world.toRotationMatrix();
            const Eigen::Vector3d ep = th == 0 ? r.pos : center + rot90z(r.pos - center);
            const Eigen::Vector3d ez = th == 0 ? r.z : rot90z(r.z);
            const Eigen::Vector3d ey = th == 0 ? r.y : rot90z(r.y);
            const Eigen::Vector3d ex = th == 0 ? r.x : rot90z(r.x);
            const std::string tag = "k=" + std::to_string(static_cast<int>(r.k)) + " theta=" + (th ? "90" : "0");
            EXPECT_LT((pose.position_world - ep).norm(), tol) << tag << " pos " << pose.position_world.transpose();
            EXPECT_LT((R.col(2) - ez).norm(), tol) << tag << " z_tcp " << R.col(2).transpose();
            EXPECT_LT((R.col(1) - ey).norm(), tol) << tag << " y_tcp " << R.col(1).transpose();
            EXPECT_LT((R.col(0) - ex).norm(), tol) << tag << " x_tcp " << R.col(0).transpose();
            EXPECT_LT((pose.approach_axis_world - ez).norm(), tol) << tag << " approach";
        }
    }
}

// Gripper symmetry: for k = 0, theta = 0 from ready_q6pi the returned branch must follow the documented
// selection rule (converged first), and a flipped result must be inside the limits (q7 included).
TEST(GraspPoseEvalTest, WristFlipIsConsistent) {
    REQUIRE_URDF_OR_SKIP();
    auto robot = tf::loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    const RobotModel::JointVector q_start = readyQ6Pi();
    const DemoReference demo = demoReferenceFromEndJoints(*robot, q_start);
    const CubeSatelliteModel model = makeRealModel();
    const IkParams params;

    const PoseEvaluation ev = evaluateGraspPose(model, GraspPointId::kP0, 0.0, *robot, q_start, demo, 0.1, 0.5, params);

    const auto pose = model.graspPoseAt(GraspPointId::kP0, 0.0);
    const Eigen::Quaterniond q_flip = pose.orientation_nominal_world *
                                      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
    const IkResult nominal = solvePoseIk(*robot, pose.position_world, pose.orientation_nominal_world, q_start, q_start, params);
    const IkResult flipped = solvePoseIk(*robot, pose.position_world, q_flip, q_start, q_start, params);

    if (nominal.converged && !flipped.converged) {
        EXPECT_FALSE(ev.wrist_flip);
    }
    if (!nominal.converged && flipped.converged) {
        EXPECT_TRUE(ev.wrist_flip);
    }
    EXPECT_EQ(ev.feasible_basic, nominal.converged || flipped.converged);
    const IkResult& chosen_ref = ev.wrist_flip ? flipped : nominal;
    EXPECT_LT((ev.ik.q - chosen_ref.q).norm(), 1e-9);  // deterministic: same inputs, same branch result

    const RobotModel::JointVector lo = robot->jointLowerLimits();
    const RobotModel::JointVector hi = robot->jointUpperLimits();
    if (ev.wrist_flip && ev.ik.converged) {
        EXPECT_GE(ev.ik.q(6), lo(6));
        EXPECT_LE(ev.ik.q(6), hi(6));
    }
    if (!nominal.converged && flipped.converged) {
        EXPECT_TRUE(ev.wrist_flip);
        EXPECT_GE(ev.ik.q(6), lo(6));
        EXPECT_LE(ev.ik.q(6), hi(6));
    }
}

TEST(GraspPoseEvalTest, EvaluateGraspPoseEndToEnd) {
    REQUIRE_URDF_OR_SKIP();
    auto robot = tf::loadPandaRobotModel();
    ASSERT_NE(robot, nullptr);
    const RobotModel::JointVector q_start = readyQ6Pi();
    const DemoReference demo = demoReferenceFromEndJoints(*robot, q_start);
    ASSERT_GT(demo.w_trans_demo, 0.0);

    // Place the satellite so that the (k = 0, theta = 0) grasp point is ~7 cm from the start TCP.
    const Eigen::Vector3d grasp_offset(0.155, -0.068, 0.0);  // body-frame offset of k = 0 at theta = 0
    const Eigen::Vector3d center = demo.position + Eigen::Vector3d(0.05, 0.0, 0.05) - grasp_offset;
    const CubeSatelliteModel model = makeRealModel(center);
    const GraspPointId k = GraspPointId::kP0;
    const double theta = 0.0;
    const auto pose = model.graspPoseAt(k, theta);
    ASSERT_LT((pose.position_world - demo.position).norm(), 0.15);

    const IkParams params;
    const PoseEvaluation ev = evaluateGraspPose(model, k, theta, *robot, q_start, demo, 0.1, 0.5, params);
    EXPECT_EQ(ev.k, k);
    EXPECT_DOUBLE_EQ(ev.theta_rad, theta);
    EXPECT_TRUE(ev.ik.q.allFinite());
    EXPECT_TRUE(std::isfinite(ev.w_trans));
    EXPECT_TRUE(std::isfinite(ev.w_hat));
    EXPECT_TRUE(std::isfinite(ev.margins.min_margin_rad));
    EXPECT_TRUE(std::isfinite(ev.dist_to_demo.d));
    EXPECT_NEAR(ev.w_hat, ev.w_trans / demo.w_trans_demo, 1e-12);
    EXPECT_NEAR(ev.w_trans, wTransAt(*robot, ev.ik.q), 1e-12);
    EXPECT_NEAR(ev.dist_to_demo.pos_m, (pose.position_world - demo.position).norm(), 1e-12);
    EXPECT_EQ(ev.feasible_basic, ev.ik.converged && ev.ik.limits_ok);
    if (ev.ik.converged) {
        EXPECT_LT(ev.ik.pos_err_m, params.pos_tol_m);
        EXPECT_LT(ev.ik.ori_err_rad, params.ori_tol_rad);
    }

    EXPECT_THROW(evaluateGraspPose(model, k, theta, *robot, q_start, DemoReference{}, 0.1, 0.5, params),
                 std::invalid_argument);
}
