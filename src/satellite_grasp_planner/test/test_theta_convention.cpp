// Proof that PhaseTracker's theta and CubeSatelliteModel's theta refer to the SAME physical rotation
// (see the grasp_planner_node plan review: the originally-suspected "V5 blocker" was retracted -
// CubeSatelliteModel's internal B matrix is theta-independent and cancels out of both identities
// below, so it never needs to equal R(satellite_q_ref)).
//
// Each theta_p is checked with a FRESH PhaseTracker (never a continuous sweep reusing one tracker's
// internal unwrap state), per the plan review's amendment: a sweep only proves unwrap() is
// monotonic, not that a single measurement is decoded correctly.

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/satellite_intercept.hpp"

using haptic_dmp_learning::core::CubeSatelliteModel;
using haptic_dmp_learning::core::GraspPointId;
namespace si = haptic_dmp_learning::core::satellite_intercept;

namespace {

// theta = 0 is the physical spawn orientation (identity), spun about world Z - matching
// free_target_object's identity spawn + a world-Z VelocityControl spin, the repo's default
// CubeSatelliteModel::Params (axis_world = z), and satellite_q_ref = identity (see
// config/grasp_planner_example.yaml). Values in (-pi, pi], including negative and zero.
const std::vector<double> kThetaValues = {-3.0, -M_PI / 2.0, -0.5, 0.0, 0.5, 1.0, 3.0, M_PI};

}  // namespace

TEST(ThetaConventionTest, PhaseTrackerRecoversPhysicalThetaFromFreshSampleEachTime) {
    const Eigen::Vector3d axis(0.0, 0.0, 1.0);
    const Eigen::Quaterniond q_ref = Eigen::Quaterniond::Identity();
    for (const double theta_p : kThetaValues) {
        si::PhaseTracker tracker(axis, q_ref);
        const Eigen::Quaterniond q_body(Eigen::AngleAxisd(theta_p, axis));
        const si::PhaseSample s = tracker.update(q_body);
        EXPECT_NEAR(s.theta_rad, theta_p, 1e-9) << "theta_p=" << theta_p;
        EXPECT_NEAR(s.off_axis_deg, 0.0, 1e-6) << "theta_p=" << theta_p;
    }
}

TEST(ThetaConventionTest, ModelGraspPointRotatesRigidlyWithTheta) {
    CubeSatelliteModel::Params params;  // repo defaults: axis_world = z, face_normal_body = x
    const CubeSatelliteModel model(params);
    const Eigen::Vector3d center = params.center_world;
    const Eigen::Vector3d axis = params.axis_world.normalized();

    for (GraspPointId k : {GraspPointId::kP0, GraspPointId::kP90, GraspPointId::kP180, GraspPointId::kP270}) {
        const Eigen::Vector3d p0 = model.graspPoseAt(k, 0.0).position_world;
        for (const double theta_p : kThetaValues) {
            const Eigen::Vector3d expected = center + Eigen::AngleAxisd(theta_p, axis) * (p0 - center);
            const Eigen::Vector3d actual = model.graspPoseAt(k, theta_p).position_world;
            EXPECT_NEAR((actual - expected).norm(), 0.0, 1e-9)
                << "k=" << static_cast<int>(k) << " theta_p=" << theta_p;
        }
    }
}

// Convention: (face_normal_body = n, theta) and (face_normal_body = -n, theta + pi) describe the SAME
// physical grasp pose. Not assumed: each pair is built from two independent models and compared
// numerically. The un-shifted pair (-n, theta) is also checked to DIFFER, so the comparison cannot pass
// vacuously (e.g. a pose that ignores the face normal).
TEST(ThetaConventionTest, OppositeFaceNormalWithThetaPlusPiGivesSameGraspPose) {
    struct Setup {
        Eigen::Vector3d center, axis, n;
    };
    const std::vector<Setup> setups = {
        {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX()},
        {Eigen::Vector3d(0.75, 0.0, 0.35), Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX()},
        {Eigen::Vector3d(0.3, -0.2, 0.5), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()},
    };
    const std::vector<double> thetas = {-3.0, -M_PI / 2.0, -0.5, 0.0, 0.5, 1.0, 3.0, M_PI};

    for (const auto& st : setups) {
        CubeSatelliteModel::Params p_pos;
        p_pos.center_world = st.center;
        p_pos.axis_world = st.axis;
        p_pos.face_normal_body = st.n;
        CubeSatelliteModel::Params p_neg = p_pos;
        p_neg.face_normal_body = -st.n;
        const CubeSatelliteModel m_pos(p_pos);
        const CubeSatelliteModel m_neg(p_neg);

        for (GraspPointId k : {GraspPointId::kP0, GraspPointId::kP90, GraspPointId::kP180, GraspPointId::kP270}) {
            for (const double theta : thetas) {
                const auto a = m_pos.graspPoseAt(k, theta);
                const auto b = m_neg.graspPoseAt(k, theta + M_PI);
                const std::string ctx = "k=" + std::to_string(static_cast<int>(k)) +
                                        " theta=" + std::to_string(theta) +
                                        " axis=" + std::to_string(st.axis.z()) + "z";
                EXPECT_NEAR((a.position_world - b.position_world).norm(), 0.0, 1e-9) << ctx;
                EXPECT_NEAR(a.orientation_nominal_world.normalized().angularDistance(
                                b.orientation_nominal_world.normalized()),
                            0.0, 1e-9)
                    << ctx;
                EXPECT_NEAR((a.approach_axis_world - b.approach_axis_world).norm(), 0.0, 1e-9) << ctx;

                // Discriminating check: WITHOUT the +pi shift the position must differ
                // (the collar sits on the opposite side of the cube).
                const auto c = m_neg.graspPoseAt(k, theta);
                EXPECT_GT((a.position_world - c.position_world).norm(), 1e-3) << ctx;
            }
        }
    }
}
