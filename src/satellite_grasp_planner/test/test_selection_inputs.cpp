// Tests for buildSelectionInputs(): synthetic weights + demo_params written to a temp directory (no
// production files, no long rollouts, no probes). Needs Pinocchio and $HOME/thesis_ws/fer_flat_effort.urdf.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "test_fixtures.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/demo_params.hpp"
#include "haptic_dmp_learning/core/grasp_cost.hpp"
#include "haptic_dmp_learning/core/math_utils.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "haptic_dmp_learning/core/types.hpp"
#include "satellite_grasp_planner/core/manipulability.hpp"
#include "satellite_grasp_planner/core/selection_inputs.hpp"

using namespace satellite_grasp_planner::core;
using haptic_dmp_learning::core::CubeSatelliteModel;
using haptic_dmp_learning::core::ProDMP;
using haptic_dmp_learning::core::Sample;
using haptic_dmp_learning::core::degToRad;
namespace dp = haptic_dmp_learning::core::demo_params;
using RobotModel = franka_cartesian_control::core::RobotModel;
using satellite_grasp_planner::test_fixtures::resetRobotToReady;
using satellite_grasp_planner::test_fixtures::urdfPath;

namespace {

std::string readAll(const std::string& p) {
    std::ifstream f(p, std::ios::binary);
    std::stringstream b;
    b << f.rdbuf();
    return b.str();
}

void writeText(const std::string& p, const std::string& text) {
    std::ofstream f(p, std::ios::binary);
    f << text;
}

class SelectionInputsTest : public ::testing::Test {
protected:
    void SetUp() override {
        if (!std::ifstream(urdfPath()).good()) {
            GTEST_SKIP() << "URDF fixture not found: " << urdfPath() << " (needs $HOME/thesis_ws/fer_flat_effort.urdf)";
        }
        char tmpl[] = "/tmp/selection_inputs_XXXXXX";
        const char* d = mkdtemp(tmpl);
        ASSERT_NE(d, nullptr);
        dir_ = d;
    }
    void TearDown() override {
        if (!dir_.empty()) std::filesystem::remove_all(dir_);
    }

    std::string file(const std::string& name) const { return dir_ + "/" + name; }
    std::string weightsPath() const { return file("weights.yaml"); }
    std::string paramsPath() const { return dp::deriveDemoParamsPath(weightsPath()); }

    /// Synthetic straight demo of length 0.1 m along x lasting @p tau s; optional trigger sample time.
    /// Writes weights.yaml (saveProDmpToYaml) and weights_demo_params.yaml (writeForWeights); returns the params.
    dp::DemoParams makeFixture(double tau, std::optional<double> trigger_t = std::nullopt) {
        std::vector<Sample> demo;
        for (int i = 0; i <= 50; ++i) {
            Sample s;
            s.t = tau * i / 50.0;
            s.position = Eigen::Vector3d(0.1 * s.t / tau, 0.0, 0.0);
            s.gripper_trigger = trigger_t && std::abs(s.t - *trigger_t) < 1e-9;
            demo.push_back(s);
        }
        ProDMP prodmp(/*num_basis=*/8);
        prodmp.learnFromDemonstration(demo);
        haptic_dmp_learning::core::prodmp_io::saveProDmpToYaml(prodmp, weightsPath());

        dp::FitInfo fit;
        fit.num_basis = 8;
        fit.ridge_lambda = 1e-9;
        std::ostringstream log;
        dp::DemoParams out;
        dp::writeForWeights(file("demo.csv"), demo, fit, weightsPath(), std::nullopt, log, &out);
        return out;
    }

    /// Rewrites the params file (same weights, so the hash stays valid).
    void rewrite(const dp::DemoParams& p) { dp::write(paramsPath(), p); }

    SatelliteSnapshot snapshot() const {
        SatelliteSnapshot s;
        s.center = Eigen::Vector3d(0.75, 0.0, 0.35);
        s.axis = Eigen::Vector3d(0.0, 0.0, 1.0);
        s.omega_rad_s = -degToRad(2.0);
        return s;
    }
    CubeSatelliteModel::Params geometry() const {
        CubeSatelliteModel::Params g;
        g.face_normal_body = Eigen::Vector3d(-1.0, 0.0, 0.0);
        return g;
    }
    SelectionInputs build() const {
        return buildSelectionInputs(paramsPath(), urdfPath(), snapshot(), geometry(), Eigen::Vector3d::Zero());
    }

    static std::array<double, 7> toArray(const RobotModel::JointVector& q) {
        std::array<double, 7> a{};
        for (int i = 0; i < 7; ++i) a[static_cast<std::size_t>(i)] = q(i);
        return a;
    }

    RobotModel makeRobot() const {
        std::stringstream buf;
        buf << std::ifstream(urdfPath()).rdbuf();
        std::vector<std::string> names;
        for (int i = 1; i <= 7; ++i) names.push_back("fer_joint" + std::to_string(i));
        return RobotModel(buf.str(), names, "fer_hand_tcp");
    }

    std::string dir_;
};

}  // namespace

// weights_sha256 is the hash of the weights file actually loaded.
TEST_F(SelectionInputsTest, WeightsSha256MatchesFile) {
    dp::DemoParams p = makeFixture(60.0);
    p.end_joints_rad = toArray(RobotModel::readyPose());
    rewrite(p);
    const SelectionInputs in = build();
    EXPECT_EQ(in.weights_sha256, dp::sha256FileHex(weightsPath()));
    EXPECT_EQ(in.weights_sha256.size(), 64u);
}

// Recorded hash different from the file's: buildSelectionInputs throws (never reports the recorded hash).
TEST_F(SelectionInputsTest, RecordedHashDifferentFromFileThrows) {
    dp::DemoParams p = makeFixture(60.0);
    p.end_joints_rad = toArray(RobotModel::readyPose());
    p.weights_sha256 = dp::sha256Hex("not the weights file");
    rewrite(p);
    try {
        build();
        ADD_FAILURE() << "expected an exception";
    } catch (const std::exception& e) {
        const std::string what = e.what();
        EXPECT_NE(what.find("hash mismatch"), std::string::npos) << what;
        EXPECT_NE(what.find(dp::sha256FileHex(weightsPath())), std::string::npos) << what;
    }
}

// (a) a byte changed in the weights after writing the demo_params: it must throw and name the file.
TEST_F(SelectionInputsTest, ChangedWeightsThrowNamingTheParamsFile) {
    dp::DemoParams p = makeFixture(60.0);
    p.end_joints_rad = toArray(RobotModel::readyPose());
    rewrite(p);
    EXPECT_NO_THROW(build());
    writeText(weightsPath(), readAll(weightsPath()) + "\n");
    try {
        build();
        ADD_FAILURE() << "expected an exception";
    } catch (const std::exception& e) {
        EXPECT_NE(std::string(e.what()).find(paramsPath()), std::string::npos) << e.what();
    }
}

// (b) contact time handling.
TEST_F(SelectionInputsTest, ContactTime) {
    dp::DemoParams p = makeFixture(60.0);  // no trigger -> t_contact null
    ASSERT_FALSE(p.t_contact_s.has_value());
    p.end_joints_rad = toArray(RobotModel::readyPose());  // avoid the (long) proxy rollout
    rewrite(p);
    SelectionInputs a = build();
    EXPECT_TRUE(a.contact_assumed_at_end);
    EXPECT_EQ(a.params.scan.tau_contact_s, 0.0);

    p = makeFixture(60.0, 30.0);  // trigger at t = 30 s
    ASSERT_TRUE(p.t_contact_s.has_value());
    p.end_joints_rad = toArray(RobotModel::readyPose());
    rewrite(p);
    SelectionInputs b = build();
    EXPECT_FALSE(b.contact_assumed_at_end);
    EXPECT_NEAR(b.params.scan.tau_contact_s, 30.0, 1e-9);
    EXPECT_GT(b.prodmp_template.tau(), 30.0);

    // t_contact valid for the demo_params (<= its tau_s) but beyond the template tau: throws.
    p.tau_s = 100.0;
    p.t_contact_s = 80.0;
    rewrite(p);
    EXPECT_THROW(build(), std::invalid_argument);
}

// (c) p0 comes from the robot kinematics at q0 (start joints, else readyPose), never from the demo frame.
TEST_F(SelectionInputsTest, P0FromKinematics) {
    RobotModel ref = makeRobot();
    dp::DemoParams p = makeFixture(60.0);
    p.end_joints_rad = toArray(RobotModel::readyPose());
    RobotModel::JointVector q;
    q << 0.1, -0.5, 0.1, -2.2, 0.1, 1.6, 0.6;
    p.start_joints_rad = toArray(q);
    rewrite(p);
    SelectionInputs a = build();
    ref.update(q, RobotModel::JointVector::Zero());
    EXPECT_NEAR((a.p0 - ref.eePosition()).norm(), 0.0, 1e-12);
    EXPECT_NEAR((a.q0 - q).norm(), 0.0, 1e-15);
    EXPECT_NEAR((a.params.scan.p0 - a.p0).norm(), 0.0, 1e-15);
    // Robot left at q0.
    EXPECT_NEAR((a.robot->eePosition() - a.p0).norm(), 0.0, 1e-12);

    p.start_joints_rad.reset();
    rewrite(p);
    SelectionInputs b = build();
    resetRobotToReady(ref);
    EXPECT_NEAR((b.p0 - ref.eePosition()).norm(), 0.0, 1e-12);
    EXPECT_NEAR((b.q0 - RobotModel::readyPose()).norm(), 0.0, 1e-15);
    // Not the demo-frame start position.
    EXPECT_GT((b.p0 - p.start_position_demo_frame_m).norm(), 1e-3);
}

// Start configuration override (planner parameter start_joints_override_rad): wins over the demo file and readyPose.
TEST_F(SelectionInputsTest, StartOverrideWinsOverDemoStartJoints) {
    RobotModel ref = makeRobot();
    dp::DemoParams p = makeFixture(60.0);
    p.end_joints_rad = toArray(RobotModel::readyPose());
    RobotModel::JointVector q_demo, q_ovr;
    q_demo << 0.1, -0.5, 0.1, -2.2, 0.1, 1.6, 0.6;
    q_ovr << -0.05, -0.7, 0.02, -2.3, 0.04, 1.55, 0.8;
    p.start_joints_rad = toArray(q_demo);
    rewrite(p);

    StartState st;
    st.q = q_ovr;
    st.source = "start_joints_override_rad parameter";
    const SelectionInputs a = buildSelectionInputs(paramsPath(), urdfPath(), snapshot(), geometry(),
                                                   Eigen::Vector3d::Zero(), st);
    ref.update(q_ovr, RobotModel::JointVector::Zero());
    EXPECT_NEAR((a.q0 - q_ovr).norm(), 0.0, 1e-15);
    EXPECT_NEAR((a.p0 - ref.eePosition()).norm(), 0.0, 1e-12);
    EXPECT_NEAR((a.params.scan.p0 - a.p0).norm(), 0.0, 1e-15);
    EXPECT_NEAR((a.robot->eePosition() - a.p0).norm(), 0.0, 1e-12);  // robot left at q0
    EXPECT_EQ(a.q0_source, "start_joints_override_rad parameter");
    std::string q0_line;
    for (const auto& l : a.provenance) {
        if (l.rfind("q0 =", 0) == 0) q0_line = l;
    }
    EXPECT_NE(q0_line.find("start_joints_override_rad parameter"), std::string::npos) << q0_line;
    EXPECT_NE(q0_line.find("IGNORATO"), std::string::npos) << q0_line;  // the demo file had start_joints_rad

    // Without start joints in the demo file, nothing is reported as ignored.
    p.start_joints_rad.reset();
    rewrite(p);
    const SelectionInputs b = buildSelectionInputs(paramsPath(), urdfPath(), snapshot(), geometry(),
                                                   Eigen::Vector3d::Zero(), st);
    for (const auto& l : b.provenance) {
        if (l.rfind("q0 =", 0) == 0) {
            EXPECT_EQ(l.find("IGNORATO"), std::string::npos) << l;
        }
    }
}

TEST_F(SelectionInputsTest, StartOutsideJointLimitsThrows) {
    dp::DemoParams p = makeFixture(60.0);
    p.end_joints_rad = toArray(RobotModel::readyPose());
    rewrite(p);
    StartState st;
    st.q = RobotModel::readyPose();
    st.q(3) = 0.5;  // fer_joint4 upper limit is -0.0698 rad
    st.source = "test";
    EXPECT_THROW(buildSelectionInputs(paramsPath(), urdfPath(), snapshot(), geometry(), Eigen::Vector3d::Zero(), st),
                 std::invalid_argument);
}

// Regression: a start equal to readyPose gives the SAME selection inputs as no start at all (demo file without
// start_joints_rad), i.e. the parameter's default (empty = readyPose) changes nothing.
namespace {
void expectSameInputs(const SelectionInputs& a, const SelectionInputs& b) {
    EXPECT_EQ(a.q0, b.q0);
    EXPECT_EQ(a.p0, b.p0);
    EXPECT_EQ(a.params.scan.p0, b.params.scan.p0);
    EXPECT_EQ(a.params.scan.theta_center_rad, b.params.scan.theta_center_rad);
    EXPECT_EQ(a.params.scan.omega_rad_s, b.params.scan.omega_rad_s);
    EXPECT_EQ(a.params.scan.tau_contact_s, b.params.scan.tau_contact_s);
    EXPECT_EQ(a.params.scan.delta_g_demo, b.params.scan.delta_g_demo);
    EXPECT_EQ(a.params.scan.contact_to_end_offset, b.params.scan.contact_to_end_offset);
    EXPECT_EQ(a.params.scan.e_v_ref, b.params.scan.e_v_ref);
    EXPECT_EQ(a.params.scan.e_g_ref, b.params.scan.e_g_ref);
    EXPECT_EQ(a.params.w_trans_demo, b.params.w_trans_demo);
    EXPECT_EQ(a.w_trans_demo_is_proxy, b.w_trans_demo_is_proxy);
    EXPECT_EQ(a.contact_assumed_at_end, b.contact_assumed_at_end);
    EXPECT_EQ(a.weights_sha256, b.weights_sha256);
    ASSERT_EQ(a.provenance.size(), b.provenance.size());
    for (std::size_t i = 0; i < a.provenance.size(); ++i) {
        // Only the q0 line differs (it names the source).
        if (a.provenance[i].rfind("q0 =", 0) == 0) continue;
        EXPECT_EQ(a.provenance[i], b.provenance[i]);
    }
}
}  // namespace

TEST_F(SelectionInputsTest, StartEqualToReadyPoseIsIdenticalToTheDefault) {
    dp::DemoParams p = makeFixture(60.0);
    p.end_joints_rad = toArray(RobotModel::readyPose());
    ASSERT_FALSE(p.start_joints_rad.has_value());
    rewrite(p);
    const SelectionInputs def = build();
    StartState st;
    st.q = RobotModel::readyPose();
    st.source = "start_joints_override_rad parameter";
    const SelectionInputs ovr = buildSelectionInputs(paramsPath(), urdfPath(), snapshot(), geometry(),
                                                     Eigen::Vector3d::Zero(), st);
    expectSameInputs(def, ovr);
    EXPECT_EQ(def.q0_source, "RobotModel::readyPose()");
}

// Same, on the production configuration: end_joints_rad null, so w_trans_demo comes from the proxy rollout that
// starts from q0.
TEST_F(SelectionInputsTest, StartEqualToReadyPoseIsIdenticalToTheDefaultWithProxyWTrans) {
    dp::DemoParams p = makeFixture(2.0);  // short template: the proxy rollout stays cheap
    ASSERT_FALSE(p.end_joints_rad.has_value());
    const SelectionInputs def = build();
    ASSERT_TRUE(def.w_trans_demo_is_proxy);
    StartState st;
    st.q = RobotModel::readyPose();
    st.source = "start_joints_override_rad parameter";
    const SelectionInputs ovr = buildSelectionInputs(paramsPath(), urdfPath(), snapshot(), geometry(),
                                                     Eigen::Vector3d::Zero(), st);
    expectSameInputs(def, ovr);
}

// The proxy depends on q0: a different start changes it (this is why the start must be visible in the log).
TEST_F(SelectionInputsTest, ProxyWTransFollowsTheStartConfiguration) {
    dp::DemoParams p = makeFixture(2.0);
    const SelectionInputs def = build();
    StartState st;
    st.q = RobotModel::readyPose();
    st.q(0) += 0.6;
    st.q(3) += 0.3;
    st.source = "test";
    const SelectionInputs ovr = buildSelectionInputs(paramsPath(), urdfPath(), snapshot(), geometry(),
                                                     Eigen::Vector3d::Zero(), st);
    EXPECT_NE(def.params.w_trans_demo, ovr.params.w_trans_demo);
    EXPECT_NE(def.p0, ovr.p0);
}

TEST_F(SelectionInputsTest, DescribeStartStateReportsSourceQ0P0AndDistance) {
    dp::DemoParams p = makeFixture(60.0);
    p.end_joints_rad = toArray(RobotModel::readyPose());
    rewrite(p);
    StartState st;
    st.q = RobotModel::readyPose();
    st.q(0) += 0.05;
    st.source = "start_joints_override_rad parameter";
    const SelectionInputs a = buildSelectionInputs(paramsPath(), urdfPath(), snapshot(), geometry(),
                                                   Eigen::Vector3d::Zero(), st);
    const std::string text = describeStartState(a);
    EXPECT_NE(text.find("q0 source = start_joints_override_rad parameter"), std::string::npos) << text;
    EXPECT_NE(text.find("distance from readyPose"), std::string::npos) << text;
    EXPECT_NE(text.find("fer_joint1"), std::string::npos) << text;
    EXPECT_NE(text.find("provenance:"), std::string::npos) << text;
    EXPECT_NE(text.find("q0 = "), std::string::npos) << text;
    // Robot left at q0 by the report.
    EXPECT_NEAR((a.robot->eePosition() - a.p0).norm(), 0.0, 1e-12);
    std::cout << text << std::endl;
}

// (d) w_trans_demo from the end joints, cross-checked against compute_w_trans_demo.py.
TEST_F(SelectionInputsTest, WTransDemoFromEndJoints) {
    RobotModel ref = makeRobot();
    dp::DemoParams p = makeFixture(60.0);
    p.end_joints_rad = toArray(RobotModel::readyPose());
    rewrite(p);
    SelectionInputs a = build();
    resetRobotToReady(ref);
    EXPECT_NEAR(a.params.w_trans_demo, wTransFromJacobian(ref.jacobian().topRows<3>()), 1e-12);
    EXPECT_FALSE(a.w_trans_demo_is_proxy);

    // Value measured with tools/compute_w_trans_demo.py for q = (0.2,-0.6,0.1,-2.0,0.1,1.7,0.7).
    RobotModel::JointVector q;
    q << 0.2, -0.6, 0.1, -2.0, 0.1, 1.7, 0.7;
    ref.update(q, RobotModel::JointVector::Zero());
    const double w_here = wTransFromJacobian(ref.jacobian().topRows<3>());
    const double w_python = 0.136659;
    std::cout << "[w_trans check] wTransFromJacobian=" << w_here << " python=" << w_python
              << " diff=" << (w_here - w_python) << std::endl;
    EXPECT_NEAR(w_here, w_python, 1e-5);

    p.end_joints_rad = toArray(q);
    rewrite(p);
    EXPECT_NEAR(build().params.w_trans_demo, w_python, 1e-5);
}

TEST_F(SelectionInputsTest, ProxyWhenNoEndJoints) {
    dp::DemoParams p = makeFixture(2.0);  // short template: the proxy rollout stays cheap
    ASSERT_FALSE(p.end_joints_rad.has_value());
    SelectionInputs a = build();
    EXPECT_TRUE(a.w_trans_demo_is_proxy);
    EXPECT_GT(a.params.w_trans_demo, 0.0);
    bool has_proxy_line = false;
    for (const auto& l : a.provenance) {
        if (l.find("PROXY") != std::string::npos) has_proxy_line = true;
    }
    EXPECT_TRUE(has_proxy_line);
    // Robot back at q0 after the proxy rollout.
    EXPECT_NEAR((a.robot->eePosition() - a.p0).norm(), 0.0, 1e-12);
}

// (e) references and (f) provenance.
TEST_F(SelectionInputsTest, ReferencesAndProvenance) {
    dp::DemoParams p = makeFixture(60.0);
    p.end_joints_rad = toArray(RobotModel::readyPose());
    rewrite(p);
    const SelectionInputs a = build();

    CubeSatelliteModel::Params cp = geometry();
    cp.center_world = snapshot().center;
    cp.axis_world = snapshot().axis;
    const CubeSatelliteModel model(cp);
    EXPECT_NEAR(a.params.scan.e_g_ref, p.delta_g_demo_m.squaredNorm(), 1e-12);
    EXPECT_NEAR(a.params.scan.e_v_ref,
                haptic_dmp_learning::core::meanSurfaceSpeedSquared(model, snapshot().omega_rad_s), 1e-12);
    EXPECT_NEAR(a.params.scan.omega_rad_s, snapshot().omega_rad_s, 1e-15);
    EXPECT_NEAR((a.cube_params.center_world - snapshot().center).norm(), 0.0, 1e-15);
    EXPECT_NEAR((a.params.scan.delta_g_demo - p.delta_g_demo_m).norm(), 0.0, 1e-15);

    for (const char* key : {"tau_contact_s", "delta_g_demo_m", "contact_to_end_offset_m", "q0 =", "p0 =",
                            "w_trans_demo", "omega_rad_s", "satellite center", "satellite axis", "e_v_ref",
                            "e_g_ref"}) {
        bool found = false;
        for (const auto& l : a.provenance) {
            if (l.rfind(key, 0) == 0) found = true;
        }
        EXPECT_TRUE(found) << "no provenance line for " << key;
    }
    for (const auto& l : a.provenance) std::cout << "[provenance] " << l << std::endl;
}

// (g) zero displacement.
TEST_F(SelectionInputsTest, ZeroDeltaGThrows) {
    dp::DemoParams p = makeFixture(60.0);
    p.end_joints_rad = toArray(RobotModel::readyPose());
    p.delta_g_demo_m = Eigen::Vector3d::Zero();
    rewrite(p);
    EXPECT_THROW(build(), std::invalid_argument);
}
