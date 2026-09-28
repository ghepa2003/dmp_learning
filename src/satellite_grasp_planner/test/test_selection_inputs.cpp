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

namespace {

std::string urdfPath() {
    const char* home = std::getenv("HOME");
    return std::string(home ? home : "/root") + "/thesis_ws/fer_flat_effort.urdf";
}

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
    ref.update(RobotModel::readyPose(), RobotModel::JointVector::Zero());
    EXPECT_NEAR((b.p0 - ref.eePosition()).norm(), 0.0, 1e-12);
    EXPECT_NEAR((b.q0 - RobotModel::readyPose()).norm(), 0.0, 1e-15);
    // Not the demo-frame start position.
    EXPECT_GT((b.p0 - p.start_position_demo_frame_m).norm(), 1e-3);
}

// (d) w_trans_demo from the end joints, cross-checked against compute_w_trans_demo.py.
TEST_F(SelectionInputsTest, WTransDemoFromEndJoints) {
    RobotModel ref = makeRobot();
    dp::DemoParams p = makeFixture(60.0);
    p.end_joints_rad = toArray(RobotModel::readyPose());
    rewrite(p);
    SelectionInputs a = build();
    ref.update(RobotModel::readyPose(), RobotModel::JointVector::Zero());
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
