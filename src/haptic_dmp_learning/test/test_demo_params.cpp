// Tests for core/demo_params.hpp (no ROS). Temporary files live in a unique directory under /tmp.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "haptic_dmp_learning/core/demo_csv_io.hpp"
#include "haptic_dmp_learning/core/demo_params.hpp"

using namespace haptic_dmp_learning::core;
using namespace haptic_dmp_learning::core::demo_params;

namespace {

/// Unique directory /tmp/demo_params_XXXXXX, removed at the end of the test.
struct TempDir {
    std::string path;
    TempDir() {
        char tmpl[] = "/tmp/demo_params_XXXXXX";
        const char* d = mkdtemp(tmpl);
        path = d ? std::string(d) : std::string("/tmp");
    }
    ~TempDir() { std::filesystem::remove_all(path); }
    std::string file(const std::string& name) const { return path + "/" + name; }
};

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

std::string abcHash() { return sha256Hex("abc"); }

DemoParams makeFull() {
    DemoParams p;
    p.weights_file = "weights.yaml";
    p.weights_sha256 = abcHash();
    p.tau_s = 60.972930046000002;
    p.t_contact_s = 12.25;
    p.delta_g_demo_m = Eigen::Vector3d(0.1, 1.0 / 3.0, -M_PI / 7.0);
    p.contact_to_end_offset_m = Eigen::Vector3d(1e-17, -0.02, 0.5);
    p.start_position_demo_frame_m = Eigen::Vector3d(0.30689, 0.0, 0.48699);
    std::array<double, 7> sj{};
    for (int i = 0; i < 7; ++i) sj[i] = 0.1 * (i + 1) / 3.0;
    p.start_joints_rad = sj;
    std::array<double, 7> e{};
    for (int i = 0; i < 7; ++i) e[i] = -0.2 * (i + 1) / 7.0;
    p.end_joints_rad = e;
    SatelliteAtDemo s;
    s.center_m = Eigen::Vector3d(0.75, 0.0, 0.35);
    s.axis = Eigen::Vector3d(0.0, 0.0, 1.0);
    s.omega_rad_s = -2.0 * M_PI / 180.0;
    s.grasp_point = GraspPointId::kP270;
    s.phase_at_contact_rad = -M_PI / 3.0;
    p.satellite_at_demo = s;
    p.fit.num_basis = 80;
    p.fit.ridge_lambda = 1e-10;
    p.fit.position_filter_window_s = 0.05;
    p.fit.fix_goal_to_demo_endpoint = true;
    return p;
}

void expectVec(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    for (int i = 0; i < 3; ++i) EXPECT_DOUBLE_EQ(a(i), b(i));
}

/// Replaces the whole line whose trimmed text starts with "<key>:" by @p new_line ("" removes it).
std::string setLine(const std::string& text, const std::string& key, const std::string& new_line) {
    std::istringstream in(text);
    std::string line, out;
    bool hit = false;
    while (std::getline(in, line)) {
        const std::size_t first = line.find_first_not_of(' ');
        if (first != std::string::npos && line.compare(first, key.size() + 1, key + ":") == 0) {
            hit = true;
            if (!new_line.empty()) out += new_line + "\n";
        } else {
            out += line + "\n";
        }
    }
    EXPECT_TRUE(hit) << "key not found in test text: " << key;
    return out;
}

void expectReadThrowsWith(const std::string& path, const std::string& needle) {
    try {
        read(path);
        ADD_FAILURE() << "read() did not throw (expected mention of '" << needle << "')";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find(needle), std::string::npos) << e.what();
    }
}

}  // namespace

TEST(DemoParamsSha256, KnownVectors) {
    EXPECT_EQ(sha256Hex("abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    EXPECT_EQ(sha256Hex(""), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
}

TEST(DemoParamsRoundTrip, FullFile) {
    TempDir d;
    const DemoParams in = makeFull();
    write(d.file("p.yaml"), in);
    const DemoParams out = read(d.file("p.yaml"));
    EXPECT_EQ(out.format_version, 1);
    EXPECT_EQ(out.weights_file, in.weights_file);
    EXPECT_EQ(out.weights_sha256, in.weights_sha256);
    EXPECT_DOUBLE_EQ(out.tau_s, in.tau_s);
    ASSERT_TRUE(out.t_contact_s.has_value());
    EXPECT_DOUBLE_EQ(*out.t_contact_s, *in.t_contact_s);
    expectVec(out.delta_g_demo_m, in.delta_g_demo_m);
    expectVec(out.contact_to_end_offset_m, in.contact_to_end_offset_m);
    expectVec(out.start_position_demo_frame_m, in.start_position_demo_frame_m);
    ASSERT_TRUE(out.start_joints_rad.has_value());
    for (int i = 0; i < 7; ++i) EXPECT_DOUBLE_EQ((*out.start_joints_rad)[i], (*in.start_joints_rad)[i]);
    ASSERT_TRUE(out.end_joints_rad.has_value());
    for (int i = 0; i < 7; ++i) EXPECT_DOUBLE_EQ((*out.end_joints_rad)[i], (*in.end_joints_rad)[i]);
    ASSERT_TRUE(out.satellite_at_demo.has_value());
    const SatelliteAtDemo& a = *out.satellite_at_demo;
    const SatelliteAtDemo& b = *in.satellite_at_demo;
    expectVec(a.center_m, b.center_m);
    expectVec(a.axis, b.axis);
    EXPECT_DOUBLE_EQ(a.omega_rad_s, b.omega_rad_s);
    EXPECT_EQ(a.grasp_point, b.grasp_point);
    EXPECT_DOUBLE_EQ(a.phase_at_contact_rad, b.phase_at_contact_rad);
    EXPECT_EQ(out.fit.num_basis, in.fit.num_basis);
    EXPECT_DOUBLE_EQ(out.fit.ridge_lambda, in.fit.ridge_lambda);
    EXPECT_DOUBLE_EQ(out.fit.position_filter_window_s, in.fit.position_filter_window_s);
    EXPECT_EQ(out.fit.fix_goal_to_demo_endpoint, in.fit.fix_goal_to_demo_endpoint);
}

TEST(DemoParamsRoundTrip, EmptyOptionalsAreNull) {
    TempDir d;
    DemoParams in = makeFull();
    in.t_contact_s.reset();
    in.start_joints_rad.reset();
    in.end_joints_rad.reset();
    in.satellite_at_demo.reset();
    write(d.file("p.yaml"), in);
    const std::string text = readAll(d.file("p.yaml"));
    EXPECT_NE(text.find("start_joints_rad: null"), std::string::npos);
    EXPECT_NE(text.find("t_contact_s: null"), std::string::npos);
    EXPECT_NE(text.find("end_joints_rad: null"), std::string::npos);
    EXPECT_NE(text.find("satellite_at_demo: null"), std::string::npos);
    const DemoParams out = read(d.file("p.yaml"));
    EXPECT_FALSE(out.t_contact_s.has_value());
    EXPECT_FALSE(out.start_joints_rad.has_value());
    EXPECT_FALSE(out.end_joints_rad.has_value());
    EXPECT_FALSE(out.satellite_at_demo.has_value());
}

TEST(DemoParamsRead, MissingRequiredKeyNamesTheKey) {
    TempDir d;
    write(d.file("p.yaml"), makeFull());
    const std::string text = readAll(d.file("p.yaml"));
    for (const char* key : {"tau_s", "num_basis", "format_version", "weights_sha256", "start_joints_rad",
                            "omega_rad_s", "fix_goal_to_demo_endpoint"}) {
        writeText(d.file("m.yaml"), setLine(text, key, ""));
        expectReadThrowsWith(d.file("m.yaml"), key);
    }
}

TEST(DemoParamsRead, AbsentOptionalKeyIsAnErrorButNullIsFine) {
    TempDir d;
    DemoParams in = makeFull();
    in.t_contact_s.reset();
    in.start_joints_rad.reset();
    in.end_joints_rad.reset();
    in.satellite_at_demo.reset();
    write(d.file("p.yaml"), in);
    const std::string text = readAll(d.file("p.yaml"));
    EXPECT_NO_THROW(read(d.file("p.yaml")));
    for (const char* key : {"t_contact_s", "start_joints_rad", "end_joints_rad", "satellite_at_demo"}) {
        writeText(d.file("m.yaml"), setLine(text, key, ""));
        expectReadThrowsWith(d.file("m.yaml"), key);
    }
}

TEST(DemoParamsRead, UnknownExtraKeyIsTolerated) {
    TempDir d;
    write(d.file("p.yaml"), makeFull());
    writeText(d.file("x.yaml"), readAll(d.file("p.yaml")) + "some_future_key: 42\n");
    const DemoParams out = read(d.file("x.yaml"));
    EXPECT_DOUBLE_EQ(out.tau_s, makeFull().tau_s);
    EXPECT_EQ(out.fit.num_basis, 80);
}

TEST(DemoParamsRead, FormatVersion2Throws) {
    TempDir d;
    write(d.file("p.yaml"), makeFull());
    writeText(d.file("m.yaml"), setLine(readAll(d.file("p.yaml")), "format_version", "format_version: 2"));
    expectReadThrowsWith(d.file("m.yaml"), "format_version");
}

TEST(DemoParamsRead, ValidationFailures) {
    TempDir d;
    write(d.file("p.yaml"), makeFull());
    const std::string text = readAll(d.file("p.yaml"));
    struct Case {
        const char* key;
        const char* new_line;
    };
    const std::vector<Case> cases = {
        {"tau_s", "  tau_s: -1"},
        {"tau_s", "  tau_s: 0"},
        {"t_contact_s", "  t_contact_s: 70"},
        {"t_contact_s", "  t_contact_s: -0.5"},
        {"start_joints_rad", "  start_joints_rad: [0, 0, 0, 0, 0, 0]"},
        {"weights_sha256", "weights_sha256: ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f2001"},
        {"weights_sha256", "weights_sha256: BA7816BF8F01CFEA414140DE5DAE2223B00361A396177A9CB410FF61F20015AD"},
        {"grasp_point", "  grasp_point: kP45"},
        {"axis", "  axis: [0, 0, 0]"},
        {"omega_rad_s", "  omega_rad_s: 0"},
        {"delta_g_demo_m", "  delta_g_demo_m: [0.1, .nan, 0.0]"},
    };
    for (const Case& c : cases) {
        writeText(d.file("m.yaml"), setLine(text, c.key, c.new_line));
        expectReadThrowsWith(d.file("m.yaml"), c.key);
    }
}

TEST(DemoParamsAlignment, MatchesMismatchesAndMissing) {
    TempDir d;
    writeText(d.file("weights.yaml"), "abc");
    DemoParams p = makeFull();  // weights_file "weights.yaml", hash of "abc"
    write(d.file("weights_demo_params.yaml"), p);
    EXPECT_NO_THROW(verifyWeightsAlignment(d.file("weights_demo_params.yaml"), p));

    writeText(d.file("weights.yaml"), "abd");  // one byte changed
    try {
        verifyWeightsAlignment(d.file("weights_demo_params.yaml"), p);
        ADD_FAILURE() << "expected a mismatch";
    } catch (const std::runtime_error& e) {
        const std::string m = e.what();
        EXPECT_NE(m.find(sha256Hex("abd")), std::string::npos) << m;
        EXPECT_NE(m.find(p.weights_sha256), std::string::npos) << m;
    }

    std::filesystem::remove(d.file("weights.yaml"));
    EXPECT_THROW(verifyWeightsAlignment(d.file("weights_demo_params.yaml"), p), std::runtime_error);
}

TEST(DemoParamsPaths, DeriveDemoParamsPath) {
    EXPECT_EQ(deriveDemoParamsPath("/a/b/weights.yaml"), "/a/b/weights_demo_params.yaml");
    EXPECT_EQ(deriveDemoParamsPath("/a/prodmp_weights_r1.yaml"), "/a/prodmp_weights_r1_demo_params.yaml");
}

TEST(DemoParamsRead, MissingFileThrows) {
    EXPECT_THROW(read("/tmp/definitely_missing_demo_params_file.yaml"), std::runtime_error);
}

// ---------------------------------------------------------------- buildFromDemo

namespace {

/// 5 samples, t = 0.5 + 0.5*i (first sample not at 0), position (i, 2i, -i)*0.1; trigger at index @p trig.
std::vector<Sample> makeDemo(int trig) {
    std::vector<Sample> demo;
    for (int i = 0; i < 5; ++i) {
        Sample s;
        s.t = 0.5 + 0.5 * i;
        s.position = Eigen::Vector3d(0.1 * i, 0.2 * i, -0.1 * i);
        s.gripper_trigger = (i == trig);
        demo.push_back(s);
    }
    return demo;
}

FitInfo makeFit() {
    FitInfo f;
    f.num_basis = 80;
    f.ridge_lambda = 1e-10;
    f.position_filter_window_s = 0.05;
    f.fix_goal_to_demo_endpoint = true;
    return f;
}

}  // namespace

TEST(DemoParamsBuild, TriggerAtThirdSample) {
    TempDir d;
    writeText(d.file("weights.yaml"), "abc");
    const DemoParams p = buildFromDemo(makeDemo(2), std::nullopt, makeFit(), d.file("weights.yaml"), std::nullopt);
    EXPECT_NEAR(p.tau_s, 2.0, 1e-12);  // 2.5 - 0.5
    ASSERT_TRUE(p.t_contact_s.has_value());
    EXPECT_NEAR(*p.t_contact_s, 1.0, 1e-12);  // t = 1.5 minus 0.5
    // final (0.4, 0.8, -0.4) minus at contact (0.2, 0.4, -0.2)
    expectVec(p.contact_to_end_offset_m, Eigen::Vector3d(0.2, 0.4, -0.2));
    expectVec(p.delta_g_demo_m, Eigen::Vector3d(0.4, 0.8, -0.4));
    expectVec(p.start_position_demo_frame_m, Eigen::Vector3d::Zero());
    EXPECT_FALSE(p.start_joints_rad.has_value());
    EXPECT_FALSE(p.end_joints_rad.has_value());
    EXPECT_FALSE(p.satellite_at_demo.has_value());
    EXPECT_EQ(p.fit.num_basis, 80);
}

TEST(DemoParamsBuild, NoTriggerMeansNulloptAndZeroOffset) {
    TempDir d;
    writeText(d.file("weights.yaml"), "abc");
    const DemoParams p = buildFromDemo(makeDemo(-1), std::nullopt, makeFit(), d.file("weights.yaml"), std::nullopt);
    EXPECT_FALSE(p.t_contact_s.has_value());
    expectVec(p.contact_to_end_offset_m, Eigen::Vector3d::Zero());
}

TEST(DemoParamsBuild, JointStatesFirstAndLastRow) {
    TempDir d;
    writeText(d.file("weights.yaml"), "abc");
    std::vector<joint_states_csv_io::JointStateSample> js(3);
    for (int r = 0; r < 3; ++r) {
        js[r].t = r;
        for (std::size_t j = 0; j < 7; ++j) js[r].q[j] = 10.0 * r + j;
    }
    const DemoParams p = buildFromDemo(makeDemo(2), js, makeFit(), d.file("weights.yaml"), std::nullopt);
    ASSERT_TRUE(p.start_joints_rad.has_value());
    ASSERT_TRUE(p.end_joints_rad.has_value());
    for (std::size_t j = 0; j < 7; ++j) {
        EXPECT_DOUBLE_EQ((*p.start_joints_rad)[j], static_cast<double>(j));
        EXPECT_DOUBLE_EQ((*p.end_joints_rad)[j], 20.0 + static_cast<double>(j));
    }
    EXPECT_THROW(buildFromDemo(makeDemo(2), std::vector<joint_states_csv_io::JointStateSample>{}, makeFit(),
                               d.file("weights.yaml"), std::nullopt),
                 std::invalid_argument);
}

TEST(DemoParamsBuild, WeightsHashAlignsAndSatelliteIsCarried) {
    TempDir d;
    writeText(d.file("weights.yaml"), "abc");
    SatelliteAtDemo sat;
    sat.omega_rad_s = -0.03;
    const DemoParams p = buildFromDemo(makeDemo(2), std::nullopt, makeFit(), d.file("weights.yaml"), sat);
    EXPECT_EQ(p.weights_file, "weights.yaml");
    EXPECT_EQ(p.weights_sha256, sha256Hex("abc"));
    ASSERT_TRUE(p.satellite_at_demo.has_value());
    write(d.file("weights_demo_params.yaml"), p);
    EXPECT_NO_THROW(verifyWeightsAlignment(d.file("weights_demo_params.yaml"), read(d.file("weights_demo_params.yaml"))));
}

TEST(DemoParamsBuild, InvalidDemosThrow) {
    TempDir d;
    writeText(d.file("weights.yaml"), "abc");
    std::vector<Sample> one = {makeDemo(-1).front()};
    EXPECT_THROW(buildFromDemo(one, std::nullopt, makeFit(), d.file("weights.yaml"), std::nullopt),
                 std::invalid_argument);
    EXPECT_THROW(buildFromDemo({}, std::nullopt, makeFit(), d.file("weights.yaml"), std::nullopt),
                 std::invalid_argument);
    std::vector<Sample> bad = makeDemo(-1);
    bad[3].t = bad[2].t;  // repeated timestamp
    EXPECT_THROW(buildFromDemo(bad, std::nullopt, makeFit(), d.file("weights.yaml"), std::nullopt),
                 std::invalid_argument);
    bad = makeDemo(-1);
    bad[3].t = 0.1;  // backwards
    EXPECT_THROW(buildFromDemo(bad, std::nullopt, makeFit(), d.file("weights.yaml"), std::nullopt),
                 std::invalid_argument);
}

// Integration with the production demo; skipped if the files are not reachable.
TEST(DemoParamsBuild, ProductionDemoValues) {
    const char* home = std::getenv("HOME");
    const std::string root = std::string(home ? home : "/root") + "/thesis_ws/";
    const std::string csv = root + "reach_task_baseline.csv";
    const std::string weights =
        root + "runs/20260914_150515_fit_reach_task_baseline_prodmp_n80_lam1e-10_w0.05/weights.yaml";
    if (!std::ifstream(csv).good() || !std::ifstream(weights).good()) {
        GTEST_SKIP() << "production demo/weights not reachable: " << csv << " / " << weights;
    }
    const std::vector<Sample> demo = demo_csv_io::readDemoCsv(csv);
    const DemoParams p = buildFromDemo(demo, std::nullopt, makeFit(), weights, std::nullopt);
    EXPECT_NEAR(p.tau_s, 60.9729, 1e-3);
    EXPECT_NEAR(p.delta_g_demo_m.norm(), 0.236633, 1e-5);
    EXPECT_FALSE(p.t_contact_s.has_value());
}

TEST(DemoParamsBuild, ParseGraspPoint) {
    EXPECT_EQ(parseGraspPoint("kP0"), GraspPointId::kP0);
    EXPECT_EQ(parseGraspPoint("kP90"), GraspPointId::kP90);
    EXPECT_EQ(parseGraspPoint("kP180"), GraspPointId::kP180);
    EXPECT_EQ(parseGraspPoint("kP270"), GraspPointId::kP270);
    EXPECT_THROW(parseGraspPoint("kP45"), std::invalid_argument);
}

// writeForWeights: joints CSV derived from the demo CSV path is used when it exists, else null.
TEST(DemoParamsWriteForWeights, JointsFromDerivedCsvOrNull) {
    TempDir d;
    writeText(d.file("weights.yaml"), "abc");
    const std::vector<Sample> demo = makeDemo(2);
    std::ostringstream log;

    // No joint-states CSV next to "d.csv": start/end joints null.
    DemoParams out;
    std::string path = writeForWeights(d.file("d.csv"), demo, makeFit(), d.file("weights.yaml"), std::nullopt, log, &out);
    EXPECT_EQ(path, d.file("weights_demo_params.yaml"));
    EXPECT_FALSE(out.start_joints_rad.has_value());
    EXPECT_NE(log.str().find("no joint-states CSV"), std::string::npos);

    // With d_joint_states.csv next to it: used.
    std::vector<joint_states_csv_io::JointStateSample> js(2);
    for (std::size_t j = 0; j < 7; ++j) {
        js[0].q[j] = 0.5 + j;
        js[1].q[j] = -0.5 - j;
    }
    joint_states_csv_io::writeJointStatesCsv(d.file("d_joint_states.csv"), js);
    std::ostringstream log2;
    path = writeForWeights(d.file("d.csv"), demo, makeFit(), d.file("weights.yaml"), std::nullopt, log2, &out);
    ASSERT_TRUE(out.start_joints_rad.has_value());
    ASSERT_TRUE(out.end_joints_rad.has_value());
    EXPECT_NEAR((*out.start_joints_rad)[3], 3.5, 1e-9);
    EXPECT_NEAR((*out.end_joints_rad)[3], -3.5, 1e-9);
    EXPECT_NE(log2.str().find("joint states from"), std::string::npos);
    EXPECT_NO_THROW(verifyWeightsAlignment(path, read(path)));
}

// Integration with the production demo: writes next to a COPY of the production weights (the run
// directory itself is never touched) and checks the alignment; skipped if the files are missing.
TEST(DemoParamsWriteForWeights, ProductionDemoAlignsWithWeightsCopy) {
    const char* home = std::getenv("HOME");
    const std::string root = std::string(home ? home : "/root") + "/thesis_ws/";
    const std::string csv = root + "reach_task_baseline.csv";
    const std::string weights =
        root + "runs/20260914_150515_fit_reach_task_baseline_prodmp_n80_lam1e-10_w0.05/weights.yaml";
    if (!std::ifstream(csv).good() || !std::ifstream(weights).good()) {
        GTEST_SKIP() << "production demo/weights not reachable: " << csv << " / " << weights;
    }
    TempDir d;
    writeText(d.file("weights.yaml"), readAll(weights));  // byte copy
    const std::vector<Sample> demo = demo_csv_io::readDemoCsv(csv);
    std::ostringstream log;
    DemoParams out;
    const std::string path = writeForWeights(csv, demo, makeFit(), d.file("weights.yaml"), std::nullopt, log, &out);
    EXPECT_EQ(path, d.file("weights_demo_params.yaml"));
    EXPECT_NO_THROW(verifyWeightsAlignment(path, read(path)));
    EXPECT_EQ(out.weights_sha256, sha256FileHex(weights));
    EXPECT_NEAR(out.tau_s, 60.9729, 1e-3);
    EXPECT_NEAR(out.delta_g_demo_m.norm(), 0.236633, 1e-5);
    EXPECT_FALSE(out.t_contact_s.has_value());
    // A one-byte change of the weights breaks the alignment.
    writeText(d.file("weights.yaml"), readAll(weights) + "\n");
    EXPECT_THROW(verifyWeightsAlignment(path, read(path)), std::runtime_error);
}

// The two failure classes are told apart by TYPE (no string matching): repeated timestamps ->
// NonIncreasingTimestampsError; every other error -> some other exception.
TEST(DemoParamsErrors, NonIncreasingTimestampsHaveTheirOwnType) {
    TempDir d;
    writeText(d.file("weights.yaml"), "abc");
    std::vector<Sample> repeated = makeDemo(-1);
    repeated[3].t = repeated[2].t;

    EXPECT_THROW(buildFromDemo(repeated, std::nullopt, makeFit(), d.file("weights.yaml"), std::nullopt),
                 NonIncreasingTimestampsError);
    // ... and it is still an std::invalid_argument for callers that do not care.
    EXPECT_THROW(buildFromDemo(repeated, std::nullopt, makeFit(), d.file("weights.yaml"), std::nullopt),
                 std::invalid_argument);

    // writeForWeights propagates the type and writes nothing.
    std::ostringstream log;
    EXPECT_THROW(writeForWeights(d.file("d.csv"), repeated, makeFit(), d.file("weights.yaml"), std::nullopt, log),
                 NonIncreasingTimestampsError);
    EXPECT_FALSE(std::filesystem::exists(d.file("weights_demo_params.yaml")));

    // Other errors are NOT that type: too few samples, empty joint states, missing weights file,
    // invalid satellite (read-back validation), and the invalid file is not left behind.
    auto isOther = [&](auto&& fn) {
        try {
            fn();
        } catch (const NonIncreasingTimestampsError&) {
            return false;
        } catch (const std::exception&) {
            return true;
        }
        return false;  // did not throw at all
    };
    const std::vector<Sample> ok = makeDemo(2);
    EXPECT_TRUE(isOther([&] { buildFromDemo({ok.front()}, std::nullopt, makeFit(), d.file("weights.yaml"), std::nullopt); }));
    EXPECT_TRUE(isOther([&] {
        buildFromDemo(ok, std::vector<joint_states_csv_io::JointStateSample>{}, makeFit(), d.file("weights.yaml"),
                      std::nullopt);
    }));
    EXPECT_TRUE(isOther([&] {
        writeForWeights(d.file("d.csv"), ok, makeFit(), d.file("missing_weights.yaml"), std::nullopt, log);
    }));
    SatelliteAtDemo bad_sat;  // omega = 0 -> fails validation on read-back
    EXPECT_TRUE(isOther([&] {
        writeForWeights(d.file("d.csv"), ok, makeFit(), d.file("weights.yaml"), bad_sat, log);
    }));
    EXPECT_FALSE(std::filesystem::exists(d.file("weights_demo_params.yaml")));
}
