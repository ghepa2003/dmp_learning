#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "haptic_dmp_learning/core/demo_csv_io.hpp"
#include "haptic_dmp_learning/core/types.hpp"

using haptic_dmp_learning::core::Sample;
namespace demo_csv_io = haptic_dmp_learning::core::demo_csv_io;

namespace {

// Small unique-ish temp path per test; removed in each test's body.
std::string tmp(const std::string& name) {
    return "/tmp/test_demo_csv_io_" + name + ".csv";
}

Sample makeSample(double t, double x, bool trigger) {
    Sample s;
    s.t = t;
    s.position = Eigen::Vector3d(x, x + 0.1, x + 0.2);
    s.orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);  // identity, already unit
    s.gripper_trigger = trigger;
    return s;
}

void writeRaw(const std::string& path, const std::string& contents) {
    std::ofstream f(path);
    f << contents;
}

}  // namespace

// --- write / read roundtrip ------------------------------------------------

TEST(DemoCsvIo, WriteReadRoundtripPreservesSamplesAndTrigger) {
    const std::string path = tmp("roundtrip");

    std::vector<Sample> in;
    in.push_back(makeSample(0.00, 0.10, false));
    in.push_back(makeSample(0.25, 0.20, false));
    in.push_back(makeSample(0.50, 0.30, true));   // trigger not on row 0
    in.push_back(makeSample(0.75, 0.40, false));

    ASSERT_NO_THROW(demo_csv_io::writeDemoCsv(path, in));

    std::vector<Sample> out;
    ASSERT_NO_THROW(out = demo_csv_io::readDemoCsv(path));

    ASSERT_EQ(out.size(), in.size());
    for (size_t i = 0; i < in.size(); ++i) {
        EXPECT_NEAR(out[i].t, in[i].t, 1e-4);
        EXPECT_NEAR(out[i].position.x(), in[i].position.x(), 1e-4);
        EXPECT_NEAR(out[i].position.y(), in[i].position.y(), 1e-4);
        EXPECT_NEAR(out[i].position.z(), in[i].position.z(), 1e-4);
        EXPECT_NEAR(out[i].orientation.w(), 1.0, 1e-6);
        EXPECT_EQ(out[i].gripper_trigger, in[i].gripper_trigger);
    }

    std::remove(path.c_str());
}

TEST(DemoCsvIo, WriteAlwaysEmitsTheNineColumnHeader) {
    const std::string path = tmp("header");
    demo_csv_io::writeDemoCsv(path, {makeSample(0.0, 0.0, false)});

    std::ifstream f(path);
    std::string header;
    std::getline(f, header);
    EXPECT_EQ(header, "t,x,y,z,qw,qx,qy,qz,gripper_trigger");

    std::remove(path.c_str());
}

// --- backward compatibility: 8-column legacy files -----------------------

TEST(DemoCsvIo, ReadsEightColumnLegacyFileWithTriggerFalse) {
    const std::string path = tmp("legacy8");
    writeRaw(path,
             "t,x,y,z,qw,qx,qy,qz\n"
             "0,0.1,0.2,0.3,1,0,0,0\n"
             "0.5,0.4,0.5,0.6,1,0,0,0\n");

    std::vector<Sample> out;
    ASSERT_NO_THROW(out = demo_csv_io::readDemoCsv(path));
    ASSERT_EQ(out.size(), 2u);
    EXPECT_NEAR(out[0].t, 0.0, 1e-9);
    EXPECT_NEAR(out[1].t, 0.5, 1e-9);
    EXPECT_FALSE(out[0].gripper_trigger);
    EXPECT_FALSE(out[1].gripper_trigger);

    std::remove(path.c_str());
}

// --- readDemoCsv error paths -------------------------------------------

TEST(DemoCsvIo, ReadThrowsOnMissingFile) {
    EXPECT_THROW(demo_csv_io::readDemoCsv("/tmp/test_demo_csv_io_does_not_exist.csv"),
                 std::runtime_error);
}

TEST(DemoCsvIo, ReadThrowsOnHeaderOnlyFile) {
    const std::string path = tmp("headeronly");
    writeRaw(path, "t,x,y,z,qw,qx,qy,qz,gripper_trigger\n");
    EXPECT_THROW(demo_csv_io::readDemoCsv(path), std::runtime_error);
    std::remove(path.c_str());
}

TEST(DemoCsvIo, ReadThrowsOnShortRow) {
    const std::string path = tmp("shortrow");
    writeRaw(path,
             "t,x,y,z,qw,qx,qy,qz,gripper_trigger\n"
             "0,0.1,0.2,0.3,1,0,0\n");  // only 7 fields
    EXPECT_THROW(demo_csv_io::readDemoCsv(path), std::runtime_error);
    std::remove(path.c_str());
}

TEST(DemoCsvIo, WriteThrowsWhenPathUnopenable) {
    EXPECT_THROW(
        demo_csv_io::writeDemoCsv("/tmp/no_such_dir_xyz/out.csv", {makeSample(0, 0, false)}),
        std::runtime_error);
}

// --- readGripperTriggerTime ------------------------------------------------

TEST(DemoCsvIo, GripperTriggerTimeFoundOnNonZeroRow) {
    const std::string path = tmp("trigrow");
    // gripper_trigger == 1 on the 3rd data row, t = 0.50.
    writeRaw(path,
             "t,x,y,z,qw,qx,qy,qz,gripper_trigger\n"
             "0.00,0,0,0,1,0,0,0,0\n"
             "0.25,0,0,0,1,0,0,0,0\n"
             "0.50,0,0,0,1,0,0,0,1\n"
             "0.75,0,0,0,1,0,0,0,0\n");
    EXPECT_NEAR(demo_csv_io::readGripperTriggerTime(path), 0.50, 1e-9);
    std::remove(path.c_str());
}

TEST(DemoCsvIo, GripperTriggerTimeReturnsMinusOneWhenColumnPresentButNeverSet) {
    const std::string path = tmp("notrig");
    writeRaw(path,
             "t,x,y,z,qw,qx,qy,qz,gripper_trigger\n"
             "0.00,0,0,0,1,0,0,0,0\n"
             "0.25,0,0,0,1,0,0,0,0\n");
    EXPECT_DOUBLE_EQ(demo_csv_io::readGripperTriggerTime(path), -1.0);
    std::remove(path.c_str());
}

TEST(DemoCsvIo, GripperTriggerTimeReturnsMinusOneWhenNoColumn) {
    const std::string path = tmp("nocol");
    writeRaw(path,
             "t,x,y,z,qw,qx,qy,qz\n"
             "0.00,0,0,0,1,0,0,0\n");
    EXPECT_DOUBLE_EQ(demo_csv_io::readGripperTriggerTime(path), -1.0);
    std::remove(path.c_str());
}

TEST(DemoCsvIo, GripperTriggerTimeReturnsMinusOneWhenFileMissing) {
    EXPECT_DOUBLE_EQ(
        demo_csv_io::readGripperTriggerTime("/tmp/test_demo_csv_io_missing_trig.csv"),
        -1.0);
}

TEST(DemoCsvIo, GripperTriggerTimeToleratesCrlfHeader) {
    const std::string path = tmp("crlf");
    writeRaw(path,
             "t,x,y,z,qw,qx,qy,qz,gripper_trigger\r\n"
             "0.00,0,0,0,1,0,0,0,0\r\n"
             "0.40,0,0,0,1,0,0,0,1\r\n");
    EXPECT_NEAR(demo_csv_io::readGripperTriggerTime(path), 0.40, 1e-9);
    std::remove(path.c_str());
}
