#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "haptic_dmp_learning/core/joint_states_csv_io.hpp"

using namespace haptic_dmp_learning::core::joint_states_csv_io;

TEST(JointStatesCsvIoTest, MapsByNameNotByPosition) {
    // Shuffled order plus extra joints (finger joints) that must be ignored.
    const std::vector<std::string> names = {"fer_finger_joint1", "fer_joint7", "fer_joint3",
                                            "fer_joint1", "fer_joint2", "fer_joint6",
                                            "fer_joint5", "fer_joint4"};
    const std::vector<double> pos = {9.0, 7.0, 3.0, 1.0, 2.0, 6.0, 5.0, 4.0};
    std::array<double, kNumJoints> q{};
    ASSERT_TRUE(extractJointPositions(names, pos, q));
    for (std::size_t j = 0; j < kNumJoints; ++j) EXPECT_DOUBLE_EQ(q[j], j + 1.0);
}

TEST(JointStatesCsvIoTest, MissingJointIsRejected) {
    std::vector<std::string> names = {"fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4",
                                      "fer_joint5", "fer_joint6"};  // no fer_joint7
    std::vector<double> pos(names.size(), 0.0);
    std::array<double, kNumJoints> q{};
    EXPECT_FALSE(extractJointPositions(names, pos, q));
    // names longer than positions: the unmatched joint counts as missing.
    names.push_back("fer_joint7");
    EXPECT_FALSE(extractJointPositions(names, pos, q));
}

TEST(JointStatesCsvIoTest, DerivesPathFromDemoPath) {
    EXPECT_EQ(deriveJointStatesCsvPath("/w/live_demo_raw.csv"), "/w/live_demo_joint_states.csv");
    EXPECT_EQ(deriveJointStatesCsvPath("/w/demo_raw.csv"), "/w/demo_joint_states.csv");
    EXPECT_EQ(deriveJointStatesCsvPath("/w/my_demo.csv"), "/w/my_demo_joint_states.csv");
    EXPECT_EQ(deriveJointStatesCsvPath(""), "");
}

TEST(JointStatesCsvIoTest, WritesExpectedHeaderAndRows) {
    const std::string path = ::testing::TempDir() + "joint_states_io_test.csv";
    std::vector<JointStateSample> s(2);
    s[0].t = 0.0;
    s[1].t = 0.5;
    for (std::size_t j = 0; j < kNumJoints; ++j) {
        s[0].q[j] = 0.1 * static_cast<double>(j);
        s[1].q[j] = -0.2 * static_cast<double>(j + 1);
    }
    writeJointStatesCsv(path, s);

    std::ifstream f(path);
    std::string line;
    std::getline(f, line);
    EXPECT_EQ(line,
              "t,fer_joint1,fer_joint2,fer_joint3,fer_joint4,fer_joint5,fer_joint6,fer_joint7");
    std::getline(f, line);
    EXPECT_EQ(line, "0,0,0.1,0.2,0.3,0.4,0.5,0.6");
    std::getline(f, line);
    EXPECT_EQ(line.rfind("0.5,-0.2,-0.4,-0.6", 0), 0u);
    std::remove(path.c_str());
}

TEST(JointStatesCsvIoTest, EmptyBufferThrowsAndWritesNoFile) {
    const std::string path = ::testing::TempDir() + "joint_states_io_empty.csv";
    std::remove(path.c_str());
    EXPECT_THROW(writeJointStatesCsv(path, {}), std::invalid_argument);
    EXPECT_FALSE(std::ifstream(path).good());
}
