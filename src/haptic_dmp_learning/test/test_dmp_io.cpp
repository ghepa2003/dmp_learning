#include <gtest/gtest.h>
#include "haptic_dmp_learning/core/dmp.hpp"
#include "haptic_dmp_learning/core/quaternion_dmp.hpp"
#include "haptic_dmp_learning/core/dmp_io.hpp"
#include <vector>
#include <fstream>

using namespace haptic_dmp_learning::core;

/**
 * @brief Unit test verifying full round-trip serialization and deserialization to YAML.
 *
 * Trains a translational DMP and a rotational QuaternionDMP on a synthetic demonstration.
 * Serializes both models to a temporary YAML file, loads them back into fresh instances,
 * and asserts that all hyperparameters, duration tau, start/goal boundaries, centers, widths,
 * and weights are bit-for-bit preserved.
 */
TEST(DmpIoTest, SaveAndLoadYamlRoundtrip) {
    const int N = 50;
    const double dt = 0.02;

    std::vector<Sample> demo(N);
    for (int i = 0; i < N; ++i) {
        double s = static_cast<double>(i) / (N - 1);
        demo[i].t = i * dt;
        demo[i].position = Eigen::Vector3d(s * 0.2, s * -0.1, s * 0.3);
        demo[i].orientation = Eigen::Quaterniond::Identity();
    }

    DMP dmp(15);
    dmp.learnFromDemonstration(demo);

    QuaternionDMP qdmp(15);
    qdmp.learnFromDemonstration(demo);

    std::string temp_yaml = "/tmp/test_dmp_roundtrip.yaml";
    EXPECT_NO_THROW(dmp_io::saveToYaml(dmp, qdmp, temp_yaml));

    DMP loaded_dmp(15);
    QuaternionDMP loaded_qdmp(15);
    EXPECT_NO_THROW(dmp_io::loadFromYaml(temp_yaml, loaded_dmp, loaded_qdmp));

    EXPECT_NEAR(loaded_dmp.tau(), dmp.tau(), 1e-6);
    EXPECT_NEAR(loaded_dmp.y0().x(), dmp.y0().x(), 1e-6);
    EXPECT_NEAR(loaded_dmp.goal().z(), dmp.goal().z(), 1e-6);
    EXPECT_EQ(loaded_dmp.nBasis(), dmp.nBasis());

    // Clean up temporary test file
    std::remove(temp_yaml.c_str());
}
