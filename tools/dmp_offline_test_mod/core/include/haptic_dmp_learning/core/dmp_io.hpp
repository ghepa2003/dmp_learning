#pragma once
#include <string>
#include "haptic_dmp_learning/core/dmp.hpp"

namespace haptic_dmp_learning {
namespace core {

struct DMPConfig {
    int n_basis = 20;
    double alpha_x = 4.6;
    double alpha_z = 25.0;
    double beta_z = 6.25;
};

namespace dmp_io {

// Reads hyper-parameters (n_basis, alpha_x, alpha_z, beta_z) from a YAML file
DMPConfig loadConfigFromYaml(const std::string& filepath);

// Creates a DMP initialized with parameters loaded from a YAML file
DMP createFromConfigYaml(const std::string& filepath);

// Minimal YAML (de)serialization for a learned DMP. Deliberately kept
// separate from the DMP class itself, so DMP has zero I/O dependencies
// (only Eigen) and stays trivially unit-testable.
void saveToYaml(const DMP& dmp, const std::string& filepath);
DMP loadFromYaml(const std::string& filepath);

}  // namespace dmp_io
}  // namespace core
}  // namespace haptic_dmp_learning
