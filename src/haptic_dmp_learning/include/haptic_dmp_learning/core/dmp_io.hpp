#pragma once
#include <string>
#include "haptic_dmp_learning/core/dmp.hpp"
#include "haptic_dmp_learning/core/quaternion_dmp.hpp" 

namespace haptic_dmp_learning {
namespace core {
namespace dmp_io {

// Saves the learned DMP parameters to a YAML file at the specified filepath.
void saveToYaml(const DMP& dmp, const std::string& filepath);

// Loads the learned DMP parameters from a YAML file at the specified filepath and returns a DMP object.
DMP loadFromYaml(const std::string& filepath);

// Saves the learned QuaternionDMP parameters to a YAML file at the specified filepath.
void saveToYaml(const DMP& dmp, const QuaternionDMP& qdmp, const std::string& filepath);

// Loads the learned DMP and QuaternionDMP parameters from a YAML file at the specified filepath and returns both objects.
void loadFromYaml(const std::string& filepath, DMP& dmp, QuaternionDMP& qdmp);

// Applies the feature configuration from a YAML file at the specified filepath to the provided DMP and QuaternionDMP objects.
void applyFeatureConfig(const std::string& filepath, DMP& dmp, QuaternionDMP& qdmp);

}  // namespace dmp_io
}  // namespace core
}  // namespace haptic_dmp_learning
