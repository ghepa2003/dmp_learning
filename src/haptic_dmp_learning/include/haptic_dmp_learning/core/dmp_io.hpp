#pragma once
#include <string>
#include "haptic_dmp_learning/core/dmp.hpp"
#include "haptic_dmp_learning/core/quaternion_dmp.hpp" 

namespace haptic_dmp_learning {
namespace core {
namespace dmp_io {

// Minimal YAML (de)serialization for a learned DMP. 
void saveToYaml(const DMP& dmp, const std::string& filepath);
DMP loadFromYaml(const std::string& filepath);

void saveToYaml(const DMP& dmp, const QuaternionDMP& qdmp, const std::string& filepath);
void loadFromYaml(const std::string& filepath, DMP& dmp, QuaternionDMP& qdmp);
void applyFeatureConfig(const std::string& filepath, DMP& dmp, QuaternionDMP& qdmp);

}  // namespace dmp_io
}  // namespace core
}  // namespace haptic_dmp_learning
