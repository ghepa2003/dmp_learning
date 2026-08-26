#pragma once

#include <string>
#include "haptic_dmp_learning/core/dmp.hpp"
#include "haptic_dmp_learning/core/quaternion_dmp.hpp" 

namespace haptic_dmp_learning {
namespace core {
namespace dmp_io {

/**
 * @brief Serializes a 3D Cartesian DMP model (weights, centers, widths, time scale, gains) into a YAML file.
 * @param dmp Trained DMP object.
 * @param filepath Destination filesystem path.
 */
void saveToYaml(const DMP& dmp, const std::string& filepath);

/**
 * @brief Deserializes a 3D Cartesian DMP model from a YAML file.
 * @param filepath Source YAML filesystem path.
 * @return DMP Reconstructed DMP instance.
 */
DMP loadFromYaml(const std::string& filepath);

/**
 * @brief Serializes both translational DMP and rotational QuaternionDMP models together into a YAML file.
 * @param dmp Trained translational DMP object.
 * @param qdmp Trained rotational QuaternionDMP object.
 * @param filepath Destination filesystem path.
 */
void saveToYaml(const DMP& dmp, const QuaternionDMP& qdmp, const std::string& filepath);

/**
 * @brief Deserializes both translational DMP and rotational QuaternionDMP models from a YAML file.
 * @param filepath Source YAML filesystem path.
 * @param[out] dmp Output reconstructed translational DMP instance.
 * @param[out] qdmp Output reconstructed rotational QuaternionDMP instance.
 */
void loadFromYaml(const std::string& filepath, DMP& dmp, QuaternionDMP& qdmp);

/**
 * @brief Reads algorithmic feature flags (ridge regression lambda, velocity filter windows) from YAML and configures DMP instances.
 * @param filepath Path to dmp_features.yaml configuration file.
 * @param[out] dmp Translational DMP to configure.
 * @param[out] qdmp Rotational QuaternionDMP to configure.
 */
void applyFeatureConfig(const std::string& filepath, DMP& dmp, QuaternionDMP& qdmp);

}  // namespace dmp_io
}  // namespace core
}  // namespace haptic_dmp_learning
