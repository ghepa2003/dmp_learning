#pragma once

#include <string>
#include <yaml-cpp/yaml.h>
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

/**
 * @brief Converts a rotational QuaternionDMP instance into a structured YAML node hierarchy
 * (the exact `quaternion_dmp:` schema used by saveToYaml(DMP, QuaternionDMP, path)/loadFromYaml).
 *
 * Exposed (in addition to the internal use within this file) so other serializers - e.g.
 * prodmp_io's unified ProDMP+orientation format - can embed an identical `quaternion_dmp:`
 * section without hand-rolling a divergent schema.
 */
YAML::Node quaternionDmpToNode(const QuaternionDMP& qdmp);

/**
 * @brief Reconstructs a QuaternionDMP from a `quaternion_dmp:` YAML node produced by
 * quaternionDmpToNode() (same schema read by loadFromYaml(path, DMP&, QuaternionDMP&)).
 */
QuaternionDMP quaternionDmpFromNode(const YAML::Node& node);

}  // namespace dmp_io
}  // namespace core
}  // namespace haptic_dmp_learning
