#pragma once

#include <string>
#include "haptic_dmp_learning/core/prodmp.hpp"

namespace haptic_dmp_learning {
namespace core {
namespace prodmp_io {

/**
 * @brief Serializes an integral-form core::ProDMP model to a YAML file.
 *
 * This is a SEPARATE serializer from dmp_io (classic core::DMP). The two file
 * formats are independent so a run can carry both a classic-DMP model and a
 * ProDMP model side by side for A/B comparison. dmp_io is not touched.
 *
 * @param prodmp   Trained ProDMP object.
 * @param filepath Destination filesystem path.
 */
void saveProDmpToYaml(const ProDMP& prodmp, const std::string& filepath);

/**
 * @brief Deserializes an integral-form core::ProDMP model from a YAML file.
 * @param filepath Source YAML filesystem path.
 * @return ProDMP Reconstructed instance (ready to roll out from its stored start).
 */
ProDMP loadProDmpFromYaml(const std::string& filepath);

/**
 * @brief Feature-flag values read from config/prodmp_features.yaml (the
 * ProDMP equivalent of dmp_features.yaml).
 *
 * num_basis / ridge_lambda are CONSTRUCTOR-ONLY on core::ProDMP (by design -
 * see core/prodmp.hpp - there is no post-construction setter, unlike
 * DMP::setRidgeRegression()), so a caller must read them via
 * loadProDmpFeatureConfig() BEFORE constructing the ProDMP and pass them to
 * the ProDMP(...) constructor itself. Only the position filter can be applied
 * after construction - see applyProDmpFeatureConfig() below.
 */
struct ProDmpFeatureConfig {
    int num_basis = 20;
    double ridge_lambda = 1e-9;
    bool position_filter_enabled = false;
    double position_filter_window_sec = 0.20;
    /// See ProDMP::setFixGoalToDemoEndpoint(): false (default) = goal estimated
    /// jointly with the shape weights, same behaviour as before this flag existed.
    bool fix_goal_to_demo_endpoint = false;
};

/**
 * @brief Loads config/prodmp_features.yaml, same search/fail-loud pattern as
 * dmp_io::applyFeatureConfig: tries @p filepath first (if non-empty), then a
 * fixed list of $HOME-relative fallbacks, and throws std::runtime_error if
 * none can be loaded (no silent fallback to hardcoded defaults).
 */
ProDmpFeatureConfig loadProDmpFeatureConfig(const std::string& filepath);

/**
 * @brief Applies the POST-CONSTRUCTION knobs from prodmp_features.yaml (i.e.
 * the position filter window and fix_goal_to_demo_endpoint) onto an
 * already-constructed ProDMP. Uses the same load/fail-loud behavior as
 * loadProDmpFeatureConfig().
 *
 * @param filepath Path to prodmp_features.yaml (or empty to use the fallback chain).
 * @param prodmp   ProDMP already constructed with the desired num_basis/ridge_lambda.
 */
void applyProDmpFeatureConfig(const std::string& filepath, ProDMP& prodmp);

}  // namespace prodmp_io
}  // namespace core
}  // namespace haptic_dmp_learning
