#pragma once

#include <string>
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/quaternion_dmp.hpp"

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
 * @brief Serializes a ProDMP model, optionally with an embedded orientation QuaternionDMP, to a
 * YAML file - the "unified" format.
 *
 * When @p qdmp is nullptr, the output is BYTE-IDENTICAL to saveProDmpToYaml(prodmp, filepath)
 * above (no `quaternion_dmp:` key is emitted at all) - this is the default/legacy behaviour.
 * When @p qdmp is non-null, an additional top-level `quaternion_dmp:` section is appended using
 * the exact same field names as dmp_io's classic-DMP `quaternion_dmp:` section (see
 * dmp_io::quaternionDmpToNode, reused here directly) so any consumer already able to parse that
 * section elsewhere can parse it here unchanged.
 *
 * @param prodmp   Trained ProDMP object (position).
 * @param qdmp     Optional trained QuaternionDMP (orientation); nullptr = position-only output.
 * @param filepath Destination filesystem path.
 */
void saveProDmpToYaml(const ProDMP& prodmp, const QuaternionDMP* qdmp, const std::string& filepath);

/**
 * @brief Deserializes a ProDMP model from a YAML file, additionally reporting/returning an
 * embedded `quaternion_dmp:` orientation section if present.
 *
 * Does NOT fabricate orientation data: if no `quaternion_dmp:` key exists in the file,
 * @p has_orientation is set to false and @p qdmp_out is left untouched - callers must check
 * @p has_orientation before using @p qdmp_out.
 *
 * @param filepath        Source YAML filesystem path.
 * @param[out] qdmp_out   Populated with the parsed QuaternionDMP iff an orientation section
 *                        was present.
 * @param[out] has_orientation  True iff a `quaternion_dmp:` section was found and parsed.
 * @return ProDMP Reconstructed position instance (same as loadProDmpFromYaml above).
 */
ProDMP loadProDmpFromYaml(const std::string& filepath, QuaternionDMP& qdmp_out,
                          bool& has_orientation);

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
