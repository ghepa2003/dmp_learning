#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "haptic_dmp_learning/core/dmp_io.hpp"
#include <yaml-cpp/yaml.h>
#include <array>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <vector>

namespace haptic_dmp_learning {
namespace core {
namespace prodmp_io {

namespace {

// Local copies of the small serialization helpers - deliberately not shared
// with dmp_io so that file stays untouched.
YAML::Node vectorToYaml(const Eigen::VectorXd& v) {
    YAML::Node node(YAML::NodeType::Sequence);
    for (int i = 0; i < v.size(); ++i) node.push_back(v(i));
    return node;
}

YAML::Node vec3ToYaml(const Eigen::Vector3d& v) {
    YAML::Node node(YAML::NodeType::Sequence);
    node.push_back(v.x());
    node.push_back(v.y());
    node.push_back(v.z());
    return node;
}

Eigen::VectorXd yamlToVector(const YAML::Node& node) {
    Eigen::VectorXd v(static_cast<int>(node.size()));
    for (std::size_t i = 0; i < node.size(); ++i) {
        v(static_cast<int>(i)) = node[i].as<double>();
    }
    return v;
}

Eigen::Vector3d yamlToVec3(const YAML::Node& node) {
    return Eigen::Vector3d(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
}

/**
 * @brief Builds the position-only ProDMP YAML node - identical content/field order to the
 * original (pre-unified-format) saveProDmpToYaml body. Shared by both the position-only and
 * unified save entry points so the position section is guaranteed byte-identical either way.
 */
YAML::Node prodmpToNode(const ProDMP& prodmp) {
    YAML::Node node;
    node["formulation"] = "prodmp_integral";
    node["num_basis"] = prodmp.numBasis();
    node["alpha"] = prodmp.alpha();
    node["alpha_x"] = prodmp.alphaX();
    node["ridge_lambda"] = prodmp.ridgeLambda();  // provenance only, no effect on rollout
    node["tau"] = prodmp.tau();
    node["relative_goal"] = prodmp.relativeGoal();
    node["init_time"] = prodmp.initTime();
    node["init_pos"] = vec3ToYaml(prodmp.initPos());
    node["init_vel"] = vec3ToYaml(prodmp.initVel());
    node["goal"] = vec3ToYaml(prodmp.goal());              // absolute, for inspection
    node["goal_param"] = vec3ToYaml(prodmp.goalParam());   // the linear parameter actually restored
    node["centers"] = vectorToYaml(prodmp.centers());
    node["widths"] = vectorToYaml(prodmp.widths());

    YAML::Node weights(YAML::NodeType::Sequence);
    static const char* dim_names[3] = {"x", "y", "z"};
    for (int d = 0; d < 3; ++d) {
        YAML::Node wd;
        wd["dim"] = dim_names[d];
        wd["values"] = vectorToYaml(prodmp.weights()[d]);
        weights.push_back(wd);
    }
    node["weights"] = weights;
    return node;
}

ProDMP nodeToProdmp(const YAML::Node& root) {
    const int num_basis = root["num_basis"].as<int>();
    const double alpha = root["alpha"].as<double>();
    const double alpha_x = root["alpha_x"].as<double>();
    // Provenance only: the loaded model never re-fits, so this has no effect on
    // rollout behaviour - kept purely for diagnostic coherence with the file.
    const double ridge_lambda = root["ridge_lambda"] ? root["ridge_lambda"].as<double>() : 1e-9;
    const double tau = root["tau"].as<double>();
    const bool relative_goal = root["relative_goal"] ? root["relative_goal"].as<bool>() : false;

    const Eigen::Vector3d init_pos = yamlToVec3(root["init_pos"]);
    const Eigen::Vector3d init_vel = yamlToVec3(root["init_vel"]);
    const Eigen::Vector3d goal_param = yamlToVec3(root["goal_param"]);
    const Eigen::VectorXd centers = yamlToVector(root["centers"]);
    const Eigen::VectorXd widths = yamlToVector(root["widths"]);

    std::array<Eigen::VectorXd, 3> weights;
    for (const auto& wd : root["weights"]) {
        std::string dim = wd["dim"].as<std::string>();
        int idx = (dim == "x") ? 0 : (dim == "y") ? 1 : 2;
        weights[idx] = yamlToVector(wd["values"]);
    }

    ProDMP prodmp(num_basis, alpha, alpha_x, ridge_lambda);
    prodmp.setLearnedParameters(tau, init_pos, init_vel, goal_param, centers, widths, weights,
                                relative_goal);
    return prodmp;
}

}  // namespace

void saveProDmpToYaml(const ProDMP& prodmp, const std::string& filepath) {
    saveProDmpToYaml(prodmp, nullptr, filepath);
}

void saveProDmpToYaml(const ProDMP& prodmp, const QuaternionDMP* qdmp, const std::string& filepath) {
    YAML::Node node = prodmpToNode(prodmp);
    if (qdmp != nullptr) {
        // Reuse dmp_io's exact `quaternion_dmp:` schema (same field names) so any existing
        // reader of that section (classic-DMP files) parses this identically.
        node["quaternion_dmp"] = dmp_io::quaternionDmpToNode(*qdmp);
    }

    std::ofstream fout(filepath);
    if (!fout.is_open()) {
        throw std::runtime_error("prodmp_io::saveProDmpToYaml: cannot open file for writing: " +
                                 filepath);
    }
    fout << node;
}

ProDMP loadProDmpFromYaml(const std::string& filepath) {
    YAML::Node root = YAML::LoadFile(filepath);
    return nodeToProdmp(root);
}

ProDMP loadProDmpFromYaml(const std::string& filepath, QuaternionDMP& qdmp_out,
                          bool& has_orientation) {
    YAML::Node root = YAML::LoadFile(filepath);
    ProDMP prodmp = nodeToProdmp(root);

    has_orientation = false;
    if (root["quaternion_dmp"]) {
        qdmp_out = dmp_io::quaternionDmpFromNode(root["quaternion_dmp"]);
        has_orientation = true;
    }
    return prodmp;
}

ProDmpFeatureConfig loadProDmpFeatureConfig(const std::string& filepath) {
    YAML::Node root;
    std::vector<std::string> attempted;

    auto try_load = [&](const std::string& path) -> bool {
        attempted.push_back(path);
        try {
            root = YAML::LoadFile(path);
            return true;
        } catch (const std::exception&) {
            return false;
        }
    };

    bool loaded = !filepath.empty() && try_load(filepath);

    if (!loaded) {
        const char* home = std::getenv("HOME");
        const std::vector<std::string> fallbacks = {
            std::string(home ? home : "/root") +
                "/thesis_ws/src/haptic_dmp_learning/config/prodmp_features.yaml",
            std::string(home ? home : "/root") + "/thesis_ws/prodmp_features.yaml"
        };
        for (const auto& fpath : fallbacks) {
            if (try_load(fpath)) {
                loaded = true;
                break;
            }
        }
    }

    if (!loaded) {
        std::string tried;
        for (const auto& p : attempted) tried += "\n  - " + p;
        throw std::runtime_error(
            "prodmp_io::loadProDmpFeatureConfig: could not load the feature configuration YAML "
            "from any known location. Proceeding would silently fall back to hardcoded defaults, "
            "which is not a safe default - refusing to continue. Paths tried:" + tried);
    }

    ProDmpFeatureConfig cfg;
    if (root["num_basis"]) cfg.num_basis = root["num_basis"].as<int>();
    if (root["ridge_lambda"]) cfg.ridge_lambda = root["ridge_lambda"].as<double>();
    if (root["position_filter"]) {
        YAML::Node pf = root["position_filter"];
        if (pf["enabled"]) cfg.position_filter_enabled = pf["enabled"].as<bool>();
        if (pf["window_sec"]) cfg.position_filter_window_sec = pf["window_sec"].as<double>();
    }
    if (root["fix_goal_to_demo_endpoint"]) {
        cfg.fix_goal_to_demo_endpoint = root["fix_goal_to_demo_endpoint"].as<bool>();
    }
    return cfg;
}

void applyProDmpFeatureConfig(const std::string& filepath, ProDMP& prodmp) {
    const ProDmpFeatureConfig cfg = loadProDmpFeatureConfig(filepath);
    prodmp.setPositionFilterWindow(cfg.position_filter_enabled ? cfg.position_filter_window_sec
                                                                : 0.0);
    prodmp.setFixGoalToDemoEndpoint(cfg.fix_goal_to_demo_endpoint);
}

}  // namespace prodmp_io
}  // namespace core
}  // namespace haptic_dmp_learning
