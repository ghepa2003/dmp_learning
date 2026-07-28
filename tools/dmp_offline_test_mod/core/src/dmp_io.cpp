#include "haptic_dmp_learning/core/dmp_io.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <stdexcept>

namespace haptic_dmp_learning {
namespace core {
namespace dmp_io {

namespace {

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

}  // namespace

DMPConfig loadConfigFromYaml(const std::string& filepath) {
    YAML::Node root = YAML::LoadFile(filepath);
    DMPConfig cfg;
    if (root["n_basis"]) cfg.n_basis = root["n_basis"].as<int>();
    if (root["alpha_x"]) cfg.alpha_x = root["alpha_x"].as<double>();
    if (root["alpha_z"]) cfg.alpha_z = root["alpha_z"].as<double>();
    if (root["beta_z"]) cfg.beta_z = root["beta_z"].as<double>();
    return cfg;
}

DMP createFromConfigYaml(const std::string& filepath) {
    DMPConfig cfg = loadConfigFromYaml(filepath);
    return DMP(cfg.n_basis, cfg.alpha_x, cfg.alpha_z, cfg.beta_z);
}

void saveToYaml(const DMP& dmp, const std::string& filepath) {
    YAML::Node root;
    root["n_basis"] = dmp.nBasis();
    root["alpha_x"] = dmp.alphaX();
    root["alpha_z"] = dmp.alphaZ();
    root["beta_z"] = dmp.betaZ();
    root["tau"] = dmp.tau();
    root["y0"] = vec3ToYaml(dmp.y0());
    root["goal"] = vec3ToYaml(dmp.goal());
    root["centers"] = vectorToYaml(dmp.centers());
    root["widths"] = vectorToYaml(dmp.widths());

    YAML::Node weights(YAML::NodeType::Sequence);
    static const char* dim_names[3] = {"x", "y", "z"};
    for (int d = 0; d < 3; ++d) {
        YAML::Node wd;
        wd["dim"] = dim_names[d];
        wd["values"] = vectorToYaml(dmp.weights()[d]);
        weights.push_back(wd);
    }
    root["weights"] = weights;

    std::ofstream fout(filepath);
    if (!fout.is_open()) {
        throw std::runtime_error("dmp_io::saveToYaml: cannot open file for writing: " + filepath);
    }
    fout << root;
}

DMP loadFromYaml(const std::string& filepath) {
    YAML::Node root = YAML::LoadFile(filepath);

    int n_basis = root["n_basis"] ? root["n_basis"].as<int>() : 20;
    double alpha_x = root["alpha_x"] ? root["alpha_x"].as<double>() : 4.6;
    double alpha_z = root["alpha_z"] ? root["alpha_z"].as<double>() : 25.0;
    double beta_z = root["beta_z"] ? root["beta_z"].as<double>() : 6.25;

    if (!root["tau"] || !root["y0"] || !root["goal"] || !root["centers"] || !root["widths"] || !root["weights"]) {
        return DMP(n_basis, alpha_x, alpha_z, beta_z);
    }

    double tau = root["tau"].as<double>();
    Eigen::Vector3d y0 = yamlToVec3(root["y0"]);
    Eigen::Vector3d goal = yamlToVec3(root["goal"]);
    Eigen::VectorXd centers = yamlToVector(root["centers"]);
    Eigen::VectorXd widths = yamlToVector(root["widths"]);

    std::array<Eigen::VectorXd, 3> weights;
    for (const auto& wd : root["weights"]) {
        std::string dim = wd["dim"].as<std::string>();
        int idx = (dim == "x") ? 0 : (dim == "y") ? 1 : 2;
        weights[idx] = yamlToVector(wd["values"]);
    }

    DMP dmp(n_basis, alpha_x, alpha_z, beta_z);
    dmp.setLearnedParameters(tau, y0, goal, centers, widths, weights);
    return dmp;
}

}  // namespace dmp_io
}  // namespace core
}  // namespace haptic_dmp_learning
