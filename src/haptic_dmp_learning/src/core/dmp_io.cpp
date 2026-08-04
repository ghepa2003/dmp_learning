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

YAML::Node dmpToNode(const DMP& dmp) {
    YAML::Node node;
    node["n_basis"] = dmp.nBasis();
    node["alpha_x"] = dmp.alphaX();
    node["alpha_z"] = dmp.alphaZ();
    node["beta_z"] = dmp.betaZ();
    node["second_order_canonical_system"] = dmp.secondOrderCanonical();
    node["tau"] = dmp.tau();
    node["y0"] = vec3ToYaml(dmp.y0());
    node["goal"] = vec3ToYaml(dmp.goal());
    node["z0"] = vec3ToYaml(dmp.z0());
    node["dG"] = vec3ToYaml(dmp.dG());
    node["A"] = vec3ToYaml(dmp.A());
    node["centers"] = vectorToYaml(dmp.centers());
    node["widths"] = vectorToYaml(dmp.widths());

    YAML::Node weights(YAML::NodeType::Sequence);
    static const char* dim_names[3] = {"x", "y", "z"};
    for (int d = 0; d < 3; ++d) {
        YAML::Node wd;
        wd["dim"] = dim_names[d];
        wd["values"] = vectorToYaml(dmp.weights()[d]);
        weights.push_back(wd);
    }
    node["weights"] = weights;
    return node;
}

YAML::Node quatToYaml(const Eigen::Quaterniond& q) {
    YAML::Node node(YAML::NodeType::Sequence);
    node.push_back(q.w()); node.push_back(q.x()); node.push_back(q.y()); node.push_back(q.z());
    return node;
}

Eigen::Quaterniond yamlToQuat(const YAML::Node& node) {
    return Eigen::Quaterniond(node[0].as<double>(), node[1].as<double>(), node[2].as<double>(), node[3].as<double>());
}

YAML::Node quaternionDmpToNode(const QuaternionDMP& qdmp) {
    YAML::Node node;
    node["n_basis"] = qdmp.nBasis();
    node["alpha_x"] = qdmp.alphaX();
    node["alpha_z"] = qdmp.alphaZ();
    node["beta_z"] = qdmp.betaZ();
    node["tau"] = qdmp.tau();
    node["q0"] = quatToYaml(qdmp.q0());
    node["goal"] = quatToYaml(qdmp.goal());
    node["eta0"] = vec3ToYaml(qdmp.eta0());
    node["centers"] = vectorToYaml(qdmp.centers());
    node["widths"] = vectorToYaml(qdmp.widths());

    YAML::Node weights(YAML::NodeType::Sequence);
    static const char* dim_names[3] = {"x", "y", "z"};
    for (int d = 0; d < 3; ++d) {
        YAML::Node wd;
        wd["dim"] = dim_names[d];
        wd["values"] = vectorToYaml(qdmp.weights()[d]);
        weights.push_back(wd);
    }
    node["weights"] = weights;
    return node;
}

}  // namespace

void saveToYaml(const DMP& dmp, const std::string& filepath) {
    YAML::Node root = dmpToNode(dmp);

    std::ofstream fout(filepath);
    if (!fout.is_open()) {
        throw std::runtime_error("dmp_io::saveToYaml: cannot open file for writing: " + filepath);
    }
    fout << root;
}

DMP loadFromYaml(const std::string& filepath) {
    YAML::Node root = YAML::LoadFile(filepath);

    int n_basis = root["n_basis"].as<int>();
    double alpha_x = root["alpha_x"].as<double>();
    double alpha_z = root["alpha_z"].as<double>();
    double beta_z = root["beta_z"].as<double>();
    bool second_order = false;
    if (root["second_order_canonical_system"]) {
        second_order = root["second_order_canonical_system"].as<bool>();
    } else if (root["second_order_canonical"]) {
        second_order = root["second_order_canonical"].as<bool>();
    }
    double tau = root["tau"].as<double>();

    Eigen::Vector3d y0 = yamlToVec3(root["y0"]);
    Eigen::Vector3d goal = yamlToVec3(root["goal"]);
    Eigen::Vector3d dG = yamlToVec3(root["dG"]);
    Eigen::Vector3d A = yamlToVec3(root["A"]);
    Eigen::Vector3d z0 = root["z0"] ? yamlToVec3(root["z0"]) : Eigen::Vector3d::Zero();
    Eigen::VectorXd centers = yamlToVector(root["centers"]);
    Eigen::VectorXd widths = yamlToVector(root["widths"]);

    std::array<Eigen::VectorXd, 3> weights;
    for (const auto& wd : root["weights"]) {
        std::string dim = wd["dim"].as<std::string>();
        int idx = (dim == "x") ? 0 : (dim == "y") ? 1 : 2;
        weights[idx] = yamlToVector(wd["values"]);
    }

    DMP dmp(n_basis, alpha_x, alpha_z, beta_z, second_order);
    dmp.setLearnedParameters(tau, y0, goal, dG, A, centers, widths, weights, z0);
    return dmp;
}

void saveToYaml(const DMP& dmp, const QuaternionDMP& qdmp, const std::string& filepath) {
    YAML::Node root;
    root["position_dmp"] = dmpToNode(dmp);
    root["quaternion_dmp"] = quaternionDmpToNode(qdmp);

    std::ofstream fout(filepath);
    if (!fout.is_open()) {
        throw std::runtime_error("dmp_io::saveToYaml: cannot open file for writing: " + filepath);
    }
    fout << root;
}

void loadFromYaml(const std::string& filepath, DMP& dmp, QuaternionDMP& qdmp) {
    YAML::Node root = YAML::LoadFile(filepath);

    // --- position ---
    YAML::Node p = root["position_dmp"];
    Eigen::Vector3d y0 = yamlToVec3(p["y0"]), goal = yamlToVec3(p["goal"]);
    Eigen::Vector3d dG = yamlToVec3(p["dG"]), A = yamlToVec3(p["A"]);
    Eigen::VectorXd centers = yamlToVector(p["centers"]), widths = yamlToVector(p["widths"]);
    std::array<Eigen::VectorXd, 3> weights;
    for (const auto& wd : p["weights"]) {
        std::string dim = wd["dim"].as<std::string>();
        int idx = (dim == "x") ? 0 : (dim == "y") ? 1 : 2;
        weights[idx] = yamlToVector(wd["values"]);
    }
    Eigen::Vector3d z0 = p["z0"] ? yamlToVec3(p["z0"]) : Eigen::Vector3d::Zero();
    bool second_order = false;
    if (p["second_order_canonical_system"]) {
        second_order = p["second_order_canonical_system"].as<bool>();
    } else if (p["second_order_canonical"]) {
        second_order = p["second_order_canonical"].as<bool>();
    }
    dmp = DMP(p["n_basis"].as<int>(), p["alpha_x"].as<double>(), p["alpha_z"].as<double>(), p["beta_z"].as<double>(), second_order);
    dmp.setLearnedParameters(p["tau"].as<double>(), y0, goal, dG, A, centers, widths, weights, z0);

    // --- orientation ---
    YAML::Node q = root["quaternion_dmp"];
    Eigen::Quaterniond q0 = yamlToQuat(q["q0"]), qgoal = yamlToQuat(q["goal"]);
    Eigen::Vector3d qeta0 = yamlToVec3(q["eta0"]);
    Eigen::VectorXd qcenters = yamlToVector(q["centers"]), qwidths = yamlToVector(q["widths"]);
    std::array<Eigen::VectorXd, 3> qweights;
    for (const auto& wd : q["weights"]) {
        std::string dim = wd["dim"].as<std::string>();
        int idx = (dim == "x") ? 0 : (dim == "y") ? 1 : 2;
        qweights[idx] = yamlToVector(wd["values"]);
    }
    qdmp = QuaternionDMP(q["n_basis"].as<int>(), q["alpha_x"].as<double>(), q["alpha_z"].as<double>(), q["beta_z"].as<double>());
    qdmp.setLearnedParameters(q["tau"].as<double>(), q0, qgoal, qcenters, qwidths, qweights, qeta0);
}

void applyFeatureConfig(const std::string& filepath, DMP& dmp, QuaternionDMP& qdmp) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(filepath);
    } catch (const YAML::BadFile&) {
        // Missing file: no modification, objects remain at current defaults
        // - feature activation is opt-in.
        return;
    }

    if (root["regression"]) {
        YAML::Node reg = root["regression"];
        bool use_ridge = false;
        double lambda = 1e-6;
        if (reg["method"]) {
            use_ridge = (reg["method"].as<std::string>() == "ridge");
        }
        if (reg["ridge_lambda"]) {
            lambda = reg["ridge_lambda"].as<double>();
        }
        dmp.setRidgeRegression(use_ridge, lambda);
        qdmp.setRidgeRegression(use_ridge, lambda);
    }

    if (root["second_order_canonical_system"]) {
        bool second_order = root["second_order_canonical_system"].as<bool>();
        dmp.setSecondOrderCanonical(second_order);
    } else if (root["second_order_canonical"]) {
        bool second_order = root["second_order_canonical"].as<bool>();
        dmp.setSecondOrderCanonical(second_order);
    }

    if (root["velocity_filter"]) {
        YAML::Node vf = root["velocity_filter"];
        bool use_filter = false;
        double w1 = 0.05, w2 = 0.05;
        if (vf["enabled"]) use_filter = vf["enabled"].as<bool>();
        if (vf["window_sec_1"]) w1 = vf["window_sec_1"].as<double>();
        if (vf["window_sec_2"]) w2 = vf["window_sec_2"].as<double>();
        dmp.setVelocityFilter(use_filter, w1, w2);
        qdmp.setVelocityFilter(use_filter, w1, w2);
    }
}

}  // namespace dmp_io
}  // namespace core
}  // namespace haptic_dmp_learning
