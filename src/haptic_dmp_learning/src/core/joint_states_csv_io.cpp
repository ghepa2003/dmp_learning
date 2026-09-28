#include "haptic_dmp_learning/core/joint_states_csv_io.hpp"

#include <fstream>
#include <stdexcept>

namespace haptic_dmp_learning {
namespace core {
namespace joint_states_csv_io {

const std::array<std::string, kNumJoints>& jointNames() {
    static const std::array<std::string, kNumJoints> names = {
        "fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4",
        "fer_joint5", "fer_joint6", "fer_joint7"};
    return names;
}

bool extractJointPositions(const std::vector<std::string>& names,
                           const std::vector<double>& positions,
                           std::array<double, kNumJoints>& q_out) {
    const auto& wanted = jointNames();
    for (std::size_t j = 0; j < kNumJoints; ++j) {
        bool found = false;
        for (std::size_t i = 0; i < names.size() && i < positions.size(); ++i) {
            if (names[i] == wanted[j]) {
                q_out[j] = positions[i];
                found = true;
                break;
            }
        }
        if (!found) return false;
    }
    return true;
}

std::string deriveJointStatesCsvPath(const std::string& demo_csv_path) {
    if (demo_csv_path.empty()) return demo_csv_path;
    const std::string raw_suffix = "_raw.csv";
    const std::string csv_suffix = ".csv";
    auto ends_with = [&](const std::string& suffix) {
        return demo_csv_path.size() >= suffix.size() &&
               demo_csv_path.compare(demo_csv_path.size() - suffix.size(), suffix.size(),
                                     suffix) == 0;
    };
    if (ends_with(raw_suffix)) {
        return demo_csv_path.substr(0, demo_csv_path.size() - raw_suffix.size()) +
               "_joint_states.csv";
    }
    if (ends_with(csv_suffix)) {
        return demo_csv_path.substr(0, demo_csv_path.size() - csv_suffix.size()) +
               "_joint_states.csv";
    }
    return demo_csv_path + "_joint_states.csv";
}

void writeJointStatesCsv(const std::string& path, const std::vector<JointStateSample>& samples) {
    if (samples.empty()) {
        throw std::invalid_argument("writeJointStatesCsv: no samples to write to " + path);
    }
    std::ofstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("writeJointStatesCsv: cannot open file for writing: " + path);
    }
    f << "t";
    for (const auto& n : jointNames()) f << "," << n;
    f << "\n";
    for (const auto& s : samples) {
        f << s.t;
        for (double q : s.q) f << "," << q;
        f << "\n";
    }
}

}  // namespace joint_states_csv_io
}  // namespace core
}  // namespace haptic_dmp_learning
