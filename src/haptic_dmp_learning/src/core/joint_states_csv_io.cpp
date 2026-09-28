#include "haptic_dmp_learning/core/joint_states_csv_io.hpp"

#include <fstream>
#include <sstream>
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

std::vector<JointStateSample> readJointStatesCsv(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) throw std::runtime_error("readJointStatesCsv: cannot open " + path);

    auto split = [](const std::string& line) {
        std::vector<std::string> out;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) out.push_back(cell);
        return out;
    };

    std::string header_line;
    if (!std::getline(f, header_line)) throw std::runtime_error("readJointStatesCsv: empty file " + path);
    const std::vector<std::string> header = split(header_line);
    auto column = [&](const std::string& name) -> std::size_t {
        for (std::size_t i = 0; i < header.size(); ++i) {
            if (header[i] == name) return i;
        }
        throw std::runtime_error("readJointStatesCsv: missing column '" + name + "' in " + path);
    };
    const std::size_t t_col = column("t");
    std::array<std::size_t, kNumJoints> q_col{};
    for (std::size_t j = 0; j < kNumJoints; ++j) q_col[j] = column(jointNames()[j]);

    std::vector<JointStateSample> out;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        const std::vector<std::string> cells = split(line);
        JointStateSample s;
        try {
            s.t = std::stod(cells.at(t_col));
            for (std::size_t j = 0; j < kNumJoints; ++j) s.q[j] = std::stod(cells.at(q_col[j]));
        } catch (const std::exception&) {
            throw std::runtime_error("readJointStatesCsv: malformed row '" + line + "' in " + path);
        }
        out.push_back(s);
    }
    if (out.empty()) throw std::runtime_error("readJointStatesCsv: no data rows in " + path);
    return out;
}

}  // namespace joint_states_csv_io
}  // namespace core
}  // namespace haptic_dmp_learning
