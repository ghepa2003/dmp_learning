#include "haptic_dmp_learning/core/demo_csv_io.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace haptic_dmp_learning {
namespace core {
namespace demo_csv_io {

namespace {

// Trailing '\r' (CRLF files) and spaces, as trimmed by the previous inline
// parser in dmp_gazebo_executor_node.
void rtrim(std::string& s) {
    while (!s.empty() && (s.back() == '\r' || s.back() == ' ')) {
        s.pop_back();
    }
}

}  // namespace

void writeDemoCsv(const std::string& path, const std::vector<Sample>& samples) {
    std::ofstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error(
            "demo_csv_io::writeDemoCsv: cannot open file for writing: " + path);
    }
    f << "t,x,y,z,qw,qx,qy,qz,gripper_trigger\n";
    for (const auto& s : samples) {
        f << s.t << ","
          << s.position.x() << "," << s.position.y() << "," << s.position.z() << ","
          << s.orientation.w() << "," << s.orientation.x() << ","
          << s.orientation.y() << "," << s.orientation.z() << ","
          << (s.gripper_trigger ? 1 : 0) << "\n";
    }
}

std::vector<Sample> readDemoCsv(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error(
            "demo_csv_io::readDemoCsv: cannot open demo CSV: " + path);
    }

    std::string line;
    std::getline(f, line);  // discard header: t,x,y,z,qw,qx,qy,qz[,gripper_trigger]

    std::vector<Sample> samples;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string field;
        std::vector<double> values;
        while (std::getline(ss, field, ',')) {
            values.push_back(std::stod(field));
        }
        if (values.size() < 8) {
            throw std::runtime_error(
                "demo_csv_io::readDemoCsv: malformed row (< 8 fields) in " + path);
        }

        Sample s;
        s.t = values[0];
        s.position = Eigen::Vector3d(values[1], values[2], values[3]);
        s.orientation =
            Eigen::Quaterniond(values[4], values[5], values[6], values[7]).normalized();
        s.gripper_trigger = (values.size() >= 9) && (values[8] != 0.0);
        samples.push_back(s);
    }

    if (samples.empty()) {
        throw std::runtime_error(
            "demo_csv_io::readDemoCsv: no data rows in " + path);
    }
    return samples;
}

double readGripperTriggerTime(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        return -1.0;
    }

    std::string line;
    if (!std::getline(f, line)) {
        return -1.0;
    }

    // Locate the `gripper_trigger` column in the header.
    int trigger_col_idx = -1;
    {
        std::stringstream ss(line);
        std::string col;
        int curr_idx = 0;
        while (std::getline(ss, col, ',')) {
            rtrim(col);
            if (col == "gripper_trigger") {
                trigger_col_idx = curr_idx;
                break;
            }
            ++curr_idx;
        }
    }
    if (trigger_col_idx < 0) {
        return -1.0;
    }

    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::stringstream lss(line);
        std::string field;
        std::vector<std::string> fields;
        while (std::getline(lss, field, ',')) {
            rtrim(field);
            fields.push_back(field);
        }
        if (static_cast<int>(fields.size()) > trigger_col_idx) {
            try {
                if (std::stoi(fields[trigger_col_idx]) == 1) {
                    return std::stod(fields[0]);
                }
            } catch (...) {
            }
        }
    }
    return -1.0;
}

}  // namespace demo_csv_io
}  // namespace core
}  // namespace haptic_dmp_learning
