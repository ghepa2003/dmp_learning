#pragma once

// Shared, header-only helpers of the tests that read the recorded Gazebo goal data
// (tools/gazebo_cartesian_eval/data/*.csv), resolved via $HOME/thesis_ws (falls back to /root).
// Used by test_manipulability and test_joint_path_simulator.

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "franka_cartesian_control/core/robot_model.hpp"

namespace satellite_grasp_planner {
namespace test_goal_data {

inline std::string workspaceRoot() {
    const char* home = std::getenv("HOME");
    return std::string(home ? home : "/root") + "/thesis_ws";
}

inline std::string dataDir() {
    return workspaceRoot() + "/tools/gazebo_cartesian_eval/data";
}

/// @brief Minimal CSV reader: header row -> column names, each following row
/// -> a name->value map. Good enough for the small, well-formed CSVs written
/// by compute_manipulability_goals.py / the Gazebo eval pipeline.
inline std::vector<std::unordered_map<std::string, double>> readCsv(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        ADD_FAILURE() << "Could not open CSV (required test fixture, see file-level NOTE): " << path;
        return {};
    }
    std::vector<std::unordered_map<std::string, double>> rows;
    std::string header_line;
    std::getline(f, header_line);
    if (!header_line.empty() && header_line.back() == '\r') header_line.pop_back();
    std::vector<std::string> cols;
    {
        std::stringstream ss(header_line);
        std::string col;
        while (std::getline(ss, col, ',')) {
            if (!col.empty() && col.back() == '\r') col.pop_back();
            cols.push_back(col);
        }
    }
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string cell;
        std::unordered_map<std::string, double> row;
        size_t i = 0;
        while (std::getline(ss, cell, ',') && i < cols.size()) {
            row[cols[i]] = std::stod(cell);
            ++i;
        }
        rows.push_back(std::move(row));
    }
    return rows;
}

inline franka_cartesian_control::core::RobotModel::JointVector jointVectorFromRow(const std::unordered_map<std::string, double>& row) {
    franka_cartesian_control::core::RobotModel::JointVector q;
    for (int i = 0; i < 7; ++i) {
        q(i) = row.at("fer_joint" + std::to_string(i + 1));
    }
    return q;
}

}  // namespace test_goal_data
}  // namespace satellite_grasp_planner
