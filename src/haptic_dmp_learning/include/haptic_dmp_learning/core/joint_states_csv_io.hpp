#pragma once

#include <array>
#include <string>
#include <vector>

namespace haptic_dmp_learning {
namespace core {
namespace joint_states_csv_io {

/**
 * @brief Buffering helpers and CSV writer for /joint_states recorded alongside a demo.
 *
 * CSV format (same as tools/gazebo_cartesian_eval/scripts/manipulability_check/
 * extract_joint_states_to_csv.py, so compute_w_trans reads it unchanged):
 *   t,fer_joint1,fer_joint2,fer_joint3,fer_joint4,fer_joint5,fer_joint6,fer_joint7
 *
 * Pure I/O: no ROS. Joints are mapped BY NAME, never by array position.
 */

constexpr std::size_t kNumJoints = 7;

/// "fer_joint1" .. "fer_joint7", in CSV column order.
const std::array<std::string, kNumJoints>& jointNames();

struct JointStateSample {
    double t = 0.0;  ///< seconds since the start of the recording
    std::array<double, kNumJoints> q{};
};

/// Fills @p q_out with the positions of fer_joint1..7 looked up by name in @p names.
/// @return false (and leaves @p q_out unspecified) if any of the 7 joints is absent from
///         @p names or has no matching entry in @p positions.
bool extractJointPositions(const std::vector<std::string>& names,
                           const std::vector<double>& positions,
                           std::array<double, kNumJoints>& q_out);

/// Derives the joint-states CSV path from the raw-demo CSV path: a trailing "_raw.csv"
/// becomes "_joint_states.csv" (live_demo_raw.csv -> live_demo_joint_states.csv), any other
/// "<stem>.csv" becomes "<stem>_joint_states.csv". An empty path stays empty.
std::string deriveJointStatesCsvPath(const std::string& demo_csv_path);

/// @throws std::invalid_argument if @p samples is empty (no misleading empty file),
///         std::runtime_error if @p path cannot be opened for writing.
void writeJointStatesCsv(const std::string& path, const std::vector<JointStateSample>& samples);

/// Reads a CSV written by writeJointStatesCsv() (or the python extractor). Columns are mapped BY NAME
/// from the header ("t" and fer_joint1..7, any order, extra columns ignored).
/// @throws std::runtime_error if the file cannot be opened, a required column is missing, a row is
///         malformed, or there are no data rows.
std::vector<JointStateSample> readJointStatesCsv(const std::string& path);

}  // namespace joint_states_csv_io
}  // namespace core
}  // namespace haptic_dmp_learning
