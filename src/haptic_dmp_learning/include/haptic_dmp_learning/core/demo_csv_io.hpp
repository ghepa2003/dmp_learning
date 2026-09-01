#pragma once

#include <string>
#include <vector>

#include "haptic_dmp_learning/core/types.hpp"

namespace haptic_dmp_learning {
namespace core {
namespace demo_csv_io {

/**
 * @brief Read/write of the raw demonstration CSV used across the package.
 *
 * Canonical format (header line, then one row per sample):
 *   t,x,y,z,qw,qx,qy,qz,gripper_trigger
 *
 * `gripper_trigger` is 0/1. Rows with only the first 8 columns (no
 * `gripper_trigger`) are still accepted on read for backward compatibility;
 * their gripper_trigger is taken as false.
 *
 * Pure I/O: no ROS, no yaml-cpp. Errors are reported with std::runtime_error
 * (except readGripperTriggerTime(), which is best-effort - see below).
 */

/// Write @p samples to @p path in the canonical 9-column format (always writes
/// the `gripper_trigger` column). Numeric formatting is the default
/// std::ofstream `operator<<` precision, matching the previous per-node
/// implementations.
/// @throws std::runtime_error if @p path cannot be opened for writing.
void writeDemoCsv(const std::string& path, const std::vector<Sample>& samples);

/// Read a demonstration CSV written by writeDemoCsv() (or an 8-column legacy
/// file). Quaternions are normalised. A row's gripper_trigger is set from the
/// 9th column when present, false otherwise.
/// @throws std::runtime_error if @p path cannot be opened, a data row has
///         fewer than 8 fields, or the file contains no data rows.
std::vector<Sample> readDemoCsv(const std::string& path);

/// Scan @p path for the first data row whose `gripper_trigger` column equals 1
/// and return that row's `t`. Best-effort: returns -1.0 if the file cannot be
/// opened, has no header, has no `gripper_trigger` column, or no row triggers.
/// Never throws.
double readGripperTriggerTime(const std::string& path);

}  // namespace demo_csv_io
}  // namespace core
}  // namespace haptic_dmp_learning
