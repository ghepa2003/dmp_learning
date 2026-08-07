#pragma once
// Evaluation metrics, kept LOCAL to tools/dmp_offline_test - not part of
// the ROS2 package (the wrapper node at runtime does not need them,
// they are a validation/testing tool, not for production).
#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace dmp_tools {
namespace metrics {

struct TrajectoryFidelity {
    double rmse_x = 0.0, rmse_y = 0.0, rmse_z = 0.0, rmse_overall = 0.0;
    double max_error = 0.0;
};

struct OrientationFidelity {
    double mean_angular_error_deg = 0.0;
    double max_angular_error_deg = 0.0;
};

// Only meaningful when a true reference trajectory exists (replay with same goal)
// - not for replay with a shifted goal.
TrajectoryFidelity computeTrajectoryFidelity(const std::vector<Eigen::Vector3d>& reference,
                                              const std::vector<Eigen::Vector3d>& replay);

OrientationFidelity computeOrientationFidelity(const std::vector<Eigen::Quaterniond>& reference,
                                                const std::vector<Eigen::Quaterniond>& replay);

// Metrics when replay and reference have different goals.
double computeEndpointError(const Eigen::Vector3d& final_point, const Eigen::Vector3d& expected_goal);
double computeAngularEndpointError(const Eigen::Quaterniond& final_q, const Eigen::Quaterniond& expected_goal);

void printReport(const std::string& label, const TrajectoryFidelity& tf);
void printReport(const std::string& label, const OrientationFidelity& of);
void printEndpointError(const std::string& label, double position_error_mm, double orientation_error_deg);

// Appends a row to summary CSV (creates header if file does not exist yet)
// - accumulates results of multiple trajectories/trials over time.
void appendToSummaryCsv(const std::string& csv_path, const std::string& trial_label,
                         const TrajectoryFidelity& tf, const OrientationFidelity& of,
                         double endpoint_pos_error, double endpoint_orient_error_deg,
                         const std::array<bool, 3>& scale_reliable = {true, true, true});

}  // namespace metrics
}  // namespace dmp_tools

