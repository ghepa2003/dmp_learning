#include "metrics.hpp"
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>

namespace dmp_tools {
namespace metrics {

TrajectoryFidelity computeTrajectoryFidelity(const std::vector<Eigen::Vector3d>& reference,
                                              const std::vector<Eigen::Vector3d>& replay) {
    TrajectoryFidelity m{};
    size_t N = std::min(reference.size(), replay.size());
    if (N == 0) return m;
    Eigen::Vector3d sq_sum = Eigen::Vector3d::Zero();
    double max_err = 0.0;
    for (size_t k = 0; k < N; ++k) {
        Eigen::Vector3d diff = replay[k] - reference[k];
        sq_sum += diff.cwiseProduct(diff);
        max_err = std::max(max_err, diff.norm());
    }
    Eigen::Vector3d rmse = (sq_sum / static_cast<double>(N)).cwiseSqrt();
    m.rmse_x = rmse.x(); m.rmse_y = rmse.y(); m.rmse_z = rmse.z();
    m.rmse_overall = rmse.norm();
    m.max_error = max_err;
    return m;
}

OrientationFidelity computeOrientationFidelity(const std::vector<Eigen::Quaterniond>& reference,
                                                const std::vector<Eigen::Quaterniond>& replay) {
    OrientationFidelity m{};
    size_t N = std::min(reference.size(), replay.size());
    if (N == 0) return m;
    double sum_err = 0.0, max_err = 0.0;
    for (size_t k = 0; k < N; ++k) {
        double dot = std::abs(reference[k].normalized().dot(replay[k].normalized()));
        dot = std::min(1.0, std::max(-1.0, dot));
        double angle_deg = 2.0 * std::acos(dot) * 180.0 / M_PI;
        sum_err += angle_deg;
        max_err = std::max(max_err, angle_deg);
    }
    m.mean_angular_error_deg = sum_err / static_cast<double>(N);
    m.max_angular_error_deg = max_err;
    return m;
}

double computeEndpointError(const Eigen::Vector3d& final_point, const Eigen::Vector3d& expected_goal) {
    return (final_point - expected_goal).norm();
}

double computeAngularEndpointError(const Eigen::Quaterniond& final_q, const Eigen::Quaterniond& expected_goal) {
    double dot = std::abs(final_q.normalized().dot(expected_goal.normalized()));
    dot = std::min(1.0, std::max(-1.0, dot));
    return 2.0 * std::acos(dot) * 180.0 / M_PI;
}

void printReport(const std::string& label, const TrajectoryFidelity& tf) {
    std::cout << "  [Posizione] RMSE x/y/z: " << tf.rmse_x << " / " << tf.rmse_y << " / " << tf.rmse_z
              << " m | RMSE totale: " << tf.rmse_overall << " m | Errore max: " << tf.max_error << " m\n";
}

void printReport(const std::string& label, const OrientationFidelity& of) {
    std::cout << "  [Orientamento] Errore angolare medio: " << of.mean_angular_error_deg
              << " deg | massimo: " << of.max_angular_error_deg << " deg\n";
}

void printEndpointError(const std::string& label, double position_error_m, double orientation_error_deg) {
    std::cout << "  [" << label << "] Errore finale - posizione: " << position_error_m
              << " m | orientamento: " << orientation_error_deg << " deg\n";
}

void appendToSummaryCsv(const std::string& csv_path, const std::string& trial_label,
                         const TrajectoryFidelity& tf, const OrientationFidelity& of,
                         double endpoint_pos_error, double endpoint_orient_error_deg,
                         const std::array<bool, 3>& scale_reliable) {
    std::ifstream check(csv_path);
    bool exists = check.good();
    check.close();

    std::ofstream f(csv_path, std::ios::app);
    if (!exists) {
        f << "trial,rmse_x,rmse_y,rmse_z,rmse_overall,max_pos_error,"
            "mean_angular_error_deg,max_angular_error_deg,"
            "endpoint_pos_error,endpoint_orient_error_deg,"
            "scale_reliable_x,scale_reliable_y,scale_reliable_z\n";
    }
    f << trial_label << "," << tf.rmse_x << "," << tf.rmse_y << "," << tf.rmse_z << ","
      << tf.rmse_overall << "," << tf.max_error << ","
      << of.mean_angular_error_deg << "," << of.max_angular_error_deg << ","
      << endpoint_pos_error << "," << endpoint_orient_error_deg << ","
      << scale_reliable[0] << "," << scale_reliable[1] << "," << scale_reliable[2] << "\n";
}

}  // namespace metrics
}  // namespace dmp_tools
