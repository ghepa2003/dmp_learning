#pragma once
// Metriche di valutazione, tenute LOCALI a tools/dmp_offline_test - non fanno
// parte del pacchetto ROS2 (il wrapper node a runtime non ne ha bisogno,
// sono uno strumento di validazione/test, non di produzione).
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

// Ha senso SOLO quando esiste una vera traiettoria di riferimento (replay a
// parita' di goal) - non per un replay su goal diverso.
TrajectoryFidelity computeTrajectoryFidelity(const std::vector<Eigen::Vector3d>& reference,
                                              const std::vector<Eigen::Vector3d>& replay);

OrientationFidelity computeOrientationFidelity(const std::vector<Eigen::Quaterniond>& reference,
                                                const std::vector<Eigen::Quaterniond>& replay);

// Uniche metriche sensate quando replay e riferimento hanno goal diversi.
double computeEndpointError(const Eigen::Vector3d& final_point, const Eigen::Vector3d& expected_goal);
double computeAngularEndpointError(const Eigen::Quaterniond& final_q, const Eigen::Quaterniond& expected_goal);

void printReport(const std::string& label, const TrajectoryFidelity& tf);
void printReport(const std::string& label, const OrientationFidelity& of);
void printEndpointError(const std::string& label, double position_error_m, double orientation_error_deg);

// Aggiunge una riga a un CSV riassuntivo (crea l'header se il file non esiste
// ancora) - accumula i risultati di piu' traiettorie/trial nel tempo.
void appendToSummaryCsv(const std::string& csv_path, const std::string& trial_label,
                         const TrajectoryFidelity& tf, const OrientationFidelity& of,
                         double endpoint_pos_error, double endpoint_orient_error_deg,
                         const std::array<bool, 3>& scale_reliable = {true, true, true});

}  // namespace metrics
}  // namespace dmp_tools
