#include "satellite_grasp_planner/core/candidate_scan.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace satellite_grasp_planner {
namespace core {

using haptic_dmp_learning::core::CubeSatelliteModel;
using haptic_dmp_learning::core::GraspPointId;
using haptic_dmp_learning::core::ProDMP;

namespace {
constexpr double kDt = 0.005;
constexpr GraspPointId kAllPoints[4] = {GraspPointId::kP0, GraspPointId::kP90,
                                        GraspPointId::kP180, GraspPointId::kP270};
}  // namespace

double windowCenterTheta(const CubeSatelliteModel& model, const Eigen::Vector3d& robot_base_world) {
    const Eigen::Vector3d u = robot_base_world - model.cubeCenterWorld();
    const Eigen::Vector3d a = model.axisWorld();
    const Eigen::Vector3d u_perp = u - u.dot(a) * a;
    if (!(u_perp.norm() >= 1e-9)) {
        throw std::invalid_argument(
            "windowCenterTheta: robot base lies on the spin axis (|u_perp| < 1e-9).");
    }
    const Eigen::Vector3d n = model.faceNormalBody();
    return std::atan2(a.dot(n.cross(u_perp)), n.dot(u_perp));
}

std::vector<CandidateRow> scanCandidates(const CubeSatelliteModel& model,
                                         const ProDMP& prodmp_template, const ScanParams& params) {
    if (!(params.step_rad > 0.0)) throw std::invalid_argument("scanCandidates: step_rad must be > 0.");
    if (!(params.half_window_rad > 0.0)) {
        throw std::invalid_argument("scanCandidates: half_window_rad must be > 0.");
    }
    if (!(params.e_v_ref > 0.0) || !(params.e_g_ref > 0.0)) {
        throw std::invalid_argument("scanCandidates: e_v_ref and e_g_ref must be > 0.");
    }
    params.weights.validate();
    const double tau_contact =
        params.tau_contact_s > 0.0 ? params.tau_contact_s : prodmp_template.tau();
    if (tau_contact > prodmp_template.tau()) {
        throw std::invalid_argument("scanCandidates: tau_contact_s exceeds the template tau.");
    }

    // Work on a copy; bring it to the contact instant with a fixed support goal.
    ProDMP prodmp = prodmp_template;
    prodmp.setRelativeGoal(false);
    prodmp.setInitialConditions(0.0, params.p0, Eigen::Vector3d::Zero());
    prodmp.setGoal(params.p0 + params.delta_g_demo);
    const int n_steps = std::max(1, static_cast<int>(std::ceil(tau_contact / kDt - 1e-9)));
    for (int i = 0; i < n_steps; ++i) prodmp.step(kDt);

    // Both window ends included; the small epsilon absorbs round-off in 2*half/step.
    const int n_theta =
        static_cast<int>(std::floor(2.0 * params.half_window_rad / params.step_rad + 1e-9)) + 1;
    std::vector<CandidateRow> rows;
    rows.reserve(static_cast<std::size_t>(n_theta) * 4);
    for (int i = 0; i < n_theta; ++i) {
        const double theta = params.theta_center_rad - params.half_window_rad + i * params.step_rad;
        for (GraspPointId k : kAllPoints) {
            CandidateRow r;
            r.k = k;
            r.theta_rad = theta;
            r.contact_point = model.graspPoseAt(k, theta).position_world;
            r.goal_param = r.contact_point + params.contact_to_end_offset;
            r.v_ee = prodmp.velocityForCandidateGoal(r.goal_param);
            r.v_target = haptic_dmp_learning::core::surfaceVelocity(model, k, theta, params.omega_rad_s);
            r.e_v = haptic_dmp_learning::core::velocityTerm(r.v_ee, r.v_target);
            r.e_g = haptic_dmp_learning::core::goalTerm(r.goal_param - params.p0, params.delta_g_demo);
            r.e_v_hat = r.e_v / params.e_v_ref;
            r.e_g_hat = r.e_g / params.e_g_ref;
            r.partial_cost = params.weights.w_v * r.e_v_hat + params.weights.w_g * r.e_g_hat;
            rows.push_back(r);
        }
    }
    return rows;
}

std::vector<CandidateRow> sortedByPartialCost(std::vector<CandidateRow> rows) {
    std::stable_sort(rows.begin(), rows.end(), [](const CandidateRow& a, const CandidateRow& b) {
        return a.partial_cost < b.partial_cost;
    });
    return rows;
}

std::vector<CandidateRow> bestPerPoint(const std::vector<CandidateRow>& rows, int n_per_point) {
    if (n_per_point <= 0) throw std::invalid_argument("bestPerPoint: n_per_point must be > 0.");
    const std::vector<CandidateRow> sorted = sortedByPartialCost(rows);
    std::vector<CandidateRow> out;
    for (GraspPointId k : kAllPoints) {
        int taken = 0;
        for (const CandidateRow& r : sorted) {
            if (r.k == k && taken < n_per_point) {
                out.push_back(r);
                ++taken;
            }
        }
    }
    return out;
}

double launchDelay(double theta_now_rad, double theta_star_rad, double omega_rad_s,
                   double tau_contact_s, double min_delay_s) {
    if (omega_rad_s == 0.0) throw std::invalid_argument("launchDelay: omega must be non-zero.");
    const double T = 2.0 * haptic_dmp_learning::core::kPi / std::abs(omega_rad_s);
    const double t = (theta_star_rad - theta_now_rad) / omega_rad_s - tau_contact_s;
    double r = std::fmod(t - min_delay_s, T);
    if (r < 0.0) r += T;
    // Round-off can leave r a hair below T (e.g. t - min_delay = -1e-15): that is the same instant
    // as 0, and the result must stay in [min_delay, min_delay + T).
    if (r >= T - 1e-9) r = 0.0;
    return min_delay_s + r;
}

}  // namespace core
}  // namespace satellite_grasp_planner
