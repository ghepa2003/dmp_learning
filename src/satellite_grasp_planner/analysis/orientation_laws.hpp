// Shared definitions of the offline orientation-law tools (orientation_law_compare, orientation_path_check):
// the three laws, the contact-phase variants and the QDMP frame handling. Header-only, analysis/ only; nothing
// here is used by the planner or the executor. Definitions are documented in orientation_law_compare.cpp.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "haptic_dmp_learning/core/grasp_orientation.hpp"
#include "haptic_dmp_learning/core/math_utils.hpp"
#include "haptic_dmp_learning/core/quaternion_dmp.hpp"

namespace orientation_laws {

namespace hdl = haptic_dmp_learning::core;
namespace go = haptic_dmp_learning::core::grasp_orientation;

constexpr double kPi = 3.14159265358979323846;
constexpr double kRad2Deg = 180.0 / kPi;
constexpr double kPhaseFraction = 0.2;  // contact-phase offset = +/- 0.2 * |omega| * tau

inline const std::array<const char*, 3> kLaws = {"slerp", "minjerk", "qdmp"};

struct Variant {
    std::string name;
    double sign;
};
inline std::vector<Variant> variants() { return {{"nominal", 0.0}, {"phase_p20", +1.0}, {"phase_m20", -1.0}}; }

inline std::vector<double> parseList(const std::string& s, size_t n, const std::string& what) {
    std::vector<double> v;
    std::stringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, ',')) v.push_back(std::stod(tok));
    if (v.size() != n) throw std::invalid_argument(what + ": expected " + std::to_string(n) + " comma-separated values");
    return v;
}

inline std::string readFile(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open " + path);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

inline Eigen::Quaterniond quatFromXyzw(const std::vector<double>& v, const std::string& what) {
    Eigen::Quaterniond q(v[3], v[0], v[1], v[2]);
    const double n = q.norm();
    if (!(std::isfinite(n) && n > 0.0)) throw std::invalid_argument(what + ": zero or non-finite norm");
    if (std::fabs(n - 1.0) > 1e-3) std::cerr << "WARNING " << what << ": |q| = " << n << ", normalizing\n";
    return q.normalized();
}

inline std::string num(double v) {
    if (!std::isfinite(v)) return "nan";
    std::ostringstream s;
    s << std::setprecision(9) << v;
    return s.str();
}

// Goal of a contact-phase variant: q_goal(theta + delta) = R_axis(delta) * q_goal(theta), in the hold hemisphere.
inline Eigen::Quaterniond variantGoal(const Eigen::Quaterniond& q_hold, const Eigen::Quaterniond& q_goal_nom,
                                      const Eigen::Vector3d& axis, double delta_rad) {
    return go::alignShortestPath(q_hold, (Eigen::Quaterniond(Eigen::AngleAxisd(delta_rad, axis)) * q_goal_nom).normalized());
}

// FrameAligner offset (ros_utils.hpp, align()): Q = q_hold * q_demo(0)^-1, applied by LEFT multiplication.
inline Eigen::Quaterniond qdmpFrameOffset(const Eigen::Quaterniond& q_hold, const Eigen::Quaterniond& q_demo0) {
    return (q_hold * q_demo0.conjugate()).normalized();
}

// QDMP rollout (frame of its weights) on a time grid whose first element is the start instant: grid[0] returns the
// start orientation, then qd.step(grid[i] - grid[i-1]). Keeps alpha_x/alpha_z/beta_z/n_basis of the fit.
inline std::vector<Eigen::Quaterniond> runQdmp(const hdl::QuaternionDMP& qdmp_file, double tau,
                                               const std::vector<double>& grid, const Eigen::Quaterniond& start,
                                               const Eigen::Quaterniond& goal_demo, const Eigen::Vector3d& eta0) {
    hdl::QuaternionDMP qd = qdmp_file;
    qd.setLearnedParameters(tau, start, go::alignShortestPath(start, goal_demo), qdmp_file.centers(), qdmp_file.widths(),
                            qdmp_file.weights(), eta0);
    qd.reset();
    std::vector<Eigen::Quaterniond> r;
    r.reserve(grid.size());
    for (size_t i = 0; i < grid.size(); ++i) r.push_back(i == 0 ? qd.orientation() : qd.step(grid[i] - grid[i - 1]));
    return r;
}

// The three laws (order of kLaws) on `grid` (grid[0] = 0), hold -> q_goal in tau:
//   slerp: go::slerpAt; minjerk: same geodesic with s(u) = 10u^3 - 15u^4 + 6u^5;
//   qdmp:  Q * rollout(start = q_demo(0), goal = Q^-1 q_goal, eta0 = 0), Q = qdmpFrameOffset.
// "Grasp" rollouts always start from rest (eta0 = 0).
inline std::array<std::vector<Eigen::Quaterniond>, 3> buildLawProfiles(
    const std::vector<double>& grid, double tau, const Eigen::Quaterniond& q_hold, const Eigen::Quaterniond& q_goal,
    const hdl::QuaternionDMP& qdmp_file, const Eigen::Quaterniond& Q, const Eigen::Quaterniond& q_demo0) {
    std::array<std::vector<Eigen::Quaterniond>, 3> out;
    const auto qroll = runQdmp(qdmp_file, tau, grid, q_demo0, Q.conjugate() * q_goal, Eigen::Vector3d::Zero());
    for (size_t i = 0; i < grid.size(); ++i) {
        const double t = grid[i];
        const double u = std::min(1.0, t / tau);
        out[0].push_back(go::slerpAt(q_hold, q_goal, t, tau));
        out[1].push_back(hdl::slerpShortestPath(q_hold, q_goal, u * u * u * (10.0 - 15.0 * u + 6.0 * u * u)));
        out[2].push_back((Q * qroll[i]).normalized());
    }
    return out;
}

}  // namespace orientation_laws
