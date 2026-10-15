// Offline comparison of three orientation laws for the approach to the grasp (hold orientation -> planner goal in
// time tau). Pure orientation analysis: no joint simulation, no position rollout. Nothing here changes planner or
// executor behaviour. Build/run instructions: analysis/README.md (target: BUILD_ANALYSIS_TOOLS).
//
//   SLERP     q(t) = q0 * (q0^-1 q1)^s,  s = t / tau   (grasp_orientation::slerpAt, as the executor in planner_goal)
//   min-jerk  same geodesic, s(u) = 10u^3 - 15u^4 + 6u^5, u = t / tau
//   QDMP      haptic_dmp_learning::core::QuaternionDMP (Ude 2014) with the learned weights, run in the DEMO frame and
//             mapped to the hold frame as the FrameAligner does (ros_utils.hpp, align()):
//               Q = q_hold * q_demo(0)^-1,  q_demo(0) = q0 stored in the weights file
//               rollout: start = q_demo(0), goal_demo = Q^-1 * q_goal, eta0 = 0;  output = Q * q_rollout(t)
//
// Contact-phase variants: the goal is graspPoseAt(k, theta + delta); since graspPoseAt builds the orientation as
// R_axis(theta) * B_k, q_goal(theta + delta) = R_axis(delta) * q_goal(theta) (left multiplication, world frame),
// with delta = +/- 0.2 * |omega| * tau.

#include <algorithm>
#include <cctype>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/demo_params.hpp"
#include "haptic_dmp_learning/core/grasp_orientation.hpp"
#include "haptic_dmp_learning/core/math_utils.hpp"
#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "haptic_dmp_learning/core/quaternion_dmp.hpp"
#include "orientation_laws.hpp"

namespace hdl = haptic_dmp_learning::core;
namespace go = haptic_dmp_learning::core::grasp_orientation;
using namespace orientation_laws;
using franka_cartesian_control::core::RobotModel;

namespace {

constexpr double kDt = 0.005;  // profile / integration step [s]

struct Args {
    std::string weights, qdmp_weights, demo_params, urdf, out, demo_csv;
    std::vector<double> q_meas, q_hold_xyzw, goal_xyzw;
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();  // satellite spin axis (world)
    double omega_deg_s = std::numeric_limits<double>::quiet_NaN();
};

Args parseArgs(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        auto val = [&]() -> std::string {
            if (i + 1 >= argc) throw std::invalid_argument("missing value for " + k);
            return argv[++i];
        };
        if (k == "--weights") a.weights = val();
        else if (k == "--qdmp-weights") a.qdmp_weights = val();
        else if (k == "--demo-params") a.demo_params = val();
        else if (k == "--urdf") a.urdf = val();
        else if (k == "--out") a.out = val();
        else if (k == "--demo-csv") a.demo_csv = val();
        else if (k == "--q-meas") a.q_meas = parseList(val(), 7, "--q-meas");
        else if (k == "--q-hold") a.q_hold_xyzw = parseList(val(), 4, "--q-hold");
        else if (k == "--goal") a.goal_xyzw = parseList(val(), 4, "--goal");
        else if (k == "--axis") {
            const auto v = parseList(val(), 3, "--axis");
            a.axis = Eigen::Vector3d(v[0], v[1], v[2]);
        } else if (k == "--omega-deg-s") a.omega_deg_s = std::stod(val());
        else throw std::invalid_argument("unknown argument " + k);
    }
    const bool hold_ok = !a.q_hold_xyzw.empty() || (!a.q_meas.empty() && !a.urdf.empty());
    if (a.weights.empty() || a.demo_params.empty() || a.out.empty() || a.goal_xyzw.empty() || !hold_ok ||
        !std::isfinite(a.omega_deg_s)) {
        throw std::invalid_argument(
            "usage: orientation_law_compare --weights <w.yaml> [--qdmp-weights <w.yaml>] --demo-params <p.yaml> "
            "--goal x,y,z,w --omega-deg-s <w> (--q-meas j1,..,j7 --urdf <fer_flat_effort.urdf> | --q-hold x,y,z,w) "
            "--out <dir> [--axis x,y,z] [--demo-csv <demo.csv>]");
    }
    if (!(a.axis.norm() > 1e-9)) throw std::invalid_argument("--axis has ~zero norm");
    a.axis.normalize();
    if (a.qdmp_weights.empty()) a.qdmp_weights = a.weights;
    return a;
}

struct Profile {
    std::vector<double> t;
    std::vector<Eigen::Quaterniond> q;
};

std::string lower(std::string s) {
    for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> o;
    std::stringstream ss(line);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        tok.erase(std::remove_if(tok.begin(), tok.end(), [](unsigned char c) { return std::isspace(c) || c == '\r'; }), tok.end());
        o.push_back(tok);
    }
    return o;
}

// Orientation of a recorded demo. Column names are deduced from the header: time = t | time | time_s | timestamp;
// quaternion components = {q,quat_,q_,orientation_,ori_} + {w,x,y,z} (case-insensitive). Quaternions are normalized
// and made sign-continuous (dot with the previous sample >= 0).
Profile loadDemoCsv(const std::string& path, std::string& cols_report) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open " + path);
    std::string line;
    if (!std::getline(f, line)) throw std::runtime_error("empty csv " + path);
    const auto hdr = splitCsv(line);
    auto find = [&](const std::vector<std::string>& names) -> int {
        for (const auto& n : names)
            for (size_t i = 0; i < hdr.size(); ++i)
                if (lower(hdr[i]) == n) return static_cast<int>(i);
        return -1;
    };
    const int ct = find({"t", "time", "time_s", "timestamp", "t_s"});
    int cq[4];  // w, x, y, z
    const char* comp[4] = {"w", "x", "y", "z"};
    for (int c = 0; c < 4; ++c) {
        std::vector<std::string> names;
        for (const char* pre : {"q", "quat_", "q_", "orientation_", "ori_"}) names.push_back(std::string(pre) + comp[c]);
        cq[c] = find(names);
    }
    if (ct < 0 || cq[0] < 0 || cq[1] < 0 || cq[2] < 0 || cq[3] < 0) {
        std::string h;
        for (const auto& c : hdr) h += c + " ";
        throw std::runtime_error("--demo-csv: cannot deduce time/quaternion columns from header: " + h);
    }
    cols_report = "time='" + hdr[ct] + "', quaternion w,x,y,z='" + hdr[cq[0]] + "','" + hdr[cq[1]] + "','" + hdr[cq[2]] +
                  "','" + hdr[cq[3]] + "'";
    Profile p;
    const size_t need = static_cast<size_t>(*std::max_element(cq, cq + 4) > ct ? *std::max_element(cq, cq + 4) : ct) + 1;
    while (std::getline(f, line)) {
        const auto v = splitCsv(line);
        if (v.size() < need) continue;
        Eigen::Quaterniond q(std::stod(v[cq[0]]), std::stod(v[cq[1]]), std::stod(v[cq[2]]), std::stod(v[cq[3]]));
        if (!(q.norm() > 0.0) || !std::isfinite(q.norm())) continue;
        q.normalize();
        if (!p.q.empty() && q.coeffs().dot(p.q.back().coeffs()) < 0.0) q.coeffs() = -q.coeffs();
        p.t.push_back(std::stod(v[ct]));
        p.q.push_back(q);
    }
    if (p.t.size() < 2) throw std::runtime_error("--demo-csv: fewer than 2 valid rows in " + path);
    return p;
}

// Demo orientation at time t (SLERP between the bracketing samples, clamped at the ends).
Eigen::Quaterniond sampleProfile(const Profile& p, double t) {
    if (t <= p.t.front()) return p.q.front();
    if (t >= p.t.back()) return p.q.back();
    const size_t i = static_cast<size_t>(std::upper_bound(p.t.begin(), p.t.end(), t) - p.t.begin());
    const double span = p.t[i] - p.t[i - 1];
    const double s = span > 0.0 ? (t - p.t[i - 1]) / span : 0.0;
    return hdl::slerpShortestPath(p.q[i - 1], p.q[i], s);
}

// World-frame rotation vector of the shortest rotation taking a to b (b * a^-1).
Eigen::Vector3d rotVec(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
    Eigen::Quaterniond d = (b * a.conjugate()).normalized();
    if (d.w() < 0.0) d.coeffs() = -d.coeffs();
    const double vn = d.vec().norm();
    if (vn < 1e-15) return Eigen::Vector3d::Zero();
    return 2.0 * std::atan2(vn, d.w()) * d.vec() / vn;
}

struct Metrics {
    double swept_deg = 0.0, omega_max = 0.0, alpha_max = 0.0, resid_deg = 0.0, max_angle_hold_deg = 0.0;
    double diff_max_deg = std::numeric_limits<double>::quiet_NaN(), diff_rms_deg = std::numeric_limits<double>::quiet_NaN();
};

// Intervals shorter than kDt/2 (a possible final remainder when tau is not a multiple of dt) are left out of the
// omega/alpha extrema (finite-difference noise) but counted in the swept angle.
Metrics profileMetrics(const Profile& p, const Eigen::Quaterniond& q_goal, const Eigen::Quaterniond& q_hold) {
    Metrics m;
    std::vector<Eigen::Vector3d> w;
    std::vector<double> tm;
    for (size_t i = 1; i < p.t.size(); ++i) {
        const double dt = p.t[i] - p.t[i - 1];
        const Eigen::Vector3d r = rotVec(p.q[i - 1], p.q[i]);
        m.swept_deg += r.norm() * kRad2Deg;
        if (dt < 0.5 * kDt) continue;
        w.push_back(r / dt);
        tm.push_back(0.5 * (p.t[i] + p.t[i - 1]));
        m.omega_max = std::max(m.omega_max, w.back().norm());
    }
    for (size_t i = 1; i < w.size(); ++i)
        m.alpha_max = std::max(m.alpha_max, (w[i] - w[i - 1]).norm() / (tm[i] - tm[i - 1]));
    m.resid_deg = go::quatAngleRad(p.q.back(), q_goal) * kRad2Deg;
    for (const auto& q : p.q) m.max_angle_hold_deg = std::max(m.max_angle_hold_deg, go::quatAngleRad(q, q_hold) * kRad2Deg);
    return m;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parseArgs(argc, argv);

        // ---- inputs -------------------------------------------------------------------------------------
        const auto dp = hdl::demo_params::read(args.demo_params);
        const double tau = dp.tau_s;
        if (!(tau > 0.0)) throw std::runtime_error("tau_s of the demo params is not > 0");

        hdl::QuaternionDMP qdmp_file;
        bool has_q = false;
        (void)hdl::prodmp_io::loadProDmpFromYaml(args.qdmp_weights, qdmp_file, has_q);
        if (!has_q || !qdmp_file.isLearned()) {
            throw std::runtime_error("no learned quaternion_dmp section in " + args.qdmp_weights);
        }
        if (std::fabs(qdmp_file.tau() - tau) > 1e-6) {
            std::cerr << "WARNING: quaternion_dmp tau (" << qdmp_file.tau() << ") != demo-params tau_s (" << tau
                      << "); using demo-params tau_s.\n";
        }

        Eigen::Quaterniond q_hold;
        std::string hold_src;
        if (!args.q_hold_xyzw.empty()) {
            q_hold = quatFromXyzw(args.q_hold_xyzw, "--q-hold");
            hold_src = "--q-hold";
        } else {
            const std::vector<std::string> joint_names = {"fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4",
                                                          "fer_joint5", "fer_joint6", "fer_joint7"};
            RobotModel robot(readFile(args.urdf), joint_names, "fer_hand_tcp");
            RobotModel::JointVector q;
            for (int j = 0; j < 7; ++j) q(j) = args.q_meas[j];
            robot.update(q, RobotModel::JointVector::Zero());
            q_hold = robot.eeOrientation().normalized();
            hold_src = "FK(--q-meas)";
        }
        const Eigen::Quaterniond q_goal_nom = quatFromXyzw(args.goal_xyzw, "--goal");

        const double omega_rad_s = std::fabs(args.omega_deg_s) / kRad2Deg;
        const double delta_rad = kPhaseFraction * omega_rad_s * tau;

        const std::vector<Variant> variants_ = variants();

        std::filesystem::create_directories(args.out);
        std::ofstream sum(args.out + "/summary.csv");
        if (!sum) throw std::runtime_error("cannot write summary.csv in " + args.out);
        sum << "variant,delta_deg,law,swept_deg,omega_max_rad_s,alpha_max_rad_s2,resid_at_tau_deg,max_angle_from_hold_deg,diff_vs_slerp_max_deg,"
               "diff_vs_slerp_rms_deg\n";

        std::cout << std::fixed << std::setprecision(4);
        std::cout << "tau=" << tau << " s (demo-params), dt=" << kDt << " s, |omega|=" << std::fabs(args.omega_deg_s)
                  << " deg/s, phase offset=+/-" << delta_rad * kRad2Deg << " deg about axis [" << args.axis.transpose()
                  << "]\nhold from " << hold_src << " xyzw=[" << q_hold.x() << ' ' << q_hold.y() << ' ' << q_hold.z() << ' '
                  << q_hold.w() << "]  goal(nominal) xyzw=[" << q_goal_nom.x() << ' ' << q_goal_nom.y() << ' '
                  << q_goal_nom.z() << ' ' << q_goal_nom.w() << "]\n";

        // time grid: i*dt up to tau, plus tau itself if it is not a multiple of dt
        std::vector<double> grid;
        for (int i = 0; i * kDt < tau - 1e-9; ++i) grid.push_back(i * kDt);
        grid.push_back(tau);

        // QDMP in the DEMO frame: weights, q0 and the forcing term live in the frame of the recorded demo. The hold
        // frame is reached with the FrameAligner's orientation offset (ros_utils.hpp, align()):
        //   Q = q_hold * q_demo(0)^-1, applied by LEFT multiplication, q_out = (Q * q_rollout).normalized().
        const Eigen::Quaterniond q_demo0 = qdmp_file.q0().normalized();
        const Eigen::Quaterniond Q = qdmpFrameOffset(q_hold, q_demo0);
        auto runQdmp = [&](const Eigen::Quaterniond& start, const Eigen::Quaterniond& goal_demo, const Eigen::Vector3d& eta0) {
            return orientation_laws::runQdmp(qdmp_file, tau, grid, start, goal_demo, eta0);
        };
        std::cout << "QDMP weights: DEMO frame (q_demo(0) from weights file xyzw=[" << q_demo0.x() << ' ' << q_demo0.y() << ' '
                  << q_demo0.z() << ' ' << q_demo0.w() << "]); output = Q * q_rollout, Q = q_hold * q_demo(0)^-1 (rotation "
                  << go::quatAngleRad(Q, Eigen::Quaterniond::Identity()) * kRad2Deg << " deg); rollout goal = Q^-1 * goal, eta0 = 0\n";

        // ---- recorded demo (optional) ----
        if (args.demo_csv.empty()) {
            std::cout << "\n== recorded demo: SKIPPED (no --demo-csv given)\n";
        } else {
            std::string cols;
            const Profile demo = loadDemoCsv(args.demo_csv, cols);
            double swept = 0.0;
            for (size_t i = 1; i < demo.q.size(); ++i) swept += go::quatAngleRad(demo.q[i - 1], demo.q[i]) * kRad2Deg;
            const double dur = demo.t.back() - demo.t.front();
            const Eigen::Quaterniond& d0 = demo.q.front();
            const Eigen::Quaterniond& d1 = demo.q.back();
            std::cout << "\n== recorded demo " << args.demo_csv << "\n  columns: " << cols << ", rows " << demo.t.size()
                      << "\n  duration " << dur << " s (tau " << tau << " s), swept " << swept << " deg, angle q(0)->q(end) "
                      << go::quatAngleRad(d0, d1) * kRad2Deg << " deg\n  angle(csv q(0), weights q0) "
                      << go::quatAngleRad(d0, q_demo0) * kRad2Deg << " deg, angle(csv q(end), weights goal) "
                      << go::quatAngleRad(d1, qdmp_file.goal()) * kRad2Deg << " deg\n";
            // Reproduction control: the file's own eta0 (initial angular-velocity offset of the fit), NOT zero.
            const Eigen::Vector3d eta0_file = qdmp_file.eta0();
            const auto r = runQdmp(d0, d1, eta0_file);  // demo frame, start = csv q(0), goal = csv q(end), tau of the demo-params
            double emax = 0.0, esq = 0.0;
            for (size_t i = 0; i < grid.size(); ++i) {
                const double e = go::quatAngleRad(r[i], sampleProfile(demo, demo.t.front() + grid[i])) * kRad2Deg;
                emax = std::max(emax, e);
                esq += e * e;
            }
            std::cout << "  reproduction control: eta0 = FROM THE WEIGHTS FILE (|eta0| = " << eta0_file.norm()
                      << " rad/s); grasp rollouts below: eta0 = 0 (start from rest)\n";
            std::cout << "  reproduction (QDMP, demo frame, start=q(0), goal=q(end)): max err " << emax << " deg, RMS "
                      << std::sqrt(esq / grid.size()) << " deg, residual at tau " << go::quatAngleRad(r.back(), d1) * kRad2Deg
                      << " deg\n";
        }

        for (const Variant& v : variants_) {
            const double delta = v.sign * delta_rad;
            // q_goal(theta + delta) = R_axis(delta) * q_goal(theta); realigned to the hold hemisphere
            const Eigen::Quaterniond q_goal = variantGoal(q_hold, q_goal_nom, args.axis, delta);

            std::array<Profile, 3> prof;
            const auto laws = buildLawProfiles(grid, tau, q_hold, q_goal, qdmp_file, Q, q_demo0);
            for (size_t l = 0; l < 3; ++l) { prof[l].t = grid; prof[l].q = laws[l]; }

            std::array<Metrics, 3> met;
            for (size_t l = 0; l < 3; ++l) met[l] = profileMetrics(prof[l], q_goal, q_hold);
            double dmax = 0.0, dsq = 0.0;
            for (size_t i = 0; i < grid.size(); ++i) {
                const double d = go::quatAngleRad(prof[0].q[i], prof[2].q[i]) * kRad2Deg;
                dmax = std::max(dmax, d);
                dsq += d * d;
            }
            met[2].diff_max_deg = dmax;
            met[2].diff_rms_deg = std::sqrt(dsq / grid.size());

            std::cout << "\n== " << v.name << "  (tau = " << tau << " s, phase offset = " << delta * kRad2Deg
                      << " deg, goal xyzw=[" << q_goal.x() << ' ' << q_goal.y() << ' ' << q_goal.z() << ' ' << q_goal.w()
                      << "], angle hold->goal = " << go::quatAngleRad(q_hold, q_goal) * kRad2Deg
                      << " deg, QDMP weights in demo frame)\n"
                      << std::left << std::setw(9) << "law" << std::right << std::setw(12) << "swept[deg]" << std::setw(14)
                      << "|w|max[rad/s]" << std::setw(16) << "|a|max[rad/s2]" << std::setw(14) << "resid@tau[deg]" << std::setw(16) << "maxang_hold[deg]" << '\n';
            for (size_t l = 0; l < 3; ++l) {
                std::cout << std::left << std::setw(9) << kLaws[l] << std::right << std::setw(12) << met[l].swept_deg
                          << std::setw(14) << met[l].omega_max << std::setw(16) << met[l].alpha_max << std::setw(14)
                          << met[l].resid_deg << std::setw(16) << met[l].max_angle_hold_deg << '\n';
                sum << v.name << ',' << num(delta * kRad2Deg) << ',' << kLaws[l] << ',' << num(met[l].swept_deg) << ','
                    << num(met[l].omega_max) << ',' << num(met[l].alpha_max) << ',' << num(met[l].resid_deg) << ',' << num(met[l].max_angle_hold_deg) << ','
                    << num(l == 2 ? met[l].diff_max_deg : std::numeric_limits<double>::quiet_NaN()) << ','
                    << num(l == 2 ? met[l].diff_rms_deg : std::numeric_limits<double>::quiet_NaN()) << '\n';
            }
            std::cout << "SLERP vs QDMP angular difference over time: max " << met[2].diff_max_deg << " deg, RMS "
                      << met[2].diff_rms_deg << " deg\n";

            std::ofstream pf(args.out + "/profile_" + v.name + ".csv");
            if (!pf) throw std::runtime_error("cannot write profile_" + v.name + ".csv in " + args.out);
            pf << "t_s";
            for (const char* law : kLaws) pf << ',' << law << "_x," << law << "_y," << law << "_z," << law << "_w";
            pf << '\n';
            for (size_t i = 0; i < grid.size(); ++i) {
                pf << num(grid[i]);
                for (size_t l = 0; l < 3; ++l) {
                    // same hemisphere as the hold (q and -q are the same rotation)
                    Eigen::Quaterniond q = prof[l].q[i];
                    if (q.coeffs().dot(q_hold.coeffs()) < 0.0) q.coeffs() = -q.coeffs();
                    pf << ',' << num(q.x()) << ',' << num(q.y()) << ',' << num(q.z()) << ',' << num(q.w());
                }
                pf << '\n';
            }
        }
        std::cout << "\nwrote " << args.out << "/summary.csv, profile_{nominal,phase_p20,phase_m20}.csv\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "orientation_law_compare: " << e.what() << "\n";
        return 1;
    }
}
