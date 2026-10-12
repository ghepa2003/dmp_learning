// Offline comparison of five orientation laws for the approach to the grasp (hold pose -> goal in time tau),
// run through the planner's own kinematic-feasibility machinery (JointPathSimulator + NullspaceBiasedResolution,
// dt = 0.005 s, same ProDMP position rollout as checkKinematicFeasibility). Nothing here changes planner or
// executor behaviour. Build/run instructions: analysis/README.md.
//
//   A0  SLERP, constant speed, angular twist = 0   (what the planner feeds the simulator today)
//   A   SLERP, constant speed, analytic angular twist
//   B   SLERP with s(u) = 10u^3 - 15u^4 + 6u^5, u = t/tau, analytic angular twist
//   C   QuaternionDMP, zero forcing term (pure critically damped attractor), twist = omega()
//   D   QuaternionDMP with the learned weights, twist = omega()

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/demo_params.hpp"
#include "haptic_dmp_learning/core/grasp_orientation.hpp"
#include "haptic_dmp_learning/core/math_utils.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "haptic_dmp_learning/core/quaternion_dmp.hpp"
#include "satellite_grasp_planner/core/joint_path_simulator.hpp"
#include "satellite_grasp_planner/core/kinematic_feasibility.hpp"

namespace hdl = haptic_dmp_learning::core;
namespace go = haptic_dmp_learning::core::grasp_orientation;
using satellite_grasp_planner::core::CartesianSample;
using satellite_grasp_planner::core::JointPathSimulator;
using satellite_grasp_planner::core::NullspaceBiasedResolution;
using franka_cartesian_control::core::RobotModel;

namespace {

constexpr double kDt = 0.005;                // planner default (kinematic_feasibility.hpp)
constexpr double kViolTol = 1e-4;            // planner default max_joint_violation_tol_rad
constexpr double kPi = 3.14159265358979323846;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kMarginAdoptRad = 0.3;

const std::array<const char*, 5> kLaws = {"A0", "A", "B", "C", "D"};

struct Args {
    std::string weights, qdmp_weights, demo_params, urdf, out, demo_k = "";
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
        else if (k == "--demo-k") a.demo_k = val();
        else throw std::invalid_argument("unknown argument " + k);
    }
    if (a.weights.empty() || a.demo_params.empty() || a.urdf.empty() || a.out.empty()) {
        throw std::invalid_argument(
            "usage: orientation_law_compare --weights <w.yaml> [--qdmp-weights <w.yaml>] --demo-params <p.yaml> "
            "--urdf <fer_flat_effort.urdf> --out <dir> [--demo-k kP0|kP90|kP180|kP270]");
    }
    if (a.qdmp_weights.empty()) a.qdmp_weights = a.weights;
    return a;
}

std::string readFile(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open " + path);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

// velocity="..." of <joint name="fer_jointN" ...> in the URDF (RobotModel does not expose velocity limits).
std::array<double, 7> urdfVelocityLimits(const std::string& urdf) {
    std::array<double, 7> v{};
    for (int j = 1; j <= 7; ++j) {
        const std::string tag = "<joint name=\"fer_joint" + std::to_string(j) + "\"";
        const auto p = urdf.find(tag);
        if (p == std::string::npos) throw std::runtime_error("URDF: joint fer_joint" + std::to_string(j) + " not found");
        const auto e = urdf.find("</joint>", p);
        const auto l = urdf.find("<limit", p);
        if (l == std::string::npos || l > e) throw std::runtime_error("URDF: no <limit> for fer_joint" + std::to_string(j));
        const auto a = urdf.find("velocity=\"", l);
        if (a == std::string::npos) throw std::runtime_error("URDF: no velocity for fer_joint" + std::to_string(j));
        v[j - 1] = std::stod(urdf.substr(a + 10));
    }
    return v;
}

struct Case {
    std::string name, variant;
    hdl::GraspPointId k;
    std::string k_name;
    double theta_deg;
    double omega_rad_s;
    Eigen::Vector3d center, axis;
    Eigen::Vector3d face_normal;
};

struct Metrics {
    bool feasible = false;
    double max_viol = 0.0;
    double ang_at_3cm_deg = kNaN, ang_at_1cm_deg = kNaN;
    double t_2deg_frac = kNaN;
    double peak_omega = 0.0;
    double peak_qdot = 0.0;
    int peak_qdot_joint = -1;
    double peak_qdot_ratio = 0.0;  // max_j |qdot_j| / vlim_j
    double min_margin = std::numeric_limits<double>::infinity();
    int min_margin_joint = -1;
    double min_margin_t = 0.0;
    double min_wtrans = std::numeric_limits<double>::infinity();
    double pos_err_final_mm = 0.0, ori_err_final_deg = 0.0;
    std::string planner_check = "";  // A0 only: agreement with checkKinematicFeasibility
};

std::string num(double v) {
    if (!std::isfinite(v)) return "nan";
    std::ostringstream s;
    s << std::setprecision(8) << v;
    return s.str();
}

double median(std::vector<double> v) {
    v.erase(std::remove_if(v.begin(), v.end(), [](double x) { return !std::isfinite(x); }), v.end());
    if (v.empty()) return kNaN;
    std::sort(v.begin(), v.end());
    const size_t n = v.size();
    return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parseArgs(argc, argv);

        // ---- inputs -------------------------------------------------------------------------------------
        hdl::QuaternionDMP qdmp_pos_file;  // QDMP found in --weights (if any)
        bool has_q_pos = false;
        const hdl::ProDMP prodmp_template = hdl::prodmp_io::loadProDmpFromYaml(args.weights, qdmp_pos_file, has_q_pos);
        hdl::QuaternionDMP qdmp_learned;
        bool has_q = false;
        if (args.qdmp_weights == args.weights) {
            qdmp_learned = qdmp_pos_file;
            has_q = has_q_pos;
        } else {
            (void)hdl::prodmp_io::loadProDmpFromYaml(args.qdmp_weights, qdmp_learned, has_q);
        }
        if (!has_q || !qdmp_learned.isLearned()) {
            throw std::runtime_error("no learned quaternion_dmp section in " + args.qdmp_weights);
        }
        const double tau = prodmp_template.tau();
        if (!(tau > 0.0)) throw std::runtime_error("tau of the position weights is not > 0");

        const auto dp = hdl::demo_params::read(args.demo_params);
        hdl::GraspPointId demo_k = hdl::GraspPointId::kP270;
        if (!args.demo_k.empty()) demo_k = hdl::demo_params::parseGraspPoint(args.demo_k);
        else if (dp.satellite_at_demo) demo_k = dp.satellite_at_demo->grasp_point;
        if (std::fabs(dp.tau_s - tau) > 1e-6) {
            std::cerr << "WARNING: demo-params tau_s (" << dp.tau_s << ") != tau of the position weights (" << tau
                      << "); using the weights' tau.\n";
        }

        const std::string urdf = readFile(args.urdf);
        const auto vlim = urdfVelocityLimits(urdf);
        const std::vector<std::string> joint_names = {"fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4",
                                                      "fer_joint5", "fer_joint6", "fer_joint7"};
        auto robot = std::make_shared<RobotModel>(urdf, joint_names, "fer_hand_tcp");
        const RobotModel::JointVector lo = robot->jointLowerLimits();
        const RobotModel::JointVector hi = robot->jointUpperLimits();

        RobotModel::JointVector q0;
        q0 << 0.0, -0.7854, 0.0, -2.3562, 0.0, 3.1416, 0.7854;  // ready_q6pi
        robot->update(q0, RobotModel::JointVector::Zero());
        const Eigen::Vector3d p0 = robot->eePosition();
        const Eigen::Quaterniond q_hold = robot->eeOrientation();  // hold orientation from FK

        // ---- cases --------------------------------------------------------------------------------------
        const std::vector<std::pair<hdl::GraspPointId, std::string>> ks = {{hdl::GraspPointId::kP0, "kP0"},
                                                                          {hdl::GraspPointId::kP90, "kP90"},
                                                                          {hdl::GraspPointId::kP180, "kP180"},
                                                                          {hdl::GraspPointId::kP270, "kP270"}};
        const std::vector<double> thetas = {-120.0, -60.0, 0.0};
        struct Variant { std::string name; double omega_scale; Eigen::Vector3d dc; Eigen::Matrix3d R; };
        const double tilt = 18.0 * kPi / 180.0;
        const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
        const std::vector<Variant> variants = {
            {"nominal", 1.0, Eigen::Vector3d::Zero(), I3},
            {"omega_p20", 1.2, Eigen::Vector3d::Zero(), I3},
            {"omega_m20", 0.8, Eigen::Vector3d::Zero(), I3},
            {"cx_p15", 1.0, Eigen::Vector3d(0.15, 0, 0), I3},
            {"cx_m15", 1.0, Eigen::Vector3d(-0.15, 0, 0), I3},
            {"cy_p15", 1.0, Eigen::Vector3d(0, 0.15, 0), I3},
            {"cy_m15", 1.0, Eigen::Vector3d(0, -0.15, 0), I3},
            {"tilt_x18", 1.0, Eigen::Vector3d::Zero(), Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitX()).toRotationMatrix()},
            {"tilt_y18", 1.0, Eigen::Vector3d::Zero(), Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitY()).toRotationMatrix()},
        };
        const double omega_nom = -2.0 * kPi / 180.0;
        std::vector<Case> cases;
        for (const auto& [k, kn] : ks)
            for (double th : thetas)
                for (const auto& v : variants) {
                    Case c;
                    c.k = k;
                    c.k_name = kn;
                    c.theta_deg = th;
                    c.variant = v.name;
                    c.omega_rad_s = omega_nom * v.omega_scale;
                    c.center = Eigen::Vector3d(0.75, 0.0, 0.77) + v.dc;
                    c.axis = v.R * Eigen::Vector3d::UnitZ();
                    c.face_normal = v.R * Eigen::Vector3d::UnitX();  // tilt the whole satellite: face stays lateral
                    std::ostringstream n;
                    n << kn << "_th" << static_cast<int>(th) << "_" << v.name;
                    c.name = n.str();
                    cases.push_back(c);
                }

        // ---- outputs ------------------------------------------------------------------------------------
        const std::string trace_dir = args.out + "/traces";
        if (std::system(("mkdir -p '" + trace_dir + "'").c_str()) != 0) throw std::runtime_error("cannot create " + trace_dir);
        std::ofstream sum(args.out + "/summary.csv");
        if (!sum) throw std::runtime_error("cannot write summary.csv in " + args.out);
        sum << "case,k,theta_deg,variant,omega_rad_s,law,feasible,max_joint_violation_rad,ang_at_pos3cm_deg,"
               "ang_at_pos1cm_deg,t_ang_below_2deg_frac_tau,peak_omega_cmd_rad_s,peak_joint_vel_rad_s,peak_joint_vel_joint,"
               "peak_joint_vel_over_limit,min_limit_margin_rad,min_margin_joint,min_margin_t_s,min_w_trans,"
               "pos_err_final_mm,ori_err_final_deg,planner_check\n";

        std::map<std::string, std::vector<std::pair<const Case*, Metrics>>> by_law;
        const int n_steps = static_cast<int>(std::ceil(tau / kDt));
        std::cout << "tau=" << tau << " s, dt=" << kDt << ", n_steps=" << n_steps << ", cases=" << cases.size()
                  << ", runs=" << cases.size() * kLaws.size() << ", demo k=" << static_cast<int>(demo_k) << "\n";

        for (const Case& c : cases) {
            hdl::CubeSatelliteModel::Params sp;
            sp.center_world = c.center;
            sp.axis_world = c.axis;
            sp.cube_side_m = 0.150;
            sp.collar_radius_m = 0.0725;
            sp.wall_thickness_m = 0.009;
            sp.grasp_radius_m = 0.068;
            sp.standoff_m = 0.08;
            sp.face_normal_body = c.face_normal;
            const hdl::CubeSatelliteModel model(sp);
            const auto target = model.graspPoseAt(c.k, c.theta_deg * kPi / 180.0);
            const Eigen::Vector3d p_goal = target.position_world;
            const Eigen::Quaterniond q_goal = go::alignShortestPath(q_hold, target.orientation_nominal_world.normalized());

            // Position trajectory: ProDMP with the case's goal, started at the FK position (as the planner).
            hdl::ProDMP prodmp = prodmp_template;
            prodmp.setRelativeGoal(false);
            prodmp.setInitialConditions(0.0, p0, Eigen::Vector3d::Zero());
            prodmp.setGoal(p_goal);
            std::vector<Eigen::Vector3d> pos(n_steps);
            for (int i = 0; i < n_steps; ++i) pos[i] = prodmp.step(kDt);

            const Eigen::Vector3d omega_const = go::slerpAngularVelocityWorld(q_hold, q_goal, tau);

            for (const char* law : kLaws) {
                const std::string L = law;
                // ---- orientation + twist per law ----
                std::vector<CartesianSample> traj(n_steps);
                hdl::QuaternionDMP qd;
                if (L == "C" || L == "D") {
                    qd = qdmp_learned;  // keeps alpha_x/alpha_z/beta_z of the production fit
                    std::array<Eigen::VectorXd, 3> w = qd.weights();
                    if (L == "C") for (auto& wi : w) wi = Eigen::VectorXd::Zero(qd.nBasis());
                    qd.setLearnedParameters(tau, q_hold, q_goal, qd.centers(), qd.widths(), w, Eigen::Vector3d::Zero());
                    qd.reset();
                }
                for (int i = 0; i < n_steps; ++i) {
                    const double t = (i + 1) * kDt;
                    const double u = std::min(1.0, t / tau);
                    CartesianSample& s = traj[i];
                    s.position = pos[i];
                    if (L == "A0" || L == "A") {
                        s.orientation = go::slerpAt(q_hold, q_goal, t, tau);
                        if (L == "A" && u < 1.0) s.angular_velocity = omega_const;
                    } else if (L == "B") {
                        const double sb = u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
                        s.orientation = hdl::slerpShortestPath(q_hold, q_goal, sb);
                        if (u < 1.0) s.angular_velocity = omega_const * (30.0 * u * u * (1.0 - u) * (1.0 - u));
                    } else {
                        s.orientation = qd.step(kDt);
                        s.angular_velocity = qd.omega();
                    }
                }

                // ---- simulate ----
                NullspaceBiasedResolution::Params ns;
                JointPathSimulator::Params sim_params;
                sim_params.dt = kDt;
                sim_params.strategy = std::make_shared<NullspaceBiasedResolution>(ns);
                const JointPathSimulator sim(robot, sim_params);
                const auto steps = sim.simulate(traj, q0);

                Metrics m;
                std::ofstream tr(trace_dir + "/" + c.name + "_" + L + ".csv");
                tr << "t_s,pos_cmd_x,pos_cmd_y,pos_cmd_z,q_cmd_x,q_cmd_y,q_cmd_z,q_cmd_w,omega_cmd_x,omega_cmd_y,omega_cmd_z,"
                      "ee_x,ee_y,ee_z,ee_qx,ee_qy,ee_qz,ee_qw,pos_dist_goal_m,ang_dist_goal_deg,q1,q2,q3,q4,q5,q6,q7,w_trans\n";
                bool got3 = false, got1 = false, got2 = false;
                for (size_t i = 0; i < steps.size(); ++i) {
                    const double t = (i + 1) * kDt;
                    robot->update(steps[i].q, RobotModel::JointVector::Zero());
                    const Eigen::Vector3d ee = robot->eePosition();
                    const Eigen::Quaterniond eq = robot->eeOrientation();
                    const double pd = (ee - p_goal).norm();
                    const double ad = go::quatAngleRad(eq, q_goal) * 180.0 / kPi;
                    if (!got3 && pd < 0.03) { got3 = true; m.ang_at_3cm_deg = ad; }
                    if (!got1 && pd < 0.01) { got1 = true; m.ang_at_1cm_deg = ad; }
                    if (!got2 && ad < 2.0) { got2 = true; m.t_2deg_frac = t / tau; }
                    m.peak_omega = std::max(m.peak_omega, traj[i].angular_velocity.norm());
                    m.min_wtrans = std::min(m.min_wtrans, steps[i].w_trans);
                    for (int j = 0; j < 7; ++j) {
                        const double qj = steps[i].q(j);
                        const double mg = std::min(qj - lo(j), hi(j) - qj);
                        if (mg < m.min_margin) { m.min_margin = mg; m.min_margin_joint = j + 1; m.min_margin_t = i * kDt; }
                        m.max_viol = std::max({m.max_viol, lo(j) - qj, qj - hi(j)});
                        if (i + 1 < steps.size()) {
                            const double qd_j = std::fabs(steps[i + 1].q(j) - qj) / kDt;
                            if (qd_j > m.peak_qdot) { m.peak_qdot = qd_j; m.peak_qdot_joint = j + 1; }
                            m.peak_qdot_ratio = std::max(m.peak_qdot_ratio, qd_j / vlim[j]);
                        }
                    }
                    tr << num(t) << ',' << num(traj[i].position.x()) << ',' << num(traj[i].position.y()) << ','
                       << num(traj[i].position.z()) << ',' << num(traj[i].orientation.x()) << ','
                       << num(traj[i].orientation.y()) << ',' << num(traj[i].orientation.z()) << ','
                       << num(traj[i].orientation.w()) << ',' << num(traj[i].angular_velocity.x()) << ','
                       << num(traj[i].angular_velocity.y()) << ',' << num(traj[i].angular_velocity.z()) << ','
                       << num(ee.x()) << ',' << num(ee.y()) << ',' << num(ee.z()) << ',' << num(eq.x()) << ','
                       << num(eq.y()) << ',' << num(eq.z()) << ',' << num(eq.w()) << ',' << num(pd) << ',' << num(ad);
                    for (int j = 0; j < 7; ++j) tr << ',' << num(steps[i].q(j));
                    tr << ',' << num(steps[i].w_trans) << '\n';
                    if (i + 1 == steps.size()) {
                        m.pos_err_final_mm = pd * 1000.0;
                        m.ori_err_final_deg = ad;
                    }
                }
                m.feasible = !steps.empty() && m.max_viol < kViolTol;  // same criterion as checkKinematicFeasibility

                if (L == "A0") {  // fidelity check against the real planner function
                    const auto kin = satellite_grasp_planner::core::checkKinematicFeasibility(
                        prodmp_template, p_goal, q_goal, robot, q0, kDt, kViolTol);
                    const double dq_final = (kin.q_final - steps.back().q).norm();
                    const bool same = (kin.feasible == m.feasible) && dq_final < 1e-9;
                    m.planner_check = same ? "match" : ("MISMATCH(dq_final=" + num(dq_final) + ")");
                    if (!same) std::cerr << "WARNING " << c.name << ": A0 differs from checkKinematicFeasibility: "
                                         << m.planner_check << "\n";
                }

                sum << c.name << ',' << c.k_name << ',' << c.theta_deg << ',' << c.variant << ',' << num(c.omega_rad_s) << ','
                    << L << ',' << (m.feasible ? "yes" : "no") << ',' << num(m.max_viol) << ',' << num(m.ang_at_3cm_deg) << ','
                    << num(m.ang_at_1cm_deg) << ',' << num(m.t_2deg_frac) << ',' << num(m.peak_omega) << ','
                    << num(m.peak_qdot) << ',' << m.peak_qdot_joint << ',' << num(m.peak_qdot_ratio) << ','
                    << num(m.min_margin) << ',' << m.min_margin_joint << ',' << num(m.min_margin_t) << ','
                    << num(m.min_wtrans) << ',' << num(m.pos_err_final_mm) << ',' << num(m.ori_err_final_deg) << ','
                    << m.planner_check << '\n';
                by_law[L].push_back({&c, m});
            }
            std::cout << "done " << c.name << "\n";
        }

        // ---- per-law summary (median / worst) and decision rule ----------------------------------------
        std::ofstream md(args.out + "/summary_by_law.md");
        md << "# Orientation laws: summary (" << cases.size() << " cases, tau=" << tau << " s)\n\n"
           << "Median over cases / worst case. 'worst' = max, except margin and w_trans (min). NaN (never reached) is "
              "excluded from medians and counted in `n_never`.\n\n"
           << "| law | feasible | ang@3cm med/worst [deg] | ang@1cm med/worst [deg] | t(<2deg) med/worst [tau] | n_never(2deg) | "
              "peak omega med/worst [rad/s] | peak qdot med/worst [rad/s] | qdot/limit worst | min margin med/worst [rad] | "
              "min w_trans med/worst | pos err final med/worst [mm] | ori err final med/worst [deg] |\n"
           << "|---|---|---|---|---|---|---|---|---|---|---|---|---|\n";
        struct Agg { bool all_feasible; double worst_margin, worst_ratio; };
        std::map<std::string, Agg> agg;
        auto col = [](const std::vector<std::pair<const Case*, Metrics>>& v, double Metrics::*f) {
            std::vector<double> o;
            for (const auto& e : v) o.push_back(e.second.*f);
            return o;
        };
        auto worstMax = [](const std::vector<double>& v) {
            double w = -std::numeric_limits<double>::infinity();
            for (double x : v) if (std::isfinite(x)) w = std::max(w, x);
            return std::isfinite(w) ? w : kNaN;
        };
        auto worstMin = [](const std::vector<double>& v) {
            double w = std::numeric_limits<double>::infinity();
            for (double x : v) if (std::isfinite(x)) w = std::min(w, x);
            return std::isfinite(w) ? w : kNaN;
        };
        for (const char* law : kLaws) {
            const auto& v = by_law[law];
            int nfeas = 0, nnever = 0;
            for (const auto& e : v) { nfeas += e.second.feasible; nnever += !std::isfinite(e.second.t_2deg_frac); }
            auto a3 = col(v, &Metrics::ang_at_3cm_deg), a1 = col(v, &Metrics::ang_at_1cm_deg), t2 = col(v, &Metrics::t_2deg_frac),
                 po = col(v, &Metrics::peak_omega), pq = col(v, &Metrics::peak_qdot), pr = col(v, &Metrics::peak_qdot_ratio),
                 mm = col(v, &Metrics::min_margin), ww = col(v, &Metrics::min_wtrans), pe = col(v, &Metrics::pos_err_final_mm),
                 oe = col(v, &Metrics::ori_err_final_deg);
            md << "| " << law << " | " << nfeas << "/" << v.size() << " | " << num(median(a3)) << " / " << num(worstMax(a3))
               << " | " << num(median(a1)) << " / " << num(worstMax(a1)) << " | " << num(median(t2)) << " / " << num(worstMax(t2))
               << " | " << nnever << " | " << num(median(po)) << " / " << num(worstMax(po)) << " | " << num(median(pq)) << " / "
               << num(worstMax(pq)) << " | " << num(worstMax(pr)) << " | " << num(median(mm)) << " / " << num(worstMin(mm))
               << " | " << num(median(ww)) << " / " << num(worstMin(ww)) << " | " << num(median(pe)) << " / " << num(worstMax(pe))
               << " | " << num(median(oe)) << " / " << num(worstMax(oe)) << " |\n";
            agg[law] = {nfeas == static_cast<int>(v.size()), worstMin(mm), worstMax(pr)};
        }

        md << "\n## Decision rule\n\nC or D adoptable iff feasible in ALL cases, worst-case min margin > " << kMarginAdoptRad
           << " rad and joint speed within the URDF limits (worst qdot/limit <= 1).\n\n";
        for (const char* law : {"C", "D"}) {
            const Agg& g = agg[law];
            const bool ok = g.all_feasible && g.worst_margin > kMarginAdoptRad && g.worst_ratio <= 1.0;
            md << "- " << law << ": all feasible=" << (g.all_feasible ? "yes" : "no") << ", worst margin=" << num(g.worst_margin)
               << " rad, worst qdot/limit=" << num(g.worst_ratio) << " -> **" << (ok ? "ADOPTABLE" : "NOT adoptable") << "**\n";
        }
        // D preferred over C only if it improves >=1 metric (median) over the cases with k != demo k.
        auto subMedian = [&](const char* law, double Metrics::*f) {
            std::vector<double> o;
            for (const auto& e : by_law[law]) if (e.first->k != demo_k) o.push_back(e.second.*f);
            return median(o);
        };
        struct M { const char* name; double Metrics::*f; bool lower_better; };
        const std::vector<M> ms = {{"ang_at_pos3cm_deg", &Metrics::ang_at_3cm_deg, true}, {"ang_at_pos1cm_deg", &Metrics::ang_at_1cm_deg, true},
                                   {"t_ang_below_2deg_frac", &Metrics::t_2deg_frac, true}, {"peak_omega", &Metrics::peak_omega, true},
                                   {"peak_joint_vel", &Metrics::peak_qdot, true}, {"min_margin", &Metrics::min_margin, false},
                                   {"min_w_trans", &Metrics::min_wtrans, false}, {"ori_err_final_deg", &Metrics::ori_err_final_deg, true}};
        md << "\nD vs C on cases with k != demo k (median over those cases):\n\n| metric | C | D | D better |\n|---|---|---|---|\n";
        int improved = 0;
        for (const auto& mt : ms) {
            const double c = subMedian("C", mt.f), d = subMedian("D", mt.f);
            const bool better = std::isfinite(c) && std::isfinite(d) && (mt.lower_better ? d < c : d > c);
            improved += better;
            md << "| " << mt.name << " | " << num(c) << " | " << num(d) << " | " << (better ? "yes" : "no") << " |\n";
        }
        md << "\nD preferred over C: **" << (improved > 0 ? "YES" : "NO") << "** (" << improved << " metric(s) improved).\n";
        std::cout << "wrote " << args.out << "/summary.csv, summary_by_law.md, traces/\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "orientation_law_compare: " << e.what() << "\n";
        return 1;
    }
}
