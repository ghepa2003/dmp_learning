// Offline kinematic-feasibility check ALONG the path for three orientation laws (SLERP, min-jerk, QDMP) and the three
// contact-phase variants (nominal, phase_p20, phase_m20). Laws, Q, goals and phase offsets are the ones of
// orientation_law_compare (analysis/orientation_laws.hpp). Nothing here changes planner or executor behaviour.
// Build/run instructions: analysis/README.md (target: BUILD_ANALYSIS_TOOLS).
//
// Chain, identical to the planner's post-hoc path check (checkKinematicFeasibility, kinematic_feasibility.cpp):
//   - position: ProDMP copy of the weights file, setRelativeGoal(false), setInitialConditions(0, p0, 0), setGoal(p_goal),
//     n = ceil(tau/dt) calls of step(dt);  p0 = FK(--q-meas), p_goal = p0 + delta_g_demo_m (demo-params) or --p-goal;
//   - orientation: the law evaluated at t_i = (i+1)*dt (the planner's SLERP has s = min(1, (i+1)dt/tau));
//   - kinematics: JointPathSimulator(dt, NullspaceBiasedResolution default Params) seeded with q0 = --q-meas, no feed-forward
//     twist (the planner passes none). There is NO per-sample IK solve and NO joint clamping in the planner's path check:
//     q is integrated (explicit Euler) and compared with the URDF limits afterwards. "IK failure" here = first instant at
//     which a joint leaves its limits by more than the planner's tolerance (max_joint_violation_tol_rad, 1e-4 rad) or q
//     is not finite; the simulation then continues exactly as the planner's does (no stop, no clamp).
// SIMPLIFICATION: the position trajectory (and therefore p_goal) is the same for every variant and law; the phase
// variants change ONLY the goal orientation.

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/demo_params.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "haptic_dmp_learning/core/quaternion_dmp.hpp"
#include "satellite_grasp_planner/core/grasp_pose_eval.hpp"
#include "satellite_grasp_planner/core/grasp_s3_selection.hpp"
#include "satellite_grasp_planner/core/joint_path_simulator.hpp"
#include "orientation_laws.hpp"

using namespace orientation_laws;
using franka_cartesian_control::core::RobotModel;
using satellite_grasp_planner::core::CartesianSample;
using satellite_grasp_planner::core::JointPathSimulator;
using satellite_grasp_planner::core::NullspaceBiasedResolution;

namespace {

constexpr double kViolTolRad = 1e-4;  // checkKinematicFeasibility: max_joint_violation_tol_rad default

struct Args {
    std::string weights, qdmp_weights, demo_params, urdf, out;
    std::vector<double> q_meas, goal_xyzw, p_goal, psi_ref_xyzw;
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    double omega_deg_s = std::numeric_limits<double>::quiet_NaN();
    double dt = 0.02;       // output / profile step [s]
    double sim_dt = 0.005;  // integration step [s] (planner: checkKinematicFeasibility dt default)
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
        else if (k == "--q-meas") a.q_meas = parseList(val(), 7, "--q-meas");
        else if (k == "--goal") a.goal_xyzw = parseList(val(), 4, "--goal");
        else if (k == "--psi-ref") a.psi_ref_xyzw = parseList(val(), 4, "--psi-ref");
        else if (k == "--p-goal") a.p_goal = parseList(val(), 3, "--p-goal");
        else if (k == "--axis") {
            const auto v = parseList(val(), 3, "--axis");
            a.axis = Eigen::Vector3d(v[0], v[1], v[2]);
        } else if (k == "--omega-deg-s") a.omega_deg_s = std::stod(val());
        else if (k == "--dt") a.dt = std::stod(val());
        else if (k == "--sim-dt") a.sim_dt = std::stod(val());
        else throw std::invalid_argument("unknown argument " + k);
    }
    if (a.weights.empty() || a.demo_params.empty() || a.urdf.empty() || a.out.empty() || a.q_meas.empty() ||
        a.goal_xyzw.empty() || !std::isfinite(a.omega_deg_s)) {
        throw std::invalid_argument(
            "usage: orientation_path_check --weights <w.yaml> [--qdmp-weights <w.yaml>] --demo-params <p.yaml> "
            "--urdf <fer_flat_effort.urdf> --q-meas j1,..,j7 --goal x,y,z,w --omega-deg-s <w> --out <dir> "
            "[--axis x,y,z] [--p-goal x,y,z] [--psi-ref x,y,z,w] [--dt 0.02] [--sim-dt 0.005]");
    }
    if (!(a.axis.norm() > 1e-9)) throw std::invalid_argument("--axis has ~zero norm");
    a.axis.normalize();
    if (!(a.sim_dt > 0.0) || !(a.dt >= a.sim_dt - 1e-12)) throw std::invalid_argument("need 0 < --sim-dt <= --dt");
    const double ratio = a.dt / a.sim_dt;
    if (std::fabs(ratio - std::round(ratio)) > 1e-6) throw std::invalid_argument("--dt must be a multiple of --sim-dt");
    if (a.qdmp_weights.empty()) a.qdmp_weights = a.weights;
    return a;
}

double wrapDeg(double d) {
    while (d > 180.0) d -= 360.0;
    while (d <= -180.0) d += 360.0;
    return d;
}

// Twist (psi) and swing of d = q_ref^-1 * q: rotation about the LOCAL z (the approach axis, tcp_roll.hpp applyRoll(q, psi)
// = q * Rz(psi)) and the remaining off-axis rotation. psi in (-180, 180] deg.
void psiSwingDeg(const Eigen::Quaterniond& q_ref, const Eigen::Quaterniond& q, double& psi_deg, double& swing_deg) {
    const Eigen::Quaterniond d = (q_ref.conjugate() * q).normalized();
    psi_deg = wrapDeg(2.0 * std::atan2(d.z(), d.w()) * kRad2Deg);
    swing_deg = 2.0 * std::atan2(std::hypot(d.x(), d.y()), std::hypot(d.w(), d.z())) * kRad2Deg;
}

struct CaseResult {
    std::string variant, law;
    double delta_deg = 0.0;
    double fail_t = -1.0;  // < 0: never
    int fail_joint = 0;
    double max_viol_rad = 0.0;
    double min_margin = std::numeric_limits<double>::infinity();
    int min_margin_joint = 0;
    double min_margin_t = 0.0;
    double min_w_hat = std::numeric_limits<double>::infinity();
    double min_w_hat_t = 0.0;
    double max_qdot = 0.0;
    int max_qdot_joint = 0;
    double max_qdot_ratio = 0.0;
    double max_angle_hold_deg = 0.0;
    double psi_deg = std::numeric_limits<double>::quiet_NaN();
    double swing_deg = std::numeric_limits<double>::quiet_NaN();
    double max_ori_track_deg = 0.0;
};

std::string cell(double v, int prec, bool viol) {
    std::ostringstream s;
    s << std::fixed << std::setprecision(prec) << v << (viol ? "*" : "");
    return s.str();
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parseArgs(argc, argv);
        const int stride = static_cast<int>(std::lround(args.dt / args.sim_dt));

        // ---- thresholds: read from the planner's constants ------------------------------------------------
        const satellite_grasp_planner::core::S3Params s3;  // defaults; the planner node declares the same ones (s1.*)
        const double margin_min = s3.margin_min_rad;
        const double w_hat_min = s3.w_hat_min;
        const NullspaceBiasedResolution::Params ns_params;  // what checkKinematicFeasibility / JointPathSimulator use
        const double qdot_limit = ns_params.ik_params.max_joint_speed;

        // ---- inputs -----------------------------------------------------------------------------------------
        const auto dp = hdl::demo_params::read(args.demo_params);
        if (!dp.end_joints_rad.has_value())
            throw std::runtime_error("demo-params has no demo.end_joints_rad: needed for w_trans_demo (as buildSelectionInputs)");

        hdl::QuaternionDMP qdmp_file;
        bool has_q = false;
        (void)hdl::prodmp_io::loadProDmpFromYaml(args.qdmp_weights, qdmp_file, has_q);
        if (!has_q || !qdmp_file.isLearned()) throw std::runtime_error("no learned quaternion_dmp section in " + args.qdmp_weights);
        const hdl::ProDMP prodmp_template = hdl::prodmp_io::loadProDmpFromYaml(args.weights);
        const double tau = prodmp_template.tau();  // as checkKinematicFeasibility (SLERP s and n_steps use the template's tau)
        if (!(tau > 0.0)) throw std::runtime_error("ProDMP tau is not > 0");
        if (std::fabs(tau - dp.tau_s) > 1e-6)
            std::cerr << "WARNING: ProDMP tau (" << tau << ") != demo-params tau_s (" << dp.tau_s << "); using the ProDMP tau (planner).\n";
        if (std::fabs(qdmp_file.tau() - tau) > 1e-6)
            std::cerr << "WARNING: quaternion_dmp tau (" << qdmp_file.tau() << ") != ProDMP tau (" << tau << "); using the ProDMP tau.\n";

        const std::vector<std::string> joint_names = {"fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4",
                                                      "fer_joint5", "fer_joint6", "fer_joint7"};
        auto robot = std::make_shared<RobotModel>(readFile(args.urdf), joint_names, "fer_hand_tcp");
        RobotModel::JointVector q0, q_end;
        for (int j = 0; j < 7; ++j) {
            q0(j) = args.q_meas[static_cast<size_t>(j)];
            q_end(j) = (*dp.end_joints_rad)[static_cast<size_t>(j)];
        }
        // w_trans_demo exactly as buildSelectionInputs / demoReferenceFromEndJoints (grasp_pose_eval.cpp:125)
        const double w_demo = satellite_grasp_planner::core::demoReferenceFromEndJoints(*robot, q_end).w_trans_demo;
        if (!(w_demo > 0.0)) throw std::runtime_error("w_trans_demo <= 0");

        robot->update(q0, RobotModel::JointVector::Zero());
        const Eigen::Vector3d p0 = robot->eePosition();
        const Eigen::Quaterniond q_hold = robot->eeOrientation().normalized();
        const Eigen::Vector3d p_goal = args.p_goal.empty() ? Eigen::Vector3d(p0 + dp.delta_g_demo_m)
                                                           : Eigen::Vector3d(args.p_goal[0], args.p_goal[1], args.p_goal[2]);
        const Eigen::Quaterniond q_goal_nom = quatFromXyzw(args.goal_xyzw, "--goal");
        const bool have_psi = !args.psi_ref_xyzw.empty();
        const Eigen::Quaterniond psi_ref = have_psi ? quatFromXyzw(args.psi_ref_xyzw, "--psi-ref") : Eigen::Quaterniond::Identity();

        const double delta_rad = kPhaseFraction * std::fabs(args.omega_deg_s) / kRad2Deg * tau;
        const Eigen::Quaterniond q_demo0 = qdmp_file.q0().normalized();
        const Eigen::Quaterniond Q = qdmpFrameOffset(q_hold, q_demo0);

        // ---- position rollout (the planner's, kinematic_feasibility.cpp:38-47) -----------------------------
        const double sdt = args.sim_dt;
        const int n = static_cast<int>(std::ceil(tau / sdt));
        hdl::ProDMP prodmp = prodmp_template;
        prodmp.setRelativeGoal(false);
        prodmp.setInitialConditions(0.0, p0, Eigen::Vector3d::Zero());
        prodmp.setGoal(p_goal);
        std::vector<Eigen::Vector3d> pos(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i) pos[static_cast<size_t>(i)] = prodmp.step(sdt);

        std::vector<double> grid(static_cast<size_t>(n) + 1);  // grid[j] = j*dt; target i uses grid[i+1]
        for (int j = 0; j <= n; ++j) grid[static_cast<size_t>(j)] = j * sdt;

        // ---- header -----------------------------------------------------------------------------------------
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "tau=" << tau << " s (ProDMP), n=" << n << " steps, sim dt=" << sdt << " s (profiles every " << args.dt
                  << " s), |omega|=" << std::fabs(args.omega_deg_s) << " deg/s, phase offset=+/-" << delta_rad * kRad2Deg
                  << " deg about axis [" << args.axis.transpose() << "]\n"
                  << "q0 = --q-meas, p0 = FK = [" << p0.transpose() << "], hold xyzw=[" << q_hold.x() << ' ' << q_hold.y() << ' '
                  << q_hold.z() << ' ' << q_hold.w() << "]\n"
                  << "p_goal = " << (args.p_goal.empty() ? "p0 + delta_g_demo_m (demo-params, planner reproduces the demo, d = 0)" : "--p-goal")
                  << " = [" << p_goal.transpose() << "]\n"
                  << "SIMPLIFICATION: same position trajectory for all laws/variants; the phase variants change ONLY the goal orientation.\n"
                  << "QDMP: weights in DEMO frame, output = Q * rollout, eta0 = 0 (start from rest), as orientation_law_compare.\n"
                  << "Kinematics: JointPathSimulator + NullspaceBiasedResolution (default Params), explicit Euler at sim dt, no feed-forward,\n"
                  << "  no clamping; 'IK fail' = first instant with a joint outside its limits by > " << kViolTolRad
                  << " rad (planner tolerance) or non-finite q.\n"
                  << "THRESHOLDS (planner constants; * marks a violation):\n"
                  << "  min joint margin >= " << margin_min << " rad (S3Params::margin_min_rad), w_hat >= " << w_hat_min
                  << " (S3Params::w_hat_min, w_hat = w_trans / w_trans_demo = " << w_demo << ", wTransFromJacobian, manipulability.hpp),\n"
                  << "  |qdot_j| <= " << qdot_limit << " rad/s (VelocityIkSolver::Params::max_joint_speed used by JointPathSimulator; one value for all joints,\n"
                  << "  the simulator has no per-joint table), IK fail must be 'never'.\n"
                  << "hand-collar distance: NON eseguito: nel core non esiste una funzione di collisione/distanza usata dal planner sulla posa finale"
                  << " (selectGraspS3: \"No collision\"; CubeSatelliteModel espone solo la geometria del collare, non una distanza)\n"
                  << "psi: " << (have_psi ? "psi of q(tau) about the local z relative to --psi-ref (nominal psi = 0 orientation, rotated by the phase offset for the variants)"
                                           : "NON calcolato: serve --psi-ref x,y,z,w (orientation_nominal_world del grasp, psi = 0)")
                  << "\nmax_ang_hold = max_t angle(q_cmd(t), q_hold) of the COMMANDED law; trk = max_t angle(FK(q(t)), q_cmd(t)).\n";

        std::filesystem::create_directories(args.out);
        std::ofstream sum(args.out + "/summary.csv");
        if (!sum) throw std::runtime_error("cannot write summary.csv in " + args.out);
        sum << "variant,law,delta_deg,ik_fail_t_s,ik_fail_joint,max_limit_violation_rad,min_margin_rad,min_margin_joint,min_margin_t_s,"
               "min_w_hat,min_w_hat_t_s,max_qdot_rad_s,max_qdot_joint,max_qdot_ratio,max_angle_from_hold_deg,psi_at_tau_deg,"
               "psi_swing_at_tau_deg,max_ori_track_deg,viol_margin,viol_w_hat,viol_qdot,ik_fail\n";

        const RobotModel::JointVector lo = robot->jointLowerLimits();
        const RobotModel::JointVector hi = robot->jointUpperLimits();

        for (const Variant& v : variants()) {
            const double delta = v.sign * delta_rad;
            const Eigen::Quaterniond q_goal = variantGoal(q_hold, q_goal_nom, args.axis, delta);
            const Eigen::Quaterniond ref_v = variantGoal(q_hold, psi_ref, args.axis, delta);
            const auto laws = buildLawProfiles(grid, tau, q_hold, q_goal, qdmp_file, Q, q_demo0);

            std::vector<CaseResult> rows;
            for (size_t l = 0; l < 3; ++l) {
                std::vector<CartesianSample> traj(static_cast<size_t>(n));
                for (int i = 0; i < n; ++i) {
                    traj[static_cast<size_t>(i)].position = pos[static_cast<size_t>(i)];
                    traj[static_cast<size_t>(i)].orientation = laws[l][static_cast<size_t>(i) + 1];
                }
                JointPathSimulator::Params sp;
                sp.dt = sdt;
                sp.strategy = std::make_shared<NullspaceBiasedResolution>(ns_params);
                const JointPathSimulator sim(robot, sp);
                const auto steps = sim.simulate(traj, q0);  // steps[i].q is the configuration at t = i*sdt

                CaseResult r;
                r.variant = v.name;
                r.law = kLaws[l];
                r.delta_deg = delta * kRad2Deg;
                const size_t ns = steps.size();
                std::vector<double> w_hat(ns), margin(ns), ang(ns);
                for (size_t i = 0; i < ns; ++i) {
                    const double t = static_cast<double>(i) * sdt;
                    const auto& q = steps[i].q;
                    // fail / violation
                    double viol = 0.0;
                    int vj = 0;
                    for (int j = 0; j < 7; ++j) {
                        const double e = std::max(lo(j) - q(j), q(j) - hi(j));
                        if (e > viol) { viol = e; vj = j + 1; }
                    }
                    r.max_viol_rad = std::max(r.max_viol_rad, viol);
                    if (r.fail_t < 0.0 && (!q.allFinite() || viol > kViolTolRad)) {
                        r.fail_t = t;
                        r.fail_joint = vj;
                    }
                    const auto m = satellite_grasp_planner::core::jointMargins(*robot, q);
                    margin[i] = m.min_margin_rad;
                    if (m.min_margin_rad < r.min_margin) {
                        r.min_margin = m.min_margin_rad;
                        r.min_margin_joint = m.worst_joint + 1;
                        r.min_margin_t = t;
                    }
                    w_hat[i] = steps[i].w_trans / w_demo;
                    if (w_hat[i] < r.min_w_hat) { r.min_w_hat = w_hat[i]; r.min_w_hat_t = t; }
                    if (i + 1 < ns) {
                        for (int j = 0; j < 7; ++j) {
                            const double qd = std::fabs(steps[i + 1].q(j) - q(j)) / sdt;
                            if (qd > r.max_qdot) { r.max_qdot = qd; r.max_qdot_joint = j + 1; }
                        }
                    }
                    ang[i] = go::quatAngleRad(laws[l][i], q_hold) * kRad2Deg;
                    robot->update(q, RobotModel::JointVector::Zero());
                    r.max_ori_track_deg = std::max(r.max_ori_track_deg, go::quatAngleRad(robot->eeOrientation(), laws[l][i]) * kRad2Deg);
                }
                for (const auto& q : laws[l]) r.max_angle_hold_deg = std::max(r.max_angle_hold_deg, go::quatAngleRad(q, q_hold) * kRad2Deg);
                r.max_qdot_ratio = r.max_qdot / qdot_limit;
                if (have_psi) psiSwingDeg(ref_v, laws[l].back(), r.psi_deg, r.swing_deg);
                rows.push_back(r);

                std::ofstream pf(args.out + "/profile_" + kLaws[l] + "_" + v.name + ".csv");
                if (!pf) throw std::runtime_error(std::string("cannot write profile for ") + kLaws[l] + "_" + v.name);
                pf << "t_s,w_hat,min_margin_rad,angle_from_hold_deg,q1,q2,q3,q4,q5,q6,q7\n";
                for (size_t i = 0; i < ns; i += static_cast<size_t>(stride)) {
                    pf << num(static_cast<double>(i) * sdt) << ',' << num(w_hat[i]) << ',' << num(margin[i]) << ',' << num(ang[i]);
                    for (int j = 0; j < 7; ++j) pf << ',' << num(steps[i].q(j));
                    pf << '\n';
                }
            }

            std::cout << "\n== " << v.name << "  (phase offset = " << delta * kRad2Deg << " deg, goal xyzw=[" << q_goal.x() << ' '
                      << q_goal.y() << ' ' << q_goal.z() << ' ' << q_goal.w() << "], angle hold->goal = "
                      << go::quatAngleRad(q_hold, q_goal) * kRad2Deg << " deg)\n"
                      << std::left << std::setw(9) << "law" << std::right << std::setw(10) << "IKfail[s]" << std::setw(18)
                      << "min_margin[rad]@j" << std::setw(16) << "min_w_hat@t" << std::setw(20) << "max_qdot[rad/s]@j" << std::setw(12)
                      << "ang_hold[d]" << std::setw(11) << "psi@tau[d]" << std::setw(10) << "trk[d]" << '\n';
            for (const CaseResult& r : rows) {
                const bool vm = r.min_margin < margin_min, vw = r.min_w_hat < w_hat_min, vq = r.max_qdot > qdot_limit,
                           vf = r.fail_t >= 0.0;
                const std::string fail = vf ? cell(r.fail_t, 3, true) + "(j" + std::to_string(r.fail_joint) + ")" : "mai";
                const std::string psi = have_psi ? cell(r.psi_deg, 2, false) : "n/a";
                std::cout << std::left << std::setw(9) << r.law << std::right << std::setw(10) << fail << std::setw(18)
                          << (cell(r.min_margin, 3, vm) + "@j" + std::to_string(r.min_margin_joint)) << std::setw(16)
                          << (cell(r.min_w_hat, 3, vw) + "@" + cell(r.min_w_hat_t, 2, false)) << std::setw(20)
                          << (cell(r.max_qdot, 3, vq) + "@j" + std::to_string(r.max_qdot_joint)) << std::setw(12)
                          << cell(r.max_angle_hold_deg, 2, false) << std::setw(11) << psi << std::setw(10)
                          << cell(r.max_ori_track_deg, 2, false) << '\n';
                sum << r.variant << ',' << r.law << ',' << num(r.delta_deg) << ',' << (vf ? num(r.fail_t) : "never") << ','
                    << r.fail_joint << ',' << num(r.max_viol_rad) << ',' << num(r.min_margin) << ',' << r.min_margin_joint << ','
                    << num(r.min_margin_t) << ',' << num(r.min_w_hat) << ',' << num(r.min_w_hat_t) << ',' << num(r.max_qdot) << ','
                    << r.max_qdot_joint << ',' << num(r.max_qdot_ratio) << ',' << num(r.max_angle_hold_deg) << ',' << num(r.psi_deg)
                    << ',' << num(r.swing_deg) << ',' << num(r.max_ori_track_deg) << ',' << vm << ',' << vw << ',' << vq << ',' << vf << '\n';
            }
        }
        std::cout << "\nwrote " << args.out << "/summary.csv and profile_<law>_<variant>.csv (every " << args.dt << " s)\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "orientation_path_check: " << e.what() << "\n";
        return 1;
    }
}
