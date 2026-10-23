#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <chrono>
#include <cmath>
#include <memory>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <set>

#include "franka_cartesian_control/core/robot_model.hpp"
#include "haptic_dmp_learning/core/cube_satellite_model.hpp"
#include "haptic_dmp_learning/core/tcp_roll.hpp"
#include "satellite_grasp_planner/core/approach_clearance.hpp"
#include "satellite_grasp_planner/core/follow_margin.hpp"
#include "satellite_grasp_planner/core/grasp_pose_eval.hpp"
#include "satellite_grasp_planner/core/grasp_s3_selection.hpp"
#include "satellite_grasp_planner/core/kinematic_feasibility.hpp"
#include "satellite_grasp_planner/core/launch_plan.hpp"
#include "satellite_grasp_planner/core/selection_inputs.hpp"
#include "satellite_grasp_planner/core/start_state.hpp"

using namespace satellite_grasp_planner::core;
using haptic_dmp_learning::core::CubeSatelliteModel;
using haptic_dmp_learning::core::GraspPointId;
using franka_cartesian_control::core::RobotModel;

struct CellInput {
    std::string cell;
    double omega_deg_s = 0.0;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double alpha_deg = 0.0;
    double phi_deg = 0.0;
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    Eigen::Quaterniond q_ref = Eigen::Quaterniond::Identity();
    Eigen::Vector3d normal = Eigen::Vector3d::UnitX();
};

std::vector<CellInput> loadCellInputs(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open input CSV: " + path);
    }
    std::vector<CellInput> cells;
    std::string line;
    std::getline(file, line); // header

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;
        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }
        if (tokens.size() < 17) {
            throw std::runtime_error("Malformed CSV line in " + path + ": " + line);
        }
        CellInput ci;
        ci.cell = tokens[0];
        ci.omega_deg_s = std::stod(tokens[1]);
        ci.center = Eigen::Vector3d(std::stod(tokens[2]), std::stod(tokens[3]), std::stod(tokens[4]));
        ci.alpha_deg = std::stod(tokens[5]);
        ci.phi_deg = std::stod(tokens[6]);
        ci.axis = Eigen::Vector3d(std::stod(tokens[7]), std::stod(tokens[8]), std::stod(tokens[9]));
        // qx, qy, qz, qw
        ci.q_ref = Eigen::Quaterniond(std::stod(tokens[13]), std::stod(tokens[10]), std::stod(tokens[11]), std::stod(tokens[12]));
        ci.normal = Eigen::Vector3d(std::stod(tokens[14]), std::stod(tokens[15]), std::stod(tokens[16]));
        cells.push_back(ci);
    }
    return cells;
}

int main(int argc, char** argv) {
    std::string input_csv = "tests/golden/g25/g25_cells_input.csv";
    std::string output_csv = "_gate_out/g25_cells_output.csv";
    double snap_theta_rad = -1.6727670737402633;
    double snap_t_s = 18.318;
    std::string cell_filter_str = "";
    bool write_csv = true;
    bool full_precision = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--input" && i + 1 < argc) {
            input_csv = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            output_csv = argv[++i];
        } else if (arg == "--snap-theta-rad" && i + 1 < argc) {
            snap_theta_rad = std::stod(argv[++i]);
        } else if (arg == "--snap-t-s" && i + 1 < argc) {
            snap_t_s = std::stod(argv[++i]);
        } else if (arg == "--cell" && i + 1 < argc) {
            cell_filter_str = argv[++i];
        } else if (arg == "--no-csv") {
            write_csv = false;
        } else if (arg == "--full-precision") {
            full_precision = true;
        }
    }

    std::set<std::string> filter_cells;
    if (!cell_filter_str.empty()) {
        std::stringstream ss(cell_filter_str);
        std::string item;
        while (std::getline(ss, item, ',')) {
            if (!item.empty()) filter_cells.insert(item);
        }
    }

    const std::string weights_dir = "tests/golden/g25";
    const std::string demo_params = weights_dir + "/weights_demo_params.yaml";
    const std::string urdf = "fer_flat_effort.urdf";

    // Setup Start State
    std::vector<double> q_start_vec = {-9e-06, -0.786572, 2e-06, -2.378914, -6e-06, 3.141609, 0.785398};
    StartState start = makeStartStateFromOverride(q_start_vec, urdf);

    // Hand mesh for clearance
    HandMesh hand_mesh = loadHandCollisionMesh(urdf);

    // Frozen S3Params (G25b)
    S3Params s3;
    s3.mode = "v2";
    s3.branch_rule = "rotation";
    s3.use_w = true;
    s3.use_clearance = true;
    s3.use_margin = true;
    s3.min_clearance_mm = 2.0;
    s3.min_margin_deg = 28.0;
    s3.finalists_n = 40;
    s3.posthoc_cluster_deg = 10.0;
    s3.theta_step_deg = 2.0;
    s3.theta_refine_step_deg = 0.1;
    s3.kappa_w_mm = 7.5;
    s3.kappa_t_mm_s = 0.0;
    s3.tau_j_mm = 2.0;
    s3.rollout_q_ref = {0.0, -0.785398163397, 0.0, -2.356194490192, 0.0, 1.570796326795, 0.785398163397};
    s3.clearance_dt_s = 0.05;
    s3.gripper_open_m = 0.060;
    s3.margin_follow_s = 3.0;
    s3.margin_follow_dt_s = 0.01;
    s3.posthoc_velocity_check = false;

    // Stamp parametri effettivi all'inizio
    std::cout << "====================================================================================================" << std::endl;
    std::cout << "G25b OFFLINE SELECTION DRIVER - PROFILO CONGELATO" << std::endl;
    std::cout << "====================================================================================================" << std::endl;
    std::cout << "Input CSV:              " << input_csv << std::endl;
    std::cout << "Output CSV:             " << (write_csv ? output_csv : "(disabilitato)") << std::endl;
    std::cout << "Snapshot theta_rad:     " << std::fixed << std::setprecision(16) << snap_theta_rad << std::endl;
    std::cout << "Snapshot t_s:           " << std::fixed << std::setprecision(4) << snap_t_s << " s" << std::endl;
    std::cout << "S3Params.mode:          " << s3.mode << std::endl;
    std::cout << "S3Params.branch_rule:   " << s3.branch_rule << std::endl;
    std::cout << "S3Params.use_w:         " << (s3.use_w ? "true" : "false") << std::endl;
    std::cout << "S3Params.use_clearance: " << (s3.use_clearance ? "true" : "false") << std::endl;
    std::cout << "S3Params.use_margin:    " << (s3.use_margin ? "true" : "false") << std::endl;
    std::cout << "S3Params.min_clr_mm:    " << s3.min_clearance_mm << " mm" << std::endl;
    std::cout << "S3Params.min_margin_deg:" << s3.min_margin_deg << " deg" << std::endl;
    std::cout << "S3Params.finalists_n:   " << s3.finalists_n << std::endl;
    std::cout << "S3Params.cluster_deg:   " << s3.posthoc_cluster_deg << " deg" << std::endl;
    std::cout << "S3Params.theta_step_deg:" << s3.theta_step_deg << " deg (refine " << s3.theta_refine_step_deg << " deg)" << std::endl;
    std::cout << "S3Params.kappa_w_mm:    " << s3.kappa_w_mm << " mm" << std::endl;
    std::cout << "S3Params.kappa_t_mm_s:  " << s3.kappa_t_mm_s << " mm/s" << std::endl;
    std::cout << "S3Params.tau_j_mm:      " << s3.tau_j_mm << " mm" << std::endl;
    std::cout << "S3Params.rollout_q_ref: [";
    for (size_t i = 0; i < s3.rollout_q_ref.size(); ++i) {
        std::cout << s3.rollout_q_ref[i] << (i + 1 < s3.rollout_q_ref.size() ? ", " : "");
    }
    std::cout << "]" << std::endl;
    std::cout << "====================================================================================================" << std::endl;

    std::vector<CellInput> all_cells = loadCellInputs(input_csv);
    std::vector<CellInput> cells;
    for (const auto& ci : all_cells) {
        if (filter_cells.empty() || filter_cells.count(ci.cell)) {
            cells.push_back(ci);
        }
    }

    std::ofstream out_file;
    if (write_csv) {
        out_file.open(output_csv);
        if (!out_file.is_open()) {
            throw std::runtime_error("Cannot open output CSV for writing: " + output_csv);
        }
        out_file << "cell,status,k,branch,theta_deg,d_mm,w,J,pred_margin_deg,pred_clearance_mm,"
                 << "finalists_examined,discarded_margin,discarded_clearance,discarded_velocity,discarded_rollout,"
                 << "selection_time_s,prevista_riuscita\n";
    }

    std::cout << std::left
              << std::setw(19) << "CELL" << " | "
              << std::setw(6)  << "OMEGA" << " | "
              << std::setw(12) << "STATUS" << " | "
              << std::setw(6)  << "CAND" << " | "
              << std::setw(9)  << "THETA*" << " | "
              << std::setw(8)  << "d [mm]" << " | "
              << std::setw(8)  << "w" << " | "
              << std::setw(8)  << "J" << " | "
              << std::setw(10) << "MARG [deg]" << " | "
              << std::setw(9)  << "CLR [mm]" << " | "
              << std::setw(4)  << "EXAM" << " | "
              << std::setw(4)  << "D_MG" << " | "
              << std::setw(4)  << "D_CL" << " | "
              << std::setw(4)  << "D_RO" << " | "
              << std::setw(8)  << "TEMPO[s]" << " | "
              << "PREVISIONE" << std::endl;
    std::cout << std::string(140, '-') << std::endl;

    for (const auto& ci : cells) {
        // Build snapshot exactly as grasp_planner_node
        SatelliteSnapshot snap;
        snap.center = ci.center;
        snap.axis = ci.axis.normalized();
        snap.omega_rad_s = ci.omega_deg_s * M_PI / 180.0;
        snap.theta_rad = snap_theta_rad;
        snap.t_s = snap_t_s;

        CubeSatelliteModel::Params cp;
        cp.center_world = snap.center;
        cp.axis_world = snap.axis;
        cp.face_normal_body = ci.normal.normalized();
        cp.cube_side_m = 0.150;
        cp.standoff_m = 0.08;
        cp.collar_radius_m = 0.0725;
        cp.wall_thickness_m = 0.009;
        cp.grasp_radius_m = 0.068;
        cp.grasp_depth_m = 0.0125;

        // Create ApproachClearanceModel for this cell
        ApproachClearanceModel clearance_model(cp, hand_mesh);

        SelectionInputs in = buildSelectionInputs(demo_params, urdf, snap, cp, start);
        CubeSatelliteModel model(in.cube_params);
        RobotModel& robot = *in.robot;
        robot.update(in.q0, RobotModel::JointVector::Zero());

        DemoReference demo;
        demo.position = in.p0 + in.delta_g_demo;
        demo.orientation = robot.eeOrientation();
        demo.w_trans_demo = in.w_trans_demo;

        S3Context ctx;
        ctx.prodmp_template = &in.prodmp_template;
        ctx.contact_to_end_offset = in.contact_to_end_offset;
        ctx.tau_contact_s = in.tau_contact_s;
        ctx.tau_launch_s = in.tau_contact_s;
        ctx.snapshot = snap;
        ctx.t_now_s = snap.t_s;
        ctx.min_delay_s = 5.0;
        ctx.clearance_eval = makeClearanceEvaluator(clearance_model, ClearanceParams{s3.clearance_dt_s, s3.gripper_open_m});

        robot.update(in.q0, RobotModel::JointVector::Zero());
        auto t0 = std::chrono::steady_clock::now();
        S3Result res = selectGraspS3(model, robot, in.q0, demo, in.delta_g_demo, ctx, s3);
        auto t1 = std::chrono::steady_clock::now();
        double elapsed_s = std::chrono::duration<double>(t1 - t0).count();

        // Count discards
        int examined = 0;
        int disc_margin = 0;
        int disc_clearance = 0;
        int disc_velocity = 0;
        int disc_rollout = 0;

        for (const auto& f : res.v2_finalists) {
            if (f.rollout_done) examined++;
            if (f.discard_reason == "margin") disc_margin++;
            else if (f.discard_reason == "clearance") disc_clearance++;
            else if (f.discard_reason == "velocity") disc_velocity++;
            else if (f.discard_reason == "rollout") disc_rollout++;
        }

        std::string status_str = (res.status == S3Result::kSelected) ? "kSelected" : "kNoCandidate";
        std::string k_str = "n/a";
        std::string branch_str = "n/a";
        std::string theta_str = "n/a";
        std::string d_str = "n/a";
        std::string w_str = "n/a";
        std::string j_str = "n/a";
        std::string margin_str = "n/a";
        std::string clr_str = "n/a";
        std::string cand_str = "NONE";
        std::string previsione = "RIFIUTO";

        double chosen_th = 0.0;
        double chosen_d = 0.0;
        double chosen_w = 0.0;
        double chosen_j = 0.0;
        double chosen_margin = 0.0;
        double chosen_clr = 0.0;

        if (res.status == S3Result::kSelected) {
            for (const auto& f : res.v2_finalists) {
                if (f.winner) {
                    int chosen_k = static_cast<int>(f.cand.eval.k);
                    std::string chosen_b = (f.cand.eval.wrist_flip ? "B" : "A");
                    chosen_th = f.cand.eval.theta_rad * 180.0 / M_PI;
                    chosen_d = f.d_mm;
                    chosen_w = f.w_rollout;
                    chosen_j = f.j;
                    chosen_margin = f.margin_deg;
                    chosen_clr = f.clearance_mm;

                    k_str = std::to_string(chosen_k);
                    branch_str = chosen_b;
                    cand_str = "k" + k_str + branch_str;

                    {
                        std::ostringstream ss;
                        ss << std::fixed << std::setprecision(1) << chosen_th;
                        theta_str = ss.str();
                    }
                    {
                        std::ostringstream ss;
                        ss << std::fixed << std::setprecision(3) << chosen_d;
                        d_str = ss.str();
                    }
                    {
                        std::ostringstream ss;
                        ss << std::fixed << std::setprecision(5) << chosen_w;
                        w_str = ss.str();
                    }
                    {
                        std::ostringstream ss;
                        ss << std::fixed << std::setprecision(3) << chosen_j;
                        j_str = ss.str();
                    }
                    {
                        std::ostringstream ss;
                        ss << std::fixed << std::setprecision(2) << chosen_margin;
                        margin_str = ss.str();
                    }
                    {
                        std::ostringstream ss;
                        ss << std::fixed << std::setprecision(2) << chosen_clr;
                        clr_str = ss.str();
                    }

                    if (chosen_margin >= 28.0 && chosen_clr >= 2.0) {
                        previsione = "PREVISTA_RIUSCITA";
                    } else {
                        previsione = "RIFIUTO";
                    }
                    break;
                }
            }
        }

        // Print stdout row
        std::cout << std::left
                  << std::setw(19) << ci.cell << " | "
                  << std::right
                  << std::fixed << std::setprecision(2)
                  << std::setw(6)  << ci.omega_deg_s << " | "
                  << std::left
                  << std::setw(12) << status_str << " | "
                  << std::setw(6)  << cand_str << " | "
                  << std::right
                  << std::setw(9)  << theta_str << " | "
                  << std::setw(8)  << d_str << " | "
                  << std::setw(8)  << w_str << " | "
                  << std::setw(8)  << j_str << " | "
                  << std::setw(10) << margin_str << " | "
                  << std::setw(9)  << clr_str << " | "
                  << std::setw(4)  << examined << " | "
                  << std::setw(4)  << disc_margin << " | "
                  << std::setw(4)  << disc_clearance << " | "
                  << std::setw(4)  << disc_rollout << " | "
                  << std::fixed << std::setprecision(3)
                  << std::setw(8)  << elapsed_s << " | "
                  << std::left << previsione << std::endl;

        if (write_csv) {
            if (full_precision) {
                out_file << ci.cell << ","
                         << status_str << ","
                         << k_str << ","
                         << branch_str << ",";
                if (res.status == S3Result::kSelected) {
                    out_file << std::defaultfloat << std::setprecision(17)
                             << chosen_th << ","
                             << chosen_d << ","
                             << chosen_w << ","
                             << chosen_j << ","
                             << chosen_margin << ","
                             << chosen_clr << ",";
                } else {
                    out_file << "n/a,n/a,n/a,n/a,n/a,n/a,";
                }
                out_file << examined << ","
                         << disc_margin << ","
                         << disc_clearance << ","
                         << disc_velocity << ","
                         << disc_rollout << ","
                         << std::defaultfloat << std::setprecision(17) << elapsed_s << ","
                         << previsione << "\n";
            } else {
                out_file << ci.cell << ","
                         << status_str << ","
                         << k_str << ","
                         << branch_str << ","
                         << theta_str << ","
                         << d_str << ","
                         << w_str << ","
                         << j_str << ","
                         << margin_str << ","
                         << clr_str << ","
                         << examined << ","
                         << disc_margin << ","
                         << disc_clearance << ","
                         << disc_velocity << ","
                         << disc_rollout << ","
                         << std::fixed << std::setprecision(4) << elapsed_s << ","
                         << previsione << "\n";
            }
        }
    }

    if (write_csv) {
        out_file.close();
        std::cout << "\nRisultati salvati in: " << output_csv << std::endl;
    }

    return 0;
}
