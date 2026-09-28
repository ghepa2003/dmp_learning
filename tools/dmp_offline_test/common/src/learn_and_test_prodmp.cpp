// Learns a ProDMP (position) + QuaternionDMP (orientation, fixed config) from a demo CSV,
// replays IN-PROCESS, and computes fidelity metrics reusing metrics.cpp/hpp.
//
// Writes the ProDMP weights YAML and replay CSV.
//
// Usage:
//   ./learn_and_test_prodmp <input_demo.csv> <output_weights.yaml>
//       <output_replay.csv> <summary.csv> <label>
//       [n_basis|-] [alpha_x|-] [alpha_z|-] [beta_z|-] [config_path='']
//       [--lambda <val>] [--window <val>] [--fix-goal] [--report-condition-number]
//       [--with-orientation]
//       [--satellite-center x y z --satellite-axis x y z --satellite-omega-deg-s w
//        --grasp-point kPxx --phase-at-contact-deg th]   (all five or none)
//   '-' or omitted argument = use config file / class defaults
//
//   --with-orientation (default OFF): also embeds the already-fitted orientation
//   QuaternionDMP into <output_weights.yaml> as a `quaternion_dmp:` section (unified
//   ProDMP+orientation file, see prodmp_io::saveProDmpToYaml(prodmp, qdmp*, path)).
//   With this flag OFF (default), <output_weights.yaml> is byte-identical to before
//   this flag existed - position-only, no quaternion_dmp section.
//
//   After the weights are saved, also writes <stem>_demo_params.yaml next to <output_weights.yaml>
//   (core::demo_params, deriveDemoParamsPath): tau, contact, displacement, SHA-256 of the weights
//   file just written, the fit configuration actually used and, if the five --satellite-* options
//   are given, the satellite state at the demo. The weights file itself is unchanged.
//   Failure handling (weights are always saved first and stay valid):
//     - demo with non-strictly-increasing timestamps (the fit drops them silently): stderr
//       "WARNING: demo_params NON scritto (timestamp ripetuti)", file not written, exit code 0;
//     - any other demo_params error (write/hash/read-back/alignment, invalid satellite values):
//       stderr "ERROR: demo_params non scritto: <reason>", the rest of the run completes and the
//       tool exits with code 3.

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <stdexcept>
#include <optional>
#include <cstdlib>
#include <cmath>

#include "haptic_dmp_learning/core/demo_params.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "haptic_dmp_learning/core/quaternion_dmp.hpp"
#include "haptic_dmp_learning/core/demo_csv_io.hpp"
#include "haptic_dmp_learning/core/types.hpp"
#include "metrics.hpp"

using haptic_dmp_learning::core::ProDMP;
using haptic_dmp_learning::core::QuaternionDMP;
using haptic_dmp_learning::core::Sample;

// Fixed QuaternionDMP parameters for orientation.
// Orientation parameters are intentionally kept fixed and independent of the ProDMP
// sweep arguments in order to isolate the effect of the ProDMP position configuration alone,
// preventing orientation variations from confounding the position sweep metrics.
// These match the winning baseline configuration found for classic DMP on trajC:
// n_basis=200, ridge regression with lambda=1e-6, velocity filter window=0.20s.
namespace {
constexpr int kQuatDmpNBasis = 200;
constexpr double kQuatDmpAlphaX = 4.6;
constexpr double kQuatDmpAlphaZ = 25.0;
constexpr double kQuatDmpBetaZ = 6.25;
constexpr double kQuatDmpRidgeLambda = 1e-6;
constexpr double kQuatDmpFilterWindowSec = 0.20;
}  // namespace

static void writeReplayCsv(const std::string& path, const std::vector<double>& t,
                           const std::vector<Eigen::Vector3d>& p,
                           const std::vector<Eigen::Quaterniond>& q) {
    std::ofstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("learn_and_test_prodmp: cannot open output replay file: " + path);
    }
    f << "t,x,y,z,qw,qx,qy,qz\n";
    for (size_t k = 0; k < t.size(); ++k) {
        f << t[k] << "," << p[k].x() << "," << p[k].y() << "," << p[k].z() << ","
          << q[k].w() << "," << q[k].x() << "," << q[k].y() << "," << q[k].z() << "\n";
    }
}

int main(int argc, char** argv) {
    if (argc < 6) {
        std::cerr << "Usage: " << argv[0]
                  << " <input_demo.csv> <output_weights.yaml> <output_replay.csv>"
                  << " <summary.csv> <label>"
                  << " [n_basis|-] [alpha_x|-] [alpha_z|-] [beta_z|-]"
                  << " [config_path='']\n"
                  << "  '-' or omitted argument = use config file / class defaults\n";
        return 1;
    }

    const std::string input_csv     = argv[1];
    const std::string output_yaml   = argv[2];
    const std::string output_replay = argv[3];
    const std::string summary_csv   = argv[4];
    const std::string label         = argv[5];

    auto parseOrDefault = [](const char* arg, double fallback) -> double {
        std::string s = arg ? arg : "-";
        return (s == "-") ? fallback : std::stod(s);
    };

    bool cli_n_basis_provided = false;
    int cli_n_basis = 20;
    double alpha_x = 4.6;
    double alpha_z = 25.0;
    double cli_lambda = -1.0;
    double cli_window = -1.0;
    bool report_condition_number = false;
    bool fix_goal_to_demo_endpoint = false;
    bool with_orientation = false;
    std::string config_path = "";

    // Optional satellite-at-demo description: all five options or none.
    bool have_center = false, have_axis = false, have_omega = false, have_gp = false, have_phase = false;
    Eigen::Vector3d sat_center = Eigen::Vector3d::Zero(), sat_axis = Eigen::Vector3d::Zero();
    double sat_omega_deg_s = 0.0, sat_phase_deg = 0.0;
    std::string sat_gp;
    auto nextDouble = [&](int& i, const char* opt) -> double {
        if (i + 1 >= argc) {
            std::cerr << "ERROR: " << opt << " requires a value\n";
            std::exit(1);
        }
        try {
            return std::stod(argv[++i]);
        } catch (const std::exception&) {
            std::cerr << "ERROR: invalid number for " << opt << "\n";
            std::exit(1);
        }
    };

    for (int i = 6; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--satellite-center") {
            for (int c = 0; c < 3; ++c) sat_center(c) = nextDouble(i, "--satellite-center");
            have_center = true;
        } else if (arg == "--satellite-axis") {
            for (int c = 0; c < 3; ++c) sat_axis(c) = nextDouble(i, "--satellite-axis");
            have_axis = true;
        } else if (arg == "--satellite-omega-deg-s") {
            sat_omega_deg_s = nextDouble(i, "--satellite-omega-deg-s");
            have_omega = true;
        } else if (arg == "--phase-at-contact-deg") {
            sat_phase_deg = nextDouble(i, "--phase-at-contact-deg");
            have_phase = true;
        } else if (arg == "--grasp-point") {
            if (i + 1 >= argc) {
                std::cerr << "ERROR: --grasp-point requires a value\n";
                return 1;
            }
            sat_gp = argv[++i];
            have_gp = true;
        } else if (arg == "--lambda" && i + 1 < argc) {
            cli_lambda = std::stod(argv[++i]);
        } else if (arg == "--window" && i + 1 < argc) {
            cli_window = std::stod(argv[++i]);
        } else if (arg == "--report-condition-number") {
            report_condition_number = true;
        } else if (arg == "--fix-goal") {
            fix_goal_to_demo_endpoint = true;
        } else if (arg == "--with-orientation") {
            with_orientation = true;
        } else if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (i == 6 && arg != "-") {
            cli_n_basis = std::stoi(arg);
            cli_n_basis_provided = true;
        } else if (i == 7 && arg != "-") {
            alpha_x = parseOrDefault(arg.c_str(), 4.6);
        } else if (i == 8 && arg != "-") {
            alpha_z = parseOrDefault(arg.c_str(), 25.0);
        } else if (i == 9 && arg != "-") {
            // beta_z
        } else if (i == 10 && arg != "-") {
            config_path = arg;
        }
    }

    const int n_sat = have_center + have_axis + have_omega + have_gp + have_phase;
    if (n_sat != 0 && n_sat != 5) {
        std::cerr << "ERROR: the satellite options must be given ALL together (--satellite-center, "
                     "--satellite-axis, --satellite-omega-deg-s, --grasp-point, --phase-at-contact-deg); "
                     "got only "
                  << n_sat << " of 5\n";
        return 1;
    }
    if (config_path.empty()) {
        const std::string default_path = "../../src/haptic_dmp_learning/config/prodmp_features.yaml";
        std::ifstream check_f(default_path);
        if (check_f.good()) {
            config_path = default_path;
        }
    }

    bool demo_params_failed = false;  // weights saved, demo_params not: exit code 3 at the end

    try {
        std::cout << "[" << label << "] Loading demo from " << input_csv << "...\n";
        std::vector<Sample> demo = haptic_dmp_learning::core::demo_csv_io::readDemoCsv(input_csv);
        if (demo.size() < 5) {
            throw std::runtime_error("learn_and_test_prodmp: demo too short (" +
                                     std::to_string(demo.size()) + " samples)");
        }
        const double duration = demo.back().t - demo.front().t;
        std::cout << "  " << demo.size() << " samples, duration " << duration << "s\n";

        // Load ProDMP feature configuration (YAML schema: num_basis, ridge_lambda, position_filter)
        haptic_dmp_learning::core::prodmp_io::ProDmpFeatureConfig prodmp_cfg;
        if (!config_path.empty()) {
            prodmp_cfg = haptic_dmp_learning::core::prodmp_io::loadProDmpFeatureConfig(config_path);
            std::cout << "  [Config YAML] Loaded " << config_path
                      << " -> num_basis: " << prodmp_cfg.num_basis
                      << " | ridge_lambda: " << prodmp_cfg.ridge_lambda
                      << " | position_filter: " << (prodmp_cfg.position_filter_enabled ? "on" : "off")
                      << " (window: " << prodmp_cfg.position_filter_window_sec << "s)\n";
        } else {
            std::cout << "  [Config YAML] No configuration file specified -> using ProDMP defaults\n";
        }

        // CLI parameters override YAML config if provided
        const int n_basis = cli_n_basis_provided ? cli_n_basis : prodmp_cfg.num_basis;
        const double final_lambda = (cli_lambda >= 0.0) ? cli_lambda : prodmp_cfg.ridge_lambda;
        const double final_window = (cli_window >= 0.0) ? cli_window : (prodmp_cfg.position_filter_enabled ? prodmp_cfg.position_filter_window_sec : 0.0);

        ProDMP prodmp(n_basis, alpha_z, alpha_x, final_lambda);
        if (final_window > 0.0) {
            prodmp.setPositionFilterWindow(final_window);
        } else {
            prodmp.setPositionFilterWindow(0.0);
        }
        if (report_condition_number) {
            prodmp.setComputeConditionNumber(true);
        }
        if (fix_goal_to_demo_endpoint) {
            prodmp.setFixGoalToDemoEndpoint(true);
        }

        // Fixed QuaternionDMP for orientation (isolated from position sweep)
        QuaternionDMP qdmp(kQuatDmpNBasis, kQuatDmpAlphaX, kQuatDmpAlphaZ, kQuatDmpBetaZ);
        qdmp.setRidgeRegression(true, kQuatDmpRidgeLambda);
        qdmp.setVelocityFilter(true, kQuatDmpFilterWindowSec, kQuatDmpFilterWindowSec);

        std::cout << "  Learning ProDMP (n_basis=" << n_basis
                  << ", ridge_lambda=" << prodmp.ridgeLambda()
                  << ", filter_window=" << prodmp.positionFilterWindow() << "s"
                  << ", fix_goal=" << (fix_goal_to_demo_endpoint ? "on" : "off") << ")...\n";
        prodmp.learnFromDemonstration(demo);
        qdmp.learnFromDemonstration(demo);

        std::cout << "  learn_residual_rms: " << prodmp.diagnostics().learn_residual_rms << "\n";
        if (report_condition_number) {
            std::cout << "  cond(H) after column-scale preconditioning: "
                      << prodmp.diagnostics().design_matrix_condition_number << "\n";
        }

        if (with_orientation) {
            haptic_dmp_learning::core::prodmp_io::saveProDmpToYaml(prodmp, &qdmp, output_yaml);
        } else {
            haptic_dmp_learning::core::prodmp_io::saveProDmpToYaml(prodmp, output_yaml);
        }

        // Demo-parameters file next to the weights (the weights file itself is untouched).
        try {
            namespace dp = haptic_dmp_learning::core::demo_params;
            dp::FitInfo fit_info;
            fit_info.num_basis = prodmp.numBasis();
            fit_info.ridge_lambda = prodmp.ridgeLambda();
            fit_info.position_filter_window_s = prodmp.positionFilterWindow();
            fit_info.fix_goal_to_demo_endpoint = prodmp.fixGoalToDemoEndpoint();
            std::optional<dp::SatelliteAtDemo> satellite;
            if (n_sat == 5) {
                dp::SatelliteAtDemo sat;
                sat.center_m = sat_center;
                sat.axis = sat_axis;
                sat.omega_rad_s = sat_omega_deg_s * M_PI / 180.0;
                sat.grasp_point = dp::parseGraspPoint(sat_gp);
                sat.phase_at_contact_rad = sat_phase_deg * M_PI / 180.0;
                satellite = sat;
            }
            dp::writeForWeights(input_csv, demo, fit_info, output_yaml, satellite, std::cout);
        } catch (const haptic_dmp_learning::core::demo_params::NonIncreasingTimestampsError&) {
            std::cerr << "WARNING: demo_params NON scritto (timestamp ripetuti)\n";
        } catch (const std::exception& e) {
            std::cerr << "ERROR: demo_params non scritto: " << e.what() << "\n";
            demo_params_failed = true;
        }

        prodmp.setInitialConditions(prodmp.initTime(), prodmp.initPos(), prodmp.initVel());
        qdmp.reset();

        std::vector<double> t_out;
        std::vector<Eigen::Vector3d> ref_pos, replay_pos;
        std::vector<Eigen::Quaterniond> ref_orient, replay_orient;
        t_out.reserve(demo.size());
        ref_pos.reserve(demo.size());
        replay_pos.reserve(demo.size());
        ref_orient.reserve(demo.size());
        replay_orient.reserve(demo.size());

        double prev_t = demo.front().t;
        for (size_t k = 0; k < demo.size(); ++k) {
            double dt = (k == 0) ? 0.0 : (demo[k].t - prev_t);
            prev_t = demo[k].t;

            Eigen::Vector3d p = (k == 0) ? prodmp.step(0.0) : prodmp.step(dt);
            Eigen::Quaterniond q = (k == 0) ? qdmp.step(0.0) : qdmp.step(dt);

            t_out.push_back(demo[k].t - demo.front().t);
            ref_pos.push_back(demo[k].position);
            ref_orient.push_back(demo[k].orientation);
            replay_pos.push_back(p);
            replay_orient.push_back(q);
        }

        writeReplayCsv(output_replay, t_out, replay_pos, replay_orient);

        using namespace dmp_tools::metrics;

        TrajectoryFidelity tf = computeTrajectoryFidelity(ref_pos, replay_pos);
        OrientationFidelity of = computeOrientationFidelity(ref_orient, replay_orient);
        double endpoint_pos_error = computeEndpointError(replay_pos.back(), ref_pos.back());
        double endpoint_orient_error = computeAngularEndpointError(replay_orient.back(), ref_orient.back());

        printReport(label, tf);
        printReport(label, of);
        printEndpointError(label, endpoint_pos_error, endpoint_orient_error);

        double max_abs_weight = 0.0;
        const auto& w_arr = prodmp.weights();
        for (int d = 0; d < 3; ++d) {
            if (w_arr[d].size() > 0) {
                max_abs_weight = std::max(max_abs_weight, w_arr[d].cwiseAbs().maxCoeff());
            }
        }
        std::cout << "  [" << label << "] Max abs weight: " << max_abs_weight << "\n";

        appendToSummaryCsv(summary_csv, label, tf, of,
                           endpoint_pos_error, endpoint_orient_error);

        std::cout << "  Saved: " << output_yaml << ", " << output_replay << "\n";
        std::cout << "  Summary appended to " << summary_csv << "\n";

    } catch (const std::exception& e) {
        std::cerr << "ERROR [" << label << "]: " << e.what() << "\n";
        return 1;
    }

    return demo_params_failed ? 3 : 0;
}
