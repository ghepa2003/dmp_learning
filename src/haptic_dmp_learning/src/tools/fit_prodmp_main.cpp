// -----------------------------------------------------------------------------
// fit_prodmp - standalone offline ProDMP fitter (NO ROS)
//
// Fits an integral-form core::ProDMP model straight from a demonstration CSV
// that was already recorded, without spinning up a new teleoperation session:
// the raw demo CSV already carries everything ProDMP::learnFromDemonstration
// needs (time-stamped Cartesian samples), so none of the recording nodes have
// to be touched - they stay tied to the classic core::DMP for their primary
// output.
//
// usage: fit_prodmp <demo_raw.csv> <output_prodmp_weights.yaml> [num_basis=20] [--features <path>]
//          [--satellite-center x y z --satellite-axis x y z --satellite-omega-deg-s w
//           --grasp-point kPxx --phase-at-contact-deg th]      (all five or none)
//
// After the weights, also writes <stem>_demo_params.yaml next to them (core::demo_params, see
// deriveDemoParamsPath): tau, contact, displacement, weights hash, the fit configuration actually
// used and, if given, the satellite state at the demo. The joint-states CSV derived from the demo
// CSV path (deriveJointStatesCsvPath) is used for start/end joints when it exists, else null.
//
//   reads   with core::demo_csv_io::readDemoCsv
//   fits    with core::ProDMP::learnFromDemonstration   (single joint LS solve)
//   writes  with core::prodmp_io::saveProDmpToYaml
//
// num_basis / ridge_lambda / the position filter / fix_goal_to_demo_endpoint
// default from config/prodmp_features.yaml (core::prodmp_io::loadProDmpFeatureConfig(),
// same fail-loud search chain as dmp_io::applyFeatureConfig). --features
// overrides which file is loaded; omit it to use the fallback chain.
//
// Prints tau, learn_residual_rms and dropped_non_monotonic_samples to stdout so
// the fit can be sanity-checked without opening the YAML. Fails loud (non-zero
// exit, clear message on stderr) on a bad path or a too-short demo - ProDMP
// already throws for the latter, this just catches and reports it.
// -----------------------------------------------------------------------------

#include <cstdlib>
#include <cmath>
#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "haptic_dmp_learning/core/demo_csv_io.hpp"
#include "haptic_dmp_learning/core/demo_params.hpp"
#include "haptic_dmp_learning/core/joint_states_csv_io.hpp"
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/prodmp_io.hpp"

namespace {
constexpr int kUsageError = 2;   ///< bad command line
constexpr int kRuntimeError = 1;  ///< read / fit / write failure
constexpr int kDemoParamsError = 3;  ///< weights saved, but demo_params could not be written
}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: fit_prodmp <demo_raw.csv> <output_prodmp_weights.yaml> "
                     "[num_basis=20] [--features <path>]\n";
        return kUsageError;
    }

    const std::string demo_csv_path = argv[1];
    const std::string output_yaml_path = argv[2];

    // num_basis and --features are both optional and order-independent among
    // themselves (demo.csv, output.yaml stay strictly positional, in order).
    bool num_basis_from_cli = false;
    int cli_num_basis = 20;
    std::string features_path;  // empty => prodmp_io's own fallback chain resolves it

    // Optional satellite-at-demo description: all five options or none.
    bool have_center = false, have_axis = false, have_omega = false, have_gp = false, have_phase = false;
    Eigen::Vector3d sat_center = Eigen::Vector3d::Zero(), sat_axis = Eigen::Vector3d::Zero();
    double sat_omega_deg_s = 0.0, sat_phase_deg = 0.0;
    std::string sat_gp;
    auto nextDouble = [&](int& i, const char* opt) {
        if (i + 1 >= argc) throw std::invalid_argument(std::string(opt) + " requires a value");
        try {
            return std::stod(argv[++i]);
        } catch (const std::exception&) {
            throw std::invalid_argument(std::string("invalid number for ") + opt);
        }
    };

    for (int i = 3; i < argc; ++i) {
        const std::string arg = argv[i];
        try {
            if (arg == "--satellite-center") {
                for (int c = 0; c < 3; ++c) sat_center(c) = nextDouble(i, "--satellite-center");
                have_center = true;
                continue;
            }
            if (arg == "--satellite-axis") {
                for (int c = 0; c < 3; ++c) sat_axis(c) = nextDouble(i, "--satellite-axis");
                have_axis = true;
                continue;
            }
            if (arg == "--satellite-omega-deg-s") {
                sat_omega_deg_s = nextDouble(i, "--satellite-omega-deg-s");
                have_omega = true;
                continue;
            }
            if (arg == "--phase-at-contact-deg") {
                sat_phase_deg = nextDouble(i, "--phase-at-contact-deg");
                have_phase = true;
                continue;
            }
            if (arg == "--grasp-point") {
                if (i + 1 >= argc) throw std::invalid_argument("--grasp-point requires a value");
                sat_gp = argv[++i];
                have_gp = true;
                continue;
            }
        } catch (const std::exception& e) {
            std::cerr << "fit_prodmp: " << e.what() << "\n";
            return kUsageError;
        }
        if (arg == "--features") {
            if (i + 1 >= argc) {
                std::cerr << "fit_prodmp: --features requires a path argument\n";
                return kUsageError;
            }
            features_path = argv[++i];
        } else {
            if (num_basis_from_cli) {
                std::cerr << "fit_prodmp: unexpected extra argument '" << arg << "'\n";
                return kUsageError;
            }
            try {
                cli_num_basis = std::stoi(arg);
            } catch (const std::exception& e) {
                std::cerr << "fit_prodmp: invalid num_basis '" << arg << "': " << e.what() << "\n";
                return kUsageError;
            }
            if (cli_num_basis < 2) {
                std::cerr << "fit_prodmp: num_basis must be >= 2 (got " << cli_num_basis << ")\n";
                return kUsageError;
            }
            num_basis_from_cli = true;
        }
    }

    const int n_sat = have_center + have_axis + have_omega + have_gp + have_phase;
    if (n_sat != 0 && n_sat != 5) {
        std::cerr << "fit_prodmp: the satellite options must be given ALL together (--satellite-center, "
                     "--satellite-axis, --satellite-omega-deg-s, --grasp-point, --phase-at-contact-deg); "
                     "got only "
                  << n_sat << " of 5\n";
        return kUsageError;
    }

    try {
        // Precedence rule: a positional CLI num_basis always overrides the
        // prodmp_features.yaml default (the file supplies defaults; the CLI is
        // an explicit override when the caller provides one). The same rule
        // would apply to any future CLI override added here.
        const haptic_dmp_learning::core::prodmp_io::ProDmpFeatureConfig features_cfg =
            haptic_dmp_learning::core::prodmp_io::loadProDmpFeatureConfig(features_path);
        const int num_basis = num_basis_from_cli ? cli_num_basis : features_cfg.num_basis;

        const std::vector<haptic_dmp_learning::core::Sample> demo =
            haptic_dmp_learning::core::demo_csv_io::readDemoCsv(demo_csv_path);

        // ridge_lambda is constructor-only on ProDMP (no post-construction
        // setter - see core/prodmp.hpp), so it must be supplied here rather
        // than through applyProDmpFeatureConfig() below.
        haptic_dmp_learning::core::ProDMP prodmp(num_basis, /*alpha=*/25.0, /*alpha_x=*/4.6,
                                                 features_cfg.ridge_lambda);
        haptic_dmp_learning::core::prodmp_io::applyProDmpFeatureConfig(features_path, prodmp);
        prodmp.learnFromDemonstration(demo);

        haptic_dmp_learning::core::prodmp_io::saveProDmpToYaml(prodmp, output_yaml_path);

        const auto& diag = prodmp.diagnostics();
        std::cout << "fit_prodmp: wrote " << output_yaml_path << "\n"
                  << "  samples read           : " << demo.size() << "\n"
                  << "  num_basis              : " << prodmp.numBasis() << "\n"
                  << "  ridge_lambda           : " << prodmp.ridgeLambda() << "\n"
                  << "  position_filter_window : " << prodmp.positionFilterWindow() << "\n"
                  << "  fix_goal_to_demo_endpoint : "
                  << (prodmp.fixGoalToDemoEndpoint() ? "true" : "false") << "\n"
                  << "  tau [s]                : " << prodmp.tau() << "\n"
                  << "  learn_residual_rms     : " << diag.learn_residual_rms << "\n"
                  << "  dropped_non_monotonic  : " << diag.dropped_non_monotonic_samples << "\n";
        // Demo-parameters file next to the weights (the weights file itself is untouched). Weights
        // are already saved and stay valid whatever happens here:
        //  - non-strictly-increasing demo timestamps (the fit drops them silently): WARNING, no
        //    file, exit code 0; any other error: ERROR and exit code 3.
        namespace dp = haptic_dmp_learning::core::demo_params;
        try {
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

            dp::DemoParams dparams;
            const std::string dparams_path = dp::writeForWeights(demo_csv_path, demo, fit_info, output_yaml_path,
                                                                 satellite, std::cout, &dparams);
            std::cout << "fit_prodmp: wrote " << dparams_path << "\n"
                      << "  tau_s                  : " << dparams.tau_s << "\n"
                      << "  t_contact_s            : ";
            if (dparams.t_contact_s) std::cout << *dparams.t_contact_s; else std::cout << "null (no trigger)";
            std::cout << "\n  delta_g_demo_m         : " << dparams.delta_g_demo_m.transpose() << "\n"
                      << "  contact_to_end_offset_m: " << dparams.contact_to_end_offset_m.transpose() << "\n"
                      << "  weights_file / sha256  : " << dparams.weights_file << " / " << dparams.weights_sha256 << "\n"
                      << "  joints start/end       : " << (dparams.start_joints_rad ? "present" : "null") << " / "
                      << (dparams.end_joints_rad ? "present" : "null") << "\n"
                      << "  satellite_at_demo      : " << (dparams.satellite_at_demo ? "present" : "null") << "\n";
        } catch (const dp::NonIncreasingTimestampsError&) {
            std::cerr << "WARNING: demo_params NON scritto (timestamp ripetuti)\n";
        } catch (const std::exception& e) {
            std::cerr << "ERROR: demo_params non scritto: " << e.what() << "\n";
            return kDemoParamsError;
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "fit_prodmp: FAILED: " << e.what() << "\n";
        return kRuntimeError;
    }
}
