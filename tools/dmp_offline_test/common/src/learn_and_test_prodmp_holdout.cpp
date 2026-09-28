// =============================================================================
// MOTIVAZIONE
// Gli sweep precedenti hanno misurato l'errore di replay SULLA STESSA demo usata 
// per il fit (in-sample) - questo non distingue un modello che generalizza bene 
// da uno che sta semplicemente interpolando i dati di training (rischio 
// concreto con n_basis alto e ridge_lambda molto basso, es. n=300/lambda=1e-12, 
// dove il numero di parametri liberi è enorme rispetto al segnale). Serve 
// valutare l'errore su una porzione della demo MAI vista durante il fit.
// =============================================================================

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <stdexcept>
#include <cmath>
#include <algorithm>

#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "haptic_dmp_learning/core/demo_csv_io.hpp"
#include "haptic_dmp_learning/core/types.hpp"
#include "metrics.hpp"

using haptic_dmp_learning::core::ProDMP;
using haptic_dmp_learning::core::Sample;

namespace {

void appendToHoldoutSummaryCsv(const std::string& csv_path,
                               const std::string& label,
                               int n_basis,
                               double ridge_lambda,
                               double window_sec,
                               const std::string& tau_mode,
                               double rmse_in_sample_mm,
                               double rmse_holdout_mm,
                               double max_abs_weight,
                               double cond_number) {
    if (csv_path.empty() || csv_path == "-") return;

    std::ifstream check_file(csv_path);
    bool exists = check_file.good() && (check_file.peek() != std::ifstream::traits_type::eof());
    check_file.close();

    std::ofstream f(csv_path, std::ios::app);
    if (!f.is_open()) {
        throw std::runtime_error("Cannot open summary CSV for writing: " + csv_path);
    }
    if (!exists) {
        f << "label,n_basis,ridge_lambda,window_sec,tau_mode,rmse_in_sample_mm,rmse_holdout_mm,"
             "max_abs_weight,cond_H_scaled\n";
    }
    f << label << ","
      << n_basis << ","
      << ridge_lambda << ","
      << window_sec << ","
      << tau_mode << ","
      << rmse_in_sample_mm << ","
      << rmse_holdout_mm << ","
      << max_abs_weight << ","
      << cond_number << "\n";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 9) {
        std::cerr << "Usage: " << argv[0]
                  << " <demo.csv> <output_weights.yaml> <summary.csv> <label>"
                  << " <n_basis> <ridge_lambda> <window_sec> <holdout_fraction>"
                  << " [--tau-mode train|full] [--report-condition-number]\n";
        return 1;
    }

    const std::string input_csv        = argv[1];
    const std::string output_yaml      = argv[2];
    const std::string summary_csv      = argv[3];
    const std::string label            = argv[4];
    const int         n_basis          = std::stoi(argv[5]);
    const double      ridge_lambda     = std::stod(argv[6]);
    const double      window_sec       = std::stod(argv[7]);
    const double      holdout_fraction = std::stod(argv[8]);

    // --------------------------------------------------------------------------
    // tau semantics of the hold-out fit (see STEP 1 of the holdout study)
    // --------------------------------------------------------------------------
    // ProDMP::learnFromDemonstration() sets tau_ = (last - first timestamp of
    // the demo it is handed). Here the demo handed to it is the TRAINING slice
    // only (first (1 - holdout_fraction) of the samples, by index), so by
    // default tau_ = duration of the training slice, NOT of the full movement.
    //
    //   --tau-mode train  (DEFAULT, = the behaviour every previous grid used):
    //       tau_ = duration of the 80% training slice. Phase s runs 0 -> 1
    //       across the training slice; the held-out 20% is then rolled out at
    //       s > 1, i.e. PAST the natural end of the primitive. The canonical
    //       phase x(s) = exp(-alpha_x s) has already collapsed to ~0 there, so
    //       the forcing term is dead and the model can only sit on the goal
    //       attractor - it is asked to invent motion beyond tau, which a
    //       phase-decreasing MP structurally cannot do.
    //
    //   --tau-mode full:
    //       tau_ = duration of the FULL demo (computed here, before the split,
    //       and passed via ProDMP::learnFromDemonstration(demo, tau_override)).
    //       The 80% training rows are fitted at their TRUE phase s in [0, ~0.8];
    //       the held-out 20% is rolled out at s in [~0.8, 1.0], i.e. genuine
    //       SHAPE extrapolation within the primitive's designed phase range.
    //
    // Both modes are provided so STEP 2/3 can measure each interpretation
    // separately; "train" stays the default so older numbers remain comparable.
    std::string tau_mode = "train";
    std::string split_mode = "temporal";
    bool report_condition_number = false;
    bool fix_goal_to_demo_endpoint = false;

    for (int i = 9; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--tau-mode" && i + 1 < argc) {
            tau_mode = argv[++i];
        } else if (arg == "--split-mode" && i + 1 < argc) {
            split_mode = argv[++i];
        } else if (arg == "--report-condition-number") {
            report_condition_number = true;
        } else if (arg == "--fix-goal") {
            fix_goal_to_demo_endpoint = true;
        } else {
            std::cerr << "ERROR: unrecognised argument '" << arg << "'\n";
            return 1;
        }
    }
    if (tau_mode != "train" && tau_mode != "full") {
        std::cerr << "ERROR: --tau-mode must be 'train' or 'full', got '" << tau_mode << "'\n";
        return 1;
    }
    if (split_mode != "temporal" && split_mode != "interleaved") {
        std::cerr << "ERROR: --split-mode must be 'temporal' or 'interleaved', got '" << split_mode << "'\n";
        return 1;
    }

    if (holdout_fraction <= 0.0 || holdout_fraction >= 1.0) {
        std::cerr << "ERROR: holdout_fraction must be strictly in (0.0, 1.0), got "
                  << holdout_fraction << "\n";
        return 1;
    }

    try {
        // 1. Carica la demo
        std::vector<Sample> demo = haptic_dmp_learning::core::demo_csv_io::readDemoCsv(input_csv);
        const size_t n_samples = demo.size();
        if (n_samples < 10) {
            throw std::runtime_error("Demo too short (" + std::to_string(n_samples) + " samples)");
        }

        std::vector<Sample> train_demo;
        std::vector<bool> is_test(n_samples, false);
        size_t n_train = 0;
        size_t n_test = 0;

        if (split_mode == "temporal") {
            // Split TEMPORALE per indice (primi (1-holdout_fraction) per train, coda per test)
            n_train = static_cast<size_t>(static_cast<double>(n_samples) * (1.0 - holdout_fraction));
            n_test  = n_samples - n_train;
            if (n_train < 5 || n_test < 1) {
                throw std::runtime_error("Invalid split size: n_train=" + std::to_string(n_train) +
                                         ", n_test=" + std::to_string(n_test));
            }
            for (size_t i = n_train; i < n_samples; ++i) {
                is_test[i] = true;
            }
            train_demo.assign(demo.begin(), demo.begin() + n_train);
        } else {
            // Split INTERLEAVED (1 campione ogni K per test, resto per train)
            const size_t K = std::max<size_t>(2, static_cast<size_t>(std::round(1.0 / holdout_fraction)));
            train_demo.reserve(n_samples);
            for (size_t i = 0; i < n_samples; ++i) {
                // Mantieni sempre il primo (i=0) e l'ultimo (i=n_samples-1) nel training set
                if (i > 0 && i < n_samples - 1 && (i % K == 0)) {
                    is_test[i] = true;
                    ++n_test;
                } else {
                    is_test[i] = false;
                    train_demo.push_back(demo[i]);
                    ++n_train;
                }
            }
        }

        // tau of the full movement (before the split)
        const double tau_full = demo.back().t - demo.front().t;
        const double tau_train = train_demo.back().t - train_demo.front().t;
        // For interleaved split, tau is ALWAYS full movement duration
        const double tau_override = (split_mode == "interleaved" || tau_mode == "full") ? tau_full : 0.0;

        // 3. Fitta ProDMP SOLO sul sottoinsieme train
        constexpr double kAlphaZ = 25.0;
        constexpr double kAlphaX = 4.6;
        ProDMP prodmp(n_basis, kAlphaZ, kAlphaX, ridge_lambda);
        if (window_sec > 0.0) {
            prodmp.setPositionFilterWindow(window_sec);
        } else {
            prodmp.setPositionFilterWindow(0.0);
        }
        if (report_condition_number) {
            prodmp.setComputeConditionNumber(true);
        }
        if (fix_goal_to_demo_endpoint) {
            prodmp.setFixGoalToDemoEndpoint(true);
        }

        prodmp.learnFromDemonstration(train_demo, tau_override);

        std::cout << "  [split-mode " << split_mode << " | tau-mode " << tau_mode
                  << "] n_train=" << n_train << " n_test=" << n_test
                  << " tau_train=" << tau_train << "s tau_full=" << tau_full
                  << "s tau_used=" << prodmp.tau() << "s\n";

        const double cond_number = prodmp.diagnostics().design_matrix_condition_number;
        if (report_condition_number) {
            std::cout << "  cond(H) after column-scale preconditioning: " << cond_number << "\n";
        }

        if (!output_yaml.empty() && output_yaml != "-") {
            haptic_dmp_learning::core::prodmp_io::saveProDmpToYaml(prodmp, output_yaml);
        }

        // 4. Rollout oltre tau fino a coprire tutti i campioni della demo completa
        prodmp.setInitialConditions(prodmp.initTime(), prodmp.initPos(), prodmp.initVel());

        std::vector<Eigen::Vector3d> ref_pos_train, replay_pos_train;
        std::vector<Eigen::Vector3d> ref_pos_test,  replay_pos_test;
        ref_pos_train.reserve(n_train);
        replay_pos_train.reserve(n_train);
        ref_pos_test.reserve(n_test);
        replay_pos_test.reserve(n_test);

        double prev_t = demo.front().t;
        for (size_t k = 0; k < n_samples; ++k) {
            double dt = (k == 0) ? 0.0 : (demo[k].t - prev_t);
            prev_t = demo[k].t;

            Eigen::Vector3d p = (k == 0) ? prodmp.step(0.0) : prodmp.step(dt);
            if (!is_test[k]) {
                ref_pos_train.push_back(demo[k].position);
                replay_pos_train.push_back(p);
            } else {
                ref_pos_test.push_back(demo[k].position);
                replay_pos_test.push_back(p);
            }
        }

        // 5. Calcola le due metriche di fedeltà (in-sample e held-out)
        using namespace dmp_tools::metrics;
        TrajectoryFidelity tf_train = computeTrajectoryFidelity(ref_pos_train, replay_pos_train);
        TrajectoryFidelity tf_test  = computeTrajectoryFidelity(ref_pos_test, replay_pos_test);

        const double rmse_in_sample_mm = tf_train.rmse_overall;
        const double rmse_holdout_mm   = tf_test.rmse_overall;

        // 6. Diagnostica ampiezza dei pesi: max(abs(weights_[d]))
        double max_abs_weight = 0.0;
        const auto& w_arr = prodmp.weights();
        for (int d = 0; d < 3; ++d) {
            if (w_arr[d].size() > 0) {
                max_abs_weight = std::max(max_abs_weight, w_arr[d].cwiseAbs().maxCoeff());
            }
        }

        const double gap_ratio =
            (rmse_in_sample_mm > 0.0) ? (rmse_holdout_mm / rmse_in_sample_mm) : 0.0;

        std::cout << "[" << label << "] n_basis: " << n_basis
                  << " | lambda: " << ridge_lambda
                  << " | window: " << window_sec << "s"
                  << " | tau_mode: " << tau_mode
                  << " | holdout: " << (holdout_fraction * 100.0) << "%\n"
                  << "  RMSE in-sample: " << rmse_in_sample_mm << " mm\n"
                  << "  RMSE held-out:   " << rmse_holdout_mm << " mm\n"
                  << "  gap ratio:       " << gap_ratio << "x\n"
                  << "  Max abs weight:  " << max_abs_weight << "\n";

        // 7. Salva summary
        appendToHoldoutSummaryCsv(summary_csv, label, n_basis, ridge_lambda,
                                  window_sec, tau_mode, rmse_in_sample_mm, rmse_holdout_mm,
                                  max_abs_weight, cond_number);

    } catch (const std::exception& e) {
        std::cerr << "ERROR [" << label << "]: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
