// Rigorous Test of DMP Guardrails: kMinDG and ratio > 2.0
//
// Evaluates both guardrails isolated on the SAME baseline trajectory (trajA)
// with the SAME test request (+5 cm shift ONLY on X, Y and Z unchanged).
//
// Conducts:
//   TEST 1: kMinDG isolated WITH guardrail (smooth minimum-jerk tapering of dG_x -> 0)
//   TEST 2: ratio > 2.0 isolated WITH guardrail (smooth reduction of dG_x -> Ratio = 3.04 > 2.0)
//   TEST 3a: kMinDG WITHOUT guardrail (showing un-guardrailed forcing term explosion)
//   TEST 3b: ratio > 2.0 WITHOUT guardrail (showing un-guardrailed forcing term amplification)
//   TEST 4: Sanity check verifying untouched axes (Y and Z) remain reliable and uncorrupted

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <stdexcept>
#include <iomanip>
#include <filesystem>
#include <algorithm>

#include "haptic_dmp_learning/core/dmp.hpp"
#include "haptic_dmp_learning/core/quaternion_dmp.hpp"
#include "haptic_dmp_learning/core/dmp_io.hpp"
#include "haptic_dmp_learning/core/types.hpp"
#include "metrics.hpp"

namespace fs = std::filesystem;
using haptic_dmp_learning::core::DMP;
using haptic_dmp_learning::core::QuaternionDMP;
using haptic_dmp_learning::core::Sample;

static std::vector<Sample> loadDemoCsv(const std::string& path) {
    std::ifstream f(path);
    if (!f.good()) {
        throw std::runtime_error("test_guardrails_rigorous: cannot open " + path);
    }

    std::vector<Sample> demo;
    std::string line;
    std::getline(f, line);

    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string field;
        std::vector<double> vals;
        while (std::getline(ss, field, ',')) {
            vals.push_back(std::stod(field));
        }
        if (vals.size() < 8) continue;

        Sample s;
        s.t = vals[0];
        s.position = Eigen::Vector3d(vals[1], vals[2], vals[3]);
        s.orientation = Eigen::Quaterniond(vals[4], vals[5], vals[6], vals[7]);
        demo.push_back(s);
    }
    return demo;
}

// Minimum-jerk polynomial interpolator: poly(s) = 10 s^3 - 15 s^4 + 6 s^5, s in [0,1]
// Continuous C1/C2 at s=0 (poly=0, poly'=0, poly''=0) and s=1 (poly=1, poly'=0, poly''=0)
static double minJerkPoly(double s) {
    s = std::max(0.0, std::min(1.0, s));
    return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
}

// Applies smooth C1/C2 minimum-jerk tapering on the X component of the demo over the last window_ratio of samples
// Brings pos.x() from raw value smoothly to target_x_final, ensuring dG_x = 0 (or target_x_final - y0_x)
static std::vector<Sample> applySmoothTaperingX(const std::vector<Sample>& orig_demo, double target_x_final, double window_ratio = 0.20) {
    std::vector<Sample> demo = orig_demo;
    const size_t N = demo.size();
    const size_t n_taper = static_cast<size_t>(N * window_ratio);
    const size_t start_idx = N - n_taper;

    for (size_t k = start_idx; k < N; ++k) {
        double s = static_cast<double>(k - start_idx) / static_cast<double>(N - 1 - start_idx);
        double poly = minJerkPoly(s);
        double x_raw = orig_demo[k].position.x();
        // Blend from x_raw at s=0 to target_x_final at s=1
        demo[k].position.x() = x_raw * (1.0 - poly) + target_x_final * poly;
    }
    return demo;
}

struct TestResult {
    std::string name;
    std::string axis_affected;
    double dG_x, dG_y, dG_z;
    double A_x, A_y, A_z;
    double ratio_x;
    std::array<bool, 3> scale_reliable;
    Eigen::Vector3d scale_factor;
    double final_pos_err_mm;
    double max_pos_err_mm;
    double settling_time_sec;
    double settling_time_pct;
    double iae_mm_s;
    std::vector<double> t_series;
    std::vector<double> err_series_mm;

    // Detailed debug logs for Test 3a tail inspection
    bool had_nan_inf = false;
    std::vector<double> raw_y_x;
    std::vector<double> raw_z_x;
    std::vector<double> raw_phase_x;
};

static TestResult runSingleTest(const std::string& test_name,
                                const std::vector<Sample>& demo,
                                const Eigen::Vector3d& goal_request,
                                bool disable_kmindg,
                                bool disable_ratio,
                                int n_basis = 100) {
    TestResult res;
    res.name = test_name;
    res.axis_affected = "X";

    DMP dmp(n_basis);
    QuaternionDMP qdmp(n_basis);
    dmp.setRidgeRegression(true, 1e-6);
    dmp.setVelocityFilter(true, 0.05, 0.05);

    dmp.learnFromDemonstration(demo);
    qdmp.learnFromDemonstration(demo);

    res.dG_x = dmp.dG().x(); res.dG_y = dmp.dG().y(); res.dG_z = dmp.dG().z();
    res.A_x = dmp.A().x();   res.A_y = dmp.A().y();   res.A_z = dmp.A().z();
    res.ratio_x = res.A_x / std::abs(res.dG_x);

    // Call setGoal with explicit guardrail override flags for testing
    dmp.setGoal(goal_request, disable_kmindg, disable_ratio);

    for (int d = 0; d < 3; ++d) {
        res.scale_reliable[d] = dmp.isScaleReliable(d);
        res.scale_factor(d) = dmp.scale()(d);
    }

    dmp.reset();
    qdmp.reset();

    std::vector<Eigen::Vector3d> replay_pos;
    replay_pos.reserve(demo.size());

    double max_err = 0.0;
    double prev_t = demo.front().t;
    const double total_duration = demo.back().t - demo.front().t;

    for (size_t k = 0; k < demo.size(); ++k) {
        double dt = (k == 0) ? 0.0 : (demo[k].t - prev_t);
        prev_t = demo[k].t;
        Eigen::Vector3d p = (k == 0) ? dmp.step(0.0) : dmp.step(dt);
        replay_pos.push_back(p);

        double t_rel = demo[k].t - demo.front().t;
        double err_k = (p - goal_request).norm() * 1000.0;

        if (std::isnan(err_k) || std::isinf(err_k) || std::isnan(p.x()) || std::isinf(p.x())) {
            res.had_nan_inf = true;
        }

        double capped_err = (std::isnan(err_k) || std::isinf(err_k) || err_k > 1e9) ? 1e9 : err_k;
        max_err = std::max(max_err, capped_err);

        res.t_series.push_back(t_rel);
        res.err_series_mm.push_back(capped_err);

        res.raw_y_x.push_back(p.x());
        res.raw_phase_x.push_back(dmp.phase());
    }

    res.max_pos_err_mm = max_err;
    double raw_err = dmp_tools::metrics::computeEndpointError(replay_pos.back(), goal_request);
    res.final_pos_err_mm = (std::isnan(raw_err) || std::isinf(raw_err) || raw_err > 1e9) ? 1e9 : raw_err;

    // FIX 2: Calculate Settling Time to 10mm (time after which err_k <= 10.0mm for all remaining samples)
    constexpr double kSettlingThreshold = 10.0; // mm
    ssize_t last_exceed_idx = -1;
    for (ssize_t k = 0; k < static_cast<ssize_t>(demo.size()); ++k) {
        if (res.err_series_mm[k] > kSettlingThreshold) {
            last_exceed_idx = k;
        }
    }

    if (last_exceed_idx == -1) {
        res.settling_time_sec = 0.0;
        res.settling_time_pct = 0.0;
    } else if (last_exceed_idx == static_cast<ssize_t>(demo.size()) - 1) {
        // Exceeds threshold at the very end => never settled below 10mm
        res.settling_time_sec = -1.0;
        res.settling_time_pct = -1.0;
    } else {
        res.settling_time_sec = res.t_series[last_exceed_idx + 1];
        res.settling_time_pct = (res.settling_time_sec / total_duration) * 100.0;
    }

    // FIX 2: Calculate IAE (Integrated Absolute Error under curve in mm*s using trapezoidal rule)
    double iae = 0.0;
    for (size_t k = 0; k < demo.size() - 1; ++k) {
        double dt = res.t_series[k + 1] - res.t_series[k];
        double e_avg = 0.5 * (res.err_series_mm[k] + res.err_series_mm[k + 1]);
        iae += e_avg * dt;
    }
    res.iae_mm_s = iae;

    return res;
}

int main(int argc, char** argv) {
    std::string demo_path = (argc >= 2) ? argv[1] : "demo_raw_trajA.csv";
    if (!fs::exists(demo_path) && fs::exists("../../" + demo_path)) {
        demo_path = "../../" + demo_path;
    }

    std::string out_dir = (argc >= 3) ? argv[2] : "plots/06_goal_generalization";
    fs::create_directories(out_dir);
    fs::create_directories(out_dir + "/data");

    std::cout << "\n========================================================================================\n";
    std::cout << "  RIGOROUS DMP GUARDRAIL BENCHMARK WITH TRANSIENT METRICS & UNIFIED OUTPUT (trajA)\n";
    std::cout << "========================================================================================\n";
    std::cout << "Loading baseline demo: " << demo_path << "...\n";
    std::vector<Sample> raw_demo = loadDemoCsv(demo_path);

    const Eigen::Vector3d y0_orig = raw_demo.front().position;
    const Eigen::Vector3d g0_orig = raw_demo.back().position;

    // Test request: +5 cm ONLY on X axis, Y and Z unchanged
    const Eigen::Vector3d test_goal_request = g0_orig + Eigen::Vector3d(0.05, 0.0, 0.0);

    std::cout << "Baseline y0_orig : [" << y0_orig.x() << ", " << y0_orig.y() << ", " << y0_orig.z() << "] m\n";
    std::cout << "Baseline g0_orig : [" << g0_orig.x() << ", " << g0_orig.y() << ", " << g0_orig.z() << "] m\n";
    std::cout << "Test Goal Request: [" << test_goal_request.x() << ", " << test_goal_request.y() << ", " << test_goal_request.z() << "] m (+5cm on X)\n\n";

    // -------------------------------------------------------------------------
    // PREPARATION OF DEMO VARIANTS:
    // -------------------------------------------------------------------------

    // Demo 1 (for Test 1 & 3a): Minimum-jerk tapering of X to y0_x over last 20% of samples -> dG_x = 0.0 < 1e-6
    std::vector<Sample> demo_kmindg = applySmoothTaperingX(raw_demo, y0_orig.x(), 0.20);

    // Demo 2 (for Test 2 & 3b): Minimum-jerk tapering of X to (y0_x + 0.006525m) -> dG_x = 6.5mm, Ratio_x = 3.04 > 2.0
    std::vector<Sample> demo_ratio = applySmoothTaperingX(raw_demo, y0_orig.x() + 0.006525, 0.20);

    // -------------------------------------------------------------------------
    // EXECUTION OF THE 4 TESTS:
    // -------------------------------------------------------------------------

    // TEST 1: kMinDG Isolated WITH Guardrail
    TestResult t1 = runSingleTest("TEST 1 (kMinDG WITH Guardrail)", demo_kmindg, test_goal_request,
                                  /*disable_kmindg=*/false, /*disable_ratio=*/false);

    // TEST 3a: kMinDG WITHOUT Guardrail (disabling all guardrails)
    TestResult t3a = runSingleTest("TEST 3a (kMinDG WITHOUT Guardrail)", demo_kmindg, test_goal_request,
                                   /*disable_kmindg=*/true, /*disable_ratio=*/true);

    // TEST 2: ratio > 2.0 Isolated WITH Guardrail
    TestResult t2 = runSingleTest("TEST 2 (Ratio>2.0 WITH Guardrail)", demo_ratio, test_goal_request,
                                  /*disable_kmindg=*/false, /*disable_ratio=*/false);

    // TEST 3b: ratio > 2.0 WITHOUT Guardrail
    TestResult t3b = runSingleTest("TEST 3b (Ratio>2.0 WITHOUT Guardrail)", demo_ratio, test_goal_request,
                                   /*disable_kmindg=*/false, /*disable_ratio=*/true);

    // -------------------------------------------------------------------------
    // PRINT UNIFIED DETAILED SUMMARY TABLE (FIX 1 & FIX 2)
    // -------------------------------------------------------------------------
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "\n=========================================================================================================================================================\n";
    std::cout << "                                              UNIFIED RIGOROUS GUARDRAIL BENCHMARK SUMMARY TABLE                                         \n";
    std::cout << "=========================================================================================================================================================\n";
    std::cout << "| Test Case                        | dG_x [m] | Ratio A/|dG| | scaleReliable | scale_x     | Max Err [mm] | Settle <10mm (s, %) | IAE [mm*s]   | Final Err [mm] |\n";
    std::cout << "---------------------------------------------------------------------------------------------------------------------------------------------------------\n";

    auto printRow = [](const TestResult& r) {
        std::string settle_str;
        if (r.settling_time_sec < 0) {
            settle_str = "N/A (never)";
        } else {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << r.settling_time_sec << "s (" << r.settling_time_pct << "%)";
            settle_str = ss.str();
        }

        std::stringstream ss_scale;
        if (r.scale_factor.x() > 1e6) {
            ss_scale << std::scientific << std::setprecision(3) << r.scale_factor.x();
        } else {
            ss_scale << std::fixed << std::setprecision(4) << r.scale_factor.x();
        }

        std::stringstream ss_max_err;
        if (r.max_pos_err_mm > 1e6) {
            ss_max_err << std::scientific << std::setprecision(2) << r.max_pos_err_mm;
        } else {
            ss_max_err << std::fixed << std::setprecision(2) << r.max_pos_err_mm;
        }

        std::stringstream ss_final_err;
        if (r.final_pos_err_mm > 1e6) {
            ss_final_err << std::scientific << std::setprecision(2) << r.final_pos_err_mm;
        } else {
            ss_final_err << std::fixed << std::setprecision(4) << r.final_pos_err_mm;
        }

        std::cout << "| " << std::left << std::setw(32) << r.name << " | "
                  << std::right << std::setw(8) << r.dG_x << " | "
                  << std::setw(12) << (std::abs(r.dG_x) < 1e-5 ? "INF (>1e5)" : std::to_string(r.ratio_x).substr(0,7)) << " | ["
                  << (r.scale_reliable[0] ? "1" : "0") << ","
                  << (r.scale_reliable[1] ? "1" : "0") << ","
                  << (r.scale_reliable[2] ? "1" : "0") << "]       | "
                  << std::setw(11) << ss_scale.str() << " | "
                  << std::setw(12) << ss_max_err.str() << " | "
                  << std::setw(19) << settle_str << " | "
                  << std::setw(12) << (r.iae_mm_s > 1e6 ? "INF (>1e6)" : std::to_string(r.iae_mm_s).substr(0,8)) << " | "
                  << std::setw(14) << ss_final_err.str() << " |\n";
    };

    printRow(t1);
    printRow(t3a);
    printRow(t2);
    printRow(t3b);
    std::cout << "=========================================================================================================================================================\n\n";

    // -------------------------------------------------------------------------
    // FIX 3: INVESTIGATION OF TEST 3A TAIL BEHAVIOR & OVERFLOW DIAGNOSTICS
    // -------------------------------------------------------------------------
    std::cout << "----------------------------------------------------------------------------------------------------\n";
    std::cout << "  FIX 3 DIAGNOSTICS: INVESTIGATION OF TEST 3a (kMinDG WITHOUT GUARDRAIL) TAIL BEHAVIOR\n";
    std::cout << "----------------------------------------------------------------------------------------------------\n";
    std::cout << "  Had IEEE-754 NaN/Inf overflow in Test 3a : " << (t3a.had_nan_inf ? "YES (Floating point overflow detected!)" : "NO") << "\n";
    std::cout << "  Inspecting RAW state values for the LAST 10 SAMPLES of Test 3a:\n";
    std::cout << "    Sample Index | Phase x(t)   | Position y_x [m]      | Error e(t) [mm]\n";
    std::cout << "    --------------------------------------------------------------------\n";
    const size_t N_t3a = t3a.t_series.size();
    for (size_t k = (N_t3a > 10 ? N_t3a - 10 : 0); k < N_t3a; ++k) {
        std::cout << "    " << std::setw(12) << k << " | "
                  << std::scientific << std::setprecision(4) << t3a.raw_phase_x[k] << " | "
                  << std::setw(21) << t3a.raw_y_x[k] << " | "
                  << std::setw(15) << t3a.err_series_mm[k] << "\n";
    }
    std::cout << "----------------------------------------------------------------------------------------------------\n\n";

    // -------------------------------------------------------------------------
    // TEST 4: SANITY CHECK VERIFICATION ON UNTOUCHED AXES (Y and Z)
    // -------------------------------------------------------------------------
    std::cout << "----------------------------------------------------------------------------------------------------\n";
    std::cout << "  TEST 4: SANITY CHECK ON UNTOUCHED AXES (Y & Z)\n";
    std::cout << "----------------------------------------------------------------------------------------------------\n";
    for (const auto* r : {&t1, &t3a, &t2, &t3b}) {
        bool y_ok = (r->scale_reliable[1] == true) && (std::abs(r->scale_factor.y() - 1.0) < 1e-4);
        bool z_ok = (r->scale_reliable[2] == true) && (std::abs(r->scale_factor.z() - 1.0) < 1e-4);
        std::cout << "  [" << std::setw(32) << r->name << "] -> Y-axis reliable: "
                  << (y_ok ? "PASS (true, scale=1.0)" : "FAIL")
                  << " | Z-axis reliable: " << (z_ok ? "PASS (true, scale=1.0)" : "FAIL") << "\n";
    }
    std::cout << "----------------------------------------------------------------------------------------------------\n\n";

    // -------------------------------------------------------------------------
    // SAVE UNIFIED SUMMARY METRICS CSV (FOR PYTHON PLOT LEGEND SYNCHRONIZATION)
    // -------------------------------------------------------------------------
    std::string summary_csv = out_dir + "/data/guardrail_summary_metrics.csv";
    std::ofstream fs_sum(summary_csv);
    fs_sum << "test_id,test_name,dG_x,A_x,ratio_x,scale_x,max_pos_err_mm,settling_sec,settling_pct,iae_mm_s,final_pos_err_mm\n";

    auto writeSummaryRow = [&fs_sum](const std::string& id, const TestResult& r) {
        fs_sum << id << ",\"" << r.name << "\","
               << r.dG_x << "," << r.A_x << "," << r.ratio_x << ","
               << r.scale_factor.x() << "," << r.max_pos_err_mm << ","
               << r.settling_time_sec << "," << r.settling_time_pct << ","
               << r.iae_mm_s << "," << r.final_pos_err_mm << "\n";
    };

    writeSummaryRow("t1", t1);
    writeSummaryRow("t3a", t3a);
    writeSummaryRow("t2", t2);
    writeSummaryRow("t3b", t3b);
    fs_sum.close();
    std::cout << "Saved summary metrics CSV: " << summary_csv << "\n";

    // -------------------------------------------------------------------------
    // WRITE TIME SERIES CSV FOR PYTHON PLOTTING
    // -------------------------------------------------------------------------
    std::string csv_path = out_dir + "/data/guardrail_timeseries.csv";
    std::ofstream fc(csv_path);
    fc << "t,err_t1_kmindg_with,err_t3a_kmindg_without,err_t2_ratio_with,err_t3b_ratio_without\n";

    size_t N_samples = t1.t_series.size();
    for (size_t k = 0; k < N_samples; ++k) {
        fc << t1.t_series[k] << ","
           << t1.err_series_mm[k] << ","
           << t3a.err_series_mm[k] << ","
           << t2.err_series_mm[k] << ","
           << t3b.err_series_mm[k] << "\n";
    }
    fc.close();
    std::cout << "Saved time-series data to: " << csv_path << "\n";

    return 0;
}
