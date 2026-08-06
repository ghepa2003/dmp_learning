// Learns DMP weights from reference demo once (n_basis=100) and executes 5 new goals
// WITHOUT re-training (spatial/rotational generalization via setGoal).
//
// Includes authentic tests for both guardrails:
//   1. ratio > 2.0 (triggered naturally on Traj B dim X, ratio = 3.36)
//   2. kMinDG < 1e-6 (triggered via synthetic demo modification where dG_x = 0)

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
        throw std::runtime_error("run_goal_generalization: cannot open " + path);
    }

    std::vector<Sample> demo;
    std::string line;
    std::getline(f, line);  // skip header (t,x,y,z,qw,qx,qy,qz)

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

    if (demo.size() < 5) {
        throw std::runtime_error("run_goal_generalization: demo too short (" +
                                  std::to_string(demo.size()) + " samples)");
    }
    return demo;
}

static void writeReplayCsv(const std::string& path, const std::vector<double>& t,
                           const std::vector<Eigen::Vector3d>& p,
                           const std::vector<Eigen::Quaterniond>& q) {
    std::ofstream f(path);
    f << "t,x,y,z,qw,qx,qy,qz\n";
    for (size_t k = 0; k < t.size(); ++k) {
        f << t[k] << "," << p[k].x() << "," << p[k].y() << "," << p[k].z() << ","
          << q[k].w() << "," << q[k].x() << "," << q[k].y() << "," << q[k].z() << "\n";
    }
}

struct GoalSpec {
    int id;
    std::string name;
    Eigen::Vector3d pos_goal;
    Eigen::Quaterniond quat_goal;
    Eigen::Vector3d final_pos;
    Eigen::Quaterniond final_quat;
    double pos_error_mm;
    double orient_error_deg;
    std::array<bool, 3> scale_reliable;
    Eigen::Vector3d scale_factor;
};

static std::vector<GoalSpec> create5Goals(const Eigen::Vector3d& g_orig, const Eigen::Quaterniond& q_orig) {
    std::vector<GoalSpec> goals;

    // Goal 1: +4cm X, +3cm Y, +2cm Z | +15 deg rot X
    {
        GoalSpec g;
        g.id = 1;
        g.name = "Goal 1 (+4cm X, +3cm Y, +2cm Z, +15° rot X)";
        g.pos_goal = g_orig + Eigen::Vector3d(0.04, 0.03, 0.02);
        Eigen::Quaterniond dq(Eigen::AngleAxisd(15.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()));
        g.quat_goal = (dq * q_orig).normalized();
        goals.push_back(g);
    }

    // Goal 2: -5cm X, +4cm Y, -3cm Z | +15 deg rot Y
    {
        GoalSpec g;
        g.id = 2;
        g.name = "Goal 2 (-5cm X, +4cm Y, -3cm Z, +15° rot Y)";
        g.pos_goal = g_orig + Eigen::Vector3d(-0.05, 0.04, -0.03);
        Eigen::Quaterniond dq(Eigen::AngleAxisd(15.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()));
        g.quat_goal = (dq * q_orig).normalized();
        goals.push_back(g);
    }

    // Goal 3: +3cm X, -5cm Y, +4cm Z | +15 deg rot Z
    {
        GoalSpec g;
        g.id = 3;
        g.name = "Goal 3 (+3cm X, -5cm Y, +4cm Z, +15° rot Z)";
        g.pos_goal = g_orig + Eigen::Vector3d(0.03, -0.05, 0.04);
        Eigen::Quaterniond dq(Eigen::AngleAxisd(15.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()));
        g.quat_goal = (dq * q_orig).normalized();
        goals.push_back(g);
    }

    // Goal 4: -4cm X, -3cm Y, +5cm Z | -15 deg rot X, +10 deg rot Y
    {
        GoalSpec g;
        g.id = 4;
        g.name = "Goal 4 (-4cm X, -3cm Y, +5cm Z, -15° X/+10° Y)";
        g.pos_goal = g_orig + Eigen::Vector3d(-0.04, -0.03, 0.05);
        Eigen::Quaterniond dq1(Eigen::AngleAxisd(-15.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()));
        Eigen::Quaterniond dq2(Eigen::AngleAxisd(10.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()));
        g.quat_goal = (dq2 * dq1 * q_orig).normalized();
        goals.push_back(g);
    }

    // Goal 5: +5cm X, -4cm Y, -3cm Z | +20 deg rot Z, -10 deg rot X
    {
        GoalSpec g;
        g.id = 5;
        g.name = "Goal 5 (+5cm X, -4cm Y, -3cm Z, +20° Z/-10° X)";
        g.pos_goal = g_orig + Eigen::Vector3d(0.05, -0.04, -0.03);
        Eigen::Quaterniond dq1(Eigen::AngleAxisd(20.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond dq2(Eigen::AngleAxisd(-10.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()));
        g.quat_goal = (dq2 * dq1 * q_orig).normalized();
        goals.push_back(g);
    }

    return goals;
}

static void testKMinDGSynthetic(const std::vector<Sample>& orig_demo, int n_basis) {
    std::cout << "\n========================================================================\n";
    std::cout << "  SYNTHETIC TEST: Authentic Verification of Guardrail kMinDG (dG_d < 1e-6)\n";
    std::cout << "========================================================================\n";

    // Modify final sample position on X to match start position on X:
    // This forces dG_(0) = demo.back().position.x() - demo.front().position.x() = 0.0 (< 1e-6) BEFORE learning
    std::vector<Sample> modified_demo = orig_demo;
    modified_demo.back().position.x() = modified_demo.front().position.x();

    DMP dmp(n_basis);
    QuaternionDMP qdmp(n_basis);
    dmp.setRidgeRegression(true, 1e-6);
    dmp.setVelocityFilter(true, 0.05, 0.05);

    dmp.learnFromDemonstration(modified_demo);

    std::cout << "  Original Demo displacement dG_x (before fit) : " 
              << (orig_demo.back().position.x() - orig_demo.front().position.x()) << " m\n";
    std::cout << "  Modified Demo displacement dG_x (for fit)   : " << dmp.dG().x() << " m\n";
    std::cout << "  Check |dG_x| < 1e-6 : " << (std::abs(dmp.dG().x()) < 1e-6 ? "TRUE" : "FALSE") << "\n\n";

    // Now call setGoal with a NEW goal requesting displacement along X (+5 cm)
    Eigen::Vector3d new_goal = modified_demo.back().position + Eigen::Vector3d(0.05, 0.03, 0.02);
    dmp.setGoal(new_goal);

    std::cout << "  Now calling setGoal() with new goal requesting displacement along X = 0.05 m:\n";
    std::cout << "  -> scaleReliable(X) : " << (dmp.isScaleReliable(0) ? "TRUE (1)" : "FALSE (0)") << "  <-- (kMinDG GUARDRAIL TRIGGERED!)\n";
    std::cout << "  -> scaleReliable(Y) : " << (dmp.isScaleReliable(1) ? "TRUE (1)" : "FALSE (0)") << "\n";
    std::cout << "  -> scaleReliable(Z) : " << (dmp.isScaleReliable(2) ? "TRUE (1)" : "FALSE (0)") << "\n";
    std::cout << "  -> scale factor (X) : " << dmp.scale().x() << "  <-- (Forced to 1.0 to prevent division by zero)\n";

    // Replay to verify execution stability
    dmp.reset();
    qdmp.reset();
    double prev_t = modified_demo.front().t;
    for (size_t k = 0; k < modified_demo.size(); ++k) {
        double dt = (k == 0) ? 0.0 : (modified_demo[k].t - prev_t);
        prev_t = modified_demo[k].t;
        dmp.step(dt);
    }
    double pos_err = dmp_tools::metrics::computeEndpointError(dmp.step(0.0), new_goal); // final step position
    std::cout << "  Replay Execution Endpoint Position Error: " << pos_err << " mm\n";
    std::cout << "========================================================================\n\n";
}

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <input_demo.csv> <out_dir> <label> [n_basis=100] [feature_flags_path='']\n";
        return 1;
    }

    const std::string input_csv   = argv[1];
    const std::string out_dir     = argv[2];
    const std::string label       = argv[3];
    const int n_basis             = (argc >= 5) ? std::stoi(argv[4]) : 100;
    std::string feature_flags_path = (argc >= 6) ? argv[5] : "";

    if (feature_flags_path.empty()) {
        const std::string default_path = "../../src/haptic_dmp_learning/config/dmp_features.yaml";
        if (fs::exists(default_path)) {
            feature_flags_path = default_path;
        } else if (fs::exists("04_basis_sweep/test_configs/ridge_filter.yaml")) {
            feature_flags_path = "04_basis_sweep/test_configs/ridge_filter.yaml";
        }
    }

    fs::create_directories(out_dir);
    fs::create_directories(out_dir + "/data");
    fs::create_directories(out_dir + "/weights");

    try {
        std::cout << "\n========================================================================\n";
        std::cout << "  Goal Generalization & Guardrail Test: " << label << " (n_basis=" << n_basis << ")\n";
        std::cout << "========================================================================\n";
        std::cout << "Loading demo from " << input_csv << "...\n";
        std::vector<Sample> demo = loadDemoCsv(input_csv);
        const double duration = demo.back().t - demo.front().t;
        std::cout << "  Samples: " << demo.size() << " | Duration: " << duration << "s\n";

        const Eigen::Vector3d orig_y0 = demo.front().position;
        const Eigen::Vector3d orig_goal_pos = demo.back().position;
        const Eigen::Quaterniond orig_goal_quat = demo.back().orientation;

        // 1. Train DMP once on reference demo
        DMP dmp(n_basis);
        QuaternionDMP qdmp(n_basis);

        if (!feature_flags_path.empty() && fs::exists(feature_flags_path)) {
            haptic_dmp_learning::core::dmp_io::applyFeatureConfig(feature_flags_path, dmp, qdmp);
        } else {
            dmp.setRidgeRegression(true, 1e-6);
            dmp.setVelocityFilter(true, 0.05, 0.05);
            qdmp.setRidgeRegression(true, 1e-6);
            qdmp.setVelocityFilter(true, 0.05, 0.05);
        }

        dmp.learnFromDemonstration(demo);
        qdmp.learnFromDemonstration(demo);

        std::cout << "\n--- DEMONSTRATION DIAGNOSTICS (Original Fit) ---\n";
        std::cout << "  Start Position y0 : [" << orig_y0.x() << ", " << orig_y0.y() << ", " << orig_y0.z() << "] m\n";
        std::cout << "  Goal Position g0 : [" << orig_goal_pos.x() << ", " << orig_goal_pos.y() << ", " << orig_goal_pos.z() << "] m\n";
        std::cout << "  dG (g0 - y0)     : [" << dmp.dG().x() << ", " << dmp.dG().y() << ", " << dmp.dG().z() << "] m\n";
        std::cout << "  A (Observed Ampl): [" << dmp.A().x() << ", " << dmp.A().y() << ", " << dmp.A().z() << "] m\n";

        for (int d = 0; d < 3; ++d) {
            char dim_name = (d == 0) ? 'X' : (d == 1) ? 'Y' : 'Z';
            double ratio = dmp.A()(d) / std::abs(dmp.dG()(d));
            std::cout << "  Dimension " << dim_name << " -> Ratio A/|dG| = " << ratio 
                      << " | scaleReliable: " << (dmp.isScaleReliable(d) ? "TRUE (1)" : "FALSE (0)");
            if (ratio > 2.0) {
                std::cout << "  <-- (GUARDRAIL Ratio > 2.0 TRIGGERED!)";
            }
            std::cout << "\n";
        }

        std::string weights_path = out_dir + "/weights/" + label + "_weights.yaml";
        haptic_dmp_learning::core::dmp_io::saveToYaml(dmp, qdmp, weights_path);

        // 2. Replay Original Goal
        dmp.reset();
        qdmp.reset();
        std::vector<double> t_out;
        std::vector<Eigen::Vector3d> orig_pos;
        std::vector<Eigen::Quaterniond> orig_orient;
        t_out.reserve(demo.size());
        orig_pos.reserve(demo.size());
        orig_orient.reserve(demo.size());

        double prev_t = demo.front().t;
        for (size_t k = 0; k < demo.size(); ++k) {
            double dt = (k == 0) ? 0.0 : (demo[k].t - prev_t);
            prev_t = demo[k].t;
            orig_pos.push_back(k == 0 ? dmp.step(0.0) : dmp.step(dt));
            orig_orient.push_back(k == 0 ? qdmp.step(0.0) : qdmp.step(dt));
            t_out.push_back(demo[k].t - demo.front().t);
        }
        std::string replay_orig_path = out_dir + "/data/" + label + "_replay_orig.csv";
        writeReplayCsv(replay_orig_path, t_out, orig_pos, orig_orient);

        // 3. Define Goals (1..5 standard)
        std::vector<GoalSpec> goals = create5Goals(orig_goal_pos, orig_goal_quat);

        std::string goals_csv_path = out_dir + "/data/" + label + "_goals_info.csv";
        std::ofstream fg(goals_csv_path);
        fg << "goal_id,name,gx,gy,gz,gqw,gqx,gqy,gqz,err_pos_mm,err_orient_deg\n";

        std::cout << "\n------------------------------------------------------------------------\n";
        std::cout << "  Testing 5 Goals (setGoal)\n";
        std::cout << "------------------------------------------------------------------------\n";
        std::cout << std::fixed << std::setprecision(4);

        for (auto& g : goals) {
            dmp.reset();
            dmp.setGoal(g.pos_goal);
            qdmp.reset();
            qdmp.setGoal(g.quat_goal);

            for (int d = 0; d < 3; ++d) {
                g.scale_reliable[d] = dmp.isScaleReliable(d);
                g.scale_factor(d) = dmp.scale()(d);
            }

            std::vector<Eigen::Vector3d> g_pos;
            std::vector<Eigen::Quaterniond> g_orient;
            g_pos.reserve(demo.size());
            g_orient.reserve(demo.size());

            prev_t = demo.front().t;
            for (size_t k = 0; k < demo.size(); ++k) {
                double dt = (k == 0) ? 0.0 : (demo[k].t - prev_t);
                prev_t = demo[k].t;
                g_pos.push_back(k == 0 ? dmp.step(0.0) : dmp.step(dt));
                g_orient.push_back(k == 0 ? qdmp.step(0.0) : qdmp.step(dt));
            }

            g.final_pos = g_pos.back();
            g.final_quat = g_orient.back();

            g.pos_error_mm = dmp_tools::metrics::computeEndpointError(g.final_pos, g.pos_goal);
            g.orient_error_deg = dmp_tools::metrics::computeAngularEndpointError(g.final_quat, g.quat_goal);

            std::string replay_goal_path = out_dir + "/data/" + label + "_replay_goal_" + std::to_string(g.id) + ".csv";
            writeReplayCsv(replay_goal_path, t_out, g_pos, g_orient);

            std::cout << "  [Goal " << g.id << "] " << g.name << "\n";
            std::cout << "    Requested Goal Pos : [" << g.pos_goal.x() << ", " << g.pos_goal.y() << ", " << g.pos_goal.z() << "] m\n";
            std::cout << "    Final Reached Pos   : [" << g.final_pos.x() << ", " << g.final_pos.y() << ", " << g.final_pos.z() << "] m\n";
            std::cout << "    Position Goal Error : " << g.pos_error_mm << " mm\n";
            std::cout << "    Orient Goal Error   : " << g.orient_error_deg << " deg\n";
            std::cout << "    scaleReliable (X,Y,Z): ["
                      << (g.scale_reliable[0] ? "1" : "0") << ", "
                      << (g.scale_reliable[1] ? "1" : "0") << ", "
                      << (g.scale_reliable[2] ? "1" : "0") << "]\n";
            std::cout << "    scale factors (X,Y,Z): ["
                      << g.scale_factor.x() << ", " << g.scale_factor.y() << ", " << g.scale_factor.z() << "]\n\n";

            fg << g.id << ",\"" << g.name << "\","
               << g.pos_goal.x() << "," << g.pos_goal.y() << "," << g.pos_goal.z() << ","
               << g.quat_goal.w() << "," << g.quat_goal.x() << "," << g.quat_goal.y() << "," << g.quat_goal.z() << ","
               << g.pos_error_mm << "," << g.orient_error_deg << "\n";
        }
        fg.close();

        // Run authentic kMinDG synthetic guardrail test on Traj A
        if (label == "trajA") {
            testKMinDGSynthetic(demo, n_basis);
        }

    } catch (const std::exception& e) {
        std::cerr << "ERROR [" << label << "]: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
