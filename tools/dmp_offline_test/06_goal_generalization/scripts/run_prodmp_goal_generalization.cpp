// Offline test runner for ProDMP Goal Generalization on real demonstrations.
// Fits ProDMP (n_basis=80, lambda=1e-10, window=0.05s, --fix-goal) on demo,
// then executes 5 new target goals via setGoal() without re-training.

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

#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/prodmp_io.hpp"
#include "haptic_dmp_learning/core/demo_csv_io.hpp"
#include "haptic_dmp_learning/core/types.hpp"
#include "metrics.hpp"

namespace fs = std::filesystem;
using haptic_dmp_learning::core::ProDMP;
using haptic_dmp_learning::core::Sample;

static void writeReplayCsv(const std::string& path, const std::vector<double>& t,
                           const std::vector<Eigen::Vector3d>& p) {
    std::ofstream f(path);
    f << "t,x,y,z\n";
    for (size_t k = 0; k < t.size(); ++k) {
        f << t[k] << "," << p[k].x() << "," << p[k].y() << "," << p[k].z() << "\n";
    }
}

struct GoalSpec {
    int id;
    std::string name;
    Eigen::Vector3d pos_goal;
    Eigen::Vector3d final_pos;
    double pos_error_mm;
};

static std::vector<GoalSpec> create5Goals(const Eigen::Vector3d& g_orig) {
    std::vector<GoalSpec> goals;

    // Goal 1: +4cm X, +3cm Y, +2cm Z
    {
        GoalSpec g;
        g.id = 1;
        g.name = "Goal 1 (+4cm X, +3cm Y, +2cm Z)";
        g.pos_goal = g_orig + Eigen::Vector3d(0.04, 0.03, 0.02);
        goals.push_back(g);
    }
    // Goal 2: -5cm X, +4cm Y, -3cm Z
    {
        GoalSpec g;
        g.id = 2;
        g.name = "Goal 2 (-5cm X, +4cm Y, -3cm Z)";
        g.pos_goal = g_orig + Eigen::Vector3d(-0.05, 0.04, -0.03);
        goals.push_back(g);
    }
    // Goal 3: +3cm X, -5cm Y, +4cm Z
    {
        GoalSpec g;
        g.id = 3;
        g.name = "Goal 3 (+3cm X, -5cm Y, +4cm Z)";
        g.pos_goal = g_orig + Eigen::Vector3d(0.03, -0.05, 0.04);
        goals.push_back(g);
    }
    // Goal 4: -4cm X, -3cm Y, +5cm Z
    {
        GoalSpec g;
        g.id = 4;
        g.name = "Goal 4 (-4cm X, -3cm Y, +5cm Z)";
        g.pos_goal = g_orig + Eigen::Vector3d(-0.04, -0.03, 0.05);
        goals.push_back(g);
    }
    // Goal 5: +5cm X, -4cm Y, -3cm Z
    {
        GoalSpec g;
        g.id = 5;
        g.name = "Goal 5 (+5cm X, -4cm Y, -3cm Z)";
        g.pos_goal = g_orig + Eigen::Vector3d(0.05, -0.04, -0.03);
        goals.push_back(g);
    }

    return goals;
}

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <demo_csv> <out_dir> <label> [n_basis=80] [lambda=1e-10] [window=0.05]\n";
        return 1;
    }

    std::string demo_path = argv[1];
    std::string out_dir   = argv[2];
    std::string label     = argv[3];
    int n_basis           = (argc > 4) ? std::stoi(argv[4]) : 80;
    double lambda         = (argc > 5) ? std::stod(argv[5]) : 1e-10;
    double window_sec     = (argc > 6) ? std::stod(argv[6]) : 0.05;

    fs::create_directories(out_dir + "/data");
    fs::create_directories(out_dir + "/weights");

    std::cout << "[" << label << "] Loading demonstration from: " << demo_path << "\n";
    auto demo = haptic_dmp_learning::core::demo_csv_io::readDemoCsv(demo_path);
    std::cout << "  Loaded " << demo.size() << " samples, span: "
              << (demo.back().t - demo.front().t) << " s\n";

    ProDMP prodmp(n_basis, 25.0, 4.6, lambda);
    prodmp.setPositionFilterWindow(window_sec);
    prodmp.setFixGoalToDemoEndpoint(true);

    std::cout << "  Fitting ProDMP (n_basis=" << n_basis << ", lambda=" << lambda
              << ", window=" << window_sec << "s, fix_goal=true)...\n";
    prodmp.learnFromDemonstration(demo);

    std::string weights_file = out_dir + "/weights/" + label + "_prodmp_weights.yaml";
    haptic_dmp_learning::core::prodmp_io::saveProDmpToYaml(prodmp, weights_file);

    Eigen::Vector3d orig_goal = prodmp.goal();
    std::cout << "  Original Goal: [" << orig_goal.transpose() << "]\n";

    // 1. Original Replay (on original goal)
    prodmp.setInitialConditions(prodmp.initTime(), prodmp.initPos(), prodmp.initVel());
    std::vector<double> t_orig;
    std::vector<Eigen::Vector3d> replay_orig_pos;
    t_orig.reserve(demo.size());
    replay_orig_pos.reserve(demo.size());

    double prev_t = demo.front().t;
    for (size_t k = 0; k < demo.size(); ++k) {
        double dt = (k == 0) ? 0.0 : (demo[k].t - prev_t);
        prev_t = demo[k].t;
        Eigen::Vector3d p = prodmp.step(dt);
        t_orig.push_back(demo[k].t - demo.front().t);
        replay_orig_pos.push_back(p);
    }
    std::string replay_orig_path = out_dir + "/data/" + label + "_replay_orig.csv";
    writeReplayCsv(replay_orig_path, t_orig, replay_orig_pos);

    double orig_ep_err = (replay_orig_pos.back() - demo.back().position).norm() * 1000.0;
    std::cout << "  Original Replay endpoint error: " << orig_ep_err << " mm\n";

    // 2. 5 New Goals
    auto goals = create5Goals(orig_goal);
    std::string info_path = out_dir + "/data/" + label + "_goals_info.csv";
    std::ofstream f_info(info_path);
    f_info << "goal_id,name,gx,gy,gz,err_pos_mm\n";

    double sum_err_pos = 0.0;

    for (auto& g : goals) {
        // Rewind and retarget
        prodmp.setInitialConditions(prodmp.initTime(), prodmp.initPos(), prodmp.initVel());
        prodmp.setGoal(g.pos_goal);

        std::vector<double> t_g;
        std::vector<Eigen::Vector3d> rep_g;
        t_g.reserve(demo.size());
        rep_g.reserve(demo.size());

        prev_t = demo.front().t;
        for (size_t k = 0; k < demo.size(); ++k) {
            double dt = (k == 0) ? 0.0 : (demo[k].t - prev_t);
            prev_t = demo[k].t;
            Eigen::Vector3d p = prodmp.step(dt);
            t_g.push_back(demo[k].t - demo.front().t);
            rep_g.push_back(p);
        }

        g.final_pos = rep_g.back();
        g.pos_error_mm = (g.final_pos - g.pos_goal).norm() * 1000.0;
        sum_err_pos += g.pos_error_mm;

        std::string g_csv = out_dir + "/data/" + label + "_replay_goal_" + std::to_string(g.id) + ".csv";
        writeReplayCsv(g_csv, t_g, rep_g);

        f_info << g.id << ",\"" << g.name << "\","
               << g.pos_goal.x() << "," << g.pos_goal.y() << "," << g.pos_goal.z() << ","
               << g.pos_error_mm << "\n";

        std::cout << "  " << g.name << " -> Final pos: [" << g.final_pos.transpose()
                  << "], Pos Err: " << g.pos_error_mm << " mm\n";
    }
    f_info.close();

    double mean_err_pos = sum_err_pos / goals.size();
    std::cout << "  Mean Final Position Error across 5 goals: " << mean_err_pos << " mm\n";

    return 0;
}
