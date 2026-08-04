// Learns DMP + QuaternionDMP ONCE from a base demo, then tests behavior
// when assigned a DIFFERENT goal (position AND orientation) via setGoal(),
// WITHOUT calling learnFromDemonstration a second time.
//
// Target goal is read from the end point of a second CSV (target_demo.csv),
// used ONLY as a reference for the new goal and comparison trajectory.
//
// Usage:
//   ./generalize_test_dmp <base_demo.csv> <target_demo.csv>
//       <output_replay.csv> <summary.csv> <label>
//       [n_basis=20] [alpha_x=4.6] [alpha_z=25] [beta_z=6.25]

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <stdexcept>

#include "haptic_dmp_learning/core/dmp.hpp"
#include "haptic_dmp_learning/core/quaternion_dmp.hpp"
#include "haptic_dmp_learning/core/dmp_io.hpp"
#include "haptic_dmp_learning/core/types.hpp"
#include "metrics.hpp"

using haptic_dmp_learning::core::DMP;
using haptic_dmp_learning::core::QuaternionDMP;
using haptic_dmp_learning::core::Sample;

static std::vector<Sample> loadDemoCsv(const std::string& path) {
    std::ifstream f(path);
    if (!f.good()) {
        throw std::runtime_error("generalize_test_dmp: cannot open " + path);
    }

    std::vector<Sample> demo;
    std::string line;
    std::getline(f, line);  // skip header

    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string field;
        std::vector<double> vals;
        while (std::getline(ss, field, ',')) {
            vals.push_back(std::stod(field));
        }
        if (vals.size() < 8) {
            throw std::runtime_error(
                "generalize_test_dmp: malformed line (expected 8 fields, found " +
                std::to_string(vals.size()) + "): " + line);
        }

        Sample s;
        s.t = vals[0];
        s.position = Eigen::Vector3d(vals[1], vals[2], vals[3]);
        s.orientation = Eigen::Quaterniond(vals[4], vals[5], vals[6], vals[7]);
        demo.push_back(s);
    }

    if (demo.size() < 5) {
        throw std::runtime_error("generalize_test_dmp: demo too short (" +
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

int main(int argc, char** argv) {
    if (argc < 6) {
        std::cerr << "Usage: " << argv[0]
                  << " <base_demo.csv> <target_demo.csv> <output_replay.csv>"
                  << " <summary.csv> <label>"
                  << " [n_basis=20] [alpha_x=4.6] [alpha_z=25] [beta_z=6.25]\n";
        return 1;
    }

    const std::string base_csv     = argv[1];
    const std::string target_csv   = argv[2];
    const std::string output_replay = argv[3];
    const std::string summary_csv  = argv[4];
    const std::string label        = argv[5];
    const int n_basis     = (argc >= 7)  ? std::stoi(argv[6]) : 20;
    const double alpha_x  = (argc >= 8)  ? std::stod(argv[7]) : 4.6;
    const double alpha_z  = (argc >= 9)  ? std::stod(argv[8]) : 25.0;
    const double beta_z   = (argc >= 10) ? std::stod(argv[9]) : 6.25;

    try {
        std::cout << "[" << label << "] Learning from " << base_csv << " (once)...\n";
        std::vector<Sample> base_demo = loadDemoCsv(base_csv);

        DMP dmp(n_basis, alpha_x, alpha_z, beta_z);
        QuaternionDMP qdmp(n_basis, alpha_x, alpha_z, beta_z);
        dmp.learnFromDemonstration(base_demo);
        qdmp.learnFromDemonstration(base_demo);

        std::cout << "  Loading target (as new goal + reference) from "
                  << target_csv << "...\n";
        std::vector<Sample> target_demo = loadDemoCsv(target_csv);

        dmp.setGoal(target_demo.back().position);
        qdmp.setGoal(target_demo.back().orientation);

        std::cout << "  scale_reliable after setGoal(): x=" << dmp.isScaleReliable(0)
                  << " y=" << dmp.isScaleReliable(1)
                  << " z=" << dmp.isScaleReliable(2) << "\n";

        dmp.reset();
        qdmp.reset();

        std::vector<double> t_out;
        std::vector<Eigen::Vector3d> ref_pos, replay_pos;
        std::vector<Eigen::Quaterniond> ref_orient, replay_orient;
        const size_t N = target_demo.size();
        t_out.reserve(N); ref_pos.reserve(N); replay_pos.reserve(N);
        ref_orient.reserve(N); replay_orient.reserve(N);

        double prev_t = target_demo.front().t;
        for (size_t k = 0; k < N; ++k) {
            double dt = (k == 0) ? 0.0 : (target_demo[k].t - prev_t);
            prev_t = target_demo[k].t;

            Eigen::Vector3d p = (k == 0) ? dmp.step(0.0) : dmp.step(dt);
            Eigen::Quaterniond q = (k == 0) ? qdmp.step(0.0) : qdmp.step(dt);

            t_out.push_back(target_demo[k].t - target_demo.front().t);
            ref_pos.push_back(target_demo[k].position);
            ref_orient.push_back(target_demo[k].orientation);
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

        appendToSummaryCsv(summary_csv, label, tf, of,
                           endpoint_pos_error, endpoint_orient_error,
                           dmp.scaleReliable());

        std::cout << "  Saved: " << output_replay << "\n";
        std::cout << "  Summary appended to " << summary_csv << "\n";

    } catch (const std::exception& e) {
        std::cerr << "ERROR [" << label << "]: " << e.what() << "\n";
        return 1;
    }

    return 0;
}

