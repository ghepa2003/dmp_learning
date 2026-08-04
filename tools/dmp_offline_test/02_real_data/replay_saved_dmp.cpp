// Loads a saved dmp_weights.yaml (combined position + orientation format)
// and generates replay CSV ready for comparison with raw demo.
// No ROS2 required: host-only with Eigen3 + yaml-cpp.
//
// Usage: ./replay_saved_dmp [path_to_dmp_weights.yaml] [extra_duration_sec]

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <filesystem>

#include "haptic_dmp_learning/core/dmp.hpp"
#include "haptic_dmp_learning/core/quaternion_dmp.hpp"
#include "haptic_dmp_learning/core/dmp_io.hpp"

using haptic_dmp_learning::core::DMP;
using haptic_dmp_learning::core::QuaternionDMP;

static void writeCsv(const std::string& path, const std::vector<double>& t,
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
    std::filesystem::create_directories("data");

    std::string yaml_path = "/home/lorenzo/thesis_ws/dmp_weights.yaml";
    if (argc >= 2) {
        yaml_path = argv[1];
    }
    double extra = (argc >= 3) ? std::stod(argv[2]) : 0.0;

    std::ifstream f_check(yaml_path);
    if (!f_check.good()) {
        if (std::ifstream("weights/dmp_weights.yaml").good()) {
            yaml_path = "weights/dmp_weights.yaml";
        } else if (std::ifstream("dmp_weights.yaml").good()) {
            yaml_path = "dmp_weights.yaml";
        }
    }

    DMP dmp;
    QuaternionDMP qdmp;
    haptic_dmp_learning::core::dmp_io::loadFromYaml(yaml_path, dmp, qdmp);

    std::cout << "Loaded (" << yaml_path << "):\n"
              << "  [Position] n_basis: " << dmp.nBasis() << " | tau: " << dmp.tau() << " s\n"
              << "    y0:   [" << dmp.y0().transpose() << "]\n"
              << "    goal: [" << dmp.goal().transpose() << "]\n"
              << "  [Orientation] n_basis: " << qdmp.nBasis() << " | tau: " << qdmp.tau() << " s\n"
              << "    q0:   (w=" << qdmp.q0().w() << ", " << qdmp.q0().vec().transpose() << ")\n"
              << "    goal: (w=" << qdmp.goal().w() << ", " << qdmp.goal().vec().transpose() << ")\n";

    if (std::abs(dmp.tau() - qdmp.tau()) > 1e-6) {
        std::cout << "WARNING: position tau (" << dmp.tau() << ") and orientation tau (" << qdmp.tau()
                  << ") do not match - check original demo timestamps.\n";
    }

    const double dt = 0.001;
    const double duration = dmp.tau() + extra;

    dmp.reset();
    qdmp.reset();
    std::vector<double> rt;
    std::vector<Eigen::Vector3d> rp;
    std::vector<Eigen::Quaterniond> rq;
    for (double t = 0.0; t <= duration + 1e-9; t += dt) {
        rp.push_back(dmp.step(dt));
        rq.push_back(qdmp.step(dt));
        rt.push_back(t);
    }

    writeCsv("data/replay_from_yaml.csv", rt, rp, rq);
    std::cout << "Saved data/replay_from_yaml.csv (" << rt.size() << " samples, duration " << duration << "s)\n";

    return 0;
}
