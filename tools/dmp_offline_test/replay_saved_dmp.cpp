// Carica un dmp_weights.yaml gia' salvato (formato combinato posizione +
// orientamento) e genera il replay, in un CSV pronto da confrontare con la
// demo grezza registrata. Nessun ROS2 richiesto: gira sull'host, con solo
// Eigen3 + yaml-cpp.
//
// Uso: ./replay_saved_dmp [percorso_dmp_weights.yaml] [durata_extra_secondi]

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

    std::cout << "Caricato (" << yaml_path << "):\n"
              << "  [Posizione] n_basis: " << dmp.nBasis() << " | tau: " << dmp.tau() << " s\n"
              << "    y0:   [" << dmp.y0().transpose() << "]\n"
              << "    goal: [" << dmp.goal().transpose() << "]\n"
              << "  [Orientamento] n_basis: " << qdmp.nBasis() << " | tau: " << qdmp.tau() << " s\n"
              << "    q0:   (w=" << qdmp.q0().w() << ", " << qdmp.q0().vec().transpose() << ")\n"
              << "    goal: (w=" << qdmp.goal().w() << ", " << qdmp.goal().vec().transpose() << ")\n";

    if (std::abs(dmp.tau() - qdmp.tau()) > 1e-6) {
        std::cout << "ATTENZIONE: tau posizione (" << dmp.tau() << ") e tau orientamento (" << qdmp.tau()
                  << ") non coincidono - controlla i timestamp della demo originale.\n";
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
    std::cout << "Salvato data/replay_from_yaml.csv (" << rt.size() << " campioni, durata " << duration << "s)\n";

    return 0;
}
