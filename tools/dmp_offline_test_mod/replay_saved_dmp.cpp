// Carica un dmp_weights.yaml gia' salvato (da una sessione reale col
// Geomagic Touch) e genera il replay, in un CSV pronto da confrontare con la
// demo grezza registrata (dmp_demo_recorded.csv). Nessun ROS2 richiesto qui:
// gira sull'host, con solo Eigen3 + yaml-cpp.
//
// Uso: ./replay_saved_dmp <percorso_dmp_weights.yaml> [durata_extra_secondi]

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "haptic_dmp_learning/core/dmp.hpp"
#include "haptic_dmp_learning/core/dmp_io.hpp"

using haptic_dmp_learning::core::DMP;

static void writeCsv(const std::string& path, const std::vector<double>& t,
                      const std::vector<Eigen::Vector3d>& p) {
    std::ofstream f(path);
    f << "t,x,y,z\n";
    for (size_t k = 0; k < t.size(); ++k) {
        f << t[k] << "," << p[k].x() << "," << p[k].y() << "," << p[k].z() << "\n";
    }
}

int main(int argc, char** argv) {
    std::string yaml_path = "/home/lorenzo/thesis_ws/dmp_weights.yaml";
    if (argc >= 2) {
        yaml_path = argv[1];
    }
    double extra = (argc >= 3) ? std::stod(argv[2]) : 0.0;

    std::ifstream f_check(yaml_path);
    if (!f_check.good()) {
        if (std::ifstream("dmp_weights.yaml").good()) {
            yaml_path = "dmp_weights.yaml";
        }
    }

    DMP dmp = haptic_dmp_learning::core::dmp_io::loadFromYaml(yaml_path);
    std::cout << "Caricato (" << yaml_path << "):\n"
              << "  n_basis: " << dmp.nBasis() << "\n"
              << "  alpha_x: " << dmp.alphaX() << "\n"
              << "  alpha_z: " << dmp.alphaZ() << "\n"
              << "  beta_z:  " << dmp.betaZ() << "\n"
              << "  tau:     " << dmp.tau() << " s\n"
              << "  y0:      [" << dmp.y0().transpose() << "]\n"
              << "  goal:    [" << dmp.goal().transpose() << "]\n";

    const double dt = 0.001;
    const double duration = dmp.tau() + extra;

    dmp.reset();
    std::vector<double> rt;
    std::vector<Eigen::Vector3d> rp;
    for (double t = 0.0; t <= duration + 1e-9; t += dt) {
        rp.push_back(dmp.step(dt));
        rt.push_back(t);
    }

    writeCsv("replay_from_yaml.csv", rt, rp);
    std::cout << "Salvato replay_from_yaml.csv (" << rt.size() << " campioni, durata " << duration << "s)\n";

    return 0;
}
