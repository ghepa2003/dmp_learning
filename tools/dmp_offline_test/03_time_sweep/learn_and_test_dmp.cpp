// Impara una DMP + QuaternionDMP da un CSV di demo, fa il replay IN-PROCESS
// (senza passare da un giro di salvataggio/ricarica YAML intermedio, per non
// perdere lo stato di scale_reliable_), e calcola le metriche di fedelta'
// riusando le funzioni gia' esistenti in metrics.cpp/hpp -- niente logica di
// metriche duplicata altrove.
//
// Scrive comunque il YAML dei pesi (utile per ispezione/riuso) e il CSV del
// replay (usato poi da plot_dmp_test.py solo per i grafici per-singola-prova,
// non per ricalcolare le metriche).
//
// Uso:
//   ./learn_and_test_dmp <input_demo.csv> <output_weights.yaml>
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

// NOTA: assume che Sample abbia i campi .t (double), .position (Eigen::Vector3d),
// .orientation (Eigen::Quaterniond) -- stesso pattern gia' visto in
// dmp.cpp/quaternion_dmp.cpp. Adattare solo qui se types.hpp usa nomi diversi.
static std::vector<Sample> loadDemoCsv(const std::string& path) {
    std::ifstream f(path);
    if (!f.good()) {
        throw std::runtime_error("learn_and_test_dmp: impossibile aprire " + path);
    }

    std::vector<Sample> demo;
    std::string line;
    std::getline(f, line);  // salta l'header (t,x,y,z,qw,qx,qy,qz)

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
                "learn_and_test_dmp: riga malformata (attesi 8 campi, trovati " +
                std::to_string(vals.size()) + "): " + line);
        }

        Sample s;
        s.t = vals[0];
        s.position = Eigen::Vector3d(vals[1], vals[2], vals[3]);
        s.orientation = Eigen::Quaterniond(vals[4], vals[5], vals[6], vals[7]);
        demo.push_back(s);
    }

    if (demo.size() < 5) {
        throw std::runtime_error("learn_and_test_dmp: demo troppo corta (" +
                                  std::to_string(demo.size()) + " campioni)");
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
        std::cerr << "Uso: " << argv[0]
                  << " <input_demo.csv> <output_weights.yaml> <output_replay.csv>"
                  << " <summary.csv> <label>"
                  << " [n_basis|-] [alpha_x|-] [alpha_z|-] [beta_z|-]"
                  << " [feature_flags_path='']\n"
                  << "  '-' o argomento omesso = usa il default reale della classe DMP\n";
        return 1;
    }

    const std::string input_csv    = argv[1];
    const std::string output_yaml  = argv[2];
    const std::string output_replay = argv[3];
    const std::string summary_csv  = argv[4];
    const std::string label        = argv[5];
    // Sonde di default: leggono i valori REALMENTE compilati in DMP/QuaternionDMP
    // (dmp.hpp/quaternion_dmp.hpp) invece di riscriverli qui come costanti --
    // se cambi un default nel core, questo tool lo segue senza modifiche.
    const DMP dmp_defaults;

    // Sentinella "-" (o argomento assente) = "usa il default reale", non un
    // numero duplicato scritto a mano.
    auto parseOrDefault = [](const char* arg, double fallback) -> double {
        std::string s = arg ? arg : "-";
        return (s == "-") ? fallback : std::stod(s);
    };
    auto parseOrDefaultInt = [](const char* arg, int fallback) -> int {
        std::string s = arg ? arg : "-";
        return (s == "-") ? fallback : std::stoi(s);
    };

    const int n_basis     = parseOrDefaultInt((argc >= 7)  ? argv[6] : nullptr, dmp_defaults.nBasis());
    const double alpha_x  = parseOrDefault((argc >= 8)  ? argv[7] : nullptr, dmp_defaults.alphaX());
    const double alpha_z  = parseOrDefault((argc >= 9)  ? argv[8] : nullptr, dmp_defaults.alphaZ());
    const double beta_z   = parseOrDefault((argc >= 10) ? argv[9] : nullptr, dmp_defaults.betaZ());
    // Vuoto (default) = nessun feature flag applicato, comportamento
    // invariato (LWR indipendente). Se fornito, deve puntare allo STESSO file
    // che legge haptic_dmp_wrapper_node.cpp in produzione -- non una copia.
    std::string feature_flags_path = (argc >= 11) ? argv[10] : "";
    if (feature_flags_path.empty()) {
        const std::string default_path = "../../src/haptic_dmp_learning/config/dmp_features.yaml";
        std::ifstream check_f(default_path);
        if (check_f.good()) {
            feature_flags_path = default_path;
        }
    }

    try {
        std::cout << "[" << label << "] Carico demo da " << input_csv << "...\n";
        std::vector<Sample> demo = loadDemoCsv(input_csv);
        const double duration = demo.back().t - demo.front().t;
        std::cout << "  " << demo.size() << " campioni, durata " << duration << "s\n";

        DMP dmp(n_basis, alpha_x, alpha_z, beta_z);
        QuaternionDMP qdmp(n_basis, alpha_x, alpha_z, beta_z);

        if (!feature_flags_path.empty()) {
            haptic_dmp_learning::core::dmp_io::applyFeatureConfig(feature_flags_path, dmp, qdmp);
            std::cout << "  [Config YAML] Caricato " << feature_flags_path
                      << " -> Metodo di regressione: " << (dmp.ridgeRegressionEnabled() ? "RIDGE" : "INDEPENDENT_LWR")
                      << "\n";
        } else {
            std::cout << "  [Config YAML] Nessun file di configurazione -> Metodo di regressione: INDEPENDENT_LWR (default)\n";
        }

        std::cout << "  Apprendimento (n_basis=" << n_basis << ")...\n";
        dmp.learnFromDemonstration(demo);
        qdmp.learnFromDemonstration(demo);

        haptic_dmp_learning::core::dmp_io::saveToYaml(dmp, qdmp, output_yaml);

        // --- Replay IN-PROCESS, stesso numero di campioni e stessa base
        // temporale della demo (cosi' il confronto per indice fatto da
        // metrics.cpp resta valido: qui non c'e' jitter/campionamento
        // irregolare come nelle demo reali, quindi il confronto per indice
        // e' sicuro; se in futuro riuserai questo stesso tool su demo REALI
        // andra' introdotto un riallineamento per timestamp, come gia' fatto
        // in plot_real_demo.py). ---
        dmp.reset();
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

            Eigen::Vector3d p = (k == 0) ? dmp.step(0.0) : dmp.step(dt);
            Eigen::Quaterniond q = (k == 0) ? qdmp.step(0.0) : qdmp.step(dt);

            t_out.push_back(demo[k].t - demo.front().t);
            ref_pos.push_back(demo[k].position);
            ref_orient.push_back(demo[k].orientation);
            replay_pos.push_back(p);
            replay_orient.push_back(q);
        }

        writeReplayCsv(output_replay, t_out, replay_pos, replay_orient);

        // --- Metriche, riusando metrics.cpp/hpp cosi' come sono ---
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

        std::cout << "  Salvati: " << output_yaml << ", " << output_replay << "\n";
        std::cout << "  Riepilogo aggiunto a " << summary_csv << "\n";

    } catch (const std::exception& e) {
        std::cerr << "ERRORE [" << label << "]: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
