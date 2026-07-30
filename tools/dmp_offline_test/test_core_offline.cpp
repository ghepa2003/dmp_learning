// Standalone test: no ROS2, no Docker, no device needed.
// Esercita il codice REALE del pacchetto (core::DMP, core::QuaternionDMP,
// core::dmp_io), su PIU' traiettorie sintetiche diverse, calcolando metriche
// di fedelta' per ciascuna invece di un solo giudizio visivo.

#include <fstream>
#include <iostream>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "haptic_dmp_learning/core/types.hpp"
#include "haptic_dmp_learning/core/dmp.hpp"
#include "haptic_dmp_learning/core/quaternion_dmp.hpp"
#include "haptic_dmp_learning/core/dmp_io.hpp"
#include "metrics.hpp"

using haptic_dmp_learning::core::Sample;
using haptic_dmp_learning::core::DMP;
using haptic_dmp_learning::core::QuaternionDMP;
namespace m = dmp_tools::metrics;

static double minJerk(double a, double b, double s) {
    double s3 = s * s * s, s4 = s3 * s, s5 = s4 * s;
    return a + (b - a) * (10 * s3 - 15 * s4 + 6 * s5);
}

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

// Definizione di una traiettoria sintetica: nome, durata, e funzioni
// posizione/orientamento parametrizzate dal tempo normalizzato s in [0,1].
struct TrajectoryDef {
    std::string name;
    double duration;
    std::function<Eigen::Vector3d(double)> position;
    std::function<Eigen::Quaterniond(double)> orientation;
    // Offset di goal usato per il test di generalizzazione (posizione, e
    // angolo/asse per l'orientamento) - applicato SOPRA il goal della demo.
    Eigen::Vector3d new_goal_offset;
    double new_goal_yaw_deg;
};

int main() {
    const double dt = 0.001;  // 1000 Hz, come il Geomagic Touch

    std::vector<TrajectoryDef> trajectories;

    // --- 1. Reach semplice: nessuna rotazione, nessuna gobba - il caso piu'
    //        facile, ci si aspetta fedelta' molto alta ---
    {
        Eigen::Vector3d p0(0.0, 0.0, 0.0), pg(0.20, 0.10, 0.0);
        TrajectoryDef def;
        def.name = "reach_semplice";
        def.duration = 1.5;
        def.position = [p0, pg](double s) {
            return Eigen::Vector3d(minJerk(p0.x(), pg.x(), s), minJerk(p0.y(), pg.y(), s), minJerk(p0.z(), pg.z(), s));
        };
        def.orientation = [](double) { return Eigen::Quaterniond::Identity(); };
        def.new_goal_offset = Eigen::Vector3d(0.03, -0.02, 0.01);
        def.new_goal_yaw_deg = 15.0;
        trajectories.push_back(def);
    }

    // --- 2. Reach + sollevamento + rotazione di pitch - il caso "misto" gia'
    //        usato nei test precedenti ---
    {
        Eigen::Vector3d p0(0.0, 0.0, 0.0), pg(0.25, 0.15, 0.05);
        double lift = 0.08;
        Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
        Eigen::Quaterniond qg(Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitY()));
        TrajectoryDef def;
        def.name = "reach_lift_pitch";
        def.duration = 2.0;
        def.position = [p0, pg, lift](double s) {
            return Eigen::Vector3d(minJerk(p0.x(), pg.x(), s), minJerk(p0.y(), pg.y(), s),
                                    minJerk(p0.z(), pg.z(), s) + lift * std::sin(M_PI * s));
        };
        def.orientation = [q0, qg](double s) { return q0.slerp(minJerk(0.0, 1.0, s), qg); };
        def.new_goal_offset = Eigen::Vector3d(0.05, -0.08, 0.03);
        def.new_goal_yaw_deg = 60.0;
        trajectories.push_back(def);
    }

    // --- 3. Rotazione pura: spostamento posizionale minimo (dG piccolo di
    //        proposito) con rotazione ampia - pensato per innescare il
    //        guardrail A (ampiezza/dG) sulla posizione, e testare la
    //        QuaternionDMP isolata dall'effetto della traslazione ---
    {
        Eigen::Vector3d p0(0.10, 0.10, 0.05), pg(0.105, 0.098, 0.052);  // dG piccolissimo
        Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
        Eigen::Quaterniond qg(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()));  // 90 deg roll
        TrajectoryDef def;
        def.name = "rotazione_pura";
        def.duration = 1.5;
        def.position = [p0, pg](double s) {
            return Eigen::Vector3d(minJerk(p0.x(), pg.x(), s), minJerk(p0.y(), pg.y(), s), minJerk(p0.z(), pg.z(), s));
        };
        def.orientation = [q0, qg](double s) { return q0.slerp(minJerk(0.0, 1.0, s), qg); };
        def.new_goal_offset = Eigen::Vector3d(0.06, 0.04, -0.03);  // spostamento GRANDE rispetto al dG originale
        def.new_goal_yaw_deg = 30.0;
        trajectories.push_back(def);
    }

    // --- 4. Traiettoria complessa multi-segmento: profilo quasi a gradino,
    //        pensato per riprodurre il limite osservato su hardware reale
    //        (basi gaussiane che non seguono bene transizioni rapide) ---
    {
        Eigen::Vector3d p0(0.0, 0.0, 0.0), p1(0.15, 0.05, 0.02), pg(0.15, 0.05, -0.10);
        Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
        Eigen::Quaterniond qg(Eigen::AngleAxisd(M_PI / 6.0, Eigen::Vector3d::UnitZ()));
        TrajectoryDef def;
        def.name = "reach_complesso_gradino";
        def.duration = 3.0;
        def.position = [p0, p1, pg](double s) {
            // Due segmenti minimum-jerk in sequenza: p0->p1 nel primo 40% del
            // tempo, plateau, poi p1->pg nel restante 40% - crea transizioni
            // piu' brusche di quelle di un singolo reach fluido.
            if (s < 0.4) {
                return Eigen::Vector3d(minJerk(p0.x(), p1.x(), s / 0.4), minJerk(p0.y(), p1.y(), s / 0.4),
                                        minJerk(p0.z(), p1.z(), s / 0.4));
            } else if (s < 0.6) {
                return p1;
            } else {
                double s2 = (s - 0.6) / 0.4;
                return Eigen::Vector3d(minJerk(p1.x(), pg.x(), s2), minJerk(p1.y(), pg.y(), s2),
                                        minJerk(p1.z(), pg.z(), s2));
            }
        };
        def.orientation = [q0, qg](double s) { return q0.slerp(minJerk(0.0, 1.0, s), qg); };
        def.new_goal_offset = Eigen::Vector3d(0.04, -0.03, 0.02);
        def.new_goal_yaw_deg = 20.0;
        trajectories.push_back(def);
    }

    const std::string summary_csv = "metrics_summary.csv";
    std::remove(summary_csv.c_str());  // riparti da un riassunto pulito ad ogni esecuzione

    for (const auto& traj : trajectories) {
        std::cout << "\n================ Traiettoria: " << traj.name << " ================\n";

        // --- Genera la demo sintetica ---
        std::vector<Sample> demo;
        std::vector<double> demo_t;
        std::vector<Eigen::Vector3d> demo_p;
        std::vector<Eigen::Quaterniond> demo_q;
        for (double t = 0.0; t <= traj.duration + 1e-9; t += dt) {
            double s = t / traj.duration;
            Eigen::Vector3d pos = traj.position(s);
            Eigen::Quaterniond quat = traj.orientation(s);
            Sample sp; sp.t = t; sp.position = pos; sp.orientation = quat;
            demo.push_back(sp);
            demo_t.push_back(t); demo_p.push_back(pos); demo_q.push_back(quat);
        }
        writeCsv("demo_original_" + traj.name + ".csv", demo_t, demo_p, demo_q);

        // --- Impara ---
        DMP dmp(20, 4.6, 25.0, 6.25);
        dmp.learnFromDemonstration(demo);
        QuaternionDMP qdmp(20, 4.6, 25.0, 6.25);
        qdmp.learnFromDemonstration(demo);

        haptic_dmp_learning::core::dmp_io::saveToYaml(dmp, qdmp, "dmp_weights_" + traj.name + ".yaml");

        // --- Replay con lo STESSO goal: qui ha senso confrontare la forma ---
        dmp.reset(); qdmp.reset();
        std::vector<double> rt; std::vector<Eigen::Vector3d> rp; std::vector<Eigen::Quaterniond> rq;
        for (double t = 0.0; t <= traj.duration + 1e-9; t += dt) {
            rp.push_back(dmp.step(dt));
            rq.push_back(qdmp.step(dt));
            rt.push_back(t);
        }
        writeCsv("replay_same_goal_" + traj.name + ".csv", rt, rp, rq);

        auto pos_fid = m::computeTrajectoryFidelity(demo_p, rp);
        auto orient_fid = m::computeOrientationFidelity(demo_q, rq);
        std::cout << "Replay stesso goal:\n";
        m::printReport(traj.name, pos_fid);
        m::printReport(traj.name, orient_fid);
        m::appendToSummaryCsv(summary_csv, traj.name + "_same_goal", pos_fid, orient_fid,
                               m::computeEndpointError(rp.back(), demo_p.back()),
                               m::computeAngularEndpointError(rq.back(), demo_q.back()),
                               dmp.scaleReliable());

        // --- Replay con goal SPOSTATO: qui ha senso solo l'errore finale ---
        Eigen::Vector3d new_pos_goal = demo_p.back() + traj.new_goal_offset;
        Eigen::Quaterniond new_quat_goal =
            Eigen::Quaterniond(Eigen::AngleAxisd(traj.new_goal_yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ())) *
            demo_q.back();

        dmp.reset(); dmp.setGoal(new_pos_goal);
        qdmp.reset(); qdmp.setGoal(new_quat_goal);

        std::vector<double> rt2; std::vector<Eigen::Vector3d> rp2; std::vector<Eigen::Quaterniond> rq2;
        for (double t = 0.0; t <= traj.duration + 1e-9; t += dt) {
            rp2.push_back(dmp.step(dt));
            rq2.push_back(qdmp.step(dt));
            rt2.push_back(t);
        }
        writeCsv("replay_new_goal_" + traj.name + ".csv", rt2, rp2, rq2);

        double endpoint_pos_err = m::computeEndpointError(rp2.back(), new_pos_goal);
        double endpoint_orient_err = m::computeAngularEndpointError(rq2.back(), new_quat_goal);
        std::cout << "Replay goal spostato:\n";
        m::printEndpointError(traj.name, endpoint_pos_err, endpoint_orient_err);

        for (int d = 0; d < 3; ++d) {
            if (!dmp.isScaleReliable(d)) {
                std::cout << "  ATTENZIONE: rescaling automatico bloccato sulla dimensione " << d
                          << " (ampiezza/dG oltre soglia) - vedi guardrail A\n";
            }
        }

        m::appendToSummaryCsv(summary_csv, traj.name + "_new_goal", m::TrajectoryFidelity{}, m::OrientationFidelity{},
                               endpoint_pos_err, endpoint_orient_err, dmp.scaleReliable());
    }

    std::cout << "\n\nTutte le traiettorie completate. Riassunto in " << summary_csv << "\n";
    return 0;
}
