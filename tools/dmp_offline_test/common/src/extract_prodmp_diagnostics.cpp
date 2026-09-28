#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <Eigen/Dense>
#include "haptic_dmp_learning/core/prodmp.hpp"
#include "haptic_dmp_learning/core/demo_csv_io.hpp"
#include "haptic_dmp_learning/core/filter_utils.hpp"
#include "metrics.hpp"

using haptic_dmp_learning::core::ProDMP;
using haptic_dmp_learning::core::Sample;

int main() {
    const std::string demo_path = "../../demo_raw_trajC.csv";
    std::vector<Sample> demo = haptic_dmp_learning::core::demo_csv_io::readDemoCsv(demo_path);
    const int N = static_cast<int>(demo.size());
    const int num_basis = 200;
    const int P = num_basis + 1;
    const double alpha = 25.0;
    const double alpha_x = 4.6;
    const double window_sec = 0.20;
    const double tau = demo.back().t - demo.front().t;
    const double t0 = demo.front().t;

    // Filter positions
    std::vector<Eigen::Vector3d> raw_pos(N);
    std::vector<double> ts(N);
    for (int k = 0; k < N; ++k) {
        raw_pos[k] = demo[k].position;
        ts[k] = demo[k].t;
    }
    std::vector<Eigen::Vector3d> pos_for_fit =
        haptic_dmp_learning::core::filter_utils::movingAverageSmooth(raw_pos, ts, window_sec);

    Eigen::VectorXd centers(num_basis);
    Eigen::VectorXd widths(num_basis);
    for (int i = 0; i < num_basis; ++i) {
        const double frac = (num_basis == 1) ? 0.0 : (static_cast<double>(i) / (num_basis - 1));
        centers(i) = std::exp(-alpha_x * frac);
    }
    for (int i = 0; i < num_basis; ++i) {
        if (num_basis == 1) {
            widths(i) = 1.0;
        } else if (i == num_basis - 1) {
            widths(i) = widths(i - 1);
        } else {
            const double diff = centers(i) - centers(i + 1);
            widths(i) = (diff > 1e-12) ? (1.0 / (diff * diff)) : 1.0;
        }
    }

    auto evalBasis = [&](double x, Eigen::VectorXd& psi_out) {
        double sum = 0.0;
        for (int i = 0; i < num_basis; ++i) {
            const double d = x - centers(i);
            const double val = std::exp(-0.5 * widths(i) * d * d);
            psi_out(i) = val;
            sum += val;
        }
        if (sum > 1e-12) psi_out /= sum;
    };

    Eigen::MatrixXd H(N, P);
    Eigen::MatrixXd B(N, 3);

    const Eigen::Vector3d init_pos = pos_for_fit.front();
    const double dt_init = (N > 1) ? std::max(demo[1].t - t0, 1e-6) : 0.001;
    const Eigen::Vector3d init_vel = (pos_for_fit[1] - pos_for_fit[0]) / dt_init;
    const Eigen::Vector3d init_vel_scaled = init_vel * tau;

    const double det_ic = 1.0;
    const double dy1_0 = -0.5 * alpha;
    const double dy2_0 = 1.0;

    Eigen::VectorXd p1 = Eigen::VectorXd::Zero(num_basis);
    Eigen::VectorXd p2 = Eigen::VectorXd::Zero(num_basis);
    Eigen::VectorXd dp1_prev(num_basis);
    Eigen::VectorXd dp2_prev(num_basis);
    Eigen::VectorXd psi(num_basis);

    {
        evalBasis(1.0, psi);
        for (int i = 0; i < num_basis; ++i) {
            dp1_prev(i) = 0.0;
            dp2_prev(i) = psi(i);
        }
    }

    double s_prev = 0.0;
    for (int k = 0; k < N; ++k) {
        const double s = (demo[k].t - t0) / tau;
        const double ds = s - s_prev;
        const double x = std::exp(-alpha_x * s);
        evalBasis(x, psi);
        const double e = std::exp(0.5 * alpha * s);

        for (int i = 0; i < num_basis; ++i) {
            const double dp1 = s * e * x * psi(i);
            const double dp2 = e * x * psi(i);
            p1(i) += 0.5 * (dp1_prev(i) + dp1) * ds;
            p2(i) += 0.5 * (dp2_prev(i) + dp2) * ds;
            dp1_prev(i) = dp1;
            dp2_prev(i) = dp2;
        }

        const double y1 = std::exp(-0.5 * alpha * s);
        const double y2 = s * y1;

        for (int i = 0; i < num_basis; ++i) {
            H(k, i) = y2 * p2(i) - y1 * p1(i);
        }

        const double q1 = (0.5 * alpha * s - 1.0) * e + 1.0;
        const double q2 = 0.5 * alpha * (e - 1.0);
        const double pos_g = y2 * q2 - y1 * q1;
        H(k, P - 1) = pos_g;

        const double xi1 = (dy2_0 * y1 - dy1_0 * y2) / det_ic;
        const double xi2 = y2;

        for (int d = 0; d < 3; ++d) {
            double target = pos_for_fit[k](d) - (init_pos(d) * xi1 + init_vel_scaled(d) * xi2);
            target -= init_pos(d) * pos_g; // relative goal
            B(k, d) = target;
        }
        s_prev = s;
    }

    // -------------------------------------------------------------
    // FIGURA 1 DATA: Column norms before and after preconditioning
    // -------------------------------------------------------------
    Eigen::VectorXd norm_before_maxabs(P);
    Eigen::VectorXd norm_before_l2(P);
    for (int j = 0; j < P; ++j) {
        norm_before_maxabs(j) = H.col(j).cwiseAbs().maxCoeff();
        norm_before_l2(j) = H.col(j).norm();
    }

    constexpr double kScaleFloorRatio = 5e-6;
    const double overall_max_abs = H.cwiseAbs().maxCoeff();
    Eigen::VectorXd scale(P);
    Eigen::MatrixXd H_scaled = H;

    for (int i = 0; i < P; ++i) {
        const double col_max_abs = norm_before_maxabs(i);
        const bool below_floor =
            (overall_max_abs < 1e-300) || (col_max_abs < kScaleFloorRatio * overall_max_abs);
        scale(i) = below_floor ? 1.0 : (1.0 / col_max_abs);
        H_scaled.col(i) *= scale(i);
    }

    Eigen::VectorXd norm_after_maxabs(P);
    Eigen::VectorXd norm_after_l2(P);
    for (int j = 0; j < P; ++j) {
        norm_after_maxabs(j) = H_scaled.col(j).cwiseAbs().maxCoeff();
        norm_after_l2(j) = H_scaled.col(j).norm();
    }

    std::ofstream f1("/home/lorenzo/thesis_ws/slides_material/data_fig1_column_norms.csv");
    f1 << "col_idx,is_goal_col,norm_before_maxabs,norm_before_l2,norm_after_maxabs,norm_after_l2,scaled_applied\n";
    for (int j = 0; j < P; ++j) {
        f1 << j << "," << (j == P - 1 ? 1 : 0) << ","
           << norm_before_maxabs(j) << "," << norm_before_l2(j) << ","
           << norm_after_maxabs(j) << "," << norm_after_l2(j) << ","
           << (scale(j) != 1.0 ? 1 : 0) << "\n";
    }
    f1.close();

    std::cout << "=== FIGURA 1 KEY NUMBERS (Column Norms of H) ===\n";
    std::cout << "BEFORE PRECONDITIONING (Unscaled):\n";
    std::cout << "  Min col max-abs: " << norm_before_maxabs.minCoeff() << "\n";
    std::cout << "  Max col max-abs: " << norm_before_maxabs.maxCoeff() << "\n";
    std::cout << "  Ratio Max/Min:   " << (norm_before_maxabs.maxCoeff() / norm_before_maxabs.minCoeff()) << "\n";
    std::cout << "  Min col L2 norm: " << norm_before_l2.minCoeff() << "\n";
    std::cout << "  Max col L2 norm: " << norm_before_l2.maxCoeff() << "\n";
    std::cout << "  Ratio L2 Max/Min:" << (norm_before_l2.maxCoeff() / norm_before_l2.minCoeff()) << "\n";

    std::cout << "AFTER PRECONDITIONING (with kScaleFloorRatio=5e-6):\n";
    std::cout << "  Min col max-abs: " << norm_after_maxabs.minCoeff() << "\n";
    std::cout << "  Max col max-abs: " << norm_after_maxabs.maxCoeff() << "\n";
    std::cout << "  Ratio Max/Min:   " << (norm_after_maxabs.maxCoeff() / norm_after_maxabs.minCoeff()) << "\n";
    std::cout << "  Min col L2 norm: " << norm_after_l2.minCoeff() << "\n";
    std::cout << "  Max col L2 norm: " << norm_after_l2.maxCoeff() << "\n";
    std::cout << "  Ratio L2 Max/Min:" << (norm_after_l2.maxCoeff() / norm_after_l2.minCoeff()) << "\n";

    // -------------------------------------------------------------
    // FIGURA 2 DATA: Ridge Lambda Sweep (Pre-fix vs Post-fix)
    // -------------------------------------------------------------
    std::vector<double> lambda_list = {
        1e-12, 1e-11, 1e-10, 1e-9, 1e-8, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1.0
    };

    std::ofstream f2("/home/lorenzo/thesis_ws/slides_material/data_fig2_ridge_sweep_before_after.csv");
    f2 << "ridge_lambda,rmse_prefix_mm,rmse_postfix_mm,max_abs_w_prefix,max_abs_w_postfix\n";

    std::cout << "\n=== FIGURA 2 KEY NUMBERS (Ridge Lambda Sweep Before/After Fix) ===\n";
    std::cout << "lambda\t\tPre-fix RMSE [mm]\tPost-fix RMSE [mm]\n";

    Eigen::MatrixXd HtH_raw = H.transpose() * H;
    Eigen::MatrixXd HtB_raw = H.transpose() * B;

    for (double lam : lambda_list) {
        // 1. Post-fix: run actual ProDMP with this lambda
        ProDMP prodmp_post(num_basis, alpha, alpha_x, lam);
        prodmp_post.setPositionFilterWindow(window_sec);
        prodmp_post.learnFromDemonstration(demo);
        prodmp_post.setInitialConditions(prodmp_post.initTime(), prodmp_post.initPos(), prodmp_post.initVel());

        std::vector<Eigen::Vector3d> replay_post(N);
        double prev_t_sim = demo.front().t;
        for (int k = 0; k < N; ++k) {
            double dt_sim = (k == 0) ? 0.0 : (demo[k].t - prev_t_sim);
            prev_t_sim = demo[k].t;
            replay_post[k] = (k == 0) ? prodmp_post.step(0.0) : prodmp_post.step(dt_sim);
        }
        double rmse_post = dmp_tools::metrics::computeTrajectoryFidelity(raw_pos, replay_post).rmse_overall;

        double max_w_post = 0.0;
        for (int d = 0; d < 3; ++d) {
            max_w_post = std::max(max_w_post, prodmp_post.weights()[d].cwiseAbs().maxCoeff());
        }

        // 2. Pre-fix: solve without column scaling (scale = 1)
        Eigen::MatrixXd A_pre = HtH_raw;
        A_pre.diagonal().array() += lam;
        Eigen::LDLT<Eigen::MatrixXd> solver_pre(A_pre);
        std::array<Eigen::VectorXd, 3> w_pre;
        Eigen::Vector3d g_pre;
        double max_w_pre = 0.0;
        for (int d = 0; d < 3; ++d) {
            Eigen::VectorXd p_pre = solver_pre.solve(HtB_raw.col(d));
            w_pre[d] = p_pre.head(num_basis);
            g_pre(d) = p_pre(num_basis);
            max_w_pre = std::max(max_w_pre, w_pre[d].cwiseAbs().maxCoeff());
        }

        ProDMP prodmp_pre(num_basis, alpha, alpha_x, lam);
        prodmp_pre.setLearnedParameters(tau, init_pos, init_vel, g_pre, centers, widths, w_pre, true);
        prodmp_pre.setInitialConditions(t0, init_pos, init_vel);

        std::vector<Eigen::Vector3d> replay_pre(N);
        prev_t_sim = demo.front().t;
        for (int k = 0; k < N; ++k) {
            double dt_sim = (k == 0) ? 0.0 : (demo[k].t - prev_t_sim);
            prev_t_sim = demo[k].t;
            replay_pre[k] = (k == 0) ? prodmp_pre.step(0.0) : prodmp_pre.step(dt_sim);
        }
        double rmse_pre = dmp_tools::metrics::computeTrajectoryFidelity(raw_pos, replay_pre).rmse_overall;

        f2 << lam << "," << rmse_pre << "," << rmse_post << "," << max_w_pre << "," << max_w_post << "\n";
        std::cout << std::scientific << std::setprecision(1) << lam << "\t\t"
                  << std::fixed << std::setprecision(4) << rmse_pre << " mm\t\t"
                  << rmse_post << " mm\n";
    }
    f2.close();

    return 0;
}
