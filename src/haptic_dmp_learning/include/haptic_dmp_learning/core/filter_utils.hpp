#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

namespace haptic_dmp_learning {
namespace core {
namespace filter_utils {

/**
 * @brief Zero-phase symmetric moving average filter for multi-dimensional time series.
 *
 * @details
 * Mathematical Theory & Filtering Rationale:
 * Demonstrations captured via manual teleoperation (e.g. Geomagic Touch or kinesthetic teaching)
 * contain high-frequency sensor noise, tremor, and discretization jitter.
 * When fitting Dynamic Movement Primitives (DMPs), computing numerical derivatives (velocity dy/dt
 * and acceleration d^2y/dt^2) drastically amplifies high-frequency noise.
 *
 * This filter applies a symmetric temporal sliding window of duration `window_sec`:
 * 1. Estimates average sampling interval: dt_est = (t_end - t_0) / (N - 1)
 * 2. Window size in discrete samples: W = round(window_sec / dt_est), with half-window M = floor(W / 2)
 * 3. Symmetric averaging:
 *      y_filtered[k] = (1 / |J_k|) * sum_{j in J_k} y[j],  where J_k = [max(0, k - M), min(N - 1, k + M)]
 *
 * Properties:
 * - Zero Phase Distortion: Symmetric centered window ensures zero phase lag / time delay.
 * - Boundary Clamping: Symmetrically handles beginning and end of trajectory without truncation.
 *
 * @tparam VectorT Eigen vector type (e.g., Eigen::Vector3d or Eigen::Matrix<double, N, 1>).
 * @param signal Vector of spatial samples to smooth.
 * @param t Vector of monotonically increasing timestamps (seconds).
 * @param window_sec Filter window width in seconds (e.g. 0.05 s). If <= 0, signal is returned unchanged.
 * @return std::vector<VectorT> Smoothed spatial trajectory with the same number of samples.
 */
template <typename VectorT>
inline std::vector<VectorT> movingAverageSmooth(
    const std::vector<VectorT>& signal,
    const std::vector<double>& t,
    double window_sec) {
    const size_t N = signal.size();
    if (window_sec <= 0.0 || N < 2) return signal;

    // Estimate nominal sampling time
    double dt_est = (t.back() - t.front()) / static_cast<double>(N - 1);
    if (dt_est <= 0.0) return signal;

    int window_samples = std::max(1, static_cast<int>(std::round(window_sec / dt_est)));
    int half = window_samples / 2;

    std::vector<VectorT> out(N);
    for (size_t k = 0; k < N; ++k) {
        int lo = std::max(0, static_cast<int>(k) - half);
        int hi = std::min(static_cast<int>(N) - 1, static_cast<int>(k) + half);
        VectorT sum = VectorT::Zero();
        int count = 0;
        for (int j = lo; j <= hi; ++j) {
            sum += signal[j];
            ++count;
        }
        out[k] = sum / static_cast<double>(count);
    }
    return out;
}

}  // namespace filter_utils
}  // namespace core
}  // namespace haptic_dmp_learning
