#pragma once

#include <cstddef>
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

#include "haptic_dmp_learning/core/types.hpp"

namespace haptic_dmp_learning {
namespace core {
namespace filter_utils {

/**
 * @brief Drop demonstration samples whose timestamp does not strictly increase.
 *
 * @details
 * DMP / QuaternionDMP learning estimate velocity and acceleration with central
 * finite differences that divide by (t[k+1] - t[k-1]). A repeated or backwards
 * timestamp makes that denominator zero (or negative); the learners then clamp
 * it to a tiny 1e-6 s, which turns the step into an enormous spurious
 * velocity/acceleration spike and inflates the fitted weights. Such rows appear
 * when a recorder stamps samples with a coarse sim-time /clock, collapsing
 * consecutive callbacks onto the same tick.
 *
 * This keeps the first sample and every later sample strictly newer than the
 * last kept one, preserving order and values otherwise.
 *
 * @param samples  Input demonstration (any ordering of timestamps).
 * @param dropped  If non-null, receives how many samples were removed.
 * @return Filtered demonstration, size == samples.size() - dropped.
 */
inline std::vector<Sample> dropNonIncreasingTimeSamples(
    const std::vector<Sample>& samples, int* dropped = nullptr) {
    std::vector<Sample> out;
    out.reserve(samples.size());
    for (const auto& s : samples) {
        if (out.empty() || s.t > out.back().t) {
            out.push_back(s);
        }
    }
    if (dropped != nullptr) {
        *dropped = static_cast<int>(samples.size() - out.size());
    }
    return out;
}

/**
 * @brief Force sign continuity on a demonstration's quaternion track (double cover).
 *
 * @details
 * A unit quaternion and its negation represent the same rotation (SU(2) double
 * covers SO(3)), but a sign flip between consecutive samples is a ~pi jump for
 * the local log-map increments QuaternionDMP accumulates, which injects an
 * absurd forcing-term spike at that point and destabilises the replay. Such
 * flips appear when a pose source (e.g. the Geomagic Touch driver) emits the
 * same physical orientation alternating between q and -q.
 *
 * Walks the sequence comparing consecutive *original* orientations: a negative
 * dot product is a genuine double-cover transition, so it toggles a running
 * sign and is counted once. Every later sample is emitted with the running sign
 * applied, yielding a globally continuous track. Positions and timestamps are
 * untouched.
 *
 * @param samples     Input demonstration.
 * @param transitions If non-null, receives the number of sign transitions found
 *                    (i.e. flip events, not the count of negated samples).
 * @return Corrected copy, same size as @p samples.
 */
inline std::vector<Sample> enforceQuaternionContinuity(
    const std::vector<Sample>& samples, int* transitions = nullptr) {
    std::vector<Sample> out = samples;
    int n = 0;
    bool negate = false;
    for (std::size_t k = 1; k < out.size(); ++k) {
        if (samples[k - 1].orientation.dot(samples[k].orientation) < 0.0) {
            negate = !negate;
            ++n;
        }
        if (negate) {
            out[k].orientation.coeffs() = -out[k].orientation.coeffs();
        }
    }
    if (transitions != nullptr) {
        *transitions = n;
    }
    return out;
}

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
