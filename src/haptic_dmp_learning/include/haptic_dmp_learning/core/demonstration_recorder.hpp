#pragma once

#include <vector>
#include "haptic_dmp_learning/core/types.hpp"

namespace haptic_dmp_learning {
namespace core {

/**
 * @brief Accumulates and buffers time-series Cartesian pose samples during a demonstration.
 *
 * @details
 * This class provides a clean in-memory trajectory buffer. During a teleoperation demonstration
 * (e.g. while the user holds a haptic button pressed), incoming pose samples are sequentially
 * pushed into this container. Once the demonstration completes, the collected sequence of Sample
 * objects is passed to DMP::learnFromDemonstration() and QuaternionDMP::learnFromDemonstration().
 */
class DemonstrationRecorder {
public:
    /// @brief Appends a new timestamped pose sample to the demonstration trajectory.
    void addSample(const Sample& s);

    /// @brief Resets the recorder buffer, removing all collected samples.
    void clear();

    /// @brief Returns a const reference to the recorded sample trajectory.
    const std::vector<Sample>& samples() const;

    /// @brief Checks if the buffer contains no samples.
    bool empty() const;

    /// @brief Returns the total number of recorded samples in the buffer.
    size_t size() const;

private:
    std::vector<Sample> samples_;
};

}  // namespace core
}  // namespace haptic_dmp_learning
