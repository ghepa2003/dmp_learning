#pragma once
#include <vector>
#include "haptic_dmp_learning/core/types.hpp"

namespace haptic_dmp_learning {
namespace core {

// Accumulates samples during a single demonstration. 
class DemonstrationRecorder {
public:

    // Adds a new sample to the demonstration. The sample's timestamp is relative to the start of the demonstration.
    void addSample(const Sample& s);

    // Clears all recorded samples, resetting the demonstration recorder to an empty state.
    void clear();

    // Returns a const reference to the vector of recorded samples.
    const std::vector<Sample>& samples() const;

    // Returns true if no samples have been recorded, false otherwise.
    bool empty() const;

    // Returns the number of samples recorded in the demonstration.
    size_t size() const;

private:

    // Vector of samples recorded during the demonstration. Each sample contains a timestamp, position, and orientation.
    std::vector<Sample> samples_;
};

}  // namespace core
}  // namespace haptic_dmp_learning
