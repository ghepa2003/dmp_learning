#pragma once
#include <vector>
#include "haptic_dmp_learning/core/types.hpp"

namespace haptic_dmp_learning {
namespace core {

// Accumulates samples during a single demonstration. 
class DemonstrationRecorder {
public:
    void addSample(const Sample& s);
    void clear();

    const std::vector<Sample>& samples() const;
    bool empty() const;
    size_t size() const;

private:
    std::vector<Sample> samples_;
};

}  // namespace core
}  // namespace haptic_dmp_learning
