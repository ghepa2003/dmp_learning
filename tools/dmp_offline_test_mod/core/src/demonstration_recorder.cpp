#include "haptic_dmp_learning/core/demonstration_recorder.hpp"

namespace haptic_dmp_learning {
namespace core {

void DemonstrationRecorder::addSample(const Sample& s) {
    samples_.push_back(s);
}

void DemonstrationRecorder::clear() {
    samples_.clear();
}

const std::vector<Sample>& DemonstrationRecorder::samples() const {
    return samples_;
}

bool DemonstrationRecorder::empty() const {
    return samples_.empty();
}

size_t DemonstrationRecorder::size() const {
    return samples_.size();
}

}  // namespace core
}  // namespace haptic_dmp_learning
