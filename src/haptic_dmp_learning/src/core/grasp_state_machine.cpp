#include "haptic_dmp_learning/core/grasp_state_machine.hpp"

#include <stdexcept>

namespace haptic_dmp_learning {
namespace core {

GraspStateMachine::GraspStateMachine(Params params) : params_(params) {
    if (params_.persistence_cycles < 1) {
        throw std::invalid_argument(
            "GraspStateMachine: persistence_cycles must be >= 1.");
    }
    if (!(params_.hard_force_limit_n > 0.0)) {
        throw std::invalid_argument(
            "GraspStateMachine: hard_force_limit_n must be > 0.");
    }
}

GraspStateMachine::State GraspStateMachine::step(const Inputs& in) {
    geom_true_streak_ = in.geometric_confirmed ? (geom_true_streak_ + 1) : 0;

    switch (state_) {
        case State::kFreeSpace:
            if (in.geometric_confirmed) {
                state_ = State::kContactPending;  // geometric true for 1 cycle
            }
            break;

        case State::kContactPending:
            if (!in.geometric_confirmed) {
                state_ = State::kFreeSpace;  // signal lost
            } else if (geom_true_streak_ >= params_.persistence_cycles &&
                       in.force_verified) {
                state_ = State::kContactConfirmed;
            }
            break;

        case State::kContactConfirmed:
            if (!in.geometric_confirmed) {
                state_ = State::kFreeSpace;  // signal lost
            } else if (in.f_norm > params_.hard_force_limit_n) {
                state_ = State::kLimitAction;  // absolute safety limit exceeded
            }
            break;

        case State::kLimitAction:
            // Latched. No automatic exit - not on signal loss, not on |F|
            // dropping back below the limit. Only requestReset() releases it,
            // so a downstream consumer can tell "repelled / still unsafe" apart
            // from "safety action handled".
            break;
    }

    return state_;
}

bool GraspStateMachine::requestReset() {
    if (state_ != State::kLimitAction) {
        return false;
    }
    state_ = State::kFreeSpace;
    geom_true_streak_ = 0;
    return true;
}

std::string GraspStateMachine::stateName(State s) {
    switch (s) {
        case State::kFreeSpace:        return "free_space";
        case State::kContactPending:   return "contact_pending";
        case State::kContactConfirmed: return "contact_confirmed";
        case State::kLimitAction:      return "limit_action";
    }
    return "free_space";
}

}  // namespace core
}  // namespace haptic_dmp_learning
