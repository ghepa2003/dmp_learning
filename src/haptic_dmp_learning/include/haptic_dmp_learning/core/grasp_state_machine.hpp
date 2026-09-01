#pragma once

#include <string>

namespace haptic_dmp_learning {
namespace core {

/**
 * @brief Pure four-state grasp state machine fusing the geometric grasp signal,
 *        the per-demo force verification, and an absolute |F| safety limit.
 *
 * No ROS / I/O dependencies. The ROS node
 * (haptic_dmp_learning::ros_wrapper::GraspStateMachineNode) owns the
 * subscriptions, the message <-> Inputs/State conversion, the ~/grasp_state
 * publisher, and the ~/reset_limit_action rising-edge detection; it drives this
 * class exactly one cycle per geometric-signal message.
 *
 * States published downstream as strings (see stateName()):
 *   free_space | contact_pending | contact_confirmed | limit_action
 *
 * Transitions, all evaluated inside step() (once per geometric cycle):
 *   kFreeSpace        -> kContactPending   : geometric_confirmed true (1 cycle)
 *   kContactPending   -> kContactConfirmed : geometric_confirmed true for
 *                                            `persistence_cycles` consecutive
 *                                            cycles AND force_verified true
 *   kContactPending / kContactConfirmed
 *                     -> kFreeSpace         : geometric_confirmed false (lost)
 *   kContactConfirmed -> kLimitAction       : f_norm > hard_force_limit_n
 *   kLimitAction      -> kFreeSpace         : ONLY via requestReset();
 *                                             never automatically, not on
 *                                             signal loss, not on |F| dropping
 *                                             back below the limit.
 *
 * `hard_force_limit_n` is an ABSOLUTE safety maximum, separate from the
 * calibration tolerance. `kLimitAction` is latched on purpose (see
 * DESIGN_NOTES): "object repelled by excessive contact" (the failure this
 * component exists to catch) must not be silently conflated with "safety action
 * handled" - both would otherwise look identical to a downstream consumer.
 */
class GraspStateMachine {
public:
    enum class State {
        kFreeSpace,
        kContactPending,
        kContactConfirmed,
        kLimitAction,
    };

    struct Params {
        /// Consecutive geometric-true cycles required to confirm contact (>= 1).
        int persistence_cycles = 5;
        /// Absolute |F| safety limit in N (> 0). No default is meaningful for
        /// real use - the node declares it as a mandatory ROS parameter.
        double hard_force_limit_n = 0.0;
    };

    struct Inputs {
        /// Geometric grasp signal for this cycle.
        bool geometric_confirmed = false;
        /// Latest per-demo force verification (the node updates it
        /// asynchronously from a separate topic; step() only reads it).
        bool force_verified = false;
        /// Latest contact-wrench-estimate force norm |F| in N (idem).
        double f_norm = 0.0;
    };

    /// @throws std::invalid_argument if persistence_cycles < 1 or
    ///         hard_force_limit_n <= 0.
    explicit GraspStateMachine(Params params);

    /// Advance the machine by one geometric cycle; returns the new state.
    State step(const Inputs& inputs);

    /// Explicit exit from kLimitAction. Returns true iff the machine actually
    /// left kLimitAction (it was latched); returns false (no-op) otherwise.
    bool requestReset();

    State state() const { return state_; }

    /// Consecutive geometric-true cycles seen so far (diagnostic / logging).
    int geometricStreak() const { return geom_true_streak_; }

    const Params& params() const { return params_; }

    static std::string stateName(State s);

private:
    Params params_;
    State state_ = State::kFreeSpace;
    int geom_true_streak_ = 0;
};

}  // namespace core
}  // namespace haptic_dmp_learning
