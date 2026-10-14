#pragma once

#include <cmath>
#include <stdexcept>

namespace haptic_dmp_learning {
namespace core {
namespace button_sequencer {

/**
 * @brief Single-button, three-press sequencer for the live stylus demo (no ROS).
 *
 *   IDLE --1st press--> STARTING --start emitted--> RECORDING
 *        --2nd press--> TRIGGERED --3rd press--> STOPPED
 *
 * Inputs are rising edges of ONE button (or an equivalent key press) with a
 * timestamp in seconds, plus the "synced start emitted" event. Outputs are the
 * actions the caller must execute: RequestStart, Trigger, Stop.
 *
 *   - Presses while STARTING (target spawn / sync window in progress) are
 *     ignored: the recorder is not live yet.
 *   - Presses after STOPPED are ignored for good (until the object is rebuilt).
 *   - Debounce: an edge closer than debounce_s to the PREVIOUS EDGE (accepted
 *     or not) is dropped. The reference slides with every edge, so contact
 *     chatter and keyboard auto-repeat (a held SPACE) never leak through as a
 *     later press; only a pause of at least debounce_s lets the next press count.
 */
enum class State { kIdle, kStarting, kRecording, kTriggered, kStopped };

enum class Action { kNone, kRequestStart, kTrigger, kStop };

enum class Outcome {
    kAccepted,         ///< state advanced, see Result::action
    kDebounced,        ///< edge within debounce_s of the previous edge
    kIgnoredStarting,  ///< press while the synced start is still pending
    kIgnoredStopped    ///< press after STOPPED
};

struct Result {
    Outcome outcome = Outcome::kAccepted;
    Action action = Action::kNone;
    State before = State::kIdle;
    State after = State::kIdle;
};

inline const char* toString(State s) {
    switch (s) {
        case State::kIdle: return "IDLE";
        case State::kStarting: return "STARTING";
        case State::kRecording: return "RECORDING";
        case State::kTriggered: return "TRIGGERED";
        case State::kStopped: return "STOPPED";
    }
    return "?";
}

class ButtonSequencer {
public:
    explicit ButtonSequencer(double debounce_s = 0.30) : debounce_s_(debounce_s) {
        if (!(debounce_s >= 0.0) || !std::isfinite(debounce_s)) {
            throw std::invalid_argument("ButtonSequencer: debounce_s must be finite and >= 0");
        }
    }

    State state() const { return state_; }
    double debounceSeconds() const { return debounce_s_; }

    /// Register one rising edge at time @p t_sec (monotonic seconds).
    Result onPress(double t_sec) {
        Result r;
        r.before = state_;
        const bool debounced = has_last_edge_ && (t_sec - last_edge_t_) < debounce_s_;
        last_edge_t_ = t_sec;
        has_last_edge_ = true;
        if (debounced) {
            r.outcome = Outcome::kDebounced;
            r.after = state_;
            return r;
        }
        switch (state_) {
            case State::kIdle:
                state_ = State::kStarting;
                r.action = Action::kRequestStart;
                break;
            case State::kStarting:
                r.outcome = Outcome::kIgnoredStarting;
                break;
            case State::kRecording:
                state_ = State::kTriggered;
                r.action = Action::kTrigger;
                break;
            case State::kTriggered:
                state_ = State::kStopped;
                r.action = Action::kStop;
                break;
            case State::kStopped:
                r.outcome = Outcome::kIgnoredStopped;
                break;
        }
        r.after = state_;
        return r;
    }

    /// The synced start [1,0] has been emitted: STARTING -> RECORDING.
    /// Returns false (state unchanged) in any other state.
    bool onStartEmitted() {
        if (state_ != State::kStarting) return false;
        state_ = State::kRecording;
        return true;
    }

private:
    double debounce_s_;
    State state_ = State::kIdle;
    bool has_last_edge_ = false;
    double last_edge_t_ = 0.0;
};

}  // namespace button_sequencer
}  // namespace core
}  // namespace haptic_dmp_learning
