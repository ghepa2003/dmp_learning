#include <gtest/gtest.h>

#include "haptic_dmp_learning/core/button_sequencer.hpp"

using namespace haptic_dmp_learning::core::button_sequencer;

namespace {
constexpr double kDebounce = 0.30;
}

TEST(ButtonSequencer, NormalThreePressSequence) {
    ButtonSequencer s(kDebounce);
    EXPECT_EQ(s.state(), State::kIdle);

    auto r = s.onPress(1.0);
    EXPECT_EQ(r.outcome, Outcome::kAccepted);
    EXPECT_EQ(r.action, Action::kRequestStart);
    EXPECT_EQ(s.state(), State::kStarting);

    EXPECT_TRUE(s.onStartEmitted());
    EXPECT_EQ(s.state(), State::kRecording);

    r = s.onPress(8.0);
    EXPECT_EQ(r.action, Action::kTrigger);
    EXPECT_EQ(r.before, State::kRecording);
    EXPECT_EQ(r.after, State::kTriggered);

    r = s.onPress(12.0);
    EXPECT_EQ(r.action, Action::kStop);
    EXPECT_EQ(s.state(), State::kStopped);
}

TEST(ButtonSequencer, BounceWithinDebounceIsDropped) {
    ButtonSequencer s(kDebounce);
    s.onPress(1.0);
    s.onStartEmitted();
    s.onPress(5.0);  // trigger
    ASSERT_EQ(s.state(), State::kTriggered);

    // Contact bounce 0.1 s after the trigger must not stop the demo...
    auto r = s.onPress(5.1);
    EXPECT_EQ(r.outcome, Outcome::kDebounced);
    EXPECT_EQ(r.action, Action::kNone);
    EXPECT_EQ(s.state(), State::kTriggered);

    // ...and the window slides: still within 0.3 s of the bounce edge.
    r = s.onPress(5.35);
    EXPECT_EQ(r.outcome, Outcome::kDebounced);
    EXPECT_EQ(s.state(), State::kTriggered);

    // A genuine press after a pause counts.
    r = s.onPress(6.0);
    EXPECT_EQ(r.action, Action::kStop);
}

TEST(ButtonSequencer, ExactlyDebounceApartIsAccepted) {
    ButtonSequencer s(kDebounce);
    s.onPress(1.0);
    s.onStartEmitted();
    EXPECT_EQ(s.onPress(1.0 + kDebounce).action, Action::kTrigger);
}

TEST(ButtonSequencer, PressDuringStartingIsIgnoredAndDoesNotAdvance) {
    ButtonSequencer s(kDebounce);
    s.onPress(1.0);  // STARTING (spawn / sync window)
    auto r = s.onPress(3.0);
    EXPECT_EQ(r.outcome, Outcome::kIgnoredStarting);
    EXPECT_EQ(r.action, Action::kNone);
    EXPECT_EQ(s.state(), State::kStarting);

    // The ignored press is NOT remembered as a trigger: after the start is
    // emitted the next press is the 2nd one.
    s.onStartEmitted();
    EXPECT_EQ(s.state(), State::kRecording);
    EXPECT_EQ(s.onPress(9.0).action, Action::kTrigger);
}

TEST(ButtonSequencer, PressesAfterStoppedAreIgnoredForever) {
    ButtonSequencer s(kDebounce);
    s.onPress(1.0);
    s.onStartEmitted();
    s.onPress(5.0);
    s.onPress(9.0);
    ASSERT_EQ(s.state(), State::kStopped);

    for (double t : {10.0, 20.0, 100.0}) {
        auto r = s.onPress(t);
        EXPECT_EQ(r.outcome, Outcome::kIgnoredStopped);
        EXPECT_EQ(r.action, Action::kNone);
        EXPECT_EQ(s.state(), State::kStopped);
    }
}

TEST(ButtonSequencer, KeyboardAndButtonShareTheSameMachine) {
    // The caller feeds SPACE through onPress() exactly like a button edge, so
    // mixing the two sources walks ONE sequence (no second logic).
    ButtonSequencer s(kDebounce);
    EXPECT_EQ(s.onPress(1.0).action, Action::kRequestStart);  // button
    s.onStartEmitted();
    EXPECT_EQ(s.onPress(6.0).action, Action::kTrigger);       // SPACE
    EXPECT_EQ(s.onPress(10.0).action, Action::kStop);         // button
    EXPECT_EQ(s.onPress(14.0).outcome, Outcome::kIgnoredStopped);  // SPACE
}

TEST(ButtonSequencer, ContinuousChatterNeverLeaksThrough) {
    // The debounce reference slides with every edge, so a train of edges each
    // < debounce_s apart (chatter, or key auto-repeat once it has started)
    // never produces a second accepted press. NOTE: a terminal's FIRST repeat
    // comes ~0.5 s after the key-down, i.e. > debounce_s, and WOULD count:
    // SPACE must be tapped, not held.
    ButtonSequencer s(kDebounce);
    s.onPress(1.0);
    s.onStartEmitted();
    EXPECT_EQ(s.onPress(5.0).action, Action::kTrigger);
    double t = 5.25;
    for (int i = 0; i < 30; ++i, t += 0.033) {
        EXPECT_EQ(s.onPress(t).outcome, Outcome::kDebounced);
    }
    EXPECT_EQ(s.state(), State::kTriggered);
}

TEST(ButtonSequencer, StartEmittedOutsideStartingIsNoOp) {
    ButtonSequencer s(kDebounce);
    EXPECT_FALSE(s.onStartEmitted());  // IDLE
    EXPECT_EQ(s.state(), State::kIdle);
    s.onPress(1.0);
    EXPECT_TRUE(s.onStartEmitted());
    EXPECT_FALSE(s.onStartEmitted());  // already RECORDING
    EXPECT_EQ(s.state(), State::kRecording);
}

TEST(ButtonSequencer, InvalidDebounceThrows) {
    EXPECT_THROW(ButtonSequencer(-0.1), std::invalid_argument);
    EXPECT_NO_THROW(ButtonSequencer(0.0));
}
