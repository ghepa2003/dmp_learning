#include <gtest/gtest.h>

#include <stdexcept>

#include "haptic_dmp_learning/core/grasp_state_machine.hpp"

using haptic_dmp_learning::core::GraspStateMachine;
using State = GraspStateMachine::State;

namespace {

// Build one geometric cycle's Inputs. force_verified defaults to true so that
// tests targeting the persistence gate are not accidentally blocked by the
// force-verification gate; f_norm defaults to a safe 0 N.
GraspStateMachine::Inputs cycle(bool geometric, bool force_verified = true,
                                double f_norm = 0.0) {
    GraspStateMachine::Inputs in;
    in.geometric_confirmed = geometric;
    in.force_verified = force_verified;
    in.f_norm = f_norm;
    return in;
}

GraspStateMachine::Params validParams(int persistence = 3, double limit = 15.0) {
    GraspStateMachine::Params p;
    p.persistence_cycles = persistence;
    p.hard_force_limit_n = limit;
    return p;
}

}  // namespace

// --- construction / validation -------------------------------------------------

TEST(GraspStateMachine, ConstructsInFreeSpace) {
    GraspStateMachine fsm(validParams());
    EXPECT_EQ(fsm.state(), State::kFreeSpace);
    EXPECT_EQ(fsm.geometricStreak(), 0);
}

TEST(GraspStateMachine, RejectsPersistenceBelowOne) {
    EXPECT_THROW(GraspStateMachine(validParams(0, 15.0)), std::invalid_argument);
    EXPECT_THROW(GraspStateMachine(validParams(-1, 15.0)), std::invalid_argument);
}

TEST(GraspStateMachine, RejectsNonPositiveForceLimit) {
    EXPECT_THROW(GraspStateMachine(validParams(3, 0.0)), std::invalid_argument);
    EXPECT_THROW(GraspStateMachine(validParams(3, -5.0)), std::invalid_argument);
}

// --- persistence gate --------------------------------------------------------

// free_space -> contact_pending on the first geometric-true cycle, then
// contact_pending -> contact_confirmed only once the geometric signal has been
// true for `persistence_cycles` consecutive cycles (force_verified held true).
TEST(GraspStateMachine, ConfirmsContactWhenPersistenceGateSatisfied) {
    GraspStateMachine fsm(validParams(/*persistence=*/3));

    EXPECT_EQ(fsm.step(cycle(true)), State::kContactPending);   // streak 1
    EXPECT_EQ(fsm.geometricStreak(), 1);

    EXPECT_EQ(fsm.step(cycle(true)), State::kContactPending);    // streak 2 < 3
    EXPECT_EQ(fsm.step(cycle(true)), State::kContactConfirmed);  // streak 3 >= 3
    EXPECT_EQ(fsm.geometricStreak(), 3);
}

// An intermittent geometric signal that never stays true for `persistence_cycles`
// consecutive cycles must never reach contact_confirmed: each false cycle
// resets the streak AND drops contact_pending back to free_space.
TEST(GraspStateMachine, NoConfirmWhenGeometricSignalIntermittent) {
    GraspStateMachine fsm(validParams(/*persistence=*/3));

    for (int i = 0; i < 10; ++i) {
        fsm.step(cycle(true));
        EXPECT_NE(fsm.state(), State::kContactConfirmed);
        State after_gap = fsm.step(cycle(false));
        EXPECT_EQ(after_gap, State::kFreeSpace);
        EXPECT_EQ(fsm.geometricStreak(), 0);
    }
}

// The persistence streak alone is not enough: force_verified must also be true
// on the cycle where the streak first reaches the threshold (or a later cycle).
TEST(GraspStateMachine, PersistenceStreakAloneDoesNotConfirmWithoutForceVerified) {
    GraspStateMachine fsm(validParams(/*persistence=*/2));

    EXPECT_EQ(fsm.step(cycle(true, /*force_verified=*/false)), State::kContactPending);
    EXPECT_EQ(fsm.step(cycle(true, /*force_verified=*/false)), State::kContactPending);
    EXPECT_EQ(fsm.step(cycle(true, /*force_verified=*/false)), State::kContactPending);
    EXPECT_GE(fsm.geometricStreak(), 2);

    // Same over-threshold streak, now with force_verified -> confirms.
    EXPECT_EQ(fsm.step(cycle(true, /*force_verified=*/true)), State::kContactConfirmed);
}

// Losing the geometric signal from contact_confirmed drops straight to
// free_space (this is the non-latched path, distinct from limit_action).
TEST(GraspStateMachine, SignalLossFromConfirmedReturnsToFreeSpace) {
    GraspStateMachine fsm(validParams(/*persistence=*/1));

    fsm.step(cycle(true));                         // free -> pending
    ASSERT_EQ(fsm.step(cycle(true)), State::kContactConfirmed);

    EXPECT_EQ(fsm.step(cycle(false)), State::kFreeSpace);
}

// --- hard force limit + latch ----------------------------------------------

// From contact_confirmed, |F| exceeding hard_force_limit_n moves to
// limit_action.
TEST(GraspStateMachine, EntersLimitActionWhenForceExceedsHardLimit) {
    GraspStateMachine fsm(validParams(/*persistence=*/1, /*limit=*/15.0));

    fsm.step(cycle(true));                                     // free -> pending
    ASSERT_EQ(fsm.step(cycle(true)), State::kContactConfirmed);

    EXPECT_EQ(fsm.step(cycle(true, true, /*f_norm=*/20.0)), State::kLimitAction);
}

// A |F| exactly at the limit is NOT over it (strict '>').
TEST(GraspStateMachine, ForceExactlyAtLimitDoesNotTrip) {
    GraspStateMachine fsm(validParams(/*persistence=*/1, /*limit=*/15.0));

    fsm.step(cycle(true));
    ASSERT_EQ(fsm.step(cycle(true)), State::kContactConfirmed);

    EXPECT_EQ(fsm.step(cycle(true, true, /*f_norm=*/15.0)), State::kContactConfirmed);
}

// limit_action is latched: it does not exit when |F| drops back below the
// limit, and it does not exit when the geometric signal is lost.
TEST(GraspStateMachine, LimitActionIsLatchedAgainstForceDropAndSignalLoss) {
    GraspStateMachine fsm(validParams(/*persistence=*/1, /*limit=*/15.0));

    fsm.step(cycle(true));
    fsm.step(cycle(true));                                    // -> confirmed
    ASSERT_EQ(fsm.step(cycle(true, true, 20.0)), State::kLimitAction);

    EXPECT_EQ(fsm.step(cycle(true, true, /*f_norm=*/0.0)), State::kLimitAction);
    EXPECT_EQ(fsm.step(cycle(false, true, /*f_norm=*/0.0)), State::kLimitAction);
    EXPECT_EQ(fsm.step(cycle(true, false, /*f_norm=*/0.0)), State::kLimitAction);
}

// The ONLY way out of limit_action is requestReset(); it lands in free_space
// with the streak cleared, and a second call is a no-op.
TEST(GraspStateMachine, RequestResetIsTheOnlyExitFromLimitAction) {
    GraspStateMachine fsm(validParams(/*persistence=*/1, /*limit=*/15.0));

    fsm.step(cycle(true));
    fsm.step(cycle(true));
    ASSERT_EQ(fsm.step(cycle(true, true, 20.0)), State::kLimitAction);

    EXPECT_TRUE(fsm.requestReset());
    EXPECT_EQ(fsm.state(), State::kFreeSpace);
    EXPECT_EQ(fsm.geometricStreak(), 0);

    EXPECT_FALSE(fsm.requestReset());  // already out -> no-op
    EXPECT_EQ(fsm.state(), State::kFreeSpace);
}

// requestReset() outside limit_action is a no-op that does not perturb state.
TEST(GraspStateMachine, RequestResetIsNoOpOutsideLimitAction) {
    GraspStateMachine fsm(validParams(/*persistence=*/1));

    EXPECT_FALSE(fsm.requestReset());
    EXPECT_EQ(fsm.state(), State::kFreeSpace);

    fsm.step(cycle(true));
    fsm.step(cycle(true));
    ASSERT_EQ(fsm.state(), State::kContactConfirmed);

    EXPECT_FALSE(fsm.requestReset());
    EXPECT_EQ(fsm.state(), State::kContactConfirmed);
}

// --- name mapping ----------------------------------------------------------

TEST(GraspStateMachine, StateNameMapping) {
    EXPECT_EQ(GraspStateMachine::stateName(State::kFreeSpace), "free_space");
    EXPECT_EQ(GraspStateMachine::stateName(State::kContactPending), "contact_pending");
    EXPECT_EQ(GraspStateMachine::stateName(State::kContactConfirmed), "contact_confirmed");
    EXPECT_EQ(GraspStateMachine::stateName(State::kLimitAction), "limit_action");
}
