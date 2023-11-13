#include "RaidZeroLib/api/Utility/StateMachine.hpp"
#include <gtest/gtest.h>

enum class State { ONE, TWO, IDLE };

TEST(StateMachineTest, constructor) {
    rz::StateMachine<State> sm;
    EXPECT_EQ(sm.getState(), State::IDLE);

    rz::StateMachine<State, State::ONE> sm2;
    EXPECT_EQ(sm2.getState(), State::ONE);
}

TEST(StateMachineTest, setState) {
    rz::StateMachine<State> sm;
    sm.setState(State::TWO);
    EXPECT_EQ(sm.getState(), State::TWO);
}
