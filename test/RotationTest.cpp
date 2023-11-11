#include "RaidZeroLib/api/Geometry/Rotation.hpp"
#include <cmath>
#include <gtest/gtest.h>
using namespace okapi;
#include "RaidZeroLib/api/Utility/StateMachine.hpp"

enum State { OPEN = 0, CLOSE = 1, IDLE = 2 };

TEST(RotationTest, constructor) {
    rz::Rotation angle(3_ft, 3_ft);
    ASSERT_EQ(angle.Theta().convert(radian), ((1_pi) / 4));
    rz::StateMachine<State> wassup;
    ASSERT_EQ(wassup.getState(), State::IDLE);
}
