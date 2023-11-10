#include "RaidZeroLib/api/Geometry/Rotation.hpp"
#include <cmath>
#include <gtest/gtest.h>
using namespace okapi;

TEST(RotationTest, constructor) {
    rz::Rotation angle(3_ft, 3_ft);
    ASSERT_EQ(angle.Theta(), ((1_pi) / 4));
}
