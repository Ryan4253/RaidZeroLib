#include "RaidZeroLib/api/Geometry/Twist.hpp"
#include <gtest/gtest.h>

constexpr double EPSILON = 0.0001;

TEST(TwistTest, constuctor) {
    rz::Twist twist(au::meters(3), au::meters(4), au::radians(2));

    EXPECT_EQ(twist.dX().in(au::meters), 3);
    EXPECT_EQ(twist.dY().in(au::meters), 4);
    EXPECT_EQ(twist.dTheta().in(au::radians), 2);
}

TEST(TwistTest, isApprox) {
    const rz::Twist twist1(au::meters(3), au::meters(4), au::radians(5));
    const rz::Twist twist2(au::meters(3), au::meters(4), au::radians(5));
    const rz::Twist twist3(au::meters(5), au::meters(3), au::radians(6));

    EXPECT_TRUE(twist1.isApprox(twist2));
    EXPECT_FALSE(twist1.isApprox(twist3));
}
