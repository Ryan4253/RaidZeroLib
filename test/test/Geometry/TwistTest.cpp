#include "RaidZeroLib/api/Geometry/Twist.hpp"
#include <gtest/gtest.h>
using namespace okapi;

constexpr double EPSILON = 0.0001;

TEST(TwistTest, constuctor) {
    rz::Twist twist(3_m, 4_m, 5_rad);

    EXPECT_EQ(twist.dX(), 3_m);
    EXPECT_EQ(twist.dY(), 4_m);
    EXPECT_EQ(twist.dTheta(), 5_rad);
}

TEST(TwistTest, copyConstructor) {
    rz::Twist twist1(3_m, 4_m, 5_rad);
    rz::Twist twist2(twist1);

    EXPECT_EQ(twist2.dX(), 3_m);
    EXPECT_EQ(twist2.dY(), 4_m);
    EXPECT_EQ(twist2.dTheta(), 5_rad);
}

TEST(TwistTest, equality) {
    rz::Twist twist1(3_m, 4_m, 5_rad);
    rz::Twist twist2(3_m, 4_m, 5_rad);
    rz::Twist twist3(5_m, 3_m, 6_rad);

    EXPECT_TRUE(twist1 == twist2);
    EXPECT_FALSE(twist1 == twist3);
}

TEST(TwistTest, inequality) {
    rz::Twist twist1(3_m, 4_m, 5_rad);
    rz::Twist twist2(3_m, 4_m, 5_rad);
    rz::Twist twist3(5_m, 3_m, 6_rad);

    EXPECT_FALSE(twist1 != twist2);
    EXPECT_TRUE(twist1 != twist3);
}

TEST(TwistTest, assignment) {
    rz::Twist twist1(3_m, 4_m, 5_rad);
    rz::Twist twist2(0_m, 0_m, 0_rad);
    twist2 = twist1;

    EXPECT_EQ(twist2.dX(), 3_m);
    EXPECT_EQ(twist2.dY(), 4_m);
    EXPECT_EQ(twist2.dTheta(), 5_rad);
}
