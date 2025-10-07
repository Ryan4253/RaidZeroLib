#include "RaidZeroLib/api/Geometry/Rotation.hpp"
#include <cmath>
#include <gtest/gtest.h>

const double EPSILON = 0.0001;

TEST(RotationTest, angleConstructor) {
    rz::Rotation angle(au::degrees(60));

    EXPECT_NEAR(angle.Theta().in(au::degrees), 60.0, EPSILON);
}

TEST(RotationTest, angleConstructorOver180) {
    rz::Rotation angle(au::degrees(210));

    EXPECT_NEAR(angle.Theta().in(au::degrees), -150, EPSILON);
}

TEST(RotationTest, unitPointConstructor) {
    rz::Rotation angle(au::meters(3), au::meters(3));

    EXPECT_EQ(angle.Theta(), au::degrees(45));
}

TEST(RotationTest, Trig) {
    rz::Rotation angle(au::degrees(60));

    EXPECT_NEAR(angle.Sin(), sqrt(3)/2.0, EPSILON);
    EXPECT_NEAR(angle.Cos(), 1.0/2.0, EPSILON);
    EXPECT_NEAR(angle.Tan(), sqrt(3), EPSILON);
}

TEST(RotationTest, addition) {
    rz::Rotation angle1(au::degrees(30));
    rz::Rotation angle2(au::degrees(60));
    rz::Rotation sum = angle1 + angle2;

    EXPECT_NEAR(sum.Theta().in(au::degrees), 90, EPSILON);
}

TEST(RotationTest, additionOver360) {
    rz::Rotation angle1(au::degrees(230));
    rz::Rotation angle2(au::degrees(160));
    rz::Rotation sum = angle1 + angle2;

    EXPECT_NEAR(sum.Theta().in(au::degrees), 30, EPSILON);
}

TEST(RotationTest, subtraction) {
    rz::Rotation angle1(au::degrees(30));
    rz::Rotation angle2(au::degrees(60));
    rz::Rotation diff = angle1 - angle2;

    EXPECT_NEAR(diff.Theta().in(au::degrees), -30, EPSILON);
}

TEST(RotationTest, negation) {
    rz::Rotation angle(au::degrees(30));
    rz::Rotation neg = -angle;

    EXPECT_NEAR(neg.Theta().in(au::degrees), -30, EPSILON);
}

TEST(RotationTest, scalarMultiplication) {
    rz::Rotation angle(au::degrees(30));
    rz::Rotation scaled = angle * 2;

    EXPECT_NEAR(scaled.Theta().in(au::degrees), 60, EPSILON);
}

TEST(RotationTest, scalarDivision) {
    rz::Rotation angle(au::degrees(60));
    rz::Rotation scaled = angle / 2;

    EXPECT_NEAR(scaled.Theta().in(au::degrees), 30, EPSILON);
}

TEST(RotationTest, approx) {
    rz::Rotation angle1(au::degrees(30));
    rz::Rotation angle2(au::degrees(30));
    rz::Rotation angle3(au::degrees(60));

    EXPECT_TRUE(angle1.isApprox(angle2));
    EXPECT_FALSE(angle1.isApprox(angle3));
}
