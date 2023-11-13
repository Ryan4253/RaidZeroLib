#include "RaidZeroLib/api/Geometry/Rotation.hpp"
#include <cmath>
#include <gtest/gtest.h>
using namespace okapi;

const double EPSILON = 0.0001;

TEST(RotationTest, defaultConstructor) {
    rz::Rotation angle;

    EXPECT_EQ(angle.Theta(), 0_deg);
    EXPECT_EQ(angle.Sin(), 0.0);
    EXPECT_EQ(angle.Cos(), 1.0);
}

TEST(RotationTest, angleConstructor) {
    rz::Rotation angle(60_deg);

    EXPECT_EQ(angle.Theta(), 60_deg);
    EXPECT_NEAR(angle.Sin(), std::sqrt(3) / 2, EPSILON);
    EXPECT_NEAR(angle.Cos(), 0.5, EPSILON);
}

TEST(RotationTest, angleConstructorOver180) {
    rz::Rotation angle(210_deg);

    EXPECT_NEAR(angle.Theta().convert(degree), -150, EPSILON);
    EXPECT_NEAR(angle.Sin(), -1.0 / 2, EPSILON);
    EXPECT_NEAR(angle.Cos(), -sqrt(3) / 2, EPSILON);
}

TEST(RotationTest, unitPointConstructor) {
    rz::Rotation angle(3_ft, 3_ft);
    EXPECT_EQ(angle.Theta(), 45_deg);

    EXPECT_NEAR(angle.Sin(), std::sqrt(2) / 2, EPSILON);
    EXPECT_NEAR(angle.Cos(), std::sqrt(2) / 2, EPSILON);
}

TEST(RotationTest, unitPointConstructorNearZero) {
    rz::Rotation angle(0.0000000001_m, 0.000000000001_m);

    EXPECT_EQ(angle.Theta(), 0_deg);
    EXPECT_EQ(angle.Sin(), 0.0);
    EXPECT_EQ(angle.Cos(), 1.0);
}

TEST(RotationTest, doublePointConstructor) {
    rz::Rotation angle(sqrt(3) / 2, 1.0 / 2);

    EXPECT_NEAR(angle.Theta().convert(degree), 30, EPSILON);
    EXPECT_NEAR(angle.Sin(), 1.0 / 2, EPSILON);
    EXPECT_NEAR(angle.Cos(), sqrt(3) / 2, EPSILON);
}

TEST(RotationTest, copyConstructor) {
    rz::Rotation angle1(60_deg);
    rz::Rotation angle2(angle1);

    EXPECT_EQ(angle2.Theta(), 60_deg);
    EXPECT_NEAR(angle2.Sin(), std::sqrt(3) / 2, EPSILON);
    EXPECT_NEAR(angle2.Cos(), 0.5, EPSILON);
}

TEST(RotationTest, Tan) {
    rz::Rotation angle(60_deg);

    EXPECT_NEAR(angle.Tan(), sqrt(3), EPSILON);
}

TEST(RotationTest, addition) {
    rz::Rotation angle1(30_deg);
    rz::Rotation angle2(60_deg);
    rz::Rotation sum = angle1 + angle2;

    EXPECT_NEAR(sum.Theta().convert(degree), 90, EPSILON);
    EXPECT_NEAR(sum.Sin(), 1, EPSILON);
    EXPECT_NEAR(sum.Cos(), 0, EPSILON);
}

TEST(RotationTest, additionOver360) {
    rz::Rotation angle1(230_deg);
    rz::Rotation angle2(160_deg);
    rz::Rotation sum = angle1 + angle2;

    EXPECT_NEAR(sum.Theta().convert(degree), 30, EPSILON);
    EXPECT_NEAR(sum.Sin(), 1.0 / 2, EPSILON);
    EXPECT_NEAR(sum.Cos(), sqrt(3) / 2, EPSILON);
}

TEST(RotationTest, subtraction) {
    rz::Rotation angle1(30_deg);
    rz::Rotation angle2(60_deg);
    rz::Rotation diff = angle1 - angle2;

    EXPECT_NEAR(diff.Theta().convert(degree), -30, EPSILON);
    EXPECT_NEAR(diff.Sin(), -1.0 / 2, EPSILON);
    EXPECT_NEAR(diff.Cos(), sqrt(3) / 2, EPSILON);
}

TEST(RotationTest, negation) {
    rz::Rotation angle(390_deg);
    rz::Rotation neg = -angle;

    EXPECT_NEAR(neg.Theta().convert(degree), -30, EPSILON);
    EXPECT_NEAR(neg.Sin(), -1.0 / 2, EPSILON);
    EXPECT_NEAR(neg.Cos(), sqrt(3) / 2, EPSILON);
}

TEST(RotationTest, scalarMultiplication) {
    rz::Rotation angle(30_deg);
    rz::Rotation scaled = angle * 2;

    EXPECT_NEAR(scaled.Theta().convert(degree), 60, EPSILON);
    EXPECT_NEAR(scaled.Sin(), sqrt(3) / 2, EPSILON);
    EXPECT_NEAR(scaled.Cos(), 1.0 / 2, EPSILON);
}

TEST(RotationTest, scalarDivision) {
    rz::Rotation angle(60_deg);
    rz::Rotation scaled = angle / 2;

    EXPECT_NEAR(scaled.Theta().convert(degree), 30, EPSILON);
    EXPECT_NEAR(scaled.Sin(), 1.0 / 2, EPSILON);
    EXPECT_NEAR(scaled.Cos(), sqrt(3) / 2, EPSILON);
}

TEST(RotationTest, equality) {
    rz::Rotation angle1(30_deg);
    rz::Rotation angle2(30_deg);
    rz::Rotation angle3(60_deg);

    EXPECT_TRUE(angle1 == angle2);
    EXPECT_FALSE(angle1 == angle3);
}

TEST(RotationTest, inequality) {
    rz::Rotation angle1(30_deg);
    rz::Rotation angle2(30_deg);
    rz::Rotation angle3(60_deg);

    EXPECT_FALSE(angle1 != angle2);
    EXPECT_TRUE(angle1 != angle3);
}

TEST(RotationTest, assignment) {
    rz::Rotation angle1(30_deg);
    rz::Rotation angle2(60_deg);
    angle1 = angle2;

    EXPECT_TRUE(angle1 == angle2);
}

TEST(RotationTest, rotateBy) {
    rz::Rotation angle1(30_deg);
    rz::Rotation angle2(60_deg);
    rz::Rotation rotated = angle1.rotateBy(angle2);

    EXPECT_NEAR(rotated.Theta().convert(degree), 90, EPSILON);
    EXPECT_NEAR(rotated.Sin(), 1, EPSILON);
    EXPECT_NEAR(rotated.Cos(), 0, EPSILON);

    EXPECT_TRUE(angle1.rotateBy(angle2) == angle2.rotateBy(angle1));
}