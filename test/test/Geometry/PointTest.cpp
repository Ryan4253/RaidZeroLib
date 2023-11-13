#include "RaidZeroLib/api/Geometry/Point.hpp"
#include <cmath>
#include <gtest/gtest.h>
using namespace okapi;

const double EPSILON = 0.0001;

TEST(TranslationTest, defaultConstructor) {
    rz::Translation point;

    EXPECT_EQ(point.X(), 0_m);
    EXPECT_EQ(point.Y(), 0_m);
}

TEST(TranslationTest, cartesianConstructor) {
    rz::Translation point(3_m, 4_m);

    EXPECT_EQ(point.X(), 3_m);
    EXPECT_EQ(point.Y(), 4_m);
}

TEST(TranslationTest, polarConstructor) {
    rz::Translation point(3_m, 45_deg);

    EXPECT_NEAR(point.X().convert(meter), 3 * sqrt(2) / 2, EPSILON);
    EXPECT_NEAR(point.Y().convert(meter), 3 * sqrt(2) / 2, EPSILON);
}

TEST(TranslationTest, copyConstructor) {
    rz::Translation point1(3_m, 4_m);
    rz::Translation point2(point1);

    EXPECT_EQ(point2.X(), 3_m);
    EXPECT_EQ(point2.Y(), 4_m);
}

TEST(TranslationTest, setX) {
    rz::Translation point(3_m, 4_m);
    point.setX(5_m);

    EXPECT_EQ(point.X(), 5_m);
}

TEST(TranslationTest, setY) {
    rz::Translation point(3_m, 4_m);
    point.setY(5_m);

    EXPECT_EQ(point.Y(), 5_m);
}

TEST(TranslationTest, addition) {
    rz::Translation point1(3_m, 4_m);
    rz::Translation point2(5_m, 6_m);
    rz::Translation sum = point1 + point2;

    ASSERT_NEAR(sum.X().convert(meter), 8, EPSILON);
    ASSERT_NEAR(sum.Y().convert(meter), 10, EPSILON);
}

TEST(TranslationTest, subtraction) {
    rz::Translation point1(3_m, 4_m);
    rz::Translation point2(5_m, 6_m);
    rz::Translation diff = point1 - point2;

    ASSERT_NEAR(diff.X().convert(meter), -2, EPSILON);
    ASSERT_NEAR(diff.Y().convert(meter), -2, EPSILON);
}

TEST(TranslationTest, negation) {
    rz::Translation point(3_m, 4_m);
    point = -point;

    ASSERT_NEAR(point.X().convert(meter), -3, EPSILON);
    ASSERT_NEAR(point.Y().convert(meter), -4, EPSILON);
}

TEST(TranslationTest, scalarMultiplication) {
    rz::Translation point(3_m, 4_m);
    rz::Translation scaled = point * 2;

    ASSERT_NEAR(scaled.X().convert(meter), 6, EPSILON);
    ASSERT_NEAR(scaled.Y().convert(meter), 8, EPSILON);
}

TEST(TranslationTest, scalarDivision) {
    rz::Translation point(3_m, 4_m);
    rz::Translation scaled = point / 2;

    ASSERT_NEAR(scaled.X().convert(meter), 1.5, EPSILON);
    ASSERT_NEAR(scaled.Y().convert(meter), 2, EPSILON);
}

TEST(TranslationTest, equality) {
    rz::Translation point1(3_m, 4_m);
    rz::Translation point2(3_m, 4_m);
    rz::Translation point3(5_m, 6_m);

    ASSERT_TRUE(point1 == point2);
    ASSERT_FALSE(point1 == point3);
}

TEST(TranslationTest, inequality) {
    rz::Translation point1(3_m, 4_m);
    rz::Translation point2(3_m, 4_m);
    rz::Translation point3(5_m, 6_m);

    ASSERT_FALSE(point1 != point2);
    ASSERT_TRUE(point1 != point3);
}

TEST(TranslationTest, assignment) {
    rz::Translation point1(3_m, 4_m);
    rz::Translation point2(5_m, 6_m);
    point1 = point2;

    ASSERT_NEAR(point1.X().convert(meter), 5, EPSILON);
    ASSERT_NEAR(point1.Y().convert(meter), 6, EPSILON);
}

TEST(TranslationTest, theta) {
    rz::Translation point(0.5 * meter, sqrt(3) / 2 * meter);

    ASSERT_NEAR(point.theta().convert(degree), 60, EPSILON);
}

TEST(TranslationTest, magnitude) {
    rz::Translation point(3_m, 4_m);

    ASSERT_NEAR(point.mag().convert(meter), 5, EPSILON);
}

TEST(TranslationTest, distTo) {
    rz::Translation point1(1_m, 2_m);
    rz::Translation point2(4_m, 6_m);

    ASSERT_NEAR(point1.distTo(point2).convert(meter), 5, EPSILON);
}

TEST(TranslationTest, angleTo) {
    rz::Translation point1(2_m, 2_m);
    rz::Translation point2(-1_m, -sqrt(3) * meter);

    ASSERT_NEAR(point1.angleTo(point2).convert(degree), -165, EPSILON);
    ASSERT_NEAR(point2.angleTo(point1).convert(degree), 165, EPSILON);
}

TEST(TranslationTest, dot) {
    rz::Point point1(3_m, 4_m);
    rz::Point point2(5_m, 6_m);

    ASSERT_NEAR(point1.dot(point2).convert(meter2), 39, EPSILON);
}

TEST(TranslationTest, wedge) {
    rz::Point point1(3_m, 4_m);
    rz::Point point2(5_m, 6_m);

    ASSERT_NEAR(point1.wedge(point2).convert(meter2), -2, EPSILON);
}

TEST(TranslationTest, project) {
    rz::Point point1(3_m, 4_m);
    rz::Point point2(12_m, 8_m);
    rz::Point projection = point1.project(point2);

    ASSERT_NEAR(projection.X().convert(meter), 51.0 / 13, EPSILON);
    ASSERT_NEAR(projection.Y().convert(meter), 34.0 / 13, EPSILON);
}

TEST(TranslationTest, rotateBy) {
    rz::Point point(1_m, 1_m);
    rz::Rotation angle(45_deg);
    rz::Point rotated = point.rotateBy(angle);

    ASSERT_NEAR(rotated.X().convert(meter), 0, EPSILON);
    ASSERT_NEAR(rotated.Y().convert(meter), sqrt(2), EPSILON);
}

TEST(TranslationTest, circumradius) {
    rz::Point point1(2_m, 8_m);
    rz::Point point2(6_m, 6_m);
    rz::Point point3(6_m, 0_m);

    ASSERT_NEAR(circumradius(point1, point2, point3).convert(meter), 5, EPSILON);
}

TEST(TranslationTest, circleLineIntersectionTwo) {
    rz::Point start(0_m, -4_m);
    rz::Point end(2_m, 2_m);
    rz::Point point(2.3_m, -1.4_m);
    rz::QLength radius = 3_m;

    ASSERT_TRUE(rz::circleLineIntersection(start, end, point, radius).has_value());
    ASSERT_NEAR(rz::circleLineIntersection(start, end, point, radius).value(), 0.926, 0.01);
}

TEST(TranslationTest, circleLineIntersectionOne) {
    rz::Point start(0_m, -4_m);
    rz::Point end(2_m, 2_m);
    rz::Point point(3.5_m, 0_m);
    rz::QLength radius = 3_m;

    ASSERT_TRUE(rz::circleLineIntersection(start, end, point, radius).has_value());
    ASSERT_NEAR(rz::circleLineIntersection(start, end, point, radius).value(), 0.4295, 0.01);
}

TEST(TranslationTest, circleLineIntersectionNone) {
    rz::Point start(0_m, -4_m);
    rz::Point end(2_m, 2_m);
    rz::Point point(4.8_m, 0_m);
    rz::QLength radius = 3_m;

    ASSERT_FALSE(rz::circleLineIntersection(start, end, point, radius).has_value());
}