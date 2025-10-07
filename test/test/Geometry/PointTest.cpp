#include "RaidZeroLib/api/Geometry/Point.hpp"
#include <cmath>
#include <gtest/gtest.h>

const double EPSILON = 0.0001;

TEST(PointTest, cartesianConstructor) {
    rz::Point point(au::meters(3), au::meters(4));

    EXPECT_EQ(point.X(), au::meters(3));
    EXPECT_EQ(point.Y(), au::meters(4));
}

TEST(PointTest, polarConstructor) {
    rz::Point point(au::meters(3), rz::Rotation(au::degrees(45)));

    EXPECT_NEAR(point.X().in(au::meter), 3 * sqrt(2) / 2, EPSILON);
    EXPECT_NEAR(point.Y().in(au::meter), 3 * sqrt(2) / 2, EPSILON);
}

TEST(PointTest, addition) {
    rz::Point p1(au::meters(3), au::meters(4));
    rz::Point p2(au::meters(5), au::meters(6));
    rz::Point sum = p1 + p2;

    EXPECT_NEAR(sum.X().in(au::meters), 8.0, EPSILON);
    EXPECT_NEAR(sum.Y().in(au::meters), 10.0, EPSILON);
}

TEST(PointTest, subtraction) {
    rz::Point p1(au::meters(3), au::meters(4));
    rz::Point p2(au::meters(5), au::meters(5));
    rz::Point diff = p1 - p2;

    EXPECT_NEAR(diff.X().in(au::meters), -2.0, EPSILON);
    EXPECT_NEAR(diff.Y().in(au::meters), -1.0, EPSILON);
}

TEST(PointTest, negation) {
    rz::Point p(au::meters(3), au::meters(4));
    p = -p;

    EXPECT_NEAR(p.X().in(au::meters), -3.0, EPSILON);
    EXPECT_NEAR(p.Y().in(au::meters), -4.0, EPSILON);
}

TEST(PointTest, scalarMultiplication) {
    rz::Point p(au::meters(3), au::meters(4));
    rz::Point scaled = p * 2.0;

    EXPECT_NEAR(scaled.X().in(au::meters), 6.0, EPSILON);
    EXPECT_NEAR(scaled.Y().in(au::meters), 8.0, EPSILON);
}

TEST(PointTest, scalarDivision) {
    rz::Point p(au::meters(3), au::meters(4));
    rz::Point scaled = p / 2.0;

    EXPECT_NEAR(scaled.X().in(au::meters), 1.5, EPSILON);
    EXPECT_NEAR(scaled.Y().in(au::meters), 2.0, EPSILON);
}

TEST(PointTest, theta) {
    rz::Point p(au::meters(0.5), au::meters(std::sqrt(3) / 2.0));

    EXPECT_NEAR(p.theta().in(au::degrees), 60.0, EPSILON);
}

TEST(PointTest, magnitude) {
    rz::Point p(au::meters(3), au::meters(4));

    EXPECT_NEAR(p.mag().in(au::meters), 5.0, EPSILON);
}

TEST(PointTest, distTo) {
    rz::Point p1(au::meters(1), au::meters(2));
    rz::Point p2(au::meters(4), au::meters(6));

    EXPECT_NEAR(p1.distTo(p2).in(au::meters), 5.0, EPSILON);
}

TEST(PointTest, angleTo) {
    rz::Point p1(au::meters(2), au::meters(2));
    rz::Point p2(au::meters(-1), au::meters(-std::sqrt(3)));

    EXPECT_NEAR(p1.angleTo(p2).in(au::degrees), -165.0, EPSILON);
    EXPECT_NEAR(p2.angleTo(p1).in(au::degrees), 165.0, EPSILON);
}

TEST(PointTest, dot) {
    rz::Point p1(au::meters(3), au::meters(4));
    rz::Point p2(au::meters(5), au::meters(6));

    EXPECT_NEAR(p1.dot(p2).in(au::squared(au::meters)), 39.0, EPSILON);
}

TEST(PointTest, wedge) {
    rz::Point p1(au::meters(3), au::meters(4));
    rz::Point p2(au::meters(5), au::meters(6));

    EXPECT_NEAR(p1.wedge(p2).in(au::squared(au::meters)), -2.0, EPSILON);
}

TEST(PointTest, project) {
    rz::Point p1(au::meters(3), au::meters(4));
    rz::Point p2(au::meters(12), au::meters(8));
    rz::Point proj = p1.project(p2);

    EXPECT_NEAR(proj.X().in(au::meters), 51.0 / 13.0, EPSILON);
    EXPECT_NEAR(proj.Y().in(au::meters), 34.0 / 13.0, EPSILON);
}

TEST(PointTest, rotateBy) {
    rz::Point p(au::meters(1), au::meters(1));
    rz::Rotation rot(au::degrees(45));
    rz::Point rotated = p.rotateBy(rot);

    EXPECT_NEAR(rotated.X().in(au::meters), 0.0, EPSILON);
    EXPECT_NEAR(rotated.Y().in(au::meters), std::sqrt(2.0), EPSILON);
}

TEST(PointTest, isApprox) {
    rz::Point p1(au::meters(3), au::meters(4));
    rz::Point p2(au::meters(3), au::meters(4));
    rz::Point p3(au::meters(5), au::meters(6));

    EXPECT_TRUE(p1.isApprox(p2));
    EXPECT_FALSE(p1.isApprox(p3));
}

TEST(PointTest, circumradius) {
    rz::Point A(au::meters(2), au::meters(8));
    rz::Point B(au::meters(6), au::meters(6));
    rz::Point C(au::meters(6), au::meters(0));

    EXPECT_NEAR(circumradius(A, B, C).in(au::meters), 5.0, EPSILON);
}

TEST(PointTest, circleLineIntersectionTwo) {
    rz::Point start(au::meters(0), au::meters(-4));
    rz::Point end(au::meters(2), au::meters(2));
    rz::Point center(au::meters(2.3), au::meters(-1.4));
    auto radius = au::meters(3);
    auto res = circleLineIntersection(start, end, center, radius);

    ASSERT_TRUE(res.has_value());
    EXPECT_NEAR(res.value(), 0.926, 0.01);
}

TEST(PointTest, circleLineIntersectionOne) {
    rz::Point start(au::meters(0), au::meters(-4));
    rz::Point end(au::meters(2), au::meters(2));
    rz::Point center(au::meters(3.5), au::meters(0));
    auto radius = au::meters(3);
    auto res = circleLineIntersection(start, end, center, radius);

    ASSERT_TRUE(res.has_value());
    EXPECT_NEAR(res.value(), 0.4295, 0.01);
}

TEST(PointTest, circleLineIntersectionNone) {
    rz::Point start(au::meters(0), au::meters(-4));
    rz::Point end(au::meters(2), au::meters(2));
    rz::Point center(au::meters(4.8), au::meters(0));
    auto radius = au::meters(3);

    EXPECT_FALSE(circleLineIntersection(start, end, center, radius).has_value());
}