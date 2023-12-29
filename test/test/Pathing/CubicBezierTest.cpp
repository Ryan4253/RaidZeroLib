#include "RaidZeroLib/api/Pathing/CubicBezier.hpp"
#include <cmath>
#include <gtest/gtest.h>
using namespace okapi;

const double EPSILON = 0.0001;

TEST(CubicBezierKnot, constructor) {
    rz::CubicBezier::Knot knot(0_m, 0_m, 45_deg, sqrt(2) * meter);

    EXPECT_EQ(knot.getPoint(), rz::Point(0_m, 0_m));
    EXPECT_EQ(knot.getForwardControl(), rz::Point(1_m, 1_m));
    EXPECT_EQ(knot.getBackwardControl(), rz::Point(-1_m, -1_m));
}

TEST(CubicBezier, getPoint) {
    rz::CubicBezier::Knot knot1(0_m, 0_m, 0_deg, 2_m);
    rz::CubicBezier::Knot knot2(4_m, 2_m, 0_deg, 2_m);
    rz::CubicBezier bezier(knot1, knot2);

    EXPECT_EQ(bezier.getPoint(0), rz::Point(0_m, 0_m));
    EXPECT_EQ(bezier.getPoint(0.25), rz::Point(1.1875_m, 0.3125_m));
    EXPECT_EQ(bezier.getPoint(0.5), rz::Point(2_m, 1_m));
    EXPECT_EQ(bezier.getPoint(0.75), rz::Point(2.8125_m, 1.6875_m));
    EXPECT_EQ(bezier.getPoint(1), rz::Point(4_m, 2_m));
}

TEST(CubicBezier, getVelocity) {
    rz::CubicBezier::Knot knot1(0_m, 0_m, 0_deg, 2_m);
    rz::CubicBezier::Knot knot2(4_m, 2_m, 0_deg, 2_m);
    rz::CubicBezier bezier(knot1, knot2);

    EXPECT_EQ(bezier.getVelocity(0), rz::Point(6_m, 0_m));
    EXPECT_EQ(bezier.getVelocity(0.25), rz::Point(3.75_m, 2.25_m));
    EXPECT_EQ(bezier.getVelocity(0.5), rz::Point(3_m, 3_m));
    EXPECT_EQ(bezier.getVelocity(0.75), rz::Point(3.75_m, 2.25_m));
    EXPECT_EQ(bezier.getVelocity(1), rz::Point(6_m, 0_m));
}

TEST(CubicBezier, getAcceleration) {
    rz::CubicBezier::Knot knot1(0_m, 0_m, 0_deg, 2_m);
    rz::CubicBezier::Knot knot2(4_m, 2_m, 0_deg, 2_m);
    rz::CubicBezier bezier(knot1, knot2);

    EXPECT_EQ(bezier.getAcceleration(0), rz::Point(-12_m, 12_m));
    EXPECT_EQ(bezier.getAcceleration(0.25), rz::Point(-6_m, 6_m));
    EXPECT_EQ(bezier.getAcceleration(0.5), rz::Point(0_m, 0_m));
    EXPECT_EQ(bezier.getAcceleration(0.75), rz::Point(6_m, -6_m));
    EXPECT_EQ(bezier.getAcceleration(1), rz::Point(12_m, -12_m));
}