#include "RaidZeroLib/api/Pathing/CubicBezier.hpp"
#include <cmath>
#include <gtest/gtest.h>

const double EPSILON = 0.0001;

TEST(CubicBezierKnotTest, constructor) {
    rz::CubicBezier::Knot control(au::meters(0), au::meters(0), au::degrees(45), au::meters(std::sqrt(2)));

    EXPECT_TRUE(control.getPoint().isApprox(rz::Point(au::meters(0), au::meters(0))));
    EXPECT_TRUE(control.getForwardControl().isApprox(rz::Point(au::meters(1), au::meters(1))));
    EXPECT_TRUE(control.getBackwardControl().isApprox(rz::Point(au::meters(-1), au::meters(-1))));
}

TEST(CubicBezierTest, getPoint) {
    rz::CubicBezier::Knot knot1(au::meters(0), au::meters(0), au::degrees(0), au::meters(2));
    rz::CubicBezier::Knot knot2(au::meters(4), au::meters(2), au::degrees(0), au::meters(2));
    rz::CubicBezier bezier(knot1, knot2);

    EXPECT_TRUE(bezier.getPoint(0).isApprox(rz::Point(au::meters(0), au::meters(0))));
    EXPECT_TRUE(bezier.getPoint(0.25).isApprox(rz::Point(au::meters(1.1875), au::meters(0.3125))));
    EXPECT_TRUE(bezier.getPoint(0.5).isApprox(rz::Point(au::meters(2), au::meters(1))));
    EXPECT_TRUE(bezier.getPoint(0.75).isApprox(rz::Point(au::meters(2.8125), au::meters(1.6875))));
    EXPECT_TRUE(bezier.getPoint(1).isApprox(rz::Point(au::meters(4), au::meters(2))));
}

TEST(CubicBezierTest, getVelocity) {
    rz::CubicBezier::Knot knot1(au::meters(0), au::meters(0), au::degrees(0), au::meters(2));
    rz::CubicBezier::Knot knot2(au::meters(4), au::meters(2), au::degrees(0), au::meters(2));
    rz::CubicBezier bezier(knot1, knot2);

    EXPECT_TRUE(bezier.getVelocity(0).isApprox(rz::Point(au::meters(6), au::meters(0))));
    EXPECT_TRUE(bezier.getVelocity(0.25).isApprox(rz::Point(au::meters(3.75), au::meters(2.25))));
    EXPECT_TRUE(bezier.getVelocity(0.5).isApprox(rz::Point(au::meters(3), au::meters(3))));
    EXPECT_TRUE(bezier.getVelocity(0.75).isApprox(rz::Point(au::meters(3.75), au::meters(2.25))));
    EXPECT_TRUE(bezier.getVelocity(1).isApprox(rz::Point(au::meters(6), au::meters(0))));
}

TEST(CubicBezierTest, getAcceleration) {
    rz::CubicBezier::Knot knot1(au::meters(0), au::meters(0), au::degrees(0), au::meters(2));
    rz::CubicBezier::Knot knot2(au::meters(4), au::meters(2), au::degrees(0), au::meters(2));
    rz::CubicBezier bezier(knot1, knot2);

    EXPECT_TRUE(bezier.getAcceleration(0).isApprox(rz::Point(au::meters(-12), au::meters(12))));
    EXPECT_TRUE(bezier.getAcceleration(0.25).isApprox(rz::Point(au::meters(-6), au::meters(6))));
    EXPECT_TRUE(bezier.getAcceleration(0.5).isApprox(rz::Point(au::meters(0), au::meters(0))));
    EXPECT_TRUE(bezier.getAcceleration(0.75).isApprox(rz::Point(au::meters(6), au::meters(-6))));
    EXPECT_TRUE(bezier.getAcceleration(1).isApprox(rz::Point(au::meters(12), au::meters(-12))));
}
