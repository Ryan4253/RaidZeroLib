#include "RaidZeroLib/api/Pathing/PiecewiseCubicBezier.hpp"
#include <cmath>
#include <gtest/gtest.h>

const double EPSILON = 0.0001;

TEST(PiecewiseCubicBezierTest, getPoint) {
    rz::CubicBezier::Knot knot1(
        au::meters(0), au::meters(0), au::degrees(0), au::meters(2));
    rz::CubicBezier::Knot knot2(
        au::meters(4), au::meters(2), au::degrees(0), au::meters(2));
    rz::CubicBezier::Knot knot3(
        au::meters(8), au::meters(0), au::degrees(45), au::meters(2));

    rz::CubicBezier bezier1(knot1, knot2);
    rz::CubicBezier bezier2(knot2, knot3);
    rz::PiecewiseCubicBezier path({knot1, knot2, knot3});

    EXPECT_TRUE(path.getPoint(0).isApprox(bezier1.getPoint(0)));
    EXPECT_TRUE(path.getPoint(0.25).isApprox(bezier1.getPoint(0.5)));
    EXPECT_TRUE(path.getPoint(0.5).isApprox(bezier2.getPoint(0)));
    EXPECT_TRUE(path.getPoint(0.6).isApprox(bezier2.getPoint(0.2)));
    EXPECT_TRUE(path.getPoint(1).isApprox(bezier2.getPoint(1)));
}

TEST(PiecewiseCubicBezierTest, getVelocity) {
    rz::CubicBezier::Knot knot1(
        au::meters(0), au::meters(0), au::degrees(0), au::meters(2));
    rz::CubicBezier::Knot knot2(
        au::meters(4), au::meters(2), au::degrees(0), au::meters(2));
    rz::CubicBezier::Knot knot3(
        au::meters(8), au::meters(0), au::degrees(45), au::meters(2));

    rz::CubicBezier bezier1(knot1, knot2);
    rz::CubicBezier bezier2(knot2, knot3);
    rz::PiecewiseCubicBezier path({knot1, knot2, knot3});

    EXPECT_TRUE(path.getVelocity(0).isApprox(bezier1.getVelocity(0)));
    EXPECT_TRUE(path.getVelocity(0.25).isApprox(bezier1.getVelocity(0.5)));
    EXPECT_TRUE(path.getVelocity(0.5).isApprox(bezier2.getVelocity(0)));
    EXPECT_TRUE(path.getVelocity(0.6).isApprox(bezier2.getVelocity(0.2)));
    EXPECT_TRUE(path.getVelocity(1).isApprox(bezier2.getVelocity(1)));
}

TEST(PiecewiseCubicBezierTest, getAcceleration) {
    rz::CubicBezier::Knot knot1(
        au::meters(0), au::meters(0), au::degrees(0), au::meters(2));
    rz::CubicBezier::Knot knot2(
        au::meters(4), au::meters(2), au::degrees(0), au::meters(2));
    rz::CubicBezier::Knot knot3(
        au::meters(8), au::meters(0), au::degrees(45), au::meters(2));

    rz::CubicBezier bezier1(knot1, knot2);
    rz::CubicBezier bezier2(knot2, knot3);
    rz::PiecewiseCubicBezier path({knot1, knot2, knot3});

    EXPECT_TRUE(path.getAcceleration(0).isApprox(bezier1.getAcceleration(0)));
    EXPECT_TRUE(path.getAcceleration(0.25).isApprox(bezier1.getAcceleration(0.5)));
    EXPECT_TRUE(path.getAcceleration(0.5).isApprox(bezier2.getAcceleration(0)));
    EXPECT_TRUE(path.getAcceleration(0.6).isApprox(bezier2.getAcceleration(0.2)));
    EXPECT_TRUE(path.getAcceleration(1).isApprox(bezier2.getAcceleration(1)));
}
