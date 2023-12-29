#include "RaidZeroLib/api/Pathing/PiecewiseCubicBezier.hpp"
#include <cmath>
#include <gtest/gtest.h>
using namespace okapi;

const double EPSILON = 0.0001;

TEST(PiecewiseCubicBezierTest, getPoint) {
    rz::CubicBezier::Knot knot1(0_m, 0_m, 0_deg, 2_m);
    rz::CubicBezier::Knot knot2(4_m, 2_m, 0_deg, 2_m);
    rz::CubicBezier::Knot knot3(8_m, 0_m, 45_deg, 2_m);

    rz::CubicBezier bezier1(knot1, knot2);
    rz::CubicBezier bezier2(knot2, knot3);
    rz::PiecewiseCubicBezier path({knot1, knot2, knot3});

    EXPECT_EQ(path.getPoint(0), bezier1.getPoint(0));
    EXPECT_EQ(path.getPoint(0.25), bezier1.getPoint(0.5));
    EXPECT_EQ(path.getPoint(0.5), bezier2.getPoint(0));
    EXPECT_EQ(path.getPoint(0.6), bezier2.getPoint(0.2));
    EXPECT_EQ(path.getPoint(1), bezier2.getPoint(1));
}

TEST(PiecewiseCubicBezierTest, getVelocity) {
    rz::CubicBezier::Knot knot1(0_m, 0_m, 0_deg, 2_m);
    rz::CubicBezier::Knot knot2(4_m, 2_m, 0_deg, 2_m);
    rz::CubicBezier::Knot knot3(8_m, 0_m, 45_deg, 2_m);

    rz::CubicBezier bezier1(knot1, knot2);
    rz::CubicBezier bezier2(knot2, knot3);
    rz::PiecewiseCubicBezier path({knot1, knot2, knot3});

    EXPECT_EQ(path.getVelocity(0), bezier1.getVelocity(0));
    EXPECT_EQ(path.getVelocity(0.25), bezier1.getVelocity(0.5));
    EXPECT_EQ(path.getVelocity(0.5), bezier2.getVelocity(0));
    EXPECT_EQ(path.getVelocity(0.6), bezier2.getVelocity(0.2));
    EXPECT_EQ(path.getVelocity(1), bezier2.getVelocity(1));
}

TEST(PiecewiseCubicBezierTest, getAcceleration) {
    rz::CubicBezier::Knot knot1(0_m, 0_m, 0_deg, 2_m);
    rz::CubicBezier::Knot knot2(4_m, 2_m, 0_deg, 2_m);
    rz::CubicBezier::Knot knot3(8_m, 0_m, 45_deg, 2_m);

    rz::CubicBezier bezier1(knot1, knot2);
    rz::CubicBezier bezier2(knot2, knot3);
    rz::PiecewiseCubicBezier path({knot1, knot2, knot3});

    EXPECT_EQ(path.getAcceleration(0), bezier1.getAcceleration(0));
    EXPECT_EQ(path.getAcceleration(0.25), bezier1.getAcceleration(0.5));
    EXPECT_EQ(path.getAcceleration(0.5), bezier2.getAcceleration(0));
    EXPECT_EQ(path.getAcceleration(0.6), bezier2.getAcceleration(0.2));
    EXPECT_EQ(path.getAcceleration(1), bezier2.getAcceleration(1));
}