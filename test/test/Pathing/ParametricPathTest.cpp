#include "RaidZeroLib/api/Pathing/ParametricPath.hpp"
#include "RaidZeroLib/api/Pathing/CubicBezier.hpp"
#include "RaidZeroLib/api/Utility/Math.hpp"
#include <gtest/gtest.h>
using namespace okapi;

constexpr double EPSILON = 0.0001;

TEST(ParametricPathTest, getX) {
    std::unique_ptr<rz::ParametricPath> path = std::make_unique<rz::CubicBezier>(
        rz::CubicBezier::Knot(0_m, 0_m, 0_deg, 2_m), rz::CubicBezier::Knot(4_m, 2_m, 0_deg, 2_m));

    EXPECT_EQ(path->getX(0.2), path->getPoint(0.2).X());
}

TEST(ParametricPathTest, getY) {
    std::unique_ptr<rz::ParametricPath> path = std::make_unique<rz::CubicBezier>(
        rz::CubicBezier::Knot(0_m, 0_m, 0_deg, 2_m), rz::CubicBezier::Knot(4_m, 2_m, 0_deg, 2_m));

    EXPECT_EQ(path->getY(0.2), path->getPoint(0.2).Y());
}

TEST(ParametricPathTest, getdX) {
    std::unique_ptr<rz::ParametricPath> path = std::make_unique<rz::CubicBezier>(
        rz::CubicBezier::Knot(0_m, 0_m, 0_deg, 2_m), rz::CubicBezier::Knot(4_m, 2_m, 0_deg, 2_m));

    EXPECT_EQ(path->getdX(0.2), path->getVelocity(0.2).X());
}

TEST(ParametricPathTest, getdY) {
    std::unique_ptr<rz::ParametricPath> path = std::make_unique<rz::CubicBezier>(
        rz::CubicBezier::Knot(0_m, 0_m, 0_deg, 2_m), rz::CubicBezier::Knot(4_m, 2_m, 0_deg, 2_m));

    EXPECT_EQ(path->getdY(0.2), path->getVelocity(0.2).Y());
}

TEST(ParametricPathTest, getddX) {
    std::unique_ptr<rz::ParametricPath> path = std::make_unique<rz::CubicBezier>(
        rz::CubicBezier::Knot(0_m, 0_m, 0_deg, 2_m), rz::CubicBezier::Knot(4_m, 2_m, 0_deg, 2_m));

    EXPECT_EQ(path->getddX(0.2), path->getAcceleration(0.2).X());
}

TEST(ParametricPathTest, getddY) {
    std::unique_ptr<rz::ParametricPath> path = std::make_unique<rz::CubicBezier>(
        rz::CubicBezier::Knot(0_m, 0_m, 0_deg, 2_m), rz::CubicBezier::Knot(4_m, 2_m, 0_deg, 2_m));

    EXPECT_EQ(path->getddY(0.2), path->getAcceleration(0.2).Y());
}

TEST(ParametricPathTest, getTheta) {
    std::unique_ptr<rz::ParametricPath> path = std::make_unique<rz::CubicBezier>(
        rz::CubicBezier::Knot(0_m, 0_m, 0_deg, 2_m), rz::CubicBezier::Knot(4_m, 2_m, 0_deg, 2_m));

    EXPECT_NEAR(path->getTheta(0.25).Theta().convert(radian), 0.54043, EPSILON);
    EXPECT_NEAR(path->getTheta(1).Theta().convert(radian), 0, EPSILON);
}

TEST(ParametricPathTest, getCurvature) {
    std::unique_ptr<rz::ParametricPath> path = std::make_unique<rz::CubicBezier>(
        rz::CubicBezier::Knot(0_m, 0_m, 0_deg, 2_m), rz::CubicBezier::Knot(4_m, 2_m, 0_deg, 2_m));

    const double curvature =
        1 /
        rz::circumradius(path->getPoint(0.2 - 0.001), path->getPoint(0.2), path->getPoint(0.2 + 0.001)).convert(meter);

    EXPECT_NEAR(path->getCurvature(0.2).convert(radpm), curvature, EPSILON);
}

TEST(ParametricPathTest, getLength) {
    std::unique_ptr<rz::ParametricPath> path = std::make_unique<rz::CubicBezier>(
        rz::CubicBezier::Knot(0_m, 0_m, 0_deg, 2_m), rz::CubicBezier::Knot(4_m, 2_m, 0_deg, 2_m));

    EXPECT_NEAR(path->getLength(0, 1).convert(meter), 4.622, EPSILON);
    EXPECT_NEAR(path->getLength(0, 0.5).convert(meter), 2.311, EPSILON);
    EXPECT_NEAR(path->getLength(0.5, 1).convert(meter), 2.311, EPSILON);
}

TEST(ParametricPathTest, toDiscreteNumber) {
    std::unique_ptr<rz::ParametricPath> path = std::make_unique<rz::CubicBezier>(
        rz::CubicBezier::Knot(0_m, 0_m, 0_deg, 2_m), rz::CubicBezier::Knot(4_m, 2_m, 0_deg, 2_m));

    const auto discrete = path->toDiscrete(10, true);

    EXPECT_EQ(discrete.size(), 11);
    EXPECT_EQ(discrete[0], path->getPoint(0));
    EXPECT_EQ(discrete[4], path->getPoint(0.4));
    EXPECT_EQ(discrete[10], path->getPoint(1));
}

TEST(ParametricPathTest, toDiscreteLength) {
    std::unique_ptr<rz::ParametricPath> path = std::make_unique<rz::CubicBezier>(
        rz::CubicBezier::Knot(0_m, 0_m, 0_deg, 2_m), rz::CubicBezier::Knot(4_m, 2_m, 0_deg, 2_m));

    const auto discrete = path->toDiscrete(6_in, true);

    for (auto it = discrete.begin(); it < discrete.end() - 2; it++) {
        EXPECT_NEAR(it->distTo(*(it + 1)).convert(inch), 6, 0.2);
    }

    EXPECT_TRUE((discrete.end() - 2)->distTo(*(discrete.end() - 1)).convert(inch) > 3);
}

TEST(ParametricPathTest, stepT) {
    std::unique_ptr<rz::ParametricPath> path = std::make_unique<rz::CubicBezier>(
        rz::CubicBezier::Knot(0_m, 0_m, 0_deg, 2_m), rz::CubicBezier::Knot(4_m, 2_m, 0_deg, 2_m));

    const double t = path->stepT(0.2, 0.01_m);
    EXPECT_NEAR(path->getPoint(0.2).distTo(path->getPoint(t)).convert(meter), 0.01, EPSILON);
}
