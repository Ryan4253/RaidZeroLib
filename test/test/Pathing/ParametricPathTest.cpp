#include "RaidZeroLib/api/Pathing/ParametricPath.hpp"
#include "RaidZeroLib/api/Pathing/CubicBezier.hpp"
#include "RaidZeroLib/api/Utility/Math.hpp"
#include <gtest/gtest.h>

constexpr double EPSILON = 0.002;

TEST(ParametricPathTest, getTheta) {
    std::unique_ptr<rz::ParametricPath> path = std::make_unique<rz::CubicBezier>(
        rz::CubicBezier::Knot(au::meters(0), au::meters(0), au::degrees(0), au::meters(2)),
        rz::CubicBezier::Knot(au::meters(4), au::meters(2), au::degrees(0), au::meters(2)));

    EXPECT_NEAR(path->getTheta(0.25).in(au::radian), 0.54043, EPSILON);
    EXPECT_NEAR(path->getTheta(1).in(au::radian), 0, EPSILON);
}

TEST(ParametricPathTest, getCurvature) {
    std::unique_ptr<rz::ParametricPath> path = std::make_unique<rz::CubicBezier>(
        rz::CubicBezier::Knot(au::meters(0), au::meters(0), au::degrees(0), au::meters(2)),
        rz::CubicBezier::Knot(au::meters(4), au::meters(2), au::degrees(0), au::meters(2)));

    const double curvature =
        1 / rz::circumradius(
                path->getPoint(0.2 - 0.001),
                path->getPoint(0.2),
                path->getPoint(0.2 + 0.001))
                .in(au::meter);

    EXPECT_NEAR(path->getCurvature(0.2).in(au::inverse(au::meter)), curvature, EPSILON);
}

TEST(ParametricPathTest, getLength) {
    std::unique_ptr<rz::ParametricPath> path = std::make_unique<rz::CubicBezier>(
        rz::CubicBezier::Knot(au::meters(0), au::meters(0), au::degrees(0), au::meters(2)),
        rz::CubicBezier::Knot(au::meters(4), au::meters(2), au::degrees(0), au::meters(2)));

    EXPECT_NEAR(path->getLength(0, 1).in(au::meter), 4.622, EPSILON);
    EXPECT_NEAR(path->getLength(0, 0.5).in(au::meter), 2.311, EPSILON);
    EXPECT_NEAR(path->getLength(0.5, 1).in(au::meter), 2.311, EPSILON);
}

TEST(ParametricPathTest, toDiscrete) {
    std::unique_ptr<rz::ParametricPath> path = std::make_unique<rz::CubicBezier>(
        rz::CubicBezier::Knot(au::meters(0), au::meters(0), au::degrees(0), au::meters(2)),
        rz::CubicBezier::Knot(au::meters(4), au::meters(2), au::degrees(0), au::meters(2)));

    const auto discrete = path->toDiscrete(au::inches(6), true);

    for (auto it = discrete.begin(); it < discrete.end() - 2; it++) {
        EXPECT_NEAR(it->distTo(*(it + 1)).in(au::inch), 6, EPSILON);
    }

    EXPECT_TRUE((discrete.end() - 2)->distTo(*(discrete.end() - 1)).in(au::inch) > 3);
}

TEST(ParametricPathTest, stepT) {
    std::unique_ptr<rz::ParametricPath> path = std::make_unique<rz::CubicBezier>(
        rz::CubicBezier::Knot(au::meters(0), au::meters(0), au::degrees(0), au::meters(2)),
        rz::CubicBezier::Knot(au::meters(4), au::meters(2), au::degrees(0), au::meters(2)));

    const double t = path->stepT(0.2, au::meters(0.1));
    EXPECT_NEAR(path->getPoint(0.2).distTo(path->getPoint(t)).in(au::meter), 0.1, EPSILON);
}
