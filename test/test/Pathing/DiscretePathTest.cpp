#include "RaidZeroLib/api/Pathing/DiscretePath.hpp"
#include <gtest/gtest.h>

constexpr double EPSILON = 0.0001;

TEST(DiscretePathTest, initializerConstructor) {
    rz::DiscretePath path({{au::meters(0), au::meters(0)}, {au::meters(1), au::meters(1)}});
    EXPECT_EQ(path.size(), 2);
    EXPECT_TRUE(path[0].isApprox(rz::Point(au::meters(0), au::meters(0))));
    EXPECT_TRUE(path[1].isApprox(rz::Point(au::meters(1), au::meters(1))));
}

TEST(DiscretePathTest, rangeConstructor) {
    std::vector<rz::Point> waypoint{{au::meters(0), au::meters(0)}, {au::meters(1), au::meters(1)}};
    rz::DiscretePath path(waypoint);

    EXPECT_EQ(path.size(), 2);
    EXPECT_TRUE(path[0].isApprox(rz::Point(au::meters(0), au::meters(0))));
    EXPECT_TRUE(path[1].isApprox(rz::Point(au::meters(1), au::meters(1))));
}

TEST(DiscretePathTest, additionPoint) {
    rz::DiscretePath path({{au::meters(0), au::meters(0)}, {au::meters(1), au::meters(1)}});
    path = path + rz::Point(au::meters(2), au::meters(2));

    EXPECT_EQ(path.size(), 3);
    EXPECT_TRUE(path[0].isApprox(rz::Point(au::meters(0), au::meters(0))));
    EXPECT_TRUE(path[1].isApprox(rz::Point(au::meters(1), au::meters(1))));
    EXPECT_TRUE(path[2].isApprox(rz::Point(au::meters(2), au::meters(2))));
}

TEST(DiscretePathTest, additionPath) {
    rz::DiscretePath path({{au::meters(0), au::meters(0)}, {au::meters(1), au::meters(1)}});
    path = path + path;

    EXPECT_EQ(path.size(), 4);
    EXPECT_TRUE(path[0].isApprox(rz::Point(au::meters(0), au::meters(0))));
    EXPECT_TRUE(path[1].isApprox(rz::Point(au::meters(1), au::meters(1))));
    EXPECT_TRUE(path[2].isApprox(rz::Point(au::meters(0), au::meters(0))));
    EXPECT_TRUE(path[3].isApprox(rz::Point(au::meters(1), au::meters(1))));
}

TEST(DiscretePathTest, begin) {
    const rz::DiscretePath path({{au::meters(0), au::meters(0)}, {au::meters(1), au::meters(1)}});
    EXPECT_TRUE((*path.begin()).isApprox(path[0]));
}

TEST(DiscretePathTest, end) {
    const rz::DiscretePath path({{au::meters(0), au::meters(0)}, {au::meters(1), au::meters(1)}});
    EXPECT_TRUE((*(path.end() - 1)).isApprox(rz::Point(au::meters(1), au::meters(1))));
}

TEST(DiscretePathTest, rbegin) {
    const rz::DiscretePath path({{au::meters(0), au::meters(0)}, {au::meters(1), au::meters(1)}});
    EXPECT_TRUE((*path.rbegin()).isApprox(rz::Point(au::meters(1), au::meters(1))));
}

TEST(DiscretePathTest, rend) {
    const rz::DiscretePath path({{au::meters(0), au::meters(0)}, {au::meters(1), au::meters(1)}});
    EXPECT_TRUE((*(path.rend() - 1)).isApprox(rz::Point(au::meters(0), au::meters(0))));
}

TEST(DiscretePathTest, front) {
    const rz::DiscretePath path({{au::meters(0), au::meters(0)}, {au::meters(1), au::meters(1)}});
    EXPECT_TRUE(path.front().isApprox(rz::Point(au::meters(0), au::meters(0))));
}

TEST(DiscretePathTest, back) {
    const rz::DiscretePath path({{au::meters(0), au::meters(0)}, {au::meters(1), au::meters(1)}});
    EXPECT_TRUE(path.back().isApprox(rz::Point(au::meters(1), au::meters(1))));
}

TEST(DiscretePathTest, getCurvature) {
    rz::DiscretePath path({{au::meters(2), au::meters(8)},
                           {au::meters(6), au::meters(6)},
                           {au::meters(6), au::meters(0)}});

    ASSERT_NEAR(path.getCurvature(1).in(au::inverse(au::meters)), 0.2, EPSILON);
}

TEST(DiscretePathTest, getCurvatureLine) {
    rz::DiscretePath path({{au::meters(0), au::meters(0)},
                           {au::meters(1), au::meters(1)},
                           {au::meters(2), au::meters(2)}});

    ASSERT_NEAR(path.getCurvature(1).in(au::inverse(au::meters)), 0, EPSILON);
}

TEST(DiscretePathTest, getCurvatureEndpoint) {
    rz::DiscretePath path({{au::meters(0), au::meters(0)},
                           {au::meters(1), au::meters(1)},
                           {au::meters(2), au::meters(2)}});

    ASSERT_NEAR(path.getCurvature(0).in(au::inverse(au::meters)), 0, EPSILON);
    ASSERT_NEAR(path.getCurvature(2).in(au::inverse(au::meters)), 0, EPSILON);
}

TEST(DiscretePathTest, closestPoint) {
    rz::DiscretePath path({{au::meters(0), au::meters(0)},
                           {au::meters(1), au::meters(1)},
                           {au::meters(2), au::meters(2)},
                           {au::meters(3), au::meters(3)},
                           {au::meters(4), au::meters(4)}});
    rz::Point point(au::meters(4.5), au::meters(-1.2));
    ASSERT_EQ(rz::closestPoint(path.begin(), path.end(), point) - path.begin(), 2);
}
