#include "RaidZeroLib/api/Pathing/DiscretePath.hpp"
#include <gtest/gtest.h>
using namespace okapi;

constexpr double EPSILON = 0.0001;

TEST(DiscretePathTest, constructor) {
    rz::DiscretePath path;
    EXPECT_EQ(path.size(), 0);
}

TEST(DiscretePathTest, initializerConstructor) {
    rz::DiscretePath path({{0_in, 0_in}, {1_in, 1_in}});
    EXPECT_EQ(path.size(), 2);
    EXPECT_EQ(path[0], rz::Point(0_in, 0_in));
    EXPECT_EQ(path[1], rz::Point(1_in, 1_in));
}

TEST(DiscretePathTest, vectorConstructor) {
    std::vector<rz::Point> waypoint{{0_in, 0_in}, {1_in, 1_in}};
    rz::DiscretePath path(waypoint);

    EXPECT_EQ(path.size(), 2);
    EXPECT_EQ(path[0], rz::Point(0_in, 0_in));
    EXPECT_EQ(path[1], rz::Point(1_in, 1_in));
}

TEST(DiscretePathTest, additionPoint) {
    rz::DiscretePath path({{0_in, 0_in}, {1_in, 1_in}});
    path = path + rz::Point(2_in, 2_in);

    EXPECT_EQ(path.size(), 3);
    EXPECT_EQ(path[0], rz::Point(0_in, 0_in));
    EXPECT_EQ(path[1], rz::Point(1_in, 1_in));
    EXPECT_EQ(path[2], rz::Point(2_in, 2_in));
}

TEST(DiscretePathTest, additionPath) {
    rz::DiscretePath path({{0_in, 0_in}, {1_in, 1_in}});
    path = path + path;

    EXPECT_EQ(path.size(), 4);
    EXPECT_EQ(path[0], rz::Point(0_in, 0_in));
    EXPECT_EQ(path[1], rz::Point(1_in, 1_in));
    EXPECT_EQ(path[2], rz::Point(0_in, 0_in));
    EXPECT_EQ(path[3], rz::Point(1_in, 1_in));
}

TEST(DiscretePathTest, addAssignmentPoint) {
    rz::DiscretePath path({{0_in, 0_in}, {1_in, 1_in}});
    path += rz::Point(2_in, 2_in);

    EXPECT_EQ(path.size(), 3);
    EXPECT_EQ(path[0], rz::Point(0_in, 0_in));
    EXPECT_EQ(path[1], rz::Point(1_in, 1_in));
    EXPECT_EQ(path[2], rz::Point(2_in, 2_in));
}

TEST(DiscretePathTet, addAssignmentPath) {
    rz::DiscretePath path({{0_in, 0_in}, {1_in, 1_in}});
    path += rz::DiscretePath({{2_in, 2_in}, {3_in, 3_in}});

    EXPECT_EQ(path.size(), 4);
    EXPECT_EQ(path[0], rz::Point(0_in, 0_in));
    EXPECT_EQ(path[1], rz::Point(1_in, 1_in));
    EXPECT_EQ(path[2], rz::Point(2_in, 2_in));
    EXPECT_EQ(path[3], rz::Point(3_in, 3_in));
}

TEST(DiscretePathTest, begin) {
    rz::DiscretePath path({{0_in, 0_in}, {1_in, 1_in}});
    EXPECT_EQ(*path.begin(), rz::Point(0_in, 0_in));
}

TEST(DiscretePathTest, beginConst) {
    const rz::DiscretePath path({{0_in, 0_in}, {1_in, 1_in}});
    EXPECT_EQ(*path.begin(), path[0]);
}

TEST(DiscretePathTest, end) {
    rz::DiscretePath path({{0_in, 0_in}, {1_in, 1_in}});
    EXPECT_EQ(*(path.end() - 1), rz::Point(1_in, 1_in));
}

TEST(DiscretePathTest, endConst) {
    const rz::DiscretePath path({{0_in, 0_in}, {1_in, 1_in}});
    EXPECT_EQ(*(path.end() - 1), rz::Point(1_in, 1_in));
}

TEST(DiscretePathTest, rbegin) {
    rz::DiscretePath path({{0_in, 0_in}, {1_in, 1_in}});
    EXPECT_EQ(*path.rbegin(), rz::Point(1_in, 1_in));
}

TEST(DiscretePathTest, rbeginConst) {
    const rz::DiscretePath path({{0_in, 0_in}, {1_in, 1_in}});
    EXPECT_EQ(*path.rbegin(), rz::Point(1_in, 1_in));
}

TEST(DiscretePathTest, rend) {
    rz::DiscretePath path({{0_in, 0_in}, {1_in, 1_in}});
    EXPECT_EQ(*(path.rend() - 1), rz::Point(0_in, 0_in));
}

TEST(DiscretePathTest, rendConst) {
    const rz::DiscretePath path({{0_in, 0_in}, {1_in, 1_in}});
    EXPECT_EQ(*(path.rend() - 1), rz::Point(0_in, 0_in));
}

TEST(DiscretePathTest, front) {
    rz::DiscretePath path({{0_in, 0_in}, {1_in, 1_in}});
    EXPECT_EQ(path.front(), rz::Point(0_in, 0_in));
}

TEST(DiscretePathTest, frontConst) {
    const rz::DiscretePath path({{0_in, 0_in}, {1_in, 1_in}});
    EXPECT_EQ(path.front(), rz::Point(0_in, 0_in));
}

TEST(DiscretePathTest, back) {
    rz::DiscretePath path({{0_in, 0_in}, {1_in, 1_in}});
    EXPECT_EQ(path.back(), rz::Point(1_in, 1_in));
}

TEST(DiscretePathTest, backConst) {
    const rz::DiscretePath path({{0_in, 0_in}, {1_in, 1_in}});
    EXPECT_EQ(path.back(), rz::Point(1_in, 1_in));
}

TEST(DiscretePathTest, getCurvature) {
    rz::DiscretePath path({{2_m, 8_m}, {6_m, 6_m}, {6_m, 0_m}});

    ASSERT_NEAR(path.getCurvature(1).convert(radpm), 0.2, EPSILON);
}

TEST(DiscretePathTest, getCurvatureLine) {
    rz::DiscretePath path({{0_m, 0_m}, {1_m, 1_m}, {2_m, 2_m}});

    ASSERT_NEAR(path.getCurvature(1).convert(radpm), 0, EPSILON);
}

TEST(DiscretePathTest, getCurvatureEndpoint) {
    rz::DiscretePath path({{0_m, 0_m}, {1_m, 1_m}, {2_m, 2_m}});

    ASSERT_NEAR(path.getCurvature(0).convert(radpm), 0, EPSILON);
    ASSERT_NEAR(path.getCurvature(2).convert(radpm), 0, EPSILON);
}

TEST(DiscretePathTest, closestPoint) {
    rz::DiscretePath path({{0_m, 0_m}, {1_m, 1_m}, {2_m, 2_m}, {3_m, 3_m}, {4_m, 4_m}});
    rz::Point point(4.5_m, -1.2_m);
    ASSERT_EQ(rz::closestPoint(path.begin(), path.end(), point) - path.begin(), 2);
}