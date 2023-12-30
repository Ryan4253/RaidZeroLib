#include "RaidZeroLib/api/Geometry/Pose.hpp"
#include <cmath>
#include <gtest/gtest.h>
using namespace okapi;

const double EPSILON = 0.0001;

TEST(PoseTest, defaultConstructor) {
    rz::Pose pose;

    EXPECT_EQ(pose.X(), 0_m);
    EXPECT_EQ(pose.Y(), 0_m);
    EXPECT_EQ(pose.Theta(), 0_deg);
}

TEST(PoseTest, constructor) {
    rz::Pose pose(rz::Translation(1_m, 2_m), 3_deg);

    EXPECT_EQ(pose.X(), 1_m);
    EXPECT_EQ(pose.Y(), 2_m);
    EXPECT_EQ(pose.Theta(), 3_deg);
}

TEST(PoseTest, xyConstructor) {
    rz::Pose pose(1_m, 2_m, 3_deg);

    EXPECT_EQ(pose.X(), 1_m);
    EXPECT_EQ(pose.Y(), 2_m);
    EXPECT_EQ(pose.Theta(), 3_deg);
}

TEST(PoseTest, odomStateConstructor) {
    OdomState state{1_m, 2_m, 3_deg};
    rz::Pose pose(state);

    EXPECT_EQ(pose.X(), 1_m);
    EXPECT_EQ(pose.Y(), -2_m);
    EXPECT_EQ(pose.Theta(), -3_deg);
}

TEST(PoseTest, getters) {
    rz::Pose pose(rz::Translation(1_m, 2_m), 3_deg);

    EXPECT_EQ(pose.getTranslation(), rz::Translation(1_m, 2_m));
    EXPECT_EQ(pose.getRotation(), rz::Rotation(3_deg));
    EXPECT_EQ(pose.X(), 1_m);
    EXPECT_EQ(pose.Y(), 2_m);
    EXPECT_EQ(pose.Theta(), 3_deg);
}

TEST(PoseTest, addition) {
    rz::Pose initial{1_m, 2_m, 45_deg};
    rz::Transform transform{rz::Translation{5_m, 0_m}, 5_deg};

    const auto transformed = initial + transform;

    EXPECT_NEAR(transformed.X().convert(meter), 1.0 + 5.0 / std::sqrt(2.0), EPSILON);
    EXPECT_NEAR(transformed.Y().convert(meter), 2.0 + 5.0 / std::sqrt(2.0), EPSILON);
    EXPECT_NEAR(transformed.Theta().convert(degree), 50.0, EPSILON);
}

TEST(PoseTest, subtraction) {
    rz::Pose initial{0_m, 0_m, 45_deg};
    rz::Pose final{5_m, 5_m, 45_deg};

    const auto finalRelativeToInitial = final - initial;

    EXPECT_NEAR(finalRelativeToInitial.X().convert(meter), 5.0 * std::sqrt(2.0), EPSILON);
    EXPECT_NEAR(finalRelativeToInitial.Y().convert(meter), 0.0, EPSILON);
    EXPECT_NEAR(finalRelativeToInitial.Theta().convert(degree), 0.0, EPSILON);
}

TEST(PoseTest, scalarMultiplication) {
    rz::Pose pose(rz::Translation(1_m, 2_m), 3_deg);

    const auto scaled = pose * 2.0;

    EXPECT_EQ(scaled.X(), 2_m);
    EXPECT_EQ(scaled.Y(), 4_m);
    EXPECT_EQ(scaled.Theta(), 6_deg);
}

TEST(PoseTest, scalarDivision) {
    rz::Pose pose(rz::Translation(1_m, 2_m), 3_deg);

    const auto scaled = pose / 2.0;

    EXPECT_EQ(scaled.X(), 0.5_m);
    EXPECT_EQ(scaled.Y(), 1_m);
    EXPECT_EQ(scaled.Theta(), 1.5_deg);
}

TEST(PoseTest, equality) {
    rz::Pose pose1(rz::Translation(1_m, 2_m), 3_deg);
    rz::Pose pose2(rz::Translation(1_m, 2_m), 3_deg);
    rz::Pose pose3(rz::Translation(1_m, 2_m), 4_deg);

    EXPECT_TRUE(pose1 == pose2);
    EXPECT_FALSE(pose1 == pose3);
}

TEST(PoseTest, inequality) {
    rz::Pose pose1(rz::Translation(1_m, 2_m), 3_deg);
    rz::Pose pose2(rz::Translation(1_m, 2_m), 3_deg);
    rz::Pose pose3(rz::Translation(1_m, 2_m), 4_deg);

    EXPECT_FALSE(pose1 != pose2);
    EXPECT_TRUE(pose1 != pose3);
}

TEST(PoseTest, assignment) {
    rz::Pose pose1(rz::Translation(1_m, 2_m), 3_deg);
    rz::Pose pose2(rz::Translation(1_m, 2_m), 4_deg);

    EXPECT_FALSE(pose1 == pose2);

    pose1 = pose2;

    EXPECT_TRUE(pose1 == pose2);
}

TEST(PoseTest, exponential) {
    rz::Pose pose1(rz::Translation(0_m, 0_m), 0_deg);
    rz::Twist twist(2.5_pi * meter, 0_m, 90_deg);

    const auto pose2 = pose1.exp(twist);

    EXPECT_NEAR(pose2.X().convert(meter), 5, EPSILON);
    EXPECT_NEAR(pose2.Y().convert(meter), 5, EPSILON);
    EXPECT_NEAR(pose2.Theta().convert(degree), 90, EPSILON);
}

TEST(PoseTest, logarithm) {
    rz::Pose pose1(rz::Translation(0_m, 0_m), 0_deg);
    rz::Pose pose2(rz::Translation(5_m, 5_m), 90_deg);

    const auto twist = pose1.log(pose2);

    EXPECT_NEAR(twist.dX().convert(meter), 2.5_pi, EPSILON);
    EXPECT_NEAR(twist.dY().convert(meter), 0, EPSILON);
    EXPECT_NEAR(twist.dTheta().convert(degree), 90, EPSILON);
}

TEST(PoseTest, curvatureToPoint) {
    rz::Pose pose(rz::Translation(4_m, 2_m), 90_deg);
    rz::Translation target(2_m, 4_m);

    EXPECT_NEAR(rz::curvatureToPoint(pose, target).convert(radpm), -0.5, EPSILON);
}