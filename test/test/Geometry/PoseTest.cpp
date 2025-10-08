#include "RaidZeroLib/api/Geometry/Pose.hpp"
#include "RaidZeroLib/api/Geometry/Twist.hpp"
#include "RaidZeroLib/api/Geometry/Transform.hpp"
#include "okapi//api/odometry/odomState.hpp"
#include <cmath>
#include <gtest/gtest.h>

const double EPSILON = 0.0001;

TEST(PoseTest, constructor) {
    rz::Pose pose(rz::Point(au::meters(1), au::meters(2)), rz::Rotation(au::degrees(3)));

    EXPECT_NEAR(pose.X().in(au::meters), 1, EPSILON);
    EXPECT_NEAR(pose.Y().in(au::meters), 2, EPSILON);
    EXPECT_NEAR(pose.Theta().in(au::degrees), 3, EPSILON);
}

TEST(PoseTest, xyRotationConstructor) {
    rz::Pose pose(au::meters(1), au::meters(2), rz::Rotation(au::degrees(3)));

    EXPECT_NEAR(pose.X().in(au::meters), 1, EPSILON);
    EXPECT_NEAR(pose.Y().in(au::meters), 2, EPSILON);
    EXPECT_NEAR(pose.Theta().in(au::degrees), 3, EPSILON);
}

TEST(PoseTest, xyaConstructor) {
    rz::Pose pose(au::meters(1), au::meters(2), au::degrees(3));

    EXPECT_NEAR(pose.X().in(au::meters), 1, EPSILON);
    EXPECT_NEAR(pose.Y().in(au::meters), 2, EPSILON);
    EXPECT_NEAR(pose.Theta().in(au::degrees), 3, EPSILON);
}

TEST(PoseTest, odomStateConstructor) {
    okapi::OdomState state{1 * okapi::meter, 2 * okapi::meter, 3 * okapi::degree};
    rz::Pose pose(state);

    EXPECT_NEAR(pose.X().in(au::meters), 1, EPSILON);
    EXPECT_NEAR(pose.Y().in(au::meters), -2, EPSILON);
    EXPECT_NEAR(pose.Theta().in(au::degrees), -3, EPSILON);
}

TEST(PoseTest, getters) {
    rz::Pose pose(au::meters(1), au::meters(2), au::degrees(3));

    EXPECT_TRUE(pose.getPoint().isApprox(rz::Point(au::meters(1), au::meters(2))));
    EXPECT_TRUE(pose.getRotation().isApprox(rz::Rotation(au::degrees(3))));
    EXPECT_NEAR(pose.X().in(au::meters), 1, EPSILON);
    EXPECT_NEAR(pose.Y().in(au::meters), 2, EPSILON);
    EXPECT_NEAR(pose.Theta().in(au::degrees), 3, EPSILON);
}

TEST(PoseTest, transformBy) {
    rz::Pose initial{au::meters(1), au::meters(2), au::degrees(45)};
    rz::Transform transform{rz::Point(au::meters(5), au::meters(0)), rz::Rotation(au::degrees(5))};

    const auto transformed = initial.transformBy(transform);

    EXPECT_NEAR(transformed.X().in(au::meter), 1.0 + 5.0 / std::sqrt(2.0), EPSILON);
    EXPECT_NEAR(transformed.Y().in(au::meters), 2.0 + 5.0 / std::sqrt(2.0), EPSILON);
    EXPECT_NEAR(transformed.Theta().in(au::degrees), 50.0, EPSILON);
}

TEST(PoseTest, relativeTo) {
    rz::Pose initial(au::meters(0), au::meters(0), au::degrees(45));
    rz::Pose final(au::meters(5), au::meters(5), au::degrees(45));

    const auto finalRelativeToInitial = final.relativeTo(initial);

    EXPECT_NEAR(finalRelativeToInitial.X().in(au::meters), 5.0 * std::sqrt(2.0), EPSILON);
    EXPECT_NEAR(finalRelativeToInitial.Y().in(au::meters), 0.0, EPSILON);
    EXPECT_NEAR(finalRelativeToInitial.Theta().in(au::degrees), 0.0, EPSILON);
}

TEST(PoseTest, isApprox) {
    rz::Pose pose1(au::meters(1), au::meters(2), au::degrees(3));
    rz::Pose pose2(au::meters(1), au::meters(2), au::degrees(3));
    rz::Pose pose3(au::meters(1), au::meters(2), au::degrees(4));

    EXPECT_TRUE(pose1.isApprox(pose2));
    EXPECT_FALSE(pose1.isApprox(pose3));
}

TEST(PoseTest, exponential) {
    rz::Pose pose1;
    rz::Twist twist(au::meters(2.5 * M_PI), au::meters(0), au::degrees(90));

    const auto pose2 = pose1.exp(twist);

    EXPECT_NEAR(pose2.X().in(au::meters), 5, EPSILON);
    EXPECT_NEAR(pose2.Y().in(au::meters), 5, EPSILON);
    EXPECT_NEAR(pose2.Theta().in(au::degrees), 90, EPSILON);
}

TEST(PoseTest, logarithm) {
    rz::Pose pose1;
    rz::Pose pose2(au::meters(5), au::meters(5), au::degrees(90));

    const auto twist = pose1.log(pose2);

    EXPECT_NEAR(twist.dX().in(au::meter), 2.5 * M_PI, EPSILON);
    EXPECT_NEAR(twist.dY().in(au::meter), 0, EPSILON);
    EXPECT_NEAR(twist.dTheta().in(au::degree), 90, EPSILON);
}

TEST(PoseTest, curvatureToPoint) {
    rz::Pose pose(au::meters(4), au::meters(2), au::degrees(90));
    rz::Point target(au::meters(2), au::meters(4));

    EXPECT_NEAR(rz::curvatureToPoint(pose, target).in(au::inverse(au::meter)), -0.5, EPSILON);
}

TEST(PoseTest, curvatureToPointZero) {
    rz::Pose pose(au::meters(4), au::meters(2), au::degrees(0));
    rz::Point target(au::meters(6), au::meters(2));

    EXPECT_NEAR(rz::curvatureToPoint(pose, target).in(au::inverse(au::meter)), 0, EPSILON);
}
