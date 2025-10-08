#include "RaidZeroLib/api/Geometry/Transform.hpp"
#include "RaidZeroLib/api/Geometry/Pose.hpp"
#include <gtest/gtest.h>

TEST(transformTest, constructor) {
    rz::Transform transform(rz::Point(au::inches(1.0), au::inches(2.0)), rz::Rotation(au::degrees(90)));

    EXPECT_TRUE(transform.getPoint().isApprox(rz::Point(au::inches(1.0), au::inches(2.0))));
    EXPECT_TRUE(transform.getRotation().isApprox(rz::Rotation(au::degrees(90))));
}
TEST(transformTest, poseConstructor) {
    rz::Pose start(au::inches(1.0), au::inches(2.0), rz::Rotation(au::degrees(90)));
    rz::Pose end(au::inches(3.0), au::inches(4.0), rz::Rotation(au::degrees(180)));
    rz::Transform transform(start, end);

    EXPECT_TRUE(transform.getPoint().isApprox(rz::Point(au::inches(2.0), au::inches(-2.0))));
    EXPECT_TRUE(transform.getRotation().isApprox(rz::Rotation(au::degrees(90))));
    EXPECT_TRUE(start.transformBy(transform).isApprox(end));
}

TEST(transformTest, getter) {
    rz::Transform transform(rz::Point(au::inches(1.0), au::inches(2.0)), rz::Rotation(au::degrees(90)));

    EXPECT_TRUE(transform.getPoint().isApprox(rz::Point(au::inches(1.0), au::inches(2.0))));
    EXPECT_TRUE(transform.getRotation().isApprox(rz::Rotation(au::degrees(90))));
    EXPECT_EQ(transform.X().in(au::inches), 1);
    EXPECT_EQ(transform.Y().in(au::inches), 2);
    EXPECT_EQ(transform.Theta().in(au::degrees), 90);
}

TEST(transformTest, addition) {
    rz::Transform transform1(rz::Point(au::inches(1.0), au::inches(2.0)), rz::Rotation(au::degrees(90)));
    rz::Transform transform2(rz::Point(au::inches(3.0), au::inches(4.0)), rz::Rotation(au::degrees(180)));
    rz::Transform transform3 = transform1 + transform2;

    rz::Pose base(au::meters(1.0), au::meters(2.0), rz::Rotation(au::degrees(30)));
    rz::Pose pose1 = base.transformBy(transform1).transformBy(transform2);
    rz::Pose pose2 = base.transformBy(transform3);

    EXPECT_TRUE(pose1.isApprox(pose2));
}

TEST(transformTest, isApprox) {
    rz::Transform transform1(rz::Point(au::inches(1.0), au::inches(2.0)),
                             rz::Rotation(au::degrees(90)));
    rz::Transform transform2(rz::Point(au::inches(1.0), au::inches(2.0)),
                             rz::Rotation(au::degrees(90)));
    rz::Transform transform3(rz::Point(au::inches(1.0), au::inches(2.0)),
                             rz::Rotation(au::degrees(45)));

    EXPECT_TRUE(transform1.isApprox(transform2));
    EXPECT_FALSE(transform1.isApprox(transform3));
}

TEST(transformTest, inverse) {
    rz::Pose start(au::inches(1.0), au::inches(2.0), rz::Rotation(au::degrees(90)));
    rz::Pose end  (au::inches(3.0), au::inches(4.0), rz::Rotation(au::degrees(180)));
    rz::Transform transform(start, end);

    EXPECT_TRUE(start.isApprox(end.transformBy(transform.inverse())));
}
