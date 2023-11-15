#include "RaidZeroLib/api/Geometry/Transform.hpp"
#include <gtest/gtest.h>
using namespace okapi;

TEST(transformTest, defaultConstructor) {
    rz::Transform transform;

    EXPECT_EQ(transform.getTranslation(), rz::Translation());
    EXPECT_EQ(transform.getRotation(), rz::Rotation());
}

TEST(transformTest, constructor) {
    rz::Transform transform(rz::Translation(1.0_in, 2.0_in), rz::Rotation(90_deg));

    EXPECT_EQ(transform.getTranslation(), rz::Translation(1.0_in, 2.0_in));
    EXPECT_EQ(transform.getRotation(), rz::Rotation(90_deg));
}

TEST(transformTest, poseConstructor) {
    rz::Pose start(1.0_in, 2.0_in, 90_deg);
    rz::Pose end(3.0_in, 4.0_in, 180_deg);
    rz::Transform transform(start, end);

    EXPECT_EQ(transform.getTranslation(), rz::Translation(2.0_in, -2.0_in));
    EXPECT_EQ(transform.getRotation(), rz::Rotation(90_deg));
    EXPECT_EQ(start + transform, end);
}

TEST(transformTest, getter) {
    rz::Transform transform(rz::Translation(1.0_in, 2.0_in), rz::Rotation(90_deg));

    EXPECT_EQ(transform.getTranslation(), rz::Translation(1.0_in, 2.0_in));
    EXPECT_EQ(transform.getRotation(), rz::Rotation(90_deg));
    EXPECT_EQ(transform.X(), 1.0_in);
    EXPECT_EQ(transform.Y(), 2.0_in);
    EXPECT_EQ(transform.Theta(), 90_deg);
}

TEST(transformTest, addition) {
    rz::Transform transform1(rz::Translation(1.0_in, 2.0_in), rz::Rotation(90_deg));
    rz::Transform transform2(rz::Translation(3.0_in, 4.0_in), rz::Rotation(180_deg));
    rz::Transform transform3 = transform1 + transform2;
    rz::Pose pose1(1.0_m, 2.0_m, 30_deg);

    EXPECT_EQ(pose1 + transform1 + transform2, pose1 + transform3);
}

TEST(transformTest, multiplication) {
    rz::Transform transform1(rz::Translation(1.0_in, 2.0_in), rz::Rotation(90_deg));
    rz::Transform scaled = transform1 * 2;

    EXPECT_EQ(scaled.getTranslation(), rz::Translation(2.0_in, 4.0_in));
    EXPECT_EQ(scaled.getRotation(), rz::Rotation(180_deg));
}

TEST(transformTest, divison) {
    rz::Transform transform1(rz::Translation(1.0_in, 2.0_in), rz::Rotation(90_deg));
    rz::Transform scaled = transform1 / 2;

    EXPECT_EQ(scaled.getTranslation(), rz::Translation(0.5_in, 1.0_in));
    EXPECT_EQ(scaled.getRotation(), rz::Rotation(45_deg));
}

TEST(transformTest, equality) {
    rz::Transform transform1(rz::Translation(1.0_in, 2.0_in), rz::Rotation(90_deg));
    rz::Transform transform2(rz::Translation(1.0_in, 2.0_in), rz::Rotation(90_deg));
    rz::Transform transform3(rz::Translation(1.0_in, 2.0_in), rz::Rotation(45_deg));

    EXPECT_EQ(transform1, transform2);
    EXPECT_NE(transform1, transform3);
}

TEST(transformTest, inequality) {
    rz::Transform transform1(rz::Translation(1.0_in, 2.0_in), rz::Rotation(90_deg));
    rz::Transform transform2(rz::Translation(1.0_in, 2.0_in), rz::Rotation(90_deg));
    rz::Transform transform3(rz::Translation(1.0_in, 2.0_in), rz::Rotation(45_deg));

    EXPECT_FALSE(transform1 != transform2);
    EXPECT_TRUE(transform1 != transform3);
}

TEST(transformTest, assignment) {
    rz::Transform transform1(rz::Translation(1.0_in, 2.0_in), rz::Rotation(90_deg));
    rz::Transform transform2(rz::Translation(3.0_in, 4.0_in), rz::Rotation(180_deg));
    transform1 = transform2;

    EXPECT_EQ(transform1, transform2);
}

TEST(transformTest, inverse) {
    rz::Pose start(1.0_in, 2.0_in, 90_deg);
    rz::Pose end(3.0_in, 4.0_in, 180_deg);
    rz::Transform transform(start, end);
    EXPECT_EQ(start, end + transform.inverse());
}
