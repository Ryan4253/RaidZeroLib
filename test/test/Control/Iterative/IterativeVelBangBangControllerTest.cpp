#include "RaidZeroLib/api/Control/Iterative/IterativeVelBangBangController.hpp"
#include <gtest/gtest.h>
using namespace okapi;

constexpr double EPSILON = 0.0001;

TEST(IterativeVelBangBangControllerGainsTest, defaultConstructor) {
    rz::IterativeVelBangBangController::Gains gains;

    EXPECT_NEAR(gains.highPower, 0, EPSILON);
    EXPECT_NEAR(gains.targetPower, 0, EPSILON);
    EXPECT_NEAR(gains.lowPower, 0, EPSILON);
    EXPECT_NEAR(gains.deadband, 0, EPSILON);
}

TEST(IterativeVelBangBangControllerGainsTest, constructor) {
    rz::IterativeVelBangBangController::Gains gains(12000, 9000, 6000, 2);

    EXPECT_NEAR(gains.highPower, 0, EPSILON);
    EXPECT_NEAR(gains.targetPower, 0, EPSILON);
    EXPECT_NEAR(gains.lowPower, 0, EPSILON);
    EXPECT_NEAR(gains.deadband, 0, EPSILON);
}

TEST(IterativeVelBangBangControllerGainsTest, equality) {
    rz::IterativeVelBangBangController::Gains gains(12000, 9000, 6000, 2);
    rz::IterativeVelBangBangController::Gains gains2(12000, 9000, 6000, 2);
    rz::IterativeVelBangBangController::Gains gains3(12100, 9200, 6300, 3);

    EXPECT_TRUE(gains == gains2);
    EXPECT_FALSE(gains == gains3);
}

TEST(IterativeVelBangBangControllerGainsTest, inequality) {
    rz::IterativeVelBangBangController::Gains gains(12000, 9000, 6000, 2);
    rz::IterativeVelBangBangController::Gains gains2(12000, 9000, 6000, 2);
    rz::IterativeVelBangBangController::Gains gains3(12100, 9200, 6300, 3);

    EXPECT_FALSE(gains != gains2);
    EXPECT_TRUE(gains != gains3);
}

TEST(IterativeVelBangBangControllerTest, constructor) {
}