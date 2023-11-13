#include "RaidZeroLib/api/Control/Feedforward/SimpleMotorFeedforward.hpp"
#include <gtest/gtest.h>
using namespace okapi;
const double EPSILON = 0.0001;

TEST(SimpleMotorFeedforwardGainTest, defaultConstructor) {
    rz::SimpleMotorFeedforward<QLength>::Gains gains;

    ASSERT_NEAR(gains.kS, 0.0, EPSILON);
    ASSERT_NEAR(gains.kV, 0.0, EPSILON);
    ASSERT_NEAR(gains.kA, 0.0, EPSILON);
    ASSERT_NEAR(gains.kD, 0.0, EPSILON);
}

TEST(SimpleMotorFeedforwardGainTest, constructor) {
    rz::SimpleMotorFeedforward<QLength>::Gains gains(0.2, 0.3, 0.4, 0.5);

    ASSERT_NEAR(gains.kS, 0.2, EPSILON);
    ASSERT_NEAR(gains.kV, 0.3, EPSILON);
    ASSERT_NEAR(gains.kA, 0.4, EPSILON);
    ASSERT_NEAR(gains.kD, 0.5, EPSILON);
}

TEST(SimpleMotorFeedforwardGainTest, noDecelConstructor) {
    rz::SimpleMotorFeedforward<QLength>::Gains gains(0.2, 0.3, 0.4);

    ASSERT_NEAR(gains.kS, 0.2, EPSILON);
    ASSERT_NEAR(gains.kV, 0.3, EPSILON);
    ASSERT_NEAR(gains.kA, 0.4, EPSILON);
    ASSERT_NEAR(gains.kD, 0.4, EPSILON);
}

TEST(SimpleMotorFeedforwardGainTest, equality) {
    rz::SimpleMotorFeedforward<QLength>::Gains gains1(0.2, 0.3, 0.4, 0.4);
    rz::SimpleMotorFeedforward<QLength>::Gains gains2(0.2, 0.3, 0.4, 0.4);
    rz::SimpleMotorFeedforward<QLength>::Gains gains3(0.5, 0.1, 0.3, 0.4);

    ASSERT_TRUE(gains1 == gains2);
    ASSERT_FALSE(gains1 == gains3);
}

TEST(SimpleMotorFeedforwardGainTest, inequality) {
    rz::SimpleMotorFeedforward<QLength>::Gains gains1(0.2, 0.3, 0.4, 0.4);
    rz::SimpleMotorFeedforward<QLength>::Gains gains2(0.2, 0.3, 0.4, 0.4);
    rz::SimpleMotorFeedforward<QLength>::Gains gains3(0.5, 0.1, 0.3, 0.4);

    ASSERT_FALSE(gains1 != gains2);
    ASSERT_TRUE(gains1 != gains3);
}

TEST(SimpleMotorFeedforwardTest, constructor) {
    rz::SimpleMotorFeedforward<QAngle>::Gains gains(0.2, 0.3, 0.4, 0.4);
    rz::SimpleMotorFeedforward<QAngle> ffController(gains);

    ASSERT_TRUE(ffController.getGains() == gains);
}

TEST(SimpleMotorFeedforwardTest, setGains) {
    rz::SimpleMotorFeedforward<QAngle>::Gains gains1;
    rz::SimpleMotorFeedforward<QAngle>::Gains gains2(0.2, 0.3, 0.4, 0.4);
    rz::SimpleMotorFeedforward<QAngle> ffController(gains1);
    ffController.setGains(gains2);

    ASSERT_TRUE(ffController.getGains() == gains2);
}

TEST(SimpleMotorFeedforwardTest, calculateAccel) {
    rz::SimpleMotorFeedforward<QAngle>::Gains gains(0.2, 0.3, 0.4, 0.4);
    rz::SimpleMotorFeedforward<QAngle> ffController(gains);

    ASSERT_NEAR(ffController.calculate(3_radps, 5_radps2), 3.1, EPSILON);
}

TEST(SimpleMotorFeedforwardTest, calculateNegativeVel) {
    rz::SimpleMotorFeedforward<QAngle>::Gains gains(0.2, 0.3, 0.4, 0.4);
    rz::SimpleMotorFeedforward<QAngle> ffController(gains);

    ASSERT_NEAR(ffController.calculate(-3_radps, 5_radps2), 0.9, EPSILON);
}

TEST(SimpleMotorFeedforwardTest, calculateDecel) {
    rz::SimpleMotorFeedforward<QLength>::Gains gains(0.2, 0.3, 0.5, 0.4);
    rz::SimpleMotorFeedforward<QLength> ffController(gains);

    ASSERT_NEAR(ffController.calculate(3_mps, -5_mps2), -0.9, EPSILON);
}