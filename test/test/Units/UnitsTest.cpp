#include "RaidZeroLib/api/Units/Units.hpp"
#include <gtest/gtest.h>
using namespace okapi::literals;

constexpr double EPSILON = 0.0001;

TEST(UnitsTest, pct) {
    okapi::Number value1 = 50_pct;
    okapi::Number value2 = 50.0_pct;

    EXPECT_NEAR(value1.convert(okapi::number), 0.5, EPSILON);
    EXPECT_NEAR(value2.convert(okapi::number), 0.5, EPSILON);
}

TEST(UnitsTest, field) {
    okapi::QLength length1 = 1_field;
    okapi::QLength length2 = 1_field;

    EXPECT_NEAR(length1.convert(okapi::foot), 12, EPSILON);
    EXPECT_NEAR(length2.convert(okapi::foot), 12, EPSILON);
}

TEST(UnitsTest, radpm) {
    okapi::QCurvature curvature1 = 1_radpm;
    okapi::QCurvature curvature2 = 1.0_radpm;

    EXPECT_NEAR(curvature1.convert(okapi::radpm), (1_rad / 1_m).convert(okapi::radpm), EPSILON);
    EXPECT_NEAR(curvature2.convert(okapi::radpm), (1_rad / 1_m).convert(okapi::radpm), EPSILON);
}

TEST(UnitsTest, ftps) {
    okapi::QSpeed speed1 = 1_ftps;
    okapi::QSpeed speed2 = 1.0_ftps;

    EXPECT_NEAR(speed1.convert(okapi::mps), 0.3048, EPSILON);
    EXPECT_NEAR(speed2.convert(okapi::mps), 0.3048, EPSILON);
}

TEST(UnitsTest, radps) {
    okapi::QAngularSpeed angularVelocity1 = 1_radps;
    okapi::QAngularSpeed angularVelocity2 = 1.0_radps;

    EXPECT_NEAR(angularVelocity1.convert(okapi::radps), (1_rad / 1_s).convert(okapi::radps), EPSILON);
    EXPECT_NEAR(angularVelocity2.convert(okapi::radps), (1_rad / 1_s).convert(okapi::radps), EPSILON);
}

TEST(UnitsTest, ftps2) {
    okapi::QAcceleration acceleration1 = 1_ftps2;
    okapi::QAcceleration acceleration2 = 1.0_ftps2;

    EXPECT_NEAR(acceleration1.convert(okapi::mps2), 0.3048, EPSILON);
    EXPECT_NEAR(acceleration2.convert(okapi::mps2), 0.3048, EPSILON);
}

TEST(UnitsTest, radps2) {
    okapi::QAngularAcceleration angularAcceleration1 = 1_radps2;
    okapi::QAngularAcceleration angularAcceleration2 = 1.0_radps2;

    EXPECT_NEAR(angularAcceleration1.convert(okapi::radps2), (1_rad / 1_s / 1_s).convert(okapi::radps2), EPSILON);
    EXPECT_NEAR(angularAcceleration2.convert(okapi::radps2), (1_rad / 1_s / 1_s).convert(okapi::radps2), EPSILON);
}

TEST(UnitsTest, mps3) {
    okapi::QJerk jerk1 = 1_mps3;
    okapi::QJerk jerk2 = 1.0_mps3;

    EXPECT_NEAR(jerk1.convert(okapi::mps3), (1.0_m / 1.0_s / 1.0_s / 1.0_s).convert(okapi::mps3), EPSILON);
    EXPECT_NEAR(jerk2.convert(okapi::mps3), (1.0_m / 1.0_s / 1.0_s / 1.0_s).convert(okapi::mps3), EPSILON);
}

TEST(UnitsTest, ftps3) {
    okapi::QJerk jerk1 = 1_ftps3;
    okapi::QJerk jerk2 = 1.0_ftps3;

    EXPECT_NEAR(jerk1.convert(okapi::mps3), 0.3048, EPSILON);
    EXPECT_NEAR(jerk2.convert(okapi::mps3), 0.3048, EPSILON);
}

TEST(UnitsTest, radps3) {
    okapi::QAngularJerk angularJerk1 = 1_radps3;
    okapi::QAngularJerk angularJerk2 = 1.0_radps3;
    okapi::QAngularJerk reference = 1.0_rad / 1_s / 1_s / 1_s;

    EXPECT_NEAR(angularJerk1.convert(okapi::radps3), reference.convert(okapi::radps3), EPSILON);
    EXPECT_NEAR(angularJerk2.convert(okapi::radps3), reference.convert(okapi::radps3), EPSILON);
}