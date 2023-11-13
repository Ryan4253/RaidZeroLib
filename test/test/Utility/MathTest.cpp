#include "RaidZeroLib/api/Utility/Math.hpp"
#include <gtest/gtest.h>
using namespace okapi;

constexpr double EPSILON = 0.001;

TEST(MathTest, sign){
    EXPECT_EQ(1, rz::sgn(1));
    EXPECT_EQ(-1, rz::sgn(-1));
    EXPECT_EQ(0, rz::sgn(0));
    EXPECT_EQ(1, rz::sgn(2_m));
    EXPECT_EQ(0, rz::sgn(0_deg));
    EXPECT_EQ(-1, rz::sgn(-2_mps));
}

TEST(MathTest, linearToWheelVelocity) {
    EXPECT_NEAR(360, rz::linearToWheelVelocity(1.55603448_mps, 3.25_in).convert(rpm), EPSILON);
}

TEST(MathTest, wheelToLinearVelocity) {
    EXPECT_NEAR(1.55603448, rz::wheelToLinearVelocity(360_rpm, 3.25_in).convert(mps), EPSILON);
}

TEST(MathTest, constrainAngle360) {
    EXPECT_NEAR(190, rz::constrainAngle360(190_deg).convert(degree), EPSILON);
    EXPECT_NEAR(30, rz::constrainAngle360(390_deg).convert(degree), EPSILON);
    EXPECT_NEAR(270, rz::constrainAngle360(-450_deg).convert(degree), EPSILON);
    
    EXPECT_NEAR(190, rz::constrainAngle360(190), EPSILON);
    EXPECT_NEAR(30, rz::constrainAngle360(390), EPSILON);
    EXPECT_NEAR(270, rz::constrainAngle360(-450), EPSILON);
}

TEST(MathTest, constrainAngle180) {
    EXPECT_NEAR(-170, rz::constrainAngle180(190_deg).convert(degree), EPSILON);
    EXPECT_NEAR(30, rz::constrainAngle180(390_deg).convert(degree), EPSILON);
    EXPECT_NEAR(-90, rz::constrainAngle180(-450_deg).convert(degree), EPSILON);
    
    EXPECT_NEAR(-170, rz::constrainAngle180(190), EPSILON);
    EXPECT_NEAR(30, rz::constrainAngle180(390), EPSILON);
    EXPECT_NEAR(-90, rz::constrainAngle180(-450), EPSILON);   
}

TEST(MathTest, sinc) {
    EXPECT_NEAR(1, rz::sinc(0), EPSILON);
    EXPECT_NEAR(0.841471, rz::sinc(1), EPSILON);
}

TEST(MathTest, quadraticFormulaTwoRoot) {
    const auto result = rz::quadraticFormula(1, 2, -3);
    EXPECT_TRUE(result.has_value());
    EXPECT_NEAR(-3, result.value().first, EPSILON);
    EXPECT_NEAR(1, result.value().second, EPSILON);
}   

TEST(MathTest, quadraticFormulaOneRoot) {
    const auto result = rz::quadraticFormula(1, 2, 1);
    EXPECT_TRUE(result.has_value());
    EXPECT_NEAR(-1, result.value().first, EPSILON);
    EXPECT_NEAR(-1, result.value().second, EPSILON);
}

TEST(MathTest, quadraticFormulaNoRoot) {
    const auto result = rz::quadraticFormula(1, 2, 3);
    EXPECT_FALSE(result.has_value());
}

TEST(MathTest, wheelForwardKinematicsVel) {
    const auto result = rz::wheelForwardKinematics(6_mps, 2_radpm, 3_m);
    EXPECT_NEAR(24, result.first.convert(mps), EPSILON);
    EXPECT_NEAR(-12, result.second.convert(mps), EPSILON);
}

TEST(MathTest, wheelForwardKinematicsAccel) {
    const auto result = rz::wheelForwardKinematics(6_mps2, 2_radpm, 3_m);
    EXPECT_NEAR(24, result.first.convert(mps2), EPSILON);
    EXPECT_NEAR(-12, result.second.convert(mps2), EPSILON);
}