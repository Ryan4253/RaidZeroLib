#include "RaidZeroLib/api/Control/Iterative/IterativeVelBangBangController.hpp"
#include "../test/include/mocks.hpp"
#include "okapi/api/filter/velMath.hpp"
#include <gtest/gtest.h>
using namespace okapi;

constexpr double EPSILON = 0.0001;

TEST(IterativeVelBangBangControllerGainsTest, gainDefaultConstructor) {
    rz::IterativeVelBangBangController::Gains gains;

    EXPECT_NEAR(gains.highPower, 0, EPSILON);
    EXPECT_NEAR(gains.targetPower, 0, EPSILON);
    EXPECT_NEAR(gains.lowPower, 0, EPSILON);
    EXPECT_NEAR(gains.deadband, 0, EPSILON);
}

TEST(IterativeVelBangBangControllerGainsTest, gainConstructor) {
    rz::IterativeVelBangBangController::Gains gains(12000, 9000, 6000, 2);

    EXPECT_NEAR(gains.highPower, 12000, EPSILON);
    EXPECT_NEAR(gains.targetPower, 9000, EPSILON);
    EXPECT_NEAR(gains.lowPower, 6000, EPSILON);
    EXPECT_NEAR(gains.deadband, 2, EPSILON);
}

TEST(IterativeVelBangBangControllerGainsTest, gainEquality) {
    rz::IterativeVelBangBangController::Gains gains(12000, 9000, 6000, 2);
    rz::IterativeVelBangBangController::Gains gains2(12000, 9000, 6000, 2);
    rz::IterativeVelBangBangController::Gains gains3(12100, 9200, 6300, 3);

    EXPECT_TRUE(gains == gains2);
    EXPECT_FALSE(gains == gains3);
}

TEST(IterativeVelBangBangControllerGainsTest, gainInequality) {
    rz::IterativeVelBangBangController::Gains gains(12000, 9000, 6000, 2);
    rz::IterativeVelBangBangController::Gains gains2(12000, 9000, 6000, 2);
    rz::IterativeVelBangBangController::Gains gains3(12100, 9200, 6300, 3);

    EXPECT_FALSE(gains != gains2);
    EXPECT_TRUE(gains != gains3);
}

TEST(IterativeVelBangBangControllerTest, step) {
    auto timeUtil = createConstantTimeUtil(10_ms);
    auto velMath = std::make_unique<VelMath>(360, std::make_unique<okapi::PassthroughFilter>(), 10_ms,
                                             createConstantTimeUtil(10_ms).getTimer());
    
    rz::IterativeVelBangBangController controller(rz::IterativeVelBangBangController::Gains(1, 0.6, 0.2, 100),
                                                  std::move(velMath), createConstantTimeUtil(10_ms));

    controller.setTarget(1000);

    EXPECT_NEAR(controller.step(100), 0.2, EPSILON); // 1666 rpm
    EXPECT_NEAR(controller.step(120), 1, EPSILON); // 333 rpm
    EXPECT_NEAR(controller.step(180), 0.6, EPSILON); // 1000 rpm
}

TEST(IterativeVelBangBangControllerTest, setTarget) {
    auto timeUtil = createConstantTimeUtil(10_ms);
    auto velMath = std::make_unique<VelMath>(360, std::make_unique<PassthroughFilter>(), 10_ms,
                                             createConstantTimeUtil(10_ms).getTimer());

    rz::IterativeVelBangBangController controller(rz::IterativeVelBangBangController::Gains(12000, 9000, 6000, 2),
                                                  std::move(velMath), createConstantTimeUtil(10_ms));

    controller.setTarget(100);
    EXPECT_NEAR(controller.getTarget(), 100, EPSILON);
}

TEST(IterativeVelbangBangControllerTest, setOutputLimits) {
    auto timeUtil = createConstantTimeUtil(10_ms);
    auto velMath = std::make_unique<VelMath>(360, std::make_unique<PassthroughFilter>(), 10_ms,
                                             createConstantTimeUtil(10_ms).getTimer());

    rz::IterativeVelBangBangController controller(rz::IterativeVelBangBangController::Gains(12000, 9000, 6000, 2),
                                                  std::move(velMath), createConstantTimeUtil(10_ms));

    controller.setOutputLimits(0, 100);
    EXPECT_NEAR(controller.getMinOutput(), 0, EPSILON);
    EXPECT_NEAR(controller.getMaxOutput(), 100, EPSILON);
}

TEST(IterativeVelBangBangControllerTest, setSampleTime) {
    auto timeUtil = createConstantTimeUtil(10_ms);
    auto velMath = std::make_unique<VelMath>(360, std::make_unique<PassthroughFilter>(), 10_ms,
                                             createConstantTimeUtil(10_ms).getTimer());

    rz::IterativeVelBangBangController controller(rz::IterativeVelBangBangController::Gains(12000, 9000, 6000, 2),
                                                  std::move(velMath), createConstantTimeUtil(10_ms));

    controller.setSampleTime(20_ms);
    EXPECT_NEAR(controller.getSampleTime().convert(millisecond), 20, EPSILON);
}

TEST(IterativeVelBangBangControllerTest, setGains) {
    auto timeUtil = createConstantTimeUtil(10_ms);
    auto velMath = std::make_unique<VelMath>(360, std::make_unique<PassthroughFilter>(), 10_ms,
                                             createConstantTimeUtil(10_ms).getTimer());

    rz::IterativeVelBangBangController controller(rz::IterativeVelBangBangController::Gains(12000, 9000, 6000, 2),
                                                  std::move(velMath), createConstantTimeUtil(10_ms));

    controller.setGains(rz::IterativeVelBangBangController::Gains(12000, 9000, 6000, 2));
    EXPECT_NEAR(controller.getGains().highPower, 12000, EPSILON);
    EXPECT_NEAR(controller.getGains().targetPower, 9000, EPSILON);
    EXPECT_NEAR(controller.getGains().lowPower, 6000, EPSILON);
    EXPECT_NEAR(controller.getGains().deadband, 2, EPSILON);
}

TEST(IterativeVelBangBangControllerTest, flipDisable) {
    auto timeUtil = createConstantTimeUtil(10_ms);
    auto velMath = std::make_unique<VelMath>(360, std::make_unique<PassthroughFilter>(), 10_ms,
                                             createConstantTimeUtil(10_ms).getTimer());

    rz::IterativeVelBangBangController controller(rz::IterativeVelBangBangController::Gains(12000, 9000, 6000, 2),
                                                  std::move(velMath), createConstantTimeUtil(10_ms));

    controller.flipDisable();
    EXPECT_TRUE(controller.isDisabled());

    controller.flipDisable(false);
    EXPECT_FALSE(controller.isDisabled());
}

TEST(IterativeVelBangBangControllerTest, reset) {
    auto timeUtil = createConstantTimeUtil(10_ms);
    auto velMath = std::make_unique<VelMath>(360, std::make_unique<PassthroughFilter>(), 10_ms,
                                             createConstantTimeUtil(10_ms).getTimer());

    rz::IterativeVelBangBangController controller(rz::IterativeVelBangBangController::Gains(12000, 9000, 6000, 2),
                                                  std::move(velMath), createConstantTimeUtil(10_ms));

    controller.reset();
    EXPECT_NEAR(controller.getTarget(), 0, EPSILON);
    EXPECT_NEAR(controller.getProcessValue(), 0, EPSILON);
    EXPECT_NEAR(controller.getOutput(), 0, EPSILON);
}

TEST(IterativeVelBangBangControllerTest, stepVel) {
    auto timeUtil = createConstantTimeUtil(10_ms);
    auto velMath = std::make_unique<VelMath>(360, std::make_unique<PassthroughFilter>(), 10_ms,
                                             createConstantTimeUtil(10_ms).getTimer());
    
    auto velMathReference = std::make_unique<VelMath>(360, std::make_unique<PassthroughFilter>(), 10_ms,
                                             createConstantTimeUtil(10_ms).getTimer());

    rz::IterativeVelBangBangController controller(rz::IterativeVelBangBangController::Gains(12000, 9000, 6000, 2),
                                                  std::move(velMath), createConstantTimeUtil(10_ms));

    EXPECT_NEAR(controller.stepVel(100).convert(rpm), velMathReference->step(100).convert(rpm), EPSILON);
    EXPECT_NEAR(controller.getVel().convert(rpm), velMathReference->getVelocity().convert(rpm), EPSILON);
    EXPECT_NEAR(controller.getProcessValue(), velMathReference->getVelocity().convert(rpm), EPSILON);
}

TEST(IterativeVelBangBangControllerTest, controllerSet) {
    auto timeUtil = createConstantTimeUtil(10_ms);
    auto velMath = std::make_unique<VelMath>(360, std::make_unique<PassthroughFilter>(), 10_ms,
                                             createConstantTimeUtil(10_ms).getTimer());
    
    auto velMathReference = std::make_unique<VelMath>(360, std::make_unique<PassthroughFilter>(), 10_ms,
                                             createConstantTimeUtil(10_ms).getTimer());

    rz::IterativeVelBangBangController controller(rz::IterativeVelBangBangController::Gains(12000, 9000, 6000, 2),
                                                  std::move(velMath), createConstantTimeUtil(10_ms));

    controller.controllerSet(0.1);
    EXPECT_NEAR(controller.getTarget(), 0.1, EPSILON);

    controller.setControllerSetTargetLimits(12000, -12000);
    controller.controllerSet(0.1);
    EXPECT_NEAR(controller.getTarget(), 1200, EPSILON);
}