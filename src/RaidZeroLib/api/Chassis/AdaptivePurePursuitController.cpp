#include "RaidZeroLib/api/Chassis/AdaptivePurePursuitController.hpp"

namespace rz {

AdaptivePurePursuitController::AdaptivePurePursuitController(
    const std::shared_ptr<OdomChassisController>& chassis, const Gains& gains,
    std::unique_ptr<FeedforwardController<QLength>> leftController,
    std::unique_ptr<FeedforwardController<QLength>> rightController, const TimeUtil& timeUtil)
    : chassis(chassis), gains(gains), timeUtil(timeUtil), task([](void*) {}, 0, "AdaptivePurePursuitController") {

    leftMotor = std::static_pointer_cast<SkidSteerModel>(chassis->getModel())->getLeftSideMotor();
    rightMotor = std::static_pointer_cast<SkidSteerModel>(chassis->getModel())->getLeftSideMotor();

    this->leftController = std::move(leftController);
    this->rightController = std::move(rightController);
}

void AdaptivePurePursuitController::followPath(DiscretePath& path, QTime timeout, bool isReversed) {
    auto closestPointIter = path.begin();
    double lookaheadPointT = 0;
    Point lookAheadPoint = path.front();
    const ChassisScales scales = chassis->getChassisScales();
    timeUtil.getTimer()->placeMark();

    do {
        const Pose pos = chassis->getState();
        closestPointIter = closestPoint(closestPointIter, path.end(), pos.getTranslation());
        lookAheadPoint =
            getLookaheadPoint(path, lookaheadPointT, pos.getTranslation(), gains.lookAhead).value_or(lookAheadPoint);

        QCurvature curvature = curvatureToReachPoint(pos, lookAheadPoint);
        QSpeed velocity;
        QAcceleration acceleration;

        if (isReversed) {
            acceleration *= -1;
            velocity *= -1;
            curvature *= -1;
        }

        const auto [leftVelocity, rightVelocity] = wheelForwardKinematics(velocity, curvature, scales.wheelTrack);
        const auto [leftAccel, rightAccel] = wheelForwardKinematics(acceleration, curvature, scales.wheelTrack);

        if (leftController && rightController) {
            const double leftVoltage = leftController->calculate(leftVelocity, leftAccel);
            const double rightVoltage = leftController->calculate(rightVelocity, rightAccel);
            chassis->getModel()->tank(leftVoltage, rightVoltage);
        } else {
            const double leftRPM = linearToWheelVelocity(leftVelocity, scales.wheelTrack).convert(rpm) *
                                   chassis->getGearsetRatioPair().ratio;
            const double rightRPM = linearToWheelVelocity(leftVelocity, scales.wheelTrack).convert(rpm) *
                                    chassis->getGearsetRatioPair().ratio;
            leftMotor->moveVelocity(leftRPM);
            rightMotor->moveVelocity(rightRPM);
        }
    } while (*closestPointIter != path.back() && timeUtil.getTimer()->getDtFromMark() < timeout &&
             !CrossPlatformThread::notifyTake(true, 10));
}

void AdaptivePurePursuitController::stop() {
    task.notify();
    timeUtil.getRate()->delayUntil(10);
    chassis->stop();
}

void AdaptivePurePursuitController::waitUntilSettled() {
    while (true) {
        timeUtil.getRate()->delayUntil(10);
    }
}

std::optional<Translation> AdaptivePurePursuitController::getLookaheadPoint(DiscretePath& path, double& minIndex,
                                                                            const Point& point, QLength radius) {
    for (int i = (int)minIndex; i < path.size(); i++) {
        const Point& start = path[i];
        const Point& end = path[i + 1];
        const auto t = circleLineIntersection(start, end, point, radius);

        if (t && t.value() >= minIndex) {
            minIndex = t.value();
            return start + (end - start) * t.value();
        }
    }

    return std::nullopt;
}

} // namespace rz
