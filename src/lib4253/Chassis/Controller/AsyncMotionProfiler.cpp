#include "AsyncMotionProfiler.hpp"

namespace lib4253{

AsyncMotionProfiler::AsyncMotionProfiler(std::shared_ptr<okapi::ChassisController> iChassis, 
                                         std::unique_ptr<LinearMotionProfile> iMove, 
                                         std::unique_ptr<MotorFFController> iLeftLinear, 
                                         std::unique_ptr<MotorFFController> iRightLinear,
                                         std::unique_ptr<MotorFFController> iLeftTrajectory,
                                         std::unique_ptr<MotorFFController> iRightTrajectory,
                                         const okapi::TimeUtil& iTimeUtil): timeUtil(iTimeUtil)
{
    chassis = std::move(iChassis);
    profiler = std::move(iMove);
    leftLinear =  std::move(iLeftLinear);
    rightLinear = std::move(iRightLinear);
    leftTrajectory = std::move(iLeftTrajectory);
    rightTrajectory = std::move(iRightTrajectory);
    rate = std::move(timeUtil.getRate());
    timer = std::move(timeUtil.getTimer());
    linearCustom = true;
    trajectoryCustom = true;

    leftMotor = std::move(std::static_pointer_cast<okapi::SkidSteerModel>(chassis->getModel())->getLeftSideMotor());
    rightMotor = std::move(std::static_pointer_cast<okapi::SkidSteerModel>(chassis->getModel())->getRightSideMotor());
}

AsyncMotionProfiler::AsyncMotionProfiler(std::shared_ptr<okapi::ChassisController> iChassis, 
    std::unique_ptr<LinearMotionProfile> iMove, 
    const okapi::TimeUtil& iTimeUtil): timeUtil(iTimeUtil){
    chassis = std::move(iChassis);
    profiler = std::move(iMove);
    rate = std::move(timeUtil.getRate());
    timer = std::move(timeUtil.getTimer());

    leftMotor = std::move(std::static_pointer_cast<okapi::SkidSteerModel>(chassis->getModel())->getLeftSideMotor());
    rightMotor = std::move(std::static_pointer_cast<okapi::SkidSteerModel>(chassis->getModel())->getRightSideMotor());
}

AsyncMotionProfiler::AsyncMotionProfiler(std::shared_ptr<okapi::ChassisController> iChassis, 
                std::unique_ptr<LinearMotionProfile> iMove, 
                std::unique_ptr<MotorFFController> iLeft,
                std::unique_ptr<MotorFFController> iRight,
                bool velFlag,
                const okapi::TimeUtil& iTimeUtil): timeUtil(iTimeUtil){
    chassis = std::move(iChassis);
    profiler = std::move(iMove);
    if(velFlag){
        leftLinear = std::move(iLeft);
        rightLinear = std::move(iRight);
        linearCustom = true;
    }
    else{
        leftTrajectory = std::move(iLeft);
        rightTrajectory = std::move(iRight);
        trajectoryCustom = true;
    }
    rate = std::move(timeUtil.getRate());
    timer = std::move(timeUtil.getTimer());

    leftMotor = std::move(std::static_pointer_cast<okapi::SkidSteerModel>(chassis->getModel())->getLeftSideMotor());
    rightMotor = std::move(std::static_pointer_cast<okapi::SkidSteerModel>(chassis->getModel())->getRightSideMotor());
}

void AsyncMotionProfiler::setTarget(okapi::QLength distance, bool waitUntilSettled){
    lock.take(5);
    setState(MotionProfileState::MOVE);
    profiler->setDistance(distance);
    leftMotor->tarePosition();
    rightMotor->tarePosition();
    chassis->getModel()->tank(0, 0);
    maxTime = profiler->getTotalTime() + 0.02 * okapi::second;
    timer->placeMark();
    lock.give();

    if(waitUntilSettled){
        this->waitUntilSettled();
    }
}

void AsyncMotionProfiler::setTarget(const Trajectory& iPath, bool waitUntilSettled){
    lock.take(5);
    setState(MotionProfileState::FOLLOW);
    leftMotor->tarePosition();
    rightMotor->tarePosition();
    chassis->getModel()->tank(0, 0);
    path = iPath;
    maxTime = path.size() * 10 * okapi::millisecond + 0.02 * okapi::millisecond;
    timer->placeMark();
    lock.give();

    if(waitUntilSettled){
        this->waitUntilSettled();
    }
}

void AsyncMotionProfiler::stop(){
    lock.take(5);
    setState(MotionProfileState::IDLE);
    (chassis->getModel())->tank(0, 0);
    lock.give();
}

void AsyncMotionProfiler::loop(){
    TrajectoryPoint pt;
    double leftPower, rightPower;

    while(true){
        lock.take(5);
        okapi::QTime time = timer->getDtFromMark();

        QLength leftPos = Math::angleToArcLength(leftMotor->getPosition() * degree, chassis->getChassisScales().wheelDiameter/2) * chassis->getGearsetRatioPair().ratio; // TODO: Check Ratio math
        QLength rightPos = Math::angleToArcLength(rightMotor->getPosition() * degree, chassis->getChassisScales().wheelDiameter/2) * chassis->getGearsetRatioPair().ratio; // TODO: Check Ratio math
        QSpeed leftVel = Math::rpmToVel(leftMotor->getActualVelocity(), chassis->getChassisScales().wheelDiameter/2, chassis->getGearsetRatioPair().ratio); // TODO: Check Ratio math
        QSpeed rightVel = Math::rpmToVel(rightMotor->getActualVelocity(), chassis->getChassisScales().wheelDiameter/2, chassis->getGearsetRatioPair().ratio); // TODO: Check Ratio math

        if(getState() == MotionProfileState::IDLE){

        }
        else if(time > maxTime){
            setState(MotionProfileState::IDLE);
            chassis->getModel()->tank(0, 0);
        }
        else if(getState() == MotionProfileState::MOVE){
            pt = profiler->get(time);
            if(linearCustom){
                leftPower = leftLinear->step(pt.leftDist.convert(foot), pt.leftVel.convert(ftps), pt.leftAccel.convert(ftps2), leftPos.convert(foot), leftVel.convert(ftps));
                rightPower = rightLinear->step(pt.rightDist.convert(foot), pt.rightVel.convert(ftps), pt.leftAccel.convert(ftps2), rightPos.convert(foot), rightVel.convert(ftps));
                chassis->getModel()->tank(leftPower, rightPower);
            }
            else{
                double vel = Math::velToRPM(pt.leftVel, chassis->getChassisScales().wheelDiameter/2, chassis->getGearsetRatioPair().ratio);
                leftMotor->moveVelocity(vel);
                rightMotor->moveVelocity(vel);
            }
        }
        else if(getState() == MotionProfileState::FOLLOW){
            pt = path[(int)(time.convert(okapi::millisecond) / 10)];
            if(trajectoryCustom){
                leftPower = leftTrajectory->step(pt.leftDist.convert(foot), pt.leftVel.convert(ftps), pt.leftAccel.convert(ftps2), leftPos.convert(foot), leftVel.convert(ftps));
                rightPower = rightTrajectory->step(pt.rightDist.convert(foot), pt.leftVel.convert(ftps), pt.rightAccel.convert(ftps2), rightPos.convert(foot), rightVel.convert(ftps));
                chassis->getModel()->tank(leftPower, rightPower);
            }
            else{
                double leftVel = Math::velToRPM(pt.leftVel, chassis->getChassisScales().wheelDiameter/2, chassis->getGearsetRatioPair().ratio);
                double rightVel = Math::velToRPM(pt.rightVel, chassis->getChassisScales().wheelDiameter/2, chassis->getGearsetRatioPair().ratio);
                leftMotor->moveVelocity(leftVel);
                rightMotor->moveVelocity(rightVel);
            }
        }

        lock.give();
        rate->delayUntil(10);
    }
}

void AsyncMotionProfiler::waitUntilSettled(){
    while(getState() != MotionProfileState::IDLE){
        pros::delay(10);
    }
}

AsyncMotionProfilerBuilder::AsyncMotionProfilerBuilder(){
    linearInit = false;
    trajInit = false;
    driveInit = false;
    profileInit = false;
}

AsyncMotionProfilerBuilder& AsyncMotionProfilerBuilder::withOutput(std::shared_ptr<okapi::ChassisController> iChassis){
    chassis = std::move(iChassis);
    driveInit = true;
    return *this;
}

AsyncMotionProfilerBuilder& AsyncMotionProfilerBuilder::withProfiler(std::unique_ptr<LinearMotionProfile> iProfiler){
    profile = std::move(iProfiler);
    profileInit = true;
    return *this;
}

AsyncMotionProfilerBuilder& AsyncMotionProfilerBuilder::withLinearController(MotorFFController iLeft, MotorFFController iRight){
    leftL = iLeft, rightL = iRight;
    linearInit = true;
    return *this;

}

AsyncMotionProfilerBuilder& AsyncMotionProfilerBuilder::withTrajectoryController(MotorFFController iLeft, MotorFFController iRight){
    leftT = iLeft, rightT = iRight;
    trajInit = true;
    return *this;
}

std::shared_ptr<AsyncMotionProfiler> AsyncMotionProfilerBuilder::build(){
    if(driveInit && profileInit && linearInit && trajInit){
        std::shared_ptr<AsyncMotionProfiler> ret(new AsyncMotionProfiler(std::move(chassis), 
                                            std::move(profile), 
                                            std::make_unique<MotorFFController>(leftL), 
                                            std::make_unique<MotorFFController>(rightL),
                                            std::make_unique<MotorFFController>(leftT), 
                                            std::make_unique<MotorFFController>(rightT),
                                            okapi::TimeUtilFactory::createDefault()));

        ret->startTask();
        return std::move(ret);
    }
    else if(driveInit && profileInit && linearInit){
        std::shared_ptr<AsyncMotionProfiler> ret(new AsyncMotionProfiler(std::move(chassis), 
                                    std::move(profile), 
                                    std::make_unique<MotorFFController>(leftL), 
                                    std::make_unique<MotorFFController>(rightL),
                                    true,
                                    okapi::TimeUtilFactory::createDefault()));

        ret->startTask();
        return std::move(ret);
    }
    else if(driveInit && profileInit && trajInit){
        std::shared_ptr<AsyncMotionProfiler> ret(new AsyncMotionProfiler(std::move(chassis), 
                                    std::move(profile), 
                                    std::make_unique<MotorFFController>(leftT), 
                                    std::make_unique<MotorFFController>(rightT),
                                    false,
                                    okapi::TimeUtilFactory::createDefault()));

        ret->startTask();
        return std::move(ret);
    }
    else if(driveInit && profileInit){
        std::shared_ptr<AsyncMotionProfiler> ret(new AsyncMotionProfiler(std::move(chassis), 
                                    std::move(profile), 
                                    okapi::TimeUtilFactory::createDefault()));

        ret->startTask();
        return std::move(ret);
    }
    else{
        throw std::runtime_error("AsyncMotionProfilerBuilder: Not all parameters supplied, failed to build (you need at least a chassis and a profiler");
    }
}

}
