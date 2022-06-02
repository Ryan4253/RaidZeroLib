#include "AsyncMotionProfiler.hpp"

namespace lib4253{

AsyncMotionProfiler::AsyncMotionProfiler(std::shared_ptr<ChassisController> iChassis, 
                                         std::unique_ptr<LinearMotionProfile> iMove, 
                                         const std::optional<MotorFFController>& iLeftLinear, 
                                         const std::optional<MotorFFController>& iRightLinear,
                                         const std::optional<MotorFFController>& iLeftTrajectory,
                                         const std::optional<MotorFFController>& iRightTrajectory,
                                         const TimeUtil& iTimeUtil): timeUtil(iTimeUtil)
{
    chassis = std::move(iChassis);
    profiler = std::move(iMove);
    leftLinear = iLeftLinear;
    rightLinear = iRightLinear;
    leftTrajectory = iLeftTrajectory;
    rightTrajectory = iRightTrajectory;
    rate = std::move(timeUtil.getRate());
    timer = std::move(timeUtil.getTimer());

    leftMotor = std::move(std::static_pointer_cast<SkidSteerModel>(chassis->getModel())->getLeftSideMotor());
    rightMotor = std::move(std::static_pointer_cast<SkidSteerModel>(chassis->getModel())->getRightSideMotor());
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
    maxTime = path.size() * 10 * okapi::millisecond + 0.02 * okapi::second;
    timer->placeMark();
    lock.give();

    if(waitUntilSettled){
        this->waitUntilSettled();
    }
}

void AsyncMotionProfiler::setTarget(QAngle iAngle, QAngle iCurrentAngle, bool waitUntilSettled){
    lock.take(5);
    auto target = iAngle - iCurrentAngle;
    setState(MotionProfileState::MOVE);
    profiler->setDistance(2 * target.convert(radian) * chassis->getChassisScales().wheelTrack / 2);
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

void AsyncMotionProfiler::setConstraint(ProfileConstraint iConstraint){
    std::lock_guard<pros::Mutex> guard(lock);
    profiler->setConstraint(iConstraint);
}

void AsyncMotionProfiler::stop(){
    std::lock_guard<pros::Mutex> guard(lock);
    setState(MotionProfileState::IDLE);
    (chassis->getModel())->tank(0, 0);
}

void AsyncMotionProfiler::loop(){
    while(true){
        if(getState() == MotionProfileState::IDLE){
            rate->delayUntil(10);
            continue;
        }

        lock.take(5);
        QTime time = timer->getDtFromMark();
        TrajectoryPoint pt;
        bool useCustomVel = false;
        
        if(time > maxTime){
            lock.give();
            stop();
            rate->delayUntil(10);
            continue;
        }

        if(getState() == MotionProfileState::MOVE){
            pt = profiler->get(time);
            if(leftLinear && rightLinear){
                useCustomVel = true;
            }
        }
        else{
            pt = path[(int)(time.convert(okapi::millisecond) / 10)];
            if(leftTrajectory && rightTrajectory){
                useCustomVel = true;
            }
        }

        QLength leftPos = Math::angleToArcLength(leftMotor->getPosition() * degree, chassis->getChassisScales().wheelDiameter/2) * chassis->getGearsetRatioPair().ratio; // TODO: Check Ratio math
        QLength rightPos = Math::angleToArcLength(rightMotor->getPosition() * degree, chassis->getChassisScales().wheelDiameter/2) * chassis->getGearsetRatioPair().ratio; // TODO: Check Ratio math
        QSpeed leftVel = Math::rpmToVel(leftMotor->getActualVelocity(), chassis->getChassisScales().wheelDiameter/2, chassis->getGearsetRatioPair().ratio); // TODO: Check Ratio math
        QSpeed rightVel = Math::rpmToVel(rightMotor->getActualVelocity(), chassis->getChassisScales().wheelDiameter/2, chassis->getGearsetRatioPair().ratio); // TODO: Check Ratio math

        if(useCustomVel){
            double leftPower = leftTrajectory->step(pt.leftDist, pt.leftVel, pt.leftAccel, leftPos, leftVel);
            double rightPower = rightTrajectory->step(pt.rightDist, pt.rightVel, pt.rightAccel, rightPos, rightVel);
            chassis->getModel()->tank(leftPower, rightPower);
        }
        else{
            double leftVel = Math::velToRPM(pt.leftVel, chassis->getChassisScales().wheelDiameter/2, chassis->getGearsetRatioPair().ratio);
            double rightVel = Math::velToRPM(pt.rightVel, chassis->getChassisScales().wheelDiameter/2, chassis->getGearsetRatioPair().ratio);
            leftMotor->moveVelocity(leftVel);
            rightMotor->moveVelocity(rightVel);
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

AsyncMotionProfiler::Builder::Builder(){
}

AsyncMotionProfiler::Builder& AsyncMotionProfiler::Builder::withOutput(std::shared_ptr<okapi::ChassisController> iChassis){
    chassis = std::move(iChassis);
    return *this;
}

AsyncMotionProfiler::Builder& AsyncMotionProfiler::Builder::withProfiler(std::unique_ptr<LinearMotionProfile> iProfiler){
    profile = std::move(iProfiler);
    return *this;
}

AsyncMotionProfiler::Builder& AsyncMotionProfiler::Builder::withLinearController(MotorFFController iLeft, MotorFFController iRight){
    leftL = iLeft, rightL = iRight;
    return *this;

}

AsyncMotionProfiler::Builder& AsyncMotionProfiler::Builder::withTrajectoryController(MotorFFController iLeft, MotorFFController iRight){
    leftT = iLeft, rightT = iRight;
    return *this;
}

std::shared_ptr<AsyncMotionProfiler> AsyncMotionProfiler::Builder::build(){
    if(chassis == nullptr || profile == nullptr){
        throw std::runtime_error("AsyncMotionProfiler::Builder - missing necessary arguments (chassis and motion profile must be provided)");
    }

    std::shared_ptr<AsyncMotionProfiler> ret(new AsyncMotionProfiler(
                                                     std::move(chassis), std::move(profile), 
                                                     leftL, rightL, leftT, rightT,
                                                     TimeUtilFactory::createDefault()));
    
    ret->startTask();
    return std::move(ret);
}

}
