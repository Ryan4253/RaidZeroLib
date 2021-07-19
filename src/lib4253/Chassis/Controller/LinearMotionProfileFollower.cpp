#include "lib4253/Chassis/Controller/LinearMotionProfileFollower.hpp"
namespace lib4253{

LinearMotionProfileFollower::LinearMotionProfileFollower(const std::shared_ptr<Chassis>& iChassis, 
                                const std::shared_ptr<LinearMotionProfile<okapi::QLength>>& iDriveProfiler, 
                                const std::shared_ptr<LinearMotionProfile<okapi::QAngle>>& iTurnProfiler)
:chassis(std::move(iChassis)), driveProfiler(std::move(iDriveProfiler)), turnProfiler(std::move(iTurnProfiler))
{}

void LinearMotionProfileFollower::moveDistance(const okapi::QLength& dist){
    driveProfiler->setDistance(dist);
    std::unique_ptr<okapi::AbstractRate> rate = timer.getRate();
    std::unique_ptr<okapi::AbstractTimer> time = timer.getTimer();
    time->placeMark();
    while(!driveProfiler->isSettled()){
        std::pair<okapi::QSpeed, okapi::QAcceleration> kinematics = driveProfiler->calculate(time->getDtFromMark());
        chassis->setVelocity(kinematics, kinematics);
        rate->delayUntil(10 * okapi::millisecond);
    }
    chassis->setPower(0, 0);
}

void LinearMotionProfileFollower::turnAngle(const okapi::QAngle& dist){
    turnProfiler->setDistance(dist);
    std::unique_ptr<okapi::AbstractRate> rate = timer.getRate();
    std::unique_ptr<okapi::AbstractTimer> time = timer.getTimer();
    time->placeMark();
    while(!turnProfiler->isSettled()){
        std::pair<okapi::QAngularSpeed, okapi::QAngularAcceleration> kinematics = turnProfiler->calculate(time->getDtFromMark());
        std::pair<okapi::QSpeed, okapi::QSpeed> velocity = chassis->inverseKinematics(0 * okapi::mps, kinematics.first);
        std::pair<okapi::QAcceleration, okapi::QAcceleration> acceleration = chassis->inverseKinematics(0 * okapi::mps2, kinematics.second);
        chassis->setVelocity({velocity.first, acceleration.first}, {velocity.second, acceleration.second});
        rate->delayUntil(10 * okapi::millisecond);
    }
    chassis->setPower(0, 0);
}
}