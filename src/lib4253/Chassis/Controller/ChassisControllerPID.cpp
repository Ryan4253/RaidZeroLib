#include "lib4253/Chassis/Controller/ChassisControllerPID.hpp"
namespace lib4253{

ChassisControllerPID::ChassisControllerPID(
    std::shared_ptr<Chassis> iChassis,
    std::unique_ptr<PID> iDrivePID,
    std::unique_ptr<PID> iTurnPID,
    std::unique_ptr<PID> iAnglePID,
    std::unique_ptr<Slew> iSlew
):chassis(iChassis), drivePID(std::move(iDrivePID)), turnPID(std::move(iTurnPID)), anglePID(std::move(iAnglePID)), slew(std::move(iSlew)){}

void ChassisControllerPID::moveDistance(const okapi::QLength& dist, Settler settler) const{
    auto time = pros::millis();

    slew->reset();
    drivePID->reset();
    anglePID->reset();
    chassis->resetSensor();

    do{
        okapi::QLength error = dist - chassis->getDistance();
        double power = drivePID->step(error.convert(okapi::inch));
        double adjustment = anglePID->step(chassis->getAngle().convert(okapi::degree));
        chassis->setPower(chassis->scaleSpeed(power, adjustment, slew->step(std::fabs(power + adjustment))));
        pros::delay(10); 
    }while(!settler.isSettled(&time, drivePID->getError()));

    chassis->setPower(0, 0);
}

void ChassisControllerPID::turnAngle(const okapi::QAngle& angle, Settler settler) const{
    okapi::QAngle target = Math::angleWrap180(angle);
    auto time = pros::millis();
    anglePID->reset();
    chassis->resetSensor();
    
    do{
        okapi::QAngle error = target - chassis->getAngle();
        double power = turnPID->step(error.convert(okapi::degree));
        chassis->setPower(chassis->desaturate(power, -power, slew->step(std::abs(power))));
        pros::delay(10); 
    }while(!settler.isSettled(&time, turnPID->getError()));

    chassis->setPower(0, 0);
}
}