#include "OdomController.hpp"

namespace lib4253{

OdomController::OdomController(std::shared_ptr<OdomChassisController> iChassis, 
                                QLength iAngleCorrectionRadius,
                                std::unique_ptr<IterativePosPIDController> iDrivePID, 
                                std::unique_ptr<IterativePosPIDController> iTurnPID, 
                                std::unique_ptr<IterativePosPIDController> iHeadingPID, 
                                std::unique_ptr<SlewController> iSlew
                                )
{
    chassis = std::move(iChassis);
    drivePID = std::move(iDrivePID);
    turnPID = std::move(iTurnPID);
    headingPID = std::move(iHeadingPID);
    angleCorrectionRadius = iAngleCorrectionRadius;
}

void OdomController::moveToPoint(const Point& target, double turnScale){
    drivePID->reset(); turnPID->reset(); driveSlew->reset();

    do {
        Pose currentPos = chassis->getState();
        Point closestPoint = currentPos.closestTo(target);

        auto distToTarget = currentPos.getTranslation().distTo(target);
        auto angleToTarget = currentPos.angleTo(target);
        auto distToClose = currentPos.getTranslation().distTo(closestPoint);
        auto angleToClose = currentPos.angleTo(closestPoint);

        auto driveError = (abs(angleToClose) >= 90 * okapi::degree) ? -distToClose : distToClose;
        auto turnError = (abs(distToTarget) < angleCorrectionRadius)  ? 0 * okapi::degree : Math::angleWrap90(angleToTarget);

        double drivePower = drivePID->step(-driveError.convert(okapi::inch));
        double turnPower = turnPID->step(-turnError.convert(okapi::degree));

        chassis->getModel()->driveVectorVoltage(drivePower, turnPower * turnScale);
        pros::delay(10);
    } while(drivePID->isSettled());
    chassis->getModel()->tank(0, 0);
}

void OdomController::moveToX(QLength targetX){
    drivePID->reset(); headingPID->reset(); driveSlew->reset();
    okapi::QAngle initAngle = chassis->getState().theta;

    do{
        Pose pos = chassis->getState();
        QLength error = (targetX - pos.X());
        QAngle aError = initAngle - pos.Theta(); 

        double power = drivePID->step(-error.convert(okapi::inch));
        double aPower = headingPID->step(-aError.convert(okapi::degree));

        chassis->getModel()->arcade(power, aPower);
        pros::delay(10); 
    }while(!drivePID->isSettled());
    chassis->getModel()->tank(0, 0);
}

void OdomController::moveToY(QLength targetY){
    drivePID->reset(); headingPID->reset(); driveSlew->reset();
    okapi::QAngle initAngle = chassis->getState().theta;

    do{
        Pose pos = chassis->getState();
        QLength error = (targetY - pos.Y());
        QAngle aError = initAngle - pos.Theta(); 

        double power = drivePID->step(-error.convert(okapi::inch));
        double aPower = headingPID->step(-aError.convert(okapi::degree));
        chassis->getModel()->arcade(power, aPower);
        pros::delay(10); 
    }while(!drivePID->isSettled());
    chassis->getModel()->tank(0, 0);
}

void OdomController::turnToAngle(QAngle angle){
    turnPID->reset();

    do{
        Pose pos = chassis->getState();
        QAngle error = angle - pos.Theta();
        double power = turnPID->step(-error.convert(okapi::degree));
        chassis->getModel()->arcade(0, power);

        pros::delay(10); 
    }while(!turnPID->isSettled());
    chassis->getModel()->tank(0, 0);
}

void OdomController::turnToPoint(const Point& target){
    turnPID->reset();

    do{
        Pose pos = chassis->getState();
        QAngle error = pos.angleTo(target);
        double power = turnPID->step(-error.convert(okapi::degree));
        chassis->getModel()->arcade(0, power);
        pros::delay(10); 
    }while(!turnPID->isSettled());
    chassis->getModel()->tank(0, 0);
}    
}


