#include "OdomController.hpp"

namespace lib4253{

OdomController::OdomController(std::shared_ptr<Chassis> iChassis, 
                                std::shared_ptr<Odometry> iOdometry, 
                                const okapi::QLength& iAngleCorrectionRadius,
                                std::unique_ptr<PID> iDrivePID, 
                                std::unique_ptr<PID> iTurnPID, 
                                std::unique_ptr<PID> iAnglePID, 
                                std::unique_ptr<Slew> iSlew
                                )
{
    chassis = iChassis;
    odom = iOdometry;
    drivePID = std::move(iDrivePID);
    turnPID = std::move(iTurnPID);
    anglePID = std::move(iAnglePID);
    angleCorrectionRadius = iAngleCorrectionRadius;
}

void OdomController::moveToPoint(const Point2D& target, const double& turnScale, Settler settler){
    drivePID->initialize();
    turnPID->initialize();
    driveSlew->reset();
    auto time = pros::millis();
    okapi::QLength distToTarget;

    do {
        Pose2D currentPos = odom->getPos();
        Point2D closestPoint = currentPos.closestTo(target);
        pros::lcd::print(0, "CURRENT X: %lf", currentPos.getX().convert(okapi::inch));
        pros::lcd::print(1, "CURRENT Y: %lf", currentPos.getY().convert(okapi::inch));
        pros::lcd::print(2, "CURRENT A: %lf", currentPos.getTheta().convert(okapi::degree));

        distToTarget = currentPos.translation.distanceTo(target);
        okapi::QAngle angleToTarget = currentPos.angleTo(target);
        okapi::QLength distToClose = currentPos.translation.distanceTo(closestPoint);
        okapi::QAngle angleToClose = currentPos.angleTo(closestPoint);

        okapi::QLength driveError = (abs(angleToClose) >= 90 * okapi::degree) ? -distToClose : distToClose;
        okapi::QAngle turnError = (abs(distToTarget) < angleCorrectionRadius)  ? 0 * okapi::degree : Math::angleWrap90(angleToTarget);

        pros::lcd::print(4, "Drive Error: %lf", driveError);
        pros::lcd::print(5, "Turn Error: %lf", turnError);

        double drivePower = drivePID->step(-driveError.convert(okapi::inch));
        double turnPower = turnPID->step(-turnError.convert(okapi::degree));

        pros::lcd::print(6, "Drive Power: %lf", drivePower);
        pros::lcd::print(7, "Turn Power: %lf", turnPower);
        std::pair<double, double> driveSpeed = chassis->scaleSpeed(drivePower, turnPower * turnScale, std::fabs(driveSlew->step(drivePower + turnPower * turnScale)));

        chassis->setPower(driveSpeed.first, driveSpeed.second);
        pros::delay(10);
    } while(!settler.isSettled(&time, distToTarget.convert(inch)));
    chassis->setPower(0, 0);
}

void OdomController::moveToX(const QLength& targetX, Settler settler){
    drivePID->initialize();
    anglePID->initialize();
    driveSlew->reset();
    auto time = pros::millis();
    okapi::QAngle initAngle = odom->getAngle();
    okapi::QLength error;

    do{
        error = (targetX - odom->getX());
        okapi::QAngle aError = initAngle - odom->getAngle(); 

        double power = drivePID->step(error.convert(okapi::inch));
        double aPower = anglePID->step(aError.convert(okapi::degree));
        std::pair<double, double> finalPower = chassis->scaleSpeed(power, aPower, driveSlew->step(std::fabs(power + aPower)));
        chassis->setPower(finalPower.first, finalPower.second);
        pros::delay(10); 
    }while(!settler.isSettled(&time, error.convert(okapi::inch)));
    chassis->setPower(0, 0);
}

void OdomController::moveToY(const QLength& targetY, Settler settler){
    drivePID->initialize();
    anglePID->initialize();
    driveSlew->reset();
    auto time = pros::millis();
    okapi::QAngle initAngle = odom->getAngle();
    okapi::QLength error; 

    do{
        error = (targetY - odom->getY());
        okapi::QAngle aError = initAngle - odom->getAngle(); 

        double power = drivePID->step(error.convert(okapi::inch));
        double aPower = anglePID->step(aError.convert(okapi::degree));
        std::pair<double, double> finalPower = chassis->scaleSpeed(power, aPower, driveSlew->step(std::fabs(power + aPower)));
        chassis->setPower(finalPower.first, finalPower.second);
        pros::delay(10); 
    }while(!settler.isSettled(&time, error.convert(okapi::inch)));
    chassis->setPower(0, 0);
}

void OdomController::turnToAngle(const okapi::QAngle& angle, Settler settler){
    turnPID->initialize();
    auto time = pros::millis();
    okapi::QAngle error;

    do{
        error = angle - odom->getAngle();
        double power = turnPID->step(error.convert(okapi::degree));
        chassis->setPower(power, -power);
        pros::delay(10); 
    }while(!settler.isSettled(&time, error.convert(okapi::degree)));
    chassis->setPower(0, 0);
}

void OdomController::turnToPoint(const Point2D& target, Settler settler){
    turnPID->initialize();
    auto time = pros::millis();
    okapi::QAngle error;

    do{
        error = (odom->getPos()).angleTo(target);
        double power = turnPID->step(error.convert(okapi::degree));
        chassis->setPower(power, -power);
        pros::delay(10); 
    }while(!settler.isSettled(&time, error.convert(okapi::degree)));
    chassis->setPower(0, 0);
}    
}


