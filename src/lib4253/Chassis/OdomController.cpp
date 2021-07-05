#include "OdomController.hpp"

namespace lib4253{

OdomController::OdomController(std::shared_ptr<Chassis> iChassis, 
                                std::shared_ptr<Odometry> iOdometry, 
                                const okapi::QLength& iAngleCorrectionRadius,
                                std::unique_ptr<PID> iDrivePID, 
                                std::unique_ptr<PID> iTurnPID, 
                                std::unique_ptr<PID> iAnglePID, 
                                std::unique_ptr<SlewController> iSlew
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
    double distToTarget;

    do {
        Pose2D currentPos = odom->getPos();
        Point2D closestPoint = currentPos.closest(target);
        pros::lcd::print(0, "CURRENT X: %lf", (double)currentPos.x);
        pros::lcd::print(1, "CURRENT Y: %lf", (double)currentPos.y);
        pros::lcd::print(2, "CURRENT A: %lf", (double)currentPos.theta);

        distToTarget = currentPos.distanceTo(target);
        double angleToTarget = currentPos.angleTo(target);
        double distToClose = currentPos.distanceTo(closestPoint);
        double angleToClose = currentPos.angleTo(closestPoint);

        double driveError = (std::fabs(angleToClose) >= 90) ? -distToClose : distToClose;
        double turnError = (std::fabs(distToTarget) < angleCorrectionRadius.convert(inch))  ? 0 : Math::wrapAngle90(angleToTarget);

        pros::lcd::print(4, "Drive Error: %lf", driveError);
        pros::lcd::print(5, "Turn Error: %lf", turnError);

        double drivePower = drivePID->update(-driveError);
        double turnPower = turnPID->update(-turnError);

        pros::lcd::print(6, "Drive Power: %lf", drivePower);
        pros::lcd::print(7, "Turn Power: %lf", turnPower);
        std::pair<double, double> driveSpeed = chassis->scaleSpeed(drivePower, turnPower * turnScale, std::fabs(driveSlew->step(drivePower + turnPower * turnScale)));

        chassis->setPower(driveSpeed.first, driveSpeed.second);
        pros::delay(10);
    } while(!settler.isSettled(&time, distToTarget));
    chassis->setPower(0, 0);
}

void OdomController::moveToX(const QLength& targetX, Settler settler){
    drivePID->initialize();
    anglePID->initialize();
    driveSlew->reset();
    auto time = pros::millis();
    double error, initAngle = odom->getAngleDeg();

    do{
        error = (targetX - odom->getQX()).convert(inch);
        double aError = initAngle - odom->getAngleDeg(); 

        double power = drivePID->update(error);
        double aPower = anglePID->update(aError);
        std::pair<double, double> finalPower = chassis->scaleSpeed(power, aPower, driveSlew->step(std::fabs(power + aPower)));
        chassis->setPower(finalPower.first, finalPower.second);
        pros::delay(10); 
    }while(!settler.isSettled(&time, error));
    chassis->setPower(0, 0);
}

void OdomController::moveToY(const QLength& targetY, Settler settler){
    drivePID->initialize();
    anglePID->initialize();
    driveSlew->reset();
    auto time = pros::millis();
    double error, initAngle = odom->getAngleDeg();

    do{
        error = (targetY - odom->getQY()).convert(inch);
        double aError = initAngle - odom->getAngleDeg(); 

        double power = drivePID->update(error);
        double aPower = anglePID->update(aError);
        std::pair<double, double> finalPower = chassis->scaleSpeed(power, aPower, driveSlew->step(std::fabs(power + aPower)));
        chassis->setPower(finalPower.first, finalPower.second);
        pros::delay(10); 
    }while(!settler.isSettled(&time, error));
    chassis->setPower(0, 0);
}

void OdomController::turnToAngle(const double& angle, Settler settler){
    turnPID->initialize();
    auto time = pros::millis();
    double error;

    do{
        error = angle - odom->getAngleDeg();
        double power = turnPID->update(error);
        chassis->setPower(power, -power);
        pros::delay(10); 
    }while(!settler.isSettled(&time, error));
    chassis->setPower(0, 0);
}

void OdomController::turnToPoint(const Point2D& target, Settler settler){
    turnPID->initialize();
    auto time = pros::millis();
    double error;

    do{
        error = (odom->getPos()).angleTo(target);
        double power = turnPID->update(error);
        chassis->setPower(power, -power);
        pros::delay(10); 
    }while(!settler.isSettled(&time, error));
    chassis->setPower(0, 0);
}    
}