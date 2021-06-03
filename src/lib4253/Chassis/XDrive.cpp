#include "lib4253/Subsystems/XDrive.hpp"
#include "Robot.hpp"
namespace lib4253{

XDrive::XDrive(okapi::Motor leftFront, okapi::Motor leftBack, okapi::Motor rightFront, okapi::Motor rightBack):

{
    base = {leftFront, leftBack, rightFront, rightBack}
    base[0] = leftFront; base[1] = leftBack;
    base[2] = rightFront; base[3] = rightBack;
}

XDrive& XDrive::withOdometry(CustomOdometry* tracker){
    odom = tracker;
    return *this;
}

XDrive& XDrive::withDrivePID(std::tuple<double, double, double> gain) {
    drivePID.setGain(std::get<0>(gain), std::get<1>(gain), std::get<2>(gain));
}

XDrive& XDrive::withConfig(double motorRPM, brakeType brake){
    this->motorRPM = motorRPM;
    for(okapi::Motor i : this->base) {
        switch(brake){
            case COAST: 
            i.setBrakeMode(AbstractMotor::brakeMode::coast);
            break;

            case HOLD: 
            i.setBrakeMode(AbstractMotor::brakeMode::hold);
            break;

            case BRAKE: 
            i.setBrakeMode(AbstractMotor::brakeMode::brake);
            break;
        }
    }
}

XDrive& XDrive::withMotorPID(std::tuple<double, double, double> gain) {
    velPID.setGain(std::get<0>(gain), std::get<1>(gain), std::get<2>(gain));
}

double XDrive::map(double value, double prevMin, double prevMax, double targetMin, double targetMax) {
    return targetMin + (targetMax - targetMin) * ((value - prevMin) / (prevMax - prevMin));
}

double XDrive::angleWrap(double angle) {
    while (angle < -PI) {
        angle += 2*PI;
    }
    while (angle > PI) {
        angle -= 2*PI;
    }
    return angle;
}

void XDrive::setDriveVel(std::array<double, 4> vel) {
    for (int i : vel) {
        double newVel = map(vel[i], -1.0, 1.0, -motorRPM, motorRPM);
        base[i].moveVelocity((int)newVel);
    }
}

void XDrive::setDriveVolt(std::array<double, 4> volt){
    for (int i : volt) {
        double newVolt = map(volt[i], -1.0, 1.0, -12000, 12000);
        base[i].moveVoltage((int)newVolt);
    }
}

std::pair<std::array<double, 4>, std::array<double 2> > moveTowards(Pose2D currPose, Pose2D targetPose, double speed) {
    // finds distance from current position to target position
    double distanceToTarget = std::hypot(targetPose.x - currPose.x, targetPose.y - currPose.y);
    // finds angle form current position to target position
    double angleToTarget = std::atan2(targetPose.y - currPose.y, targetPose.x - currPose.x) + 0.5*PI;
    // finds angle from relative angle of current postion to target angle
    double relativeAngleToTarget = angleWrap(angleToTarget - currPose.angle);
    // finds the change in heading
    double deltaHeading = targetPose.angle - currPose.angle;

    // calculates x & y vectors
    double relativeXToTarget = std::cos(relativeAngleToTarget);
    double relativeYToTarget = std::sin(relativeAngleToTarget);
    // scales heading down from [-2PI, 2PI] [-1, 1]
    double scaledHeading = deltaHeading / (2*PI);
    // converts everything to motor velocities
    std::array<double, 4> power = {relativeYToTarget + relativeXToTarget + scaledHeading, 
                                   relativeYToTarget - relativeXToTarget + scaledHeading,
                                   relativeYToTarget - relativeXToTarget - scaledHeading, 
                                   relativeYToTarget + relativeXToTarget - scaledHeading};
    std::array<double, 2> error = {distanceToTarget, deltaHeading};

    double max = std::max(std::max(std::fabs(power[0]), std::fabs(power[1])), std::max(std::fabs(power[2]), std::fabs(power[3])));
    if(max > 1) {
        for(int i : power) {
            power[i] /= max;
        }
    } else {
        double diff = 1 - max;
        for (int i : power) {
            if (std::signbit(power[i])) {
                power[i] += diff;
            } else {
                power[i] -= diff;
            }
        }
    }
    return std::make_pair(power, error);
}

void XDrive::moveTo(Pose2D targetPose, std::pair<double, double> margins) {
    do {
        setDriveVel(moveTowards(odom->getPos(), targetPose).first);
		pros::delay(10);
	} while (std::fabs(moveTowards(odom->getPos(), targetPose).second[0]) > 0 && 
             std::fabs(moveTowards(odom->getPos(), targetPose).second[1]) > 0);
}
}