#pragma once
#include "main.h"
#include "Odometry.hpp"
#include "lib4253/Controller/PID.hpp"
#include "lib4253/Controller/Slew.hpp"
#include "lib4253/Splines/Point2D.hpp"
#include "lib4253/Controller/MotorVelocity.hpp"
namespace lib4253{
class XDrive {
    public:
    XDrive(okapi::Motor leftFront, okapi::Motor leftBack, okapi::Motor rightFront, okapi::Motor rightBack); 
    XDrive& withOdometry(CustomOdometry* tracker);
    XDrive& withDrivePID(std::tuple<double, double, double> gain);
    XDrive& withConfig(double motorRPM, brakeType brake);
    XDrive& withMotorPID(std::tuple<double, double, double> gain);
    double map(double value, double prevMin, double prevMax, double targetMin, double targetMax);
    double angleWrap(double angle);
    void setDriveVel(std::array<double, 4> vel);
    void setDriveVolt(std::array<double, 4> volt);
    std::pair<std::array<double, 4>, std::array<double, 2>> moveTowards(Pose2D currPose, Pose2D targetPose, double speed);
    void moveTo(Pose2D targetPose);
    // std::tuple<double, double, double> getVector()
    // void moveTo(Pose2D pose, );
    // XDrive& withDimensions(double wheelDiameter, double gearRatio);

    private:
    okapi::Motor base[4];
    CustomOdometry* odom;
    PID drivePID;
    double motorRPM;
    MotorVelocityController velPID;
};
}