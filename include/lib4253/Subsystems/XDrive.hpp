#pragma once
#include "main.h"
#include "Odometry.hpp"
#include "lib4253/Controller/PID.hpp"
#include "lib4253/Controller/Slew.hpp"
#include "lib4253/Splines/Point2D.hpp"
#include "lib4253/Controller/MotorVelocity.hpp"

class XDrive {
    public:
    XDrive(Motor leftFront, Motor leftBack, Motor rightFront, Motor rightBack); 
    XDrive& withOdometry(CustomOdometry* tracker);
    XDrive& withDrivePID(std::tuple<double, double, double> gain);
    XDrive& withConfig(double motorRPM, brakeType brake);
    XDrive& withMotorPID(std::tuple<double, double, double> gain);
    double map(double value, double prevMin, double prevMax, double targetMin, double targetMax);
    double angleWrap(double angle);
    void setDriveVel(std::array<int, 4> vel);
    void setDriveVolt(std::array<int, 4> volt);
    std::vector<double> moveTowards(Pose2D currPose, Pose2D targetPose);
    void moveTo(Pose2D targetPose);
    // std::tuple<double, double, double> getVector()
    // void moveTo(Pose2D pose, );
    // XDrive& withDimensions(double wheelDiameter, double gearRatio);

    private:
    std::array<Motor, 4> base;
    CustomOdometry* odom;
    PID drivePID;
    double motorRPM;
    MotorVelocityController velPID;
}