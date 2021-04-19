#pragma once
#include "main.h"
#include "Odometry.hpp"
#include "lib4253/Controller/PID.hpp"
#include "lib4253/Controller/Slew.hpp"
#include "lib4253/Controller/PurePursuit.hpp"
#include "lib4253/Controller/LinearMotionProfile.hpp"


class Drive{
  public:
    enum State{
      TANK, ARCADE
    };

    Drive(const std::initializer_list<Motor> &l, const std::initializer_list<Motor> &r);
    Drive& withOdometry(CustomOdometry* tracker);
    Drive& withDrivePID(std::tuple<double, double, double> gain, std::tuple<double, double> IGain, std::tuple<double> emaGain);
    Drive& withTurnPID(std::tuple<double, double, double> gain, std::tuple<double, double> IGain, std::tuple<double> emaGain);
    Drive& withPurePursuit(std::tuple<double, double, double> gain, std::tuple<double> turnGain, std::tuple<double, double> kinematics);
    Drive& withSlew(int acc, int dec);
    void initialize();

    State getState();
    void setState(State s);

    void resetIMU();
    double getAngle();

    void moveDistance(double distance, QTime timeLimit);
    void moveTo(Point2D target, double turnScale, QTime timeLimit);
    void turnAngle(double angle, QTime timeLimit);
    void turnToAngle(double angle, QTime timeLimit);

    void moveDistanceLMP(double distance);
    void moveDistanceLMPD(double distance);

    static void driveTask(void *ptr);

  protected:
    MotorGroup left;
    MotorGroup right;

  private:
    CustomOdometry* odom;
    PID drivePID; PID turnPID;
    SlewController driveSlew;
    PurePursuitFollower PPTenshi;
    LinearMotionProfileController* bruhMobile;
    std::shared_ptr<ChassisController> chassis;
    std::shared_ptr<AsyncMotionProfileController> profileController;

    double maxVelocity, maxAcceleration;
    State driveState = TANK;
    int prevAState = 0;

    Point2D scaleSpeed(double linear, double turn, double turnScale);

    void updateState();
    void run();

    void tank();
    void arcade();
};
