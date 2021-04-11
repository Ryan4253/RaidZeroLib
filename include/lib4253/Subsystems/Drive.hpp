#pragma once
#include "main.h"

class Drive{
  public:
    enum State{
      BUSY, IDLE, TANK, ARCADE
    };

    Drive(const std::initializer_list<Motor> &l, const std::initializer_list<Motor> &r);
    Drive& withDrivePID(std::tuple<double, double, double> gain, std::tuple<double, double> IGain, std::tuple<double> emaGain);
    Drive& withTurnPID(std::tuple<double, double, double> gain, std::tuple<double, double> IGain, std::tuple<double> emaGain);
    Drive& withSlew(int acc, int dec);
    Drive& withPurePursuit(std::tuple<double, double, double> gain, std::tuple<double> turnGain, std::tuple<double, double> kinematics);

    State getState();
    void setState(State s);

    void resetEncoders();
    void resetIMU();
    double getDistance();
    double getAngle();

    void moveDistance(double distance, QTime timeLimit);
    void moveTo(Vector target, double turnScale, QTime timeLimit);
    void turnAngle(double angle, QTime timeLimit);
    void turnToAngle(double angle, QTime timeLimit);

    static void driveTask(void *ptr);

  protected:
    MotorGroup left;
    MotorGroup right;

  private:
    PID drivePID; PID turnPID;
    SlewController driveSlew;
    PurePursuitFollower PPTenshi;
    std::shared_ptr<ChassisController> chassis;
    std::shared_ptr<AsyncMotionProfileController> profileController;

    double maxVelocity, maxAcceleration;
    State driveState = TANK;

    Vector scaleSpeed(double linear, double turn, double turnScale);

    void updateState();
    void execute();
    void run();

    void tank();
    void arcade();
    void driverControl();
};

extern Drive drive;
