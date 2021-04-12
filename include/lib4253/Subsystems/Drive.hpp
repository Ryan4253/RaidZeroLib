#pragma once
#include "main.h"

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
    CustomOdometry* odom;

    double maxVelocity, maxAcceleration;
    State driveState = TANK;
    int prevAState = 0;

    Vector scaleSpeed(double linear, double turn, double turnScale);

    void updateState();
    void run();

    void tank();
    void arcade();
};

extern Drive drive;
