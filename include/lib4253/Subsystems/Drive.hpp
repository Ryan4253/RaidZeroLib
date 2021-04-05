#pragma once
#include "main.h"

class Drive{
  public:
    enum State{
      BUSY, IDLE, TANK, ARCADE
    };

    PID drivePID; PID turnPID;
    SlewController driveSlew;
    PurePursuitFollower PPTenshi;
    std::shared_ptr<ChassisController> chassis;
    std::shared_ptr<AsyncMotionProfileController> profileController;

    Drive(const std::initializer_list<Motor> &l, const std::initializer_list<Motor> &r);
    Drive& withMaxVelocity();
    Drive& withMaxAcceleration();

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
    MotorGroup left, right;

  private:
    double maxVelocity, maxAcceleration;
    State driveState = IDLE;

    Vector scaleSpeed(double linear, double turn, double turnScale);

    void updateState();
    void execute();
    void run();

    void tank();
    void arcade();
    void driverControl();
};

//extern int lmao;
extern Drive drive;
