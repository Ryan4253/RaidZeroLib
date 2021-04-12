#pragma once
#include "main.h"

extern Controller master;

extern Motor LF;
extern Motor LB;
extern Motor RF;
extern Motor RB;

extern ADIEncoder leftEncoder;
extern ADIEncoder rightEncoder;
extern ADIEncoder midEncoder;
extern ADIButton leftAutonSelector;
extern ADIButton rightAutonSelector;

extern pros::Imu imuTop;
extern pros::Imu imuBottom;

extern MotorGroup baseLeft;
extern MotorGroup baseRight;
extern MotorGroup base;

enum brakeType{
  COAST, BRAKE, HOLD
};


enum competition{
  OPCONTROL, AUTONOMOUS, INITIALIZE, DISABLED
};

class Robot{
  public:
    static void setPower(MotorGroup motor, double power);
    static void setPower(Motor motor, double power);
    static void setBrakeMode(MotorGroup Motor, brakeType mode);
    static void setBrakeMode(Motor motor, brakeType mode);

    static void startTask(std::string name, void (*func)(void *), void *param);
    static bool taskExists(std::string name);
    static void endTask(std::string name);

    static void displayPosition(void *ptr);

    static std::map<std::string, std::unique_ptr<pros::Task>> tasks;
};


extern competition matchState;
